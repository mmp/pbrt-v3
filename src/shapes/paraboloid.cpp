
/*
    pbrt source code is Copyright(c) 1998-2015
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include "stdafx.h"

// shapes/paraboloid.cpp*
#include "shapes/paraboloid.h"
#include "paramset.h"
#include "efloat.h"

// Paraboloid Method Definitions
Paraboloid::Paraboloid(const Transform *o2w, const Transform *w2o, bool ro,
                       Float radius, Float z0, Float z1, Float phiMax)
    : Shape(o2w, w2o, ro),
      radius(radius),
      zMin(std::min(z0, z1)),
      zMax(std::max(z0, z1)),
      phiMax(Radians(Clamp(phiMax, 0, 360))) {}
Bounds3f Paraboloid::ObjectBound() const {
    Point3f p1 = Point3f(-radius, -radius, zMin);
    Point3f p2 = Point3f(radius, radius, zMax);
    return Bounds3f(p1, p2);
}

bool Paraboloid::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                           bool testAlphaTexture) const {
    Float phi;
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Compute quadratic paraboloid coefficients

    // Initialize _EFloat_ ray coordinate values
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat k = EFloat(zMax) / (EFloat(radius) * EFloat(radius));
    EFloat a = k * (dx * dx + dy * dy);
    EFloat b = 2 * k * (dx * ox + dy * oy) - dz;
    EFloat c = k * (ox * ox + oy * oy) - oz;

    // Solve quadratic equation for _t_ values
    EFloat t0, t1;
    if (!Quadratic(a, b, c, &t0, &t1)) return false;

    // Check quadric shape _t0_ and _t1_ for nearest intersection
    if (t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.LowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.UpperBound() > ray.tMax) return false;
    }

    // Compute paraboloid inverse mapping
    pHit = ray((Float)tShapeHit);
    phi = std::atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * Pi;

    // Test paraboloid intersection against clipping parameters
    if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) {
        if (tShapeHit == t1) return false;
        tShapeHit = t1;
        if (t1.UpperBound() > ray.tMax) return false;
        // Compute paraboloid inverse mapping
        pHit = ray((Float)tShapeHit);
        phi = std::atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * Pi;
        if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) return false;
    }

    // Find parametric representation of paraboloid hit
    Float u = phi / phiMax;
    Float v = (pHit.z - zMin) / (zMax - zMin);

    // Compute paraboloid $\dpdu$ and $\dpdv$
    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0.);
    Vector3f dpdv = (zMax - zMin) *
                    Vector3f(pHit.x / (2 * pHit.z), pHit.y / (2 * pHit.z), 1.);

    // Compute paraboloid $\dndu$ and $\dndv$
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f(pHit.x, pHit.y, 0);
    Vector3f d2Pduv =
        (zMax - zMin) * phiMax *
        Vector3f(-pHit.y / (2 * pHit.z), pHit.x / (2 * pHit.z), 0);
    Vector3f d2Pdvv = -(zMax - zMin) * (zMax - zMin) *
                      Vector3f(pHit.x / (4 * pHit.z * pHit.z),
                               pHit.y / (4 * pHit.z * pHit.z), 0);

    // Compute coefficients for fundamental forms
    Float E = Dot(dpdu, dpdu);
    Float F = Dot(dpdu, dpdv);
    Float G = Dot(dpdv, dpdv);
    Vector3f N = Normalize(Cross(dpdu, dpdv));
    Float e = Dot(N, d2Pduu);
    Float f = Dot(N, d2Pduv);
    Float g = Dot(N, d2Pdvv);

    // Compute $\dndu$ and $\dndv$ from fundamental form coefficients
    Float invEGF2 = 1 / (E * G - F * F);
    Normal3f dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu +
                             (e * F - f * E) * invEGF2 * dpdv);
    Normal3f dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu +
                             (f * F - g * E) * invEGF2 * dpdv);

    // Compute error bounds for paraboloid intersection

    // Compute error bounds for intersection computed with ray equation
    EFloat px = ox + tShapeHit * dx;
    EFloat py = oy + tShapeHit * dy;
    EFloat pz = oz + tShapeHit * dz;
    Vector3f pError = Vector3f(px.GetAbsoluteError(), py.GetAbsoluteError(),
                               pz.GetAbsoluteError());

    // Initialize _SurfaceInteraction_ from parametric information
    *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v),
                                                 -ray.d, dpdu, dpdv, dndu, dndv,
                                                 ray.time, this));
    *tHit = (Float)tShapeHit;
    return true;
}

bool Paraboloid::IntersectP(const Ray &r, bool testAlphaTexture) const {
    Float phi;
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Compute quadratic paraboloid coefficients

    // Initialize _EFloat_ ray coordinate values
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat k = EFloat(zMax) / (EFloat(radius) * EFloat(radius));
    EFloat a = k * (dx * dx + dy * dy);
    EFloat b = 2 * k * (dx * ox + dy * oy) - dz;
    EFloat c = k * (ox * ox + oy * oy) - oz;

    // Solve quadratic equation for _t_ values
    EFloat t0, t1;
    if (!Quadratic(a, b, c, &t0, &t1)) return false;

    // Check quadric shape _t0_ and _t1_ for nearest intersection
    if (t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.LowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.UpperBound() > ray.tMax) return false;
    }

    // Compute paraboloid inverse mapping
    pHit = ray((Float)tShapeHit);
    phi = std::atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * Pi;

    // Test paraboloid intersection against clipping parameters
    if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) {
        if (tShapeHit == t1) return false;
        tShapeHit = t1;
        if (t1.UpperBound() > ray.tMax) return false;
        // Compute paraboloid inverse mapping
        pHit = ray((Float)tShapeHit);
        phi = std::atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * Pi;
        if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) return false;
    }
    return true;
}

Float Paraboloid::Area() const {
    Float radius2 = radius * radius;
    Float k = 4 * zMax / radius2;
    return (radius2 * radius2 * phiMax / (12 * zMax * zMax)) *
           (std::pow(k * zMax + 1, 1.5f) - std::pow(k * zMin + 1, 1.5f));
}

Interaction Paraboloid::Sample(const Point2f &u) const {
    Severe("Paraboloid::Sample not implemented.");
    return Interaction();
}

std::shared_ptr<Paraboloid> CreateParaboloidShape(const Transform *o2w,
                                                  const Transform *w2o,
                                                  bool reverseOrientation,
                                                  const ParamSet &params) {
    Float radius = params.FindOneFloat("radius", 1);
    Float zmin = params.FindOneFloat("zmin", 0);
    Float zmax = params.FindOneFloat("zmax", 1);
    Float phimax = params.FindOneFloat("phimax", 360);
    return std::make_shared<Paraboloid>(o2w, w2o, reverseOrientation, radius,
                                        zmin, zmax, phimax);
}
