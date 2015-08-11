
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

// shapes/hyperboloid.cpp*
#include "shapes/hyperboloid.h"
#include "paramset.h"
#include "efloat.h"

// Hyperboloid Method Definitions
Hyperboloid::Hyperboloid(const Transform *o2w, const Transform *w2o, bool ro,
                         const Point3f &point1, const Point3f &point2, Float tm)
    : Shape(o2w, w2o, ro) {
    p1 = point1;
    p2 = point2;
    phiMax = Radians(Clamp(tm, 0, 360));
    Float radius1 = std::sqrt(p1.x * p1.x + p1.y * p1.y);
    Float radius2 = std::sqrt(p2.x * p2.x + p2.y * p2.y);
    rMax = std::max(radius1, radius2);
    zMin = std::min(p1.z, p2.z);
    zMax = std::max(p1.z, p2.z);
    // Compute implicit function coefficients for hyperboloid
    if (p2.z == 0.f) std::swap(p1, p2);
    Point3f pp = p1;
    Float xy1, xy2;
    do {
        pp += (Float)2. * (p2 - p1);
        xy1 = pp.x * pp.x + pp.y * pp.y;
        xy2 = p2.x * p2.x + p2.y * p2.y;
        ah = (1.f / xy1 - (pp.z * pp.z) / (xy1 * p2.z * p2.z)) /
             (1 - (xy2 * pp.z * pp.z) / (xy1 * p2.z * p2.z));
        ch = (ah * xy2 - 1) / (p2.z * p2.z);
    } while (std::isinf(ah) || std::isnan(ah));
}

Bounds3f Hyperboloid::ObjectBound() const {
    Point3f p1 = Point3f(-rMax, -rMax, zMin);
    Point3f p2 = Point3f(rMax, rMax, zMax);
    return Bounds3f(p1, p2);
}

bool Hyperboloid::Intersect(const Ray &r, Float *tHit,
                            SurfaceInteraction *isect,
                            bool testAlphaTexture) const {
    Float phi, v;
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Compute quadratic hyperboloid coefficients

    // Initialize _EFloat_ ray coordinate values
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat a = ah * dx * dx + ah * dy * dy - ch * dz * dz;
    EFloat b = 2.f * (ah * dx * ox + ah * dy * oy - ch * dz * oz);
    EFloat c = ah * ox * ox + ah * oy * oy - ch * oz * oz - 1.f;

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

    // Compute hyperboloid inverse mapping
    pHit = ray((Float)tShapeHit);
    v = (pHit.z - p1.z) / (p2.z - p1.z);
    Point3f pr = (1 - v) * p1 + v * p2;
    phi = std::atan2(pr.x * pHit.y - pHit.x * pr.y,
                     pHit.x * pr.x + pHit.y * pr.y);
    if (phi < 0) phi += 2 * Pi;

    // Test hyperboloid intersection against clipping parameters
    if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) {
        if (tShapeHit == t1) return false;
        tShapeHit = t1;
        if (t1.UpperBound() > ray.tMax) return false;
        // Compute hyperboloid inverse mapping
        pHit = ray((Float)tShapeHit);
        v = (pHit.z - p1.z) / (p2.z - p1.z);
        Point3f pr = (1 - v) * p1 + v * p2;
        phi = std::atan2(pr.x * pHit.y - pHit.x * pr.y,
                         pHit.x * pr.x + pHit.y * pr.y);
        if (phi < 0) phi += 2 * Pi;
        if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) return false;
    }

    // Compute parametric representation of hyperboloid hit
    Float u = phi / phiMax;

    // Compute hyperboloid $\dpdu$ and $\dpdv$
    Float cosPhi = std::cos(phi), sinPhi = std::sin(phi);
    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0.);
    Vector3f dpdv((p2.x - p1.x) * cosPhi - (p2.y - p1.y) * sinPhi,
                  (p2.x - p1.x) * sinPhi + (p2.y - p1.y) * cosPhi, p2.z - p1.z);

    // Compute hyperboloid $\dndu$ and $\dndv$
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f(pHit.x, pHit.y, 0);
    Vector3f d2Pduv = phiMax * Vector3f(-dpdv.y, dpdv.x, 0.);
    Vector3f d2Pdvv(0, 0, 0);

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

    // Compute error bounds for hyperboloid intersection

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

bool Hyperboloid::IntersectP(const Ray &r, bool testAlphaTexture) const {
    Float phi, v;
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Compute quadratic hyperboloid coefficients

    // Initialize _EFloat_ ray coordinate values
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat a = ah * dx * dx + ah * dy * dy - ch * dz * dz;
    EFloat b = 2.f * (ah * dx * ox + ah * dy * oy - ch * dz * oz);
    EFloat c = ah * ox * ox + ah * oy * oy - ch * oz * oz - 1.f;

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

    // Compute hyperboloid inverse mapping
    pHit = ray((Float)tShapeHit);
    v = (pHit.z - p1.z) / (p2.z - p1.z);
    Point3f pr = (1 - v) * p1 + v * p2;
    phi = std::atan2(pr.x * pHit.y - pHit.x * pr.y,
                     pHit.x * pr.x + pHit.y * pr.y);
    if (phi < 0) phi += 2 * Pi;

    // Test hyperboloid intersection against clipping parameters
    if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) {
        if (tShapeHit == t1) return false;
        tShapeHit = t1;
        if (t1.UpperBound() > ray.tMax) return false;
        // Compute hyperboloid inverse mapping
        pHit = ray((Float)tShapeHit);
        v = (pHit.z - p1.z) / (p2.z - p1.z);
        Point3f pr = (1 - v) * p1 + v * p2;
        phi = std::atan2(pr.x * pHit.y - pHit.x * pr.y,
                         pHit.x * pr.x + pHit.y * pr.y);
        if (phi < 0) phi += 2 * Pi;
        if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) return false;
    }
    return true;
}

#define SQR(a) ((a) * (a))
#define QUAD(a) ((SQR(a)) * (SQR(a)))
Float Hyperboloid::Area() const {
    return phiMax / 6.f *
           (2 * QUAD(p1.x) - 2 * p1.x * p1.x * p1.x * p2.x + 2 * QUAD(p2.x) +
            2 * (p1.y * p1.y + p1.y * p2.y + p2.y * p2.y) *
                (SQR(p1.y - p2.y) + SQR(p1.z - p2.z)) +
            p2.x * p2.x * (5 * p1.y * p1.y + 2 * p1.y * p2.y - 4 * p2.y * p2.y +
                           2 * SQR(p1.z - p2.z)) +
            p1.x * p1.x * (-4 * p1.y * p1.y + 2 * p1.y * p2.y +
                           5 * p2.y * p2.y + 2 * SQR(p1.z - p2.z)) -
            2 * p1.x * p2.x *
                (p2.x * p2.x - p1.y * p1.y + 5 * p1.y * p2.y - p2.y * p2.y -
                 p1.z * p1.z + 2 * p1.z * p2.z - p2.z * p2.z));
}

#undef SQR
#undef QUAD
Interaction Hyperboloid::Sample(const Point2f &u) const {
    Severe("Hyperboloid::Sample not implemented.");
    return Interaction();
}

std::shared_ptr<Shape> CreateHyperboloidShape(const Transform *o2w,
                                              const Transform *w2o,
                                              bool reverseOrientation,
                                              const ParamSet &params) {
    Point3f p1 = params.FindOnePoint3f("p1", Point3f(0, 0, 0));
    Point3f p2 = params.FindOnePoint3f("p2", Point3f(1, 1, 1));
    Float phimax = params.FindOneFloat("phimax", 360);
    return std::make_shared<Hyperboloid>(o2w, w2o, reverseOrientation, p1, p2,
                                         phimax);
}
