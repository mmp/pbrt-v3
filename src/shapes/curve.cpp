
/*
    pbrt source code is Copyright(c) 1998-2016
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


// shapes/curve.cpp*
#include "shapes/curve.h"
#include "paramset.h"
#include "stats.h"
STAT_MEMORY_COUNTER("Memory/Curves", curveBytes);
STAT_PERCENT("Intersections/Ray-curve intersection tests", nHits, nTests);

// Curve Utility Functions
static Point3f BlossomBezier(const Point3f p[4], Float u0, Float u1, Float u2) {
    Point3f a[3] = {Lerp(u0, p[0], p[1]), Lerp(u0, p[1], p[2]),
                    Lerp(u0, p[2], p[3])};
    Point3f b[2] = {Lerp(u1, a[0], a[1]), Lerp(u1, a[1], a[2])};
    return Lerp(u2, b[0], b[1]);
}

inline void SubdivideBezier(const Point3f cp[4], Point3f cpSplit[7]) {
    cpSplit[0] = cp[0];
    cpSplit[1] = (cp[0] + cp[1]) / 2;
    cpSplit[2] = (cp[0] + 2 * cp[1] + cp[2]) / 4;
    cpSplit[3] = (cp[0] + 3 * cp[1] + 3 * cp[2] + cp[3]) / 8;
    cpSplit[4] = (cp[1] + 2 * cp[2] + cp[3]) / 4;
    cpSplit[5] = (cp[2] + cp[3]) / 2;
    cpSplit[6] = cp[3];
}

static Point3f EvalBezier(const Point3f cp[4], Float u,
                          Vector3f *deriv = nullptr) {
    Point3f cp1[3] = {Lerp(u, cp[0], cp[1]), Lerp(u, cp[1], cp[2]),
                      Lerp(u, cp[2], cp[3])};
    Point3f cp2[2] = {Lerp(u, cp1[0], cp1[1]), Lerp(u, cp1[1], cp1[2])};
    if (deriv) *deriv = (Float)3 * (cp2[1] - cp2[0]);
    return Lerp(u, cp2[0], cp2[1]);
}

// Curve Method Definitions
CurveCommon::CurveCommon(const Point3f c[4], Float width0, Float width1,
                         CurveType type, const Normal3f *norm)
    : type(type) {
    width[0] = width0;
    width[1] = width1;
    for (int i = 0; i < 4; ++i)
        cpObj[i] = c[i];
    if (norm) {
        n[0] = Normalize(norm[0]);
        n[1] = Normalize(norm[1]);
        normalAngle = std::acos(Clamp(Dot(n[0], n[1]), 0, 1));
        invSinNormalAngle = 1 / std::sin(normalAngle);
    }
}

std::vector<std::shared_ptr<Shape>> CreateCurve(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    const Point3f c[4], Float w0, Float w1, CurveType type,
    const Normal3f *norm, int splitDepth) {
    std::vector<std::shared_ptr<Shape>> segments;
    std::shared_ptr<CurveCommon> common =
        std::make_shared<CurveCommon>(c, w0, w1, type, norm);
    const int nSegments = 1 << splitDepth;
    segments.reserve(nSegments);
    for (int i = 0; i < nSegments; ++i) {
        Float uMin = i / (Float)nSegments;
        Float uMax = (i + 1) / (Float)nSegments;
        segments.push_back(std::make_shared<Curve>(o2w, w2o, reverseOrientation,
                                                   common, uMin, uMax));
    }
    curveBytes += sizeof(CurveCommon) + nSegments * sizeof(Curve);
    return segments;
}

Bounds3f Curve::ObjectBound() const {
    // Compute object-space control points for curve segment, _cpObj_
    Point3f cpObj[4];
    cpObj[0] = BlossomBezier(common->cpObj, uMin, uMin, uMin);
    cpObj[1] = BlossomBezier(common->cpObj, uMin, uMin, uMax);
    cpObj[2] = BlossomBezier(common->cpObj, uMin, uMax, uMax);
    cpObj[3] = BlossomBezier(common->cpObj, uMax, uMax, uMax);
    Bounds3f b =
        Union(Bounds3f(cpObj[0], cpObj[1]), Bounds3f(cpObj[2], cpObj[3]));
    Float width[2] = {Lerp(uMin, common->width[0], common->width[1]),
                      Lerp(uMax, common->width[0], common->width[1])};
    return Expand(b, std::max(width[0], width[1]) * 0.5f);
}

bool Curve::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                      bool testAlphaTexture) const {
    ++nTests;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    // Compute object-space control points for curve segment, _cpObj_
    Point3f cpObj[4];
    cpObj[0] = BlossomBezier(common->cpObj, uMin, uMin, uMin);
    cpObj[1] = BlossomBezier(common->cpObj, uMin, uMin, uMax);
    cpObj[2] = BlossomBezier(common->cpObj, uMin, uMax, uMax);
    cpObj[3] = BlossomBezier(common->cpObj, uMax, uMax, uMax);

    // Project curve control points to plane perpendicular to ray
    Vector3f dx, dy;
    CoordinateSystem(ray.d, &dx, &dy);
    Transform objectToRay = LookAt(ray.o, ray.o + ray.d, dx);
    Point3f cp[4] = {objectToRay(cpObj[0]), objectToRay(cpObj[1]),
                     objectToRay(cpObj[2]), objectToRay(cpObj[3])};

    // Compute refinement depth for curve, _maxDepth_
    Float L0 = 0;
    for (int i = 0; i < 2; ++i)
        L0 = std::max(
            L0, std::max(
                    std::max(std::abs(cp[i].x - 2 * cp[i + 1].x + cp[i + 2].x),
                             std::abs(cp[i].y - 2 * cp[i + 1].y + cp[i + 2].y)),
                    std::abs(cp[i].z - 2 * cp[i + 1].z + cp[i + 2].z)));
    Float eps =
        std::max(common->width[0], common->width[1]) * .05f;  // width / 20
#define LOG4(x) (std::log(x) * 0.7213475108f)
    Float fr0 = LOG4(1.41421356237f * 12.f * L0 / (8.f * eps));
#undef LOG4
    int r0 = (int)std::round(fr0);
    int maxDepth = Clamp(r0, 0, 10);
    return recursiveIntersect(ray, tHit, isect, cp, Inverse(objectToRay), uMin,
                              uMax, maxDepth);
}

bool Curve::recursiveIntersect(const Ray &ray, Float *tHit,
                               SurfaceInteraction *isect, const Point3f cp[4],
                               const Transform &rayToObject, Float u0, Float u1,
                               int depth) const {
    // Try to cull curve segment versus ray

    // Compute bounding box of curve segment, _curveBounds_
    Bounds3f curveBounds =
        Union(Bounds3f(cp[0], cp[1]), Bounds3f(cp[2], cp[3]));
    Float maxWidth = std::max(Lerp(u0, common->width[0], common->width[1]),
                              Lerp(u1, common->width[0], common->width[1]));
    curveBounds = Expand(curveBounds, 0.5 * maxWidth);

    // Compute bounding box of ray, _rayBounds_
    Float rayLength = ray.d.Length();
    Float zMax = rayLength * ray.tMax;
    Bounds3f rayBounds(Point3f(0, 0, 0), Point3f(0, 0, zMax));
    if (Overlaps(curveBounds, rayBounds) == false) return false;
    if (depth > 0) {
        // Split curve segment into sub-segments and test for intersection
        Float uMid = 0.5f * (u0 + u1);
        Point3f cpSplit[7];
        SubdivideBezier(cp, cpSplit);
        return (recursiveIntersect(ray, tHit, isect, &cpSplit[0], rayToObject,
                                   u0, uMid, depth - 1) ||
                recursiveIntersect(ray, tHit, isect, &cpSplit[3], rayToObject,
                                   uMid, u1, depth - 1));
    } else {
        // Intersect ray with curve segment

        // Test ray against segment endpoint boundaries

        // Test sample point against tangent perpendicular at curve start
        Float edge =
            (cp[1].y - cp[0].y) * -cp[0].y + cp[0].x * (cp[0].x - cp[1].x);
        if (edge < 0) return false;

        // Test sample point against tangent perpendicular at curve end
        edge = (cp[2].y - cp[3].y) * -cp[3].y + cp[3].x * (cp[3].x - cp[2].x);
        if (edge < 0) return false;

        // Compute line $w$ that gives minimum distance to sample point
        Vector2f segmentDirection = Point2f(cp[3]) - Point2f(cp[0]);
        Float denom = segmentDirection.LengthSquared();
        if (denom == 0) return false;
        Float w = Dot(-Vector2f(cp[0]), segmentDirection) / denom;

        // Compute $u$ coordinate of curve intersection point and _hitWidth_
        Float u = Clamp(Lerp(w, u0, u1), u0, u1);
        Float hitWidth = Lerp(u, common->width[0], common->width[1]);
        Normal3f nHit;
        if (common->type == CurveType::Ribbon) {
            // Scale _hitWidth_ based on ribbon orientation
            Float sin0 = std::sin((1 - u) * common->normalAngle) *
                         common->invSinNormalAngle;
            Float sin1 =
                std::sin(u * common->normalAngle) * common->invSinNormalAngle;
            nHit = sin0 * common->n[0] + sin1 * common->n[1];
            hitWidth *= AbsDot(nHit, ray.d) / rayLength;
        }

        // Test intersection point against curve width
        Vector3f dpcdw;
        Point3f pc = EvalBezier(cp, Clamp(w, 0, 1), &dpcdw);
        Float ptCurveDist2 = pc.x * pc.x + pc.y * pc.y;
        if (ptCurveDist2 > hitWidth * hitWidth * .25) return false;
        if (pc.z < 0 || pc.z > zMax) return false;

        // Compute $v$ coordinate of curve intersection point
        Float ptCurveDist = std::sqrt(ptCurveDist2);
        Float edgeFunc = dpcdw.x * -pc.y + pc.x * dpcdw.y;
        Float v = (edgeFunc > 0) ? 0.5f + ptCurveDist / hitWidth
                                 : 0.5f - ptCurveDist / hitWidth;

        // Compute hit _t_ and partial derivatives for curve intersection
        if (tHit != nullptr) {
            // FIXME: this tHit isn't quite right for ribbons...
            *tHit = pc.z / rayLength;
            // Compute error bounds for curve intersection
            Vector3f pError(2 * hitWidth, 2 * hitWidth, 2 * hitWidth);

            // Compute $\dpdu$ and $\dpdv$ for curve intersection
            Vector3f dpdu, dpdv;
            EvalBezier(common->cpObj, u, &dpdu);
            if (common->type == CurveType::Ribbon)
                dpdv = Normalize(Cross(nHit, dpdu)) * hitWidth;
            else {
                // Compute curve $\dpdv$ for flat and cylinder curves
                Vector3f dpduPlane = (Inverse(rayToObject))(dpdu);
                Vector3f dpdvPlane =
                    Normalize(Vector3f(-dpduPlane.y, dpduPlane.x, 0)) *
                    hitWidth;
                if (common->type == CurveType::Cylinder) {
                    // Rotate _dpdvPlane_ to give cylindrical appearance
                    Float theta = Lerp(v, -90., 90.);
                    Transform rot = Rotate(-theta, dpduPlane);
                    dpdvPlane = rot(dpdvPlane);
                }
                dpdv = rayToObject(dpdvPlane);
            }
            *isect = (*ObjectToWorld)(SurfaceInteraction(
                ray(pc.z), pError, Point2f(u, v), -ray.d, dpdu, dpdv,
                Normal3f(0, 0, 0), Normal3f(0, 0, 0), ray.time, this));
        }
        ++nHits;
        return true;
    }
}

Float Curve::Area() const {
    // Compute object-space control points for curve segment, _cpObj_
    Point3f cpObj[4];
    cpObj[0] = BlossomBezier(common->cpObj, uMin, uMin, uMin);
    cpObj[1] = BlossomBezier(common->cpObj, uMin, uMin, uMax);
    cpObj[2] = BlossomBezier(common->cpObj, uMin, uMax, uMax);
    cpObj[3] = BlossomBezier(common->cpObj, uMax, uMax, uMax);
    Float width0 = Lerp(uMin, common->width[0], common->width[1]);
    Float width1 = Lerp(uMax, common->width[0], common->width[1]);
    Float avgWidth = (width0 + width1) * 0.5f;
    Float approxLength = 0.f;
    for (int i = 0; i < 3; ++i)
        approxLength += Distance(cpObj[i], cpObj[i + 1]);
    return approxLength * avgWidth;
}

Interaction Curve::Sample(const Point2f &u) const {
    Severe("Curve::Sample not implemented.");
    return Interaction();
}

std::vector<std::shared_ptr<Shape>> CreateCurveShape(const Transform *o2w,
                                                     const Transform *w2o,
                                                     bool reverseOrientation,
                                                     const ParamSet &params) {
    Float width = params.FindOneFloat("width", 1.f);
    Float width0 = params.FindOneFloat("width0", width);
    Float width1 = params.FindOneFloat("width1", width);

    int ncp;
    const Point3f *cp = params.FindPoint3f("P", &ncp);
    if (ncp != 4) {
        Error(
            "Must provide 4 control points for \"curve\" primitive. "
            "(Provided %d).",
            ncp);
        return std::vector<std::shared_ptr<Shape>>();
    }

    CurveType type;
    std::string curveType = params.FindOneString("type", "flat");
    if (curveType == "flat")
        type = CurveType::Flat;
    else if (curveType == "ribbon")
        type = CurveType::Ribbon;
    else if (curveType == "cylinder")
        type = CurveType::Cylinder;
    else {
        Error("Unknown curve type \"%s\".  Using \"flat\".", curveType.c_str());
        type = CurveType::Cylinder;
    }
    int nnorm;
    const Normal3f *n = params.FindNormal3f("N", &nnorm);
    if (n != nullptr) {
        if (type != CurveType::Ribbon) {
            Warning("Curve normals are only used with \"ribbon\" type curves.");
            n = nullptr;
        } else if (nnorm != 2) {
            Error(
                "Must provide two normals with \"N\" parameter for ribbon "
                "curves. "
                "(Provided %d).",
                nnorm);
            return std::vector<std::shared_ptr<Shape>>();
        }
    }

    int sd = params.FindOneFloat("splitdepth", 2);

    if (type == CurveType::Ribbon && !n) {
        Error(
            "Must provide normals \"N\" at curve endpoints with ribbon "
            "curves.");
        return std::vector<std::shared_ptr<Shape>>();
    } else
        return CreateCurve(o2w, w2o, reverseOrientation, cp, width0, width1,
                           type, n, sd);
}
