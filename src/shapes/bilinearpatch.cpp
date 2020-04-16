
#include "shapes/bilinearpatch.h"

#include "paramset.h"
#include "profile.h"
#include "rng.h"
#include "sampling.h"
#include "reflection.h"

#include <array>
#include <cmath>

namespace pbrt {

// Returns the solid angle subtended by a spherical quadrilateral with the
// given vertices. Note: the vertex order for a quad is different than for
// a bilinear patch!
Float SphericalQuadArea(const Vector3f &a, const Vector3f &b,
                        const Vector3f &c, const Vector3f &d) {
    Vector3f axb = Cross(a, b), bxc = Cross(b, c);
    Vector3f cxd = Cross(c, d), dxa = Cross(d, a);
    if (axb.LengthSquared() == 0 || bxc.LengthSquared() == 0 ||
        cxd.LengthSquared() == 0 || dxa.LengthSquared() == 0)
        return 0;
    axb = Normalize(axb);
    bxc = Normalize(bxc);
    cxd = Normalize(cxd);
    dxa = Normalize(dxa);

    Float alpha = AngleBetween(dxa, -axb);
    Float beta = AngleBetween(axb, -bxc);
    Float gamma = AngleBetween(bxc, -cxd);
    Float delta = AngleBetween(cxd, -dxa);

    return std::abs(alpha + beta + gamma + delta - 2 * Pi);
}

BilinearPatchMesh::BilinearPatchMesh(const Transform &worldFromObject, bool reverseOrientation,
                                     std::vector<int> Indices, std::vector<Point3f> P,
                                     std::vector<Normal3f> N, std::vector<Point2f> UV)
    : reverseOrientation(reverseOrientation),
      transformSwapsHandedness(worldFromObject.SwapsHandedness()),
      nPatches(Indices.size() / 4),
      nVertices(P.size()),
      vertexIndices(std::move(Indices)),
      p(std::move(P)),
      n(std::move(N)),
      uv(std::move(UV)) {
    CHECK_EQ((vertexIndices.size() % 4), 0);

    // Transform mesh vertices to world space
    for (Point3f &pt : p)
        pt = worldFromObject(pt);

    if (!N.empty()) {
        CHECK_EQ(nVertices, N.size());
        for (Normal3f &nn : n)
            nn = worldFromObject(nn);
    }
}

std::vector<std::shared_ptr<Shape>> BilinearPatchMesh::Create(const Transform *worldFromObject,
                                                              const Transform *objectFromWorld,
                                                              bool reverseOrientation,
                                                              const ParamSet &dict) {
    int nIndices;
    const int *vi = dict.FindInt("indices", &nIndices);
    if (!vi) {
        Error("Vertex indices \"indices\" must be provided with bilinear patch mesh shape.");
        return {};
    }
    CHECK_EQ(0, nIndices % 4);
    std::vector<int> vertexIndices(vi, vi + nIndices);

    int nPoints;
    const Point3f *pts = dict.FindPoint3f("P", &nPoints);
    if (!pts) {
        Error("Vertex positions \"P\" must be provided with bilinear patch mesh shape.");
        return {};
    }
    std::vector<Point3f> P(pts, pts + nPoints);

    int nUV;
    const Point2f *uvs = dict.FindPoint2f("uv", &nUV);
    std::vector<Point2f> uv;
    if (uvs) {
        CHECK_EQ(nUV, nPoints);
        uv = std::vector<Point2f>(uvs, uvs + nUV);
    }

    int nN;
    const Normal3f *ns = dict.FindNormal3f("N", &nN);
    std::vector<Normal3f> N;
    if (ns) {
        CHECK_EQ(nN, nPoints);
        N = std::vector<Normal3f>(ns, ns + nN);
    }

    for (size_t i = 0; i < vertexIndices.size(); ++i)
        if (vertexIndices[i] >= P.size()) {
            Error(
                "Bilinear patch mesh has out of-bounds vertex index %d (%d \"P\" "
                "values were given. Discarding this mesh.",
                vertexIndices[i], (int)P.size());
            return {};
        }

    // Grab this before the vertexIndices are std::moved...
    size_t nBlps = vertexIndices.size() / 4;

    CHECK_LT(allMeshes.size(), 1 << 31);
    int meshIndex = int(allMeshes.size());

    allMeshes.push_back(new BilinearPatchMesh(*worldFromObject, reverseOrientation,
                                              std::move(vertexIndices), std::move(P),
                                              std::move(N), std::move(uv)));

    std::vector<std::shared_ptr<Shape>> patches;
    int nPatches = nIndices / 4;
    for (int i = 0; i < nPatches; ++i)
        patches.push_back(std::make_shared<BilinearPatch>(worldFromObject, objectFromWorld,
                                                          reverseOrientation, meshIndex, i));
    return patches;
}

inline Normal3f Lerp(Float t, Normal3f a, Normal3f b) {
    return (1.f - t) * a + t * b;
}

inline Transform Rotate(Float sinTheta, Float cosTheta, const Vector3f &axis) {
    Vector3f a = Normalize(axis);
    Matrix4x4 m;
    // Compute rotation of first basis vector
    m.m[0][0] = a.x * a.x + (1 - a.x * a.x) * cosTheta;
    m.m[0][1] = a.x * a.y * (1 - cosTheta) - a.z * sinTheta;
    m.m[0][2] = a.x * a.z * (1 - cosTheta) + a.y * sinTheta;
    m.m[0][3] = 0;

    // Compute rotations of second and third basis vectors
    m.m[1][0] = a.x * a.y * (1 - cosTheta) + a.z * sinTheta;
    m.m[1][1] = a.y * a.y + (1 - a.y * a.y) * cosTheta;
    m.m[1][2] = a.y * a.z * (1 - cosTheta) - a.x * sinTheta;
    m.m[1][3] = 0;

    m.m[2][0] = a.x * a.z * (1 - cosTheta) - a.y * sinTheta;
    m.m[2][1] = a.y * a.z * (1 - cosTheta) + a.x * sinTheta;
    m.m[2][2] = a.z * a.z + (1 - a.z * a.z) * cosTheta;
    m.m[2][3] = 0;
    return Transform(m, Transpose(m));
}

bool BilinearPatch::Intersect(const Ray &ray,
                              const Point3f &p00, const Point3f &p10,
                              const Point3f &p01, const Point3f &p11,
                              BilinearIntersection *bi) {
    Vector3f qn = Cross(p10-p00, p01-p11);
    Vector3f e10 = p10 - p00; // p01------u--------p11
    Vector3f e11 = p11 - p10; // |                   |
    Vector3f e00 = p01 - p00; // v e00           e11 v
                              // |        e10        |
                              // p00------u--------p10
    Vector3f q00 = p00 - ray.o;
    Vector3f q10 = p10 - ray.o;
    Float a = Dot(Cross(q00, ray.d), e00); // the equation is
    Float c = Dot(qn, ray.d);              // a + b u + c u^2
    Float b = Dot(Cross(q10, ray.d), e11); // first compute a+b+c
    b -= a + c;                                    // and then b
    Float det = b*b - 4*a*c;
    if (det < 0) return false;
    det = std::sqrt(det);
    Float u1, u2;             // two roots (u parameter)
    Float t = ray.tMax, u, v; // need solution for the smallest t > 0
    if (c == 0) {                         // if c == 0, it is a trapezoid
        u1  = -a/b; u2 = -1;              // and there is only one root
    } else {                              // (c != 0 in Stanford models)
        u1  = (-b - copysignf(det, b))/2; // numerically "stable" root
        u2  = a/u1;                       // Viete's formula for u1*u2
        u1 /= c;
    }
    if (0 <= u1 && u1 <= 1) {                // is it inside the patch?
        Vector3f pa = Lerp(u1, q00, q10);       // point on edge e10 (Figure 8.4)
        Vector3f pb = Lerp(u1, e00, e11);       // it is, actually, pb - pa
        Vector3f n  = Cross(ray.d, pb);
        det = Dot(n, n);
        n = Cross(n, pa);
        Float t1 = Dot(n, pb);
        Float v1 = Dot(n, ray.d);     // no need to check t1 < t
        if (t1 > 0 && 0 <= v1 && v1 <= det) {
            t = t1/det; u = u1; v = v1/det;
        }
    }
    if (0 <= u2 && u2 <= 1) {                 // it is slightly different,
        Vector3f pa = Lerp(u2, q00, q10);     // since u1 might be good
        Vector3f pb = Lerp(u2, e00, e11);     // and we need 0 < t2 < t1
        Vector3f n  = Cross(ray.d, pb);
        det = Dot(n, n);
        n = Cross(n, pa);
        Float t2 = Dot(n, pb)/det;
        Float v2 = Dot(n, ray.d);
        if (0 <= v2 && v2 <= det && t > t2 && t2 > 0) {
            t = t2; u = u2; v = v2/det;
        }
    }

    if (t >= ray.tMax)
        return false;

    *bi = BilinearIntersection{{u, v}, t};
    return true;
}


SurfaceInteraction BilinearPatchMesh::InteractionFromIntersection(int patchIndex, const Point2f &uvHit,
                                                   Float time, const Vector3f &wo, const BilinearPatch *patch,
                                                   Transform *worldFromInstance) const {
    const int *v = &vertexIndices[4 * patchIndex];
    Point3f p00 = p[v[0]], p10 = p[v[1]], p01 = p[v[2]], p11 = p[v[3]];

    if (worldFromInstance) {
        p00 = (*worldFromInstance)(p00);
        p10 = (*worldFromInstance)(p10);
        p01 = (*worldFromInstance)(p01);
        p11 = (*worldFromInstance)(p11);
    }

    Point3f pHit = Lerp(uvHit[0], Lerp(uvHit[1], p00, p01), Lerp(uvHit[1], p10, p11));

    Vector3f dpdu = Lerp(uvHit[1], p10, p11) - Lerp(uvHit[1], p00, p01);
    Vector3f dpdv = Lerp(uvHit[0], p01, p11) - Lerp(uvHit[0], p00, p10);

    // Interpolate texture coordinates, if provided
    Point2f uv = uvHit;
    if (!this->uv.empty()) {
        const Point2f &uv00 = this->uv[v[0]];
        const Point2f &uv10 = this->uv[v[1]];
        const Point2f &uv01 = this->uv[v[2]];
        const Point2f &uv11 = this->uv[v[3]];

        Float dsdu = -uv00[0] + uv10[0] + uv[1] * (uv00[0] - uv01[0] - uv10[0] + uv11[0]);
        Float dsdv = -uv00[0] + uv01[0] + uv[0] * (uv00[0] - uv01[0] - uv10[0] + uv11[0]);
        Float dtdu = -uv00[1] + uv10[1] + uv[1] * (uv00[1] - uv01[1] - uv10[1] + uv11[1]);
        Float dtdv = -uv00[1] + uv01[1] + uv[0] * (uv00[1] - uv01[1] - uv10[1] + uv11[1]);

        Float duds = std::abs(dsdu) < 1e-8f ? 0 : 1 / dsdu;
        Float dvds = std::abs(dsdv) < 1e-8f ? 0 : 1 / dsdv;
        Float dudt = std::abs(dtdu) < 1e-8f ? 0 : 1 / dtdu;
        Float dvdt = std::abs(dtdv) < 1e-8f ? 0 : 1 / dtdv;

        // actually this is st (and confusing)
        uv = Lerp(uv[0], Lerp(uv[1], uv00, uv01), Lerp(uv[1], uv10, uv11));

        // dpds = dpdu * duds + dpdv * dvds, etc
        // duds = 1/dsdu
        Vector3f dpds = dpdu * duds + dpdv * dvds;
        Vector3f dpdt = dpdu * dudt + dpdv * dvdt;

        // These end up as zero-vectors of the mapping is degenerate...
        if (Cross(dpds, dpdt) != Vector3f(0, 0, 0)) {
            dpdu = dpds;
            dpdv = dpdt;
        }
    }

    // Compute coefficients for fundamental forms
    Float E = Dot(dpdu, dpdu);
    Float F = Dot(dpdu, dpdv);
    Float G = Dot(dpdv, dpdv);
    Vector3f N = Normalize(Cross(dpdu, dpdv));
    Float e = 0;  // 2nd derivative d2p/du2 == 0
    Vector3f d2Pduv(p00.x - p01.x - p10.x + p11.x,
                    p00.y - p01.y - p10.y + p11.y,
                    p00.z - p01.z - p10.z + p11.z);
    Float f = Dot(N, d2Pduv);
    Float g = 0;  // samesies

    // Compute $\dndu$ and $\dndv$ from fundamental form coefficients
    Float invEGF2 = 1 / (E * G - F * F);
    Normal3f dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu +
                             (e * F - f * E) * invEGF2 * dpdv);
    Normal3f dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu +
                             (f * F - g * E) * invEGF2 * dpdv);

    // Two lerps; each is gamma(3).
    Vector3f pError = gamma(6) * Vector3f(Max(Max(Abs(p00), Abs(p10)), Max(Abs(p01), Abs(p11))));

    // Initialize _SurfaceInteraction_ from parametric information
    int faceIndex = 0;
    SurfaceInteraction isect(pHit, pError, uv, wo, dpdu, dpdv, dndu, dndv, time,
                             patch, faceIndex);

    if (!n.empty()) {
        Normal3f n00 = n[v[0]], n10 = n[v[1]], n01 = n[v[2]], n11 = n[v[3]];
        if (worldFromInstance) {
            n00 = (*worldFromInstance)(n00);
            n10 = (*worldFromInstance)(n10);
            n01 = (*worldFromInstance)(n01);
            n11 = (*worldFromInstance)(n11);
        }

        Normal3f dndu = Lerp(uvHit[1], n10, n11) - Lerp(uvHit[1], n00, n01);
        Normal3f dndv = Lerp(uvHit[0], n01, n11) - Lerp(uvHit[0], n00, n10);

        Normal3f ns = Lerp(uvHit[0], Lerp(uvHit[1], n00, n01), Lerp(uvHit[1], n10, n11));
        if (ns.LengthSquared() > 0) {
            ns = Normalize(ns);
            Normal3f n = Normal3f(Normalize(Cross(dpdu, dpdv)));
            Vector3f axis = Cross(Vector3f(n), Vector3f(ns));
            if (axis.LengthSquared() > .0001f) {
                axis = Normalize(axis);
                // The shading normal is different enough.
                //
                // Don't worry about if ns == -n; that, too, is handled
                // naturally by the following.
                //
                // Rotate dpdu and dpdv around the axis perpendicular to the
                // plane defined by n and ns by the angle between them -> their
                // dot product will equal ns.
                Float cosTheta = Dot(n, ns), sinTheta = std::sqrt(std::max<Float>(0, 1 - cosTheta * cosTheta));
                Transform r = Rotate(sinTheta, cosTheta, axis);
                Vector3f sdpdu = r(dpdu), sdpdv = r(dpdv);

                if (reverseOrientation) {
                    ns = -ns;
                    sdpdv = -sdpdv;
                }
                isect.SetShadingGeometry(sdpdu, sdpdv, dndu, dndv, true);
            }
        }
    }

    return isect;
}

std::vector<const BilinearPatchMesh *> BilinearPatchMesh::allMeshes;

BilinearPatch::BilinearPatch(const Transform *ObjectToWorld, const Transform *WorldToObject,
                             bool reverseOrientation, int meshIndex, int blpIndex)
    : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
      meshIndex(meshIndex), blpIndex(blpIndex) {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    if (IsQuad())
        area = Distance(p00, p01) * Distance(p00, p10);
    else {
        // Note: it would be good to skip this for flat patches, or to be
        // adaptive based on curvature in some manner
        constexpr int na = 8;
        Point3f p[na+1][na+1];
        for (int i = 0; i <= na; ++i) {
            Float u = Float(i) / Float(na);
            for (int j = 0; j <= na; ++j) {
                Float v = Float(j) / Float(na);
                p[i][j] = Lerp(u, Lerp(v, p00, p01), Lerp(v, p10, p11));
            }
        }

        area = 0;
        for (int i = 0; i < na; ++i)
            for (int j = 0; j < na; ++j)
                area += 0.5f * Cross(p[i+1][j+1] - p[i][j],
                                     p[i+1][j] - p[i][j+1]).Length();
    }
}

Bounds3f BilinearPatch::WorldBound() const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    return Union(Bounds3f(p00, p01), Bounds3f(p10, p11));
}

Bounds3f BilinearPatch::ObjectBound() const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    return Union(Bounds3f((*WorldToObject)(p00), (*WorldToObject)(p01)),
                 Bounds3f((*WorldToObject)(p10), (*WorldToObject)(p11)));
}

bool BilinearPatch::Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                              bool testAlpha) const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    BilinearIntersection blpIsect;
    if (!Intersect(ray, p00, p10, p01, p11, &blpIsect))
        return false;

    *isect = mesh->InteractionFromIntersection(blpIndex, blpIsect.uv, ray.time, -ray.d, this);
    *tHit = blpIsect.t;
    return true;
}

bool BilinearPatch::IntersectP(const Ray &ray, bool testAlpha) const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    BilinearIntersection blpIsect;
    return Intersect(ray, p00, p10, p01, p11, &blpIsect);
}

Float BilinearPatch::Area() const {
    return area;
}

bool BilinearPatch::IsQuad() const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    Point3f p11quad = p00 + (p10 - p00) + (p01 - p00);
    Float diag = std::max(Distance(p00, p11), Distance(p01, p11));
    return Distance(p11quad, p11) < .001f * diag;
}

Interaction BilinearPatch::Sample(const Interaction &ref, const Point2f &u,
                                  Float *pdfOut) const {
    if (IsQuad()) {
        // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
        auto mesh = GetMesh();
        const int *v = &mesh->vertexIndices[4 * blpIndex];
        const Point3f &p00 = mesh->p[v[0]];
        const Point3f &p10 = mesh->p[v[1]];
        const Point3f &p01 = mesh->p[v[2]];
        const Point3f &p11 = mesh->p[v[3]];

        if (ref.IsSurfaceInteraction()) {
            // Warp the PSS sample |u| based on the BSDF at the point being shaded.
            std::array<std::array<Float, 3>, 3> w = biquadraticBSDFWeights(ref);
            Float pdf;
            Point2f up = SampleBezier2D(u, w, &pdf);

            // Sample a point on the quadrilateral (w.r.t. area here) using
            // the warped point |up|.
            Point3f p = Lerp(up[1], Lerp(up[0], p00, p10), Lerp(up[0], p01, p11));
            Normal3f n = Normal3f(Normalize(Cross(p10 - p00, p01 - p00)));
            Vector3f wi = Normalize(p - ref.p);
            if (AbsDot(n, wi) == 0)
                pdf = 0;
            else
                // The overall PDF is the product of the PSS warp PDF and
                // the uniform area sampling PDF.
                pdf *= DistanceSquared(ref.p, p) / (area * AbsDot(n, wi));
            *pdfOut = pdf;

            Interaction sampleIntr(p, ref.time, MediumInterface());
            sampleIntr.n = n;
            sampleIntr.uv = up;
            return sampleIntr;
        }

        Float quadPDF;
        Point3f p = SampleSphericalQuad(ref.p, p00, p10 - p00, p01 - p00, u, &quadPDF);
        Normal3f n = Normal3f(Normalize(Cross(p10 - p00, p01 - p00)));
        if (reverseOrientation) n *= -1;
        Point2f uv(Dot(p - p00, p10 - p00) / DistanceSquared(p10, p00),
                   Dot(p - p00, p01 - p00) / DistanceSquared(p01, p00));

        *pdfOut = quadPDF;
        Interaction intr(p, ref.time, MediumInterface());
        intr.n = n;
        intr.uv = uv;
        return intr;
    }

    // Sample by area and then convert to solid angle.
    Interaction intr = Sample(u, pdfOut);
    Vector3f wi = intr.p - ref.p;
    if (wi.LengthSquared() == 0)
        *pdfOut = 0;
    else {
        wi = Normalize(wi);
        // Convert from area measure, as returned by the Sample() call
        // above, to solid angle measure.
        *pdfOut *= DistanceSquared(ref.p, intr.p) / AbsDot(intr.n, -wi);
        if (std::isinf(*pdfOut)) *pdfOut = 0.f;
    }
    return intr;
}

Float BilinearPatch::Pdf(const Interaction &ref, const Vector3f &wi) const {
    // Intersect sample ray with area light geometry
    Ray ray = ref.SpawnRay(wi);
    Float tHit;
    SurfaceInteraction isectLight;
    if (!Intersect(ray, &tHit, &isectLight, false)) return 0;

    if (IsQuad()) {
        // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
        auto mesh = GetMesh();
        const int *v = &mesh->vertexIndices[4 * blpIndex];
        const Point3f &p00 = mesh->p[v[0]];
        const Point3f &p10 = mesh->p[v[1]];
        const Point3f &p01 = mesh->p[v[2]];
        const Point3f &p11 = mesh->p[v[3]];

        Vector3f v00 = Normalize(p00 - ref.p);
        Vector3f v10 = Normalize(p10 - ref.p);
        Vector3f v01 = Normalize(p01 - ref.p);
        Vector3f v11 = Normalize(p11 - ref.p);
        Float quadSamplePDF = 1.f / SphericalQuadArea(v00, v10, v11, v01);

        if (ref.IsSurfaceInteraction()) {
            std::array<std::array<Float, 3>, 3> wts = biquadraticBSDFWeights(ref);

            if (AbsDot(ref.n, wi) == 0) return 0;

            // Algorithm 2 to compute the PDF for the sample on the light
            // source.
            Float quadSamplePDF = DistanceSquared(ref.p, isectLight.p) /
                (area * AbsDot(isectLight.n, wi));
            return Bezier2DPDF(isectLight.uv, wts) * quadSamplePDF;
        }

        return quadSamplePDF;
    } else {
        // Convert light sample weight to solid angle measure
        Float pdf = Pdf(isectLight) * DistanceSquared(ref.p, isectLight.p) /
            AbsDot(isectLight.n, -wi);
        if (std::isinf(pdf)) pdf = 0.f;
        return pdf;
    }
}

std::array<std::array<Float, 3>, 3> BilinearPatch::biquadraticBSDFWeights(const Interaction &ref) const {
    CHECK(ref.IsSurfaceInteraction());
    const SurfaceInteraction *intr = (const SurfaceInteraction *)&ref;

    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    const SurfaceInteraction *rintr = (const SurfaceInteraction *)&ref;
    Normal3f nf = Faceforward(rintr->shading.n, ref.wo);

    // Evaluate the BSDF * cosine term for the given point on the light
    // source.
    auto evalf = [&](Point3f p) -> Float {
        Vector3f wi = p - ref.p;
        if (Dot(nf, wi) < 0) return 0.01f;
        wi = Normalize(wi);
        return 0.01f + intr->bsdf->f(intr->wo, wi).MaxComponentValue() * Dot(nf, wi);
    };

    std::array<std::array<Float, 3>, 3> w;
    w[0][0] = evalf(p00);
    w[1][0] = evalf((p00 + p10) / 2);
    w[2][0] = evalf(p10);

    w[0][1] = evalf((p00 + p01) / 2);
    w[1][1] = evalf((p00 + p10 + p01 + p11) / 4);
    w[2][1] = evalf((p10 + p11) / 2);

    w[0][2] = evalf(p01);
    w[1][2] = evalf((p01 + p11) / 2);
    w[2][2] = evalf(p11);

    return w;
}

Interaction BilinearPatch::Sample(const Point2f &uo, Float *pdfOut) const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    Point2f u = uo;
    Float pdf = 1;

    if (!IsQuad()) {
        // Approximate uniform area sampling by computing weights
        // proportional to the differential area factor at each corner in
        // parametric space.
        std::array<Float, 4> w = { Cross(p10 - p00, p01 - p00).Length(),
                                   Cross(p10 - p00, p11 - p10).Length(),
                                   Cross(p01 - p00, p11 - p01).Length(),
                                   Cross(p11 - p10, p11 - p01).Length() };

        // Warp the original sample according to these weights.
        u = SampleBilinear(u, w);
        pdf = BilinearPDF(u, w);
    }

    Point3f pu0 = Lerp(u[1], p00, p01), pu1 = Lerp(u[1], p10, p11);
    Vector3f dpdu = pu1 - pu0;
    Vector3f dpdv = Lerp(u[0], p01, p11) - Lerp(u[0], p00, p10);
    if (dpdu.LengthSquared() == 0 || dpdv.LengthSquared() == 0) {
        *pdfOut = 0;
        return {};
    }

    Normal3f n = Normal3f(Normalize(Cross(dpdu, dpdv)));
    if (reverseOrientation) n = -n;

    Point3f p = Lerp(u[0], pu0, pu1);
    Vector3f pError = gamma(6) *
        Vector3f(Max(Max(Abs(p00), Abs(p10)), Max(Abs(p01), Abs(p11))));

    // The overall PDF is the product of the PSS warp's PDF and the PDF for
    // uniform parametric sampling of a bilinear patch,
    // (1 / Cross(dpdu, dpdv).Length()).
    *pdfOut = pdf / Cross(dpdu, dpdv).Length();

    Interaction intr(p, n, pError, Vector3f(), 0 /* time */, MediumInterface());
    intr.uv = u;
    return intr;
}

Float BilinearPatch::Pdf(const Interaction &intr) const {
    // Get bilinear patch vertices in _p00_, _p01_, _p10_, and _p11_
    auto mesh = GetMesh();
    const int *v = &mesh->vertexIndices[4 * blpIndex];
    const Point3f &p00 = mesh->p[v[0]];
    const Point3f &p10 = mesh->p[v[1]];
    const Point3f &p01 = mesh->p[v[2]];
    const Point3f &p11 = mesh->p[v[3]];

    Float pdf = 1;

    if (!IsQuad()) {
        std::array<Float, 4> w = { Cross(p10 - p00, p01 - p00).Length(),
                                   Cross(p10 - p00, p11 - p10).Length(),
                                   Cross(p01 - p00, p11 - p01).Length(),
                                   Cross(p11 - p10, p11 - p01).Length() };
        pdf = BilinearPDF(intr.uv, w);
    }

    CHECK(!intr.uv.HasNaNs());
    Point3f pu0 = Lerp(intr.uv[1], p00, p01), pu1 = Lerp(intr.uv[1], p10, p11);
    Vector3f dpdu = pu1 - pu0;
    Vector3f dpdv = Lerp(intr.uv[0], p01, p11) - Lerp(intr.uv[0], p00, p10);
    return pdf / Cross(dpdu, dpdv).Length();
}

}  // namespace pbrt
