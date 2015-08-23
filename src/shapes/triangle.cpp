
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

// shapes/triangle.cpp*
#include "shapes/triangle.h"
#include "texture.h"
#include "textures/constant.h"
#include "paramset.h"
#include "sampling.h"
#include "efloat.h"
#include "ext/rply.h"
STAT_PERCENT("Intersections/Ray-triangle intersection tests", nHits, nTests);

// Triangle Local Definitions
static void PlyErrorCallback(p_ply, const char *message) {
    Error("PLY writing error: %s", message);
}

// Triangle Method Definitions
STAT_RATIO("Scene/Triangles per triangle mesh", nTris, nMeshes);
TriangleMesh::TriangleMesh(const Transform &ObjectToWorld, int nTriangles,
                           const int *vertexIndices, int nVertices,
                           const Point3f *P, const Vector3f *S,
                           const Normal3f *N, const Point2f *UV,
                           const std::shared_ptr<Texture<Float>> &alphaMask)
    : nTriangles(nTriangles),
      nVertices(nVertices),
      vertexIndices(vertexIndices, vertexIndices + 3 * nTriangles),
      alphaMask(alphaMask) {
    ++nMeshes;
    nTris += nTriangles;
    triMeshBytes += sizeof(*this) + (3 * nTriangles * sizeof(int)) +
                    nVertices * (sizeof(*P) + (N ? sizeof(*N) : 0) +
                                 (S ? sizeof(*S) : 0) + (UV ? sizeof(*UV) : 0));
    if (getenv("PBRT_DUMP_PLY")) {
        // Write out triangle mesh as PLY file
        static int count = 1;
        char fn[128];
        sprintf(fn, "mesh_%05d.ply", count++);
        p_ply plyFile =
            ply_create(fn, PLY_DEFAULT, PlyErrorCallback, 0, nullptr);
        if (plyFile != nullptr) {
            ply_add_element(plyFile, "vertex", nVertices);
            ply_add_scalar_property(plyFile, "x", PLY_FLOAT);
            ply_add_scalar_property(plyFile, "y", PLY_FLOAT);
            ply_add_scalar_property(plyFile, "z", PLY_FLOAT);
            if (N != nullptr) {
                ply_add_scalar_property(plyFile, "nx", PLY_FLOAT);
                ply_add_scalar_property(plyFile, "ny", PLY_FLOAT);
                ply_add_scalar_property(plyFile, "nz", PLY_FLOAT);
            }
            if (UV != nullptr) {
                ply_add_scalar_property(plyFile, "u", PLY_FLOAT);
                ply_add_scalar_property(plyFile, "v", PLY_FLOAT);
            }
            // TODO(?): tangent vectors...

            ply_add_element(plyFile, "face", nTriangles);
            ply_add_list_property(plyFile, "vertex_indices", PLY_UINT8,
                                  PLY_INT);
            ply_write_header(plyFile);

            for (int i = 0; i < nVertices; ++i) {
                ply_write(plyFile, P[i].x);
                ply_write(plyFile, P[i].y);
                ply_write(plyFile, P[i].z);
                if (N) {
                    ply_write(plyFile, N[i].x);
                    ply_write(plyFile, N[i].y);
                    ply_write(plyFile, N[i].z);
                }
                if (UV) {
                    ply_write(plyFile, UV[i].x);
                    ply_write(plyFile, UV[i].y);
                }
            }

            for (int i = 0; i < nTriangles; ++i) {
                ply_write(plyFile, 3);
                ply_write(plyFile, vertexIndices[3 * i]);
                ply_write(plyFile, vertexIndices[3 * i + 1]);
                ply_write(plyFile, vertexIndices[3 * i + 2]);
            }
            ply_close(plyFile);
        }
    }
    // Transform mesh vertices to world space
    p.reset(new Point3f[nVertices]);
    for (int i = 0; i < nVertices; ++i) p[i] = ObjectToWorld(P[i]);

    // Copy _UV_, _N_, and _S_ vertex data, if present
    if (UV) {
        uv.reset(new Point2f[nVertices]);
        memcpy(uv.get(), UV, nVertices * sizeof(Point2f));
    }
    if (N) {
        n.reset(new Normal3f[nVertices]);
        for (int i = 0; i < nVertices; ++i) n[i] = ObjectToWorld(N[i]);
    }
    if (S) {
        s.reset(new Vector3f[nVertices]);
        for (int i = 0; i < nVertices; ++i) s[i] = ObjectToWorld(S[i]);
    }
}

std::vector<std::shared_ptr<Shape>> CreateTriangleMesh(
    const Transform *ObjectToWorld, const Transform *WorldToObject,
    bool reverseOrientation, int nTriangles, const int *vertexIndices,
    int nVertices, const Point3f *p, const Vector3f *s, const Normal3f *n,
    const Point2f *uv, const std::shared_ptr<Texture<Float>> &alphaMask) {
    std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>(
        *ObjectToWorld, nTriangles, vertexIndices, nVertices, p, s, n, uv,
        alphaMask);
    std::vector<std::shared_ptr<Shape>> tris;
    tris.reserve(nTriangles);
    for (int i = 0; i < nTriangles; ++i)
        tris.push_back(std::make_shared<Triangle>(ObjectToWorld, WorldToObject,
                                                  reverseOrientation, mesh, i));
    return tris;
}

Bounds3f Triangle::ObjectBound() const {
    // Get triangle vertices in _p0_, _p1_, and _p2_
    const Point3f &p0 = mesh->p[v[0]];
    const Point3f &p1 = mesh->p[v[1]];
    const Point3f &p2 = mesh->p[v[2]];
    return Union(Bounds3f((*WorldToObject)(p0), (*WorldToObject)(p1)),
                 (*WorldToObject)(p2));
}

Bounds3f Triangle::WorldBound() const {
    // Get triangle vertices in _p0_, _p1_, and _p2_
    const Point3f &p0 = mesh->p[v[0]];
    const Point3f &p1 = mesh->p[v[1]];
    const Point3f &p2 = mesh->p[v[2]];
    return Union(Bounds3f(p0, p1), p2);
}

bool Triangle::Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                         bool testAlphaTexture) const {
    ProfilePhase p(Prof::TriIntersect);
    ++nTests;
    // Get triangle vertices in _p0_, _p1_, and _p2_
    const Point3f &p0 = mesh->p[v[0]];
    const Point3f &p1 = mesh->p[v[1]];
    const Point3f &p2 = mesh->p[v[2]];

    // Perform ray--triangle intersection test

    // Transform triangle vertices to ray coordinate space

    // Translate vertices based on ray origin
    Point3f p0t = p0 - Vector3f(ray.o);
    Point3f p1t = p1 - Vector3f(ray.o);
    Point3f p2t = p2 - Vector3f(ray.o);

    // Permute components of triangle vertices and ray direction
    int kz = MaxDimension(Abs(ray.d));
    int kx = kz + 1;
    if (kx == 3) kx = 0;
    int ky = kx + 1;
    if (ky == 3) ky = 0;
    Vector3f d = Permute(ray.d, kx, ky, kz);
    p0t = Permute(p0t, kx, ky, kz);
    p1t = Permute(p1t, kx, ky, kz);
    p2t = Permute(p2t, kx, ky, kz);

    // Apply shear transformation to translated vertex positions
    Float Sx = -d.x / d.z;
    Float Sy = -d.y / d.z;
    Float Sz = 1.f / d.z;
    p0t.x += Sx * p0t.z;
    p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z;
    p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z;
    p2t.y += Sy * p2t.z;

    // Compute edge function coefficients _e0_, _e1_, and _e2_
    Float e0 = p1t.x * p2t.y - p1t.y * p2t.x;
    Float e1 = p2t.x * p0t.y - p2t.y * p0t.x;
    Float e2 = p0t.x * p1t.y - p0t.y * p1t.x;

    // Fall back to double precision test at triangle edges
    if (sizeof(Float) == sizeof(float) &&
        (e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
        double p2txp1ty = (double)p2t.x * (double)p1t.y;
        double p2typ1tx = (double)p2t.y * (double)p1t.x;
        e0 = (float)(p2typ1tx - p2txp1ty);
        double p0txp2ty = (double)p0t.x * (double)p2t.y;
        double p0typ2tx = (double)p0t.y * (double)p2t.x;
        e1 = (float)(p0typ2tx - p0txp2ty);
        double p1txp0ty = (double)p1t.x * (double)p0t.y;
        double p1typ0tx = (double)p1t.y * (double)p0t.x;
        e2 = (float)(p1typ0tx - p1txp0ty);
    }

    // Perform triangle edge and determinant tests
    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
        return false;
    Float det = e0 + e1 + e2;
    if (det == 0) return false;

    // Compute scaled hit distance to triangle and test against ray $t$ range
    p0t.z *= Sz;
    p1t.z *= Sz;
    p2t.z *= Sz;
    Float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
        return false;
    else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
        return false;

    // Compute barycentric coordinates and $t$ value for triangle intersection
    Float invDet = 1 / det;
    Float b0 = e0 * invDet;
    Float b1 = e1 * invDet;
    Float b2 = e2 * invDet;
    Float t = tScaled * invDet;

    // Ensure that computed triangle $t$ is conservatively greater than zero

    // Compute $\delta_z$ term for triangle $t$ error bounds
    Float maxZt = MaxComponent(Abs(Vector3f(p0t.z, p1t.z, p2t.z)));
    Float deltaZ = gamma(3) * maxZt;

    // Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
    Float maxXt = MaxComponent(Abs(Vector3f(p0t.x, p1t.x, p2t.x)));
    Float maxYt = MaxComponent(Abs(Vector3f(p0t.y, p1t.y, p2t.y)));
    Float deltaX = gamma(5) * (maxXt + maxZt);
    Float deltaY = gamma(5) * (maxYt + maxZt);

    // Compute $\delta_e$ term for triangle $t$ error bounds
    Float deltaE =
        2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);

    // Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
    Float maxE = MaxComponent(Abs(Vector3f(e0, e1, e2)));
    Float deltaT = 3 *
                   (gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
                   std::abs(invDet);
    if (t <= deltaT) return false;
#ifndef NDEBUG
    if (false) {
// Check triangle $t$ against reference high-precision solution
#define P(f)                                                             \
    fprintf(stderr, "%s: %g / %Lg (abs err %g rel %g ulpdiff %d)\n", #f, \
            (float)(f), (f).PreciseValue(), (f).GetAbsoluteError(),      \
            (f).GetRelativeError(),                                      \
            FloatToBits((float)(f).PreciseValue()) - FloatToBits((float)(f)))
        EFloat Sx = EFloat(ray.d[kx]) / EFloat(ray.d[kz]);
        EFloat Sy = EFloat(ray.d[ky]) / EFloat(ray.d[kz]);
        EFloat Sz = EFloat(1.0f) / EFloat(ray.d[kz]);

        EFloat p0t[3], p1t[3], p2t[3];
        for (int i = 0; i < 3; ++i) {
            p0t[i] = EFloat(p0[i]) - EFloat(ray.o[i]);
            p1t[i] = EFloat(p1[i]) - EFloat(ray.o[i]);
            p2t[i] = EFloat(p2[i]) - EFloat(ray.o[i]);
        }

        // Transform triangle vertices to ray coordinate space
        EFloat p0tx = p0t[kx] - Sx * p0t[kz];
        EFloat p0ty = p0t[ky] - Sy * p0t[kz];
        EFloat p1tx = p1t[kx] - Sx * p1t[kz];
        EFloat p1ty = p1t[ky] - Sy * p1t[kz];
        EFloat p2tx = p2t[kx] - Sx * p2t[kz];
        EFloat p2ty = p2t[ky] - Sy * p2t[kz];

        // Compute edge function coefficients \mono{e0}, \mono{e1}, and
        // \mono{e2}
        EFloat e0 = p2tx * p1ty - p2ty * p1tx;
        EFloat e1 = p0tx * p2ty - p0ty * p2tx;
        EFloat e2 = p1tx * p0ty - p1ty * p0tx;

        EFloat det = e0 + e1 + e2;

        // Compute scaled hit distance to triangle and test against ray $t$
        // range
        EFloat p0tz = Sz * p0t[kz];
        EFloat p1tz = Sz * p1t[kz];
        EFloat p2tz = Sz * p2t[kz];
        EFloat tScaled = e0 * p0tz + e1 * p1tz + e2 * p2tz;

        EFloat invDet = 1.0f / det;
        EFloat t = tScaled * invDet;

        if ((float)t > 0 && t.PreciseValue() < 0.f) {
            fprintf(stderr,
                    "\nBoundz? t %.10g precise %.10Lg, tScaled %.10g "
                    "tErrorBound %.10g (fail factor %f), fwe bound %.10g\n",
                    (float)t, t.PreciseValue(), (float)tScaled, deltaT,
                    (float)tScaled / deltaT, t.GetAbsoluteError());
#if 1
            // P(p0tx);
            // P(p0ty);
            // P(p1tx);
            // P(p1ty);
            // P(p2tx);
            // P(p2ty);
            P(e0);
            P(e1);
            P(e2);
            P(det);
            P(p0tz);
            P(p1tz);
            P(p2tz);
            P(e0 * p0tz);
            P(e1 * p1tz);
            P(e2 * p2tz);
            P(tScaled);
            P(invDet);
            P(t);
            P((e0 / det) * p0tz + (e1 / det) * p1tz + (e2 / det) * p2tz);
#endif
            //    abort();
            return false;
        }
#undef P
    }
#endif  // !NDEBUG

    // Compute triangle partial derivatives
    Vector3f dpdu, dpdv;
    Point2f uv[3];
    GetUVs(uv);

    // Compute deltas for triangle partial derivatives
    Vector2f duv02 = uv[0] - uv[2], duv12 = uv[1] - uv[2];
    Vector3f dp02 = p0 - p2, dp12 = p1 - p2;
    Float determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
    if (determinant == 0) {
        // Handle zero determinant for triangle partial derivative matrix
        CoordinateSystem(Normalize(Cross(p2 - p0, p1 - p0)), &dpdu, &dpdv);
    } else {
        Float invdet = 1 / determinant;
        dpdu = (duv12[1] * dp02 - duv02[1] * dp12) * invdet;
        dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * invdet;
    }

    // Compute error bounds for triangle intersection
    Float xAbsSum =
        (std::abs(b0 * p0.x) + std::abs(b1 * p1.x) + std::abs(b2 * p2.x));
    Float yAbsSum =
        (std::abs(b0 * p0.y) + std::abs(b1 * p1.y) + std::abs(b2 * p2.y));
    Float zAbsSum =
        (std::abs(b0 * p0.z) + std::abs(b1 * p1.z) + std::abs(b2 * p2.z));
    Vector3f pError = gamma(7) * Vector3f(xAbsSum, yAbsSum, zAbsSum);

    // Interpolate $(u,v)$ parametric coordinates and hit point
    Point3f pHit = b0 * p0 + b1 * p1 + b2 * p2;
    Point2f uvHit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];

    // Test intersection against alpha texture, if present
    if (testAlphaTexture && mesh->alphaMask) {
        SurfaceInteraction isectLocal(
            pHit, Vector3f(0, 0, 0), uvHit, Vector3f(0, 0, 0), dpdu, dpdv,
            Normal3f(0, 0, 0), Normal3f(0, 0, 0), ray.time, this);
        if (mesh->alphaMask->Evaluate(isectLocal) == 0) return false;
    }

    // Fill in _SurfaceInteraction_ from triangle hit
    *isect = SurfaceInteraction(pHit, pError, uvHit, -ray.d, dpdu, dpdv,
                                Normal3f(0, 0, 0), Normal3f(0, 0, 0), ray.time,
                                this);

    // Override surface normal in _isect_ for triangle
    isect->n = isect->shading.n = Normal3f(Normalize(Cross(dp02, dp12)));
    if (mesh->n || mesh->s) {
        // Initialize _Triangle_ shading geometry

        // Compute shading normal _ns_ for triangle
        Normal3f ns;
        if (mesh->n)
            ns = Normalize(b0 * mesh->n[v[0]] + b1 * mesh->n[v[1]] +
                           b2 * mesh->n[v[2]]);
        else
            ns = isect->n;

        // Compute shading tangent _ss_ for triangle
        Vector3f ss;
        if (mesh->s)
            ss = Normalize(b0 * mesh->s[v[0]] + b1 * mesh->s[v[1]] +
                           b2 * mesh->s[v[2]]);
        else
            ss = Normalize(isect->dpdu);

        // Compute shading bitangent _ts_ for triangle and adjust _ss_
        Vector3f ts = Cross(ss, ns);
        if (ts.LengthSquared() > 0.f) {
            ts = Normalize(ts);
            ss = Cross(ts, ns);
        } else
            CoordinateSystem((Vector3f)ns, &ss, &ts);

        // Compute $\dndu$ and $\dndv$ for triangle shading geometry
        Normal3f dndu, dndv;
        if (mesh->n) {
            // Compute deltas for triangle partial derivatives of normal
            Vector2f duv02 = uv[0] - uv[2];
            Vector2f duv12 = uv[1] - uv[2];
            Normal3f dn1 = mesh->n[v[0]] - mesh->n[v[2]];
            Normal3f dn2 = mesh->n[v[1]] - mesh->n[v[2]];
            Float determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
            if (determinant == 0)
                dndu = dndv = Normal3f(0, 0, 0);
            else {
                Float invDet = 1 / determinant;
                dndu = (duv12[1] * dn1 - duv02[1] * dn2) * invDet;
                dndv = (-duv12[0] * dn1 + duv02[0] * dn2) * invDet;
            }
        } else
            dndu = dndv = Normal3f(0, 0, 0);
        isect->SetShadingGeometry(ss, ts, dndu, dndv, true);
    }

    // Ensure correct orientation of the geometric normal
    if (mesh->n)
        isect->n = Faceforward(isect->n, isect->shading.n);
    else if (reverseOrientation ^ transformSwapsHandedness)
        isect->n = isect->shading.n = -isect->n;
    *tHit = t;
    ++nHits;
    return true;
}

bool Triangle::IntersectP(const Ray &ray, bool testAlphaTexture) const {
    ProfilePhase p(Prof::TriIntersectP);
    ++nTests;
    // Get triangle vertices in _p0_, _p1_, and _p2_
    const Point3f &p0 = mesh->p[v[0]];
    const Point3f &p1 = mesh->p[v[1]];
    const Point3f &p2 = mesh->p[v[2]];

    // Perform ray--triangle intersection test

    // Transform triangle vertices to ray coordinate space

    // Translate vertices based on ray origin
    Point3f p0t = p0 - Vector3f(ray.o);
    Point3f p1t = p1 - Vector3f(ray.o);
    Point3f p2t = p2 - Vector3f(ray.o);

    // Permute components of triangle vertices and ray direction
    int kz = MaxDimension(Abs(ray.d));
    int kx = kz + 1;
    if (kx == 3) kx = 0;
    int ky = kx + 1;
    if (ky == 3) ky = 0;
    Vector3f d = Permute(ray.d, kx, ky, kz);
    p0t = Permute(p0t, kx, ky, kz);
    p1t = Permute(p1t, kx, ky, kz);
    p2t = Permute(p2t, kx, ky, kz);

    // Apply shear transformation to translated vertex positions
    Float Sx = -d.x / d.z;
    Float Sy = -d.y / d.z;
    Float Sz = 1.f / d.z;
    p0t.x += Sx * p0t.z;
    p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z;
    p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z;
    p2t.y += Sy * p2t.z;

    // Compute edge function coefficients _e0_, _e1_, and _e2_
    Float e0 = p1t.x * p2t.y - p1t.y * p2t.x;
    Float e1 = p2t.x * p0t.y - p2t.y * p0t.x;
    Float e2 = p0t.x * p1t.y - p0t.y * p1t.x;

    // Fall back to double precision test at triangle edges
    if (sizeof(Float) == sizeof(float) &&
        (e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
        double p2txp1ty = (double)p2t.x * (double)p1t.y;
        double p2typ1tx = (double)p2t.y * (double)p1t.x;
        e0 = (float)(p2typ1tx - p2txp1ty);
        double p0txp2ty = (double)p0t.x * (double)p2t.y;
        double p0typ2tx = (double)p0t.y * (double)p2t.x;
        e1 = (float)(p0typ2tx - p0txp2ty);
        double p1txp0ty = (double)p1t.x * (double)p0t.y;
        double p1typ0tx = (double)p1t.y * (double)p0t.x;
        e2 = (float)(p1typ0tx - p1txp0ty);
    }

    // Perform triangle edge and determinant tests
    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
        return false;
    Float det = e0 + e1 + e2;
    if (det == 0) return false;

    // Compute scaled hit distance to triangle and test against ray $t$ range
    p0t.z *= Sz;
    p1t.z *= Sz;
    p2t.z *= Sz;
    Float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
        return false;
    else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
        return false;

    // Compute barycentric coordinates and $t$ value for triangle intersection
    Float invDet = 1 / det;
    Float b0 = e0 * invDet;
    Float b1 = e1 * invDet;
    Float b2 = e2 * invDet;
    Float t = tScaled * invDet;

    // Ensure that computed triangle $t$ is conservatively greater than zero

    // Compute $\delta_z$ term for triangle $t$ error bounds
    Float maxZt = MaxComponent(Abs(Vector3f(p0t.z, p1t.z, p2t.z)));
    Float deltaZ = gamma(3) * maxZt;

    // Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
    Float maxXt = MaxComponent(Abs(Vector3f(p0t.x, p1t.x, p2t.x)));
    Float maxYt = MaxComponent(Abs(Vector3f(p0t.y, p1t.y, p2t.y)));
    Float deltaX = gamma(5) * (maxXt + maxZt);
    Float deltaY = gamma(5) * (maxYt + maxZt);

    // Compute $\delta_e$ term for triangle $t$ error bounds
    Float deltaE =
        2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);

    // Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
    Float maxE = MaxComponent(Abs(Vector3f(e0, e1, e2)));
    Float deltaT = 3 *
                   (gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
                   std::abs(invDet);
    if (t <= deltaT) return false;
#ifndef NDEBUG
    if (false) {
// Check triangle $t$ against reference high-precision solution
#define P(f)                                                             \
    fprintf(stderr, "%s: %g / %Lg (abs err %g rel %g ulpdiff %d)\n", #f, \
            (float)(f), (f).PreciseValue(), (f).GetAbsoluteError(),      \
            (f).GetRelativeError(),                                      \
            FloatToBits((float)(f).PreciseValue()) - FloatToBits((float)(f)))
        EFloat Sx = EFloat(ray.d[kx]) / EFloat(ray.d[kz]);
        EFloat Sy = EFloat(ray.d[ky]) / EFloat(ray.d[kz]);
        EFloat Sz = EFloat(1.0f) / EFloat(ray.d[kz]);

        EFloat p0t[3], p1t[3], p2t[3];
        for (int i = 0; i < 3; ++i) {
            p0t[i] = EFloat(p0[i]) - EFloat(ray.o[i]);
            p1t[i] = EFloat(p1[i]) - EFloat(ray.o[i]);
            p2t[i] = EFloat(p2[i]) - EFloat(ray.o[i]);
        }

        // Transform triangle vertices to ray coordinate space
        EFloat p0tx = p0t[kx] - Sx * p0t[kz];
        EFloat p0ty = p0t[ky] - Sy * p0t[kz];
        EFloat p1tx = p1t[kx] - Sx * p1t[kz];
        EFloat p1ty = p1t[ky] - Sy * p1t[kz];
        EFloat p2tx = p2t[kx] - Sx * p2t[kz];
        EFloat p2ty = p2t[ky] - Sy * p2t[kz];

        // Compute edge function coefficients \mono{e0}, \mono{e1}, and
        // \mono{e2}
        EFloat e0 = p2tx * p1ty - p2ty * p1tx;
        EFloat e1 = p0tx * p2ty - p0ty * p2tx;
        EFloat e2 = p1tx * p0ty - p1ty * p0tx;

        EFloat det = e0 + e1 + e2;

        // Compute scaled hit distance to triangle and test against ray $t$
        // range
        EFloat p0tz = Sz * p0t[kz];
        EFloat p1tz = Sz * p1t[kz];
        EFloat p2tz = Sz * p2t[kz];
        EFloat tScaled = e0 * p0tz + e1 * p1tz + e2 * p2tz;

        EFloat invDet = 1.0f / det;
        EFloat t = tScaled * invDet;

        if ((float)t > 0 && t.PreciseValue() < 0.f) {
            fprintf(stderr,
                    "\nBoundz? t %.10g precise %.10Lg, tScaled %.10g "
                    "tErrorBound %.10g (fail factor %f), fwe bound %.10g\n",
                    (float)t, t.PreciseValue(), (float)tScaled, deltaT,
                    (float)tScaled / deltaT, t.GetAbsoluteError());
#if 1
            // P(p0tx);
            // P(p0ty);
            // P(p1tx);
            // P(p1ty);
            // P(p2tx);
            // P(p2ty);
            P(e0);
            P(e1);
            P(e2);
            P(det);
            P(p0tz);
            P(p1tz);
            P(p2tz);
            P(e0 * p0tz);
            P(e1 * p1tz);
            P(e2 * p2tz);
            P(tScaled);
            P(invDet);
            P(t);
            P((e0 / det) * p0tz + (e1 / det) * p1tz + (e2 / det) * p2tz);
#endif
            //    abort();
            return false;
        }
#undef P
    }
#endif  // !NDEBUG

    // Test shadow ray intersection against alpha texture, if present
    if (testAlphaTexture && mesh->alphaMask) {
        // Compute triangle partial derivatives
        Vector3f dpdu, dpdv;
        Point2f uv[3];
        GetUVs(uv);

        // Compute deltas for triangle partial derivatives
        Vector2f duv02 = uv[0] - uv[2], duv12 = uv[1] - uv[2];
        Vector3f dp02 = p0 - p2, dp12 = p1 - p2;
        Float determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
        if (determinant == 0) {
            // Handle zero determinant for triangle partial derivative matrix
            CoordinateSystem(Normalize(Cross(p2 - p0, p1 - p0)), &dpdu, &dpdv);
        } else {
            Float invdet = 1 / determinant;
            dpdu = (duv12[1] * dp02 - duv02[1] * dp12) * invdet;
            dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * invdet;
        }

        // Interpolate $(u,v)$ parametric coordinates and hit point
        Point3f pHit = b0 * p0 + b1 * p1 + b2 * p2;
        Point2f uvHit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];
        SurfaceInteraction isectLocal(
            pHit, Vector3f(0, 0, 0), uvHit, Vector3f(0, 0, 0), dpdu, dpdv,
            Normal3f(0, 0, 0), Normal3f(0, 0, 0), ray.time, this);
        if (mesh->alphaMask->Evaluate(isectLocal) == 0) return false;
    }
    ++nHits;
    return true;
}

Float Triangle::Area() const {
    // Get triangle vertices in _p0_, _p1_, and _p2_
    const Point3f &p0 = mesh->p[v[0]];
    const Point3f &p1 = mesh->p[v[1]];
    const Point3f &p2 = mesh->p[v[2]];
    return 0.5 * Cross(p1 - p0, p2 - p0).Length();
}

Interaction Triangle::Sample(const Point2f &u) const {
    Interaction it;
    Point2f b = UniformSampleTriangle(u);
    // Get triangle vertices in _p0_, _p1_, and _p2_
    const Point3f &p0 = mesh->p[v[0]];
    const Point3f &p1 = mesh->p[v[1]];
    const Point3f &p2 = mesh->p[v[2]];
    it.p = b[0] * p0 + b[1] * p1 + (1 - b[0] - b[1]) * p2;
    // Compute surface normal for sampled point on triangle
    if (mesh->n)
        it.n = Normalize(b[0] * mesh->n[v[0]] + b[1] * mesh->n[v[1]] +
                         (1 - b[0] - b[1]) * mesh->n[v[2]]);
    else
        it.n = Normalize(Normal3f(Cross(p1 - p0, p2 - p0)));
    if (reverseOrientation) it.n *= -1.f;

    // Compute error bounds for sampled point on triangle
    Point3f pAbsSum =
        Abs(b[0] * p0) + Abs(b[1] * p1) + Abs((1 - b[0] - b[1]) * p2);
    it.pError = gamma(6) * Vector3f(pAbsSum.x, pAbsSum.y, pAbsSum.z);
    return it;
}

std::vector<std::shared_ptr<Shape>> CreateTriangleMeshShape(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    const ParamSet &params,
    std::map<std::string, std::shared_ptr<Texture<Float>>> *floatTextures) {
    int nvi, npi, nuvi, nsi, nni;
    const int *vi = params.FindInt("indices", &nvi);
    const Point3f *P = params.FindPoint3f("P", &npi);
    const Point2f *uvs = params.FindPoint2f("uv", &nuvi);
    if (!uvs) uvs = params.FindPoint2f("st", &nuvi);
    std::vector<Point2f> tempUVs;
    if (!uvs) {
        const Float *fuv = params.FindFloat("uv", &nuvi);
        if (!fuv) fuv = params.FindFloat("st", &nuvi);
        if (fuv) {
            nuvi /= 2;
            tempUVs.reserve(nuvi);
            for (int i = 0; i < nuvi; ++i)
                tempUVs.push_back(Point2f(fuv[2 * i], fuv[2 * i + 1]));
            uvs = &tempUVs[0];
        }
    }
    bool discardDegenerateUVs =
        params.FindOneBool("discarddegenerateUVs", false);
    if (uvs) {
        if (nuvi < npi) {
            Error(
                "Not enough of \"uv\"s for triangle mesh.  Expencted %d, "
                "found %d.  Discarding.",
                npi, nuvi);
            uvs = nullptr;
        } else if (nuvi > npi)
            Warning(
                "More \"uv\"s provided than will be used for triangle "
                "mesh.  (%d expcted, %d found)",
                npi, nuvi);
    }
    if (!vi) {
        Error(
            "Vertex indices \"indices\" not provided with triangle mesh shape");
        return std::vector<std::shared_ptr<Shape>>();
        ;
    }
    if (!P) {
        Error("Vertex positions \"P\" not provided with triangle mesh shape");
        return std::vector<std::shared_ptr<Shape>>();
        ;
    }
    const Vector3f *S = params.FindVector3f("S", &nsi);
    if (S && nsi != npi) {
        Error("Number of \"S\"s for triangle mesh must match \"P\"s");
        S = nullptr;
    }
    const Normal3f *N = params.FindNormal3f("N", &nni);
    if (N && nni != npi) {
        Error("Number of \"N\"s for triangle mesh must match \"P\"s");
        N = nullptr;
    }
    if (discardDegenerateUVs && uvs && N) {
        // if there are normals, check for bad uv's that
        // give degenerate mappings; discard them if so
        const int *vp = vi;
        for (int i = 0; i < nvi; i += 3, vp += 3) {
            Float area =
                .5f * Cross(P[vp[0]] - P[vp[1]], P[vp[2]] - P[vp[1]]).Length();
            if (area < 1e-7) continue;  // ignore degenerate tris.
            if ((uvs[vp[0]].x == uvs[vp[1]].x &&
                 uvs[vp[0]].y == uvs[vp[1]].y) ||
                (uvs[vp[1]].x == uvs[vp[2]].x &&
                 uvs[vp[1]].y == uvs[vp[2]].y) ||
                (uvs[vp[2]].x == uvs[vp[0]].x &&
                 uvs[vp[2]].y == uvs[vp[0]].y)) {
                Warning(
                    "Degenerate uv coordinates in triangle mesh.  Discarding "
                    "all uvs.");
                uvs = nullptr;
                break;
            }
        }
    }
    for (int i = 0; i < nvi; ++i)
        if (vi[i] >= npi) {
            Error(
                "trianglemesh has out of-bounds vertex index %d (%d \"P\" "
                "values were given",
                vi[i], npi);
            return std::vector<std::shared_ptr<Shape>>();
            ;
        }

    std::shared_ptr<Texture<Float>> alphaTex;
    std::string alphaTexName = params.FindTexture("alpha");
    if (alphaTexName != "") {
        if (floatTextures->find(alphaTexName) != floatTextures->end())
            alphaTex = (*floatTextures)[alphaTexName];
        else
            Error("Couldn't find float texture \"%s\" for \"alpha\" parameter",
                  alphaTexName.c_str());
    } else if (params.FindOneFloat("alpha", 1.f) == 0.f)
        alphaTex.reset(new ConstantTexture<Float>(0.f));
    return CreateTriangleMesh(o2w, w2o, reverseOrientation, nvi / 3, vi, npi, P,
                              S, N, uvs, alphaTex);
}
