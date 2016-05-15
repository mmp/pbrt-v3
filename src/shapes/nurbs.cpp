
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


// shapes/nurbs.cpp*
#include "shapes/nurbs.h"
#include "shapes/triangle.h"
#include "paramset.h"
#include "texture.h"

// NURBS Evaluation Functions
static int KnotOffset(const Float *knot, int order, int np, Float t) {
    int firstKnot = order - 1;

    int knotOffset = firstKnot;
    while (t > knot[knotOffset + 1]) ++knotOffset;
    Assert(knotOffset < np);  // np == lastKnot
    Assert(t >= knot[knotOffset] && t <= knot[knotOffset + 1]);
    return knotOffset;
}

// doesn't handle flat out discontinuities in the curve...

struct Homogeneous3 {
    Homogeneous3() { x = y = z = w = 0.; }
    Homogeneous3(Float xx, Float yy, Float zz, Float ww) {
        x = xx;
        y = yy;
        z = zz;
        w = ww;
    }

    Float x, y, z, w;
};

static Homogeneous3 NURBSEvaluate(int order, const Float *knot,
                                  const Homogeneous3 *cp, int np, int cpStride,
                                  Float t, Vector3f *deriv = nullptr) {
    //    int nKnots = np + order;
    Float alpha;

    int knotOffset = KnotOffset(knot, order, np, t);
    knot += knotOffset;

    int cpOffset = knotOffset - order + 1;
    Assert(cpOffset >= 0 && cpOffset < np);

    Homogeneous3 *cpWork = ALLOCA(Homogeneous3, order);
    for (int i = 0; i < order; ++i) cpWork[i] = cp[(cpOffset + i) * cpStride];

    for (int i = 0; i < order - 2; ++i)
        for (int j = 0; j < order - 1 - i; ++j) {
            alpha = (knot[1 + j] - t) / (knot[1 + j] - knot[j + 2 - order + i]);
            Assert(alpha >= 0. && alpha <= 1.);

            cpWork[j].x = cpWork[j].x * alpha + cpWork[j + 1].x * (1 - alpha);
            cpWork[j].y = cpWork[j].y * alpha + cpWork[j + 1].y * (1 - alpha);
            cpWork[j].z = cpWork[j].z * alpha + cpWork[j + 1].z * (1 - alpha);
            cpWork[j].w = cpWork[j].w * alpha + cpWork[j + 1].w * (1 - alpha);
        }

    alpha = (knot[1] - t) / (knot[1] - knot[0]);
    Assert(alpha >= 0. && alpha <= 1.);

    Homogeneous3 val(cpWork[0].x * alpha + cpWork[1].x * (1 - alpha),
                     cpWork[0].y * alpha + cpWork[1].y * (1 - alpha),
                     cpWork[0].z * alpha + cpWork[1].z * (1 - alpha),
                     cpWork[0].w * alpha + cpWork[1].w * (1 - alpha));

    if (deriv) {
        Float factor = (order - 1) / (knot[1] - knot[0]);
        Homogeneous3 delta((cpWork[1].x - cpWork[0].x) * factor,
                           (cpWork[1].y - cpWork[0].y) * factor,
                           (cpWork[1].z - cpWork[0].z) * factor,
                           (cpWork[1].w - cpWork[0].w) * factor);

        deriv->x = delta.x / val.w - (val.x * delta.w / (val.w * val.w));
        deriv->y = delta.y / val.w - (val.y * delta.w / (val.w * val.w));
        deriv->z = delta.z / val.w - (val.z * delta.w / (val.w * val.w));
    }

    return val;
}

static Point3f NURBSEvaluateSurface(int uOrder, const Float *uKnot, int ucp,
                                    Float u, int vOrder, const Float *vKnot,
                                    int vcp, Float v, const Homogeneous3 *cp,
                                    Vector3f *dpdu, Vector3f *dpdv) {
    Homogeneous3 *iso = ALLOCA(Homogeneous3, std::max(uOrder, vOrder));

    int uOffset = KnotOffset(uKnot, uOrder, ucp, u);
    int uFirstCp = uOffset - uOrder + 1;
    Assert(uFirstCp >= 0 && uFirstCp + uOrder - 1 < ucp);

    for (int i = 0; i < uOrder; ++i)
        iso[i] = NURBSEvaluate(vOrder, vKnot, &cp[uFirstCp + i], vcp, ucp, v);

    int vOffset = KnotOffset(vKnot, vOrder, vcp, v);
    int vFirstCp = vOffset - vOrder + 1;
    Assert(vFirstCp >= 0 && vFirstCp + vOrder - 1 < vcp);

    Homogeneous3 P =
        NURBSEvaluate(uOrder, uKnot, iso - uFirstCp, ucp, 1, u, dpdu);

    if (dpdv) {
        for (int i = 0; i < vOrder; ++i)
            iso[i] = NURBSEvaluate(uOrder, uKnot, &cp[(vFirstCp + i) * ucp],
                                   ucp, 1, u);
        (void)NURBSEvaluate(vOrder, vKnot, iso - vFirstCp, vcp, 1, v, dpdv);
    }
    return Point3f(P.x / P.w, P.y / P.w, P.z / P.w);
}

std::vector<std::shared_ptr<Shape>> CreateNURBS(const Transform *o2w,
                                                const Transform *w2o,
                                                bool reverseOrientation,
                                                const ParamSet &params) {
    int nu = params.FindOneInt("nu", -1);
    if (nu == -1) {
        Error("Must provide number of control points \"nu\" with NURBS shape.");
        return std::vector<std::shared_ptr<Shape>>();
    }

    int uorder = params.FindOneInt("uorder", -1);
    if (uorder == -1) {
        Error("Must provide u order \"uorder\" with NURBS shape.");
        return std::vector<std::shared_ptr<Shape>>();
    }
    int nuknots, nvknots;
    const Float *uknots = params.FindFloat("uknots", &nuknots);
    if (!uknots) {
        Error("Must provide u knot vector \"uknots\" with NURBS shape.");
        return std::vector<std::shared_ptr<Shape>>();
    }

    if (nuknots != nu + uorder) {
        Error(
            "Number of knots in u knot vector %d doesn't match sum of "
            "number of u control points %d and u order %d.",
            nuknots, nu, uorder);
        return std::vector<std::shared_ptr<Shape>>();
    }

    Float u0 = params.FindOneFloat("u0", uknots[uorder - 1]);
    Float u1 = params.FindOneFloat("u1", uknots[nu]);

    int nv = params.FindOneInt("nv", -1);
    if (nv == -1) {
        Error("Must provide number of control points \"nv\" with NURBS shape.");
        return std::vector<std::shared_ptr<Shape>>();
    }

    int vorder = params.FindOneInt("vorder", -1);
    if (vorder == -1) {
        Error("Must provide v order \"vorder\" with NURBS shape.");
        return std::vector<std::shared_ptr<Shape>>();
    }

    const Float *vknots = params.FindFloat("vknots", &nvknots);
    if (!vknots) {
        Error("Must provide v knot vector \"vknots\" with NURBS shape.");
        return std::vector<std::shared_ptr<Shape>>();
    }

    if (nvknots != nv + vorder) {
        Error(
            "Number of knots in v knot vector %d doesn't match sum of "
            "number of v control points %d and v order %d.",
            nvknots, nv, vorder);
        return std::vector<std::shared_ptr<Shape>>();
    }

    Float v0 = params.FindOneFloat("v0", vknots[vorder - 1]);
    Float v1 = params.FindOneFloat("v1", vknots[nv]);

    bool isHomogeneous = false;
    int npts;
    const Float *P = (const Float *)params.FindPoint3f("P", &npts);
    if (!P) {
        P = params.FindFloat("Pw", &npts);
        if (!P) {
            Error(
                "Must provide control points via \"P\" or \"Pw\" parameter to "
                "NURBS shape.");
            return std::vector<std::shared_ptr<Shape>>();
        }
        if ((npts % 4) != 0) {
            Error(
                "Number of \"Pw\" control points provided to NURBS shape must "
                "be "
                "multiple of four");
            return std::vector<std::shared_ptr<Shape>>();
        }
        npts /= 4;
        isHomogeneous = true;
    }
    if (npts != nu * nv) {
        Error("NURBS shape was expecting %dx%d=%d control points, was given %d",
              nu, nv, nu * nv, npts);
        return std::vector<std::shared_ptr<Shape>>();
    }

    // Compute NURBS dicing rates
    int diceu = 30, dicev = 30;
    std::unique_ptr<Float[]> ueval(new Float[diceu]);
    std::unique_ptr<Float[]> veval(new Float[dicev]);
    std::unique_ptr<Point3f[]> evalPs(new Point3f[diceu * dicev]);
    std::unique_ptr<Normal3f[]> evalNs(new Normal3f[diceu * dicev]);
    int i;
    for (i = 0; i < diceu; ++i)
        ueval[i] = Lerp((float)i / (float)(diceu - 1), u0, u1);
    for (i = 0; i < dicev; ++i)
        veval[i] = Lerp((float)i / (float)(dicev - 1), v0, v1);

    // Evaluate NURBS over grid of points
    memset(evalPs.get(), 0, diceu * dicev * sizeof(Point3f));
    memset(evalNs.get(), 0, diceu * dicev * sizeof(Point3f));
    std::unique_ptr<Point2f[]> uvs(new Point2f[diceu * dicev]);

    // Turn NURBS into triangles
    std::unique_ptr<Homogeneous3[]> Pw(new Homogeneous3[nu * nv]);
    if (isHomogeneous) {
        for (int i = 0; i < nu * nv; ++i) {
            Pw[i].x = P[4 * i];
            Pw[i].y = P[4 * i + 1];
            Pw[i].z = P[4 * i + 2];
            Pw[i].w = P[4 * i + 3];
        }
    } else {
        for (int i = 0; i < nu * nv; ++i) {
            Pw[i].x = P[3 * i];
            Pw[i].y = P[3 * i + 1];
            Pw[i].z = P[3 * i + 2];
            Pw[i].w = 1.;
        }
    }

    for (int v = 0; v < dicev; ++v) {
        for (int u = 0; u < diceu; ++u) {
            uvs[(v * diceu + u)].x = ueval[u];
            uvs[(v * diceu + u)].y = veval[v];

            Vector3f dpdu, dpdv;
            Point3f pt = NURBSEvaluateSurface(uorder, uknots, nu, ueval[u],
                                              vorder, vknots, nv, veval[v],
                                              Pw.get(), &dpdu, &dpdv);
            evalPs[v * diceu + u].x = pt.x;
            evalPs[v * diceu + u].y = pt.y;
            evalPs[v * diceu + u].z = pt.z;
            evalNs[v * diceu + u] = Normal3f(Normalize(Cross(dpdu, dpdv)));
        }
    }

    // Generate points-polygons mesh
    int nTris = 2 * (diceu - 1) * (dicev - 1);
    std::unique_ptr<int[]> vertices(new int[3 * nTris]);
    int *vertp = vertices.get();
    // Compute the vertex offset numbers for the triangles
    for (int v = 0; v < dicev - 1; ++v) {
        for (int u = 0; u < diceu - 1; ++u) {
#define VN(u, v) ((v)*diceu + (u))
            *vertp++ = VN(u, v);
            *vertp++ = VN(u + 1, v);
            *vertp++ = VN(u + 1, v + 1);

            *vertp++ = VN(u, v);
            *vertp++ = VN(u + 1, v + 1);
            *vertp++ = VN(u, v + 1);
#undef VN
        }
    }
    int nVerts = diceu * dicev;

    return CreateTriangleMesh(o2w, w2o, reverseOrientation, nTris,
                              vertices.get(), nVerts, evalPs.get(), nullptr,
                              evalNs.get(), uvs.get(), nullptr, nullptr);
}
