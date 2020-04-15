
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


// core/sampling.cpp*
#include "sampling.h"
#include "geometry.h"
#include "shape.h"

namespace pbrt {

// Sampling Function Definitions
void StratifiedSample1D(Float *samp, int nSamples, RNG &rng, bool jitter) {
    Float invNSamples = (Float)1 / nSamples;
    for (int i = 0; i < nSamples; ++i) {
        Float delta = jitter ? rng.UniformFloat() : 0.5f;
        samp[i] = std::min((i + delta) * invNSamples, OneMinusEpsilon);
    }
}

void StratifiedSample2D(Point2f *samp, int nx, int ny, RNG &rng, bool jitter) {
    Float dx = (Float)1 / nx, dy = (Float)1 / ny;
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
            Float jx = jitter ? rng.UniformFloat() : 0.5f;
            Float jy = jitter ? rng.UniformFloat() : 0.5f;
            samp->x = std::min((x + jx) * dx, OneMinusEpsilon);
            samp->y = std::min((y + jy) * dy, OneMinusEpsilon);
            ++samp;
        }
}

void LatinHypercube(Float *samples, int nSamples, int nDim, RNG &rng) {
    // Generate LHS samples along diagonal
    Float invNSamples = (Float)1 / nSamples;
    for (int i = 0; i < nSamples; ++i)
        for (int j = 0; j < nDim; ++j) {
            Float sj = (i + (rng.UniformFloat())) * invNSamples;
            samples[nDim * i + j] = std::min(sj, OneMinusEpsilon);
        }

    // Permute LHS samples in each dimension
    for (int i = 0; i < nDim; ++i) {
        for (int j = 0; j < nSamples; ++j) {
            int other = j + rng.UniformUInt32(nSamples - j);
            std::swap(samples[nDim * j + i], samples[nDim * other + i]);
        }
    }
}

Point2f RejectionSampleDisk(RNG &rng) {
    Point2f p;
    do {
        p.x = 1 - 2 * rng.UniformFloat();
        p.y = 1 - 2 * rng.UniformFloat();
    } while (p.x * p.x + p.y * p.y > 1);
    return p;
}

Vector3f UniformSampleHemisphere(const Point2f &u) {
    Float z = u[0];
    Float r = std::sqrt(std::max((Float)0, (Float)1. - z * z));
    Float phi = 2 * Pi * u[1];
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

Float UniformHemispherePdf() { return Inv2Pi; }

Vector3f UniformSampleSphere(const Point2f &u) {
    Float z = 1 - 2 * u[0];
    Float r = std::sqrt(std::max((Float)0, (Float)1 - z * z));
    Float phi = 2 * Pi * u[1];
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

Float UniformSpherePdf() { return Inv4Pi; }

Point2f UniformSampleDisk(const Point2f &u) {
    Float r = std::sqrt(u[0]);
    Float theta = 2 * Pi * u[1];
    return Point2f(r * std::cos(theta), r * std::sin(theta));
}

Point2f ConcentricSampleDisk(const Point2f &u) {
    // Map uniform random numbers to $[-1,1]^2$
    Point2f uOffset = 2.f * u - Vector2f(1, 1);

    // Handle degeneracy at the origin
    if (uOffset.x == 0 && uOffset.y == 0) return Point2f(0, 0);

    // Apply concentric mapping to point
    Float theta, r;
    if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
        r = uOffset.x;
        theta = PiOver4 * (uOffset.y / uOffset.x);
    } else {
        r = uOffset.y;
        theta = PiOver2 - PiOver4 * (uOffset.x / uOffset.y);
    }
    return r * Point2f(std::cos(theta), std::sin(theta));
}

Float UniformConePdf(Float cosThetaMax) {
    return 1 / (2 * Pi * (1 - cosThetaMax));
}

Vector3f UniformSampleCone(const Point2f &u, Float cosThetaMax) {
    Float cosTheta = ((Float)1 - u[0]) + u[0] * cosThetaMax;
    Float sinTheta = std::sqrt((Float)1 - cosTheta * cosTheta);
    Float phi = u[1] * 2 * Pi;
    return Vector3f(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta,
                    cosTheta);
}

Vector3f UniformSampleCone(const Point2f &u, Float cosThetaMax,
                           const Vector3f &x, const Vector3f &y,
                           const Vector3f &z) {
    Float cosTheta = Lerp(u[0], cosThetaMax, 1.f);
    Float sinTheta = std::sqrt((Float)1. - cosTheta * cosTheta);
    Float phi = u[1] * 2 * Pi;
    return std::cos(phi) * sinTheta * x + std::sin(phi) * sinTheta * y +
           cosTheta * z;
}

Point2f UniformSampleTriangle(const Point2f &u) {
    Float su0 = std::sqrt(u[0]);
    return Point2f(1 - su0, u[1] * su0);
}

Distribution2D::Distribution2D(const Float *func, int nu, int nv) {
    pConditionalV.reserve(nv);
    for (int v = 0; v < nv; ++v) {
        // Compute conditional sampling distribution for $\tilde{v}$
        pConditionalV.emplace_back(new Distribution1D(&func[v * nu], nu));
    }
    // Compute marginal sampling distribution $p[\tilde{v}]$
    std::vector<Float> marginalFunc;
    marginalFunc.reserve(nv);
    for (int v = 0; v < nv; ++v)
        marginalFunc.push_back(pConditionalV[v]->funcInt);
    pMarginal.reset(new Distribution1D(&marginalFunc[0], nv));
}

float SafeSqrt(float x) { return std::sqrt(std::max<float>(0, x)); }

Float Sqr(Float x) { return x * x; }

std::array<Float, 3> SampleSphericalTriangle(const std::array<Point3f, 3> &v,
                                             const Point3f &p, const Point2f &u,
                                             Float *pdf) {
    using Vector3d = Vector3<double>;
    Vector3d a(v[0] - p), b(v[1] - p), c(v[2] - p);
    CHECK_GT(a.LengthSquared(), 0);
    CHECK_GT(b.LengthSquared(), 0);
    CHECK_GT(c.LengthSquared(), 0);
    a = Normalize(a);
    b = Normalize(b);
    c = Normalize(c);

    Vector3d axb = Cross(a, b), bxc = Cross(b, c), cxa = Cross(c, a);
    if (axb.LengthSquared() == 0 || bxc.LengthSquared() == 0 || cxa.LengthSquared() == 0) {
        if (pdf != nullptr) *pdf = 0;
        return {};
    }
    axb = Normalize(axb);
    bxc = Normalize(bxc);
    cxa = Normalize(cxa);

    // See comment in Triangle::SolidAngle() for ordering...
    double alpha = AngleBetween(cxa, -axb);
    double beta = AngleBetween(axb, -bxc);
    double gamma = AngleBetween(bxc, -cxa);

    // Spherical area of the triangle.
    double A = alpha + beta + gamma - Pi;
    if (A <= 0) {
        if (pdf != nullptr) *pdf = 0;
        return {};
    }
    if (pdf != nullptr) *pdf = 1 / A;

    // Uniformly sample triangle area
    double Ap = u[0] * A;

    // Compute sin beta' and cos beta' for the point along the edge b
    // corresponding to the area sampled, A'.

    double sinPhi = std::sin(Ap - alpha);
    double cosPhi = std::cos(Ap - alpha);

    double cosAlpha = std::cos(alpha);
    double uu = cosPhi - cosAlpha;
    double sinAlpha = std::sin(alpha);

    double vv = sinPhi + sinAlpha * Dot(a, b) /* cos c */;
    double cosBetap = (((vv * cosPhi - uu * sinPhi) * cosAlpha - vv) /
                      ((vv * sinPhi + uu * cosPhi) * sinAlpha));
    // Happens if the triangle basically covers the entire hemisphere.
    // We currently depend on calling code to detect this case, which
    // is sort of ugly/unfortunate.
    CHECK(!std::isnan(cosBetap));
    cosBetap = Clamp(cosBetap, -1, 1);
    double sinBetap = SafeSqrt(1 - cosBetap * cosBetap);

    // Gram-Schmidt
    auto GS = [](const Vector3d &a, const Vector3d &b) {
        return Normalize(a - Dot(a, b) * b);
    };

    // Compute c', the point along the arc between b' and a.
    Vector3d cp = cosBetap * a + sinBetap * GS(c, a);

    double cosTheta = 1 - u[1] * (1 - Dot(cp, b));
    double sinTheta = SafeSqrt(1 - cosTheta * cosTheta);

    // Compute direction on the sphere.
    Vector3d w = cosTheta * b + sinTheta * GS(cp, b);

    // Compute barycentrics. Subset of Moller-Trumbore intersection test.
    Vector3d e1(v[1] - v[0]), e2(v[2] - v[0]);
    Vector3d s1 = Cross(w, e2);
    double divisor = Dot(s1, e1);

    if (divisor == 0) {
        // This happens with triangles that cover (nearly) the whole
        // hemisphere.
        return {1.f/3.f, 1.f/3.f, 1.f/3.f};
    }
    double invDivisor = 1 / divisor;

    // Compute first barycentric coordinate
    Vector3d s(p - v[0]);
    double b1 = Dot(s, s1) * invDivisor;

    // Compute second barycentric coordinate
    Vector3d s2 = Cross(s, e1);
    double b2 = Dot(w, s2) * invDivisor;

    // We get goofy barycentrics for very small and very large (w.r.t. the sphere) triangles. Again,
    // we expect the caller to not use this in that case.
    b1 = Clamp(b1, 0, 1);
    b2 = Clamp(b2, 0, 1);
    if (b1 + b2 > 1) {
        b1 /= b1 + b2;
        b2 /= b1 + b2;
    }

    return {Float(1 - b1 - b2), Float(b1), Float(b2)};
}

// From Jim Arvo's SphTri.C
Point2f InvertSphericalTriangleSample(const std::array<Point3f, 3> &v,
                                      const Point3f &p, const Vector3f &w) {
    using Vector3d = Vector3<double>;
    Vector3d a(v[0] - p), b(v[1] - p), c(v[2] - p);
    CHECK_GT(a.LengthSquared(), 0);
    CHECK_GT(b.LengthSquared(), 0);
    CHECK_GT(c.LengthSquared(), 0);
    a = Normalize(a);
    b = Normalize(b);
    c = Normalize(c);

    Vector3d axb = Cross(a, b), bxc = Cross(b, c), cxa = Cross(c, a);
    if (axb.LengthSquared() == 0 || bxc.LengthSquared() == 0 || cxa.LengthSquared() == 0)
        return Point2f(0.5, 0.5);

    axb = Normalize(axb);
    bxc = Normalize(bxc);
    cxa = Normalize(cxa);

    // See comment in Triangle::SolidAngle() for ordering...
    double alpha = AngleBetween(cxa, -axb);
    double beta = AngleBetween(axb, -bxc);
    double gamma = AngleBetween(bxc, -cxa);

    // Spherical area of the triangle.
    double A = alpha + beta + gamma - Pi;

    // Assume that w is normalized...

    // Compute the new C vertex, which lies on the arc defined by b-w
    // and the arc defined by a-c.
    Vector3d cp = Normalize(Cross(Cross(b, Vector3d(w)), Cross(c, a)));

    // Adjust the sign of cp.  Make sure it's on the arc between A and C.
    if (Dot(cp, a + c) < 0) cp = -cp;

    // Compute x1, the area of the sub-triangle over the original area.
    // The AngleBetween() calls are computing the dihedral angles (a, b, cp)
    // and (a, cp, b) respectively, FWIW...
    Vector3d cnxb = Cross(cp, b), axcn = Cross(a, cp);
    if (cnxb.LengthSquared() == 0 || axcn.LengthSquared() == 0)
        return Point2f(0.5, 0.5);
    cnxb = Normalize(cnxb);
    axcn = Normalize(axcn);

    Float sub_area = alpha + AngleBetween(axb, cnxb) + AngleBetween(axcn, -cnxb) - Pi;
    Float u0 = sub_area / A;

    // Now compute the second coordinate using the new C vertex.
    Float z = Dot(Vector3d(w), b);
    Float u1 = (1 - z) / (1 - Dot(cp, b));

    return Point2f(Clamp(u0, 0, 1), Clamp(u1, 0, 1));
}

Point3f SampleSphericalQuad(const Point3f &pRef, const Point3f &s, const Vector3f &ex,
                            const Vector3f &ey, const Point2f &u,
                            Float *pdf) {
    // SphQuadInit()
    // local reference system ’R’
    Float exl = ex.Length(), eyl = ey.Length();
    Frame R = Frame::FromXY(ex / exl, ey / eyl);

    // compute rectangle coords in local reference system
    Vector3f d = s - pRef;
    Vector3f dLocal = R.ToLocal(d);
    Float z0 = dLocal.z;

    // flip ’z’ to make it point against ’Q’
    if (z0 > 0) {
        R.z = -R.z;
        z0 *= -1;
    }
    Float z0sq = Sqr(z0);
    Float x0 = dLocal.x;
    Float y0 = dLocal.y;
    Float x1 = x0 + exl;
    Float y1 = y0 + eyl;
    Float y0sq = Sqr(y0), y1sq = Sqr(y1);

    // create vectors to four vertices
    Vector3f v00(x0, y0, z0), v01(x0, y1, z0);
    Vector3f v10(x1, y0, z0), v11(x1, y1, z0);

    // compute normals to edges
    Vector3f n0 = Normalize(Cross(v00, v10));
    Vector3f n1 = Normalize(Cross(v10, v11));
    Vector3f n2 = Normalize(Cross(v11, v01));
    Vector3f n3 = Normalize(Cross(v01, v00));

    // compute internal angles (gamma_i)
    Float g0 = AngleBetween(-n0, n1);
    Float g1 = AngleBetween(-n1, n2);
    Float g2 = AngleBetween(-n2, n3);
    Float g3 = AngleBetween(-n3, n0);

    // compute predefined constants
    Float b0 = n0.z, b1 = n2.z, b0sq = Sqr(b0), b1sq = Sqr(b1);

    // compute solid angle from internal angles
    Float solidAngle = double(g0) + double(g1) + double(g2) + double(g3) - 2. * Pi;
    if (solidAngle <= 0) {
        if (pdf != nullptr) *pdf = 0;
        return Point3f(s + u[0] * ex + u[1] * ey);
    }
    if (pdf != nullptr) *pdf = std::max<Float>(0, 1 / solidAngle);

    if (solidAngle < 1e-3)
        return Point3f(s + u[0] * ex + u[1] * ey);

    // SphQuadSample
    // 1. compute ’cu’
    //Float au = u[0] * solidAngle + k;   // original
    Float au = u[0] * solidAngle - g2 - g3;
    Float fu = (std::cos(au) * b0 - b1) / std::sin(au);
    Float fusq = Sqr(fu);
    Float cu = std::copysign(1 / std::sqrt(Sqr(fu) + b0sq), fu);
    cu = Clamp(cu, -OneMinusEpsilon, OneMinusEpsilon); // avoid NaNs

    // 2. compute ’xu’
    Float xu = -(cu * z0) / SafeSqrt(1 - Sqr(cu));
    xu = Clamp(xu, x0, x1); // avoid Infs

    // 3. compute ’yv’
    Float dd = std::sqrt(Sqr(xu) + z0sq);
    Float h0 = y0 / std::sqrt(Sqr(dd) + y0sq);
    Float h1 = y1 / std::sqrt(Sqr(dd) + y1sq);
    Float hv = h0 + u[1] * (h1 - h0), hvsq = Sqr(hv);
    const Float eps = 1e-6;
    Float yv = (hvsq < 1 - eps) ? (hv * dd) / std::sqrt(1 - hvsq) : y1;

    // 4. transform (xu,yv,z0) to world coords
    return pRef + R.FromLocal(Vector3f(xu, yv, z0));
}

Point2f InvertSphericalQuadSample(const Point3f &pRef, const Point3f &s, const Vector3f &ex,
                                  const Vector3f &ey, const Point3f &pQuad) {
    // SphQuadInit()
    // local reference system ’R’
    Float exl = ex.Length(), eyl = ey.Length();
    Frame R = Frame::FromXY(ex / exl, ey / eyl);

    // compute rectangle coords in local reference system
    Vector3f d = s - pRef;
    Vector3f dLocal = R.ToLocal(d);
    Float z0 = dLocal.z;

    // flip ’z’ to make it point against ’Q’
    if (z0 > 0) {
        R.z = -R.z;
        z0 *= -1;
    }
    Float z0sq = Sqr(z0);
    Float x0 = dLocal.x;
    Float y0 = dLocal.y;
    Float x1 = x0 + exl;
    Float y1 = y0 + eyl;
    Float y0sq = Sqr(y0), y1sq = Sqr(y1);

    // create vectors to four vertices
    Vector3f v00(x0, y0, z0), v01(x0, y1, z0);
    Vector3f v10(x1, y0, z0), v11(x1, y1, z0);

    // compute normals to edges
    Vector3f n0 = Normalize(Cross(v00, v10));
    Vector3f n1 = Normalize(Cross(v10, v11));
    Vector3f n2 = Normalize(Cross(v11, v01));
    Vector3f n3 = Normalize(Cross(v01, v00));

    // compute internal angles (gamma_i)
    Float g0 = AngleBetween(-n0, n1);
    Float g1 = AngleBetween(-n1, n2);
    Float g2 = AngleBetween(-n2, n3);
    Float g3 = AngleBetween(-n3, n0);

    // compute predefined constants
    Float b0 = n0.z, b1 = n2.z, b0sq = Sqr(b0), b1sq = Sqr(b1);

    // compute solid angle from internal angles
    Float solidAngle = double(g0) + double(g1) + double(g2) + double(g3) - 2. * Pi;

    if (solidAngle < 1e-3) {
        Vector3f pq = pQuad - s;
        return Point2f(Dot(pq, ex) / ex.LengthSquared(),
                       Dot(pq, ey) / ey.LengthSquared());
    }

    Vector3f v = R.ToLocal(pQuad - pRef);
    Float xu = v.x, yv = v.y;

    xu = Clamp(xu, x0, x1); // avoid Infs
    if (xu == 0) xu = 1e-10;

    Float invcusq = 1 + z0sq / Sqr(xu);
    Float fusq = invcusq - b0sq;
    Float fu = std::copysign(std::sqrt(fusq), xu);

    Float sqrt = SafeSqrt(b0*b0 - b1*b1 + fusq);
    Float au = std::atan2(-(b1*fu) - std::copysign(b0 * sqrt, fu * b0),
                          b0*b1 - sqrt*std::abs(fu));
    if (au > 0) au -= 2 * Pi;

    if (fu == 0) au = Pi;
    Float u0 = (au + g2 + g3) / solidAngle;

    Float ddsq = Sqr(xu) + z0sq;
    Float dd = std::sqrt(ddsq);
    Float h0 = y0 / std::sqrt(ddsq + y0sq);
    Float h1 = y1 / std::sqrt(ddsq + y1sq);
    Float yvsq = Sqr(yv);

    Float u1[2] = {
                   (h0*h0 - h0*h1 -
                    std::abs(h0 - h1) * std::sqrt(yvsq * (ddsq + yvsq)) / (ddsq + yvsq)) /
                   Sqr(h0 - h1),
                   (h0*h0 - h0*h1 +
                    std::abs(h0 - h1) * std::sqrt(yvsq * (ddsq + yvsq)) / (ddsq + yvsq)) /
                   Sqr(h0 - h1)
    };

    Float hv[2] = { Lerp(u1[0], h0, h1), Lerp(u1[1], h0, h1) };
    Float hvsq[2] = { Sqr(hv[0]), Sqr(hv[1]) };
    Float yz[2] = { (hv[0] * dd) / std::sqrt(1 - hvsq[0]),
                    (hv[1] * dd) / std::sqrt(1 - hvsq[1]) };

    Point2f u = (std::abs(yz[0] - yv) < std::abs(yz[1] - yv)) ?
        Point2f(Clamp(u0, 0, 1), u1[0]) : Point2f(Clamp(u0, 0, 1), u1[1]);

    return u;
}

}  // namespace pbrt
