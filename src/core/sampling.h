
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_SAMPLING_H
#define PBRT_CORE_SAMPLING_H

// core/sampling.h*
#include "pbrt.h"
#include "geometry.h"
#include "rng.h"
#include <algorithm>
#include <array>

namespace pbrt {

// Sampling Declarations
void StratifiedSample1D(Float *samples, int nsamples, RNG &rng,
                        bool jitter = true);
void StratifiedSample2D(Point2f *samples, int nx, int ny, RNG &rng,
                        bool jitter = true);
void LatinHypercube(Float *samples, int nSamples, int nDim, RNG &rng);
struct Distribution1D {
    // Distribution1D Public Methods
    Distribution1D(const Float *f, int n) : func(f, f + n), cdf(n + 1) {
        // Compute integral of step function at $x_i$
        cdf[0] = 0;
        for (int i = 1; i < n + 1; ++i) cdf[i] = cdf[i - 1] + func[i - 1] / n;

        // Transform step function integral into CDF
        funcInt = cdf[n];
        if (funcInt == 0) {
            for (int i = 1; i < n + 1; ++i) cdf[i] = Float(i) / Float(n);
        } else {
            for (int i = 1; i < n + 1; ++i) cdf[i] /= funcInt;
        }
    }
    int Count() const { return (int)func.size(); }
    Float SampleContinuous(Float u, Float *pdf, int *off = nullptr) const {
        // Find surrounding CDF segments and _offset_
        int offset = FindInterval((int)cdf.size(),
                                  [&](int index) { return cdf[index] <= u; });
        if (off) *off = offset;
        // Compute offset along CDF segment
        Float du = u - cdf[offset];
        if ((cdf[offset + 1] - cdf[offset]) > 0) {
            CHECK_GT(cdf[offset + 1], cdf[offset]);
            du /= (cdf[offset + 1] - cdf[offset]);
        }
        DCHECK(!std::isnan(du));

        // Compute PDF for sampled offset
        if (pdf) *pdf = (funcInt > 0) ? func[offset] / funcInt : 0;

        // Return $x\in{}[0,1)$ corresponding to sample
        return (offset + du) / Count();
    }
    int SampleDiscrete(Float u, Float *pdf = nullptr,
                       Float *uRemapped = nullptr) const {
        // Find surrounding CDF segments and _offset_
        int offset = FindInterval((int)cdf.size(),
                                  [&](int index) { return cdf[index] <= u; });
        if (pdf) *pdf = (funcInt > 0) ? func[offset] / (funcInt * Count()) : 0;
        if (uRemapped)
            *uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
        if (uRemapped) CHECK(*uRemapped >= 0.f && *uRemapped <= 1.f);
        return offset;
    }
    Float DiscretePDF(int index) const {
        CHECK(index >= 0 && index < Count());
        return func[index] / (funcInt * Count());
    }

    // Distribution1D Public Data
    std::vector<Float> func, cdf;
    Float funcInt;
};

// Sample proportional to the function Lerp(x, a, b) over [0,1].
inline Float SampleLinear(Float u, Float a, Float b) {
    DCHECK(a >= 0 && b >= 0);
    if (a == b) return u;
    Float x = (a - std::sqrt(Lerp(u, a*a, b*b))) / (a - b);
    return std::min(x, OneMinusEpsilon);
}

// Returns the PDF for Lerp(x, a, b) over [0,1].
inline Float LinearPDF(Float x, Float a, Float b) {
    DCHECK(a >= 0 && b >= 0);
    if (x < 0 || x > 1) return 0;
    return Lerp(x, a, b) / ((a + b) / 2);
}

// Given a point x in [0,1], returns the sample u such that
// x=SampleLinear(u, a, b).
inline Float InvertLinearSample(Float x, Float a, Float b) {
    return x * (a * (2 - x) + b * x) / (a + b);
}

template <typename Float, typename C>
constexpr Float EvaluatePolynomial(Float t, C c) {
    return c;
}

template <typename Float, typename C, typename ...Args>
constexpr Float EvaluatePolynomial(Float t, C c, Args... cRemaining) {
    return std::fma(t, EvaluatePolynomial(t, cRemaining...), c);
}

// Sample the quadratic function a x^2 + b x + c == 0 over [0,1)
Float SampleQuadratic(Float u, Float a, Float b, Float c, Float *pdf = nullptr);
Float QuadraticPDF(Float x, Float a, Float b, Float c);
// Inverse of SampleQuadratic(), along the lines of InvertLinearSample.
inline Float InvertQuadraticSample(Float x, Float a, Float b, Float c) {
    // Just evaluate the CDF...
    Float norm = (a / 3 + b / 2 + c);
    return EvaluatePolynomial(x, 0, c / norm, b / (2 * norm), a / (3 * norm));
}

// Sample the Bezier curve specified by the three control points cp* over
// [0,1].
inline Float SampleBezierCurve(Float u, Float cp0, Float cp1, Float cp2,
                               Float *pdf) {
    // Convert from Bezier to power basis...
    return SampleQuadratic(u, cp0 - 2*cp1 + cp2, -2*cp0 + 2*cp1, cp0, pdf);
}

inline Float BezierCurvePDF(Float x, Float cp0, Float cp1, Float cp2) {
    return QuadraticPDF(x, cp0 - 2*cp1 + cp2, -2*cp0 + 2*cp1, cp0);
}

inline Float InvertBezierCurveSample(Float x, Float cp0, Float cp1, Float cp2) {
    return InvertQuadraticSample(x, cp0 - 2*cp1 + cp2, -2*cp0 + 2*cp1, cp0);
}

// v: (0,0), (1,0), (0,1), (1,1)
inline Point2f SampleBilinear(Point2f u, std::array<Float, 4> w) {
    Point2f p;
    // First sample in the v dimension. Compute the endpoints of the line
    // that's the average of the two lines at the edges at u=0 and u=1.
    Float v0 = w[0] + w[1], v1 = w[2] + w[3];
    // Sample along that line.
    p[1] = SampleLinear(u[1], v0, v1);
    // Now in sample in the u direction from the two line end points at the
    // sampled v position.
    p[0] = SampleLinear(u[0], Lerp(p[1], w[0], w[2]), Lerp(p[1], w[1], w[3]));
    return p;
}

// s.t. InvertBilinearSample(SampleBilinear(u, v), v) == u
inline Point2f InvertBilinearSample(Point2f p, std::array<Float, 4> v) {
    // This is just evaluating the CDF at x...
    auto InvertLinear = [](Float x, Float a, Float b) {
        x = Clamp(x, 0, 1);
        return x * (-a * (x - 2) + b * x) / (a + b);
    };
    return {InvertLinear(p[0], Lerp(p[1], v[0], v[2]), Lerp(p[1], v[1], v[3])),
            InvertLinear(p[1], v[0] + v[1], v[2] + v[3])};
}

inline Float Bilerp(std::array<Float, 2> p, std::array<Float, 4> v) {
    return ((1 - p[0]) * (1 - p[1]) * v[0] +
                 p[0]  * (1 - p[1]) * v[1] +
            (1 - p[0]) *      p[1]  * v[2] +
                 p[0]  *      p[1]  * v[3]);
}

inline Float BilinearPDF(Point2f p, std::array<Float, 4> w) {
    if (p.x < 0 || p.x > 1 || p.y < 0 || p.y > 1) return 0;
    if (w[0] + w[1] + w[2] + w[3] == 0) return 1;
    return 4 * Bilerp({p[0], p[1]}, w) / (w[0] + w[1] + w[2] + w[3]);
}

// w[u][v]
Point2f SampleBezier2D(Point2f u, std::array<std::array<Float, 3>, 3> w,
                       Float *pdf = nullptr);
Float Bezier2DPDF(Point2f p, std::array<std::array<Float, 3>, 3> w);
Point2f InvertBezier2DSample(Point2f p, std::array<std::array<Float, 3>, 3> w);

// Uniform sampling of a spherical triangle [Arvo 1995]
// Parameters:
// v: vertices of the triangle
// p: reference point
// u: uniform 2D random sampling in [0,1)^2
// pdf (out): pdf of the sampled point: 1 / the solid angle the triangle subtends from |p|.
std::array<Float, 3> SampleSphericalTriangle(const std::array<Point3f, 3> &v,
                                             const Point3f &p, const Point2f &u,
                                             Float *pdf = nullptr);

// Inverse of Arvo's algorithm: takes a point in the spherical triangle
// and returns the 2D sample u that maps to that point.
Point2f InvertSphericalTriangleSample(const std::array<Point3f, 3> &v,
                                      const Point3f &p, const Vector3f &w);
// Uniform sampling of a spherical quadrilteral. [Urena et al 2013]
// Parameters:
// pRef: reference point
// s: corner of the quad
// ex, ey: edges of the quad
// u: uniform 2D sample in [0,1)^2
// pdf (out): pdf of the sampled point, aka 1 / the solid angle the quad subtends.
Point3f SampleSphericalQuad(const Point3f &pRef, const Point3f &s, const Vector3f &ex,
                            const Vector3f &ey, const Point2f &u,
                            Float *pdf);
// Inverse of Urena et al's sampling algorithm: given a point on the spherical quad,
// returns the 2D sample u that maps to the point.
Point2f InvertSphericalQuadSample(const Point3f &pRef, const Point3f &s, const Vector3f &ex,
                                  const Vector3f &ey, const Point3f &pQuad);

Point2f RejectionSampleDisk(RNG &rng);
Vector3f UniformSampleHemisphere(const Point2f &u);
Float UniformHemispherePdf();
Vector3f UniformSampleSphere(const Point2f &u);
Float UniformSpherePdf();
Vector3f UniformSampleCone(const Point2f &u, Float thetamax);
Vector3f UniformSampleCone(const Point2f &u, Float thetamax, const Vector3f &x,
                           const Vector3f &y, const Vector3f &z);
Float UniformConePdf(Float thetamax);
Point2f UniformSampleDisk(const Point2f &u);
Point2f ConcentricSampleDisk(const Point2f &u);
Point2f UniformSampleTriangle(const Point2f &u);
class Distribution2D {
  public:
    // Distribution2D Public Methods
    Distribution2D(const Float *data, int nu, int nv);
    Point2f SampleContinuous(const Point2f &u, Float *pdf) const {
        Float pdfs[2];
        int v;
        Float d1 = pMarginal->SampleContinuous(u[1], &pdfs[1], &v);
        Float d0 = pConditionalV[v]->SampleContinuous(u[0], &pdfs[0]);
        *pdf = pdfs[0] * pdfs[1];
        return Point2f(d0, d1);
    }
    Float Pdf(const Point2f &p) const {
        int iu = Clamp(int(p[0] * pConditionalV[0]->Count()), 0,
                       pConditionalV[0]->Count() - 1);
        int iv =
            Clamp(int(p[1] * pMarginal->Count()), 0, pMarginal->Count() - 1);
        return pConditionalV[iv]->func[iu] / pMarginal->funcInt;
    }

  private:
    // Distribution2D Private Data
    std::vector<std::unique_ptr<Distribution1D>> pConditionalV;
    std::unique_ptr<Distribution1D> pMarginal;
};

// Sampling Inline Functions
template <typename T>
void Shuffle(T *samp, int count, int nDimensions, RNG &rng) {
    for (int i = 0; i < count; ++i) {
        int other = i + rng.UniformUInt32(count - i);
        for (int j = 0; j < nDimensions; ++j)
            std::swap(samp[nDimensions * i + j], samp[nDimensions * other + j]);
    }
}

inline Vector3f CosineSampleHemisphere(const Point2f &u) {
    Point2f d = ConcentricSampleDisk(u);
    Float z = std::sqrt(std::max((Float)0, 1 - d.x * d.x - d.y * d.y));
    return Vector3f(d.x, d.y, z);
}

inline Float CosineHemispherePdf(Float cosTheta) { return cosTheta * InvPi; }

inline Float BalanceHeuristic(int nf, Float fPdf, int ng, Float gPdf) {
    return (nf * fPdf) / (nf * fPdf + ng * gPdf);
}

inline Float PowerHeuristic(int nf, Float fPdf, int ng, Float gPdf) {
    Float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
}

}  // namespace pbrt

#endif  // PBRT_CORE_SAMPLING_H
