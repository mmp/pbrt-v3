
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

// core/interpolation.cpp*
#include "interpolation.h"

// Spline Interpolation Definitions
Float CatmullRom(int size, const Float *nodes, const Float *values, Float x) {
    if (!(x >= nodes[0] && x <= nodes[size - 1])) return 0;
    int idx = FindInterval(size, [&](int i) { return nodes[i] <= x; });
    Float x0 = nodes[idx], x1 = nodes[idx + 1];
    Float f0 = values[idx], f1 = values[idx + 1];
    Float width = x1 - x0;
    Float d0, d1;
    if (idx > 0)
        d0 = width * (f1 - values[idx - 1]) / (x1 - nodes[idx - 1]);
    else
        d0 = f1 - f0;

    if (idx + 2 < size)
        d1 = width * (values[idx + 2] - f0) / (nodes[idx + 2] - x0);
    else
        d1 = f1 - f0;

    Float t = (x - x0) / (x1 - x0), t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * f0 + (-2 * t3 + 3 * t2) * f1 +
           (t3 - 2 * t2 + t) * d0 + (t3 - t2) * d1;
}

bool CatmullRomWeights(int size, const Float *nodes, Float x, int *offset,
                       Float *weights) {
    // Return _false_ if _x_ is out of bounds
    if (!(x >= nodes[0] && x <= nodes[size - 1])) return false;

    // Search for the interval _idx_ containing _x_
    int idx = FindInterval(size, [&](int i) { return nodes[i] <= x; });
    *offset = idx - 1;
    Float x0 = nodes[idx], x1 = nodes[idx + 1];

    // Compute the $t$ parameter and powers
    Float t = (x - x0) / (x1 - x0), t2 = t * t, t3 = t2 * t;

    // Compute initial node weights $w_1$ and $w_2$
    weights[1] = 2 * t3 - 3 * t2 + 1;
    weights[2] = -2 * t3 + 3 * t2;

    // Compute first node weight $w_0$
    if (idx > 0) {
        Float w0 = (t3 - 2 * t2 + t) * (x1 - x0) / (x1 - nodes[idx - 1]);
        weights[0] = -w0;
        weights[2] += w0;
    } else {
        Float w0 = t3 - 2 * t2 + t;
        weights[0] = 0;
        weights[1] -= w0;
        weights[2] += w0;
    }

    // Compute last node weight $w_3$
    if (idx + 2 < size) {
        Float w3 = (t3 - t2) * (x1 - x0) / (nodes[idx + 2] - x0);
        weights[1] -= w3;
        weights[3] = w3;
    } else {
        Float w3 = t3 - t2;
        weights[1] -= w3;
        weights[2] += w3;
        weights[3] = 0;
    }
    return true;
}

Float SampleCatmullRom(int n, const Float *x, const Float *values,
                       const Float *F, Float u, Float *fval, Float *pdf) {
    // Map _u_ to a spline interval by inverting _F_
    Float maximum = F[n - 1];
    u *= maximum;
    int i = FindInterval(n, [&](int i) { return F[i] <= u; });

    // Look up $x_i$ and function values of spline segment _i_
    Float x0 = x[i], x1 = x[i + 1];
    Float f0 = values[i], f1 = values[i + 1];
    Float width = x1 - x0;

    // Approximate derivatives using finite differences
    Float d0, d1;
    if (i > 0)
        d0 = width * (f1 - values[i - 1]) / (x1 - x[i - 1]);
    else
        d0 = f1 - f0;
    if (i + 2 < n)
        d1 = width * (values[i + 2] - f0) / (x[i + 2] - x0);
    else
        d1 = f1 - f0;

    // Re-scale _u_ for continous spline sampling step
    u = (u - F[i]) / width;

    // Invert definite integral over spline segment and return solution

    // Set initial guess for $t$ by importance sampling a linear interpolant
    Float t;
    if (f0 != f1)
        t = (f0 - std::sqrt(std::max((Float)0, f0 * f0 + 2 * u * (f1 - f0)))) /
            (f0 - f1);
    else
        t = u / f0;
    Float a = 0, b = 1, Fhat, fhat;
    while (true) {
        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);

        // Evaluate target function and its derivative in Horner form
        Fhat = t * (f0 +
                    t * (.5f * d0 +
                         t * ((1.f / 3.f) * (-2 * d0 - d1) + f1 - f0 +
                              t * (.25f * (d0 + d1) + .5f * (f0 - f1)))));
        fhat = f0 +
               t * (d0 +
                    t * (-2 * d0 - d1 + 3 * (f1 - f0) +
                         t * (d0 + d1 + 2 * (f0 - f1))));

        // Stop the iteration if converged
        if (std::abs(Fhat - u) < 1e-6f || b - a < 1e-6f) break;

        // Update bisection bounds using updated _t_
        if (Fhat - u < 0)
            a = t;
        else
            b = t;

        // Perform a Newton step
        t -= (Fhat - u) / fhat;
    }

    // Return the sample position and function value
    if (fval) *fval = fhat;
    if (pdf) *pdf = fhat / maximum;
    return x0 + width * t;
}

Float SampleCatmullRom2D(int size1, int size2, const Float *nodes1,
                         const Float *nodes2, const Float *values,
                         const Float *cdf, Float alpha, Float u, Float *fval,
                         Float *pdf) {
    // Determine offset and coefficients for the _alpha_ parameter
    int offset;
    Float weights[4];
    if (!CatmullRomWeights(size1, nodes1, alpha, &offset, weights)) return 0;

    // Define a lambda function to interpolate table entries
    auto interpolate = [&](const Float *array, int idx) {
        Float value = 0;
        for (int i = 0; i < 4; ++i)
            if (weights[i] != 0)
                value += array[(offset + i) * size2 + idx] * weights[i];
        return value;
    };

    // Map _u_ to a spline interval by inverting the interpolated _cdf_
    Float maximum = interpolate(cdf, size2 - 1);
    u *= maximum;
    int idx =
        FindInterval(size2, [&](int i) { return interpolate(cdf, i) <= u; });

    // Look up node positions and interpolated function values
    Float f0 = interpolate(values, idx), f1 = interpolate(values, idx + 1);
    Float x0 = nodes2[idx], x1 = nodes2[idx + 1];
    Float width = x1 - x0;
    Float d0, d1;

    // Re-scale _u_ using the interpolated _cdf_
    u = (u - interpolate(cdf, idx)) / width;

    // Approximate derivatives using finite differences of the interpolant
    if (idx > 0)
        d0 = width * (f1 - interpolate(values, idx - 1)) /
             (x1 - nodes2[idx - 1]);
    else
        d0 = f1 - f0;
    if (idx + 2 < size2)
        d1 = width * (interpolate(values, idx + 2) - f0) /
             (nodes2[idx + 2] - x0);
    else
        d1 = f1 - f0;

    // Invert definite integral over spline segment and return solution

    // Set initial guess for $t$ by importance sampling a linear interpolant
    Float t;
    if (f0 != f1)
        t = (f0 - std::sqrt(std::max((Float)0, f0 * f0 + 2 * u * (f1 - f0)))) /
            (f0 - f1);
    else
        t = u / f0;
    Float a = 0, b = 1, Fhat, fhat;
    while (true) {
        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);

        // Evaluate target function and its derivative in Horner form
        Fhat = t * (f0 +
                    t * (.5f * d0 +
                         t * ((1.f / 3.f) * (-2 * d0 - d1) + f1 - f0 +
                              t * (.25f * (d0 + d1) + .5f * (f0 - f1)))));
        fhat = f0 +
               t * (d0 +
                    t * (-2 * d0 - d1 + 3 * (f1 - f0) +
                         t * (d0 + d1 + 2 * (f0 - f1))));

        // Stop the iteration if converged
        if (std::abs(Fhat - u) < 1e-6f || b - a < 1e-6f) break;

        // Update bisection bounds using updated _t_
        if (Fhat - u < 0)
            a = t;
        else
            b = t;

        // Perform a Newton step
        t -= (Fhat - u) / fhat;
    }

    // Return the sample position and function value
    if (fval) *fval = fhat;
    if (pdf) *pdf = fhat / maximum;
    return x0 + width * t;
}

Float IntegrateCatmullRom(int n, const Float *x, const Float *values,
                          Float *cdf) {
    Float sum = 0;
    cdf[0] = 0;
    for (int i = 0; i < n - 1; ++i) {
        // Look up $x_i$ and function values of spline segment _i_
        Float x0 = x[i], x1 = x[i + 1];
        Float f0 = values[i], f1 = values[i + 1];
        Float width = x1 - x0;

        // Approximate derivatives using finite differences
        Float d0, d1;
        if (i > 0)
            d0 = width * (f1 - values[i - 1]) / (x1 - x[i - 1]);
        else
            d0 = f1 - f0;
        if (i + 2 < n)
            d1 = width * (values[i + 2] - f0) / (x[i + 2] - x0);
        else
            d1 = f1 - f0;

        // Keep a running sum and build a cumulative distribution function
        sum += ((d0 - d1) * (1.f / 12.f) + (f0 + f1) * .5f) * width;
        cdf[i + 1] = sum;
    }
    return sum;
}

Float InvertCatmullRom(int n, const Float *x, const Float *values, Float u) {
    // Stop when _u_ is out of bounds
    if (!(u > values[0]))
        return x[0];
    else if (!(u < values[n - 1]))
        return x[n - 1];

    // Map _u_ to a spline interval by inverting _values_
    int i = FindInterval(n, [&](int i) { return values[i] <= u; });

    // Look up $x_i$ and function values of spline segment _i_
    Float x0 = x[i], x1 = x[i + 1];
    Float f0 = values[i], f1 = values[i + 1];
    Float width = x1 - x0;

    // Approximate derivatives using finite differences
    Float d0, d1;
    if (i > 0)
        d0 = width * (f1 - values[i - 1]) / (x1 - x[i - 1]);
    else
        d0 = f1 - f0;
    if (i + 2 < n)
        d1 = width * (values[i + 2] - f0) / (x[i + 2] - x0);
    else
        d1 = f1 - f0;

    // Invert the spline interpolant using Newton-Bisection
    Float a = 0, b = 1, t = .5f;
    Float Fhat, fhat;
    while (true) {
        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);

        // Compute powers of _t_
        Float t2 = t * t, t3 = t2 * t;

        // Set _Fhat_ using Equation (\ref{eq:cubicspline-as-basisfunctions})
        Fhat = (2 * t3 - 3 * t2 + 1) * f0 + (-2 * t3 + 3 * t2) * f1 +
               (t3 - 2 * t2 + t) * d0 + (t3 - t2) * d1;

        // Set _fhat_ using Equation (\ref{eq:cubicspline-derivative})
        fhat = (6 * t2 - 6 * t) * f0 + (-6 * t2 + 6 * t) * f1 +
               (3 * t2 - 4 * t + 1) * d0 + (3 * t2 - 2 * t) * d1;

        // Stop the iteration if converged
        if (std::abs(Fhat - u) < 1e-6f || b - a < 1e-6f) break;

        // Update bisection bounds using updated _t_
        if (Fhat - u < 0)
            a = t;
        else
            b = t;

        // Perform a Newton step
        t -= (Fhat - u) / fhat;
    }
    return x0 + t * width;
}

// Fourier Interpolation Definitions
Float Fourier(const Float *a, int m, double cosPhi) {
    double value = 0.0;
    // Initialize cosine iterates
    double cosKMinusOnePhi = cosPhi;
    double cosKPhi = 1;
    for (int k = 0; k < m; ++k) {
        // Add the current summand and update the cosine iterates
        value += a[k] * cosKPhi;
        double cosKPlusOnePhi = 2 * cosPhi * cosKPhi - cosKMinusOnePhi;
        cosKMinusOnePhi = cosKPhi;
        cosKPhi = cosKPlusOnePhi;
    }
    return value;
}

Float SampleFourier(const Float *ak, const Float *recip, int m, Float u,
                    Float *pdf, Float *phiPtr) {
    // Pick a side and declare bisection variables
    bool flip = (u >= 0.5);
    if (flip)
        u = 1 - 2 * (u - .5f);
    else
        u *= 2;
    double a = 0, b = Pi, phi = 0.5 * Pi;
    double F, f;
    while (true) {
        // Evaluate $F(\phi)$ and its derivative $f(\phi)$

        // Initialize sine and cosine iterates
        double cosPhi = std::cos(phi);
        double sinPhi = std::sqrt(1 - cosPhi * cosPhi);
        double cosPhiPrev = cosPhi, cosPhiCur = 1;
        double sinPhiPrev = -sinPhi, sinPhiCur = 0;

        // Initialize _F_ and _f_ with the first series term
        F = ak[0] * phi;
        f = ak[0];
        for (int j = 1; j < m; ++j) {
            // Compute next sine and cosine iterates
            double sinPhiNext = 2 * cosPhi * sinPhiCur - sinPhiPrev;
            double cosPhiNext = 2 * cosPhi * cosPhiCur - cosPhiPrev;
            sinPhiPrev = sinPhiCur;
            sinPhiCur = sinPhiNext;
            cosPhiPrev = cosPhiCur;
            cosPhiCur = cosPhiNext;

            // Add the next series term to _F_ and _f_
            F += ak[j] * recip[j] * sinPhiNext;
            f += ak[j] * cosPhiNext;
        }
        F -= u * ak[0] * Pi;

        // Update bisection bounds using updated $\phi$
        if (F > 0)
            b = phi;
        else
            a = phi;

        // Stop the Fourier bisection iteration if converged
        if (std::abs(F) < 1e-6f || b - a < 1e-6f) break;

        // Perform a Newton step given $f(\phi)$ and $F(\phi)$
        phi -= F / f;

        // Fall back to a bisection step when $\phi$ is out of bounds
        if (!(phi >= a && phi <= b)) phi = 0.5f * (a + b);
    }
    // Potentially flip $\phi$ and return the result
    if (flip) phi = 2 * Pi - phi;
    *pdf = (Float)(Inv2Pi * f / ak[0]);
    *phiPtr = (Float)phi;
    return f;
}
