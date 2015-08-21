
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

Float SampleCatmullRom(int size, const Float *nodes, const Float *values,
                       const Float *cdf, Float sample, Float *fval,
                       Float *pdf) {
    // Map _sample_ to a spline interval by inverting _cdf_
    Float maximum = cdf[size - 1];
    sample *= maximum;
    int idx = FindInterval(size, [&](int i) { return cdf[i] <= sample; });

    // Look up node positions and function values
    Float x0 = nodes[idx], x1 = nodes[idx + 1], f0 = values[idx],
          f1 = values[idx + 1], width = x1 - x0;

    // Approximate derivatives using finite differences
    Float d0, d1;
    if (idx > 0)
        d0 = width * (f1 - values[idx - 1]) / (x1 - nodes[idx - 1]);
    else
        d0 = f1 - f0;
    if (idx + 2 < size)
        d1 = width * (values[idx + 2] - f0) / (nodes[idx + 2] - x0);
    else
        d1 = f1 - f0;

    // Re-scale _sample_
    sample = (sample - cdf[idx]) / width;

    // Invert definite integral over spline segment and return solution

    // Set initial guess for $t$ by importance sampling a linear interpolant
    Float t;
    if (f0 != f1)
        t = (f0 - std::sqrt(
                      std::max((Float).0f, f0 * f0 + 2 * sample * (f1 - f0)))) /
            (f0 - f1);
    else
        t = sample / f0;
    Float a = 0.f, b = 1.f, value, deriv;
    while (true) {
        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);

        // Evaluate target function and its derivative in Horner form
        value = t * (f0 +
                     t * (.5f * d0 +
                          t * ((1.f / 3.f) * (-2.f * d0 - d1) + f1 - f0 +
                               t * (.25f * (d0 + d1) + .5f * (f0 - f1))))) -
                sample;
        deriv = f0 +
                t * (d0 +
                     t * (-2.f * d0 - d1 + 3.f * (f1 - f0) +
                          t * (d0 + d1 + 2.f * (f0 - f1))));

        // Stop the iteration if converged
        if (std::abs(value) < 1e-6f || b - a < 1e-6f) break;

        // Update bisection bounds
        if (value > 0)
            b = t;
        else
            a = t;

        // Perform a Newton step
        t -= value / deriv;
    }

    // Return the sample position and function value
    if (fval) *fval = deriv;
    if (pdf) *pdf = deriv / maximum;
    return x0 + width * t;
}

Float SampleCatmullRom2D(int size1, int size2, const Float *nodes1,
                         const Float *nodes2, const Float *values,
                         const Float *cdf, Float alpha, Float sample,
                         Float *fval, Float *pdf) {
    // Determine offset and coefficients for the _alpha_ parameter
    int offset;
    Float weights[4];
    if (!CatmullRomWeights(size1, nodes1, alpha, &offset, weights)) return 0.f;

    // Define a lambda function to interpolate table entries
    auto interpolate = [&](const Float *array, int idx) {
        Float value = 0.0f;
        for (int i = 0; i < 4; ++i)
            if (weights[i] != 0.f)
                value += array[(offset + i) * size2 + idx] * weights[i];
        return value;
    };

    // Map _sample_ to a spline interval by inverting the interpolated _cdf_
    Float maximum = interpolate(cdf, size2 - 1);
    sample *= maximum;
    int idx = FindInterval(
        size2, [&](int i) { return interpolate(cdf, i) <= sample; });

    // Look up node positions and interpolated function values
    Float f0 = interpolate(values, idx), f1 = interpolate(values, idx + 1),
          x0 = nodes2[idx], x1 = nodes2[idx + 1], width = x1 - x0, d0, d1;

    // Re-scale _sample_ using the interpolated _cdf_
    sample = (sample - interpolate(cdf, idx)) / width;

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
        t = (f0 - std::sqrt(
                      std::max((Float).0f, f0 * f0 + 2 * sample * (f1 - f0)))) /
            (f0 - f1);
    else
        t = sample / f0;
    Float a = 0.f, b = 1.f, value, deriv;
    while (true) {
        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);

        // Evaluate target function and its derivative in Horner form
        value = t * (f0 +
                     t * (.5f * d0 +
                          t * ((1.f / 3.f) * (-2.f * d0 - d1) + f1 - f0 +
                               t * (.25f * (d0 + d1) + .5f * (f0 - f1))))) -
                sample;
        deriv = f0 +
                t * (d0 +
                     t * (-2.f * d0 - d1 + 3.f * (f1 - f0) +
                          t * (d0 + d1 + 2.f * (f0 - f1))));

        // Stop the iteration if converged
        if (std::abs(value) < 1e-6f || b - a < 1e-6f) break;

        // Update bisection bounds
        if (value > 0)
            b = t;
        else
            a = t;

        // Perform a Newton step
        t -= value / deriv;
    }

    // Return the sample position and function value
    if (fval) *fval = deriv;
    if (pdf) *pdf = deriv / maximum;
    return x0 + width * t;
}

Float IntegrateCatmullRom(int size, const Float *nodes, const Float *values,
                          Float *cdf) {
    Float sum = 0.f;
    cdf[0] = 0.f;
    for (int idx = 0; idx < size - 1; ++idx) {
        // Look up node positions and function values
        Float x0 = nodes[idx], x1 = nodes[idx + 1], f0 = values[idx],
              f1 = values[idx + 1], width = x1 - x0;

        // Approximate derivatives using finite differences
        Float d0, d1;
        if (idx > 0)
            d0 = width * (f1 - values[idx - 1]) / (x1 - nodes[idx - 1]);
        else
            d0 = f1 - f0;
        if (idx + 2 < size)
            d1 = width * (values[idx + 2] - f0) / (nodes[idx + 2] - x0);
        else
            d1 = f1 - f0;

        // Keep a running sum and build a cumulative distribution function
        sum += ((d0 - d1) * (1.f / 12.f) + (f0 + f1) * .5f) * width;
        cdf[idx + 1] = sum;
    }
    return sum;
}

Float InvertCatmullRom(int size, const Float *nodes, const Float *values,
                       Float y) {
    // Stop when _y_ is out of bounds
    if (!(y > values[0]))
        return nodes[0];
    else if (!(y < values[size - 1]))
        return nodes[size - 1];

    // Map _y_ to a spline interval by inverting _values_
    int idx = FindInterval(size, [&](int i) { return values[i] <= y; });

    // Look up node positions and function values
    Float x0 = nodes[idx], x1 = nodes[idx + 1], f0 = values[idx],
          f1 = values[idx + 1], width = x1 - x0;

    // Approximate derivatives using finite differences
    Float d0, d1;
    if (idx > 0)
        d0 = width * (f1 - values[idx - 1]) / (x1 - nodes[idx - 1]);
    else
        d0 = f1 - f0;
    if (idx + 2 < size)
        d1 = width * (values[idx + 2] - f0) / (nodes[idx + 2] - x0);
    else
        d1 = f1 - f0;

    // Invert the spline interpolant using Newton-Bisection
    Float a = 0.f, b = 1.f, t = .5f;
    Float value, deriv;
    while (true) {
        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);

        // Compute powers of _t_
        Float t2 = t * t, t3 = t2 * t;

        // Set _value_ using Equation (\ref{eq:cubicspline-as-basisfunctions})
        value = (2 * t3 - 3 * t2 + 1) * f0 + (-2 * t3 + 3 * t2) * f1 +
                (t3 - 2 * t2 + t) * d0 + (t3 - t2) * d1;

        // Set _deriv_ using Equation (\ref{eq:cubicspline-derivative})
        deriv = (6 * t2 - 6 * t) * f0 + (-6 * t2 + 6 * t) * f1 +
                (3 * t2 - 4 * t + 1) * d0 + (3 * t2 - 2 * t) * d1;
        value -= y;
        // Stop the iteration if converged
        if (std::abs(value) < 1e-6f || b - a < 1e-6f) break;

        // Update bisection bounds
        if (value > 0)
            b = t;
        else
            a = t;

        // Perform a Newton step
        t -= value / deriv;
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

Float SampleFourier(const Float *coeffs, const Float *recip, int nCoeffs,
                    Float sample, Float *pdf, Float *phi) {
    // Pick a side and declare bisection variables
    bool flip = false;
    if (sample < .5f)
        sample *= 2.f;
    else {
        sample = 1.f - 2.f * (sample - .5f);
        flip = true;
    }
    double a = 0.0, b = Pi, t = 0.5 * Pi;
    double value, deriv;
    while (true) {
        // Evaluate $F(t)$ and its derivative

        // Initialize sine and cosine iterates
        double cosT = std::cos(t), sinT = std::sqrt(1 - cosT * cosT),
               cosTPrev = cosT, cosTCur = 1.0, sinTPrev = -sinT, sinTCur = 0.0;

        // Initialize _mono_ and _deriv_ with the first series term
        value = coeffs[0] * t;
        deriv = coeffs[0];
        for (int j = 1; j < nCoeffs; ++j) {
            // Compute next sine and cosine iterates
            double sinT_next = 2 * cosT * sinTCur - sinTPrev,
                   cosT_next = 2 * cosT * cosTCur - cosTPrev;
            sinTPrev = sinTCur;
            sinTCur = sinT_next;
            cosTPrev = cosTCur;
            cosTCur = cosT_next;

            // Add the next series term to _value_ and _deriv_
            double coeff = coeffs[j];
            value += coeff * recip[j] * sinT_next;
            deriv += coeff * cosT_next;
        }
        value -= coeffs[0] * Pi * sample;

        // Update bisection bounds
        if (value > 0)
            b = t;
        else
            a = t;

        // Stop the iteration if converged
        if (std::abs(value) < 1e-6f || b - a < 1e-6f) break;

        // Perform a Newton step
        t -= value / deriv;

        // Fall back to a bisection step when _t_ is out of bounds
        if (!(t >= a && t <= b)) t = 0.5f * (a + b);
    }
    // Potentially flip _t_ and return the result
    if (flip) t = 2 * Pi - t;
    *pdf = (Float)(Inv2Pi * deriv / coeffs[0]);
    *phi = (Float)t;
    return deriv;
}
