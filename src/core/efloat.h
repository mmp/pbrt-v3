
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_EFLOAT_H
#define PBRT_CORE_EFLOAT_H
#include "stdafx.h"

// core/efloat.h*
#include "pbrt.h"

// EFloat Declarations
class EFloat {
  public:
    // EFloat Public Methods
    EFloat() {}
    EFloat(float v, float err = 0.f) : v(v) {
        if (err == 0.)
            low = high = v;
        else {
            // Compute conservative bounds by rounding the endpoints away
            // from the middle. Note that this will be over-conservative in
            // cases where v-err or v+err are exactly representable in
            // floating-point, but it's probably not worth the trouble of
            // checking this case.
            low = NextFloatDown(v - err);
            high = NextFloatUp(v + err);
        }
// Store high precision reference value in _EFloat_
#ifndef NDEBUG
        vPrecise = v;
        Check();
#endif  // NDEBUG
    }
#ifndef NDEBUG
    EFloat(float v, long double lD, float err) : EFloat(v, err) {
        vPrecise = lD;
        Check();
    }
#endif  // DEBUG
    EFloat operator+(EFloat ef) const {
        EFloat r;
        r.v = v + ef.v;
#ifndef NDEBUG
        r.vPrecise = vPrecise + ef.vPrecise;
#endif  // DEBUG
        // Interval arithemetic addition, with the result rounded away from
        // the value r.v in order to be conservative.
        r.low = NextFloatDown(LowerBound() + ef.LowerBound());
        r.high = NextFloatUp(UpperBound() + ef.UpperBound());
        r.Check();
        return r;
    }
    explicit operator float() const { return v; }
    explicit operator double() const { return v; }
    float GetAbsoluteError() const { return high - low; }
    float UpperBound() const { return high; }
    float LowerBound() const { return low; }
#ifndef NDEBUG
    float GetRelativeError() const {
        return std::abs((vPrecise - v) / vPrecise);
    }
    long double PreciseValue() const { return vPrecise; }
#endif
    EFloat operator-(EFloat ef) const {
        EFloat r;
        r.v = v - ef.v;
#ifndef NDEBUG
        r.vPrecise = vPrecise - ef.vPrecise;
#endif
        r.low = NextFloatDown(LowerBound() - ef.UpperBound());
        r.high = NextFloatUp(UpperBound() - ef.LowerBound());
        r.Check();
        return r;
    }
    EFloat operator*(EFloat ef) const {
        EFloat r;
        r.v = v * ef.v;
#ifndef NDEBUG
        r.vPrecise = vPrecise * ef.vPrecise;
#endif
        Float prod[4] = {
            LowerBound() * ef.LowerBound(), UpperBound() * ef.LowerBound(),
            LowerBound() * ef.UpperBound(), UpperBound() * ef.UpperBound()};
        r.low = NextFloatDown(
            std::min(std::min(prod[0], prod[1]), std::min(prod[2], prod[3])));
        r.high = NextFloatUp(
            std::max(std::max(prod[0], prod[1]), std::max(prod[2], prod[3])));
        r.Check();
        return r;
    }
    EFloat operator/(EFloat ef) const {
        EFloat r;
        r.v = v / ef.v;
#ifndef NDEBUG
        r.vPrecise = vPrecise / ef.vPrecise;
#endif
        if (ef.low < 0 && ef.high > 0) {
            // Bah. The interval we're dividing by straddles zero, so just
            // return an interval of everything.
            r.low = -Infinity;
            r.high = Infinity;
        } else {
            Float div[4] = {
                LowerBound() / ef.LowerBound(), UpperBound() / ef.LowerBound(),
                LowerBound() / ef.UpperBound(), UpperBound() / ef.UpperBound()};
            r.low = NextFloatDown(
                std::min(std::min(div[0], div[1]), std::min(div[2], div[3])));
            r.high = NextFloatUp(
                std::max(std::max(div[0], div[1]), std::max(div[2], div[3])));
        }
        r.Check();
        return r;
    }
    EFloat operator-() const {
        EFloat r;
        r.v = -v;
#ifndef NDEBUG
        r.vPrecise = -vPrecise;
#endif
        r.low = -high;
        r.high = -low;
        r.Check();
        return r;
    }
    inline bool operator==(EFloat fe) const { return v == fe.v; }
    inline void Check() const {
        if (!std::isinf(low) && !std::isnan(low) && !std::isinf(high) &&
            !std::isnan(high))
            Assert(low <= high);
#ifndef NDEBUG
        if (!std::isinf(v) && !std::isnan(v))
            Assert(LowerBound() <= vPrecise && vPrecise <= UpperBound());
#endif
    }
    EFloat(const EFloat &ef) {
        ef.Check();
        v = ef.v;
        low = ef.low;
        high = ef.high;
#ifndef NDEBUG
        vPrecise = ef.vPrecise;
#endif
    }
    EFloat &operator=(const EFloat &ef) {
        ef.Check();
        if (&ef != this) {
            v = ef.v;
            low = ef.low;
            high = ef.high;
#ifndef NDEBUG
            vPrecise = ef.vPrecise;
#endif
        }
        return *this;
    }

  private:
    // EFloat Private Data
    float v, low, high;
#ifndef NDEBUG
    long double vPrecise;
#endif  // NDEBUG
    friend inline EFloat sqrt(EFloat fe);
    friend inline EFloat abs(EFloat fe);
    friend inline bool Quadratic(EFloat A, EFloat B, EFloat C, EFloat *t0,
                                 EFloat *t1);
};

// EFloat Inline Functions
inline EFloat operator*(float f, EFloat fe) { return EFloat(f) * fe; }

inline EFloat operator/(float f, EFloat fe) { return EFloat(f) / fe; }

inline EFloat operator+(float f, EFloat fe) { return EFloat(f) + fe; }

inline EFloat operator-(float f, EFloat fe) { return EFloat(f) - fe; }

inline EFloat sqrt(EFloat fe) {
    EFloat r;
    r.v = std::sqrt(fe.v);
#ifndef NDEBUG
    r.vPrecise = std::sqrt(fe.vPrecise);
#endif
    r.low = NextFloatDown(std::sqrt(fe.low));
    r.high = NextFloatUp(std::sqrt(fe.high));
    r.Check();
    return r;
}

inline EFloat abs(EFloat fe) {
    if (fe.low >= 0)
        // The entire interval is greater than zero, so we're all set.
        return fe;
    else if (fe.high <= 0) {
        // The entire interval is less than zero.
        EFloat r;
        r.v = -fe.v;
#ifndef NDEBUG
        r.vPrecise = -fe.vPrecise;
#endif
        r.low = -fe.high;
        r.high = -fe.low;
        r.Check();
        return r;
    } else {
        // The interval straddles zero.
        EFloat r;
        r.v = std::abs(fe.v);
#ifndef NDEBUG
        r.vPrecise = std::abs(fe.vPrecise);
#endif
        r.low = 0;
        r.high = std::max(-fe.low, fe.high);
        r.Check();
        return r;
    }
}

inline bool Quadratic(EFloat A, EFloat B, EFloat C, EFloat *t0, EFloat *t1);
inline bool Quadratic(EFloat A, EFloat B, EFloat C, EFloat *t0, EFloat *t1) {
    // Find quadratic discriminant
    double discrim = (double)B.v * (double)B.v - 4. * (double)A.v * (double)C.v;
    if (discrim < 0.) return false;
    double rootDiscrim = std::sqrt(discrim);

    EFloat floatRootDiscrim(rootDiscrim, MachineEpsilon * rootDiscrim);

    // Compute quadratic _t_ values
    EFloat q;
    if ((float)B < 0)
        q = -.5 * (B - floatRootDiscrim);
    else
        q = -.5 * (B + floatRootDiscrim);
    *t0 = q / A;
    *t1 = C / q;
    if ((float)*t0 > (float)*t1) std::swap(*t0, *t1);
    return true;
}

#endif  // PBRT_CORE_EFLOAT_H
