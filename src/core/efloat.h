
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
    EFloat(float v, float err = 0.f) : v(v), err(err) {
// Store high precision reference value in _EFloat_
#ifndef NDEBUG
        ld = v;
        Check();
#endif  // NDEBUG
    }
#ifndef NDEBUG
    EFloat(float v, long double ld, float err) : v(v), err(err), ld(ld) {
        Check();
    }
#endif  // DEBUG
    EFloat operator+(EFloat f) const {
        EFloat r;
        r.v = v + f.v;
#ifndef NDEBUG
        r.ld = ld + f.ld;
#endif  // DEBUG
        r.err =
            err + f.err + MachineEpsilon * (std::abs(v + f.v) + err + f.err);
        r.Check();
        return r;
    }
    explicit operator float() const { return v; }
    float GetAbsoluteError() const { return err; }
    float UpperBound() const { return NextFloatUp(v + err); }
    float LowerBound() const { return NextFloatDown(v - err); }
#ifndef NDEBUG
    float GetRelativeError() const { return std::abs((ld - v) / ld); }
    long double PreciseValue() const { return ld; }
#endif
    EFloat operator-(EFloat fe) const {
        EFloat r;
        r.v = v - fe.v;
#ifndef NDEBUG
        r.ld = ld - fe.ld;
#endif
        r.err =
            err + fe.err + MachineEpsilon * (std::abs(v - fe.v) + err + fe.err);
        Assert(r.err >= 0);
        r.Check();
        return r;
    }
    EFloat operator*(EFloat fe) const {
        EFloat r;
        r.v = v * fe.v;
#ifndef NDEBUG
        r.ld = ld * fe.ld;
#endif
        r.err = std::abs(fe.v * err) + std::abs(v * fe.err) + err * fe.err +
                MachineEpsilon * (std::abs(v * fe.v));
        Assert(r.err >= 0);
        r.Check();
        return r;
    }
    EFloat operator/(EFloat fe) const {
        EFloat r;
        r.v = v / fe.v;
#ifndef NDEBUG
        r.ld = ld / fe.ld;
#endif
        r.err =
            (std::abs(v) + err) / (std::abs(fe.v) - fe.err) -
            std::abs(v / fe.v) +
            MachineEpsilon * (std::abs(v) + err) / (std::abs(fe.v) - fe.err);
        Assert(r.err >= 0);
        r.Check();
        return r;
    }
    EFloat operator-() const {
#ifndef NDEBUG
        return EFloat(-v, -ld, err);
#else
        return EFloat(-v, err);
#endif
    }
    inline bool operator==(EFloat fe) const {
        return v == fe.v && err == fe.err;
    }
    inline void Check() const {
        Assert(err >= 0);
        if (!std::isinf(v) && !std::isnan(v))
            Assert(v - err <= ld && ld <= v + err);
    }
    EFloat(const EFloat &ef) {
        ef.Check();
        v = ef.v;
        err = ef.err;
#ifndef NDEBUG
        ld = ef.ld;
#endif
    }
    EFloat &operator=(const EFloat &ef) {
        ef.Check();
        if (&ef != this) {
            v = ef.v;
            err = ef.err;
#ifndef NDEBUG
            ld = ef.ld;
#endif
        }
        return *this;
    }

  private:
    // EFloat Private Data
    float v;
    float err;
#ifndef NDEBUG
    long double ld;
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
    r.ld = std::sqrt(fe.ld);
#endif
    // upper bound on error: sqrt(x \pm delta) ->
    // sqrt(x + delta) - sqrt(x - delta).
    // Greater than sqrt(x + delta) - sqrt(x)!
    r.err = std::sqrt(fe.v + fe.err) - std::sqrt(fe.v - fe.err) +
            MachineEpsilon * std::sqrt(fe.v + fe.err);
    return r;
}

inline EFloat abs(EFloat fe) {
#ifndef NDEBUG
    return EFloat(std::abs(fe.v), std::abs(fe.ld), fe.err);
#else
    return EFloat(std::abs(fe.v), fe.err);
#endif
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
