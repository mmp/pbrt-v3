
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


#ifndef PBRT_CORE_HALF_H
#define PBRT_CORE_HALF_H

#include "pbrt.h"
#include <stdint.h>

namespace pbrt {

// TODO: define a small Half struct/class?

#ifdef PBRT_HAVE_BINARY_CONSTANTS
static const int kHalfExponentMask = 0b0111110000000000;
static const int kHalfSignificandMask = 0b1111111111;
static const int kHalfNegativeZero = 0b1000000000000000;
static const int kHalfPositiveZero = 0;
// Exponent all 1s, significand zero
static const int kHalfNegativeInfinity = 0b1111110000000000;
static const int kHalfPositiveInfinity = 0b0111110000000000;
#else
static const int kHalfExponentMask = 0x7c00;
static const int kHalfSignificandMask = 0x3ff;
static const int kHalfNegativeZero = 0x8000;
static const int kHalfPositiveZero = 0;
// Exponent all 1s, significand zero
static const int kHalfNegativeInfinity = 0xfc00;
static const int kHalfPositiveInfinity = 0x7c00;
#endif

inline bool HalfIsInf(uint16_t h) {
    return h == kHalfPositiveInfinity || h == kHalfNegativeInfinity;
}

// -1, 1
inline int HalfSign(uint16_t h) { return (h >> 15) ? -1 : 1; }

inline bool HalfIsNaN(uint16_t h) {
    return ((h & kHalfExponentMask) == kHalfExponentMask &&
            (h & kHalfSignificandMask) != 0);
}

inline uint16_t NextHalfUp(uint16_t v) {
    if (HalfIsInf(v) && HalfSign(v) == 1) return v;
    if (v == kHalfNegativeZero) v = kHalfPositiveZero;

    // Advance _v_ to next higher float
    if (HalfSign(v) >= 0)
        ++v;
    else
        --v;
    return v;
}

inline uint16_t NextHalfDown(uint16_t v) {
    if (HalfIsInf(v) && HalfSign(v) == -1) return v;
    if (v == kHalfPositiveZero) v = kHalfNegativeZero;
    if (v > 0)
        --v;
    else
        ++v;
    return v;
}

// TODO: these should live in maybe a new core/fp.h file (that also
// picks up a bunch of other stuff...)
// TODO: support for non-AVX systems, check CPUID stuff, etc..

// https://gist.github.com/rygorous/2156668
union FP32 {
    uint32_t u;
    float f;
    struct {
        unsigned int Mantissa : 23;
        unsigned int Exponent : 8;
        unsigned int Sign : 1;
    };
};

union FP16 {
    uint16_t u;
    struct {
        unsigned int Mantissa : 10;
        unsigned int Exponent : 5;
        unsigned int Sign : 1;
    };
};

// Same, but rounding ties to nearest even instead of towards +inf
inline uint16_t FloatToHalf(float ff) {
    FP32 f;
    f.f = ff;
    FP32 f32infty = {255 << 23};
    FP32 f16max = {(127 + 16) << 23};
    FP32 denorm_magic = {((127 - 15) + (23 - 10) + 1) << 23};
    unsigned int sign_mask = 0x80000000u;
    FP16 o = {0};

    unsigned int sign = f.u & sign_mask;
    f.u ^= sign;

    // NOTE all the integer compares in this function can be safely
    // compiled into signed compares since all operands are below
    // 0x80000000. Important if you want fast straight SSE2 code
    // (since there's no unsigned PCMPGTD).

    if (f.u >= f16max.u)  // result is Inf or NaN (all exponent bits set)
        o.u = (f.u > f32infty.u) ? 0x7e00 : 0x7c00;  // NaN->qNaN and Inf->Inf
    else  // (De)normalized number or zero
    {
        if (f.u < (113 << 23))  // resulting FP16 is subnormal or zero
        {
            // use a magic value to align our 10 mantissa bits at the bottom of
            // the float. as long as FP addition is round-to-nearest-even this
            // just works.
            f.f += denorm_magic.f;

            // and one integer subtract of the bias later, we have our final
            // float!
            o.u = f.u - denorm_magic.u;
        } else {
            unsigned int mant_odd = (f.u >> 13) & 1;  // resulting mantissa is odd

            // update exponent, rounding bias part 1
            f.u += (uint32_t(15 - 127) << 23) + 0xfff;
            // rounding bias part 2
            f.u += mant_odd;
            // take the bits!
            o.u = f.u >> 13;
        }
    }

    o.u |= sign >> 16;
    return o.u;
}

inline float HalfToFloat(uint16_t hh) {
    FP16 h;
    h.u = hh;
    static const FP32 magic = {113 << 23};
    static const unsigned int shifted_exp = 0x7c00 << 13;  // exponent mask after shift
    FP32 o;

    o.u = (h.u & 0x7fff) << 13;    // exponent/mantissa bits
    unsigned int exp = shifted_exp & o.u;  // just the exponent
    o.u += (127 - 15) << 23;       // exponent adjust

    // handle exponent special cases
    if (exp == shifted_exp)       // Inf/NaN?
        o.u += (128 - 16) << 23;  // extra exp adjust
    else if (exp == 0)            // Zero/Denormal?
    {
        o.u += 1 << 23;  // extra exp adjust
        o.f -= magic.f;  // renormalize
    }

    o.u |= (h.u & 0x8000) << 16;  // sign bit
    return o.f;
}

}  // namespace pbrt

#endif  // PBRT_CORE_HALF_H
