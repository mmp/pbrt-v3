
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

#ifndef PBRT_CORE_LOWDISCREPANCY_H
#define PBRT_CORE_LOWDISCREPANCY_H
#include "stdafx.h"

// core/lowdiscrepancy.h*
#include "pbrt.h"
#include "rng.h"
#include "sampling.h"
#include "sobolmatrices.h"

// Low Discrepancy Declarations
Float RadicalInverse(int baseIndex, uint64_t a);
std::vector<uint16_t> ComputeRadicalInversePermutations(RNG &rng);
static constexpr int PrimeTableSize = 1000;
extern const int Primes[PrimeTableSize];
Float ScrambledRadicalInverse(int baseIndex, uint64_t a, const uint16_t *perm);
extern const int PrimeSums[PrimeTableSize];
inline void Sobol2D(int nSamplesPerPixelSample, int nPixelSamples,
                    Point2f *samples, RNG &rng);
extern uint32_t CMaxMinDist[17][32];
inline uint64_t SobolIntervalToIndex(const uint32_t log2Resolution,
                                     uint64_t sampleNum, const Point2i &p);
inline float SobolSampleFloat(int64_t index, int dimension,
                              uint32_t scramble = 0);
inline double SobolSampleDouble(int64_t index, int dimension,
                                uint64_t scramble = 0);

// Low Discrepancy Inline Functions
inline uint32_t ReverseBits32(uint32_t n) {
    n = (n << 16) | (n >> 16);
    n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
    n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
    n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
    n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
    return n;
}

inline uint64_t ReverseBits64(uint64_t n) {
    uint64_t n0 = ReverseBits32((uint32_t)n);
    uint64_t n1 = ReverseBits32((uint32_t)(n >> 32));
    return (n0 << 32) | n1;
}

template <int base>
inline uint64_t InverseRadicalInverse(uint64_t inverse, int nDigits) {
    uint64_t index = 0;
    for (int i = 0; i < nDigits; ++i) {
        uint64_t digit = inverse % base;
        inverse /= base;
        index = index * base + digit;
    }
    return index;
}

inline uint32_t MultiplyGenerator(const uint32_t *C, uint32_t a) {
    uint32_t v = 0;
    for (int i = 0; a != 0; ++i, a >>= 1)
        if (a & 1) v ^= C[i];
    return v;
}

inline uint32_t ReverseMultiplyGenerator(const uint32_t *C, uint32_t a) {
    uint32_t v = 0;
    for (int i = 31; a != 0; --i, a >>= 1) {
        if (a & 1) v ^= C[i];
    }
    return v;
}

inline Float SampleGeneratorMatrix(const uint32_t *C, uint32_t a,
                                   uint32_t scramble = 0) {
#ifdef PBRT_IS_MSVC
    return (ReverseMultiplyGenerator(C, a) ^ scramble) * 2.3283064365386963e-10f;
#else
    return (ReverseMultiplyGenerator(C, a) ^ scramble) * 0x1p-32f;
#endif
}

inline uint32_t GrayCode(uint32_t v) { return (v >> 1) ^ v; }

inline void GrayCodeSample(const uint32_t *C, uint32_t n, uint32_t scramble,
                           Float *p) {
    uint32_t v = scramble;
    for (uint32_t i = 0; i < n; ++i) {
#ifdef PBRT_IS_MSVC
        p[i] = v * 2.3283064365386963e-10f; /* 1/2^32 */
#else
        p[i] = v * 0x1p-32f; /* 1/2^32 */
#endif
        v ^= C[31 - CountTrailingZeros(i + 1)];
    }
}

inline void GrayCodeSample(const uint32_t *C0, const uint32_t *C1, uint32_t n,
                           const Point2i &scramble, Point2f *p) {
    uint32_t v[2] = {(uint32_t)scramble.x, (uint32_t)scramble.y};
    for (uint32_t i = 0; i < n; ++i) {
#ifdef PBRT_IS_MSVC
        p[i].x = v[0] * 2.3283064365386963e-10f;
        p[i].y = v[1] * 2.3283064365386963e-10f;
#else
        p[i].x = v[0] * 0x1p-32f;
        p[i].y = v[1] * 0x1p-32f;
#endif
        v[0] ^= C0[31 - CountTrailingZeros(i + 1)];
        v[1] ^= C1[31 - CountTrailingZeros(i + 1)];
    }
}

inline void VanDerCorput(int nSamplesPerPixelSample, int nPixelSamples,
                         Float *samples, RNG &rng) {
    uint32_t scramble = rng.UniformUInt32();
    // Define _CVanDerCorput_ Generator Matrix
    const uint32_t CVanDerCorput[] = {
        0b00000000000000000000000000000001, 0b00000000000000000000000000000010,
        0b00000000000000000000000000000100, 0b00000000000000000000000000001000,
        // Remainder of Van Der Corput generator matrix entries
        0b10000, 0b100000, 0b1000000, 0b10000000, 0b100000000, 0b1000000000,
        0b10000000000, 0b100000000000, 0b1000000000000, 0b10000000000000,
        0b100000000000000, 0b1000000000000000, 0b10000000000000000,
        0b100000000000000000, 0b1000000000000000000, 0b10000000000000000000,
        0b100000000000000000000, 0b1000000000000000000000,
        0b10000000000000000000000, 0b100000000000000000000000,
        0b1000000000000000000000000, 0b10000000000000000000000000,
        0b100000000000000000000000000, 0b1000000000000000000000000000,
        0b10000000000000000000000000000, 0b100000000000000000000000000000,
        0b1000000000000000000000000000000, 0b10000000000000000000000000000000,
    };
    int totalSamples = nSamplesPerPixelSample * nPixelSamples;
    GrayCodeSample(CVanDerCorput, totalSamples, scramble, samples);
    // Randomly shuffle 1D sample points
    for (int i = 0; i < nPixelSamples; ++i)
        Shuffle(samples + i * nSamplesPerPixelSample, nSamplesPerPixelSample, 1,
                rng);
    Shuffle(samples, nPixelSamples, nSamplesPerPixelSample, rng);
}

inline void Sobol2D(int nSamplesPerPixelSample, int nPixelSamples,
                    Point2f *samples, RNG &rng) {
    Point2i scramble(rng.UniformUInt32(), rng.UniformUInt32());
    // Define 2D Sobol$'$ generator matrices _CSobol[2]_
    const uint32_t CSobol[2][32] = {
        {0x1u, 0x2u, 0x4u, 0x8u, 0x10u, 0x20u, 0x40u, 0x80u, 0x100u, 0x200u,
         0x400u, 0x800u, 0x1000u, 0x2000u, 0x4000u, 0x8000u, 0x10000u, 0x20000u,
         0x40000u, 0x80000u, 0x100000u, 0x200000u, 0x400000u, 0x800000u,
         0x1000000u, 0x2000000u, 0x4000000u, 0x8000000u, 0x10000000u,
         0x20000000u, 0x40000000u, 0x80000000u},
        {
            0xffffffffu, 0xaaaaaaaau, 0xccccccccu, 0x88888888u, 0xf0f0f0f0u,
            0xa0a0a0a0u, 0xc0c0c0c0u, 0x80808080u, 0xff00ff00u, 0xaa00aa00u,
            0xcc00cc00u, 0x88008800u, 0xf000f000u, 0xa000a000u, 0xc000c000u,
            0x80008000u, 0xffff0000u, 0xaaaa0000u, 0xcccc0000u, 0x88880000u,
            0xf0f00000u, 0xa0a00000u, 0xc0c00000u, 0x80800000u, 0xff000000u,
            0xaa000000u, 0xcc000000u, 0x88000000u, 0xf0000000u, 0xa0000000u,
            0xc0000000u, 0x80000000u,
        }};
    GrayCodeSample(CSobol[0], CSobol[1], nSamplesPerPixelSample * nPixelSamples,
                   scramble, samples);
    for (int i = 0; i < nPixelSamples; ++i)
        Shuffle(samples + i * nSamplesPerPixelSample, nSamplesPerPixelSample, 1,
                rng);
    Shuffle(samples, nPixelSamples, nSamplesPerPixelSample, rng);
}

inline uint64_t SobolIntervalToIndex(const uint32_t m, uint64_t frame,
                                     const Point2i &p) {
    if (m == 0) return 0;

    const uint32_t m2 = m << 1;
    uint64_t index = uint64_t(frame) << m2;

    uint64_t delta = 0;
    for (int c = 0; frame; frame >>= 1, ++c)
        if (frame & 1)  // Add flipped column m + c + 1.
            delta ^= VdCSobolMatrices[m - 1][c];

    // flipped b
    uint64_t b = (((uint64_t)((uint32_t)p.x) << m) | ((uint32_t)p.y)) ^ delta;

    for (int c = 0; b; b >>= 1, ++c)
        if (b & 1)  // Add column 2 * m - c.
            index ^= VdCSobolMatricesInv[m - 1][c];

    return index;
}

inline Float SobolSample(int64_t index, int dimension, uint64_t scramble = 0) {
#ifdef PBRT_FLOAT_AS_DOUBLE
    return SobolSampleDouble(index, dimension, scramble);
#else
    return SobolSampleFloat(index, dimension, scramble);
#endif
}

inline float SobolSampleFloat(int64_t a, int dimension, uint32_t scramble) {
    Assert(dimension < NumSobolDimensions);
    uint32_t v = scramble;
    for (int i = dimension * SobolMatrixSize + SobolMatrixSize - 1; a != 0;
         a >>= 1, --i)
        if (a & 1) v ^= SobolMatrices32[i];
#ifdef PBRT_IS_MSVC
    return v * 2.3283064365386963e-10f; /* 1/2^32 */
#else
    return v * 0x1p-32f; /* 1/2^32 */
#endif
}

inline double SobolSampleDouble(int64_t index, int dimension,
                                uint64_t scramble) {
    Assert(dimension < NumSobolDimensions);
    uint64_t result = scramble & ~ - (1LL << SobolMatrixSize);
    for (int i = dimension * SobolMatrixSize + SobolMatrixSize - 1; index != 0;
         index >>= 1, --i)
        if (index & 1) result ^= SobolMatrices64[i];
    return result * (1.0 / (1ULL << SobolMatrixSize));
}

#endif  // PBRT_CORE_LOWDISCREPANCY_H
