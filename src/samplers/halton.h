
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

#ifndef PBRT_SAMPLERS_HALTON_H
#define PBRT_SAMPLERS_HALTON_H

// samplers/halton.h*
#include "sampler.h"
#include "lowdiscrepancy.h"

// HaltonSampler Declarations
class HaltonSampler : public GlobalSampler {
  public:
    // HaltonSampler Public Methods
    HaltonSampler(int nsamp, const Bounds2i &sampleBounds);
    int64_t GetIndexForSample(int64_t sampleNum) const;
    Float SampleDimension(int64_t index, int dimension) const;
    std::unique_ptr<Sampler> Clone(int seed);

  private:
    // HaltonSampler Private Data
    static std::vector<uint16_t> radicalInversePermutations;
    Point2i baseScales, baseExponents;
    int sampleStride;
    int multInverse[2];
    mutable Point2i pixelForOffset = Point2i(std::numeric_limits<int>::max(),
                                             std::numeric_limits<int>::max());
    mutable int64_t offsetForCurrentPixel;

    // HaltonSampler Private Methods
    const uint16_t *PermutationForDimension(int dim) const {
        if (dim >= PrimeTableSize)
            LOG(FATAL) << StringPrintf("HaltonSampler can only sample %d "
                                       "dimensions.", PrimeTableSize);
        return &radicalInversePermutations[PrimeSums[dim]];
    }
};

HaltonSampler *CreateHaltonSampler(const ParamSet &params,
                                   const Bounds2i &sampleBounds);

#endif  // PBRT_SAMPLERS_HALTON_H
