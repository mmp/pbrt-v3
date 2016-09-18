
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

#ifndef PBRT_SAMPLERS_SOBOL_H
#define PBRT_SAMPLERS_SOBOL_H

// samplers/sobol.h*
#include "sampler.h"

// SobolSampler Declarations
class SobolSampler : public GlobalSampler {
  public:
    // SobolSampler Public Methods
    std::unique_ptr<Sampler> Clone(int seed);
    SobolSampler(int64_t samplesPerPixel, const Bounds2i &sampleBounds)
        : GlobalSampler(RoundUpPow2(samplesPerPixel)),
          sampleBounds(sampleBounds) {
        if (!IsPowerOf2(samplesPerPixel))
            Warning("Non power-of-two sample count rounded up to %" PRId64
                    " for SobolSampler.",
                    samplesPerPixel);
        resolution = RoundUpPow2(
            std::max(sampleBounds.Diagonal().x, sampleBounds.Diagonal().y));
        log2Resolution = Log2Int(resolution);
        CHECK_EQ(1 << log2Resolution, resolution);
    }
    int64_t GetIndexForSample(int64_t sampleNum) const;
    Float SampleDimension(int64_t index, int dimension) const;

  private:
    // SobolSampler Private Data
    const Bounds2i sampleBounds;
    int resolution, log2Resolution;
};

SobolSampler *CreateSobolSampler(const ParamSet &params,
                                 const Bounds2i &sampleBounds);

#endif  // PBRT_SAMPLERS_SOBOL_H
