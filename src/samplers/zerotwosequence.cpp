
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


// samplers/zerotwosequence.cpp*
#include "samplers/zerotwosequence.h"
#include "lowdiscrepancy.h"
#include "paramset.h"

// ZeroTwoSequenceSampler Method Definitions
ZeroTwoSequenceSampler::ZeroTwoSequenceSampler(int64_t samplesPerPixel,
                                               int nSampledDimensions)
    : PixelSampler(RoundUpPow2(samplesPerPixel), nSampledDimensions) {
    if (!IsPowerOf2(samplesPerPixel))
        Warning(
            "Pixel samples being rounded up to power of 2 "
            "(from %" PRId64 " to %" PRId64 ").",
            samplesPerPixel, RoundUpPow2(samplesPerPixel));
}

void ZeroTwoSequenceSampler::StartPixel(const Point2i &p) {
    // Generate 1D and 2D pixel sample components using $(0,2)$-sequence
    for (size_t i = 0; i < samples1D.size(); ++i)
        VanDerCorput(1, samplesPerPixel, &samples1D[i][0], rng);
    for (size_t i = 0; i < samples2D.size(); ++i)
        Sobol2D(1, samplesPerPixel, &samples2D[i][0], rng);

    // Generate 1D and 2D array samples using $(0,2)$-sequence
    for (size_t i = 0; i < samples1DArraySizes.size(); ++i)
        VanDerCorput(samples1DArraySizes[i], samplesPerPixel,
                     &sampleArray1D[i][0], rng);
    for (size_t i = 0; i < samples2DArraySizes.size(); ++i)
        Sobol2D(samples2DArraySizes[i], samplesPerPixel, &sampleArray2D[i][0],
                rng);
    PixelSampler::StartPixel(p);
}

std::unique_ptr<Sampler> ZeroTwoSequenceSampler::Clone(int seed) {
    ZeroTwoSequenceSampler *lds = new ZeroTwoSequenceSampler(*this);
    lds->rng.SetSequence(seed);
    return std::unique_ptr<Sampler>(lds);
}

ZeroTwoSequenceSampler *CreateZeroTwoSequenceSampler(const ParamSet &params) {
    int nsamp = params.FindOneInt("pixelsamples", 16);
    int sd = params.FindOneInt("dimensions", 4);
    if (PbrtOptions.quickRender) nsamp = 1;
    return new ZeroTwoSequenceSampler(nsamp, sd);
}
