
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


// samplers/sobol.cpp*
#include "samplers/sobol.h"
#include "lowdiscrepancy.h"
#include "paramset.h"

namespace pbrt {

// SobolSampler Method Definitions
int64_t SobolSampler::GetIndexForSample(int64_t sampleNum) const {
    return SobolIntervalToIndex(log2Resolution, sampleNum,
                                Point2i(currentPixel - sampleBounds.pMin));
}

Float SobolSampler::SampleDimension(int64_t index, int dim) const {
    if (dim >= NumSobolDimensions)
        LOG(FATAL) << StringPrintf("SobolSampler can only sample up to %d "
                                   "dimensions! Exiting.",
                                   NumSobolDimensions);
    Float s = SobolSample(index, dim);
    // Remap Sobol$'$ dimensions used for pixel samples
    if (dim == 0 || dim == 1) {
        s = s * resolution + sampleBounds.pMin[dim];
        s = Clamp(s - currentPixel[dim], (Float)0, OneMinusEpsilon);
    }
    return s;
}

std::unique_ptr<Sampler> SobolSampler::Clone(int seed) {
    return std::unique_ptr<Sampler>(new SobolSampler(*this));
}

SobolSampler *CreateSobolSampler(const ParamSet &params,
                                 const Bounds2i &sampleBounds) {
    int nsamp = params.FindOneInt("pixelsamples", 16);
    if (PbrtOptions.quickRender) nsamp = 1;
    return new SobolSampler(nsamp, sampleBounds);
}

}  // namespace pbrt
