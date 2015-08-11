
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

// samplers/maxmin.cpp*
#include "samplers/maxmin.h"
#include "paramset.h"

// MaxMinDistSampler Method Definitions
void MaxMinDistSampler::StartPixel(const Point2i &p) {
    Float invSPP = (Float)1 / samplesPerPixel;
    for (int i = 0; i < samplesPerPixel; ++i)
        samples2D[0][i] = Point2f(i * invSPP, SampleGeneratorMatrix(CPixel, i));
    Shuffle(&samples2D[0][0], samplesPerPixel, 1, rng);
    // Generate remaining samples for _MaxMinDistSampler_
    for (size_t i = 0; i < samples1D.size(); ++i)
        VanDerCorput(1, samplesPerPixel, &samples1D[i][0], rng);

    for (size_t i = 1; i < samples2D.size(); ++i)
        Sobol2D(1, samplesPerPixel, &samples2D[i][0], rng);

    for (size_t i = 0; i < samples1DArraySizes.size(); ++i) {
        int count = samples1DArraySizes[i];
        VanDerCorput(count, samplesPerPixel, &sampleArray1D[i][0], rng);
    }

    for (size_t i = 0; i < samples2DArraySizes.size(); ++i) {
        int count = samples2DArraySizes[i];
        Sobol2D(count, samplesPerPixel, &sampleArray2D[i][0], rng);
    }
    PixelSampler::StartPixel(p);
}

std::unique_ptr<Sampler> MaxMinDistSampler::Clone(int seed) {
    MaxMinDistSampler *mmds = new MaxMinDistSampler(*this);
    mmds->rng.SetSequence(seed);
    return std::unique_ptr<Sampler>(mmds);
}

MaxMinDistSampler *CreateMaxMinDistSampler(const ParamSet &params) {
    int nsamp = params.FindOneInt("pixelsamples", 4);
    int sd = params.FindOneInt("dimensions", 4);
    if (PbrtOptions.quickRender) nsamp = 1;
    return new MaxMinDistSampler(nsamp, sd);
}
