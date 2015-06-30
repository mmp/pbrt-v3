
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

// integrators/mlt.cpp*
#include "integrators/mlt.h"
#include "integrators/bdpt.h"
#include "scene.h"
#include "film.h"
#include "sampler.h"
#include "integrator.h"
#include "camera.h"
#include "stats.h"
#include "filters/box.h"
#include "paramset.h"
#include "sampling.h"
#include "progressreporter.h"

using std::cout;
using std::endl;

STAT_TIMER("Time/Rendering", renderingTime);
STAT_PERCENT("Integrator/Acceptance rate", acceptedMutations, totalMutations);

// MLTSampler Method Definitions
MLTSampler::MLTSampler(int chainIndex, Float sigma, Float largeStepProb)
    : Sampler(16),
      rng(PCG32_DEFAULT_STATE, (uint32_t)chainIndex),
      sigma(sigma),
      largeStepProb(largeStepProb) {}
Float MLTSampler::Get1D() { return GetSample(sampleIndex++); }

Point2f MLTSampler::Get2D() {
    Float value1 = Get1D(), value2 = Get1D();
    return Point2f(value1, value2);
}

std::unique_ptr<Sampler> MLTSampler::Clone(int seed) {
    throw std::runtime_error("Unsupported operation");
}

void MLTSampler::SetStream(int streamCount_, int streamIndex_) {
    streamCount = streamCount_;
    streamIndex = streamIndex_;
    sampleIndex = 0;
}

void MLTSampler::Begin() {
    largeStep = rng.UniformFloat() < largeStepProb;
    iteration++;
}

void MLTSampler::Accept() {
    if (largeStep) lastLargeStep = iteration;
}

void MLTSampler::Reject() {
    for (auto &sample : samples) {
        if (sample.modify == iteration) sample.Restore();
    }
    --iteration;
}

Float MLTSampler::NextNormalVariate() {
    Float result = normalSample;

    if (normalSample == Infinity) {
        Float Z = std::sqrt(-2.f * std::log(1.f - rng.UniformFloat()));
        Float phi = 2 * Pi * rng.UniformFloat();
        result = Z * std::cos(phi);
        normalSample = Z * std::sin(phi);
    } else {
        normalSample = Infinity;
    }

    return result;
}

Float MLTSampler::GetSample(int index) {
    index = streamCount * index + streamIndex;

    while (index >= samples.size()) samples.push_back(MLTSample());

    MLTSample &sample = samples[index];

    if (sample.modify < lastLargeStep) {
        sample.value = rng.UniformFloat();
        sample.modify = lastLargeStep;
    }

    sample.Backup();
    if (largeStep) {
        sample.value = rng.UniformFloat();
    } else {
        Float combinedSigma =
            std::sqrt((Float)(iteration - sample.modify)) * sigma;
        sample.value += NextNormalVariate() * combinedSigma;
        sample.value -= std::floor(sample.value);
    }

    sample.modify = iteration;
    return sample.value;
}

// MLT Method Definitions
Spectrum MLTIntegrator::EvaluateSample(const Scene &scene, MemoryArena &arena,
                                       MLTSampler &sampler, int k,
                                       Point2f *samplePos) {
    sampler.SetStream(3, 0);

    int s, t, nStrategies;

    if (k == 0) {
        s = 0;
        t = 2;
        nStrategies = 1;
    } else {
        nStrategies = k + 2;
        s = std::min((int)(sampler.Get1D() * nStrategies), nStrategies - 1);
        t = nStrategies - s;
    }

    Vertex *cameraSubpath = (Vertex *)arena.Alloc<Vertex>(t);
    Vertex *lightSubpath = (Vertex *)arena.Alloc<Vertex>(s);

    Point2f filmSample = sampler.Get2D();
    const Bounds2i sampleBounds = camera->film->GetSampleBounds();
    *samplePos =
        Point2f(sampleBounds.pMin.x +
                    (sampleBounds.pMax.x - sampleBounds.pMin.x) * filmSample.x,
                sampleBounds.pMin.y +
                    (sampleBounds.pMax.y - sampleBounds.pMin.y) * filmSample.y);

    if (GenerateCameraSubpath(scene, sampler, arena, t, *camera, *samplePos,
                              cameraSubpath) != t)
        return Spectrum(0.f);

    sampler.SetStream(3, 1);
    if (GenerateLightSubpath(scene, sampler, arena, s,
                             cameraSubpath[0].GetTime(), *lightDistr,
                             lightSubpath) != s)
        return Spectrum(0.f);

    sampler.SetStream(3, 2);
    Float misWeight;
    Spectrum weight =
        ConnectBDPT(scene, lightSubpath, cameraSubpath, s, t, *lightDistr,
                    *camera, sampler, samplePos, &misWeight);

    arena.Reset();

    return weight * misWeight * nStrategies;
}

void MLTIntegrator::Render(const Scene &scene) {
    lightDistr =
        std::unique_ptr<Distribution1D>(ComputeLightSamplingCDF(scene));

    int bootstrapSamples = nBootstrap * (maxdepth + 1);
    std::unique_ptr<Float[]> bootstrapWeights(new Float[bootstrapSamples]);
    {
        ProgressReporter progress(nBootstrap, "Generating bootstrap paths");
#if defined(PBRT_IS_MSVC) && (__MWKM__)
		// VS2015_mwkm: ParallelFor ambiguous call
		ParallelFor((const std::function<void(int)>)[&](int i) {
#else
		ParallelFor([&](int i) {
#endif
            MemoryArena arena;
            for (int k = 0; k <= maxdepth; ++k) {
                uint32_t sampleIndex = i * (maxdepth + 1) + k;
                MLTSampler sampler(sampleIndex, sigma, largeStepProb);
                Point2f samplePos;
                bootstrapWeights[sampleIndex] =
                    EvaluateSample(scene, arena, sampler, k, &samplePos).y();
            }
            progress.Update();
        }, nBootstrap);
        progress.Done();
    }
    Distribution1D bootstrap(bootstrapWeights.get(), bootstrapSamples);
    Float average = 0.f;
    for (int i = 0; i < bootstrapSamples; ++i) average += bootstrapWeights[i];
    average /= nBootstrap;

    int64_t nMutations =
        mutationsPerPixel * (int64_t)camera->film->GetSampleBounds().Area();
    int64_t mutationsPerChain = nMutations / nChains;
    Film &film = *camera->film;
    {
        StatTimer timer(&renderingTime);
        ProgressReporter progress(nMutations / 100, "Rendering");
#if defined(PBRT_IS_MSVC) && (__MWKM__)
		// VS2015_mwkm: ParallelFor ambiguous call
		ParallelFor((const std::function<void(int)>)[&](int taskNum) {
#else
		ParallelFor([&](int taskNum) {
#endif
            int64_t start = taskNum * mutationsPerChain;
            int64_t end =
                std::min((taskNum + 1) * mutationsPerChain, nMutations);
            Assert(start < end);
            MemoryArena arena;

            RNG rng;
            rng.Advance(start);
            Float pdf;
            int chainIndex = bootstrap.SampleDiscrete(rng.UniformFloat(), &pdf);

            MLTSampler sampler(chainIndex, sigma, largeStepProb);

            Point2f currentPos, proposalPos;
            Spectrum currentValue, proposalValue;

            uint32_t k = chainIndex % (maxdepth + 1);
            currentValue =
                EvaluateSample(scene, arena, sampler, k, &currentPos);

            for (int64_t i = start; i != end; ++i) {
                sampler.Begin();
                proposalValue =
                    EvaluateSample(scene, arena, sampler, k, &proposalPos);
                Float accept = proposalValue.y() / currentValue.y();
                if (rng.UniformFloat() < accept) {
                    currentPos = proposalPos;
                    currentValue = proposalValue;
                    sampler.Accept();
                    ++acceptedMutations;
                } else {
                    sampler.Reject();
                }
                ++totalMutations;
                film.Splat(currentPos, currentValue / currentValue.y());
                if (i % 100 == 0) progress.Update();
            }

        }, nChains);
        progress.Done();
    }

    film.WriteImage(average / mutationsPerPixel);
}

MLTIntegrator *CreateMLTIntegrator(const ParamSet &params,
                                   std::shared_ptr<const Camera> camera) {
    int maxdepth = params.FindOneInt("maxdepth", 5);
    int nBootstrap = params.FindOneInt("bootstrapsamples", 100000);
    int64_t nChains = params.FindOneInt("chains", 1000);
    int64_t mutationsPerPixel = params.FindOneInt("mutationsperpixel", 100);
    Float largeStepProbability =
        params.FindOneFloat("largestepprobability", 0.3f);
    Float sigma = params.FindOneFloat("sigma", .01f);
    if (PbrtOptions.quickRender)
        mutationsPerPixel = std::max((int64_t)1, mutationsPerPixel / 16);
    return new MLTIntegrator(camera, maxdepth, nBootstrap, nChains,
                             mutationsPerPixel, sigma, largeStepProbability);
}
