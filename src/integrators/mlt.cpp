
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

STAT_TIMER("Time/Rendering", renderingTime);
STAT_PERCENT("Integrator/Acceptance rate", acceptedMutations, totalMutations);

// MLTSampler Method Definitions
MLTSampler::MLTSampler(int64_t mutationsPerPixel, int id, Float sigma,
                       Float largeStepProb)
    : Sampler(mutationsPerPixel),
      rng(PCG32_DEFAULT_STATE, (uint64_t)id),
      sigma(sigma),
      largeStepProb(largeStepProb) {}
Float MLTSampler::Get1D() {
    int index = GetNextIndex();
    EnsureReady(index);
    return samples[index].value;
}

Point2f MLTSampler::Get2D() { return Point2f(Get1D(), Get1D()); }

std::unique_ptr<Sampler> MLTSampler::Clone(int seed) {
    throw std::runtime_error("Unsupported operation");
}

void MLTSampler::Begin() {
    iteration++;
    largeStep = rng.UniformFloat() < largeStepProb;
}

void MLTSampler::Accept() {
    if (largeStep) lastLargeStep = iteration;
}

void MLTSampler::Reject() {
    for (auto &sample : samples)
        if (sample.modify == iteration) sample.Restore();
    --iteration;
}

void MLTSampler::EnsureReady(int index) {
    // Get current _sample_ and enlarge _MLTSampler::samples_ if necessary
    while (index >= samples.size()) samples.push_back(PrimarySample());
    PrimarySample &sample = samples[index];

    // Reset _sample_ if a large step took place in the meantime
    if (sample.modify < lastLargeStep) {
        sample.value = rng.UniformFloat();
        sample.modify = lastLargeStep;
    }

    // Apply remaining sequence of mutations to _sample_
    sample.Backup();
    if (largeStep) {
        sample.value = rng.UniformFloat();
    } else {
        int nSmall = iteration - sample.modify;
        // Apply _nSmall_ small step mutations

        // Sample the standard normal distribution $N(0, 1)$
        Float normalSample = Sqrt2 * ErfInv(2 * rng.UniformFloat() - 1);

        // Compute the effective standard deviation and apply the perturbation
        Float effSigma = sigma * std::sqrt((Float)nSmall);
        sample.value += normalSample * effSigma;
        sample.value -= std::floor(sample.value);
    }
    sample.modify = iteration;
}

void MLTSampler::SetStream(int streamCount_, int streamIndex_) {
    streamCount = streamCount_;
    streamIndex = streamIndex_;
    sampleIndex = 0;
}

// MLT Method Definitions
Spectrum MLTIntegrator::EvaluateSample(const Scene &scene, MemoryArena &arena,
                                       MLTSampler &sampler, int k,
                                       Point2f *samplePos) {
    sampler.SetStream(3, 0);
    // Determine the number of available strategies and pick a specific one
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

    // Generate a camera subpath with exactly _t_ vertices
    Vertex *cameraSubpath = (Vertex *)arena.Alloc<Vertex>(t);
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i diag = sampleBounds.Diagonal();
    Point2f filmSample = sampler.Get2D();
    *samplePos = Point2f(sampleBounds.pMin.x + diag.x * filmSample.x,
                         sampleBounds.pMin.y + diag.y * filmSample.y);
    if (GenerateCameraSubpath(scene, sampler, arena, t, *camera, *samplePos,
                              cameraSubpath) != t)
        return Spectrum(0.f);

    // Generate a light subpath with exactly _s_ vertices
    Vertex *lightSubpath = (Vertex *)arena.Alloc<Vertex>(s);
    sampler.SetStream(3, 1);
    if (GenerateLightSubpath(scene, sampler, arena, s,
                             cameraSubpath[0].GetTime(), *lightDistr,
                             lightSubpath) != s)
        return Spectrum(0.f);

    // Execute connection strategy and return the radiance estimate
    sampler.SetStream(3, 2);
    Float misWeight;
    Spectrum weight =
        ConnectBDPT(scene, lightSubpath, cameraSubpath, s, t, *lightDistr,
                    *camera, sampler, samplePos, &misWeight);
    arena.Reset();
    return weight * misWeight * nStrategies;
}

void MLTIntegrator::Render(const Scene &scene) {
    Film &film = *camera->film;
    lightDistr =
        std::unique_ptr<Distribution1D>(ComputeLightSamplingCDF(scene));
    // Generate bootstrap samples
    int bootstrapSamples = nBootstrap * (maxdepth + 1);
    std::unique_ptr<Float[]> bootstrapWeights(new Float[bootstrapSamples]);
    {
        ProgressReporter progress(nBootstrap, "Generating bootstrap paths");
        ParallelFor([&](int i) {
            MemoryArena arena;
            for (int k = 0; k <= maxdepth; ++k) {
                uint32_t sampleIndex = i * (maxdepth + 1) + k;
                MLTSampler sampler(mutationsPerPixel, sampleIndex, sigma,
                                   largeStepProb);
                Point2f samplePos;
                bootstrapWeights[sampleIndex] =
                    EvaluateSample(scene, arena, sampler, k, &samplePos).y();
            }
            progress.Update();
        }, nBootstrap);
        progress.Done();
    }

    // Create bootstrap distribution and estimate integral of $I$ over entire
    // domain
    Distribution1D bootstrap(bootstrapWeights.get(), bootstrapSamples);
    Float average = 0.f;
    for (int i = 0; i < bootstrapSamples; ++i) average += bootstrapWeights[i];
    average /= nBootstrap;

    // Run _nChains_ Markov Chains in parallel
    int64_t nMutations =
        mutationsPerPixel * (int64_t)camera->film->GetSampleBounds().Area();
    int64_t mutationsPerChain = nMutations / nChains;
    {
        StatTimer timer(&renderingTime);
        ProgressReporter progress(nMutations / 100, "Rendering");
        ParallelFor([&](int taskNum) {
            int64_t start = taskNum * mutationsPerChain;
            int64_t end =
                std::min((taskNum + 1) * mutationsPerChain, nMutations);
            MemoryArena arena;
            // Select initial state from bootstrap samples
            RNG rng;
            rng.Advance(start);  // XXX
            Float pdf;
            int chainIndex = bootstrap.SampleDiscrete(rng.UniformFloat(), &pdf);

            MLTSampler sampler(mutationsPerPixel, chainIndex, sigma,
                               largeStepProb);

            Point2f currentPos, proposalPos;
            Spectrum currentValue, proposalValue;

            int k = chainIndex % (maxdepth + 1);
            currentValue =
                EvaluateSample(scene, arena, sampler, k, &currentPos);

            // Run the Markov Chain
            for (int64_t i = start; i != end; ++i) {
                sampler.Begin();
                proposalValue =
                    EvaluateSample(scene, arena, sampler, k, &proposalPos);
                // Accept or reject the proposal
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
