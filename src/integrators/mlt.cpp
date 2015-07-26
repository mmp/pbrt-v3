
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

// MLTSampler Constants
static const int cameraStreamIndex = 0;
static const int lightStreamIndex = 1;
static const int connectionStreamIndex = 2;
static const int numSampleStreams = 3;

// MLTSampler Method Definitions
Float MLTSampler::Get1D() {
    int index = GetNextIndex();
    EnsureReady(index);
    return X[index].value;
}

Point2f MLTSampler::Get2D() { return Point2f(Get1D(), Get1D()); }

std::unique_ptr<Sampler> MLTSampler::Clone(int seed) {
    Severe("MLTSampler::Clone() is not implemented");
    return nullptr;
}

void MLTSampler::StartIteration() {
    iteration++;
    largeStep = rng.UniformFloat() < largeStepProb;
}

void MLTSampler::Accept() {
    if (largeStep) lastLargeStepIteration = iteration;
}

void MLTSampler::Reject() {
    for (auto &sample : X)
        if (sample.lastModificationIteration == iteration) sample.Restore();
    --iteration;
}

void MLTSampler::EnsureReady(int index) {
    // Get current $x_i$ and enlarge _MLTSampler::X_ if necessary
    if (index >= X.size()) X.resize(index + 1);
    PrimarySample &xi = X[index];

    // Reset $x_i$ if a large step took place in the meantime
    if (xi.lastModificationIteration < lastLargeStepIteration) {
        xi.value = rng.UniformFloat();
        xi.lastModificationIteration = lastLargeStepIteration;
    }

    // Apply remaining sequence of mutations to _sample_
    xi.Backup();
    if (largeStep) {
        xi.value = rng.UniformFloat();
    } else {
        int nSmall = iteration - xi.lastModificationIteration;
        // Apply _nSmall_ small step mutations

        // Sample the standard normal distribution $N(0, 1)$
        Float normalSample = Sqrt2 * ErfInv(2 * rng.UniformFloat() - 1);

        // Compute the effective standard deviation and apply the perturbation
        Float effSigma = sigma * std::sqrt((Float)nSmall);
        xi.value += normalSample * effSigma;
        xi.value -= std::floor(xi.value);
    }
    xi.lastModificationIteration = iteration;
}

void MLTSampler::StartStream(int index) {
    streamIndex = index;
    sampleIndex = 0;
}

// MLT Method Definitions
Spectrum MLTIntegrator::L(const Scene &scene, MemoryArena &arena,
                          MLTSampler &sampler, int depth, Point2f *samplePos) {
    sampler.StartStream(cameraStreamIndex);
    // Determine the number of available strategies and pick a specific one
    int s, t, nStrategies;
    if (depth == 0) {
        nStrategies = 1;
        s = 0;
        t = 2;
    } else {
        nStrategies = depth + 2;
        s = std::min((int)(sampler.Get1D() * nStrategies), nStrategies - 1);
        t = nStrategies - s;
    }

    // Generate a camera subpath with exactly _t_ vertices
    Vertex *cameraVertices = (Vertex *)arena.Alloc<Vertex>(t);
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i diag = sampleBounds.Diagonal();
    *samplePos = Point2f(sampleBounds.pMin.x + diag.x * sampler.Get1D(),
                         sampleBounds.pMin.y + diag.y * sampler.Get1D());
    if (GenerateCameraSubpath(scene, sampler, arena, t, *camera, *samplePos,
                              cameraVertices) != t)
        return Spectrum(0.f);

    // Generate a light subpath with exactly _s_ vertices
    sampler.StartStream(lightStreamIndex);
    Vertex *lightVertices = (Vertex *)arena.Alloc<Vertex>(s);
    if (GenerateLightSubpath(scene, sampler, arena, s, cameraVertices[0].time(),
                             *lightDistr, lightVertices) != s)
        return Spectrum(0.f);

    // Execute connection strategy and return the radiance estimate
    sampler.StartStream(connectionStreamIndex);
    Spectrum L = ConnectBDPT(scene, lightVertices, cameraVertices, s, t,
                             *lightDistr, *camera, sampler, samplePos);
    arena.Reset();
    return L * nStrategies;
}

void MLTIntegrator::Render(const Scene &scene) {
    lightDistr = ComputeLightPowerDistribution(scene);
    Film &film = *camera->film;
    // Generate bootstrap samples and compute $b$
    int bootstrapSamples = nBootstrap * (maxDepth + 1);
    std::unique_ptr<Float[]> bootstrapWeights(new Float[bootstrapSamples]);
    {
        ProgressReporter progress(nBootstrap, "Generating bootstrap paths");
        ParallelFor([&](int k) {
            // Generate a single bootstrap sample
            MemoryArena arena;
            for (int depth = 0; depth <= maxDepth; ++depth) {
                int uIndex = k * (maxDepth + 1) + depth;
                MLTSampler sampler(mutationsPerPixel, uIndex, sigma,
                                   largeStepProb, numSampleStreams);
                Point2f samplePos;
                bootstrapWeights[uIndex] =
                    L(scene, arena, sampler, depth, &samplePos).y();
            }
            progress.Update();
        }, nBootstrap);
        progress.Done();
    }
    Distribution1D bootstrap(bootstrapWeights.get(), bootstrapSamples);
    Float b = bootstrap.funcInt * (maxDepth + 1);

    // Run _nChains_ Markov Chains in parallel
    int64_t nTotalMutations =
        mutationsPerPixel * (int64_t)film.GetSampleBounds().Area();
    {
        StatTimer timer(&renderingTime);
        ProgressReporter progress(nTotalMutations / 100, "Rendering");
        ParallelFor([&](int k) {
            int64_t nChainMutations =
                std::min((k + 1) * nTotalMutations / nChains, nTotalMutations) -
                k * nTotalMutations / nChains;
            MemoryArena arena;
            std::unique_ptr<FilmTile> filmTile = film.GetFilmTile(Bounds2i(
                film.croppedPixelBounds.pMin, film.croppedPixelBounds.pMin));
            // Select initial state from the set of bootstrap samples
            RNG rng(PCG32_DEFAULT_STATE, k);
            int bootstrapIndex = bootstrap.SampleDiscrete(rng.UniformFloat());
            int depth = bootstrapIndex % (maxDepth + 1);

            // Initialize local variables for selected state
            MLTSampler sampler(mutationsPerPixel, bootstrapIndex, sigma,
                               largeStepProb, numSampleStreams);
            Point2f pCurrent;
            Spectrum LCurrent = L(scene, arena, sampler, depth, &pCurrent);

            // Run the Markov Chain for _nChainMutations_ steps
            for (int64_t i = 0; i < nChainMutations; ++i) {
                sampler.StartIteration();
                Point2f pProposed;
                Spectrum LProposed =
                    L(scene, arena, sampler, depth, &pProposed);
                // Compute acceptance proability for proposed sample
                Float accept = std::min((Float)1, LProposed.y() / LCurrent.y());

                // Splat both current and proposed samples to _FilmTile_
                if (accept > 0)
                    filmTile->AddSplat(pProposed,
                                       LProposed * accept / LProposed.y());
                filmTile->AddSplat(pCurrent,
                                   LCurrent * (1 - accept) / LCurrent.y());

                // Accept or reject the proposal
                if (rng.UniformFloat() < accept) {
                    pCurrent = pProposed;
                    LCurrent = LProposed;
                    sampler.Accept();
                    ++acceptedMutations;
                } else {
                    sampler.Reject();
                }
                ++totalMutations;
                if (i % 100 == 0) progress.Update();
            }
            film.MergeFilmTile(std::move(filmTile));
        }, nChains);
        progress.Done();
    }
    film.WriteImage(b / mutationsPerPixel);
}

MLTIntegrator *CreateMLTIntegrator(const ParamSet &params,
                                   std::shared_ptr<const Camera> camera) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int nBootstrap = params.FindOneInt("bootstrapsamples", 100000);
    int64_t nChains = params.FindOneInt("chains", 1000);
    int64_t mutationsPerPixel = params.FindOneInt("mutationsperpixel", 100);
    Float largeStepProbability =
        params.FindOneFloat("largestepprobability", 0.3f);
    Float sigma = params.FindOneFloat("sigma", .01f);
    if (PbrtOptions.quickRender) {
        mutationsPerPixel = std::max((int64_t)1, mutationsPerPixel / 16);
        nBootstrap = std::max(1, nBootstrap / 16);
    }
    return new MLTIntegrator(camera, maxDepth, nBootstrap, nChains,
                             mutationsPerPixel, sigma, largeStepProbability);
}
