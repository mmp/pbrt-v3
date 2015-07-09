
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

#ifndef PBRT_INTEGRATORS_MLT_H
#define PBRT_INTEGRATORS_MLT_H
#include "stdafx.h"

// integrators/mlt.h*
#include "pbrt.h"
#include "integrator.h"
#include "sampler.h"
#include "spectrum.h"
#include "film.h"
#include "rng.h"

// MLTSampler Declarations
class MLTSampler : public Sampler {
  public:
    // MLTSampler Public Methods
    MLTSampler(int64_t mutationsPerPixel, int id, Float sigma,
               Float largeStepProb);
    Float Get1D();
    Point2f Get2D();
    std::unique_ptr<Sampler> Clone(int seed);
    void Begin();
    void Accept();
    void Reject();
    void SetStream(int streamCount_, int streamIndex_);
    int GetNextIndex() { return streamIndex + streamCount * sampleIndex++; }

  protected:
    // MLTSampler Private Declarations
    struct PrimarySample {
        Float value = 0;
        // PrimarySample Members
        int modify = 0;
        Float value_backup = 0;
        int modify_backup = 0;
        void Backup() {
            value_backup = value;
            modify_backup = modify;
        }
        void Restore() {
            value = value_backup;
            modify = modify_backup;
        }
    };

    // MLTSampler Private Methods
    void EnsureReady(int index);

    // MLTSampler Private Data
    RNG rng;
    Float sigma;
    Float largeStepProb;
    std::vector<PrimarySample> samples;
    int iteration = 0;
    bool largeStep = true;
    int lastLargeStep = 0;
    int streamIndex;
    int streamCount;
    int sampleIndex;
};

// MLT Declarations
class MLTIntegrator : public Integrator {
  public:
    // MLTIntegrator Public Methods
    MLTIntegrator(std::shared_ptr<const Camera> camera, int maxdepth,
                  int nBootstrap, int nChains, int64_t mutationsPerPixel,
                  Float sigma, Float largeStepProb)
        : camera(camera),
          maxdepth(maxdepth),
          nBootstrap(nBootstrap),
          nChains(nChains),
          mutationsPerPixel(mutationsPerPixel),
          sigma(sigma),
          largeStepProb(largeStepProb){};
    void Render(const Scene &scene);
    Spectrum EvaluateSample(const Scene &scene, MemoryArena &arena,
                            MLTSampler &sampler, int k, Point2f *samplePos);

  private:
    // MLTIntegrator Private Data
    std::shared_ptr<const Camera> camera;
    int maxdepth;
    int nBootstrap;
    int nChains;
    int64_t mutationsPerPixel;
    Float sigma;
    Float largeStepProb;
    std::unique_ptr<Distribution1D> lightDistr;
};

MLTIntegrator *CreateMLTIntegrator(const ParamSet &params,
                                   std::shared_ptr<const Camera> camera);

#endif  // PBRT_INTEGRATORS_MLT_H
