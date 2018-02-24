
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

#ifndef PBRT_INTEGRATORS_IISPT_D_H
#define PBRT_INTEGRATORS_IISPT_D_H

// integrators/iispt_d.h*
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"
#include "integrators/directlighting.h"

namespace pbrt {

// Configurable size of auxiliary films
const int IISPT_D_SIZE_X = 16;
const int IISPT_D_SIZE_Y = 16;

// IISPTdIntegrator Declarations
class IISPTdIntegrator {
public:

    // IISPTdIntegrator Public Methods

    // Constructor
    IISPTdIntegrator(LightStrategy strategy, int maxDepth,
                             std::shared_ptr<Camera> camera,
                             std::shared_ptr<Sampler> sampler,
                             const Bounds2i &pixelBounds) :
        camera(camera),
        sampler(sampler),
        pixelBounds(pixelBounds),
        strategy(strategy),
        maxDepth(maxDepth)
    {

    }

    virtual Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth);

    void Preprocess(const Scene &scene);

    Spectrum SpecularReflect(
        const RayDifferential &ray, const SurfaceInteraction &isect,
        const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth);

    Spectrum SpecularTransmit(
        const RayDifferential &ray, const SurfaceInteraction &isect,
        const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth);

    void RenderView(const Scene &scene, std::shared_ptr<Camera> camera);

  private:
    // IISPTdIntegrator Private Data
    std::shared_ptr<Camera> camera;
    std::shared_ptr<Sampler> sampler;
    Bounds2i pixelBounds;
    const LightStrategy strategy;
    const int maxDepth;
    std::vector<int> nLightSamples;
};

std::shared_ptr<IISPTdIntegrator> CreateIISPTdIntegrator(
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_IISPT_D_H
