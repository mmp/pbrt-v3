
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

#ifndef PBRT_INTEGRATORS_IISPT_H
#define PBRT_INTEGRATORS_IISPT_H

#include <signal.h>

// integrators/iispt.h*
#include "pbrt.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "integrators/iispt_d.h"

namespace pbrt {

const int IISPT_NORMALIZATION_ESTIMATION_SAMPLES = 10000;

// IISPTIntegrator Declarations
class IISPTIntegrator : public SamplerIntegrator {
public:
    // IISPTIntegrator Public Methods

    // Constructor ------------------------------------------------------------
    IISPTIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
                   std::shared_ptr<Sampler> sampler,
                   const Bounds2i &pixelBounds,
                   std::shared_ptr<Camera> dcamera,
                   Float rrThreshold = 1,
                   const std::string &lightSampleStrategy = "spatial"
    );

    // Public methods ---------------------------------------------------------

    void Preprocess(const Scene &scene);

    Spectrum Li(const RayDifferential &r,
                 const Scene &scene,
                 Sampler &sampler,
                 MemoryArena &arena,
                 int depth
                 ) const;

    Spectrum Li(const RayDifferential &r,
                 const Scene &scene,
                 Sampler &sampler,
                 MemoryArena &arena,
                 int depth,
                 Point2i pixel
                 ) const;

    void Render(const Scene &scene);

    void render_normal_2(const Scene &scene);

    void render_reference(const Scene &scene);

    Float get_normalization_intensity() {
        if (max_intensity == -1) {
            std::cerr << "iispt.h: max_intensity was not populated";
            raise(SIGKILL);
        }
        return max_intensity;
    }

    Float get_normalization_distance() {
        if (max_distance == -1) {
            std::cerr << "iispt.h: max_distance was not populated";
            raise(SIGKILL);
        }
        return max_distance;
    }

private:
    // IISPTIntegrator Private Data -------------------------------------------
    const int maxDepth;
    const Float rrThreshold;
    const std::string lightSampleStrategy;
    std::unique_ptr<LightDistribution> lightDistribution;
    std::shared_ptr<Sampler> sampler;

    std::shared_ptr<Camera> dcamera;
    std::shared_ptr<IISPTdIntegrator> dintegrator;

    Float max_intensity = -1;
    Float max_distance = -1;

    // Private methods --------------------------------------------------------

    Spectrum SpecularTransmit(
            const RayDifferential &ray,
            const SurfaceInteraction &isect,
            const Scene &scene,
            Sampler &sampler,
            MemoryArena &arena,
            int depth,
            Point2i pixel
            ) const;

    Spectrum SpecularReflect(
            const RayDifferential &ray,
            const SurfaceInteraction &isect,
            const Scene &scene,
            Sampler &sampler,
            MemoryArena &arena,
            int depth,
            Point2i pixel
            ) const;

    Spectrum  Li_direct(
            const RayDifferential &ray,
            const Scene &scene,
            Sampler &sampler,
            MemoryArena &arena,
            int depth,
            Point2i pixel
            ) const;

    void estimate_normalization(const Scene &scene);

    void estimate_normalization_values(
            const Scene &scene,
            Vector2i sample_extent
            );

    void write_info_file(std::string out_filename);

};

IISPTIntegrator *CreateIISPTIntegrator(
        const ParamSet &params,
        std::shared_ptr<Sampler> sampler,
        std::shared_ptr<const Camera> camera,
        std::shared_ptr<Camera> dcamera
);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_IISPT_H
