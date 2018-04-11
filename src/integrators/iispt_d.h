
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
#include "film/distancefilm.h"
#include "film/normalfilm.h"
#include "lightdistrib.h"
#include "film/intensityfilm.h"

namespace pbrt {

const std::string IISPT_REFERENCE_DIRECTORY = std::string("out/");
const std::string IISPT_REFERENCE_TRAIN_INFO = std::string("train.json");
const int IISPT_REFERENCE_PATH_MAX_DEPTH = 24;

// IISPTdIntegrator Declarations
class IISPTdIntegrator : public SamplerIntegrator {

private:
  // IISPTdIntegrator Private Data
  std::shared_ptr<Camera> camera;
  std::shared_ptr<Sampler> sampler;
  Bounds2i pixelBounds;
  const int maxDepth;
  std::vector<int> nLightSamples;
  std::shared_ptr<DistanceFilm> distance_film;
  std::shared_ptr<NormalFilm> normal_film;

  const Float rrThreshold = 0.5;
  const std::string lightSampleStrategy = std::string("spatial");
  std::unique_ptr<LightDistribution> lightDistribution;

public:

    // IISPTdIntegrator Public Methods

    // Constructor
    IISPTdIntegrator(int maxDepth,
                             std::shared_ptr<Camera> camera,
                             std::shared_ptr<Sampler> sampler,
                             const Bounds2i &pixelBounds) :
        SamplerIntegrator(camera, sampler, pixelBounds),
        camera(camera),
        sampler(sampler),
        pixelBounds(pixelBounds),
        maxDepth(maxDepth)
    {
        distance_film = std::shared_ptr<DistanceFilm>(
            new DistanceFilm(
                pixelBounds.pMax.x,
                pixelBounds.pMax.y
            )
        );

        normal_film = std::make_shared<NormalFilm>(
                    pixelBounds.pMax.x,
                    pixelBounds.pMax.y
                    );
    }

    Spectrum Li(const RayDifferential &ray,
                                          const Scene &scene, Sampler &sampler,
                                          MemoryArena &arena, int depth, int x, int y);

    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth) const {
        std::cerr << "Call on iispt_d: Li's default implementation, empty." << std::endl;
        exit(1);
    }

    void Preprocess(const Scene &scene);

    void RenderView(const Scene &scene, std::shared_ptr<Camera> camera);

    void save_reference(std::shared_ptr<Camera> camera,
                                          std::string distance_filename,
                                          std::string normal_filename
                                          );

    std::shared_ptr<IntensityFilm> get_intensity_film(std::shared_ptr<Camera> camera);

    std::shared_ptr<NormalFilm> get_normal_film();

    std::shared_ptr<DistanceFilm> get_distance_film();

};

std::shared_ptr<IISPTdIntegrator> CreateIISPTdIntegrator(
    std::shared_ptr<Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_IISPT_D_H
