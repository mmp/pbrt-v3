
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

// integrators/path.cpp*
#include "integrators/path.h"
#include "scene.h"
#include "interaction.h"
#include "paramset.h"

// PathIntegrator Method Definitions
Spectrum PathIntegrator::Li(const RayDifferential &r, const Scene &scene,
                            Sampler &sampler, MemoryArena &arena) const {
    Spectrum L(0.f);
    // Declare common path integration variables
    RayDifferential ray(r);
    Spectrum pathThroughput = Spectrum(1.f);
    bool specularBounce = false;
    for (int bounces = 0;; ++bounces) {
        // Store intersection into _isect_
        SurfaceInteraction isect;
        bool foundIntersection = scene.Intersect(ray, &isect);

        // Possibly add emitted light and terminate
        if (bounces == 0 || specularBounce) {
            // Add emitted light at path vertex or from the environment
            if (foundIntersection) {
                L += pathThroughput * isect.Le(-ray.d);
            } else {
                for (const auto &light : scene.lights)
                    L += pathThroughput * light->Le(ray);
            }
        }
        if (!foundIntersection || bounces >= maxDepth) break;

        // Compute scattering functions and skip over medium boundaries
        isect.ComputeScatteringFunctions(ray, arena, true);
        if (!isect.bsdf) {
            ray = isect.SpawnRay(ray.d);
            bounces--;
            continue;
        }

        // Sample illumination from lights to find path contribution
        L += pathThroughput *
             UniformSampleOneLight(isect, scene, sampler, arena);

        // Sample BSDF to get new path direction
        Vector3f wo = -ray.d, wi;
        Float pdf;
        BxDFType flags;
        Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                                          BSDF_ALL, &flags);

        if (f.IsBlack() || pdf == 0.f) break;
        pathThroughput *= f * AbsDot(wi, isect.shading.n) / pdf;
#ifndef NDEBUG
        Assert(std::isinf(pathThroughput.y()) == false);
#endif
        specularBounce = (flags & BSDF_SPECULAR) != 0;
        ray = isect.SpawnRay(wi);

        // Account for subsurface scattering, if applicable
        if (isect.bssrdf && (flags & BSDF_TRANSMISSION)) {
            // Importance sample the BSSRDF
            BSSRDFSample bssrdfSample;
            bssrdfSample.uDiscrete = sampler.Get1D();
            bssrdfSample.pos = sampler.Get2D();
            SurfaceInteraction isect_out = isect;
            pathThroughput *= isect.bssrdf->Sample_f(
                isect_out, scene, ray.time, bssrdfSample, arena, &isect, &pdf);
#ifndef NDEBUG
            Assert(std::isinf(pathThroughput.y()) == false);
#endif
            if (pathThroughput.IsBlack()) break;

            // Account for the direct subsurface scattering component
            isect.wo = Vector3f(isect.shading.n);

            // Sample illumination from lights to find path contribution
            L += pathThroughput *
                 UniformSampleOneLight(isect, scene, sampler, arena);

            // Account for the indirect subsurface scattering component
            Spectrum f = isect.bsdf->Sample_f(isect.wo, &wi, sampler.Get2D(),
                                              &pdf, BSDF_ALL, &flags);
            if (f.IsBlack() || pdf == 0.f) break;
            pathThroughput *= f * AbsDot(wi, isect.shading.n) / pdf;
#ifndef NDEBUG
            Assert(std::isinf(pathThroughput.y()) == false);
#endif
            specularBounce = (flags & BSDF_SPECULAR) != 0;
            ray = isect.SpawnRay(wi);
        }

        // Possibly terminate the path
        if (bounces > 3) {
            Float continueProbability = std::min((Float).5, pathThroughput.y());
            if (sampler.Get1D() > continueProbability) break;
            pathThroughput /= continueProbability;
            Assert(std::isinf(pathThroughput.y()) == false);
        }
    }
    return L;
}

PathIntegrator *CreatePathIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    return new PathIntegrator(maxDepth, sampler, camera);
}
