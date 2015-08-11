
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

// integrators/volpath.cpp*
#include "integrators/volpath.h"
#include "scene.h"
#include "interaction.h"
#include "paramset.h"
#include "bssrdf.h"
#include "stats.h"

// VolPathIntegrator Method Definitions
Spectrum VolPathIntegrator::Li(const RayDifferential &r, const Scene &scene,
                               Sampler &sampler, MemoryArena &arena,
                               int depth) const {
    ProfilePhase p(Prof::SamplerIntegratorLi);
    Spectrum L(0.f), alpha(1.f);
    RayDifferential ray(r);
    bool specularBounce = false;
    for (int bounces = 0;; ++bounces) {
        // Store intersection into _isect_
        SurfaceInteraction isect;
        bool foundIntersection = scene.Intersect(ray, &isect);

        // Sample the participating medium, if present
        MediumInteraction mi;
        if (ray.medium) alpha *= ray.medium->Sample(ray, sampler, arena, &mi);
        if (alpha.IsBlack()) break;

        // Handle an interaction with a medium or a surface
        if (mi.IsValid()) {
            // Handle medium scattering case
            Vector3f wo = -ray.d, wi;
            L += alpha * UniformSampleOneLight(mi, scene, sampler, arena, true);
            Point2f phaseSample = sampler.Get2D();
            mi.phase->Sample_p(wo, &wi, phaseSample);
            ray = mi.SpawnRay(wi);
        } else {
            // Handle surface scattering case

            // Possibly add emitted light and terminate
            if (bounces == 0 || specularBounce) {
                // Add emitted light at path vertex or from the environment
                if (foundIntersection)
                    L += alpha * isect.Le(-ray.d);
                else
                    for (const auto &light : scene.lights)
                        L += alpha * light->Le(ray);
            }
            if (!foundIntersection || bounces >= maxDepth) break;

            // Compute scattering functions and skip over medium boundaries
            isect.ComputeScatteringFunctions(ray, arena, true);
            if (!isect.bsdf) {
                ray = isect.SpawnRay(ray.d);
                bounces--;
                continue;
            }

            // Sample illumination from lights to find attenuated path
            // contribution
            L += alpha *
                 UniformSampleOneLight(isect, scene, sampler, arena, true);

            // Sample BSDF to get new path direction
            Vector3f wo = -ray.d, wi;
            Float pdf;
            BxDFType flags;
            Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                                              BSDF_ALL, &flags);
            if (f.IsBlack() || pdf == 0.f) break;
            alpha *= f * AbsDot(wi, isect.shading.n) / pdf;
            Assert(std::isinf(alpha.y()) == false);
            specularBounce = (flags & BSDF_SPECULAR) != 0;
            ray = isect.SpawnRay(wi);

            // Account for attenuated subsurface scattering, if applicable
            if (isect.bssrdf && (flags & BSDF_TRANSMISSION)) {
                // Importance sample the BSSRDF
                SurfaceInteraction pi;
                Spectrum S = isect.bssrdf->Sample_S(
                    scene, sampler.Get1D(), sampler.Get2D(), arena, &pi, &pdf);
#ifndef NDEBUG
                Assert(std::isinf(alpha.y()) == false);
#endif
                if (S.IsBlack() || pdf == 0) break;
                alpha *= S / pdf;

                // Account for the attenuated direct subsurface scattering
                // component
                L += alpha *
                     UniformSampleOneLight(pi, scene, sampler, arena, true);

                // Account for the indirect subsurface scattering component
                Spectrum f = pi.bsdf->Sample_f(pi.wo, &wi, sampler.Get2D(),
                                               &pdf, BSDF_ALL, &flags);
                if (f.IsBlack() || pdf == 0.f) break;
                alpha *= f * AbsDot(wi, pi.shading.n) / pdf;
#ifndef NDEBUG
                Assert(std::isinf(alpha.y()) == false);
#endif
                specularBounce = (flags & BSDF_SPECULAR) != 0;
                ray = pi.SpawnRay(wi);
            }
        }

        // Possibly terminate the path
        if (bounces > 3) {
            Float continueProbability = std::min((Float).5, alpha.y());
            if (sampler.Get1D() > continueProbability) break;
            alpha /= continueProbability;
            Assert(std::isinf(alpha.y()) == false);
        }
    }
    return L;
}

VolPathIntegrator *CreateVolPathIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    return new VolPathIntegrator(maxDepth, camera, sampler);
}
