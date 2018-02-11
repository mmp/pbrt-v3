
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

// integrators/iispt.cpp*
#include "integrators/iispt.h"
#include "integrators/iispt_d.h"
#include "bssrdf.h"
#include "camera.h"
#include "film.h"
#include "interaction.h"
#include "paramset.h"
#include "scene.h"
#include "stats.h"
#include "progressreporter.h"
#include "cameras/perspective.h"

#include <cstdlib>

namespace pbrt {

STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIBUTION("Integrator/Path length", pathLength);

// IISPTIntegrator Method Definitions

// Constructor
IISPTIntegrator::IISPTIntegrator(int maxDepth,
                               std::shared_ptr<const Camera> camera,
                               std::shared_ptr<Sampler> sampler,
                               const Bounds2i &pixelBounds,
                               std::shared_ptr<Sampler> dsampler,
                               std::shared_ptr<Camera> dcamera,
                               Float rrThreshold,
                               const std::string &lightSampleStrategy
) :
    SamplerIntegrator(camera, sampler, pixelBounds),
    sampler(sampler),
    maxDepth(maxDepth),
    rrThreshold(rrThreshold),
    lightSampleStrategy(lightSampleStrategy),
    dsampler(dsampler),
    dcamera(dcamera)
{

}

void IISPTIntegrator::Preprocess(const Scene &scene, Sampler &sampler) {
    lightDistribution =
        CreateLightSampleDistribution(lightSampleStrategy, scene);
}

static bool is_debug_pixel(Point2i pixel) {
//    return (pixel.x == 365 && pixel.y == 500) ||
//            (pixel.x == 450 && pixel.y == 120) ||
//            (pixel.x == 464 && pixel.y == 614);
    return (pixel.x == 1159 && pixel.y == 659) ||
            (pixel.x == 179 && pixel.y == 159);
}

void IISPTIntegrator::Render(const Scene &scene) {

    Preprocess(scene, *sampler);

    // Render image tiles in parallel

    // Create the auxiliary integrator for intersection-view
    this->dintegrator = std::shared_ptr<IISPTdIntegrator>(CreateIISPTdIntegrator(
        dsampler, dcamera));
    // Preprocess on auxiliary integrator
    dintegrator->Preprocess(scene);

    // Compute number of tiles, _nTiles_, to use for parallel rendering
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    const int tileSize = 16;
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
                   (sampleExtent.y + tileSize - 1) / tileSize);
    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
    {
        ParallelFor2D([&](Point2i tile) {

            // Render section of image corresponding to _tile_

            // Allocate _MemoryArena_ for tile
            MemoryArena arena;

            // Get sampler instance for tile
            int seed = tile.y * nTiles.x + tile.x;
            std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);

            // Compute sample bounds for tile
            int x0 = sampleBounds.pMin.x + tile.x * tileSize;
            int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + tile.y * tileSize;
            int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
            Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
            // LOG(INFO) << "Starting image tile " << tileBounds;

            // Get _FilmTile_ for tile
            std::unique_ptr<FilmTile> filmTile =
                camera->film->GetFilmTile(tileBounds);

            // Loop over pixels in tile to render them
            for (Point2i pixel : tileBounds) {

                // TODO remove debug stuff here
                if (is_debug_pixel(pixel)) {
                    LOG(INFO) << "IISPTIntegrator::Render: Starting pixel ["<< pixel.x <<"] ["<< pixel.y <<"]";
                }

                {
                    ProfilePhase pp(Prof::StartPixel);
                    tileSampler->StartPixel(pixel);
                }

                // Do this check after the StartPixel() call; this keeps
                // the usage of RNG values from (most) Samplers that use
                // RNGs consistent, which improves reproducability /
                // debugging.
                if (!InsideExclusive(pixel, pixelBounds))
                    continue;

                do {
                    // Initialize _CameraSample_ for current sample
                    CameraSample cameraSample =
                        tileSampler->GetCameraSample(pixel);

                    // Generate camera ray for current sample
                    RayDifferential ray;
                    Float rayWeight =
                        camera->GenerateRayDifferential(cameraSample, &ray);
                    ray.ScaleDifferentials(
                        1 / std::sqrt((Float)tileSampler->samplesPerPixel));
                    ++nCameraRays;

                    // Evaluate radiance along camera ray
                    Spectrum L(0.f);
                    // NOTE Passing a depth=0 here
                    if (rayWeight > 0) {
                        L = Li(ray, scene, *tileSampler, arena, 0, pixel);
                    }

                    // Issue warning if unexpected radiance value returned
                    if (L.HasNaNs()) {
                        LOG(ERROR) << StringPrintf(
                            "Not-a-number radiance value returned "
                            "for pixel (%d, %d), sample %d. Setting to black.",
                            pixel.x, pixel.y,
                            (int)tileSampler->CurrentSampleNumber());
                        L = Spectrum(0.f);
                    } else if (L.y() < -1e-5) {
                        LOG(ERROR) << StringPrintf(
                            "Negative luminance value, %f, returned "
                            "for pixel (%d, %d), sample %d. Setting to black.",
                            L.y(), pixel.x, pixel.y,
                            (int)tileSampler->CurrentSampleNumber());
                        L = Spectrum(0.f);
                    } else if (std::isinf(L.y())) {
                          LOG(ERROR) << StringPrintf(
                            "Infinite luminance value returned "
                            "for pixel (%d, %d), sample %d. Setting to black.",
                            pixel.x, pixel.y,
                            (int)tileSampler->CurrentSampleNumber());
                        L = Spectrum(0.f);
                    }
                    VLOG(1) << "Camera sample: " << cameraSample << " -> ray: " <<
                        ray << " -> L = " << L;

                    // Add camera ray's contribution to image
                    filmTile->AddSample(cameraSample.pFilm, L, rayWeight);

                    // Free _MemoryArena_ memory from computing image sample
                    // value
                    arena.Reset();
                } while (tileSampler->StartNextSample());
            }
            // LOG(INFO) << "Finished image tile " << tileBounds;

            // Merge image tile into _Film_
            camera->film->MergeFilmTile(std::move(filmTile));
            reporter.Update();
        }, nTiles);
        reporter.Done();
    }
    LOG(INFO) << "Rendering finished";

    // Save final image after rendering
    camera->film->WriteImage();
}

Spectrum IISPTIntegrator::Li(const RayDifferential &r,
                             const Scene &scene,
                             Sampler &sampler,
                             MemoryArena &arena,
                             int depth
                             ) const {
    LOG(INFO) << "ERROR IISPTIntegrator::Li, overridden version, is not defined in this debug version.";
    exit(1);
}

Spectrum IISPTIntegrator::Li(const RayDifferential &r,
                             const Scene &scene,
                             Sampler &sampler,
                             MemoryArena &arena,
                             int depth,
                             Point2i pixel
                             ) const {
    ProfilePhase p(Prof::SamplerIntegratorLi);
    Spectrum L(0.f), beta(1.f);
    RayDifferential ray(r);
    bool specularBounce = false;
    int bounces;
    // Added after book publication: etaScale tracks the accumulated effect
    // of radiance scaling due to rays passing through refractive
    // boundaries (see the derivation on p. 527 of the third edition). We
    // track this value in order to remove it from beta when we apply
    // Russian roulette; this is worthwhile, since it lets us sometimes
    // avoid terminating refracted rays that are about to be refracted back
    // out of a medium and thus have their beta value increased.
    Float etaScale = 1;

    for (bounces = 0;; ++bounces) {
        // Find next path vertex and accumulate contribution
        VLOG(2) << "Path tracer bounce " << bounces << ", current L = " << L
                << ", beta = " << beta;

        // Intersect _ray_ with scene and store intersection in _isect_
        // SurfaceInteraction defined in interaction.cpp
        SurfaceInteraction isect;
        bool foundIntersection = scene.Intersect(ray, &isect);

        // Possibly add emitted light at intersection
        if (bounces == 0 || specularBounce) {
            // Add emitted light at path vertex or from the environment
            if (foundIntersection) {
                L += beta * isect.Le(-ray.d);
                VLOG(2) << "Added Le -> L = " << L;
            } else {
                for (const auto &light : scene.infiniteLights)
                    L += beta * light->Le(ray);
                VLOG(2) << "Added infinite area lights -> L = " << L;
            }
        }

        // Terminate path if ray escaped or _maxDepth_ was reached
        if (!foundIntersection || bounces >= maxDepth) break;

        // NOTE We have a valid intersection
        // Compute from this point the hemispherical map

        // Compute scattering functions and skip over medium boundaries
        isect.ComputeScatteringFunctions(ray, arena, true);
        if (!isect.bsdf) {
            VLOG(2) << "Skipping intersection due to null bsdf";
            ray = isect.SpawnRay(ray.d);
            bounces--;
            continue;
        }

        const Distribution1D *distrib = lightDistribution->Lookup(isect.p);

        // Sample illumination from lights to find path contribution.
        // (But skip this for perfectly specular BSDFs.)
        if (isect.bsdf->NumComponents(BxDFType(BSDF_ALL & ~BSDF_SPECULAR)) >
            0) {
            ++totalPaths;
            Spectrum Ld = beta * UniformSampleOneLight(isect, scene, arena,
                                                       sampler, false, distrib);
            VLOG(2) << "Sampled direct lighting Ld = " << Ld;
            if (Ld.IsBlack()) ++zeroRadiancePaths;
            CHECK_GE(Ld.y(), 0.f);
            L += Ld;
        }

        // Sample BSDF to get new path direction
        // wo = -ray.d
        // wi = (new)
        Vector3f wo = -ray.d, wi;
        Float pdf;
        BxDFType flags;
        // NOTE this is where importance sampling should go
        // Spectrum is a typedef for RGBSpectrum
        // Look at reflection.cpp:BxDF::Sample_f wi and pdf are updated
        // Look at sampling.h:CosineSampleHemisphere, sample is generated from
        //     sampler.Get2D(), which is called Point2f u
        // Look at sampling.cpp:ConcentricSampleDisk
        //     it maps [0,1] to [-1,1]
        //     it returns a 2D point on a circle surface,
        //     from which CosineSampleHemisphere derives the z coordinate
        // Possible approach: modify the sampler.Get2D to use the importance
        //     map. Then modify the pdf dividing it by the pdf of the importance
        //     map.
        // In reflection.cpp:SpecularReflection::Sample_f The reflected ray
        //     is deterministic and pdf=1, in this case we should not use
        //     the importance map.
        // The importance map should be only used to sample a 'normal'
        //     BRDF.
        // Or we need to pre-compute a full map by multiplying the importance
        // map by the BRDF
        // It seems that wo is the outgoing ray, in the coordinate space
        //     of the surface: x and y on the surface, z point outwards from the surface
        //     in direction of the normal
        //     The normal is in isect.shading.n
        Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                                          BSDF_ALL, &flags);
        VLOG(2) << "Sampled BSDF, f = " << f << ", pdf = " << pdf;
        if (f.IsBlack() || pdf == 0.f) break;
        beta *= f * AbsDot(wi, isect.shading.n) / pdf;
        VLOG(2) << "Updated beta = " << beta;
        CHECK_GE(beta.y(), 0.f);
        DCHECK(!std::isinf(beta.y()));
        specularBounce = (flags & BSDF_SPECULAR) != 0;

        // TODO Remove debug statements
        if (is_debug_pixel(pixel)) {
            LOG(INFO) << "INFO for pixel ["<< pixel <<"]";
            LOG(INFO) << "Outgoing ray was o=["<< ray.o <<"] and d=["<< ray.d <<"]";
            LOG(INFO) << "Li: path tracing, bounce ["<< bounces <<"], intersection found ["<< foundIntersection <<"]";
            LOG(INFO) << "Intersection n is ["<< isect.n <<"]";
            LOG(INFO) << "Intersection p is ["<< isect.p <<"]";

            if (specularBounce) {
                LOG(INFO) << "This is a specular bounce";
            }

            // Create camera for auxiliary integrator
            if (foundIntersection && bounces == 0) {
                // Invert normal if the surface's normal was pointing inwards
                Normal3f surfNormal = isect.n;
                if (Dot(Vector3f(isect.n.x, isect.n.y, isect.n.z), Vector3f(ray.o.x, ray.o.y, ray.o.z)) < 0) {
                    surfNormal = Normal3f(-isect.n.x, -isect.n.y, -isect.n.z);
                }

                std::shared_ptr<Camera> testCamera (CreateIISPTPerspectiveCamera(
                            IISPT_D_SIZE_X, IISPT_D_SIZE_Y, dcamera->medium,
                            isect.p, Point3f(surfNormal.x, surfNormal.y, surfNormal.z)));
                LOG(INFO) << "Created auxiliary camera";
                this->dintegrator->RenderView(scene, testCamera);
            }

        }

        if ((flags & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION)) {
            Float eta = isect.bsdf->eta;
            // Update the term that tracks radiance scaling for refraction
            // depending on whether the ray is entering or leaving the
            // medium.
            etaScale *= (Dot(wo, isect.n) > 0) ? (eta * eta) : 1 / (eta * eta);
        }
        ray = isect.SpawnRay(wi);

        // Account for subsurface scattering, if applicable
        if (isect.bssrdf && (flags & BSDF_TRANSMISSION)) {
            // Importance sample the BSSRDF
            SurfaceInteraction pi;
            Spectrum S = isect.bssrdf->Sample_S(
                scene, sampler.Get1D(), sampler.Get2D(), arena, &pi, &pdf);
            DCHECK(!std::isinf(beta.y()));
            if (S.IsBlack() || pdf == 0) break;
            beta *= S / pdf;

            // Account for the direct subsurface scattering component
            L += beta * UniformSampleOneLight(pi, scene, arena, sampler, false,
                                              lightDistribution->Lookup(pi.p));

            // Account for the indirect subsurface scattering component
            Spectrum f = pi.bsdf->Sample_f(pi.wo, &wi, sampler.Get2D(), &pdf,
                                           BSDF_ALL, &flags);
            if (f.IsBlack() || pdf == 0) break;
            beta *= f * AbsDot(wi, pi.shading.n) / pdf;
            DCHECK(!std::isinf(beta.y()));
            specularBounce = (flags & BSDF_SPECULAR) != 0;
            ray = pi.SpawnRay(wi);
        }

        // Possibly terminate the path with Russian roulette.
        // Factor out radiance scaling due to refraction in rrBeta.
        Spectrum rrBeta = beta * etaScale;
        if (rrBeta.MaxComponentValue() < rrThreshold && bounces > 3) {
            Float q = std::max((Float).05, 1 - rrBeta.MaxComponentValue());
            if (sampler.Get1D() < q) break;
            beta /= 1 - q;
            DCHECK(!std::isinf(beta.y()));
        }
    }
    ReportValue(pathLength, bounces);
    return L;
}

IISPTIntegrator *CreateIISPTIntegrator(const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera,
    std::shared_ptr<Sampler> dsampler,
    std::shared_ptr<Camera> dcamera
) {
    LOG(INFO) << "CreateIISPTIntegrator: in";

    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    Float rrThreshold = params.FindOneFloat("rrthreshold", 1.);
    std::string lightStrategy =
        params.FindOneString("lightsamplestrategy", "spatial");
    return new IISPTIntegrator(maxDepth, camera, sampler, pixelBounds,
        dsampler, dcamera, rrThreshold, lightStrategy);
}

}  // namespace pbrt
