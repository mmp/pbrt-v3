
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
#include "cameras/hemispheric.h"

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

void IISPTIntegrator::Preprocess(const Scene &scene) {

    LOG(INFO) << "IISPTIntegrator preprocess";

}

Spectrum IISPTIntegrator::SpecularTransmit(
        const RayDifferential &ray,
        const SurfaceInteraction &isect,
        const Scene &scene,
        Sampler &sampler,
        MemoryArena &arena,
        int depth,
        Point2i pixel
        ) const {
    Vector3f wo = isect.wo, wi;
    Float pdf;
    const Point3f &p = isect.p;
    const Normal3f &ns = isect.shading.n;
    const BSDF &bsdf = *isect.bsdf;
    Spectrum f = bsdf.Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                               BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
    Spectrum L = Spectrum(0.f);
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f) {
        // Compute ray differential _rd_ for specular transmission
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = p + isect.dpdx;
            rd.ryOrigin = p + isect.dpdy;

            Float eta = bsdf.eta;
            Vector3f w = -wo;
            if (Dot(wo, ns) < 0) eta = 1.f / eta;

            Normal3f dndx = isect.shading.dndu * isect.dudx +
                            isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy +
                            isect.shading.dndv * isect.dvdy;

            Vector3f dwodx = -ray.rxDirection - wo,
                     dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);

            Float mu = eta * Dot(w, ns) - Dot(wi, ns);
            Float dmudx =
                (eta - (eta * eta * Dot(w, ns)) / Dot(wi, ns)) * dDNdx;
            Float dmudy =
                (eta - (eta * eta * Dot(w, ns)) / Dot(wi, ns)) * dDNdy;

            rd.rxDirection =
                wi + eta * dwodx - Vector3f(mu * dndx + dmudx * ns);
            rd.ryDirection =
                wi + eta * dwody - Vector3f(mu * dndy + dmudy * ns);
        }
        L = f * Li_direct(rd, scene, sampler, arena, depth + 1, pixel) * AbsDot(wi, ns) / pdf;
    }
    return L;
}

Spectrum IISPTIntegrator::SpecularReflect(
        const RayDifferential &ray,
        const SurfaceInteraction &isect,
        const Scene &scene,
        Sampler &sampler,
        MemoryArena &arena,
        int depth,
        Point2i pixel
        ) const {
    // Compute specular reflection direction _wi_ and BSDF value
    Vector3f wo = isect.wo, wi;
    Float pdf;
    BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
    Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, type);

    // Return contribution of specular reflection
    const Normal3f &ns = isect.shading.n;
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f) {
        // Compute ray differential _rd_ for specular reflection
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = isect.p + isect.dpdx;
            rd.ryOrigin = isect.p + isect.dpdy;
            // Compute differential reflected directions
            Normal3f dndx = isect.shading.dndu * isect.dudx +
                            isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy +
                            isect.shading.dndv * isect.dvdy;
            Vector3f dwodx = -ray.rxDirection - wo,
                     dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);
            rd.rxDirection =
                wi - dwodx + 2.f * Vector3f(Dot(wo, ns) * dndx + dDNdx * ns);
            rd.ryDirection =
                wi - dwody + 2.f * Vector3f(Dot(wo, ns) * dndy + dDNdy * ns);
        }
        return f * Li_direct(rd, scene, sampler, arena, depth + 1, pixel) * AbsDot(wi, ns) /
               pdf;
    } else
        return Spectrum(0.f);
}

static bool is_debug_pixel(Point2i pixel) {
//    return (pixel.x == 365 && pixel.y == 500) ||
//            (pixel.x == 450 && pixel.y == 120) ||
//            (pixel.x == 464 && pixel.y == 614);

//    return (pixel.x == 1159 && pixel.y == 659) ||
//            (pixel.x == 179 && pixel.y == 159);

//    return (pixel.x == 610 && pixel.y == 560) ||
//            (pixel.x == 600 && pixel.y == 600);

    return (pixel.x % 100 == 0) && (pixel.y % 100 == 0);
}

/*
Reimplementing pixel estimation using Direct Illumination and rendered hemisphere:

in integrator.cpp: UniformSampleAllLights
    for each light, calls EstimateDirect
    Instead of using the number of samples for the light source,
    I do the for loop for every pixel in my hemisphere

in integrator.cpp: EstimateDirect
    here is Multiple Importance Sampling
    light.Sample_Li is sampling the illumination. I can replace this to sample from my hemisphere.
*/



// Render =====================================================================
void IISPTIntegrator::Render(const Scene &scene) {

    Preprocess(scene);

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

// Estimate direct ============================================================
static Spectrum IISPTEstimateDirect(
        const Interaction &it,
        int hem_x,
        int hem_y,
        HemisphericCamera* auxCamera
        ) {

    bool specular = false; // Default value

    BxDFType bsdfFlags =
        specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Spectrum Ld(0.f);

    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 1.0 / 3.14;
    Float scatteringPdf = 0;
    VisibilityTester visibility;

    // TODO replace Sample_Li with custom code to sample from hemisphere instead
    // Writes into wi the vector towards the light source. Derived from hem_x and hem_y
    // For the hemisphere, lightPdf would be a constant (probably 1/(2pi))
    // We don't need to have a visibility object
    // Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
    Spectrum Li = auxCamera->getLightSample(hem_x, hem_y, &wi);

    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample

        Spectrum f;

        if (it.IsSurfaceInteraction()) {

            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;

            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) * AbsDot(wi, isect.shading.n);

            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);

        } else {

            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;

        }

        if (!f.IsBlack()) {
            // Compute effect of visibility for light source sample
            // Always unoccluded visibility using hemispherical map

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                Ld += f * Li / lightPdf;
            }
        }
    }

    // Skipping sampling BSDF with multiple importance sampling
    // because we gather all information from lights (hemisphere)

    return Ld;

}

// Sample hemisphere ==========================================================
static Spectrum IISPTSampleHemisphere(
        const Interaction &it,
        const Scene &scene,
        MemoryArena &arena,
        Sampler &sampler,
        HemisphericCamera* auxCamera
        ) {
    ProfilePhase p(Prof::DirectLighting);
    Spectrum L(0.f);

    // Loop for every pixel in the hemisphere
    for (int hemi_x = 0; hemi_x < IISPT_D_SIZE_X; hemi_x++) {
        for (int hemi_y = 0; hemi_y < IISPT_D_SIZE_Y; hemi_y++) {
            L += IISPTEstimateDirect(it, hemi_x, hemi_y, auxCamera);
        }
    }

    int n_samples = IISPT_D_SIZE_X * IISPT_D_SIZE_Y;

    return L / n_samples;
}

// Disabled version ===========================================================
Spectrum IISPTIntegrator::Li(const RayDifferential &r,
                             const Scene &scene,
                             Sampler &sampler,
                             MemoryArena &arena,
                             int depth
                             ) const {
    fprintf(stderr, "ERROR IISPTIntegrator::Li, overridden version, is not defined in this debug version.");
    exit(1);
}

// Direct version used by specular and transmit ===============================
Spectrum IISPTIntegrator::Li_direct(
        const RayDifferential &ray,
        const Scene &scene,
        Sampler &sampler,
        MemoryArena &arena,
        int depth,
        Point2i pixel
        ) const {
    Spectrum L (0.f);
    return L;
}

// New version ================================================================
Spectrum IISPTIntegrator::Li(const RayDifferential &ray,
                             const Scene &scene,
                             Sampler &sampler,
                             MemoryArena &arena,
                             int depth,
                             Point2i pixel
                             ) const {

    ProfilePhase p(Prof::SamplerIntegratorLi);
    Spectrum L (0.f);

    // Find closest ray intersection or return background radiance
    SurfaceInteraction isect;
    if (!scene.Intersect(ray, &isect)) {
        for (const auto &light : scene.lights) {
            L += light->Le(ray);
            return L;
        }
    }

    // Compute the hemisphere -------------------------------------------------

    // Invert normal if the surface's normal was pointing inwards
    Normal3f surfNormal = isect.n;
    if (Dot(Vector3f(isect.n.x, isect.n.y, isect.n.z), Vector3f(ray.d.x, ray.d.y, ray.d.z)) > 0.0) {
        surfNormal = Normal3f(-isect.n.x, -isect.n.y, -isect.n.z);
    }

    // auxRay is centered at the intersection point, and points towards the intersection
    // surface normal
    Ray auxRay = isect.SpawnRay(Vector3f(surfNormal));

    // testCamera is used for the hemispheric rendering
    std::shared_ptr<HemisphericCamera> auxCamera (
                CreateHemisphericCamera(
                        IISPT_D_SIZE_X, IISPT_D_SIZE_Y, dcamera->medium,
                        auxRay.o, Point3f(auxRay.d.x, auxRay.d.y, auxRay.d.z),
                        pixel
                    )
                );

    // Start rendering the hemispherical view
    this->dintegrator->RenderView(scene, auxCamera);

    // Use the hemispherical view to obtain illumination ----------------------

    // Compute scattering functions for surface interaction
    isect.ComputeScatteringFunctions(ray, arena);
    if (!isect.bsdf) {
        return Li(isect.SpawnRay(ray.d), scene, sampler, arena, depth);
    }

    // wo should be the vector towards camera, from intersection
    Vector3f wo = isect.wo;
    Float woLength = Dot(wo, wo);
    if (woLength == 0) {
        fprintf(stderr, "Detected a 0 length wo");
        exit(1);
    }

    // Compute emitted light if ray hit an area light source
    L += isect.Le(wo);
    if (scene.lights.size() > 0) {
        // Compute direct lighting using hemisphere information TODO
        L += IISPTSampleHemisphere(isect, scene, arena, sampler, auxCamera.get());
    }

//    if (depth + 1 < maxDepth) {
//        // Trace rays for specular reflection and refraction
//        L += SpecularReflect(ray, isect, scene, sampler, arena, depth, pixel);
//        L += SpecularTransmit(ray, isect, scene, sampler, arena, depth, pixel);
//    }

    return L;

}

// Creator ====================================================================
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