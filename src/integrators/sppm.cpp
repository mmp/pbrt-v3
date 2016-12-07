
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


// integrators/sppm.cpp*
#include "integrators/sppm.h"
#include "parallel.h"
#include "scene.h"
#include "imageio.h"
#include "spectrum.h"
#include "rng.h"
#include "paramset.h"
#include "progressreporter.h"
#include "interaction.h"
#include "sampling.h"
#include "samplers/halton.h"
#include "stats.h"

namespace pbrt {

STAT_RATIO(
    "Stochastic Progressive Photon Mapping/Visible points checked per photon "
    "intersection",
    visiblePointsChecked, totalPhotonSurfaceInteractions);
STAT_COUNTER("Stochastic Progressive Photon Mapping/Photon paths followed",
             photonPaths);
STAT_INT_DISTRIBUTION(
    "Stochastic Progressive Photon Mapping/Grid cells per visible point",
    gridCellsPerVisiblePoint);
STAT_MEMORY_COUNTER("Memory/SPPM Pixels", pixelMemoryBytes);
STAT_FLOAT_DISTRIBUTION("Memory/SPPM BSDF and Grid Memory", memoryArenaMB);

// SPPM Local Definitions
struct SPPMPixel {
    // SPPMPixel Public Methods
    SPPMPixel() : M(0) {}

    // SPPMPixel Public Data
    Float radius = 0;
    Spectrum Ld;
    struct VisiblePoint {
        // VisiblePoint Public Methods
        VisiblePoint() {}
        VisiblePoint(const Point3f &p, const Vector3f &wo, const BSDF *bsdf,
                     const Spectrum &beta)
            : p(p), wo(wo), bsdf(bsdf), beta(beta) {}
        Point3f p;
        Vector3f wo;
        const BSDF *bsdf = nullptr;
        Spectrum beta;
    } vp;
    AtomicFloat Phi[Spectrum::nSamples];
    std::atomic<int> M;
    Float N = 0;
    Spectrum tau;
};

struct SPPMPixelListNode {
    SPPMPixel *pixel;
    SPPMPixelListNode *next;
};

static bool ToGrid(const Point3f &p, const Bounds3f &bounds,
                   const int gridRes[3], Point3i *pi) {
    bool inBounds = true;
    Vector3f pg = bounds.Offset(p);
    for (int i = 0; i < 3; ++i) {
        (*pi)[i] = (int)(gridRes[i] * pg[i]);
        inBounds &= ((*pi)[i] >= 0 && (*pi)[i] < gridRes[i]);
        (*pi)[i] = Clamp((*pi)[i], 0, gridRes[i] - 1);
    }
    return inBounds;
}

inline unsigned int hash(const Point3i &p, int hashSize) {
    return (unsigned int)((p.x * 73856093) ^ (p.y * 19349663) ^
                          (p.z * 83492791)) %
           hashSize;
}

// SPPM Method Definitions
void SPPMIntegrator::Render(const Scene &scene) {
    ProfilePhase p(Prof::IntegratorRender);
    // Initialize _pixelBounds_ and _pixels_ array for SPPM
    Bounds2i pixelBounds = camera->film->croppedPixelBounds;
    int nPixels = pixelBounds.Area();
    std::unique_ptr<SPPMPixel[]> pixels(new SPPMPixel[nPixels]);
    for (int i = 0; i < nPixels; ++i) pixels[i].radius = initialSearchRadius;
    const Float invSqrtSPP = 1.f / std::sqrt(nIterations);
    pixelMemoryBytes = nPixels * sizeof(SPPMPixel);
    // Compute _lightDistr_ for sampling lights proportional to power
    std::unique_ptr<Distribution1D> lightDistr =
        ComputeLightPowerDistribution(scene);

    // Perform _nIterations_ of SPPM integration
    HaltonSampler sampler(nIterations, pixelBounds);

    // Compute number of tiles to use for SPPM camera pass
    Vector2i pixelExtent = pixelBounds.Diagonal();
    const int tileSize = 16;
    Point2i nTiles((pixelExtent.x + tileSize - 1) / tileSize,
                   (pixelExtent.y + tileSize - 1) / tileSize);
    ProgressReporter progress(2 * nIterations, "Rendering");
    for (int iter = 0; iter < nIterations; ++iter) {
        // Generate SPPM visible points
        std::vector<MemoryArena> perThreadArenas(MaxThreadIndex());
        {
            ProfilePhase _(Prof::SPPMCameraPass);
            ParallelFor2D([&](Point2i tile) {
                MemoryArena &arena = perThreadArenas[ThreadIndex];
                // Follow camera paths for _tile_ in image for SPPM
                int tileIndex = tile.y * nTiles.x + tile.x;
                std::unique_ptr<Sampler> tileSampler = sampler.Clone(tileIndex);

                // Compute _tileBounds_ for SPPM tile
                int x0 = pixelBounds.pMin.x + tile.x * tileSize;
                int x1 = std::min(x0 + tileSize, pixelBounds.pMax.x);
                int y0 = pixelBounds.pMin.y + tile.y * tileSize;
                int y1 = std::min(y0 + tileSize, pixelBounds.pMax.y);
                Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
                for (Point2i pPixel : tileBounds) {
                    // Prepare _tileSampler_ for _pPixel_
                    tileSampler->StartPixel(pPixel);
                    tileSampler->SetSampleNumber(iter);

                    // Generate camera ray for pixel for SPPM
                    CameraSample cameraSample =
                        tileSampler->GetCameraSample(pPixel);
                    RayDifferential ray;
                    Spectrum beta =
                        camera->GenerateRayDifferential(cameraSample, &ray);
                    ray.ScaleDifferentials(invSqrtSPP);

                    // Follow camera ray path until a visible point is created

                    // Get _SPPMPixel_ for _pPixel_
                    Point2i pPixelO = Point2i(pPixel - pixelBounds.pMin);
                    int pixelOffset =
                        pPixelO.x +
                        pPixelO.y * (pixelBounds.pMax.x - pixelBounds.pMin.x);
                    SPPMPixel &pixel = pixels[pixelOffset];
                    bool specularBounce = false;
                    for (int depth = 0; depth < maxDepth; ++depth) {
                        SurfaceInteraction isect;
                        ++totalPhotonSurfaceInteractions;
                        if (!scene.Intersect(ray, &isect)) {
                            // Accumulate light contributions for ray with no
                            // intersection
                            for (const auto &light : scene.lights)
                                pixel.Ld += beta * light->Le(ray);
                            break;
                        }
                        // Process SPPM camera ray intersection

                        // Compute BSDF at SPPM camera ray intersection
                        isect.ComputeScatteringFunctions(ray, arena, true);
                        if (!isect.bsdf) {
                            ray = isect.SpawnRay(ray.d);
                            --depth;
                            continue;
                        }
                        const BSDF &bsdf = *isect.bsdf;

                        // Accumulate direct illumination at SPPM camera ray
                        // intersection
                        Vector3f wo = -ray.d;
                        if (depth == 0 || specularBounce)
                            pixel.Ld += beta * isect.Le(wo);
                        pixel.Ld +=
                            beta * UniformSampleOneLight(isect, scene, arena,
                                                         *tileSampler);

                        // Possibly create visible point and end camera path
                        bool isDiffuse = bsdf.NumComponents(BxDFType(
                                             BSDF_DIFFUSE | BSDF_REFLECTION |
                                             BSDF_TRANSMISSION)) > 0;
                        bool isGlossy = bsdf.NumComponents(BxDFType(
                                            BSDF_GLOSSY | BSDF_REFLECTION |
                                            BSDF_TRANSMISSION)) > 0;
                        if (isDiffuse || (isGlossy && depth == maxDepth - 1)) {
                            pixel.vp = {isect.p, wo, &bsdf, beta};
                            break;
                        }

                        // Spawn ray from SPPM camera path vertex
                        if (depth < maxDepth - 1) {
                            Float pdf;
                            Vector3f wi;
                            BxDFType type;
                            Spectrum f =
                                bsdf.Sample_f(wo, &wi, tileSampler->Get2D(),
                                              &pdf, BSDF_ALL, &type);
                            if (pdf == 0. || f.IsBlack()) break;
                            specularBounce = (type & BSDF_SPECULAR) != 0;
                            beta *= f * AbsDot(wi, isect.shading.n) / pdf;
                            if (beta.y() < 0.25) {
                                Float continueProb =
                                    std::min((Float)1, beta.y());
                                if (tileSampler->Get1D() > continueProb) break;
                                beta /= continueProb;
                            }
                            ray = (RayDifferential)isect.SpawnRay(wi);
                        }
                    }
                }
            }, nTiles);
        }
        progress.Update();

        // Create grid of all SPPM visible points
        int gridRes[3];
        Bounds3f gridBounds;
        // Allocate grid for SPPM visible points
        const int hashSize = nPixels;
        std::vector<std::atomic<SPPMPixelListNode *>> grid(hashSize);
        {
            ProfilePhase _(Prof::SPPMGridConstruction);

            // Compute grid bounds for SPPM visible points
            Float maxRadius = 0.;
            for (int i = 0; i < nPixels; ++i) {
                const SPPMPixel &pixel = pixels[i];
                if (pixel.vp.beta.IsBlack()) continue;
                Bounds3f vpBound = Expand(Bounds3f(pixel.vp.p), pixel.radius);
                gridBounds = Union(gridBounds, vpBound);
                maxRadius = std::max(maxRadius, pixel.radius);
            }

            // Compute resolution of SPPM grid in each dimension
            Vector3f diag = gridBounds.Diagonal();
            Float maxDiag = MaxComponent(diag);
            int baseGridRes = (int)(maxDiag / maxRadius);
            CHECK_GT(baseGridRes, 0);
            for (int i = 0; i < 3; ++i)
                gridRes[i] = std::max((int)(baseGridRes * diag[i] / maxDiag), 1);

            // Add visible points to SPPM grid
            ParallelFor([&](int pixelIndex) {
                MemoryArena &arena = perThreadArenas[ThreadIndex];
                SPPMPixel &pixel = pixels[pixelIndex];
                if (!pixel.vp.beta.IsBlack()) {
                    // Add pixel's visible point to applicable grid cells
                    Float radius = pixel.radius;
                    Point3i pMin, pMax;
                    ToGrid(pixel.vp.p - Vector3f(radius, radius, radius),
                           gridBounds, gridRes, &pMin);
                    ToGrid(pixel.vp.p + Vector3f(radius, radius, radius),
                           gridBounds, gridRes, &pMax);
                    for (int z = pMin.z; z <= pMax.z; ++z)
                        for (int y = pMin.y; y <= pMax.y; ++y)
                            for (int x = pMin.x; x <= pMax.x; ++x) {
                                // Add visible point to grid cell $(x, y, z)$
                                int h = hash(Point3i(x, y, z), hashSize);
                                SPPMPixelListNode *node =
                                    arena.Alloc<SPPMPixelListNode>();
                                node->pixel = &pixel;

                                // Atomically add _node_ to the start of
                                // _grid[h]_'s linked list
                                node->next = grid[h];
                                while (grid[h].compare_exchange_weak(
                                           node->next, node) == false)
                                    ;
                            }
                    ReportValue(gridCellsPerVisiblePoint,
                                (1 + pMax.x - pMin.x) * (1 + pMax.y - pMin.y) *
                                    (1 + pMax.z - pMin.z));
                }
            }, nPixels, 4096);
        }

        // Trace photons and accumulate contributions
        {
            ProfilePhase _(Prof::SPPMPhotonPass);
            std::vector<MemoryArena> photonShootArenas(MaxThreadIndex());
            ParallelFor([&](int photonIndex) {
                MemoryArena &arena = photonShootArenas[ThreadIndex];
                // Follow photon path for _photonIndex_
                uint64_t haltonIndex =
                    (uint64_t)iter * (uint64_t)photonsPerIteration +
                    photonIndex;
                int haltonDim = 0;

                // Choose light to shoot photon from
                Float lightPdf;
                Float lightSample = RadicalInverse(haltonDim++, haltonIndex);
                int lightNum =
                    lightDistr->SampleDiscrete(lightSample, &lightPdf);
                const std::shared_ptr<Light> &light = scene.lights[lightNum];

                // Compute sample values for photon ray leaving light source
                Point2f uLight0(RadicalInverse(haltonDim, haltonIndex),
                                RadicalInverse(haltonDim + 1, haltonIndex));
                Point2f uLight1(RadicalInverse(haltonDim + 2, haltonIndex),
                                RadicalInverse(haltonDim + 3, haltonIndex));
                Float uLightTime =
                    Lerp(RadicalInverse(haltonDim + 4, haltonIndex),
                         camera->shutterOpen, camera->shutterClose);
                haltonDim += 5;

                // Generate _photonRay_ from light source and initialize _beta_
                RayDifferential photonRay;
                Normal3f nLight;
                Float pdfPos, pdfDir;
                Spectrum Le =
                    light->Sample_Le(uLight0, uLight1, uLightTime, &photonRay,
                                     &nLight, &pdfPos, &pdfDir);
                if (pdfPos == 0 || pdfDir == 0 || Le.IsBlack()) return;
                Spectrum beta = (AbsDot(nLight, photonRay.d) * Le) /
                                (lightPdf * pdfPos * pdfDir);
                if (beta.IsBlack()) return;

                // Follow photon path through scene and record intersections
                SurfaceInteraction isect;
                for (int depth = 0; depth < maxDepth; ++depth) {
                    if (!scene.Intersect(photonRay, &isect)) break;
                    ++totalPhotonSurfaceInteractions;
                    if (depth > 0) {
                        // Add photon contribution to nearby visible points
                        Point3i photonGridIndex;
                        if (ToGrid(isect.p, gridBounds, gridRes,
                                   &photonGridIndex)) {
                            int h = hash(photonGridIndex, hashSize);
                            // Add photon contribution to visible points in
                            // _grid[h]_
                            for (SPPMPixelListNode *node =
                                     grid[h].load(std::memory_order_relaxed);
                                 node != nullptr; node = node->next) {
                                ++visiblePointsChecked;
                                SPPMPixel &pixel = *node->pixel;
                                Float radius = pixel.radius;
                                if (DistanceSquared(pixel.vp.p, isect.p) >
                                    radius * radius)
                                    continue;
                                // Update _pixel_ $\Phi$ and $M$ for nearby
                                // photon
                                Vector3f wi = -photonRay.d;
                                Spectrum Phi =
                                    beta * pixel.vp.bsdf->f(pixel.vp.wo, wi);
                                for (int i = 0; i < Spectrum::nSamples; ++i)
                                    pixel.Phi[i].Add(Phi[i]);
                                ++pixel.M;
                            }
                        }
                    }
                    // Sample new photon ray direction

                    // Compute BSDF at photon intersection point
                    isect.ComputeScatteringFunctions(photonRay, arena, true,
                                                     TransportMode::Importance);
                    if (!isect.bsdf) {
                        --depth;
                        photonRay = isect.SpawnRay(photonRay.d);
                        continue;
                    }
                    const BSDF &photonBSDF = *isect.bsdf;

                    // Sample BSDF _fr_ and direction _wi_ for reflected photon
                    Vector3f wi, wo = -photonRay.d;
                    Float pdf;
                    BxDFType flags;

                    // Generate _bsdfSample_ for outgoing photon sample
                    Point2f bsdfSample(
                        RadicalInverse(haltonDim, haltonIndex),
                        RadicalInverse(haltonDim + 1, haltonIndex));
                    haltonDim += 2;
                    Spectrum fr = photonBSDF.Sample_f(wo, &wi, bsdfSample, &pdf,
                                                      BSDF_ALL, &flags);
                    if (fr.IsBlack() || pdf == 0.f) break;
                    Spectrum bnew =
                        beta * fr * AbsDot(wi, isect.shading.n) / pdf;

                    // Possibly terminate photon path with Russian roulette
                    Float q = std::max((Float)0, 1 - bnew.y() / beta.y());
                    if (RadicalInverse(haltonDim++, haltonIndex) < q) break;
                    beta = bnew / (1 - q);
                    photonRay = (RayDifferential)isect.SpawnRay(wi);
                }
                arena.Reset();
            }, photonsPerIteration, 8192);
            progress.Update();
            photonPaths += photonsPerIteration;
        }

        // Update pixel values from this pass's photons
        {
            ProfilePhase _(Prof::SPPMStatsUpdate);
            ParallelFor([&](int i) {
                SPPMPixel &p = pixels[i];
                if (p.M > 0) {
                    // Update pixel photon count, search radius, and $\tau$ from
                    // photons
                    Float gamma = (Float)2 / (Float)3;
                    Float Nnew = p.N + gamma * p.M;
                    Float Rnew = p.radius * std::sqrt(Nnew / (p.N + p.M));
                    Spectrum Phi;
                    for (int j = 0; j < Spectrum::nSamples; ++j)
                        Phi[j] = p.Phi[j];
                    p.tau = (p.tau + p.vp.beta * Phi) * (Rnew * Rnew) /
                            (p.radius * p.radius);
                    p.N = Nnew;
                    p.radius = Rnew;
                    p.M = 0;
                    for (int j = 0; j < Spectrum::nSamples; ++j)
                        p.Phi[j] = (Float)0;
                }
                // Reset _VisiblePoint_ in pixel
                p.vp.beta = 0.;
                p.vp.bsdf = nullptr;
            }, nPixels, 4096);
        }

        // Periodically store SPPM image in film and write image
        if (iter + 1 == nIterations || ((iter + 1) % writeFrequency) == 0) {
            int x0 = pixelBounds.pMin.x;
            int x1 = pixelBounds.pMax.x;
            uint64_t Np = (uint64_t)(iter + 1) * (uint64_t)photonsPerIteration;
            std::unique_ptr<Spectrum[]> image(new Spectrum[pixelBounds.Area()]);
            int offset = 0;
            for (int y = pixelBounds.pMin.y; y < pixelBounds.pMax.y; ++y) {
                for (int x = x0; x < x1; ++x) {
                    // Compute radiance _L_ for SPPM pixel _pixel_
                    const SPPMPixel &pixel =
                        pixels[(y - pixelBounds.pMin.y) * (x1 - x0) + (x - x0)];
                    Spectrum L = pixel.Ld / (iter + 1);
                    L += pixel.tau / (Np * Pi * pixel.radius * pixel.radius);
                    image[offset++] = L;
                }
            }
            camera->film->SetImage(image.get());
            camera->film->WriteImage();
            // Write SPPM radius image, if requested
            if (getenv("SPPM_RADIUS")) {
                std::unique_ptr<Float[]> rimg(
                    new Float[3 * pixelBounds.Area()]);
                Float minrad = 1e30f, maxrad = 0;
                for (int y = pixelBounds.pMin.y; y < pixelBounds.pMax.y; ++y) {
                    for (int x = x0; x < x1; ++x) {
                        const SPPMPixel &p =
                            pixels[(y - pixelBounds.pMin.y) * (x1 - x0) +
                                   (x - x0)];
                        minrad = std::min(minrad, p.radius);
                        maxrad = std::max(maxrad, p.radius);
                    }
                }
                fprintf(stderr,
                        "iterations: %d (%.2f s) radius range: %f - %f\n",
                        iter + 1, progress.ElapsedMS() / 1000., minrad, maxrad);
                int offset = 0;
                for (int y = pixelBounds.pMin.y; y < pixelBounds.pMax.y; ++y) {
                    for (int x = x0; x < x1; ++x) {
                        const SPPMPixel &p =
                            pixels[(y - pixelBounds.pMin.y) * (x1 - x0) +
                                   (x - x0)];
                        Float v = 1.f - (p.radius - minrad) / (maxrad - minrad);
                        rimg[offset++] = v;
                        rimg[offset++] = v;
                        rimg[offset++] = v;
                    }
                }
                Point2i res(pixelBounds.pMax.x - pixelBounds.pMin.x,
                            pixelBounds.pMax.y - pixelBounds.pMin.y);
                WriteImage("sppm_radius.png", rimg.get(), pixelBounds, res);
            }
        }
    }
    progress.Done();
}

Integrator *CreateSPPMIntegrator(const ParamSet &params,
                                 std::shared_ptr<const Camera> camera) {
    int nIterations =
        params.FindOneInt("iterations",
                          params.FindOneInt("numiterations", 64));
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int photonsPerIter = params.FindOneInt("photonsperiteration", -1);
    int writeFreq = params.FindOneInt("imagewritefrequency", 1 << 31);
    Float radius = params.FindOneFloat("radius", 1.f);
    if (PbrtOptions.quickRender) nIterations = std::max(1, nIterations / 16);
    return new SPPMIntegrator(camera, nIterations, photonsPerIter, maxDepth,
                              radius, writeFreq);
}

}  // namespace pbrt
