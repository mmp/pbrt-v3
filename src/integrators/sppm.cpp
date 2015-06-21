
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

// integrators/sppm.cpp*
#include "integrators/sppm.h"
#include "parallel.h"
#include "scene.h"
#include "imageio.h"
#include "spectrum.h"
#include "camera.h"
#include "film.h"
#include "rng.h"
#include "paramset.h"
#include "progressreporter.h"
#include "interaction.h"
#include "sampling.h"
#include "samplers/halton.h"
#include "stats.h"
STAT_TIMER("Time/SPPM camera pass", hitPointTimer);
STAT_TIMER("Time/SPPM hit point grid construction", gridConstructionTimer);
STAT_TIMER("Time/SPPM photon pass", photonTimer);
STAT_TIMER("Time/SPPM statistics update", statsUpdateTimer);
STAT_RATIO(
    "Stochastic Progressive Photon Mapping/Hit points checked per photon "
    "intersection",
    hitPointsChecked, totalPhotonSurfaceInteractions);
STAT_COUNTER("Stochastic Progressive Photon Mapping/Photon paths followed",
             photonPaths);
STAT_INT_DISTRIBUTION(
    "Stochastic Progressive Photon Mapping/Grid cells per hit point",
    gridCellsPerHitPoint);
STAT_MEMORY_COUNTER("Memory/SPPM Pixels", pixelMemoryBytes);
STAT_FLOAT_DISTRIBUTION("Memory/SPPM BSDF and Grid Memory", memoryArenaMB);

// SPPM Local Definitions
struct SPPMPixel {
    Spectrum Ld, Li;
    Float N = 0;
    Spectrum tau;
    Float radius = 0;

    void Reset() {
        Phi = 0.;
        M = 0;
        pathThroughput = 0.;
        bsdf = nullptr;
    }
    Spectrum Phi;
    int M;

    // HitPoint stuff...
    void RecordSurfaceInteraction(const Point3f &pp, const Vector3f &w,
                                  const BSDF *b, const Spectrum &pt) {
        p = pp;
        wo = w;
        bsdf = b;
        pathThroughput = pt;
    }
    Point3f p;
    Vector3f wo;
    Spectrum pathThroughput;
    const BSDF *bsdf = nullptr;
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

inline unsigned int hash(int ix, int iy, int iz, int hashSize) {
    return (unsigned int)((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) %
           hashSize;
}

struct SPPMPixelListNode {
    SPPMPixel *pixel;
    SPPMPixelListNode *next;
};

// SPPM Method Definitions
Integrator *CreateSPPMIntegrator(const ParamSet &params,
                                 std::shared_ptr<const Camera> camera) {
    int nIterations = params.FindOneInt("numiterations", 4);
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int direct = params.FindOneBool("directwithphotons", true);
    int photonsPerIter = params.FindOneInt("photonsperiteration", -1);
    int writeFreq = params.FindOneInt("imagewritefrequency", 1 << 31);
    Float radius = params.FindOneFloat("radius", 1.f);
    if (PbrtOptions.quickRender) nIterations = std::max(1, nIterations / 16);
    return new SPPMIntegrator(camera, nIterations, photonsPerIter, maxDepth,
                              radius, direct, writeFreq, camera->film);
}

SPPMIntegrator::SPPMIntegrator(std::shared_ptr<const Camera> &cam, int ni,
                               int ppi, int md, Float radius, bool direct,
                               int wf, const Film *film)
    : camera(cam),
      nIterations(ni),
      photonsPerIteration(ppi),
      maxDepth(md),
      initialSearchRadius(radius),
      directWithPhotons(direct),
      writeFrequency(wf) {
    baseGridRes = std::max(film->fullResolution.x, film->fullResolution.y);
}

void SPPMIntegrator::Render(const Scene &scene) {
    Bounds2i pixelBounds = camera->film->croppedPixelBounds;
    int nPixels = pixelBounds.Area();
    const Float invSqrtSPP = 1.f / std::sqrt(nIterations);
    std::vector<SPPMPixel> pixels(nPixels);
    pixelMemoryBytes = nPixels * sizeof(SPPMPixel);
    HaltonSampler mainSampler(nIterations, pixelBounds);
    // Compute number of buckets to use for SPPM camera pass
    Vector2i pixelExtent = pixelBounds.Diagonal();
    const int bucketSize = 32;
    int nXBuckets = (pixelExtent.x + bucketSize - 1) / bucketSize;
    int nYBuckets = (pixelExtent.y + bucketSize - 1) / bucketSize;
    const int nPhotonTasks = nXBuckets * nYBuckets;

    // Compute _lightDistribution_ for sampling lights proportional to power
    std::unique_ptr<Distribution1D> lightDistribution(
        ComputeLightSamplingCDF(scene));
    ProgressReporter progress(
        nIterations * (nXBuckets * nYBuckets + nPhotonTasks), "Rendering");
    for (int iter = 0; iter < nIterations; ++iter) {
        // Generate SPPM camera path hit points
        std::vector<MemoryArena> perThreadArenas(MaxThreadIndex());
        {
            StatTimer timer(&hitPointTimer);
            ParallelFor([&](const Point2i bucket, const int threadIndex) {
                MemoryArena &arena = perThreadArenas[threadIndex];
                // Follow camera paths for _bucket_ in image for SPPM
                int seed = bucket.y * nXBuckets + bucket.x;
                std::unique_ptr<Sampler> sampler = mainSampler.Clone(seed);
                int x0 = pixelBounds.pMin.x + bucket.x * bucketSize;
                int x1 = std::min(x0 + bucketSize, pixelBounds.pMax.x);
                int y0 = pixelBounds.pMin.y + bucket.y * bucketSize;
                int y1 = std::min(y0 + bucketSize, pixelBounds.pMax.y);
                Bounds2i bucketBounds(Point2i(x0, y0), Point2i(x1, y1));
                for (Point2i pPixel : bucketBounds) {
                    sampler->StartPixel(pPixel);
                    sampler->SetSampleNumber(iter);
                    int pixelOffset =
                        (pPixel.x - pixelBounds.pMin.x) +
                        (pPixel.y - pixelBounds.pMin.y) *
                            (pixelBounds.pMax.x - pixelBounds.pMin.x);
                    SPPMPixel &pixel = pixels[pixelOffset];
                    pixel.Reset();
                    if (iter == 0) pixel.radius = initialSearchRadius;
                    // Compute camera ray for pixel for SPPM
                    CameraSample cameraSample;
                    cameraSample.pFilm = (Point2f)pPixel + sampler->Get2D();
                    cameraSample.time = sampler->Get1D();
                    cameraSample.pLens = sampler->Get2D();
                    RayDifferential ray;
                    Spectrum pathThroughput =
                        camera->GenerateRayDifferential(cameraSample, &ray);
                    ray.ScaleDifferentials(1.f * invSqrtSPP);

                    // Follow camera ray path until a diffuse surface is found
                    for (int depth = 0; depth < maxDepth; ++depth) {
                        SurfaceInteraction isect;
                        if (!scene.Intersect(ray, &isect)) {
                            for (const auto &light : scene.lights)
                                pixel.Ld += pathThroughput * light->Le(ray);
                            break;
                        }
                        isect.ComputeScatteringFunctions(ray, arena, true);
                        const BSDF &bsdf = *isect.bsdf;
                        Vector3f wo = -ray.d;
                        Spectrum Le = isect.Le(wo);
                        pixel.Ld += pathThroughput * Le;
                        bool isDiffuseSurfaceInteraction =
                            bsdf.NumComponents(BxDFType(BSDF_DIFFUSE |
                                                        BSDF_REFLECTION |
                                                        BSDF_TRANSMISSION)) > 0;
                        if (isDiffuseSurfaceInteraction == false ||
                            directWithPhotons == false) {
                            // Accumulate direct lighting at SPPM ray
                            // intersection point
                            Spectrum Ld = UniformSampleOneLight(
                                isect, scene, *sampler, arena, false);
                            pixel.Ld += pathThroughput * Ld;
                        }
                        // Sample BSDF and either record intersection for SPPM
                        // or follow bounce
                        if (isDiffuseSurfaceInteraction) {
                            pixel.RecordSurfaceInteraction(isect.p, wo, &bsdf,
                                                           pathThroughput);
                            break;
                        } else {
                            Float pdf;
                            Vector3f wi;
                            Spectrum f =
                                bsdf.Sample_f(wo, &wi, sampler->Get2D(), &pdf);
                            if (pdf == 0. || f.IsBlack()) break;
                            pathThroughput *=
                                f * AbsDot(wi, isect.shading.n) / pdf;
                            ray = (RayDifferential)isect.SpawnRay(
                                wi, ray.depth + 1);
                        }
                    }
                }
                progress.Update();
            }, Point2i(nXBuckets, nYBuckets));
        }

        // Create grid of all SPPM hit points
        Bounds3f gridBounds;
        int hashSize = pixels.size();
        std::vector<std::atomic<SPPMPixelListNode *>> grid(hashSize);
        int gridRes[3];
        {
            StatTimer timer(&gridConstructionTimer);

            // Compute grid bounds for SPPM hit points
            Float maxRadius = 0.;
            for (const SPPMPixel &pixel : pixels) {
                if (pixel.pathThroughput.IsBlack()) continue;
                Vector3f delta(pixel.radius, pixel.radius, pixel.radius);
                gridBounds = Union(gridBounds, pixel.p + delta);
                gridBounds = Union(gridBounds, pixel.p - delta);
                maxRadius = std::max(maxRadius, pixel.radius);
            }

            // Compute resolution of SPPM grid in each dimension
            Vector3f diag = gridBounds.Diagonal();
            Float maxDiag = std::max(diag.x, std::max(diag.y, diag.z));
            int br = (int)(maxDiag / maxRadius);
            Assert(br > 0);
            for (int i = 0; i < 3; ++i) {
                gridRes[i] =
                    std::max((int)(/*baseGridRes*/ br * diag[i] / maxDiag), 1);
                //    fprintf(stderr, "res %d (diag %f / %f)\n", gridRes[i],
                //    diag[i], maxDiag);
            }
            ParallelFor([&](const int pixelIndex, const int threadIndex) {
                MemoryArena &arena = perThreadArenas[threadIndex];
                SPPMPixel &pixel = pixels[pixelIndex];
                if (pixel.pathThroughput.IsBlack() == false) {
                    // Add pixel's diffuse hit point to applicable grid cells
                    float radius = pixel.radius;
                    Point3i pMin, pMax;
                    ToGrid(pixel.p - Vector3f(radius, radius, radius),
                           gridBounds, gridRes, &pMin);
                    ToGrid(pixel.p + Vector3f(radius, radius, radius),
                           gridBounds, gridRes, &pMax);
                    for (int z = pMin.z; z <= pMax.z; ++z) {
                        for (int y = pMin.y; y <= pMax.y; ++y) {
                            for (int x = pMin.x; x <= pMax.x; ++x) {
                                // Add hit point to grid cell $(x, y, z)$
                                int h = hash(x, y, z, hashSize);
                                SPPMPixelListNode *node =
                                    arena.Alloc<SPPMPixelListNode>();
                                node->pixel = &pixel;
                                do {
                                    node->next = grid[h];
                                } while (std::atomic_compare_exchange_weak(
                                             &grid[h], &node->next, node) ==
                                         false);
                            }
                        }
                    }
                    ReportValue(gridCellsPerHitPoint,
                                (1 + pMax.x - pMin.x) * (1 + pMax.y - pMin.y) *
                                    (1 + pMax.z - pMin.z));
                }
            }, pixels.size(), 4096);
        }

        // Trace photons and accumulate photon contributions
        {
            StatTimer timer(&photonTimer);
            if (photonsPerIteration <= 0) photonsPerIteration = nPixels;
            int photonsPerTask = photonsPerIteration / nPhotonTasks;
            // constexpr int kMutexPoolSize = 32;
            // std::mutex gridMutexPool[kMutexPoolSize];
            ParallelFor([&](const int taskNum) {
                MemoryArena arena;
                uint64_t haltonIndex = taskNum + nPixels * iter;
                photonPaths += photonsPerTask;
                for (int photon = 0; photon < photonsPerTask;
                     ++photon, haltonIndex += nPhotonTasks) {
                    int haltonDim = 0;
                    // Choose light to shoot photon from
                    Float lightPdf;
                    Float lightSample =
                        RadicalInverse(haltonDim++, haltonIndex);
                    int lightNum = lightDistribution->SampleDiscrete(
                        lightSample, &lightPdf);
                    const std::shared_ptr<Light> &light =
                        scene.lights[lightNum];

                    // Generate _photonRay_ from light source and initialize
                    // _alpha_
                    RayDifferential photonRay;
                    Point2f lightSample0(
                        RadicalInverse(haltonDim, haltonIndex),
                        RadicalInverse(haltonDim + 1, haltonIndex));
                    Point2f lightSample1(
                        RadicalInverse(haltonDim + 2, haltonIndex),
                        RadicalInverse(haltonDim + 3, haltonIndex));
                    Float lightSampleTime =
                        Lerp(RadicalInverse(haltonDim + 4, haltonIndex),
                             camera->shutterOpen, camera->shutterClose);
                    haltonDim += 5;
                    Normal3f Nl;
                    Float pdfPos, pdfDir;
                    Spectrum Le = light->Sample_L(lightSample0, lightSample1,
                                                  lightSampleTime, &photonRay,
                                                  &Nl, &pdfPos, &pdfDir);
                    if (pdfPos == 0.f || pdfDir == 0.f || Le.IsBlack())
                        continue;
                    Spectrum alpha = (AbsDot(Nl, photonRay.d) * Le) /
                                     (pdfPos * pdfDir * lightPdf);
                    if (alpha.IsBlack()) continue;
                    // Follow photon path through scene and record intersections
                    SurfaceInteraction photonIsect;
                    for (int depth = 0; depth < maxDepth; ++depth) {
                        if (!scene.Intersect(photonRay, &photonIsect)) break;
                        // Handle photon/surface intersection
                        ++totalPhotonSurfaceInteractions;
                        if (directWithPhotons == true || depth > 0) {
                            // Add photon contribution to nearby camera path hit
                            // points
                            Point3i pPhoton;
                            if (ToGrid(photonIsect.p, gridBounds, gridRes,
                                       &pPhoton)) {
                                int h = hash(pPhoton.x, pPhoton.y, pPhoton.z,
                                             hashSize);
                                //    int mutexIndex = h & (kMutexPoolSize - 1);
                                //    std::lock_guard<std::mutex>
                                //    lock(gridMutexPool[mutexIndex]);
                                Assert(h < grid.size());
                                for (SPPMPixelListNode *node = grid[h].load(
                                         std::memory_order_relaxed);
                                     node; node = node->next) {
                                    ++hitPointsChecked;
                                    SPPMPixel &pixel = *node->pixel;
                                    Float radius = pixel.radius;
                                    if (DistanceSquared(pixel.p,
                                                        photonIsect.p) >
                                        radius * radius)
                                        continue;
                                    Vector3f wi = -photonRay.d;
                                    pixel.Phi += alpha * pixel.pathThroughput *
                                                 pixel.bsdf->f(pixel.wo, wi);
                                    ++pixel.M;
                                }
                            }
                        }

                        // Sample new photon ray direction
                        photonIsect.ComputeScatteringFunctions(
                            photonRay, arena, true, TransportMode::Importance);
                        const BSDF &photonBSDF = *photonIsect.bsdf;
                        Vector3f wo = -photonRay.d;
                        Vector3f wi;
                        Float pdf;
                        BxDFType flags;
                        Point2f bsdfSample(
                            RadicalInverse(haltonDim, haltonIndex),
                            RadicalInverse(haltonDim + 1, haltonIndex));
                        haltonDim += 2;
                        Spectrum fr = photonBSDF.Sample_f(
                            wo, &wi, bsdfSample, &pdf, BSDF_ALL, &flags);
                        if (fr.IsBlack() || pdf == 0.f) break;
                        Spectrum anew = alpha * fr *
                                        AbsDot(wi, photonIsect.shading.n) / pdf;

                        // Possibly terminate photon path with Russian roulette
                        Float continueProb =
                            std::min((Float)1., anew.y() / alpha.y());
                        if (RadicalInverse(haltonDim++, haltonIndex) >
                            continueProb)
                            break;
                        alpha = anew / continueProb;
                        photonRay = (RayDifferential)photonIsect.SpawnRay(
                            wi, photonRay.depth + 1);
                    }
                    arena.Reset();
                }
                progress.Update();
            }, nPhotonTasks);
        }

        // Update pixel values from this pass's photons
        {
            StatTimer timer(&statsUpdateTimer);
            Float alpha = 0.6666666666667f;
            for (SPPMPixel &p : pixels) {
                if (p.M == 0) continue;
                Float Nnew = p.N + alpha * p.M;
                Float Rnew = p.radius * std::sqrt(Nnew / (p.N + p.M));
                p.tau = (p.tau + p.Phi) * (Rnew * Rnew) / (p.radius * p.radius);
                p.N = Nnew;
                p.radius = Rnew;
            }
        }
        if (iter + 1 == nIterations || ((iter + 1) % writeFrequency) == 0) {
            // Store SPPM image in film and write image
            int x0 = pixelBounds.pMin.x;
            int x1 = pixelBounds.pMax.x;
            uint64_t totalPhotons =
                (uint64_t)(iter + 1) * (uint64_t)photonsPerIteration;
            std::unique_ptr<Spectrum[]> image(new Spectrum[pixelBounds.Area()]);
            int offset = 0;
            for (int y = pixelBounds.pMin.y; y < pixelBounds.pMax.y; ++y) {
                for (int x = x0; x < x1; ++x) {
                    const SPPMPixel &p =
                        pixels[(y - pixelBounds.pMin.y) * (x1 - x0) + (x - x0)];
                    Spectrum L = p.Ld / (iter + 1);
                    L += p.tau / (totalPhotons * Pi * p.radius * p.radius);
                    image[offset++] = L;
                }
            }
            camera->film->SetImage(image.get());
            camera->film->WriteImage();

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
                WriteImage("sppm_radius.png", rimg.get(), pixelBounds, res,
                           2.2f);
            }
        }
    }
    progress.Done();
}
