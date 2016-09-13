
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

// integrators/bdpt.cpp*
#include "integrators/bdpt.h"
#include "film.h"
#include "filters/box.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "paramset.h"
#include "progressreporter.h"
#include "sampler.h"
#include "stats.h"

STAT_TIMER("Time/Rendering", renderingTime);
STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIBUTION("Integrator/Path length", pathLength);

// BDPT Forward Declarations
int RandomWalk(const Scene &scene, RayDifferential ray, Sampler &sampler,
               MemoryArena &arena, Spectrum beta, Float pdf, int maxDepth,
               TransportMode mode, Vertex *path);

// BDPT Utility Functions
Float CorrectShadingNormal(const SurfaceInteraction &isect, const Vector3f &wo,
                           const Vector3f &wi, TransportMode mode) {
    if (mode == TransportMode::Importance) {
        Float num = AbsDot(wo, isect.shading.n) * AbsDot(wi, isect.n);
        Float denom = AbsDot(wo, isect.n) * AbsDot(wi, isect.shading.n);
        if (denom == 0) {
            CHECK_EQ(num, 0);
            return 0;
        }
        return num / denom;
    } else
        return 1;
}

int GenerateCameraSubpath(const Scene &scene, Sampler &sampler,
                          MemoryArena &arena, int maxDepth,
                          const Camera &camera, const Point2f &pFilm,
                          Vertex *path) {
    if (maxDepth == 0) return 0;
    // Sample initial ray for camera subpath
    CameraSample cameraSample;
    cameraSample.pFilm = pFilm;
    cameraSample.time = sampler.Get1D();
    cameraSample.pLens = sampler.Get2D();
    RayDifferential ray;
    Spectrum beta = camera.GenerateRayDifferential(cameraSample, &ray);
    ray.ScaleDifferentials(1 / std::sqrt(sampler.samplesPerPixel));

    // Generate first vertex on camera subpath and start random walk
    Float pdfPos, pdfDir;
    path[0] = Vertex::CreateCamera(&camera, ray, beta);
    camera.Pdf_We(ray, &pdfPos, &pdfDir);
    VLOG(2) << "Starting camera subpath. Ray: " << ray << ", beta " << beta
            << ", pdfPos " << pdfPos << ", pdfDir " << pdfDir;
    return RandomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1,
                      TransportMode::Radiance, path + 1) +
           1;
}

int GenerateLightSubpath(
    const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
    Float time, const Distribution1D &lightDistr,
    const std::unordered_map<const Light *, size_t> &lightToIndex,
    Vertex *path) {
    if (maxDepth == 0) return 0;
    // Sample initial ray for light subpath
    Float lightPdf;
    int lightNum = lightDistr.SampleDiscrete(sampler.Get1D(), &lightPdf);
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    RayDifferential ray;
    Normal3f nLight;
    Float pdfPos, pdfDir;
    Spectrum Le = light->Sample_Le(sampler.Get2D(), sampler.Get2D(), time, &ray,
                                   &nLight, &pdfPos, &pdfDir);
    if (pdfPos == 0 || pdfDir == 0 || Le.IsBlack()) return 0;

    // Generate first vertex on light subpath and start random walk
    path[0] =
        Vertex::CreateLight(light.get(), ray, nLight, Le, pdfPos * lightPdf);
    Spectrum beta = Le * AbsDot(nLight, ray.d) / (lightPdf * pdfPos * pdfDir);
    VLOG(2) << "Starting light subpath. Ray: " << ray << ", Le " << Le <<
        ", beta " << beta << ", pdfPos " << pdfPos << ", pdfDir " << pdfDir;
    int nVertices =
        RandomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1,
                   TransportMode::Importance, path + 1);

    // Correct subpath sampling densities for infinite area lights
    if (path[0].IsInfiniteLight()) {
        // Set spatial density of _path[1]_ for infinite area light
        if (nVertices > 0) {
            path[1].pdfFwd = pdfPos;
            if (path[1].IsOnSurface())
                path[1].pdfFwd *= AbsDot(ray.d, path[1].ng());
        }

        // Set spatial density of _path[0]_ for infinite area light
        path[0].pdfFwd =
            InfiniteLightDensity(scene, lightDistr, lightToIndex, ray.d);
    }
    return nVertices + 1;
}

int RandomWalk(const Scene &scene, RayDifferential ray, Sampler &sampler,
               MemoryArena &arena, Spectrum beta, Float pdf, int maxDepth,
               TransportMode mode, Vertex *path) {
    if (maxDepth == 0) return 0;
    int bounces = 0;
    // Declare variables for forward and reverse probability densities
    Float pdfFwd = pdf, pdfRev = 0;
    while (true) {
        // Attempt to create the next subpath vertex in _path_
        MediumInteraction mi;

        VLOG(2) << "Random walk. Bounces " << bounces << ", beta " << beta <<
            ", pdfFwd " << pdfFwd << ", pdfRev " << pdfRev;
        // Trace a ray and sample the medium, if any
        SurfaceInteraction isect;
        bool foundIntersection = scene.Intersect(ray, &isect);
        if (ray.medium) beta *= ray.medium->Sample(ray, sampler, arena, &mi);
        if (beta.IsBlack()) break;
        Vertex &vertex = path[bounces], &prev = path[bounces - 1];
        if (mi.IsValid()) {
            // Record medium interaction in _path_ and compute forward density
            vertex = Vertex::CreateMedium(mi, beta, pdfFwd, prev);
            if (++bounces >= maxDepth) break;

            // Sample direction and compute reverse density at preceding vertex
            Vector3f wi;
            pdfFwd = pdfRev = mi.phase->Sample_p(-ray.d, &wi, sampler.Get2D());
            ray = mi.SpawnRay(wi);
        } else {
            // Handle surface interaction for path generation
            if (!foundIntersection) {
                // Capture escaped rays when tracing from the camera
                if (mode == TransportMode::Radiance) {
                    vertex = Vertex::CreateLight(EndpointInteraction(ray), beta,
                                                 pdfFwd);
                    ++bounces;
                }
                break;
            }

            // Compute scattering functions for _mode_ and skip over medium
            // boundaries
            isect.ComputeScatteringFunctions(ray, arena, true, mode);
            if (!isect.bsdf) {
                ray = isect.SpawnRay(ray.d);
                continue;
            }

            // Initialize _vertex_ with surface intersection information
            vertex = Vertex::CreateSurface(isect, beta, pdfFwd, prev);
            if (++bounces >= maxDepth) break;

            // Sample BSDF at current vertex and compute reverse probability
            Vector3f wi, wo = isect.wo;
            BxDFType type;
            Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdfFwd,
                                              BSDF_ALL, &type);
            VLOG(2) << "Random walk sampled dir " << wi << " f: " << f <<
                ", pdfFwd: " << pdfFwd;
            if (f.IsBlack() || pdfFwd == 0.f) break;
            beta *= f * AbsDot(wi, isect.shading.n) / pdfFwd;
            VLOG(2) << "Random walk beta now " << beta;
            pdfRev = isect.bsdf->Pdf(wi, wo, BSDF_ALL);
            if (type & BSDF_SPECULAR) {
                vertex.delta = true;
                pdfRev = pdfFwd = 0;
            }
            beta *= CorrectShadingNormal(isect, wo, wi, mode);
            VLOG(2) << "Random walk beta after shading normal correction " << beta;
            ray = isect.SpawnRay(wi);
        }

        // Compute reverse area density at preceding vertex
        prev.pdfRev = vertex.ConvertDensity(pdfRev, prev);
    }
    return bounces;
}

Spectrum G(const Scene &scene, Sampler &sampler, const Vertex &v0,
           const Vertex &v1) {
    Vector3f d = v0.p() - v1.p();
    Float g = 1 / d.LengthSquared();
    d *= std::sqrt(g);
    if (v0.IsOnSurface()) g *= AbsDot(v0.ns(), d);
    if (v1.IsOnSurface()) g *= AbsDot(v1.ns(), d);
    VisibilityTester vis(v0.GetInteraction(), v1.GetInteraction());
    return g * vis.Tr(scene, sampler);
}

Float MISWeight(const Scene &scene, Vertex *lightVertices,
                Vertex *cameraVertices, Vertex &sampled, int s, int t,
                const Distribution1D &lightPdf,
                const std::unordered_map<const Light *, size_t> &lightToIndex) {
    if (s + t == 2) return 1;
    Float sumRi = 0;
    // Define helper function _remap0_ that deals with Dirac delta functions
    auto remap0 = [](Float f) -> Float { return f != 0 ? f : 1; };

    // Temporarily update vertex properties for current strategy

    // Look up connection vertices and their predecessors
    Vertex *qs = s > 0 ? &lightVertices[s - 1] : nullptr,
           *pt = t > 0 ? &cameraVertices[t - 1] : nullptr,
           *qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr,
           *ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

    // Update sampled vertex for $s=1$ or $t=1$ strategy
    ScopedAssignment<Vertex> a1;
    if (s == 1)
        a1 = {qs, sampled};
    else if (t == 1)
        a1 = {pt, sampled};

    // Mark connection vertices as non-degenerate
    ScopedAssignment<bool> a2, a3;
    if (pt) a2 = {&pt->delta, false};
    if (qs) a3 = {&qs->delta, false};

    // Update reverse density of vertex $\pt{}_{t-1}$
    ScopedAssignment<Float> a4;
    if (pt)
        a4 = {&pt->pdfRev, s > 0 ? qs->Pdf(scene, qsMinus, *pt)
                                 : pt->PdfLightOrigin(scene, *ptMinus, lightPdf,
                                                      lightToIndex)};

    // Update reverse density of vertex $\pt{}_{t-2}$
    ScopedAssignment<Float> a5;
    if (ptMinus)
        a5 = {&ptMinus->pdfRev, s > 0 ? pt->Pdf(scene, qs, *ptMinus)
                                      : pt->PdfLight(scene, *ptMinus)};

    // Update reverse density of vertices $\pq{}_{s-1}$ and $\pq{}_{s-2}$
    ScopedAssignment<Float> a6;
    if (qs) a6 = {&qs->pdfRev, pt->Pdf(scene, ptMinus, *qs)};
    ScopedAssignment<Float> a7;
    if (qsMinus) a7 = {&qsMinus->pdfRev, qs->Pdf(scene, pt, *qsMinus)};

    // Consider hypothetical connection strategies along the camera subpath
    Float ri = 1;
    for (int i = t - 1; i > 0; --i) {
        ri *=
            remap0(cameraVertices[i].pdfRev) / remap0(cameraVertices[i].pdfFwd);
        if (!cameraVertices[i].delta && !cameraVertices[i - 1].delta)
            sumRi += ri;
    }

    // Consider hypothetical connection strategies along the light subpath
    ri = 1;
    for (int i = s - 1; i >= 0; --i) {
        ri *= remap0(lightVertices[i].pdfRev) / remap0(lightVertices[i].pdfFwd);
        bool deltaLightvertex = i > 0 ? lightVertices[i - 1].delta
                                      : lightVertices[0].IsDeltaLight();
        if (!lightVertices[i].delta && !deltaLightvertex) sumRi += ri;
    }
    return 1 / (1 + sumRi);
}

// BDPT Method Definitions
inline int BufferIndex(int s, int t) {
    int above = s + t - 2;
    return s + above * (5 + above) / 2;
}

void BDPTIntegrator::Render(const Scene &scene) {
    ProfilePhase p(Prof::IntegratorRender);

    // We'll generally use a SpatialLightDistribution for the sampling
    // distribution for choosing the light at the start of a light path,
    // using the first visible point from the camera as the lookup
    // point. For cases where the camera ray doesn't intersect anything,
    // we'll fall back to sampling based on light power.
    std::unique_ptr<LightDistribution> defaultLightDistrib =
        CreateLightSampleDistribution(lightSampleStrategy, scene);
    PowerLightDistribution powerLightDistrib(scene);

    // Compute a reverse mapping from light pointers to offsets into the
    // scene lights vector (and, equivalently, offsets into
    // lightDistr). Added after book text was finalized; this is critical
    // to reasonable performance with 100s+ of light sources.
    std::unordered_map<const Light *, size_t> lightToIndex;
    for (size_t i = 0; i < scene.lights.size(); ++i)
        lightToIndex[scene.lights[i].get()] = i;

    // Partition the image into tiles
    Film *film = camera->film;
    const Bounds2i sampleBounds = film->GetSampleBounds();
    const Vector2i sampleExtent = sampleBounds.Diagonal();
    const int tileSize = 16;
    const int nXTiles = (sampleExtent.x + tileSize - 1) / tileSize;
    const int nYTiles = (sampleExtent.y + tileSize - 1) / tileSize;
    ProgressReporter reporter(nXTiles * nYTiles, "Rendering");

    // Allocate buffers for debug visualization
    const int bufferCount = (1 + maxDepth) * (6 + maxDepth) / 2;
    std::vector<std::unique_ptr<Film>> weightFilms(bufferCount);
    if (visualizeStrategies || visualizeWeights) {
        for (int depth = 0; depth <= maxDepth; ++depth) {
            for (int s = 0; s <= depth + 2; ++s) {
                int t = depth + 2 - s;
                if (t == 0 || (s == 1 && t == 1)) continue;

                std::string filename =
                    StringPrintf("bdpt_d%02i_s%02i_t%02i.exr", depth, s, t);

                weightFilms[BufferIndex(s, t)] = std::unique_ptr<Film>(new Film(
                    film->fullResolution,
                    Bounds2f(Point2f(0, 0), Point2f(1, 1)),
                    std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
                    film->diagonal * 1000, filename, 1.f));
            }
        }
    }

    // Render and write the output image to disk
    if (scene.lights.size() > 0) {
        StatTimer timer(&renderingTime);
        ParallelFor2D([&](const Point2i tile) {
            // Render a single tile using BDPT
            MemoryArena arena;
            int seed = tile.y * nXTiles + tile.x;
            std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);
            int x0 = sampleBounds.pMin.x + tile.x * tileSize;
            int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + tile.y * tileSize;
            int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
            Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
            std::unique_ptr<FilmTile> filmTile =
                camera->film->GetFilmTile(tileBounds);
            for (Point2i pPixel : tileBounds) {
                tileSampler->StartPixel(pPixel);
                if (!InsideExclusive(pPixel, pixelBounds))
                    continue;
                do {
                    // Generate a single sample using BDPT
                    Point2f pFilm = (Point2f)pPixel + tileSampler->Get2D();

                    // Trace the camera subpath
                    Vertex *cameraVertices = arena.Alloc<Vertex>(maxDepth + 2);
                    Vertex *lightVertices = arena.Alloc<Vertex>(maxDepth + 1);
                    int nCamera = GenerateCameraSubpath(
                        scene, *tileSampler, arena, maxDepth + 2, *camera,
                        pFilm, cameraVertices);
                    // Get a distribution for sampling the light at the start
                    // of the light subpath.
                    const Distribution1D *lightDistr = (nCamera >= 2) ?
                        defaultLightDistrib->Lookup(cameraVertices[1].p()) :
                        powerLightDistrib.Lookup(cameraVertices[0].p());
                    // Now trace the light subpath
                    int nLight = GenerateLightSubpath(
                        scene, *tileSampler, arena, maxDepth + 1,
                        cameraVertices[0].time(), *lightDistr, lightToIndex,
                        lightVertices);

                    // Execute all BDPT connection strategies
                    Spectrum L(0.f);
                    for (int t = 1; t <= nCamera; ++t) {
                        for (int s = 0; s <= nLight; ++s) {
                            int depth = t + s - 2;
                            if ((s == 1 && t == 1) || depth < 0 ||
                                depth > maxDepth)
                                continue;
                            // Execute the $(s, t)$ connection strategy and
                            // update _L_
                            Point2f pFilmNew = pFilm;
                            Float misWeight = 0.f;
                            Spectrum Lpath = ConnectBDPT(
                                scene, lightVertices, cameraVertices, s, t,
                                *lightDistr, lightToIndex, *camera, *tileSampler,
                                &pFilmNew, &misWeight);
                            VLOG(2) << "Connect bdpt s: " << s <<", t: " << t <<
                                ", Lpath: " << Lpath << ", misWeight: " << misWeight;
                            if (visualizeStrategies || visualizeWeights) {
                                Spectrum value;
                                if (visualizeStrategies)
                                    value =
                                        misWeight == 0 ? 0 : Lpath / misWeight;
                                if (visualizeWeights) value = Lpath;
                                weightFilms[BufferIndex(s, t)]->AddSplat(
                                    pFilmNew, value);
                            }
                            if (t != 1)
                                L += Lpath;
                            else
                                film->AddSplat(pFilmNew, Lpath);
                        }
                    }
                    VLOG(2) << "Add film sample pFilm: " << pFilm << ", L: " << L <<
                        ", (y: " << L.y() << ")";
                    filmTile->AddSample(pFilm, L);
                    arena.Reset();
                } while (tileSampler->StartNextSample());
            }
            film->MergeFilmTile(std::move(filmTile));
            reporter.Update();
        }, Point2i(nXTiles, nYTiles));
        reporter.Done();
    }
    film->WriteImage(1.0f / sampler->samplesPerPixel);

    // Write buffers for debug visualization
    if (visualizeStrategies || visualizeWeights) {
        const Float invSampleCount = 1.0f / sampler->samplesPerPixel;
        for (size_t i = 0; i < weightFilms.size(); ++i)
            if (weightFilms[i]) weightFilms[i]->WriteImage(invSampleCount);
    }
}

Spectrum ConnectBDPT(
    const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, int s,
    int t, const Distribution1D &lightDistr,
    const std::unordered_map<const Light *, size_t> &lightToIndex,
    const Camera &camera, Sampler &sampler, Point2f *pRaster,
    Float *misWeightPtr) {
    Spectrum L(0.f);
    // Ignore invalid connections related to infinite area lights
    if (t > 1 && s != 0 && cameraVertices[t - 1].type == VertexType::Light)
        return Spectrum(0.f);

    // Perform connection and write contribution to _L_
    Vertex sampled;
    if (s == 0) {
        // Interpret the camera subpath as a complete path
        const Vertex &pt = cameraVertices[t - 1];
        if (pt.IsLight()) L = pt.Le(scene, cameraVertices[t - 2]) * pt.beta;
        DCHECK(!L.HasNaNs());
    } else if (t == 1) {
        // Sample a point on the camera and connect it to the light subpath
        const Vertex &qs = lightVertices[s - 1];
        if (qs.IsConnectible()) {
            VisibilityTester vis;
            Vector3f wi;
            Float pdf;
            Spectrum Wi = camera.Sample_Wi(qs.GetInteraction(), sampler.Get2D(),
                                           &wi, &pdf, pRaster, &vis);
            if (pdf > 0 && !Wi.IsBlack()) {
                // Initialize dynamically sampled vertex and _L_ for $t=1$ case
                sampled = Vertex::CreateCamera(&camera, vis.P1(), Wi / pdf);
                L = qs.beta * qs.f(sampled) * sampled.beta;
                if (qs.IsOnSurface()) L *= AbsDot(wi, qs.ns());
                DCHECK(!L.HasNaNs());
                // Only check visibility after we know that the path would
                // make a non-zero contribution.
                if (!L.IsBlack()) L *= vis.Tr(scene, sampler);
            }
        }
    } else if (s == 1) {
        // Sample a point on a light and connect it to the camera subpath
        const Vertex &pt = cameraVertices[t - 1];
        if (pt.IsConnectible()) {
            Float lightPdf;
            VisibilityTester vis;
            Vector3f wi;
            Float pdf;
            int lightNum =
                lightDistr.SampleDiscrete(sampler.Get1D(), &lightPdf);
            const std::shared_ptr<Light> &light = scene.lights[lightNum];
            Spectrum lightWeight = light->Sample_Li(
                pt.GetInteraction(), sampler.Get2D(), &wi, &pdf, &vis);
            if (pdf > 0 && !lightWeight.IsBlack()) {
                EndpointInteraction ei(vis.P1(), light.get());
                sampled =
                    Vertex::CreateLight(ei, lightWeight / (pdf * lightPdf), 0);
                sampled.pdfFwd =
                    sampled.PdfLightOrigin(scene, pt, lightDistr, lightToIndex);
                L = pt.beta * pt.f(sampled) * sampled.beta;
                if (pt.IsOnSurface()) L *= AbsDot(wi, pt.ns());
                // Only check visibility if the path would carry radiance.
                if (!L.IsBlack()) L *= vis.Tr(scene, sampler);
            }
        }
    } else {
        // Handle all other bidirectional connection cases
        const Vertex &qs = lightVertices[s - 1], &pt = cameraVertices[t - 1];
        if (qs.IsConnectible() && pt.IsConnectible()) {
            L = qs.beta * qs.f(pt) * pt.f(qs) * pt.beta;
            VLOG(2) << "General connect s: " << s << ", t: " << t <<
                " qs: " << qs << ", pt: " << pt << ", qs.f(pt): " << qs.f(pt) <<
                ", pt.f(qs): " << pt.f(qs);
            if (!L.IsBlack()) L *= G(scene, sampler, qs, pt);
        }
    }

    ++totalPaths;
    if (L.IsBlack()) ++zeroRadiancePaths;
    ReportValue(pathLength, s + t - 2);

    // Compute MIS weight for connection strategy
    Float misWeight =
        L.IsBlack() ? 0.f : MISWeight(scene, lightVertices, cameraVertices,
                                      sampled, s, t, lightDistr, lightToIndex);
    DCHECK(!std::isnan(misWeight));
    L *= misWeight;
    if (misWeightPtr) *misWeightPtr = misWeight;
    return L;
}

BDPTIntegrator *CreateBDPTIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    bool visualizeStrategies = params.FindOneBool("visualizestrategies", false);
    bool visualizeWeights = params.FindOneBool("visualizeweights", false);

    if ((visualizeStrategies || visualizeWeights) && maxDepth > 5) {
        Warning(
            "visualizestrategies/visualizeweights was enabled, limiting "
            "maxdepth to 5");
        maxDepth = 5;
    }
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

    std::string lightStrategy = params.FindOneString("lightsamplestrategy",
                                                     "power");
    return new BDPTIntegrator(sampler, camera, maxDepth, visualizeStrategies,
                              visualizeWeights, pixelBounds, lightStrategy);
}
