
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

// integrators/bdpt.cpp*
#include "integrators/bdpt.h"
#include "film.h"
#include "sampler.h"
#include "integrator.h"
#include "stats.h"
#include "filters/box.h"
#include "paramset.h"
#include "progressreporter.h"

STAT_TIMER("Time/Rendering", renderingTime);

// BDPT Forward Declarations
int RandomWalk(const Scene &scene, RayDifferential ray, Sampler &sampler,
               MemoryArena &arena, Spectrum weight, Float pdfFwd, int maxDepth,
               TransportMode mode, Vertex *path);

// BDPT Utility Functions
Float ShadingNormalCorrection(const SurfaceInteraction &isect,
                              const Vector3f &wo, const Vector3f &wi,
                              TransportMode mode) {
    if (mode == TransportMode::Importance)
        return std::abs((Dot(wo, isect.shading.n) * Dot(wi, isect.n)) /
                        (Dot(wo, isect.n) * Dot(wi, isect.shading.n)));
    else
        return 1;
}

Float ConvertDensity(const Vertex &cur, Float pdf, const Vertex &next) {
    // Return solid angle density if _next_ is an infinite area light
    if (next.IsInfiniteLight()) return pdf;
    Vector3f d = next.p() - cur.p();
    Float invL2 = 1 / d.LengthSquared();
    if (next.IsOnSurface()) pdf *= AbsDot(next.ng(), d * std::sqrt(invL2));
    return pdf * invL2;
}

int GenerateCameraSubpath(const Scene &scene, Sampler &sampler,
                          MemoryArena &arena, int maxDepth,
                          const Camera &camera, Point2f &pFilm, Vertex *path) {
    if (maxDepth == 0) return 0;
    // Sample initial ray for camera subpath
    CameraSample cameraSample;
    cameraSample.pFilm = pFilm;
    cameraSample.time = sampler.Get1D();
    cameraSample.pLens = sampler.Get2D();
    RayDifferential ray;
    Spectrum rayWeight = camera.GenerateRayDifferential(cameraSample, &ray);
    ray.ScaleDifferentials(1 / std::sqrt(sampler.samplesPerPixel));

    // Generate first vertex on camera subpath and start random walk
    path[0] = Vertex(VertexType::Camera, EndpointInteraction(&camera, ray),
                     Spectrum(1.0f));
    return RandomWalk(scene, ray, sampler, arena, rayWeight,
                      camera.Pdf_Wi(path[0].ei, ray.d), maxDepth - 1,
                      TransportMode::Radiance, path + 1) +
           1;
}

int GenerateLightSubpath(const Scene &scene, Sampler &sampler,
                         MemoryArena &arena, int maxDepth, Float time,
                         const Distribution1D &lightDistr, Vertex *path) {
    if (maxDepth == 0) return 0;
    // Sample initial ray for light subpath
    Float lightPdf;
    int lightNum = lightDistr.SampleDiscrete(sampler.Get1D(), &lightPdf);
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    RayDifferential ray;
    Normal3f Nl;
    Float pdfPos, pdfDir;
    Spectrum Le = light->Sample_Le(sampler.Get2D(), sampler.Get2D(), time, &ray,
                                   &Nl, &pdfPos, &pdfDir);
    if (pdfPos == 0.f || pdfDir == 0.f || Le.IsBlack()) return 0;

    // Generate first vertex on light subpath and start random walk
    Spectrum weight = Le * AbsDot(Nl, ray.d) / (lightPdf * pdfPos * pdfDir);
    path[0] = Vertex(VertexType::Light,
                     EndpointInteraction(light.get(), ray, Nl), Le);
    path[0].pdfFwd = pdfPos * lightPdf;
    int nVertices =
        RandomWalk(scene, ray, sampler, arena, weight, pdfDir, maxDepth - 1,
                   TransportMode::Importance, path + 1);

    // Correct sampling densities for infinite area lights
    if (path[0].IsInfiniteLight()) {
        // Set spatial density of _path[1]_
        if (nVertices > 0) {
            path[1].pdfFwd = pdfPos;
            if (path[1].IsOnSurface())
                path[1].pdfFwd *= AbsDot(ray.d, path[1].ng());
        }

        // Set spatial density of _path[0]_
        path[0].pdfFwd = InfiniteLightDensity(scene, lightDistr, ray.d);
    }
    return nVertices + 1;
}

int RandomWalk(const Scene &scene, RayDifferential ray, Sampler &sampler,
               MemoryArena &arena, Spectrum weight, Float pdfFwd, int maxDepth,
               TransportMode mode, Vertex *path) {
    int bounces = 0;
    if (maxDepth == 0) return 0;
    while (true) {
        // Trace a ray and sample the medium, if any
        SurfaceInteraction isect;
        MediumInteraction mi;
        bool foundIntersection = scene.Intersect(ray, &isect);
        if (ray.medium) weight *= ray.medium->Sample(ray, sampler, arena, &mi);
        if (weight.IsBlack()) break;
        Vertex &vertex = path[bounces], &prev = path[bounces - 1];
        Float pdfRev;
        if (mi.IsValid()) {
            // Handle the medium case

            // Record medium interaction in _path_ and compute forward density
            vertex = Vertex(mi, weight);
            vertex.pdfFwd = ConvertDensity(prev, pdfFwd, vertex);
            if (++bounces >= maxDepth) break;

            // Sample direction and compute reverse density at preceding vertex
            Vector3f wi;
            pdfFwd = pdfRev = mi.phase->Sample_p(-ray.d, &wi, sampler.Get2D());
            ray = mi.SpawnRay(wi);
        } else {
            // Handle the surface case
            if (!foundIntersection) {
                // Capture escaped rays when tracing from the camera
                if (mode == TransportMode::Radiance) {
                    vertex = Vertex(VertexType::Light, EndpointInteraction(ray),
                                    weight);
                    vertex.pdfFwd = pdfFwd;
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

            // Fill _vertex_ with intersection information
            vertex = Vertex(isect, weight);
            vertex.pdfFwd = ConvertDensity(prev, pdfFwd, vertex);
            if (++bounces >= maxDepth) break;

            // Sample BSDF at current vertex and compute reverse probability
            Vector3f wi, wo = isect.wo;
            BxDFType flags;
            Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdfFwd,
                                              BSDF_ALL, &flags);
            if (f.IsBlack() || pdfFwd == 0.f) break;
            weight *= f * AbsDot(wi, isect.shading.n) / pdfFwd;
            pdfRev = isect.bsdf->Pdf(wi, wo, BSDF_ALL);
            if (flags & BSDF_SPECULAR) {
                vertex.delta = true;
                pdfRev = pdfFwd = 0;
            }
            weight *= ShadingNormalCorrection(isect, wo, wi, mode);
            ray = isect.SpawnRay(wi);
        }
        // Compute reverse area density at preceding vertex
        prev.pdfRev = ConvertDensity(vertex, pdfRev, prev);
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
    return g * vis.T(scene, sampler);
}

Float MISWeight(const Scene &scene, Vertex *lightVertices,
                Vertex *cameraVertices, Vertex &sampled, int s, int t,
                const Distribution1D &lightPdf) {
    if (s + t == 2) return 1.0f;
    Float sum = 0.f;
    // Define helper function _p_ that deals with Dirac delta functions
    auto p = [](float f) -> float { return f != 0 ? f : 1.0f; };

    // Temporarily update vertex properties for current strategy

    // Look up connection vertices and their predecessors
    Vertex *qs = s > 0 ? &lightVertices[s - 1] : nullptr,
           *pt = t > 0 ? &cameraVertices[t - 1] : nullptr,
           *qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr,
           *ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

    // Account for $s=1$ or $t=1$ strategy if applicable
    ScopedAssignment<Vertex> a1;
    if (s == 1)
        a1 = {qs, sampled};
    else if (t == 1)
        a1 = {pt, sampled};

    // Mark connection vertices as non-degenerate
    ScopedAssignment<bool> a2, a3;
    if (pt) a2 = {&pt->delta, false};
    if (qs) a3 = {&qs->delta, false};

    // Update reverse density of vertex $\pt{t}$
    ScopedAssignment<Float> a4;
    if (pt)
        a4 = {&pt->pdfRev, s > 0
                               ? qs->Pdf(scene, qsMinus, *pt)
                               : pt->PdfLightOrigin(scene, *ptMinus, lightPdf)};

    // Update reverse density of vertex $\pt{t-1}$
    ScopedAssignment<Float> a5;
    if (ptMinus)
        a5 = {&ptMinus->pdfRev, s > 0 ? pt->Pdf(scene, qs, *ptMinus)
                                      : pt->PdfLight(scene, *ptMinus)};

    // Update reverse density of vertex $\pq{s}$
    ScopedAssignment<Float> a6;
    if (qs) a6 = {&qs->pdfRev, pt->Pdf(scene, ptMinus, *qs)};

    // Update reverse density of vertex $\pq{s-1}$
    ScopedAssignment<Float> a7;
    if (qsMinus) a7 = {&qsMinus->pdfRev, qs->Pdf(scene, pt, *qsMinus)};

    // Consider hypothetical connection strategies along the camera subpath
    Float ratio = 1.0f;
    for (int i = t - 1; i > 0; --i) {
        ratio *= p(cameraVertices[i].pdfRev) / p(cameraVertices[i].pdfFwd);
        if (!cameraVertices[i].delta && !cameraVertices[i - 1].delta)
            sum += ratio;
    }

    // Consider hypothetical connection strategies along the light subpath
    ratio = 1.0f;
    for (int i = s - 1; i >= 0; --i) {
        ratio *= p(lightVertices[i].pdfRev) / p(lightVertices[i].pdfFwd);
        bool deltaLightvertex = i > 0 ? lightVertices[i - 1].delta
                                      : lightVertices[0].IsDeltaLight();
        if (!lightVertices[i].delta && !deltaLightvertex) sum += ratio;
    }
    return 1.f / (1.f + sum);
}

// BDPT Method Definitions
inline int BufferIndex(int s, int t) {
    int above = s + t - 2;
    return s + above * (5 + above) / 2;
}

void BDPTIntegrator::Render(const Scene &scene) {
    // Compute _lightDistr_ for sampling lights proportional to power
    std::unique_ptr<Distribution1D> lightDistr =
        ComputeLightPowerDistribution(scene);

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

                char filename[32];
                snprintf(filename, sizeof(filename),
                         "bdpt_d%02i_s%02i_t%02i.exr", depth, s, t);

                weightFilms[BufferIndex(s, t)] = std::unique_ptr<Film>(
                    new Film(film->fullResolution,
                             Bounds2f(Point2f(0, 0), Point2f(1, 1)),
                             CreateBoxFilter(ParamSet()), film->diagonal * 1000,
                             filename, 1.f, 2.2f));
            }
        }
    }

    // Render and write the output image to disk
    {
        StatTimer timer(&renderingTime);
        ParallelFor([&](const Point2i tile) {
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
            std::vector<std::unique_ptr<FilmTile>> weightFilmTiles;
            if (visualizeStrategies || visualizeWeights)
                for (auto &wFilm : weightFilms)
                    weightFilmTiles.emplace_back(
                        wFilm->GetFilmTile(tileBounds));
            for (Point2i pPixel : tileBounds) {
                tileSampler->StartPixel(pPixel);
                do {
                    // Generate a single sample using BDPT
                    Point2f pFilm = (Point2f)pPixel + tileSampler->Get2D();

                    // Trace the light and camera subpaths
                    Vertex *cameraVertices = arena.Alloc<Vertex>(maxDepth + 2);
                    Vertex *lightVertices = arena.Alloc<Vertex>(maxDepth + 1);
                    int nCamera = GenerateCameraSubpath(
                        scene, *tileSampler, arena, maxDepth + 2, *camera,
                        pFilm, cameraVertices);
                    int nLight = GenerateLightSubpath(
                        scene, *tileSampler, arena, maxDepth + 1,
                        cameraVertices[0].time(), *lightDistr, lightVertices);

                    // Execute all BDPT connection strategies
                    Spectrum L(0.f);
                    for (int t = 1; t <= nCamera; ++t) {
                        for (int s = 0; s <= nLight; ++s) {
                            int depth = t + s - 2;
                            if ((s == 1 && t == 1) || depth < 0 ||
                                depth > maxDepth)
                                continue;
                            // Execute the $(s, t)$ connection strategy
                            Point2f pFilmNew = pFilm;
                            Float misWeight = 0.f;
                            Spectrum Lpath = ConnectBDPT(
                                scene, lightVertices, cameraVertices, s, t,
                                *lightDistr, *camera, *tileSampler, &pFilmNew,
                                &misWeight);
                            if (visualizeStrategies || visualizeWeights) {
                                Spectrum value;
                                if (visualizeStrategies)
                                    value = Lpath / misWeight;
                                if (visualizeWeights) value = Lpath;
                                weightFilmTiles[BufferIndex(s, t)]->AddSplat(
                                    pFilmNew, value);
                            }
                            if (t != 1)
                                L += Lpath;
                            else
                                filmTile->AddSplat(pFilmNew, Lpath);
                        }
                    }
                    filmTile->AddSample(pFilm, L);
                    arena.Reset();
                } while (tileSampler->StartNextSample());
            }
            film->MergeFilmTile(std::move(filmTile));
            for (size_t i = 0; i < weightFilmTiles.size(); ++i)
                weightFilms[i]->MergeFilmTile(std::move(weightFilmTiles[i]));
            reporter.Update();
        }, Point2i(nXTiles, nYTiles));
        reporter.Done();
    }
    film->WriteImage(1.0f / sampler->samplesPerPixel);

    // Write buffers for debug visualization
    if (visualizeStrategies || visualizeWeights) {
        const Float invSampleCount = 1.0f / sampler->samplesPerPixel;
        for (size_t i = 0; i < weightFilms.size(); ++i) {
            if (weightFilms[i]) weightFilms[i]->WriteImage(invSampleCount);
        }
    }
}

Spectrum ConnectBDPT(const Scene &scene, Vertex *lightVertices,
                     Vertex *cameraVertices, int s, int t,
                     const Distribution1D &lightDistr, const Camera &camera,
                     Sampler &sampler, Point2f *pFilm, Float *misWeightPtr) {
    Spectrum L(0.f);
    // Ignore invalid connections related to infinite area lights
    if (t > 1 && s != 0 && cameraVertices[t - 1].type == VertexType::Light)
        return Spectrum(0.f);

    // Perform connection and write contribution to _weight_
    Vertex sampled;
    if (s == 0) {
        // Interpret the camera subpath as a complete path
        const Vertex &pt = cameraVertices[t - 1];
        if (pt.IsLight()) L = pt.Le(scene, cameraVertices[t - 2]) * pt.weight;
    } else if (t == 1) {
        // Sample a point on the camera and connect it to the light subpath
        const Vertex &qs = lightVertices[s - 1];
        if (qs.IsConnectible()) {
            VisibilityTester vis;
            Vector3f wi;
            Float pdf;
            Spectrum Wi = camera.Sample_Wi(qs.GetInteraction(), sampler.Get2D(),
                                           &wi, &pdf, pFilm, &vis);
            if (pdf > 0 && !Wi.IsBlack()) {
                // Initialize dynamically sampled vertex and _L_ for $t=1$ case
                sampled =
                    Vertex(VertexType::Camera,
                           EndpointInteraction(vis.P1(), &camera), Wi / pdf);
                L = qs.weight * qs.f(sampled) * vis.T(scene, sampler) *
                    sampled.weight;
                if (qs.IsOnSurface()) L *= AbsDot(wi, qs.ns());
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
                sampled = Vertex(VertexType::Light,
                                 EndpointInteraction(vis.P1(), light.get()),
                                 lightWeight / (pdf * lightPdf));
                sampled.pdfFwd = sampled.PdfLightOrigin(scene, pt, lightDistr);
                L = pt.weight * pt.f(sampled) * vis.T(scene, sampler) *
                    sampled.weight;
                if (pt.IsOnSurface()) L *= AbsDot(wi, pt.ns());
            }
        }
    } else {
        // Handle all other bidirectional connection cases
        const Vertex &qs = lightVertices[s - 1], &pt = cameraVertices[t - 1];
        if (qs.IsConnectible() && pt.IsConnectible())
            L = qs.weight * qs.f(pt) * G(scene, sampler, qs, pt) * pt.f(qs) *
                pt.weight;
    }

    // Compute MIS weight for connection strategy
    Float misWeight =
        L.IsBlack() ? 0.f : MISWeight(scene, lightVertices, cameraVertices,
                                      sampled, s, t, lightDistr);
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

    return new BDPTIntegrator(sampler, camera, maxDepth, visualizeStrategies,
                              visualizeWeights);
}
