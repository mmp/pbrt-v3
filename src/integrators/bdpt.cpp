
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

using std::cout;
using std::endl;

STAT_TIMER("Time/Rendering", renderingTime);

// BDPT Infrastructure
Float ShadingNormalCorrection(const SurfaceInteraction &isect,
                              const Vector3f &wo, const Vector3f &wi,
                              TransportMode mode) {
    if (mode == TransportMode::Importance)
        return std::abs((Dot(wo, isect.shading.n) * Dot(wi, isect.n)) /
                        (Dot(wo, isect.n) * Dot(wi, isect.shading.n)));
    else
        return 1.f;
}

Float ConvertDensity(const Vertex &cur, Float pdf, const Vertex &next) {
    // Return solid angle density if _next_ is an infinite area light
    if (next.IsInfiniteLight()) return pdf;
    Vector3f d = next.GetPosition() - cur.GetPosition();
    Float invL2 = 1.f / d.LengthSquared();
    if (next.IsOnSurface())
        pdf *= AbsDot(next.GetGeoNormal(), d * std::sqrt(invL2));
    return pdf * invL2;
}

int RandomWalk(const Scene &scene, RayDifferential ray, Sampler &sampler,
               MemoryArena &arena, Spectrum weight, Float pdfFwd, int maxdepth,
               TransportMode mode, Vertex *path) {
    int bounces = 0;
    if (maxdepth == 0) return 0;
    SurfaceInteraction isect;
    MediumInteraction mi;
    while (true) {
        // Trace a ray and sample the medium, if any
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
            if (++bounces >= maxdepth) break;

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
            if (++bounces >= maxdepth) break;

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

int GenerateCameraSubpath(const Scene &scene, Sampler &sampler,
                          MemoryArena &arena, int maxdepth,
                          const Camera &camera, Point2f &samplePos,
                          Vertex *path) {
    if (maxdepth == 0) return 0;
    // Sample initial ray for camera subpath
    CameraSample cameraSample;
    cameraSample.pFilm = samplePos;
    cameraSample.time = sampler.Get1D();
    cameraSample.pLens = sampler.Get2D();
    RayDifferential ray;
    Spectrum rayWeight(camera.GenerateRayDifferential(cameraSample, &ray));
    ray.ScaleDifferentials(1.f / std::sqrt(sampler.samplesPerPixel));

    // Generate first vertex on camera subpath and start random walk
    path[0] = Vertex(VertexType::Camera, EndpointInteraction(&camera, ray),
                     Spectrum(1.0f));
    return RandomWalk(scene, ray, sampler, arena, rayWeight,
                      camera.Pdf(path[0].interaction.ei, ray.d), maxdepth - 1,
                      TransportMode::Radiance, path + 1) +
           1;
}

int GenerateLightSubpath(const Scene &scene, Sampler &sampler,
                         MemoryArena &arena, int maxdepth, Float time,
                         const Distribution1D &lightDistribution,
                         Vertex *path) {
    if (maxdepth == 0) return 0;
    // Sample initial ray for light subpath
    Float lightPdf;
    int lightNum = lightDistribution.SampleDiscrete(sampler.Get1D(), &lightPdf);
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    RayDifferential ray;
    Normal3f Nl;
    Float pdfPos, pdfDir;
    Spectrum Le = light->Sample_L(sampler.Get2D(), sampler.Get2D(), time, &ray,
                                  &Nl, &pdfPos, &pdfDir);
    if (pdfPos == 0.f || pdfDir == 0.f || Le.IsBlack()) return 0;

    // Generate first vertex on light subpath and start random walk
    Spectrum weight = Le * AbsDot(Nl, ray.d) / (lightPdf * pdfPos * pdfDir);
    path[0] = Vertex(VertexType::Light,
                     EndpointInteraction(light.get(), ray, Nl), Le);
    path[0].pdfFwd = pdfPos * lightPdf;
    int nvertices =
        RandomWalk(scene, ray, sampler, arena, weight, pdfDir, maxdepth - 1,
                   TransportMode::Importance, path + 1);

    // Correct sampling densities for infinite area lights
    if (path[0].IsInfiniteLight()) {
        path[0].pdfFwd = InfiniteLightDensity(scene, lightDistribution, ray.d);
        if (nvertices > 0) {
            path[1].pdfFwd = pdfPos;
            if (path[1].IsOnSurface())
                path[1].pdfFwd *= AbsDot(ray.d, path[1].GetGeoNormal());
        }
    }
    return nvertices + 1;
}

Spectrum GeometryTerm(const Scene &scene, Sampler &sampler, const Vertex &v0,
                      const Vertex &v1) {
    Vector3f d = v0.GetPosition() - v1.GetPosition();
    Float G = 1.f / d.LengthSquared();
    d *= std::sqrt(G);
    if (v0.IsOnSurface()) G *= Dot(v0.GetShadingNormal(), d);
    if (v1.IsOnSurface()) G *= Dot(v1.GetShadingNormal(), d);
    VisibilityTester vis(v0.GetInteraction(), v1.GetInteraction());
    return std::abs(G) * vis.T(scene, sampler);
}

Float MISWeight(const Scene &scene, Vertex *lightSubpath, Vertex *cameraSubpath,
                Vertex &sampled, int s, int t, const Distribution1D &lightPdf) {
    if (s + t == 2) return 1.0f;
    Float sum = 0.f;
    // Determine connection vertices
    Vertex *vs = s > 0 ? &lightSubpath[s - 1] : nullptr,
           *vt = t > 0 ? &cameraSubpath[t - 1] : nullptr,
           *vsPred = s > 1 ? &lightSubpath[s - 2] : nullptr,
           *vtPred = t > 1 ? &cameraSubpath[t - 2] : nullptr;

    // Temporarily update vertex properties for current strategy
    ScopedAssign<Vertex> s0;
    ScopedAssign<bool> s1, s2;
    ScopedAssign<Float> s3, s4, s5, s6;
    if (s == 1 || t == 1)
        s0 = ScopedAssign<Vertex>((s == 1) ? vs : vt, sampled);
    if (vs) {
        s1 = ScopedAssign<bool>(&vs->delta, false),
        s3 = ScopedAssign<Float>(&vs->pdfRev, vt->Pdf(scene, vtPred, *vs));
    }
    if (vsPred)
        s5 = ScopedAssign<Float>(&vsPred->pdfRev, vs->Pdf(scene, vt, *vsPred));
    if (vt) {
        s2 = ScopedAssign<bool>(&vt->delta, false);
        s4 = ScopedAssign<Float>(
            &vt->pdfRev, s > 0 ? vs->Pdf(scene, vsPred, *vt)
                               : vt->PdfLightOrigin(scene, *vtPred, lightPdf));
    }
    if (vtPred)
        s6 = ScopedAssign<Float>(
            &vtPred->pdfRev,
            s > 0 ? vt->Pdf(scene, vs, *vtPred) : vt->PdfLight(scene, *vtPred));

    // Consider hypothetical connection strategies along the camera subpath
    auto p = [](float f) -> float { return f != 0 ? f : 1.0f; };
    Float ratio = 1.0f;
    for (int i = t - 1; i > 0; --i) {
        ratio *= p(cameraSubpath[i].pdfRev) / p(cameraSubpath[i].pdfFwd);
        if (!cameraSubpath[i].delta && !cameraSubpath[i - 1].delta)
            sum += ratio;
    }

    // Consider hypothetical connection strategies along the light subpath
    ratio = 1.0f;
    for (int i = s - 1; i >= 0; --i) {
        ratio *= p(lightSubpath[i].pdfRev) / p(lightSubpath[i].pdfFwd);
        bool delta_lightvertex =
            i > 0 ? lightSubpath[i - 1].delta : lightSubpath[0].IsDeltaLight();
        if (!lightSubpath[i].delta && !delta_lightvertex) sum += ratio;
    }
    return 1.f / (1.f + sum);
}

// BDPT Method Definitions
inline int BufferIndex(int s, int t) {
    int above = s + t - 2;
    return s + above * (5 + above) / 2;
}

void BDPTIntegrator::Render(const Scene &scene) {
    // Compute _lightDistribution_ for sampling lights proportional to power
    std::unique_ptr<Distribution1D> lightDistribution(
        ComputeLightSamplingCDF(scene));

    // Partition the image into buckets
    Film *film = camera->film;
    const Bounds2i sampleBounds = film->GetSampleBounds();
    const Vector2i sampleExtent = sampleBounds.Diagonal();
    const int bucketSize = 16;
    const int nXBuckets = (sampleExtent.x + bucketSize - 1) / bucketSize;
    const int nYBuckets = (sampleExtent.y + bucketSize - 1) / bucketSize;
    ProgressReporter reporter(nXBuckets * nYBuckets, "Rendering");

    // Allocate buffers for debug visualization
    const int bufferCount = (1 + maxdepth) * (6 + maxdepth) / 2;
    std::vector<std::unique_ptr<Film>> films(bufferCount);
    if (visualize_strategies || visualize_weights) {
        for (int depth = 0; depth <= maxdepth; ++depth) {
            for (int s = 0; s <= depth + 2; ++s) {
                int t = depth + 2 - s;
                if (t == 0 || (s == 1 && t == 1)) continue;

                char filename[32];
                snprintf(filename, sizeof(filename),
                         "bdpt_d%02i_s%02i_t%02i.exr", depth, s, t);

                films[BufferIndex(s, t)] = std::unique_ptr<Film>(new Film(
                    film->fullResolution,
                    Bounds2f(Point2f(0, 0), Point2f(1, 1)),
                    CreateBoxFilter(ParamSet()),
                    film->diagonal * 1000,  // XXX what does this parameter
                                            // mean? Why the multiplication?
                    filename, 1.f, 2.2f));
            }
        }
    }

    // Render and write the output image to disk
    {
        StatTimer timer(&renderingTime);
        ParallelFor([&](const Point2i bucket) {
            // Render a single bucket using BDPT
            MemoryArena arena;
            int seed = bucket.y * nXBuckets + bucket.x;
            std::unique_ptr<Sampler> bucketSampler = sampler->Clone(seed);
            int x0 = sampleBounds.pMin.x + bucket.x * bucketSize;
            int x1 = std::min(x0 + bucketSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + bucket.y * bucketSize;
            int y1 = std::min(y0 + bucketSize, sampleBounds.pMax.y);
            Bounds2i bucketBounds(Point2i(x0, y0), Point2i(x1, y1));
            std::unique_ptr<FilmTile> filmTile =
                camera->film->GetFilmTile(bucketBounds);
            for (Point2i pixel : bucketBounds) {
                bucketSampler->StartPixel(pixel);
                do {
                    // Generate a single sample using BDPT
                    Point2f samplePos((Float)pixel.x, (Float)pixel.y);
                    samplePos += bucketSampler->Get2D();

                    // Trace the light and camera subpaths
                    Vertex *cameraSubpath =
                        (Vertex *)arena.Alloc<Vertex>(maxdepth + 2);
                    Vertex *lightSubpath =
                        (Vertex *)arena.Alloc<Vertex>(maxdepth + 1);
                    int nCamera = GenerateCameraSubpath(
                        scene, *bucketSampler, arena, maxdepth + 2, *camera,
                        samplePos, cameraSubpath);
                    int nLight = GenerateLightSubpath(
                        scene, *bucketSampler, arena, maxdepth + 1,
                        cameraSubpath[0].GetTime(), *lightDistribution,
                        lightSubpath);

                    // Execute all connection strategies
                    Spectrum pixelWeight(0.f);
                    for (int t = 1; t <= nCamera; ++t) {
                        for (int s = 0; s <= nLight; ++s) {
                            int depth = t + s - 2;
                            if ((s == 1 && t == 1) || depth < 0 ||
                                depth > maxdepth)
                                continue;
                            // Execute the $(s, t)$ connection strategy
                            Point2f finalSamplePos = samplePos;
                            Float misWeight = 0.f;
                            Spectrum weight = ConnectBDPT(
                                scene, lightSubpath, cameraSubpath, s, t,
                                *lightDistribution, camera.get(),
                                *bucketSampler, &finalSamplePos, &misWeight);
                            if (visualize_strategies || visualize_weights) {
                                Spectrum value(1.0f);
                                if (visualize_strategies) value *= weight;
                                if (visualize_weights) value *= misWeight;
                                films[BufferIndex(s, t)]->Splat(finalSamplePos,
                                                                value);
                            }
                            if (t != 1)
                                pixelWeight += weight * misWeight;
                            else
                                film->Splat(finalSamplePos, weight * misWeight);
                        }
                    }
                    filmTile->AddSample(samplePos, pixelWeight, 1.0f);
                    arena.Reset();
                } while (bucketSampler->StartNextSample());
            }
            film->MergeFilmTile(std::move(filmTile));
            reporter.Update();
        }, Point2i(nXBuckets, nYBuckets));
        reporter.Done();
    }
    film->WriteImage(1.0f / sampler->samplesPerPixel);

    // Write buffers for debug visualization
    if (visualize_strategies || visualize_weights) {
        const Float invSampleCount = 1.0f / sampler->samplesPerPixel;
        for (size_t i = 0; i < films.size(); ++i) {
            if (films[i]) films[i]->WriteImage(invSampleCount);
        }
    }
}

Spectrum ConnectBDPT(const Scene &scene, Vertex *lightSubpath,
                     Vertex *cameraSubpath, int s, int t,
                     const Distribution1D &lightDistribution,
                     const Camera *camera, Sampler &sampler, Point2f *samplePos,
                     Float *misWeight) {
    Spectrum weight(0.f);
    Vertex sampled;
    if (t > 1 && s != 0 && cameraSubpath[t - 1].type == VertexType::Light) {
        /* ignore */
    } else if (s == 0) {
        // Directly connect a camera subpath to an intersected light source
        const Vertex &vt = cameraSubpath[t - 1];
        if (vt.IsLight()) {
            Spectrum Le;
            if (vt.type == VertexType::Surface) {
                Le = vt.interaction.isect.primitive->GetAreaLight()->L(vt.interaction.isect,
                                                                       vt.interaction.isect.wo);
            } else {
                for (const auto &light : scene.lights)
                    Le += light->Le(
                        Ray(vt.GetPosition(), -Vector3f(vt.GetGeoNormal())));
            }
            weight = Le * vt.weight;
        }
    } else if (s == 1) {
        // Sample a point on a light source and connect it to the camera subpath
        const Vertex &vt = cameraSubpath[t - 1];
        if (vt.IsConnectable()) {
            Float lightPdf;
            VisibilityTester vis;
            Vector3f wi;
            Float pdf;
            int lightNum =
                lightDistribution.SampleDiscrete(sampler.Get1D(), &lightPdf);
            const std::shared_ptr<Light> &light = scene.lights[lightNum];
            Spectrum lightWeight = light->Sample_L(
                vt.GetInteraction(), sampler.Get2D(), &wi, &pdf, &vis);
            if (pdf > 0 && !lightWeight.IsBlack()) {
                sampled = Vertex(VertexType::Light,
                                 EndpointInteraction(vis.P1(), light.get()),
                                 lightWeight / (pdf * lightPdf));
                sampled.pdfFwd =
                    sampled.PdfLightOrigin(scene, vt, lightDistribution);
                weight = vt.weight * vt.f(sampled) *
                         AbsDot(wi, vt.GetShadingNormal()) *
                         vis.T(scene, sampler) * sampled.weight;
            }
        }
    } else if (t == 1) {
        // Sample a point on a camera and connect it to the light subpath
        const Vertex &vs = lightSubpath[s - 1];
        if (vs.IsConnectable()) {
            VisibilityTester vis;
            Vector3f wi;
            Float pdf;
            Spectrum cameraWeight =
                camera->Sample_We(vs.GetInteraction(), sampler.Get2D(), &wi,
                                  &pdf, samplePos, &vis);
            if (pdf > 0 && !cameraWeight.IsBlack()) {
                sampled =
                    Vertex(VertexType::Camera,
                           EndpointInteraction(vis.P1(), camera), cameraWeight);
                weight = vs.weight * vs.f(sampled) *
                         AbsDot(wi, vs.GetShadingNormal()) *
                         vis.T(scene, sampler) * sampled.weight;
            }
        }
    } else {
        // Handle all other cases
        const Vertex &vs = lightSubpath[s - 1], &vt = cameraSubpath[t - 1];
        if (vs.IsConnectable() && vt.IsConnectable()) {
            weight = vs.weight * vs.f(vt) *
                     GeometryTerm(scene, sampler, vs, vt) * vt.f(vs) *
                     vt.weight;
        }
    }
    *misWeight =
        weight.IsBlack() ? 0.f : MISWeight(scene, lightSubpath, cameraSubpath,
                                           sampled, s, t, lightDistribution);
    return weight;
}

BDPTIntegrator *CreateBDPTIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera) {
    int maxdepth = params.FindOneInt("maxdepth", 5);
    bool visualize_strategies =
        params.FindOneBool("visualize_strategies", false);
    bool visualize_weights = params.FindOneBool("visualize_weights", false);

    if ((visualize_strategies || visualize_weights) && maxdepth > 5) {
        Warning(
            "visualize_strategies/visualize_weights was enabled, limiting "
            "maxdepth to 5");
        maxdepth = 5;
    }

    return new BDPTIntegrator(sampler, camera, maxdepth, visualize_strategies,
                              visualize_weights);
}
