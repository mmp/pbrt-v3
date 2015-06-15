
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_BDPT_H
#define PBRT_INTEGRATORS_BDPT_H
#include "stdafx.h"

// integrators/bdpt.h*
#include "pbrt.h"
#include "integrator.h"
#include "interaction.h"
#include "light.h"
#include "scene.h"
#include "reflection.h"
#include "camera.h"
#include "sampling.h"

// EndpointInteraction Declarations
struct EndpointInteraction : Interaction {
    union {
        const Camera *camera;
        const Light *light;
    };
    // EndpointInteraction Public Methods
    EndpointInteraction() : Interaction(), light(nullptr) {}
    EndpointInteraction(const Interaction &it, const Camera *camera)
        : Interaction(it), camera(camera) {}
    EndpointInteraction(const Camera *camera, const Ray &ray)
        : Interaction(ray.o, ray.time, ray.medium), camera(camera) {}
    EndpointInteraction(const Light *light, const Ray &r, const Normal3f &_n)
        : Interaction(r.o, r.time, r.medium), light(light) {
        n = _n;
    }
    EndpointInteraction(const Interaction &it, const Light *light)
        : Interaction(it), light(light) {}
};

// BDPT Declarations
class BDPTIntegrator : public Integrator {
  public:
    // BDPTIntegrator Public Methods
    BDPTIntegrator(std::shared_ptr<Sampler> sampler,
                   std::shared_ptr<const Camera> camera, int maxdepth,
                   bool visualize_strategies, bool visualize_weights)
        : sampler(sampler),
          camera(camera),
          maxdepth(maxdepth),
          visualize_strategies(visualize_strategies),
          visualize_weights(visualize_weights){};
    void Render(const Scene &scene);

  private:
    // BDPTIntegrator Private Data
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<const Camera> camera;
    int maxdepth;
    bool visualize_strategies;
    bool visualize_weights;
};

BDPTIntegrator *CreateBDPTIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera);
enum class VertexType { Camera, Light, Surface, Medium };

struct Vertex {
    // Vertex Public Data
    VertexType type;
    Spectrum weight;
    Float pdfFwd = 0.f;
    Float pdfRev = 0.f;
    bool delta = false;
#ifdef NDEBUG
    union
#else
    struct
#endif
        {
        EndpointInteraction ei;
        MediumInteraction mi;
        SurfaceInteraction isect;
    };

    // Vertex Public Methods
    Vertex() : ei() {}
    Vertex(VertexType type, const EndpointInteraction &ei,
           const Spectrum &weight)
        : type(type), weight(weight), ei(ei) {}
    Vertex(const SurfaceInteraction &isect, const Spectrum &weight)
        : type(VertexType::Surface), weight(weight), isect(isect) {}
    Vertex(const MediumInteraction &mi, const Spectrum &weight)
        : type(VertexType::Medium), weight(weight), mi(mi) {}
    const Interaction &GetInteraction() const {
        switch (type) {
        case VertexType::Medium:
            return mi;
        case VertexType::Surface:
            return isect;
        default:
            return ei;
        };
    }
    const Point3f &GetPosition() const { return GetInteraction().p; }
    Float GetTime() const { return GetInteraction().time; }
    const Normal3f &GetNormal() const {
        if (type == VertexType::Surface)
            return isect.shading.n;
        else
            return GetInteraction().n;
    }
    Float Pdf(const Vertex *prev, const Vertex &next) const {
        Float pdf, tmp;
        // Compute directions and inverse squared distance
        Vector3f wp;
        if (prev) wp = Normalize(prev->GetPosition() - GetPosition());
        Vector3f wn = next.GetPosition() - GetPosition();
        Float invL2 = 1 / wn.LengthSquared();
        wn *= std::sqrt(invL2);

        // Compute directional density for different vertex types
        switch (type) {
        case VertexType::Camera:
            pdf = ei.camera->Pdf(ei, wn);
            break;
        case VertexType::Light:
            ei.light->Pdf(Ray(GetPosition(), wn, GetTime()), GetNormal(), &tmp,
                          &pdf);
            break;
        case VertexType::Surface:
            pdf = isect.bsdf->Pdf(wp, wn);
            break;
        case VertexType::Medium:
            pdf = mi.phase->p(wp, wn);
            break;
        default:
            Error("Unimplemented");
            return 0.0f;
        }

        // Convert to probability per solid angle at vertex _next_
        if (next.IsOnSurface()) pdf *= AbsDot(next.GetNormal(), wn);
        return pdf * invL2;
    }
    bool IsOnSurface() const { return GetNormal() != Normal3f(); }
    Spectrum f(const Vertex &next) const {
        Vector3f wi = Normalize(next.GetPosition() - GetPosition());
        switch (type) {
        case VertexType::Surface:
            return isect.bsdf->f(isect.wo, wi);
        case VertexType::Medium:
            return mi.phase->p(mi.wo, wi);
        default:
            Error("Unimplemented");
            return Spectrum(0.0f);
        }
    }
    Float PdfLightArea(const Vertex &v, const Distribution1D &lightPdf) const {
        if (!IsLight()) return 0.f;
        const Light *light = type == VertexType::Light
                                 ? ei.light
                                 : isect.primitive->GetAreaLight();
        Vector3f d = Normalize(v.GetPosition() - GetPosition());
        Float unused, pdfPos, pdfChoice = light->Power().y() /
                                          (lightPdf.funcInt * lightPdf.Count());
        light->Pdf(Ray(GetPosition(), d, GetTime()), GetNormal(), &pdfPos,
                   &unused);
        return pdfPos * pdfChoice;
    }
    Float PdfLightDir(const Vertex &v) const {
        if (!IsLight()) return 0.f;
        const Light *light = type == VertexType::Light
                                 ? ei.light
                                 : isect.primitive->GetAreaLight();

        Vector3f d = v.GetPosition() - GetPosition();
        Float invL2 = 1 / d.LengthSquared();
        d *= std::sqrt(invL2);

        Float unused, pdf;
        light->Pdf(Ray(GetPosition(), d, GetTime()), GetNormal(), &unused,
                   &pdf);
        pdf *= invL2;
        if (v.IsOnSurface()) pdf *= AbsDot(v.GetNormal(), d);
        return pdf;
    }
    bool IsConnectable() const {
        switch (type) {
        case VertexType::Surface:
            return isect.bsdf->NumComponents(
                       BxDFType(BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_REFLECTION |
                                BSDF_TRANSMISSION)) > 0;
        default:
            return true;
        };
    }
    bool IsLight() const {
        return type == VertexType::Light ||
               (type == VertexType::Surface &&
                isect.primitive->GetAreaLight() != nullptr);
    }
    bool IsDeltaLight() const {
        if (!IsLight()) return false;
        const Light *light = (type == VertexType::Light)
                                 ? ei.light
                                 : isect.primitive->GetAreaLight();
        return ::IsDeltaLight(light->flags);
    }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v) {
        os << "Vertex[" << std::endl << "  type = ";
        switch (v.type) {
        case VertexType::Camera:
            os << "camera";
            break;
        case VertexType::Light:
            os << "light";
            break;
        case VertexType::Surface:
            os << "surface";
            break;
        case VertexType::Medium:
            os << "medium";
            break;
        }
        os << "," << std::endl
           << "  connectable = " << v.IsConnectable() << "," << std::endl
           << "  p = " << v.GetPosition() << "," << std::endl
           << "  n = " << v.GetNormal() << "," << std::endl
           << "  pdfFwd = " << v.pdfFwd << "," << std::endl
           << "  pdfRev = " << v.pdfRev << "," << std::endl
           << "  weight = " << v.weight << std::endl
           << "]" << std::endl;
        return os;
    };
};

extern int GenerateCameraSubpath(const Scene &scene, Sampler &sampler,
                                 MemoryArena &arena, int maxdepth,
                                 const Camera &camera, Point2f &samplePos,
                                 Vertex *path);

extern int GenerateLightSubpath(const Scene &scene, Sampler &sampler,
                                MemoryArena &arena, int maxdepth, Float time,
                                const Distribution1D &lightDistribution,
                                Vertex *path);

extern Spectrum ConnectBDPT(const Scene &scene, Vertex *lightSubpath,
                            Vertex *cameraSubpath, int s, int t,
                            const Distribution1D &lightDistribution,
                            const Camera *camera, Sampler &sampler,
                            Point2f *samplePos, Float *misWeight);

#endif  // PBRT_INTEGRATORS_BDPT_H
