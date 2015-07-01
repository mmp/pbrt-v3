
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
    EndpointInteraction(const Ray &ray)
        : Interaction(ray(1.f), ray.time, ray.medium), light(nullptr) {
        n = Normal3f(-ray.d);
    }
};

// BDPT Helper Definitions
struct Vertex;
Float ConvertDensity(const Vertex &cur, Float pdfDir, const Vertex &next);
template <typename Type>
class ScopedAssignment {
  public:
    // ScopedAssignment Public Methods
    ScopedAssignment(Type *ptr = nullptr, Type value = Type()) : target(ptr) {
        if (target) {
            backup = *target;
            *target = value;
        }
    }
    ~ScopedAssignment() {
        if (target) *target = backup;
    }
    ScopedAssignment(const ScopedAssignment &) = delete;
    ScopedAssignment &operator=(const ScopedAssignment &) = delete;
    ScopedAssignment &operator=(ScopedAssignment &&other) {
        target = other.target;
        backup = other.backup;
        other.target = nullptr;
        return *this;
    }

  private:
    Type *target, backup;
};

inline Float InfiniteLightDensity(const Scene &scene,
                                  const Distribution1D &lightDistr,
                                  const Vector3f &d) {
    Float pdf = 0;
    for (size_t i = 0; i < scene.lights.size(); ++i)
        if (scene.lights[i]->flags == LightFlags::Infinite)
            pdf += scene.lights[i]->Pdf(Interaction(), -d) * lightDistr.func[i];
    return pdf / (lightDistr.funcInt * lightDistr.Count());
}

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
          visualize_weights(visualize_weights) {}
    void Render(const Scene &scene);

  private:
    // BDPTIntegrator Private Data
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<const Camera> camera;
    int maxdepth;
    bool visualize_strategies;
    bool visualize_weights;
};

enum class VertexType { Camera = 0, Light, Surface, Medium };

struct Vertex {
    // Vertex Public Data
    VertexType type;
    Spectrum weight;
    Float pdfFwd = 0.f;
    Float pdfRev = 0.f;
    bool delta = false;
// Switch to a struct in debug mode to avoid a compiler error regarding
// non-trivial constructors
#if defined(NDEBUG) && !defined(PBRT_IS_MSVC) && !defined(PBRT_IS_INTEL)
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
    const Normal3f &GetGeoNormal() const { return GetInteraction().n; }
    const Normal3f &GetShadingNormal() const {
        if (type == VertexType::Surface)
            return isect.shading.n;
        else
            return GetInteraction().n;
    }
    Float Pdf(const Scene &scene, const Vertex *prev,
              const Vertex &next) const {
        if (type == VertexType::Light) return PdfLight(scene, next);
        // Compute directions to preceding and next vertex
        Vector3f wp, wn = Normalize(next.GetPosition() - GetPosition());
        if (prev) wp = Normalize(prev->GetPosition() - GetPosition());

        // Compute directional density depending on the vertex types
        Float pdf;
        switch (type) {
        case VertexType::Camera:
            pdf = ei.camera->Pdf(ei, wn);
            break;
        case VertexType::Surface:
            pdf = isect.bsdf->Pdf(wp, wn);
            break;
        case VertexType::Medium:
            pdf = mi.phase->p(wp, wn);
            break;
        default:
            Error("Vertex::Pdf(): Unimplemented");
            return 0.f;
        }

        // Convert to probability per unit area at vertex _next_
        return ConvertDensity(*this, pdf, next);
    }
    bool IsOnSurface() const { return GetGeoNormal() != Normal3f(); }
    Spectrum f(const Vertex &next) const {
        Vector3f wi = Normalize(next.GetPosition() - GetPosition());
        switch (type) {
        case VertexType::Surface:
            return isect.bsdf->f(isect.wo, wi);
        case VertexType::Medium:
            return mi.phase->p(mi.wo, wi);
        default:
            Error("Vertex::f(): Unimplemented");
            return Spectrum(0.f);
        }
    }
    bool IsConnectible() const {
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
        return type == VertexType::Light && ei.light != nullptr &&
               ::IsDeltaLight(ei.light->flags);
    }
    bool IsInfiniteLight() const {
        return type == VertexType::Light &&
               (ei.light == nullptr || ei.light->flags == LightFlags::Infinite);
    }
    Float PdfLight(const Scene &scene, const Vertex &v) const {
        Vector3f d = v.GetPosition() - GetPosition();
        Float invL2 = 1.f / d.LengthSquared();
        d *= std::sqrt(invL2);
        Float unused, pdf;
        if (IsInfiniteLight()) {
            // Compute planar sampling density for infinite light sources
            Point3f worldCenter;
            Float worldRadius;
            scene.WorldBound().BoundingSphere(&worldCenter, &worldRadius);
            pdf = 1.f / (Pi * worldRadius * worldRadius);
        } else {
            // Get pointer _light_ to the light source
            Assert(IsLight());
            const Light *light = type == VertexType::Light
                                     ? ei.light
                                     : isect.primitive->GetAreaLight();
            light->Pdf(Ray(GetPosition(), d, GetTime()), GetGeoNormal(),
                       &unused, &pdf);
            pdf *= invL2;
        }
        if (v.IsOnSurface()) pdf *= AbsDot(v.GetGeoNormal(), d);
        return pdf;
    }
    Float PdfLightOrigin(const Scene &scene, const Vertex &v,
                         const Distribution1D &lightDistr) const {
        Vector3f d = Normalize(v.GetPosition() - GetPosition());
        if (IsInfiniteLight()) {
            // Return solid angle density for infinite light sources
            return InfiniteLightDensity(scene, lightDistr, d);
        } else {
            Float unused, pdfPos, pdfChoice = 0;
            // Get pointer _light_ to the light source
            Assert(IsLight());
            const Light *light = type == VertexType::Light
                                     ? ei.light
                                     : isect.primitive->GetAreaLight();

            // Set _pdfChoice_ to the discrete probability of sampling _light_
            for (int i = 0; i < scene.lights.size(); ++i) {
                if (scene.lights[i].get() == light) {
                    pdfChoice = lightDistr.func[i] /
                                (lightDistr.funcInt * lightDistr.Count());
                    break;
                }
            }
            Assert(pdfChoice != 0);
            light->Pdf(Ray(GetPosition(), d, GetTime()), GetGeoNormal(),
                       &pdfPos, &unused);
            return pdfPos * pdfChoice;
        }
    }
    Spectrum Le(const Scene &scene, const Vertex &v) const {
        Vector3f d = Normalize(v.GetPosition() - GetPosition());
        if (IsInfiniteLight()) {
            // Return emitted radiance for infinite light sources
            Spectrum Le(0.f);
            for (const auto &light : scene.lights)
                Le += light->Le(Ray(GetPosition(), -d));
            return Le;
        } else {
            const AreaLight *light = isect.primitive->GetAreaLight();
            return light->L(isect, d);
        }
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
           << "  connectible = " << v.IsConnectible() << "," << std::endl
           << "  p = " << v.GetPosition() << "," << std::endl
           << "  n = " << v.GetGeoNormal() << "," << std::endl
           << "  pdfFwd = " << v.pdfFwd << "," << std::endl
           << "  pdfRev = " << v.pdfRev << "," << std::endl
           << "  weight = " << v.weight << std::endl
           << "]" << std::endl;
        return os;
    };
};

extern int GenerateCameraSubpath(const Scene &scene, Sampler &sampler,
                                 MemoryArena &arena, int maxdepth,
                                 const Camera &camera, Point2f &rasterPos,
                                 Vertex *path);

extern int GenerateLightSubpath(const Scene &scene, Sampler &sampler,
                                MemoryArena &arena, int maxdepth, Float time,
                                const Distribution1D &lightDistr, Vertex *path);

extern Spectrum ConnectBDPT(const Scene &scene, Vertex *lightSubpath,
                            Vertex *cameraSubpath, int s, int t,
                            const Distribution1D &lightDistr,
                            const Camera &camera, Sampler &sampler,
                            Point2f *rasterPos, Float *misWeight);
BDPTIntegrator *CreateBDPTIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera);

#endif  // PBRT_INTEGRATORS_BDPT_H
