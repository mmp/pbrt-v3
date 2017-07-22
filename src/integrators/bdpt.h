
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_BDPT_H
#define PBRT_INTEGRATORS_BDPT_H

// integrators/bdpt.h*
#include <unordered_map>
#include "camera.h"
#include "integrator.h"
#include "interaction.h"
#include "light.h"
#include "pbrt.h"
#include "reflection.h"
#include "sampling.h"
#include "scene.h"

namespace pbrt {

/// Forward declaration (correction term for adjoint BSDF with shading normals)
extern Float CorrectShadingNormal(const SurfaceInteraction &isect,
                                  const Vector3f &wo, const Vector3f &wi,
                                  TransportMode mode);

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
    EndpointInteraction(const Light *light, const Ray &r, const Normal3f &nl)
        : Interaction(r.o, r.time, r.medium), light(light) {
        n = nl;
    }
    EndpointInteraction(const Interaction &it, const Light *light)
        : Interaction(it), light(light) {}
    EndpointInteraction(const Ray &ray)
        : Interaction(ray(1), ray.time, ray.medium), light(nullptr) {
        n = Normal3f(-ray.d);
    }
};

// BDPT Helper Definitions
enum class VertexType { Camera, Light, Surface, Medium };
struct Vertex;
template <typename Type>
class ScopedAssignment {
  public:
    // ScopedAssignment Public Methods
    ScopedAssignment(Type *target = nullptr, Type value = Type())
        : target(target) {
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

inline Float InfiniteLightDensity(
    const Scene &scene, const Distribution1D &lightDistr,
    const std::unordered_map<const Light *, size_t> &lightToDistrIndex,
    const Vector3f &w) {
    Float pdf = 0;
    for (const auto &light : scene.infiniteLights) {
        CHECK(lightToDistrIndex.find(light.get()) != lightToDistrIndex.end());
        size_t index = lightToDistrIndex.find(light.get())->second;
        pdf += light->Pdf_Li(Interaction(), -w) * lightDistr.func[index];
    }
    return pdf / (lightDistr.funcInt * lightDistr.Count());
}

// BDPT Declarations
class BDPTIntegrator : public Integrator {
  public:
    // BDPTIntegrator Public Methods
    BDPTIntegrator(std::shared_ptr<Sampler> sampler,
                   std::shared_ptr<const Camera> camera, int maxDepth,
                   bool visualizeStrategies, bool visualizeWeights,
                   const Bounds2i &pixelBounds,
                   const std::string &lightSampleStrategy = "power")
        : sampler(sampler),
          camera(camera),
          maxDepth(maxDepth),
          visualizeStrategies(visualizeStrategies),
          visualizeWeights(visualizeWeights),
          pixelBounds(pixelBounds),
          lightSampleStrategy(lightSampleStrategy) {}
    void Render(const Scene &scene);

  private:
    // BDPTIntegrator Private Data
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<const Camera> camera;
    const int maxDepth;
    const bool visualizeStrategies;
    const bool visualizeWeights;
    const Bounds2i pixelBounds;
    const std::string lightSampleStrategy;
};

struct Vertex {
    // Vertex Public Data
    VertexType type;
    Spectrum beta;
#ifdef PBRT_HAVE_NONPOD_IN_UNIONS
    union {
#else
    struct {
#endif  // PBRT_HAVE_NONPOD_IN_UNIONS
        EndpointInteraction ei;
        MediumInteraction mi;
        SurfaceInteraction si;
    };
    bool delta = false;
    Float pdfFwd = 0, pdfRev = 0;

    // Vertex Public Methods
    Vertex() : ei() {}
    Vertex(VertexType type, const EndpointInteraction &ei, const Spectrum &beta)
        : type(type), beta(beta), ei(ei) {}
    Vertex(const SurfaceInteraction &si, const Spectrum &beta)
        : type(VertexType::Surface), beta(beta), si(si) {}

    // Need to define these two to make compilers happy with the non-POD
    // objects in the anonymous union above.
    Vertex(const Vertex &v) { memcpy(this, &v, sizeof(Vertex)); }
    Vertex &operator=(const Vertex &v) {
        memcpy(this, &v, sizeof(Vertex));
        return *this;
    }

    static inline Vertex CreateCamera(const Camera *camera, const Ray &ray,
                                      const Spectrum &beta);
    static inline Vertex CreateCamera(const Camera *camera,
                                      const Interaction &it,
                                      const Spectrum &beta);
    static inline Vertex CreateLight(const Light *light, const Ray &ray,
                                     const Normal3f &nLight, const Spectrum &Le,
                                     Float pdf);
    static inline Vertex CreateLight(const EndpointInteraction &ei,
                                     const Spectrum &beta, Float pdf);
    static inline Vertex CreateMedium(const MediumInteraction &mi,
                                      const Spectrum &beta, Float pdf,
                                      const Vertex &prev);
    static inline Vertex CreateSurface(const SurfaceInteraction &si,
                                       const Spectrum &beta, Float pdf,
                                       const Vertex &prev);
    Vertex(const MediumInteraction &mi, const Spectrum &beta)
        : type(VertexType::Medium), beta(beta), mi(mi) {}
    const Interaction &GetInteraction() const {
        switch (type) {
        case VertexType::Medium:
            return mi;
        case VertexType::Surface:
            return si;
        default:
            return ei;
        }
    }
    const Point3f &p() const { return GetInteraction().p; }
    Float time() const { return GetInteraction().time; }
    const Normal3f &ng() const { return GetInteraction().n; }
    const Normal3f &ns() const {
        if (type == VertexType::Surface)
            return si.shading.n;
        else
            return GetInteraction().n;
    }
    bool IsOnSurface() const { return ng() != Normal3f(); }
    Spectrum f(const Vertex &next, TransportMode mode) const {
        Vector3f wi = next.p() - p();
        if (wi.LengthSquared() == 0) return 0.;
        wi = Normalize(wi);
        switch (type) {
        case VertexType::Surface:
            return si.bsdf->f(si.wo, wi) *
                CorrectShadingNormal(si, si.wo, wi, mode);
        case VertexType::Medium:
            return mi.phase->p(mi.wo, wi);
        default:
            LOG(FATAL) << "Vertex::f(): Unimplemented";
            return Spectrum(0.f);
        }
    }
    bool IsConnectible() const {
        switch (type) {
        case VertexType::Medium:
            return true;
        case VertexType::Light:
            return (ei.light->flags & (int)LightFlags::DeltaDirection) == 0;
        case VertexType::Camera:
            return true;
        case VertexType::Surface:
            return si.bsdf->NumComponents(BxDFType(BSDF_DIFFUSE | BSDF_GLOSSY |
                                                   BSDF_REFLECTION |
                                                   BSDF_TRANSMISSION)) > 0;
        }
        LOG(FATAL) << "Unhandled vertex type in IsConnectable()";
        return false;  // NOTREACHED
    }
    bool IsLight() const {
        return type == VertexType::Light ||
               (type == VertexType::Surface && si.primitive->GetAreaLight());
    }
    bool IsDeltaLight() const {
        return type == VertexType::Light && ei.light &&
               pbrt::IsDeltaLight(ei.light->flags);
    }
    bool IsInfiniteLight() const {
        return type == VertexType::Light &&
               (!ei.light || ei.light->flags & (int)LightFlags::Infinite ||
                ei.light->flags & (int)LightFlags::DeltaDirection);
    }
    Spectrum Le(const Scene &scene, const Vertex &v) const {
        if (!IsLight()) return Spectrum(0.f);
        Vector3f w = v.p() - p();
        if (w.LengthSquared() == 0) return 0.;
        w = Normalize(w);
        if (IsInfiniteLight()) {
            // Return emitted radiance for infinite light sources
            Spectrum Le(0.f);
            for (const auto &light : scene.infiniteLights)
                Le += light->Le(Ray(p(), -w));
            return Le;
        } else {
            const AreaLight *light = si.primitive->GetAreaLight();
            CHECK_NOTNULL(light);
            return light->L(si, w);
        }
    }
    friend std::ostream &operator<<(std::ostream &os, const Vertex &v) {
        return os << v.ToString();
    }
    std::string ToString() const {
        std::string s = std::string("[Vertex type: ");
        switch (type) {
        case VertexType::Camera:
            s += "camera";
            break;
        case VertexType::Light:
            s += "light";
            break;
        case VertexType::Surface:
            s += "surface";
            break;
        case VertexType::Medium:
            s += "medium";
            break;
        }
        s += std::string(" connectible: ") +
            std::string(IsConnectible() ? "true" : "false");
        s += StringPrintf("\n  p: [ %f, %f, %f ] ng: [ %f, %f, %f ]", p().x, p().y,
                          p().z, ng().x, ng().y, ng().z);
        s += StringPrintf("\n  pdfFwd: %f pdfRev: %f beta: ", pdfFwd, pdfRev) +
             beta.ToString();
        switch (type) {
        case VertexType::Camera:
            // TODO
            break;
        case VertexType::Light:
            // TODO
            break;
        case VertexType::Surface:
            s += std::string("\n  bsdf: ") + si.bsdf->ToString();
            break;
        case VertexType::Medium:
            s += std::string("\n  phase: ") + mi.phase->ToString();
            break;
        }
        s += std::string(" ]");
        return s;
    }
    Float ConvertDensity(Float pdf, const Vertex &next) const {
        // Return solid angle density if _next_ is an infinite area light
        if (next.IsInfiniteLight()) return pdf;
        Vector3f w = next.p() - p();
        if (w.LengthSquared() == 0) return 0;
        Float invDist2 = 1 / w.LengthSquared();
        if (next.IsOnSurface())
            pdf *= AbsDot(next.ng(), w * std::sqrt(invDist2));
        return pdf * invDist2;
    }
    Float Pdf(const Scene &scene, const Vertex *prev,
              const Vertex &next) const {
        if (type == VertexType::Light) return PdfLight(scene, next);
        // Compute directions to preceding and next vertex
        Vector3f wn = next.p() - p();
        if (wn.LengthSquared() == 0) return 0;
        wn = Normalize(wn);
        Vector3f wp;
        if (prev) {
            wp = prev->p() - p();
            if (wp.LengthSquared() == 0) return 0;
            wp = Normalize(wp);
        } else
            CHECK(type == VertexType::Camera);

        // Compute directional density depending on the vertex types
        Float pdf = 0, unused;
        if (type == VertexType::Camera)
            ei.camera->Pdf_We(ei.SpawnRay(wn), &unused, &pdf);
        else if (type == VertexType::Surface)
            pdf = si.bsdf->Pdf(wp, wn);
        else if (type == VertexType::Medium)
            pdf = mi.phase->p(wp, wn);
        else
            LOG(FATAL) << "Vertex::Pdf(): Unimplemented";

        // Return probability per unit area at vertex _next_
        return ConvertDensity(pdf, next);
    }
    Float PdfLight(const Scene &scene, const Vertex &v) const {
        Vector3f w = v.p() - p();
        Float invDist2 = 1 / w.LengthSquared();
        w *= std::sqrt(invDist2);
        Float pdf;
        if (IsInfiniteLight()) {
            // Compute planar sampling density for infinite light sources
            Point3f worldCenter;
            Float worldRadius;
            scene.WorldBound().BoundingSphere(&worldCenter, &worldRadius);
            pdf = 1 / (Pi * worldRadius * worldRadius);
        } else {
            // Get pointer _light_ to the light source at the vertex
            CHECK(IsLight());
            const Light *light = type == VertexType::Light
                                     ? ei.light
                                     : si.primitive->GetAreaLight();
            CHECK_NOTNULL(light);

            // Compute sampling density for non-infinite light sources
            Float pdfPos, pdfDir;
            light->Pdf_Le(Ray(p(), w, time()), ng(), &pdfPos, &pdfDir);
            pdf = pdfDir * invDist2;
        }
        if (v.IsOnSurface()) pdf *= AbsDot(v.ng(), w);
        return pdf;
    }
    Float PdfLightOrigin(const Scene &scene, const Vertex &v,
                         const Distribution1D &lightDistr,
                         const std::unordered_map<const Light *, size_t>
                             &lightToDistrIndex) const {
        Vector3f w = v.p() - p();
        if (w.LengthSquared() == 0) return 0.;
        w = Normalize(w);
        if (IsInfiniteLight()) {
            // Return solid angle density for infinite light sources
            return InfiniteLightDensity(scene, lightDistr, lightToDistrIndex,
                                        w);
        } else {
            // Return solid angle density for non-infinite light sources
            Float pdfPos, pdfDir, pdfChoice = 0;

            // Get pointer _light_ to the light source at the vertex
            CHECK(IsLight());
            const Light *light = type == VertexType::Light
                                     ? ei.light
                                     : si.primitive->GetAreaLight();
            CHECK_NOTNULL(light);

            // Compute the discrete probability of sampling _light_, _pdfChoice_
            CHECK(lightToDistrIndex.find(light) != lightToDistrIndex.end());
            size_t index = lightToDistrIndex.find(light)->second;
            pdfChoice = lightDistr.DiscretePDF(index);

            light->Pdf_Le(Ray(p(), w, time()), ng(), &pdfPos, &pdfDir);
            return pdfPos * pdfChoice;
        }
    }
};

extern int GenerateCameraSubpath(const Scene &scene, Sampler &sampler,
                                 MemoryArena &arena, int maxDepth,
                                 const Camera &camera, const Point2f &pFilm,
                                 Vertex *path);

extern int GenerateLightSubpath(
    const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
    Float time, const Distribution1D &lightDistr,
    const std::unordered_map<const Light *, size_t> &lightToIndex,
    Vertex *path);
Spectrum ConnectBDPT(
    const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, int s,
    int t, const Distribution1D &lightDistr,
    const std::unordered_map<const Light *, size_t> &lightToIndex,
    const Camera &camera, Sampler &sampler, Point2f *pRaster,
    Float *misWeight = nullptr);
BDPTIntegrator *CreateBDPTIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera);

// Vertex Inline Method Definitions
inline Vertex Vertex::CreateCamera(const Camera *camera, const Ray &ray,
                                   const Spectrum &beta) {
    return Vertex(VertexType::Camera, EndpointInteraction(camera, ray), beta);
}

inline Vertex Vertex::CreateCamera(const Camera *camera, const Interaction &it,
                                   const Spectrum &beta) {
    return Vertex(VertexType::Camera, EndpointInteraction(it, camera), beta);
}

inline Vertex Vertex::CreateLight(const Light *light, const Ray &ray,
                                  const Normal3f &Nl, const Spectrum &Le,
                                  Float pdf) {
    Vertex v(VertexType::Light, EndpointInteraction(light, ray, Nl), Le);
    v.pdfFwd = pdf;
    return v;
}

inline Vertex Vertex::CreateSurface(const SurfaceInteraction &si,
                                    const Spectrum &beta, Float pdf,
                                    const Vertex &prev) {
    Vertex v(si, beta);
    v.pdfFwd = prev.ConvertDensity(pdf, v);
    return v;
}

inline Vertex Vertex::CreateMedium(const MediumInteraction &mi,
                                   const Spectrum &beta, Float pdf,
                                   const Vertex &prev) {
    Vertex v(mi, beta);
    v.pdfFwd = prev.ConvertDensity(pdf, v);
    return v;
}

inline Vertex Vertex::CreateLight(const EndpointInteraction &ei,
                                  const Spectrum &beta, Float pdf) {
    Vertex v(VertexType::Light, ei, beta);
    v.pdfFwd = pdf;
    return v;
}

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_BDPT_H
