
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

#ifndef PBRT_CORE_MEDIUM_H
#define PBRT_CORE_MEDIUM_H

// core/medium.h*
#include "pbrt.h"
#include "geometry.h"
#include "spectrum.h"
#include <memory>

namespace pbrt {

constexpr inline Float dwivedi_normalization(Float nu) {
  return 1.f / log((nu + 1.f) / (nu - 1.f));
}

class GuidedSamplingInfo {
public:
    /**
     * @brief This object is used in Dwivedi sampling method (analytical path guiding)
     * @param normal normal at the medium entry point 
     * @param poe point of entry at the entry point
     */

    GuidedSamplingInfo(Float _nu): nu(_nu), norm_nu(dwivedi_normalization(_nu)), 
        normal(Vector3f(1, 0, 0)) {}
    GuidedSamplingInfo(Float _nu, const Vector3f& _normal):
        nu(_nu), norm_nu(dwivedi_normalization(_nu)), normal(_normal) {}
    
    // FIXME: poe might be removed
    const Float nu;
    const Float norm_nu;        // p(w_z) normalization factor
    Vector3f normal;            // normal of the entry surface
};

// Media Declarations
class PhaseFunction {
  public:
    // PhaseFunction Interface
    virtual ~PhaseFunction();
    virtual Float p(const Vector3f &wo, const Vector3f &wi) const = 0;
    virtual Float Sample_p(const Vector3f &wo, Vector3f *wi,
                           const Point2f &u) const = 0;
    virtual std::string ToString() const = 0;
    virtual Float DvdSample_p(Vector3f *wi, const Point2f &sample,
                    GuidedSamplingInfo* dvd_info = nullptr) const = 0;
    virtual Float dvd_p(const Vector3f &w, GuidedSamplingInfo* dvd_info) const = 0;
};

inline std::ostream &operator<<(std::ostream &os, const PhaseFunction &p) {
    os << p.ToString();
    return os;
}

bool GetMediumScatteringProperties(const std::string &name, Spectrum *sigma_a,
                                   Spectrum *sigma_s);

// Media Inline Functions
inline Float PhaseHG(Float cosTheta, Float g) {
    Float denom = 1 + g * g + 2 * g * cosTheta;
    return Inv4Pi * (1 - g * g) / (denom * std::sqrt(denom));
}

inline Float DvdPdf(Float cosTheta, GuidedSamplingInfo* dvd_info) {
    return dvd_info->norm_nu / (dvd_info->nu - cosTheta) * Inv2Pi;
}

// Medium Declarations
class Medium {
  public:
    // Medium Interface
    virtual ~Medium() {}
    virtual Spectrum Tr(const Ray &ray, Sampler &sampler) const = 0;
    virtual Spectrum Sample(const Ray &ray, Sampler &sampler,
                            MemoryArena &arena,
                            MediumInteraction *mi, const GuidedSamplingInfo* guide_info = nullptr) const = 0;
};

// HenyeyGreenstein Declarations
class HenyeyGreenstein : public PhaseFunction {
  public:
    // HenyeyGreenstein Public Methods
    HenyeyGreenstein(Float g) : g(g) {}
    Float p(const Vector3f &wo, const Vector3f &wi) const;
    Float Sample_p(const Vector3f &wo, Vector3f *wi,
                   const Point2f &sample) const;

    /**
     * @brief Dwivedi sampling method (Dwivedi 1982, Kˇrivánek and d’Eon 2014)
     */
    Float DvdSample_p(Vector3f *wi, const Point2f &sample,
                    GuidedSamplingInfo* dvd_info = nullptr) const;

    Float dvd_p(const Vector3f &w, GuidedSamplingInfo* dvd_info) const {
      // Be aware, the direction of w should be checked
      	return DvdPdf(Dot(w / w.Length(), dvd_info->normal), dvd_info);
    }

    std::string ToString() const {
        return StringPrintf("[ HenyeyGreenstein g: %f ]", g);
    }

  private:
    const Float g;
};

// MediumInterface Declarations
struct MediumInterface {
    MediumInterface() : inside(nullptr), outside(nullptr) {}
    // MediumInterface Public Methods
    MediumInterface(const Medium *medium) : inside(medium), outside(medium) {}
    MediumInterface(const Medium *inside, const Medium *outside)
        : inside(inside), outside(outside) {}
    bool IsMediumTransition() const { return inside != outside; }
    const Medium *inside, *outside;
};

}  // namespace pbrt

#endif  // PBRT_CORE_MEDIUM_H
