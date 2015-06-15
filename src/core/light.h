
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

#ifndef PBRT_CORE_LIGHT_H
#define PBRT_CORE_LIGHT_H
#include "stdafx.h"

// core/light.h*
#include "pbrt.h"
#include "memory.h"
#include "interaction.h"

// LightFlags Declarations
enum class LightFlags { DeltaPosition, DeltaDirection, Area, Infinite };

inline bool IsDeltaLight(LightFlags flags) {
    return flags == LightFlags::DeltaPosition ||
           flags == LightFlags::DeltaDirection;
}

// Light Declarations
class Light {
  public:
    // Light Interface
    virtual ~Light();
    Light(LightFlags flags, const Transform &LightToWorld, const Medium *medium,
          int nSamples = 1)
        : flags(flags),
          nSamples(std::max(1, nSamples)),
          medium(medium),
          LightToWorld(LightToWorld),
          WorldToLight(Inverse(LightToWorld)) {
        // Warn if light has transformation with non-uniform scale
        if (WorldToLight.HasScale())
            Warning(
                "Scaling detected in world to light transformation!\n"
                "The system has numerous assumptions, implicit and explicit,\n"
                "that this transform will have no scale factors in it.\n"
                "Proceed at your own risk; your image may have errors or\n"
                "the system may crash as a result of this.");
    }
    virtual Spectrum Sample_L(const Interaction &ref, const Point2f &sample,
                              Vector3f *wi, Float *pdf,
                              VisibilityTester *vis) const = 0;
    virtual Spectrum Power() const = 0;
    virtual void Preprocess(const Scene &scene) {}
    virtual Spectrum Le(const RayDifferential &r) const;
    virtual Float Pdf(const Interaction &vis, const Vector3f &wi) const = 0;
    virtual Spectrum Sample_L(const Point2f &sample1, const Point2f &sample2,
                              Float time, Ray *ray, Normal3f *Ns, Float *pdfPos,
                              Float *pdfDir) const = 0;
    virtual void Pdf(const Ray &ray, const Normal3f &Ns, Float *pdfPos,
                     Float *pdfDir) const = 0;

    // Light Public Data
    const LightFlags flags;
    const int nSamples;
    const Medium *medium;

  protected:
    // Light Protected Data
    const Transform LightToWorld, WorldToLight;
};

class VisibilityTester {
  public:
    VisibilityTester() {}
    // VisibilityTester Public Methods
    VisibilityTester(const Interaction &p0, const Interaction &p1)
        : p0(p0), p1(p1) {}
    const Interaction &P0() const { return p0; }
    const Interaction &P1() const { return p1; }
    bool Unoccluded(const Scene &scene) const;
    Spectrum T(const Scene &scene, Sampler &sampler) const;

  private:
    Interaction p0, p1;
};

class AreaLight : public Light {
  public:
    // AreaLight Interface
    AreaLight(const Transform &LightToWorld, const Medium *medium, int nSamples)
        : Light(LightFlags::Area, LightToWorld, medium, nSamples) {}
    virtual Spectrum L(const Interaction &intr, const Vector3f &w) const = 0;
};

#endif  // PBRT_CORE_LIGHT_H
