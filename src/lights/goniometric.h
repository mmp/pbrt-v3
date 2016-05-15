
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

#ifndef PBRT_LIGHTS_GONIOMETRIC_H
#define PBRT_LIGHTS_GONIOMETRIC_H

// lights/goniometric.h*
#include "pbrt.h"
#include "light.h"
#include "shape.h"
#include "scene.h"
#include "mipmap.h"
#include "imageio.h"

// GonioPhotometricLight Declarations
class GonioPhotometricLight : public Light {
  public:
    // GonioPhotometricLight Public Methods
    Spectrum Sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi,
                       Float *pdf, VisibilityTester *vis) const;
    GonioPhotometricLight(const Transform &LightToWorld,
                          const MediumInterface &mediumInterface,
                          const Spectrum &I, const std::string &texname)
        : Light((int)LightFlags::DeltaPosition, LightToWorld, mediumInterface),
          pLight(LightToWorld(Point3f(0, 0, 0))),
          I(I) {
        // Create _mipmap_ for _GonioPhotometricLight_
        Point2i resolution;
        std::unique_ptr<RGBSpectrum[]> texels = ReadImage(texname, &resolution);
        if (texels)
            mipmap.reset(new MIPMap<RGBSpectrum>(resolution, texels.get()));
    }
    Spectrum Scale(const Vector3f &w) const {
        Vector3f wp = Normalize(WorldToLight(w));
        std::swap(wp.y, wp.z);
        Float theta = SphericalTheta(wp);
        Float phi = SphericalPhi(wp);
        Point2f st(phi * Inv2Pi, theta * InvPi);
        return !mipmap ? RGBSpectrum(1.f)
                       : Spectrum(mipmap->Lookup(st), SpectrumType::Illuminant);
    }
    Spectrum Power() const;
    Float Pdf_Li(const Interaction &, const Vector3f &) const;
    Spectrum Sample_Le(const Point2f &u1, const Point2f &u2, Float time,
                       Ray *ray, Normal3f *nLight, Float *pdfPos,
                       Float *pdfDir) const;
    void Pdf_Le(const Ray &, const Normal3f &, Float *pdfPos,
                Float *pdfDir) const;

  private:
    // GonioPhotometricLight Private Data
    const Point3f pLight;
    const Spectrum I;
    std::unique_ptr<MIPMap<RGBSpectrum>> mipmap;
};

std::shared_ptr<GonioPhotometricLight> CreateGoniometricLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet);

#endif  // PBRT_LIGHTS_GONIOMETRIC_H
