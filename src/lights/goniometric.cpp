
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


// lights/goniometric.cpp*
#include "lights/goniometric.h"
#include "paramset.h"
#include "sampling.h"
#include "stats.h"

namespace pbrt {

// GonioPhotometricLight Method Definitions
Spectrum GonioPhotometricLight::Sample_Li(const Interaction &ref,
                                          const Point2f &u, Vector3f *wi,
                                          Float *pdf,
                                          VisibilityTester *vis) const {
    ProfilePhase _(Prof::LightSample);
    *wi = Normalize(pLight - ref.p);
    *pdf = 1.f;
    *vis =
        VisibilityTester(ref, Interaction(pLight, ref.time, mediumInterface));
    return I * Scale(-*wi) / DistanceSquared(pLight, ref.p);
}

Spectrum GonioPhotometricLight::Power() const {
    return 4 * Pi * I * Spectrum(mipmap ? mipmap->Lookup(Point2f(.5f, .5f), .5f)
                                        : RGBSpectrum(1.f),
                                 SpectrumType::Illuminant);
}

Float GonioPhotometricLight::Pdf_Li(const Interaction &,
                                    const Vector3f &) const {
    return 0.f;
}

Spectrum GonioPhotometricLight::Sample_Le(const Point2f &u1, const Point2f &u2,
                                          Float time, Ray *ray,
                                          Normal3f *nLight, Float *pdfPos,
                                          Float *pdfDir) const {
    ProfilePhase _(Prof::LightSample);
    *ray = Ray(pLight, UniformSampleSphere(u1), Infinity, time,
               mediumInterface.inside);
    *nLight = (Normal3f)ray->d;
    *pdfPos = 1.f;
    *pdfDir = UniformSpherePdf();
    return I * Scale(ray->d);
}

void GonioPhotometricLight::Pdf_Le(const Ray &, const Normal3f &, Float *pdfPos,
                                   Float *pdfDir) const {
    ProfilePhase _(Prof::LightPdf);
    *pdfPos = 0.f;
    *pdfDir = UniformSpherePdf();
}

std::shared_ptr<GonioPhotometricLight> CreateGoniometricLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet) {
    Spectrum I = paramSet.FindOneSpectrum("I", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    std::string texname = paramSet.FindOneFilename("mapname", "");
    return std::make_shared<GonioPhotometricLight>(light2world, medium, I * sc,
                                                   texname);
}

}  // namespace pbrt
