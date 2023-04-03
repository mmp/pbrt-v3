
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
    
    CollimatedLight light source implementation for transient rendering
    @author: Qianyue He
    @date: 2023-4-2
 */


// lights/collimated.cpp*
#include "lights/collimated.h"
#include "paramset.h"
#include "sampling.h"
#include "stats.h"

namespace pbrt {

// CollimatedLight Method Definitions
CollimatedLight::CollimatedLight(const Transform &LightToWorld, const Spectrum &L,
                           const MediumInterface &mediumInterface, const Vector3f &wLight)
    : PointLight(LightToWorld, mediumInterface, L),
      wLight(Normalize(LightToWorld(wLight))) {}

Spectrum CollimatedLight::Sample_Li(const Interaction &ref, const Point2f &u,
                                 Vector3f *wi, Float *pdf,
                                 VisibilityTester *vis) const {
    ProfilePhase _(Prof::LightSample);
    // Given an interaction, the proba of hitting the place is 0.0
    return Spectrum(0.f);
}

Spectrum CollimatedLight::Power() const {
    // No attenuation
    return I;
}

Float CollimatedLight::Pdf_Li(const Interaction &, const Vector3f &) const {
    return 0.f;
}

Spectrum CollimatedLight::Sample_Le(const Point2f &u1, const Point2f &u2,
                                 Float time, Ray *ray, Normal3f *nLight,
                                 Float *pdfPos, Float *pdfDir) const {
    ProfilePhase _(Prof::LightSample);
    // TODO: direction of the ray should be examined
    // light ray direction: from - to (towards origin, therefore we inverse the direction)
    *ray = Ray(pLight, -wLight, Infinity, time);
    *nLight = (Normal3f)ray->d;
    *pdfPos = 1;
    *pdfDir = 1;
    return I;
}

void CollimatedLight::Pdf_Le(const Ray &, const Normal3f &, Float *pdfPos,
                          Float *pdfDir) const {
    ProfilePhase _(Prof::LightPdf);
    // TODO: is this correct to set pdfPos to be 0.0? This is only used in PdfLightOrigin
    *pdfPos = 0;
    *pdfDir = 0;
}

std::shared_ptr<CollimatedLight> CreateCollimatedLight(const Transform &light2world,
                                    const Medium *medium, const ParamSet &paramSet) {
    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    Point3f from = paramSet.FindOnePoint3f("from", Point3f(0, 0, 0));
    Point3f to = paramSet.FindOnePoint3f("to", Point3f(0, 0, 1));
    Vector3f dir = from - to;
    Transform l2w = Translate(Vector3f(from.x, from.y, from.z)) * light2world;
    return std::make_shared<CollimatedLight>(l2w, L * sc, medium, dir);
}

}  // namespace pbrt
