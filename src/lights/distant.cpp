
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

// lights/distant.cpp*
#include "lights/distant.h"
#include "paramset.h"
#include "sampling.h"

// DistantLight Method Definitions
DistantLight::DistantLight(const Transform &LightToWorld, const Spectrum &L,
                           const Vector3f &wLight)
    : Light(LightFlags::DeltaDirection, LightToWorld, nullptr),
      L(L),
      wLight(Normalize(LightToWorld(wLight))) {}
Spectrum DistantLight::Sample_L(const Interaction &ref, const Point2f &sample,
                                Vector3f *wi, Float *pdf,
                                VisibilityTester *vis) const {
    *wi = wLight;
    *pdf = 1.f;
    Point3f pOutside = ref.p + wLight * (2 * worldRadius);
    *vis = VisibilityTester(ref, Interaction(pOutside, ref.time, medium));
    return L;
}

Spectrum DistantLight::Power() const {
    return L * Pi * worldRadius * worldRadius;
}

Float DistantLight::Pdf(const Interaction &, const Vector3f &) const {
    return 0.f;
}

Spectrum DistantLight::Sample_L(const Point2f &sample1, const Point2f &sample2,
                                Float time, Ray *ray, Normal3f *Ns,
                                Float *pdfPos, Float *pdfDir) const {
    // Choose point on disk oriented toward infinite light direction
    Vector3f v1, v2;
    CoordinateSystem(wLight, &v1, &v2);
    Point2f cd = ConcentricSampleDisk(sample1);
    Point3f Pdisk = worldCenter + worldRadius * (cd.x * v1 + cd.y * v2);

    // Set ray origin and direction for infinite light ray
    *ray = Ray(Pdisk + worldRadius * wLight, -wLight, Infinity, time);
    *Ns = (Normal3f)ray->d;
    *pdfPos = 1.f / (Pi * worldRadius * worldRadius);
    *pdfDir = 1.f;
    return L;
}

void DistantLight::Pdf(const Ray &, const Normal3f &, Float *pdfPos,
                       Float *pdfDir) const {
    *pdfPos = 1.f / (Pi * worldRadius * worldRadius);
    *pdfDir = 0.f;
}

std::shared_ptr<DistantLight> CreateDistantLight(const Transform &light2world,
                                                 const ParamSet &paramSet) {
    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    Point3f from = paramSet.FindOnePoint3f("from", Point3f(0, 0, 0));
    Point3f to = paramSet.FindOnePoint3f("to", Point3f(0, 0, 1));
    Vector3f dir = from - to;
    return std::make_shared<DistantLight>(light2world, L * sc, dir);
}
