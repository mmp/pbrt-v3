
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

// lights/diffuse.cpp*
#include "lights/diffuse.h"
#include "paramset.h"
#include "sampling.h"

// DiffuseAreaLight Method Definitions
DiffuseAreaLight::DiffuseAreaLight(const Transform &LightToWorld,
                                   const Medium *medium, const Spectrum &Lemit,
                                   int nSamples,
                                   const std::shared_ptr<Shape> &shape)
    : AreaLight(LightToWorld, medium, nSamples), Lemit(Lemit), shape(shape) {
    area = shape->Area();
}

Spectrum DiffuseAreaLight::Power() const { return Lemit * area * Pi; }

Spectrum DiffuseAreaLight::Sample_L(const Interaction &ref, const Point2f &u,
                                    Vector3f *wi, Float *pdf,
                                    VisibilityTester *vis) const {
    Interaction p1;
    if (!shape->Sample(ref, u, &p1) || ((ref.p - p1.p).Length() == 0.f)) {
        *pdf = 0.f;
        return Spectrum(0.f);
    }
    p1.mediumInterface = MediumInterface(medium);
    *vis = VisibilityTester(ref, p1);
    *wi = Normalize(p1.p - ref.p);
    *pdf = shape->Pdf(ref, *wi);
    return L(p1, -*wi);
}

Float DiffuseAreaLight::Pdf(const Interaction &ref, const Vector3f &wi) const {
    return shape->Pdf(ref, wi);
}

Spectrum DiffuseAreaLight::Sample_L(const Point2f &u1, const Point2f &u2,
                                    Float time, Ray *ray, Normal3f *nLight,
                                    Float *pdfPos, Float *pdfDir) const {
    Interaction it;
    if (!shape->Sample(u1, &it)) {
        *pdfPos = *pdfDir = 0.f;
        return Spectrum(0.f);
    }
    Vector3f w = CosineSampleHemisphere(u2);
    *pdfPos = shape->Pdf(it);
    *pdfDir = CosineHemispherePdf(w.z);
    Vector3f v1, v2, n(it.n);
    CoordinateSystem(Vector3f(n), &v1, &v2);
    w = w.x * v1 + w.y * v2 + w.z * n;
    *ray = it.SpawnRay(w);
    *nLight = it.n;
    it.mediumInterface = MediumInterface(medium);
    return L(it, w);
}

void DiffuseAreaLight::Pdf(const Ray &ray, const Normal3f &n, Float *pdfPos,
                           Float *pdfDir) const {
    Interaction it(ray.o, n, Vector3f(), Vector3f(), ray.time,
                   MediumInterface(medium));
    *pdfPos = shape->Pdf(it);
    *pdfDir = CosineHemispherePdf(Dot(n, ray.d));
}

std::shared_ptr<AreaLight> CreateDiffuseAreaLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet, const std::shared_ptr<Shape> &shape) {
    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.FindOneInt("nsamples", 1);
    if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);
    return std::make_shared<DiffuseAreaLight>(light2world, medium, L * sc,
                                              nSamples, shape);
}
