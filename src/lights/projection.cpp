
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

// lights/projection.cpp*
#include "lights/projection.h"
#include "sampling.h"
#include "paramset.h"
#include "reflection.h"
#include "stats.h"

namespace pbrt {

// ProjectionLight Method Definitions
ProjectionLight::ProjectionLight(const Transform &LightToWorld,
                                 const MediumInterface &mediumInterface,
                                 const Spectrum &I, const std::string &texname,
                                 Float fov)
    : Light((int)LightFlags::DeltaPosition, LightToWorld, mediumInterface),
      pLight(LightToWorld(Point3f(0, 0, 0))),
      I(I) {
    // Create _ProjectionLight_ MIP map
    if (!Image::Read(texname, &image)) {
        std::vector<Float> one = {(Float)1};
        image = Image(std::move(one), PixelFormat::Y32, {1, 1});
    }

    // Initialize _ProjectionLight_ projection matrix
    Float aspect = Float(image.resolution.x) / Float(image.resolution.y);
    if (aspect > 1)
        screenBounds = Bounds2f(Point2f(-aspect, -1), Point2f(aspect, 1));
    else
        screenBounds =
            Bounds2f(Point2f(-1, -1 / aspect), Point2f(1, 1 / aspect));
    hither = 1e-3f;
    yon = 1e30f;
    lightProjection = Perspective(fov, hither, yon);

    // Compute cosine of cone surrounding projection directions
    Float opposite = std::tan(Radians(fov) / 2.f);
    Float tanDiag = opposite * std::sqrt(1 + 1 / (aspect * aspect));
    cosTotalWidth = std::cos(std::atan(tanDiag));
}

Spectrum ProjectionLight::Sample_Li(const Interaction &ref, const Point2f &u,
                                    Vector3f *wi, Float *pdf,
                                    VisibilityTester *vis) const {
    ProfilePhase _(Prof::LightSample);
    *wi = Normalize(pLight - ref.p);
    *pdf = 1;
    *vis =
        VisibilityTester(ref, Interaction(pLight, ref.time, mediumInterface));
    return Projection(-*wi) / DistanceSquared(pLight, ref.p);
}

Spectrum ProjectionLight::Projection(const Vector3f &w) const {
    Vector3f wl = WorldToLight(w);
    // Discard directions behind projection light
    if (wl.z < hither) return 0;

    // Project point onto projection plane and compute light
    Point3f p = lightProjection(Point3f(wl.x, wl.y, wl.z));
    if (!Inside(Point2f(p.x, p.y), screenBounds)) return 0.f;
    Point2f st = Point2f(screenBounds.Offset(Point2f(p.x, p.y)));
    return I * image.BilerpSpectrum(st, SpectrumType::Illuminant);
}

Spectrum ProjectionLight::Power() const {
    Spectrum sum(0.f);
    for (int v = 0; v < image.resolution.y; ++v)
        for (int u = 0; u < image.resolution.x; ++u)
            sum += image.GetSpectrum({u, v}, SpectrumType::Illuminant);
    return I * 2 * Pi * (1.f - cosTotalWidth) * sum /
           (image.resolution.x * image.resolution.y);
}

Float ProjectionLight::Pdf_Li(const Interaction &, const Vector3f &) const {
    return 0.f;
}

Spectrum ProjectionLight::Sample_Le(const Point2f &u1, const Point2f &u2,
                                    Float time, Ray *ray, Normal3f *nLight,
                                    Float *pdfPos, Float *pdfDir) const {
    ProfilePhase _(Prof::LightSample);
    Vector3f v = UniformSampleCone(u1, cosTotalWidth);
    *ray = Ray(pLight, LightToWorld(v), Infinity, time, mediumInterface.inside);
    *nLight = (Normal3f)ray->d;  /// same here
    *pdfPos = 1.f;
    *pdfDir = UniformConePdf(cosTotalWidth);
    return Projection(ray->d);
}

void ProjectionLight::Pdf_Le(const Ray &ray, const Normal3f &, Float *pdfPos,
                             Float *pdfDir) const {
    ProfilePhase _(Prof::LightPdf);
    *pdfPos = 0.f;
    *pdfDir = (CosTheta(WorldToLight(ray.d)) >= cosTotalWidth)
                  ? UniformConePdf(cosTotalWidth)
                  : 0;
}

std::shared_ptr<ProjectionLight> CreateProjectionLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet) {
    Spectrum I = paramSet.FindOneSpectrum("I", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    Float fov = paramSet.FindOneFloat("fov", 45.);
    std::string texname = paramSet.FindOneFilename("mapname", "");
    return std::make_shared<ProjectionLight>(light2world, medium, I * sc,
                                             texname, fov);
}

}  // namespace pbrt
