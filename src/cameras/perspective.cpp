
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

// cameras/perspective.cpp*
#include "cameras/perspective.h"
#include "paramset.h"
#include "sampler.h"
#include "sampling.h"
#include "light.h"

// PerspectiveCamera Method Definitions
PerspectiveCamera::PerspectiveCamera(const AnimatedTransform &CameraToWorld,
                                     const Bounds2f &screenWindow,
                                     Float shutterOpen, Float shutterClose,
                                     Float lensRadius, Float focalDistance,
                                     Float fov, Film *film,
                                     const Medium *medium)
    : ProjectiveCamera(CameraToWorld, Perspective(fov, 1e-2f, 1000.f),
                       screenWindow, shutterOpen, shutterClose, lensRadius,
                       focalDistance, film, medium) {
    // Compute differential changes in origin for perspective camera rays
    dxCamera =
        RasterToCamera(Point3f(1, 0, 0)) - RasterToCamera(Point3f(0, 0, 0));
    dyCamera =
        RasterToCamera(Point3f(0, 1, 0)) - RasterToCamera(Point3f(0, 0, 0));
}

Float PerspectiveCamera::GenerateRay(const CameraSample &sample,
                                     Ray *ray) const {
    // Compute raster and camera sample positions
    Point3f pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0.);
    Point3f pCamera = RasterToCamera(pFilm);
    *ray = Ray(Point3f(0, 0, 0), Normalize(Vector3f(pCamera)));
    // Modify ray for depth of field
    if (lensRadius > 0.f) {
        // Sample point on lens
        Point2f pLens = lensRadius * ConcentricSampleDisk(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray->d.z;
        Point3f pFocus = (*ray)(ft);

        // Update ray for effect of lens
        ray->o = Point3f(pLens.x, pLens.y, 0.f);
        ray->d = Normalize(pFocus - ray->o);
    }
    ray->time = Lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = CameraToWorld(*ray);
    return 1.f;
}

Float PerspectiveCamera::GenerateRayDifferential(const CameraSample &sample,
                                                 RayDifferential *ray) const {
    // Compute raster and camera sample positions
    Point3f pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0.);
    Point3f pCamera = RasterToCamera(pFilm);
    Vector3f dir = Normalize(Vector3f(pCamera.x, pCamera.y, pCamera.z));
    *ray = RayDifferential(Point3f(0, 0, 0), dir);
    // Modify ray for depth of field
    if (lensRadius > 0.f) {
        // Sample point on lens
        Point2f pLens = lensRadius * ConcentricSampleDisk(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray->d.z;
        Point3f pFocus = (*ray)(ft);

        // Update ray for effect of lens
        ray->o = Point3f(pLens.x, pLens.y, 0.f);
        ray->d = Normalize(pFocus - ray->o);
    }

    // Compute offset rays for _PerspectiveCamera_ ray differentials
    if (lensRadius > 0.) {
        // Compute _PerspectiveCamera_ ray differentials accounting for lens

        // Sample point on lens
        Point2f pLens = lensRadius * ConcentricSampleDisk(sample.pLens);
        Vector3f dx = Normalize(Vector3f(pCamera + dxCamera));
        Float ft = focalDistance / dx.z;
        Point3f pFocus = Point3f(0, 0, 0) + (ft * dx);
        ray->rxOrigin = Point3f(pLens.x, pLens.y, 0.f);
        ray->rxDirection = Normalize(pFocus - ray->rxOrigin);

        Vector3f dy = Normalize(Vector3f(pCamera + dyCamera));
        ft = focalDistance / dy.z;
        pFocus = Point3f(0, 0, 0) + (ft * dy);
        ray->ryOrigin = Point3f(pLens.x, pLens.y, 0.f);
        ray->ryDirection = Normalize(pFocus - ray->ryOrigin);
    } else {
        ray->rxOrigin = ray->ryOrigin = ray->o;
        ray->rxDirection = Normalize(Vector3f(pCamera) + dxCamera);
        ray->ryDirection = Normalize(Vector3f(pCamera) + dyCamera);
    }
    ray->time = Lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = CameraToWorld(*ray);
    ray->hasDifferentials = true;
    return 1.f;
}

Spectrum PerspectiveCamera::We(const Interaction &it, const Vector3f &w,
                               Point2f *raster) const {
    // Interpolate camera matrix and check if _w_ is forward-facing
    Transform c2w;
    CameraToWorld.Interpolate(it.time, &c2w);
    Float cosTheta = Dot(w, c2w(Vector3f(0, 0, 1)));
    if (cosTheta <= 0.f) return 0.f;

    // Map direction _w_ onto the raster grid
    Float invCosTheta = 1.f / cosTheta;
    Point3f pFocus =
        it.p + w * invCosTheta * (lensRadius > 0 ? focalDistance : 1.0f);
    Point3f pRaster = Inverse(RasterToCamera)(Inverse(c2w)(pFocus));
    Point2i res = film->fullResolution;
    if (pRaster.x < 0 || pRaster.x >= res.x || pRaster.y < 0 ||
        pRaster.y >= res.y)
        return 0.f;

    // Compute image plane bounds
    Point3f pMin = RasterToCamera(Point3f(0, 0, 0));
    Point3f pMax = RasterToCamera(Point3f(res.x, res.y, 0));
    pMin /= pMin.z;
    pMax /= pMax.z;
    Float area = (pMax.x - pMin.x) * (pMax.y - pMin.y);

    // Compute importance
    Float importance = std::abs(invCosTheta * invCosTheta * invCosTheta / area);
    if (raster) *raster = Point2f(pRaster.x, pRaster.y);
    return Spectrum(importance);
}

Float PerspectiveCamera::Pdf(const Interaction &it, const Vector3f &w) const {
    return We(it, w).y();
}

Spectrum PerspectiveCamera::Sample_We(const Interaction &ref,
                                      const Point2f &sample, Vector3f *wi,
                                      Float *pdf, Point2f *raster,
                                      VisibilityTester *vis) const {
    // Uniformly sample a lens point _lensP_
    Point2f tmp = lensRadius * ConcentricSampleDisk(sample);
    Interaction lensP(CameraToWorld(ref.time, Point3f(tmp.x, tmp.y, 0)),
                      ref.time, medium);
    lensP.n = Normal3f(CameraToWorld(ref.time, Vector3f(0, 0, 1)));

    // Populate arguments and compute the importance value
    *vis = VisibilityTester(ref, lensP);
    *wi = lensP.p - ref.p;
    Float dist = wi->Length();
    *wi /= dist;
    *pdf = lensRadius == 0.f ? 1.0f : (1 / (Pi * lensRadius * lensRadius));
    return We(lensP, -*wi, raster) / (dist * dist);
}

PerspectiveCamera *CreatePerspectiveCamera(const ParamSet &params,
                                           const AnimatedTransform &cam2world,
                                           Film *film, const Medium *medium) {
    // Extract common camera parameters from _ParamSet_
    Float shutteropen = params.FindOneFloat("shutteropen", 0.f);
    Float shutterclose = params.FindOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        Warning("Shutter close time [%f] < shutter open [%f].  Swapping them.",
                shutterclose, shutteropen);
        std::swap(shutterclose, shutteropen);
    }
    Float lensradius = params.FindOneFloat("lensradius", 0.f);
    Float focaldistance = params.FindOneFloat("focaldistance", 1e30f);
    Float frame = params.FindOneFloat(
        "frameaspectratio",
        Float(film->fullResolution.x) / Float(film->fullResolution.y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    int swi;
    const Float *sw = params.FindFloat("screenwindow", &swi);
    if (sw) {
        if (swi == 4) {
            screen.pMin.x = sw[0];
            screen.pMax.x = sw[1];
            screen.pMin.y = sw[2];
            screen.pMax.y = sw[3];
        } else {
            Error("\"screenwindow\" should have four values");
        }
    }
    Float fov = params.FindOneFloat("fov", 90.);
    Float halffov = params.FindOneFloat("halffov", -1.f);
    if (halffov > 0.f)
        // hack for structure synth, which exports half of the full fov
        fov = 2.f * halffov;
    return new PerspectiveCamera(cam2world, screen, shutteropen, shutterclose,
                                 lensradius, focaldistance, fov, film, medium);
}
