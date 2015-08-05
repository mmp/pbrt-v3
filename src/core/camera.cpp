
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

// core/camera.cpp*
#include "camera.h"
#include "film.h"
#include "sampling.h"
#include "sampler.h"

// Camera Method Definitions
Camera::~Camera() { delete film; }

Camera::Camera(const AnimatedTransform &CameraToWorld, Float shutterOpen,
               Float shutterClose, Film *film, const Medium *medium)
    : CameraToWorld(CameraToWorld),
      shutterOpen(shutterOpen),
      shutterClose(shutterClose),
      film(film),
      medium(medium) {
    if (CameraToWorld.HasScale())
        Warning(
            "Scaling detected in world-to-camera transformation!\n"
            "The system has numerous assumptions, implicit and explicit,\n"
            "that this transform will have no scale factors in it.\n"
            "Proceed at your own risk; your image may have errors or\n"
            "the system may crash as a result of this.");
}

Float Camera::GenerateRayDifferential(const CameraSample &sample,
                                      RayDifferential *rd) const {
    Float wt = GenerateRay(sample, rd);
    // Find camera ray after shifting one pixel in the $x$ direction
    CameraSample sshift = sample;
    sshift.pFilm.x++;
    Ray rx;
    Float wtx = GenerateRay(sshift, &rx);
    if (wtx == 0.f) return 0.f;
    rd->rxOrigin = rx.o;
    rd->rxDirection = rx.d;

    // Find camera ray after shifting one pixel in the $y$ direction
    sshift.pFilm.x--;
    sshift.pFilm.y++;
    Ray ry;
    Float wty = GenerateRay(sshift, &ry);
    if (wty == 0.f) return 0.f;
    rd->ryOrigin = ry.o;
    rd->ryDirection = ry.d;
    rd->hasDifferentials = true;
    return wt;
}

ProjectiveCamera::ProjectiveCamera(const AnimatedTransform &CameraToWorld,
                                   const Transform &CameraToScreen,
                                   const Bounds2f &screenWindow,
                                   Float shutterOpen, Float shutterClose,
                                   Float lensr, Float focald, Film *film,
                                   const Medium *medium)
    : Camera(CameraToWorld, shutterOpen, shutterClose, film, medium),
      CameraToScreen(CameraToScreen) {
    // Initialize depth of field parameters
    lensRadius = lensr;
    focalDistance = focald;

    // Compute projective camera transformations

    // Compute projective camera screen transformations
    ScreenToRaster =
        Scale(film->fullResolution.x, film->fullResolution.y, 1.f) *
        Scale(1.f / (screenWindow.pMax.x - screenWindow.pMin.x),
              1.f / (screenWindow.pMin.y - screenWindow.pMax.y), 1.f) *
        Translate(Vector3f(-screenWindow.pMin.x, -screenWindow.pMax.y, 0.f));
    RasterToScreen = Inverse(ScreenToRaster);
    RasterToCamera = Inverse(CameraToScreen) * RasterToScreen;
}

Spectrum Camera::We(const Interaction &p0, const Vector3f &w,
                    Point2f *pRasterPtr) const {
    Severe("Camera::We() is not implemented!");
    return Spectrum(0.f);
}

Float Camera::Pdf_We(const Interaction &p0, const Vector3f &w) const {
    Severe("Camera::Pdf_We() is not implemented!");
    return 0.f;
}

Spectrum Camera::Sample_Wi(const Interaction &ref, const Point2f &u,
                           Vector3f *wi, Float *pdf, Point2f *pRaster,
                           VisibilityTester *vis) const {
    Severe("Camera::Sample_Wi() is not implemented!");
    return Spectrum(0.f);
}
