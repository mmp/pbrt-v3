
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

#ifndef PBRT_CAMERAS_REALISTIC_H
#define PBRT_CAMERAS_REALISTIC_H
#include "stdafx.h"

// cameras/realistic.h*
#include "pbrt.h"
#include "camera.h"
#include "film.h"

// RealisticCamera Declarations
class RealisticCamera : public Camera {
  public:
    // RealisticCamera Public Methods
    RealisticCamera(const AnimatedTransform &CameraToWorld, Float shutterOpen,
                    Float shutterClose, Float apertureDiameter,
                    Float focusDistance, bool simpleWeighting,
                    const char *lensFile, Film *film, const Medium *medium);
    Float GenerateRay(const CameraSample &sample, Ray *) const;

  private:
    // RealisticCamera Private Declarations
    struct LensElementInterface {
        Float curvatureRadius;
        Float thickness;
        Float eta;
        Float apertureRadius;
    };

    // RealisticCamera Private Data
    const bool simpleWeighting;
    std::vector<LensElementInterface> elementInterfaces;
    std::vector<Bounds2f> exitPupilBounds;

    // RealisticCamera Private Methods
    Float LensRearZ() const { return elementInterfaces.back().thickness; }
    Float LensFrontZ() const {
        Float zSum = 0;
        for (const LensElementInterface &element : elementInterfaces)
            zSum += element.thickness;
        return zSum;
    }
    Float RearElementRadius() const {
        return elementInterfaces.back().apertureRadius;
    }
    bool TraceLensesFromFilm(const Ray &ray, Ray *rOut) const;
    static bool IntersectSphericalElement(Float radius, Float center,
                                          const Ray &ray, Float *t,
                                          Normal3f *n);
    bool TraceLensesFromScene(const Ray &ray, Ray *rOut) const;
    void DrawLensSystem() const;
    void DrawRayPathFromFilm(const Ray &r, bool arrow,
                             bool toOpticalIntercept) const;
    void DrawRayPathFromScene(const Ray &r, bool arrow,
                              bool toOpticalIntercept) const;
    static void ComputeCardinalPoints(const Ray &rIn, const Ray &rOut, Float *p,
                                      Float *f);
    void ComputeThickLensApproximation(Float pz[2], Float f[2]) const;
    Float FocusThickLens(Float focusDistance);
    Float FocusBinarySearch(Float focusDistance);
    Float FocusDistance(Float filmDist);
    Bounds2f BoundExitPupil(Float pFilmX0, Float pFilmX1) const;
    void RenderExitPupil(Float sx, Float sy, const char *filename) const;
    Point3f SampleExitPupil(const Point2f &pFilm,
                            const Point2f &lensSample) const;
    void TestExitPupilBounds() const;
    Float ExitPupilPdf(const Point3f &pFilm, const Point3f &pExitPupil) const;
};

RealisticCamera *CreateRealisticCamera(const ParamSet &params,
                                       const AnimatedTransform &cam2world,
                                       Film *film, const Medium *medium);

#endif  // PBRT_CAMERAS_REALISTIC_H
