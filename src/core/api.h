
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

#ifndef PBRT_CORE_API_H
#define PBRT_CORE_API_H

// core/api.h*
#include "pbrt.h"

// API Function Declarations
void pbrtInit(const Options &opt);
void pbrtCleanup();
void pbrtIdentity();
void pbrtTranslate(Float dx, Float dy, Float dz);
void pbrtRotate(Float angle, Float ax, Float ay, Float az);
void pbrtScale(Float sx, Float sy, Float sz);
void pbrtLookAt(Float ex, Float ey, Float ez, Float lx, Float ly, Float lz,
                Float ux, Float uy, Float uz);
void pbrtConcatTransform(Float transform[16]);
void pbrtTransform(Float transform[16]);
void pbrtCoordinateSystem(const std::string &);
void pbrtCoordSysTransform(const std::string &);
void pbrtActiveTransformAll();
void pbrtActiveTransformEndTime();
void pbrtActiveTransformStartTime();
void pbrtTransformTimes(Float start, Float end);
void pbrtPixelFilter(const std::string &name, const ParamSet &params);
void pbrtFilm(const std::string &type, const ParamSet &params);
void pbrtSampler(const std::string &name, const ParamSet &params);
void pbrtAccelerator(const std::string &name, const ParamSet &params);
void pbrtIntegrator(const std::string &name, const ParamSet &params);
void pbrtCamera(const std::string &, const ParamSet &cameraParams);
void pbrtMakeNamedMedium(const std::string &name, const ParamSet &params);
void pbrtMediumInterface(const std::string &insideName,
                         const std::string &outsideName);
void pbrtWorldBegin();
void pbrtAttributeBegin();
void pbrtAttributeEnd();
void pbrtTransformBegin();
void pbrtTransformEnd();
void pbrtTexture(const std::string &name, const std::string &type,
                 const std::string &texname, const ParamSet &params);
void pbrtMaterial(const std::string &name, const ParamSet &params);
void pbrtMakeNamedMaterial(const std::string &name, const ParamSet &params);
void pbrtNamedMaterial(const std::string &name);
void pbrtLightSource(const std::string &name, const ParamSet &params);
void pbrtAreaLightSource(const std::string &name, const ParamSet &params);
void pbrtShape(const std::string &name, const ParamSet &params);
void pbrtReverseOrientation();
void pbrtObjectBegin(const std::string &name);
void pbrtObjectEnd();
void pbrtObjectInstance(const std::string &name);
void pbrtWorldEnd();

#endif  // PBRT_CORE_API_H
