
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

#ifndef PBRT_TEXTURES_FBM_H
#define PBRT_TEXTURES_FBM_H

// textures/fbm.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"

// FBmTexture Declarations
template <typename T>
class FBmTexture : public Texture<T> {
  public:
    // FBmTexture Public Methods
    FBmTexture(std::unique_ptr<TextureMapping3D> mapping, int octaves,
               Float omega)
        : mapping(std::move(mapping)), omega(omega), octaves(octaves) {}
    T Evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f P = mapping->Map(si, &dpdx, &dpdy);
        return FBm(P, dpdx, dpdy, omega, octaves);
    }

  private:
    std::unique_ptr<TextureMapping3D> mapping;
    const Float omega;
    const int octaves;
};

FBmTexture<Float> *CreateFBmFloatTexture(const Transform &tex2world,
                                         const TextureParams &tp);
FBmTexture<Spectrum> *CreateFBmSpectrumTexture(const Transform &tex2world,
                                               const TextureParams &tp);

#endif  // PBRT_TEXTURES_FBM_H
