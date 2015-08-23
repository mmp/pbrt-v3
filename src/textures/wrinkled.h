
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

#ifndef PBRT_TEXTURES_WRINKLED_H
#define PBRT_TEXTURES_WRINKLED_H
#include "stdafx.h"

// textures/wrinkled.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"

// WrinkledTexture Declarations
template <typename T>
class WrinkledTexture : public Texture<T> {
  public:
    // WrinkledTexture Public Methods
    WrinkledTexture(std::unique_ptr<TextureMapping3D> mapping, int octaves,
                    Float omega)
        : mapping(std::move(mapping)), octaves(octaves), omega(omega) {}
    T Evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f p = mapping->Map(si, &dpdx, &dpdy);
        return Turbulence(p, dpdx, dpdy, omega, octaves);
    }

  private:
    // WrinkledTexture Private Data
    std::unique_ptr<TextureMapping3D> mapping;
    int octaves;
    Float omega;
};

WrinkledTexture<Float> *CreateWrinkledFloatTexture(const Transform &tex2world,
                                                   const TextureParams &tp);
WrinkledTexture<Spectrum> *CreateWrinkledSpectrumTexture(
    const Transform &tex2world, const TextureParams &tp);

#endif  // PBRT_TEXTURES_WRINKLED_H
