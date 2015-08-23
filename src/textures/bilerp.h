
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

#ifndef PBRT_TEXTURES_BILERP_H
#define PBRT_TEXTURES_BILERP_H
#include "stdafx.h"

// textures/bilerp.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"

// BilerpTexture Declarations
template <typename T>
class BilerpTexture : public Texture<T> {
  public:
    // BilerpTexture Public Methods
    BilerpTexture(std::unique_ptr<TextureMapping2D> mapping, const T &v00,
                  const T &v01, const T &v10, const T &v11)
        : mapping(std::move(mapping)), v00(v00), v01(v01), v10(v10), v11(v11) {}
    T Evaluate(const SurfaceInteraction &si) const {
        Vector2f dstdx, dstdy;
        Point2f st = mapping->Map(si, &dstdx, &dstdy);
        return (1 - st[0]) * (1 - st[1]) * v00 + (1 - st[0]) * (st[1]) * v01 +
               (st[0]) * (1 - st[1]) * v10 + (st[0]) * (st[1]) * v11;
    }

  private:
    // BilerpTexture Private Data
    std::unique_ptr<TextureMapping2D> mapping;
    const T v00, v01, v10, v11;
};

BilerpTexture<Float> *CreateBilerpFloatTexture(const Transform &tex2world,
                                               const TextureParams &tp);
BilerpTexture<Spectrum> *CreateBilerpSpectrumTexture(const Transform &tex2world,
                                                     const TextureParams &tp);

#endif  // PBRT_TEXTURES_BILERP_H
