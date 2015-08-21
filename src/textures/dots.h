
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

#ifndef PBRT_TEXTURES_DOTS_H
#define PBRT_TEXTURES_DOTS_H
#include "stdafx.h"

// textures/dots.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"

// DotsTexture Declarations
template <typename T>
class DotsTexture : public Texture<T> {
  public:
    // DotsTexture Public Methods
    DotsTexture(std::unique_ptr<TextureMapping2D> mapping,
                const std::shared_ptr<Texture<T>> &outsideDot,
                const std::shared_ptr<Texture<T>> &insideDot)
        : mapping(std::move(mapping)),
          outsideDot(outsideDot),
          insideDot(insideDot) {}
    T Evaluate(const SurfaceInteraction &si) const {
        // Compute cell indices for dots
        Vector2f dstdx, dstdy;
        Point2f st = mapping->Map(si, &dstdx, &dstdy);
        int sCell = std::floor(st[0] + .5f), tCell = std::floor(st[1] + .5f);

        // Return _insideDot_ result if point is inside dot
        if (Noise(sCell + .5f, tCell + .5f) > 0) {
            Float radius = .35f;
            Float maxShift = 0.5f - radius;
            Float sCenter =
                sCell + maxShift * Noise(sCell + 1.5f, tCell + 2.8f);
            Float tCenter =
                tCell + maxShift * Noise(sCell + 4.5f, tCell + 9.8f);
            Vector2f dst = st - Point2f(sCenter, tCenter);
            if (dst.LengthSquared() < radius * radius)
                return insideDot->Evaluate(si);
        }
        return outsideDot->Evaluate(si);
    }

  private:
    // DotsTexture Private Data
    std::unique_ptr<TextureMapping2D> mapping;
    std::shared_ptr<Texture<T>> outsideDot, insideDot;
};

DotsTexture<Float> *CreateDotsFloatTexture(const Transform &tex2world,
                                           const TextureParams &tp);
DotsTexture<Spectrum> *CreateDotsSpectrumTexture(const Transform &tex2world,
                                                 const TextureParams &tp);

#endif  // PBRT_TEXTURES_DOTS_H
