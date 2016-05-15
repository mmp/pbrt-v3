
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

#ifndef PBRT_TEXTURES_CHECKERBOARD_H
#define PBRT_TEXTURES_CHECKERBOARD_H

// textures/checkerboard.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"

// AAMethod Declaration
enum class AAMethod { None, ClosedForm };

// CheckerboardTexture Declarations
template <typename T>
class Checkerboard2DTexture : public Texture<T> {
  public:
    // Checkerboard2DTexture Public Methods
    Checkerboard2DTexture(std::unique_ptr<TextureMapping2D> mapping,
                          const std::shared_ptr<Texture<T>> &tex1,
                          const std::shared_ptr<Texture<T>> &tex2,
                          AAMethod aaMethod)
        : mapping(std::move(mapping)),
          tex1(tex1),
          tex2(tex2),
          aaMethod(aaMethod) {}
    T Evaluate(const SurfaceInteraction &si) const {
        Vector2f dstdx, dstdy;
        Point2f st = mapping->Map(si, &dstdx, &dstdy);
        if (aaMethod == AAMethod::None) {
            // Point sample _Checkerboard2DTexture_
            if (((int)std::floor(st[0]) + (int)std::floor(st[1])) % 2 == 0)
                return tex1->Evaluate(si);
            return tex2->Evaluate(si);
        } else {
            // Compute closed-form box-filtered _Checkerboard2DTexture_ value

            // Evaluate single check if filter is entirely inside one of them
            Float ds = std::max(std::abs(dstdx[0]), std::abs(dstdy[0]));
            Float dt = std::max(std::abs(dstdx[1]), std::abs(dstdy[1]));
            Float s0 = st[0] - ds, s1 = st[0] + ds;
            Float t0 = st[1] - dt, t1 = st[1] + dt;
            if (std::floor(s0) == std::floor(s1) &&
                std::floor(t0) == std::floor(t1)) {
                // Point sample _Checkerboard2DTexture_
                if (((int)std::floor(st[0]) + (int)std::floor(st[1])) % 2 == 0)
                    return tex1->Evaluate(si);
                return tex2->Evaluate(si);
            }

            // Apply box filter to checkerboard region
            auto bumpInt = [](Float x) {
                return (int)std::floor(x / 2) +
                       2 * std::max(x / 2 - (int)std::floor(x / 2) - (Float)0.5,
                                    (Float)0);
            };
            Float sint = (bumpInt(s1) - bumpInt(s0)) / (2 * ds);
            Float tint = (bumpInt(t1) - bumpInt(t0)) / (2 * dt);
            Float area2 = sint + tint - 2 * sint * tint;
            if (ds > 1 || dt > 1) area2 = .5f;
            return (1 - area2) * tex1->Evaluate(si) +
                   area2 * tex2->Evaluate(si);
        }
    }

  private:
    // Checkerboard2DTexture Private Data
    std::unique_ptr<TextureMapping2D> mapping;
    const std::shared_ptr<Texture<T>> tex1, tex2;
    const AAMethod aaMethod;
};

template <typename T>
class Checkerboard3DTexture : public Texture<T> {
  public:
    // Checkerboard3DTexture Public Methods
    Checkerboard3DTexture(std::unique_ptr<TextureMapping3D> mapping,
                          const std::shared_ptr<Texture<T>> &tex1,
                          const std::shared_ptr<Texture<T>> &tex2)
        : mapping(std::move(mapping)), tex1(tex1), tex2(tex2) {}
    T Evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f p = mapping->Map(si, &dpdx, &dpdy);
        if (((int)std::floor(p.x) + (int)std::floor(p.y) +
             (int)std::floor(p.z)) %
                2 ==
            0)
            return tex1->Evaluate(si);
        else
            return tex2->Evaluate(si);
    }

  private:
    // Checkerboard3DTexture Private Data
    std::unique_ptr<TextureMapping3D> mapping;
    std::shared_ptr<Texture<T>> tex1, tex2;
};

Texture<Float> *CreateCheckerboardFloatTexture(const Transform &tex2world,
                                               const TextureParams &tp);
Texture<Spectrum> *CreateCheckerboardSpectrumTexture(const Transform &tex2world,
                                                     const TextureParams &tp);

#endif  // PBRT_TEXTURES_CHECKERBOARD_H
