
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

#ifndef PBRT_TEXTURES_IMAGEMAP_H
#define PBRT_TEXTURES_IMAGEMAP_H

// textures/imagemap.h*
#include "pbrt.h"
#include "texture.h"
#include "mipmap.h"
#include "paramset.h"
#include <map>

namespace pbrt {

// TexInfo Declarations
struct TexInfo {
    TexInfo(const std::string &f, const std::string &filt, Float ma, WrapMode wm,
            bool gamma)
        : filename(f),
          filter(filt),
          maxAniso(ma),
          wrapMode(wm),
          gamma(gamma) {}
    std::string filename;
    std::string filter;
    Float maxAniso;
    WrapMode wrapMode;
    bool gamma;
    bool operator<(const TexInfo &t2) const {
      return std::tie(filename, filter, maxAniso, gamma, wrapMode) <
          std::tie(t2.filename, t2.filter, t2.maxAniso, t2.gamma, t2.wrapMode);
    }
};

// ImageTexture Declarations
template <typename T>
class ImageTexture : public Texture<T> {
  public:
    // ImageTexture Public Methods
    ImageTexture(std::unique_ptr<TextureMapping2D> m,
                 const std::string &filename, const std::string &filter,
                 Float maxAniso, WrapMode wm, Float scale, bool gamma);
    static void ClearCache() {
        textures.erase(textures.begin(), textures.end());
    }
    T Evaluate(const SurfaceInteraction &si) const {
        if (!mipmap) return T(scale);
        Vector2f dstdx, dstdy;
        Point2f st = mapping->Map(si, &dstdx, &dstdy);
        // Texture coordinates are (0,0) in the lower left corner, but
        // image coordinates are (0,0) in the upper left.
        st[1] = 1 - st[1];
        return scale * mipmap->template Lookup<T>(st, dstdx, dstdy);
    }

  private:
    // ImageTexture Private Methods
    static MIPMap *GetTexture(
        const std::string &filename, const std::string &filter, Float maxAniso,
        WrapMode wm, bool gamma);

    // ImageTexture Private Data
    std::unique_ptr<TextureMapping2D> mapping;
    const Float scale;
    MIPMap *mipmap;
    static std::map<TexInfo, std::unique_ptr<MIPMap>> textures;
};

ImageTexture<Float> *CreateImageFloatTexture(const Transform &tex2world,
                                             const TextureParams &tp);
ImageTexture<Spectrum> *CreateImageSpectrumTexture(
    const Transform &tex2world, const TextureParams &tp);

}  // namespace pbrt

#endif  // PBRT_TEXTURES_IMAGEMAP_H
