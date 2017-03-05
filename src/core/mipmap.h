
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

#ifndef PBRT_CORE_MIPMAP_H
#define PBRT_CORE_MIPMAP_H

// core/mipmap.h*
#include "pbrt.h"
#include "image.h"
#include "spectrum.h"
#include "texcache.h"

namespace pbrt {

enum class FilterFunction { Point, Bilinear, Trilinear, EWA };

static bool ParseFilter(const std::string &f, FilterFunction *func) {
    if (f == "ewa" || f == "EWA") {
        *func = FilterFunction::EWA;
        return true;
    } else if (f == "trilinear") {
        *func = FilterFunction::Trilinear;
        return true;
    } else if (f == "bilinear") {
        *func = FilterFunction::Bilinear;
        return true;
    } else if (f == "point") {
        *func = FilterFunction::Point;
        return true;
    } else
        return false;
}

struct MIPMapFilterOptions {
    FilterFunction filter = FilterFunction::EWA;
    Float maxAnisotropy = 8.f;
};

class TexelProvider {
  public:
    virtual ~TexelProvider();
    virtual int Levels() const = 0;
    virtual Point2i LevelResolution(int level) const = 0;
    virtual Float TexelFloat(int level, Point2i st) const = 0;
    virtual Spectrum TexelSpectrum(int level, Point2i st) const = 0;
    virtual Float BilerpFloat(int level, Point2f st) const = 0;
    virtual Spectrum BilerpSpectrum(int level, Point2f st) const = 0;
};

class ImageTexelProvider : public TexelProvider {
  public:
    ImageTexelProvider(Image image, WrapMode wrapMode,
                       SpectrumType spectrumType);

    int Levels() const override;
    Point2i LevelResolution(int level) const override;
    Float TexelFloat(int level, Point2i st) const override;
    Spectrum TexelSpectrum(int level, Point2i st) const override;
    Float BilerpFloat(int level, Point2f st) const override;
    Spectrum BilerpSpectrum(int level, Point2f st) const override;

  private:
    std::vector<Image> pyramid;
    const WrapMode wrapMode;
    const SpectrumType spectrumType;
};

class CachedTexelProvider : public TexelProvider {
  public:
    static std::unique_ptr<CachedTexelProvider> CreateFromFile(
        const std::string &filename, WrapMode wrapMode);
    ~CachedTexelProvider();

    int Levels() const override;
    Point2i LevelResolution(int level) const override;
    Float TexelFloat(int level, Point2i st) const override;
    Spectrum TexelSpectrum(int level, Point2i st) const override;
    Float BilerpFloat(int level, Point2f st) const override;
    Spectrum BilerpSpectrum(int level, Point2f st) const override;

    static TextureCache *textureCache;

  private:
    CachedTexelProvider(const std::string &filename, WrapMode wrapMode, int id);

    const std::string filename;
    const WrapMode wrapMode;
    const int id;
};

class MIPMap {
  public:
    MIPMap(std::unique_ptr<TexelProvider> tp,
           const MIPMapFilterOptions &options);
    static std::unique_ptr<MIPMap> CreateFromFile(
        const std::string &filename, const MIPMapFilterOptions &options,
        WrapMode wrapMode, bool gamma);
    ~MIPMap();

    template <typename T>
    T Lookup(const Point2f &st, Float width = 0.f) const;
    template <typename T>
    T Lookup(const Point2f &st, Vector2f dstdx, Vector2f dstdy) const;

  private:
    int Levels() const;
    Point2i LevelResolution(int level) const;
    template <typename T>
    T Texel(int level, Point2i st) const;
    template <typename T>
    T Bilerp(int level, Point2f st) const;
    template <typename T>
    T EWA(int level, Point2f st, Vector2f dst0, Vector2f dst1) const;

    std::unique_ptr<TexelProvider> texelProvider;
    MIPMapFilterOptions options;
};

}  // namespace pbrt

#endif  // PBRT_CORE_MIPMAP_H
