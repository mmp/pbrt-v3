
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

#include "mipmap.h"
#include <algorithm>
#include <array>
#include "fileutil.h"
#include "fp16.h"
#include "imageio.h"
#include "parallel.h"
#include "stats.h"
#include "texture.h"

namespace pbrt {

STAT_COUNTER("Texture/Texture map EWA lookups", nEWALookups);
STAT_COUNTER("Texture/Texture map trilinear lookups", nTrilinearLookups);
STAT_COUNTER("Texture/Texture map bilinear lookups", nBilinearLookups);
STAT_COUNTER("Texture/Texture map point lookups", nPointLookups);
STAT_MEMORY_COUNTER("Memory/Preloaded image maps", imageMapMemory);

///////////////////////////////////////////////////////////////////////////
// MIPMap Helper Declarations

TexelProvider::~TexelProvider() {}

TextureCache *CachedTexelProvider::textureCache;

std::unique_ptr<CachedTexelProvider> CachedTexelProvider::CreateFromFile(
    const std::string &filename, WrapMode wrapMode) {
    if (!textureCache) textureCache = new TextureCache;

    int id = textureCache->AddTexture(filename);
    if (id < 0) return nullptr;
    if (wrapMode != textureCache->GetWrapMode(id))
        Warning("%s: wrap mode in file, %s, doesn't match expected, %s.",
                filename.c_str(), WrapModeString(textureCache->GetWrapMode(id)),
                WrapModeString(wrapMode));

    return std::unique_ptr<CachedTexelProvider>(
        new CachedTexelProvider(filename, wrapMode, id));
}

CachedTexelProvider::CachedTexelProvider(const std::string &filename,
                                         WrapMode wrapMode, int id)
    : filename(filename), wrapMode(wrapMode), id(id) {}

CachedTexelProvider::~CachedTexelProvider() {}

int CachedTexelProvider::Levels() const {
    return textureCache->GetLevelResolution(id).size();
}

Point2i CachedTexelProvider::LevelResolution(int level) const {
    return textureCache->GetLevelResolution(id, level);
}

Float CachedTexelProvider::TexelFloat(int level, Point2i p) const {
    if (!RemapPixelCoords(&p, LevelResolution(level), wrapMode)) return 0;
    return textureCache->Texel<Float>(id, level, p);
}

Spectrum CachedTexelProvider::TexelSpectrum(int level, Point2i p) const {
    if (!RemapPixelCoords(&p, LevelResolution(level), wrapMode)) return 0;
    return textureCache->Texel<Spectrum>(id, level, p);
}

Float CachedTexelProvider::BilerpFloat(int level, Point2f st) const {
    Point2i resolution = textureCache->GetLevelResolution(id, level);
    Float s = st[0] * resolution.x - 0.5f;
    Float t = st[1] * resolution.y - 0.5f;
    int si = std::floor(s), ti = std::floor(t);

    Float ds = s - si, dt = t - ti;
    return ((1 - ds) * (1 - dt) * TexelFloat(level, {si, ti}) +
            (1 - ds) * dt * TexelFloat(level, {si, ti + 1}) +
            ds * (1 - dt) * TexelFloat(level, {si + 1, ti}) +
            ds * dt * TexelFloat(level, {si + 1, ti + 1}));
}

Spectrum CachedTexelProvider::BilerpSpectrum(int level, Point2f st) const {
    Point2i resolution = textureCache->GetLevelResolution(id, level);
    Float s = st[0] * resolution.x - 0.5f;
    Float t = st[1] * resolution.y - 0.5f;
    int si = std::floor(s), ti = std::floor(t);

    Float ds = s - si, dt = t - ti;
    return ((1 - ds) * (1 - dt) * TexelSpectrum(level, {si, ti}) +
            (1 - ds) * dt * TexelSpectrum(level, {si, ti + 1}) +
            ds * (1 - dt) * TexelSpectrum(level, {si + 1, ti}) +
            ds * dt * TexelSpectrum(level, {si + 1, ti + 1}));
}

///////////////////////////////////////////////////////////////////////////
// ImageTexelProvider

ImageTexelProvider::ImageTexelProvider(Image image, WrapMode wrapMode,
                                       SpectrumType spectrumType)
    : wrapMode(wrapMode), spectrumType(spectrumType) {
    pyramid = image.GenerateMIPMap(wrapMode);
    std::for_each(pyramid.begin(), pyramid.end(),
                  [](const Image &im) { imageMapMemory += im.BytesUsed(); });
}

Point2i ImageTexelProvider::LevelResolution(int level) const {
    CHECK(level >= 0 && level < pyramid.size());
    return pyramid[level].resolution;
}

int ImageTexelProvider::Levels() const { return int(pyramid.size()); }

Float ImageTexelProvider::TexelFloat(int level, Point2i st) const {
    CHECK(level >= 0 && level < pyramid.size());
    return pyramid[level].GetY(st, wrapMode);
}

Spectrum ImageTexelProvider::TexelSpectrum(int level, Point2i st) const {
    CHECK(level >= 0 && level < pyramid.size());
    return pyramid[level].GetSpectrum(st, spectrumType, wrapMode);
}

Float ImageTexelProvider::BilerpFloat(int level, Point2f st) const {
    CHECK(level >= 0 && level < pyramid.size());
    return pyramid[level].BilerpY(st, wrapMode);
}

Spectrum ImageTexelProvider::BilerpSpectrum(int level, Point2f st) const {
    CHECK(level >= 0 && level < pyramid.size());
    return pyramid[level].BilerpSpectrum(st, spectrumType, wrapMode);
}

///////////////////////////////////////////////////////////////////////////

static PBRT_CONSTEXPR int WeightLUTSize = 128;
static Float weightLut[WeightLUTSize];

// MIPMap Method Definitions
MIPMap::MIPMap(std::unique_ptr<TexelProvider> tp,
               const MIPMapFilterOptions &options)
    : texelProvider(std::move(tp)), options(options) {
    // Initialize EWA filter weights if needed
    if (weightLut[0] == 0.) {
        for (int i = 0; i < WeightLUTSize; ++i) {
            Float alpha = 2;
            Float r2 = Float(i) / Float(WeightLUTSize - 1);
            weightLut[i] = std::exp(-alpha * r2) - std::exp(-alpha);
        }
    }
}

std::unique_ptr<MIPMap> MIPMap::CreateFromFile(
    const std::string &filename, const MIPMapFilterOptions &options,
    WrapMode wrapMode, bool gamma) {
    ProfilePhase _(Prof::MIPMapCreation);

    if (HasExtension(filename, "txp")) {
        std::unique_ptr<TexelProvider> tp =
            CachedTexelProvider::CreateFromFile(filename, wrapMode);
        return std::unique_ptr<MIPMap>(new MIPMap(std::move(tp), options));
    } else {
        Image image;
        if (!Image::Read(filename, &image, gamma)) return nullptr;

        // TODO: make spectrum type configurable, or eliminate...
        std::unique_ptr<TexelProvider> tp(new ImageTexelProvider(
            std::move(image), wrapMode, SpectrumType::Reflectance));
        return std::unique_ptr<MIPMap>(new MIPMap(std::move(tp), options));
    }
}

MIPMap::~MIPMap() {}

int MIPMap::Levels() const { return texelProvider->Levels(); }

Point2i MIPMap::LevelResolution(int level) const {
    return texelProvider->LevelResolution(level);
}

template <typename T>
T MIPMap::Lookup(const Point2f &st, Float width) const {
    ProfilePhase p(Prof::TexFiltBasic);
    // Compute MIPMap level
    int nLevels = Levels();
    Float level = nLevels - 1 + Log2(std::max(width, (Float)1e-8));

    if (level >= Levels() - 1) return Texel<T>(Levels() - 1, {0, 0});

    int iLevel = std::max(0, int(std::floor(level)));
    if (options.filter == FilterFunction::Point) {
        ++nPointLookups;
        Point2i resolution = LevelResolution(iLevel);
        Point2i sti(std::round(st[0] * resolution[0] - 0.5f),
                    std::round(st[1] * resolution[1] - 0.5f));
        return Texel<T>(iLevel, sti);
    } else if (options.filter == FilterFunction::Bilinear) {
        ++nBilinearLookups;
        return Bilerp<T>(iLevel, st);
    } else {
        CHECK(options.filter == FilterFunction::Trilinear);
        ++nTrilinearLookups;

        if (iLevel == 0)
            return Bilerp<T>(0, st);
        else {
            Float delta = level - iLevel;
            CHECK_LE(delta, 1);
            return ((1 - delta) * Bilerp<T>(iLevel, st) +
                    delta * Bilerp<T>(iLevel + 1, st));
        }
    }
}

template <typename T>
T MIPMap::Lookup(const Point2f &st, Vector2f dst0, Vector2f dst1) const {
    if (options.filter != FilterFunction::EWA) {
        Float width = std::max(std::max(std::abs(dst0[0]), std::abs(dst0[1])),
                               std::max(std::abs(dst1[0]), std::abs(dst1[1])));
        return Lookup<T>(st, 2 * width);
    }
    ++nEWALookups;
    ProfilePhase p(Prof::TexFiltEWA);
    // Compute ellipse minor and major axes
    if (dst0.LengthSquared() < dst1.LengthSquared()) std::swap(dst0, dst1);
    Float majorLength = dst0.Length();
    Float minorLength = dst1.Length();

    // Clamp ellipse eccentricity if too large
    if (minorLength * options.maxAnisotropy < majorLength && minorLength > 0) {
        Float scale = majorLength / (minorLength * options.maxAnisotropy);
        dst1 *= scale;
        minorLength *= scale;
    }
    if (minorLength == 0) return Bilerp<T>(0, st);

    // Choose level of detail for EWA lookup and perform EWA filtering
    Float lod = std::max((Float)0, Levels() - (Float)1 + Log2(minorLength));
    int ilod = std::floor(lod);
    // TODO: just do return EWA<T>(ilog, st, dst0, dst1);
    // TODO: also, when scaling camera ray differentials, just do e.g.
    // 1 / std::min(sqrtSamplesPerPixel, 8);
    return ((1 - (lod - ilod)) * EWA<T>(ilod, st, dst0, dst1) +
            (lod - ilod) * EWA<T>(ilod + 1, st, dst0, dst1));
}

template <typename T>
T MIPMap::Texel(int level, Point2i st) const {
    T::unimplemented_function;
}

template <>
Float MIPMap::Texel(int level, Point2i st) const {
    return texelProvider->TexelFloat(level, st);
}

template <>
Spectrum MIPMap::Texel(int level, Point2i st) const {
    return texelProvider->TexelSpectrum(level, st);
}

template <typename T>
T MIPMap::Bilerp(int level, Point2f st) const {
    T::unimplemented_function;
}

template <>
Float MIPMap::Bilerp(int level, Point2f st) const {
    return texelProvider->BilerpFloat(level, st);
}

template <>
Spectrum MIPMap::Bilerp(int level, Point2f st) const {
    return texelProvider->BilerpSpectrum(level, st);
}

template <typename T>
T MIPMap::EWA(int level, Point2f st, Vector2f dst0, Vector2f dst1) const {
    if (level >= Levels()) return Texel<T>(Levels() - 1, {0, 0});

    // Convert EWA coordinates to appropriate scale for level
    Point2i levelRes = LevelResolution(level);
    st[0] = st[0] * levelRes[0] - 0.5f;
    st[1] = st[1] * levelRes[1] - 0.5f;
    dst0[0] *= levelRes[0];
    dst0[1] *= levelRes[1];
    dst1[0] *= levelRes[0];
    dst1[1] *= levelRes[1];

    // Compute ellipse coefficients to bound EWA filter region
    Float A = dst0[1] * dst0[1] + dst1[1] * dst1[1] + 1;
    Float B = -2 * (dst0[0] * dst0[1] + dst1[0] * dst1[1]);
    Float C = dst0[0] * dst0[0] + dst1[0] * dst1[0] + 1;
    Float invF = 1 / (A * C - B * B * 0.25f);
    A *= invF;
    B *= invF;
    C *= invF;

    // Compute the ellipse's $(s,t)$ bounding box in texture space
    Float det = -B * B + 4 * A * C;
    Float invDet = 1 / det;
    Float uSqrt = std::sqrt(det * C), vSqrt = std::sqrt(A * det);
    int s0 = std::ceil(st[0] - 2 * invDet * uSqrt);
    int s1 = std::floor(st[0] + 2 * invDet * uSqrt);
    int t0 = std::ceil(st[1] - 2 * invDet * vSqrt);
    int t1 = std::floor(st[1] + 2 * invDet * vSqrt);

    // Scan over ellipse bound and compute quadratic equation
    T sum{};
    Float sumWts = 0;
    for (int it = t0; it <= t1; ++it) {
        Float tt = it - st[1];
        for (int is = s0; is <= s1; ++is) {
            Float ss = is - st[0];
            // Compute squared radius and filter texel if inside ellipse
            Float r2 = A * ss * ss + B * ss * tt + C * tt * tt;
            if (r2 < 1) {
                int index =
                    std::min((int)(r2 * WeightLUTSize), WeightLUTSize - 1);
                Float weight = weightLut[index];
                sum += weight * Texel<T>(level, {is, it});
                sumWts += weight;
            }
        }
    }
    return sum / sumWts;
}

// Explicit template instantiation..
template Float MIPMap::Lookup(const Point2f &st, Float width) const;
template Spectrum MIPMap::Lookup(const Point2f &st, Float width) const;
template Float MIPMap::Lookup(const Point2f &st, Vector2f, Vector2f) const;
template Spectrum MIPMap::Lookup(const Point2f &st, Vector2f, Vector2f) const;

}  // namespace pbrt
