
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

#ifndef PBRT_CORE_IMAGE_H
#define PBRT_CORE_IMAGE_H

// core/image.h*
#include "pbrt.h"
#include "geometry.h"
#include "fp16.h"
#include "spectrum.h"
#include <array>

namespace pbrt {

extern Float LinearToSRGB[256];

///////////////////////////////////////////////////////////////////////////
// PixelFormat

// TODO: Y8 -> G8 (or GREY8?)
enum class PixelFormat { SY8, Y8, RGB8, SRGB8, Y16, RGB16, Y32, RGB32 };

inline bool Is8Bit(PixelFormat format) {
    return (format == PixelFormat::SY8 || format == PixelFormat::Y8 ||
            format == PixelFormat::SRGB8 || format == PixelFormat::RGB8);
}

inline bool Is16Bit(PixelFormat format) {
    return (format == PixelFormat::Y16 || format == PixelFormat::RGB16);
}

inline bool Is32Bit(PixelFormat format) {
    return (format == PixelFormat::Y32 || format == PixelFormat::RGB32);
}

inline int nChannels(PixelFormat format) {
    switch (format) {
    case PixelFormat::SY8:
    case PixelFormat::Y8:
    case PixelFormat::Y16:
    case PixelFormat::Y32:
        return 1;
    case PixelFormat::RGB8:
    case PixelFormat::SRGB8:
    case PixelFormat::RGB16:
    case PixelFormat::RGB32:
        return 3;
    }
}

inline int TexelBytes(PixelFormat format) {
    switch (format) {
    case PixelFormat::SY8:
    case PixelFormat::Y8:
        return 1;
    case PixelFormat::RGB8:
    case PixelFormat::SRGB8:
        return 3;
    case PixelFormat::Y16:
        return 2;
    case PixelFormat::RGB16:
        return 6;
    case PixelFormat::Y32:
        return 4;
    case PixelFormat::RGB32:
        return 12;
    default:
        LOG(ERROR) << "Unhandled PixelFormat in TexelBytes()";
    }
}

template <typename T>
static T ConvertTexel(const void *ptr, PixelFormat format) {
    T::unimplemented_function;
}

template <>
Spectrum ConvertTexel(const void *ptr, PixelFormat format);

template <>
Float ConvertTexel(const void *ptr, PixelFormat format) {
    if (nChannels(format) != 1)
        return ConvertTexel<Spectrum>(ptr, format).Average();
    if (ptr == nullptr) return 0;

    // TODO: are those pointer casts ok or not? ok if char I think, not
    // sure about uint8_t, strictly speaking...
    switch (format) {
    case PixelFormat::SY8:
        return LinearToSRGB[*((uint8_t *)ptr)];
    case PixelFormat::Y8:
        return Float(*((uint8_t *)ptr)) / 255.f;
    case PixelFormat::Y16:
        return HalfToFloat(*((uint16_t *)ptr));
    case PixelFormat::Y32:
        return Float(*((float *)ptr));
    default:
        LOG(FATAL) << "Unhandled PixelFormat";
    }
}

template <>
Spectrum ConvertTexel(const void *ptr, PixelFormat format) {
    if (nChannels(format) == 1)
        return Spectrum(ConvertTexel<Float>(ptr, format));
    if (ptr == nullptr) return Spectrum(0);

    CHECK_EQ(3, nChannels(format));
    Float rgb[3];
    for (int c = 0; c < 3; ++c) switch (format) {
        case PixelFormat::SRGB8:
            rgb[c] = LinearToSRGB[((uint8_t *)ptr)[c]];
            break;
        case PixelFormat::RGB8:
            rgb[c] = Float(((uint8_t *)ptr)[c]) / 255.f;
            break;
        case PixelFormat::RGB16:
            rgb[c] = HalfToFloat(((uint16_t *)ptr)[c]);
            break;
        case PixelFormat::RGB32:
            rgb[c] = Float(((float *)ptr)[c]);
            break;
        default:
            LOG(FATAL) << "Unhandled pixelformat";
        }

    // TODO: pass through illuminant/reflectance enum? (Or nix this whole
    // idea)...
    return Spectrum::FromRGB(rgb);
}

///////////////////////////////////////////////////////////////////////////
// WrapMode

enum class WrapMode { Repeat, Black, Clamp };

// TODO: use in imagemap
inline bool ParseWrapMode(const char *w, WrapMode *wrapMode) {
    if (!strcmp(w, "clamp")) {
        *wrapMode = WrapMode::Clamp;
        return true;
    } else if (!strcmp(w, "repeat")) {
        *wrapMode = WrapMode::Repeat;
        return true;
    } else if (!strcmp(w, "black")) {
        *wrapMode = WrapMode::Black;
        return true;
    }
    return false;
}

inline const char *WrapModeString(WrapMode mode) {
    switch (mode) {
    case WrapMode::Clamp:
        return "clamp";
    case WrapMode::Repeat:
        return "repeat";
    case WrapMode::Black:
        return "black";
    default:
        LOG(FATAL) << "Unhandled wrap mode";
        return nullptr;
    }
}

bool RemapPixelCoords(Point2i *p, Point2i resolution, WrapMode wrapMode);

// Important: coordinate system for our images has (0,0) at the
// upper left corner.
// Write code does this.

class Image {
  public:
    // TODO: array slice this up...
    Image() : format(PixelFormat::Y8), resolution(0, 0) {}
    Image(std::vector<uint8_t> p8, PixelFormat format, Point2i resolution);
    Image(std::vector<uint16_t> p16, PixelFormat format, Point2i resolution);
    Image(std::vector<float> p32, PixelFormat format, Point2i resolution);
    Image(PixelFormat format, Point2i resolution);

    // TODO: make gamme option more flexible: sRGB vs provided gamma
    // exponent...
    static bool Read(const std::string &filename, Image *image,
                     bool gamma = true, Bounds2i *dataWindow = nullptr,
                     Bounds2i *displayWindow = nullptr);
    bool Write(const std::string &name) const;
    bool Write(const std::string &name, const Bounds2i &pixelBounds,
               Point2i fullResolution) const;

    Image ConvertToFormat(PixelFormat format) const;

    // TODO? provide an iterator to iterate over all pixels and channels?

    Float GetChannel(Point2i p, int c,
                     WrapMode wrapMode = WrapMode::Clamp) const;
    Float GetY(Point2i p, WrapMode wrapMode = WrapMode::Clamp) const;
    Spectrum GetSpectrum(Point2i p,
                         SpectrumType spectrumType = SpectrumType::Reflectance,
                         WrapMode wrapMode = WrapMode::Clamp) const;

    Float BilerpChannel(Point2f p, int c,
                        WrapMode wrapMode = WrapMode::Clamp) const;
    Float BilerpY(Point2f p, WrapMode wrapMode = WrapMode::Clamp) const;
    Spectrum BilerpSpectrum(
        Point2f p, SpectrumType spectrumType = SpectrumType::Reflectance,
        WrapMode wrapMode = WrapMode::Clamp) const;

    void SetChannel(Point2i p, int c, Float value);
    void SetY(Point2i p, Float value);
    void SetSpectrum(Point2i p, const Spectrum &value);

    void Resize(Point2i newResolution, WrapMode wrap);
    void FlipY();
    std::vector<Image> GenerateMIPMap(WrapMode wrapMode) const;

    int nChannels() const { return pbrt::nChannels(format); }
    size_t BytesUsed() const {
        return p8.size() + 2 * p16.size() + 4 * p32.size();
    }

    PixelFormat format;
    Point2i resolution;

    size_t PixelOffset(Point2i p, int c = 0) const {
        CHECK(c >= 0 && c < nChannels());
        CHECK(InsideExclusive(p, Bounds2i({0, 0}, resolution)));
        return nChannels() * (p.y * resolution.x + p.x) + c;
    }
    const void *RawPointer(Point2i p) const {
        if (Is8Bit(format)) return p8.data() + PixelOffset(p);
        if (Is16Bit(format))
            return p16.data() + PixelOffset(p);
        else {
            CHECK(Is32Bit(format));
            return p32.data() + PixelOffset(p);
        }
    }
    void *RawPointer(Point2i p) {
        return const_cast<void *>(((const Image *)this)->RawPointer(p));
    }

  private:
    std::array<Float, 3> GetRGB(Point2i p, WrapMode wrapMode) const;

    bool WriteEXR(const std::string &name, const Bounds2i &pixelBounds,
                  Point2i fullResolution) const;
    bool WritePFM(const std::string &name) const;
    bool WritePNG(const std::string &name) const;
    bool WriteTGA(const std::string &name) const;

    std::vector<uint8_t> p8;
    std::vector<uint16_t> p16;
    std::vector<float> p32;
};

}  // namespace pbrt

#endif  // PBRT_CORE_IMAGE_H
