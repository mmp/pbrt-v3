
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

// core/image.cpp*
#include "image.h"
#include "parallel.h"
#include "texture.h"

namespace pbrt {

Float LinearToSRGB[256] = {
    0.0000000000, 0.0003035270, 0.0006070540, 0.0009105810, 0.0012141080,
    0.0015176350, 0.0018211619, 0.0021246888, 0.0024282159, 0.0027317430,
    0.0030352699, 0.0033465356, 0.0036765069, 0.0040247170, 0.0043914421,
    0.0047769533, 0.0051815170, 0.0056053917, 0.0060488326, 0.0065120910,
    0.0069954102, 0.0074990317, 0.0080231922, 0.0085681248, 0.0091340570,
    0.0097212177, 0.0103298230, 0.0109600937, 0.0116122449, 0.0122864870,
    0.0129830306, 0.0137020806, 0.0144438436, 0.0152085144, 0.0159962922,
    0.0168073755, 0.0176419523, 0.0185002182, 0.0193823613, 0.0202885624,
    0.0212190095, 0.0221738834, 0.0231533647, 0.0241576303, 0.0251868572,
    0.0262412224, 0.0273208916, 0.0284260381, 0.0295568332, 0.0307134409,
    0.0318960287, 0.0331047624, 0.0343398079, 0.0356013142, 0.0368894450,
    0.0382043645, 0.0395462364, 0.0409151986, 0.0423114114, 0.0437350273,
    0.0451862030, 0.0466650836, 0.0481718220, 0.0497065634, 0.0512694679,
    0.0528606549, 0.0544802807, 0.0561284944, 0.0578054339, 0.0595112406,
    0.0612460710, 0.0630100295, 0.0648032799, 0.0666259527, 0.0684781820,
    0.0703601092, 0.0722718611, 0.0742135793, 0.0761853904, 0.0781874284,
    0.0802198276, 0.0822827145, 0.0843762159, 0.0865004659, 0.0886556059,
    0.0908417329, 0.0930589810, 0.0953074843, 0.0975873619, 0.0998987406,
    0.1022417471, 0.1046164930, 0.1070231125, 0.1094617173, 0.1119324341,
    0.1144353822, 0.1169706732, 0.1195384338, 0.1221387982, 0.1247718409,
    0.1274376959, 0.1301364899, 0.1328683347, 0.1356333494, 0.1384316236,
    0.1412633061, 0.1441284865, 0.1470272839, 0.1499598026, 0.1529261619,
    0.1559264660, 0.1589608639, 0.1620294005, 0.1651322246, 0.1682693958,
    0.1714410931, 0.1746473908, 0.1778884083, 0.1811642349, 0.1844749898,
    0.1878207624, 0.1912016720, 0.1946178079, 0.1980693042, 0.2015562356,
    0.2050787061, 0.2086368501, 0.2122307271, 0.2158605307, 0.2195262313,
    0.2232279778, 0.2269658893, 0.2307400703, 0.2345506549, 0.2383976579,
    0.2422811985, 0.2462013960, 0.2501583695, 0.2541521788, 0.2581829131,
    0.2622507215, 0.2663556635, 0.2704978585, 0.2746773660, 0.2788943350,
    0.2831487954, 0.2874408960, 0.2917706966, 0.2961383164, 0.3005438447,
    0.3049873710, 0.3094689548, 0.3139887452, 0.3185468316, 0.3231432438,
    0.3277781308, 0.3324515820, 0.3371636569, 0.3419144452, 0.3467040956,
    0.3515326977, 0.3564002514, 0.3613068759, 0.3662526906, 0.3712377846,
    0.3762622178, 0.3813261092, 0.3864295185, 0.3915725648, 0.3967553079,
    0.4019778669, 0.4072403014, 0.4125427008, 0.4178851545, 0.4232677519,
    0.4286905527, 0.4341537058, 0.4396572411, 0.4452012479, 0.4507858455,
    0.4564110637, 0.4620770514, 0.4677838385, 0.4735315442, 0.4793202281,
    0.4851499796, 0.4910208881, 0.4969330430, 0.5028865933, 0.5088814497,
    0.5149177909, 0.5209956765, 0.5271152258, 0.5332764983, 0.5394796133,
    0.5457245708, 0.5520114899, 0.5583404899, 0.5647116303, 0.5711249113,
    0.5775805116, 0.5840784907, 0.5906189084, 0.5972018838, 0.6038274169,
    0.6104956269, 0.6172066331, 0.6239604354, 0.6307572126, 0.6375969648,
    0.6444797516, 0.6514056921, 0.6583748460, 0.6653873324, 0.6724432111,
    0.6795425415, 0.6866854429, 0.6938719153, 0.7011020184, 0.7083759308,
    0.7156936526, 0.7230552435, 0.7304608822, 0.7379105687, 0.7454043627,
    0.7529423237, 0.7605246305, 0.7681512833, 0.7758223414, 0.7835379243,
    0.7912980318, 0.7991028428, 0.8069523573, 0.8148466945, 0.8227858543,
    0.8307699561, 0.8387991190, 0.8468732834, 0.8549926877, 0.8631572723,
    0.8713672161, 0.8796223402, 0.8879231811, 0.8962693810, 0.9046613574,
    0.9130986929, 0.9215820432, 0.9301108718, 0.9386858940, 0.9473065734,
    0.9559735060, 0.9646862745, 0.9734454751, 0.9822505713, 0.9911022186,
    1.0000000000};

bool RemapPixelCoords(Point2i *p, Point2i resolution, WrapMode wrapMode) {
    switch (wrapMode) {
    case WrapMode::Repeat:
        (*p)[0] = Mod((*p)[0], resolution[0]);
        (*p)[1] = Mod((*p)[1], resolution[1]);
        return true;
    case WrapMode::Clamp:
        (*p)[0] = Clamp((*p)[0], 0, resolution[0] - 1);
        (*p)[1] = Clamp((*p)[1], 0, resolution[1] - 1);
        return true;
    case WrapMode::Black:
        return ((*p)[0] >= 0 && (*p)[0] < resolution[0] && (*p)[1] >= 0 &&
                (*p)[1] < resolution[1]);
    default:
        LOG(ERROR) << "Unhandled WrapMode mode";
    }
}

Image::Image(PixelFormat format, Point2i resolution)
    : format(format), resolution(resolution) {
    if (Is8Bit(format))
        p8.resize(nChannels() * resolution[0] * resolution[1]);
    else if (Is16Bit(format))
        p16.resize(nChannels() * resolution[0] * resolution[1]);
    else if (Is32Bit(format))
        p32.resize(nChannels() * resolution[0] * resolution[1]);
    else
        LOG(FATAL) << "Unhandled format in Image::Image()";
}

Image::Image(std::vector<uint8_t> p8c, PixelFormat format, Point2i resolution)
    : format(format), resolution(resolution), p8(std::move(p8c)) {
    CHECK_EQ(p8.size(), nChannels() * resolution[0] * resolution[1]);
    CHECK(Is8Bit(format));
}

Image::Image(std::vector<uint16_t> p16c, PixelFormat format, Point2i resolution)
    : format(format), resolution(resolution), p16(std::move(p16c)) {
    CHECK_EQ(p16.size(), nChannels() * resolution[0] * resolution[1]);
    CHECK(Is16Bit(format));
}

Image::Image(std::vector<float> p32c, PixelFormat format, Point2i resolution)
    : format(format), resolution(resolution), p32(std::move(p32c)) {
    CHECK_EQ(p32.size(), nChannels() * resolution[0] * resolution[1]);
    CHECK(Is32Bit(format));
}

Image Image::ConvertToFormat(PixelFormat newFormat) const {
    if (newFormat == format) return *this;
    CHECK_EQ(pbrt::nChannels(newFormat), nChannels());

    Image newImage(newFormat, resolution);
    int nc = nChannels();
    for (int y = 0; y < resolution.y; ++y)
        for (int x = 0; x < resolution.x; ++x)
            for (int c = 0; c < nc; ++c)
                newImage.SetChannel({x, y}, c, GetChannel({x, y}, c));
    return newImage;
}

Float Image::GetChannel(Point2i p, int c, WrapMode wrapMode) const {
    if (!RemapPixelCoords(&p, resolution, wrapMode)) return 0;

    // Use convert()? Some rewrite/refactor?
    switch (format) {
    case PixelFormat::SY8:
    case PixelFormat::SRGB8:
        return LinearToSRGB[p8[PixelOffset(p, c)]];
    case PixelFormat::Y8:
    case PixelFormat::RGB8:
        return Float(p8[PixelOffset(p, c)]) / 255.f;
    case PixelFormat::Y16:
    case PixelFormat::RGB16:
        return HalfToFloat(p16[PixelOffset(p, c)]);
    case PixelFormat::Y32:
    case PixelFormat::RGB32:
        return Float(p32[PixelOffset(p, c)]);
    default:
        LOG(FATAL) << "Unhandled PixelFormat";
    }
}

Float Image::GetY(Point2i p, WrapMode wrapMode) const {
    if (nChannels() == 1) return GetChannel(p, 0, wrapMode);
    CHECK_EQ(3, nChannels());
    std::array<Float, 3> rgb = GetRGB(p, wrapMode);
    // FIXME: um, this isn't luminance as we think of it...
    return (rgb[0] + rgb[1] + rgb[2]) / 3;
}

std::array<Float, 3> Image::GetRGB(Point2i p, WrapMode wrapMode) const {
    CHECK_EQ(3, nChannels());

    if (!RemapPixelCoords(&p, resolution, wrapMode))
        return {(Float)0, (Float)0, (Float)0};

    std::array<Float, 3> rgb;
    switch (format) {
    case PixelFormat::SRGB8:
        for (int c = 0; c < 3; ++c)
            rgb[c] = LinearToSRGB[p8[PixelOffset(p, c)]];
        break;
    case PixelFormat::RGB8:
        for (int c = 0; c < 3; ++c)
            rgb[c] = Float(p8[PixelOffset(p, c)]) / 255.f;
        break;
    case PixelFormat::RGB16:
        for (int c = 0; c < 3; ++c)
            rgb[c] = HalfToFloat(p16[PixelOffset(p, c)]);
        break;
    case PixelFormat::RGB32:
        for (int c = 0; c < 3; ++c) rgb[c] = p32[PixelOffset(p, c)];
        break;
    default:
        LOG(FATAL) << "Unhandled PixelFormat";
    }

    return rgb;
}

Spectrum Image::GetSpectrum(Point2i p, SpectrumType spectrumType,
                            WrapMode wrapMode) const {
    if (nChannels() == 1) return GetChannel(p, 0, wrapMode);
    std::array<Float, 3> rgb = GetRGB(p, wrapMode);
    return Spectrum::FromRGB(&rgb[0], spectrumType);
}

Float Image::BilerpChannel(Point2f p, int c, WrapMode wrapMode) const {
    Float s = p[0] * resolution.x - 0.5f;
    Float t = p[1] * resolution.y - 0.5f;
    int si = std::floor(s), ti = std::floor(t);
    Float ds = s - si, dt = t - ti;
    return ((1 - ds) * (1 - dt) * GetChannel({si, ti}, c, wrapMode) +
            (1 - ds) * dt * GetChannel({si, ti + 1}, c, wrapMode) +
            ds * (1 - dt) * GetChannel({si + 1, ti}, c, wrapMode) +
            ds * dt * GetChannel({si + 1, ti + 1}, c, wrapMode));
}

Float Image::BilerpY(Point2f p, WrapMode wrapMode) const {
    if (nChannels() == 1) return BilerpChannel(p, 0, wrapMode);
    CHECK_EQ(3, nChannels());
    return (BilerpChannel(p, 0, wrapMode) + BilerpChannel(p, 1, wrapMode) +
            BilerpChannel(p, 2, wrapMode)) /
           3;
}

Spectrum Image::BilerpSpectrum(Point2f p, SpectrumType spectrumType,
                               WrapMode wrapMode) const {
    if (nChannels() == 1) return Spectrum(BilerpChannel(p, 0, wrapMode));
    std::array<Float, 3> rgb = {BilerpChannel(p, 0, wrapMode),
                                BilerpChannel(p, 1, wrapMode),
                                BilerpChannel(p, 2, wrapMode)};
    return Spectrum::FromRGB(&rgb[0], spectrumType);
}

void Image::SetChannel(Point2i p, int c, Float value) {
    CHECK(!std::isnan(value));
    if (format == PixelFormat::SRGB8 || format == PixelFormat::SY8)
        // TODO: use a LUT for sRGB 8 bit
        value = GammaCorrect(value);

    switch (format) {
    case PixelFormat::SY8:
    case PixelFormat::SRGB8:
    case PixelFormat::Y8:
    case PixelFormat::RGB8:
        value = Clamp((value * 255.f) + 0.5f, 0, 255);
        p8[PixelOffset(p, c)] = uint8_t(value);
        break;
    case PixelFormat::Y16:
    case PixelFormat::RGB16:
        p16[PixelOffset(p, c)] = FloatToHalf(value);
        break;
    case PixelFormat::Y32:
    case PixelFormat::RGB32:
        p32[PixelOffset(p, c)] = value;
        break;
    default:
        LOG(FATAL) << "Unhandled PixelFormat in Image::SetChannel()";
    }
}

void Image::SetY(Point2i p, Float value) {
    for (int c = 0; c < nChannels(); ++c) SetChannel(p, c, value);
}

void Image::SetSpectrum(Point2i p, const Spectrum &s) {
    if (nChannels() == 1)
        SetChannel(p, 0, s.Average());
    else {
        CHECK_EQ(3, nChannels());
        Float rgb[3];
        s.ToRGB(rgb);
        for (int c = 0; c < 3; ++c) SetChannel(p, c, rgb[c]);
    }
}

struct ResampleWeight {
    int firstTexel;
    Float weight[4];
};

static std::unique_ptr<ResampleWeight[]> resampleWeights(int oldRes,
                                                         int newRes) {
    CHECK_GE(newRes, oldRes);
    std::unique_ptr<ResampleWeight[]> wt(new ResampleWeight[newRes]);
    Float filterwidth = 2.f;
    for (int i = 0; i < newRes; ++i) {
        // Compute image resampling weights for _i_th texel
        Float center = (i + .5f) * oldRes / newRes;
        wt[i].firstTexel = std::floor((center - filterwidth) + 0.5f);
        for (int j = 0; j < 4; ++j) {
            Float pos = wt[i].firstTexel + j + .5f;
            wt[i].weight[j] = Lanczos((pos - center) / filterwidth);
        }

        // Normalize filter weights for texel resampling
        Float invSumWts = 1 / (wt[i].weight[0] + wt[i].weight[1] +
                               wt[i].weight[2] + wt[i].weight[3]);
        for (int j = 0; j < 4; ++j) wt[i].weight[j] *= invSumWts;
    }
    return wt;
}

void Image::Resize(Point2i newResolution, WrapMode wrapMode) {
    CHECK_GE(newResolution.x, resolution.x);
    CHECK_GE(newResolution.y, resolution.y);

    // Resample image in $s$ direction
    std::unique_ptr<ResampleWeight[]> sWeights =
        resampleWeights(resolution[0], newResolution[0]);
    const int nc = nChannels();
    CHECK(nc == 1 || nc == 3);
    Image resampledImage(nc == 1 ? PixelFormat::Y32 : PixelFormat::RGB32,
                         newResolution);

    // Apply _sWeights_ to zoom in $s$ direction
    ParallelFor(
        [&](int t) {
            for (int s = 0; s < newResolution[0]; ++s) {
                // Compute texel $(s,t)$ in $s$-zoomed image
                for (int c = 0; c < nc; ++c) {
                    Float value = 0;
                    for (int j = 0; j < 4; ++j) {
                        int origS = sWeights[s].firstTexel + j;
                        value += sWeights[s].weight[j] *
                                 GetChannel({origS, t}, c, wrapMode);
                    }
                    resampledImage.SetChannel({s, t}, c, value);
                }
            }
        },
        resolution[1], 16);

    // Resample image in $t$ direction
    std::unique_ptr<ResampleWeight[]> tWeights =
        resampleWeights(resolution[1], newResolution[1]);
    std::vector<Float *> resampleBufs;
    int nThreads = MaxThreadIndex();
    for (int i = 0; i < nThreads; ++i)
        resampleBufs.push_back(new Float[nc * newResolution[1]]);
    ParallelFor(
        [&](int s) {
            Float *workData = resampleBufs[ThreadIndex];
            memset(workData, 0, sizeof(Float) * nc * newResolution[1]);

            for (int t = 0; t < newResolution[1]; ++t) {
                for (int j = 0; j < 4; ++j) {
                    int tSrc = tWeights[t].firstTexel + j;
                    for (int c = 0; c < nc; ++c)
                        workData[t * nc + c] +=
                            tWeights[t].weight[j] *
                            resampledImage.GetChannel({s, tSrc}, c);
                }
            }
            for (int t = 0; t < newResolution[1]; ++t)
                for (int c = 0; c < nc; ++c) {
                    Float v = Clamp(workData[nc * t + c], 0, Infinity);
                    resampledImage.SetChannel({s, t}, c, v);
                }
        },
        newResolution[0], 32);

    resolution = newResolution;
    if (Is8Bit(format))
        p8.resize(nc * newResolution[0] * newResolution[1]);
    else if (Is16Bit(format))
        p16.resize(nc * newResolution[0] * newResolution[1]);
    else if (Is32Bit(format))
        p32.resize(nc * newResolution[0] * newResolution[1]);
    else
        LOG(FATAL) << "unexpected PixelFormat";

    for (int t = 0; t < resolution[1]; ++t)
        for (int s = 0; s < resolution[0]; ++s)
            for (int c = 0; c < nc; ++c)
                SetChannel(Point2i(s, t), c,
                           resampledImage.GetChannel({s, t}, c));
}

void Image::FlipY() {
    const int nc = nChannels();
    for (int y = 0; y < resolution.y / 2; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            size_t o1 = PixelOffset({x, y}),
                   o2 = PixelOffset({x, resolution.y - 1 - y});
            for (int c = 0; c < nc; ++c) {
                if (Is8Bit(format))
                    std::swap(p8[o1 + c], p8[o2 + c]);
                else if (Is16Bit(format))
                    std::swap(p16[o1 + c], p16[o2 + c]);
                else if (Is32Bit(format))
                    std::swap(p32[o1 + c], p32[o2 + c]);
                else
                    LOG(FATAL) << "unexpected format";
            }
        }
    }
}

std::vector<Image> Image::GenerateMIPMap(WrapMode wrapMode) const {
    // Make a copy for level 0.
    Image image = *this;

    if (!IsPowerOf2(resolution[0]) || !IsPowerOf2(resolution[1])) {
        // Resample image to power-of-two resolution
        image.Resize({RoundUpPow2(resolution[0]), RoundUpPow2(resolution[1])},
                     wrapMode);
    }

    // Initialize levels of MIPMap from image
    int nLevels =
        1 + Log2Int(std::max(image.resolution[0], image.resolution[1]));
    std::vector<Image> pyramid(nLevels);

    // Initialize most detailed level of MIPMap
    pyramid[0] = std::move(image);

    Point2i levelResolution = pyramid[0].resolution;
    const int nc = nChannels();
    for (int i = 1; i < nLevels; ++i) {
        // Initialize $i$th MIPMap level from $i-1$st level
        levelResolution[0] = std::max(1, levelResolution[0] / 2);
        levelResolution[1] = std::max(1, levelResolution[1] / 2);
        pyramid[i] = Image(pyramid[0].format, levelResolution);

        // Filter four texels from finer level of pyramid
        ParallelFor(
            [&](int t) {
                for (int s = 0; s < levelResolution[0]; ++s) {
                    for (int c = 0; c < nc; ++c) {
                        Float texel =
                            .25f *
                            (pyramid[i - 1].GetChannel(Point2i(2 * s, 2 * t), c,
                                                       wrapMode) +
                             pyramid[i - 1].GetChannel(
                                 Point2i(2 * s + 1, 2 * t), c, wrapMode) +
                             pyramid[i - 1].GetChannel(
                                 Point2i(2 * s, 2 * t + 1), c, wrapMode) +
                             pyramid[i - 1].GetChannel(
                                 Point2i(2 * s + 1, 2 * t + 1), c, wrapMode));
                        pyramid[i].SetChannel(Point2i(s, t), c, texel);
                    }
                }
            },
            levelResolution[1], 16);
    }
    return pyramid;
}

}  // namespace pbrt
