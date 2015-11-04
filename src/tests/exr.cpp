
#include "tests/gtest/gtest.h"
#include <cmath>
#include <stdint.h>

#include "pbrt.h"
#include "rng.h"
#include "ext/tinyexr.h"

#if defined(__LITTLE_ENDIAN__) || defined(__i386__) || defined(__x86_64__) || \
    defined(WIN32)
#define IS_BIG_ENDIAN (0)
#elif defined(__BIG_ENDIAN__)
#define IS_BIG_ENDIAN (1)
#elif defined(__sparc) || defined(__sparc__)
#define IS_BIG_ENDIAN (1)
#else
#error "Can't detect machine endian-ness at compile-time."
#endif

union FP32 {
    uint32_t u;
    float f;
    struct {
#if !IS_BIG_ENDIAN
        unsigned int Mantissa : 23;
        unsigned int Exponent : 8;
        unsigned int Sign : 1;
#else
        unsigned int Sign : 1;
        unsigned int Exponent : 8;
        unsigned int Mantissa : 23;
#endif
    };
};

union FP16 {
    unsigned short u;
    struct {
#if !IS_BIG_ENDIAN
        unsigned int Mantissa : 10;
        unsigned int Exponent : 5;
        unsigned int Sign : 1;
#else
        unsigned int Sign : 1;
        unsigned int Exponent : 5;
        unsigned int Mantissa : 10;
#endif
    };
};

static FP32 half_to_float_full(FP16 h) {
    FP32 o = {0};

    // From ISPC ref code
    if (h.Exponent == 0 && h.Mantissa == 0)  // (Signed) zero
        o.Sign = h.Sign;
    else {
        if (h.Exponent == 0) {  // Denormal (will convert to normalized)
            // Adjust mantissa so it's normalized (and keep track of exp adjust)
            int e = -1;
            unsigned int m = h.Mantissa;
            do {
                e++;
                m <<= 1;
            } while ((m & 0x400) == 0);

            o.Mantissa = (m & 0x3ff) << 13;
            o.Exponent = 127 - 15 - e;
            o.Sign = h.Sign;
        } else if (h.Exponent == 0x1f) {  // Inf/NaN
            // NOTE: It's safe to treat both with the same code path by just
            // truncating lower Mantissa bits in NaNs (this is valid).
            o.Mantissa = h.Mantissa << 13;
            o.Exponent = 255;
            o.Sign = h.Sign;
        } else {  // Normalized number
            o.Mantissa = h.Mantissa << 13;
            o.Exponent = 127 - 15 + h.Exponent;
            o.Sign = h.Sign;
        }
    }

    return o;
}

int float_to_half(float f) {
    unsigned int sign_mask = 0x80000000u;
    int o;

    int fint = FloatToBits(f);
    int sign = fint & sign_mask;
    fint ^= sign;

    // NOTE all the integer compares in this function can be safely
    // compiled into signed compares since all operands are below
    // 0x80000000. Important if you want fast straight SSE2 code (since
    // there's no unsigned PCMPGTD).

    // Inf or NaN (all exponent bits set)
    // NaN->qNaN and Inf->Inf
    // unconditional assignment here, will override with right value for
    // the regular case below.
    int f32infty = 255ul << 23;
    o = (fint > f32infty) ? 0x7e00u : 0x7c00u;

    // (De)normalized number or zero
    // update fint unconditionally to save the blending; we don't need it
    // anymore for the Inf/NaN case anyway.
    const unsigned int round_mask = ~0xfffu;
    const uint32_t magic = 15ul << 23;
    const int f16infty = 31ul << 23;

    int fint2 =
        FloatToBits(BitsToFloat(fint & round_mask) * BitsToFloat(magic)) -
        round_mask;
    // Clamp to signed infinity if overflowed
    fint2 = (fint2 > f16infty) ? f16infty : fint2;

    if (fint < f32infty) o = fint2 >> 13;  // Take the bits!

    return (o | (sign >> 16));
}

TEST(EXR, BasicRoundTrip) {
    int width = 289;
    int height = 1 + 65536 / width;

    // Make sure that all of the exact half values come through unchanged
    // after a write and a read.
    float *buf = new float[width * height];
    for (int i = 0; i < 65536; ++i) {
        FP16 half;
        half.u = i;
        buf[i] = half_to_float_full(half).f;
    }
    for (int i = 65536; i < width * height; ++i) buf[i] = 0;

    EXRImage image = {0};
    image.num_channels = 1;
    const char *channels[] = {"R"};
    image.channel_names = channels;
    unsigned char *images[] = {(unsigned char *)buf};
    image.images = images;
    int pixel_types[] = {TINYEXR_PIXELTYPE_FLOAT};
    image.pixel_types = pixel_types;
    int requested_pixel_types[] = {TINYEXR_PIXELTYPE_HALF};
    image.requested_pixel_types = requested_pixel_types;
    image.width = width;
    image.height = height;

    const char *err = nullptr;
    EXPECT_EQ(0, SaveMultiChannelEXRToFile(&image, "test.exr", &err)) << err;

    EXRImage readImage;
    readImage.requested_pixel_types = pixel_types;
    EXPECT_EQ(0, LoadMultiChannelEXRFromFile(&readImage, "test.exr", &err))
        << err;

    EXPECT_EQ(1, readImage.num_channels);
    EXPECT_EQ(width, readImage.width);
    EXPECT_EQ(height, readImage.height);
    EXPECT_EQ(TINYEXR_PIXELTYPE_FLOAT, readImage.pixel_types[0]);
    for (int i = 0; i < width * height; ++i)
        if (!isnan(buf[i]))
            EXPECT_EQ(buf[i], ((float *)readImage.images[0])[i]);
}

TEST(EXR, RandomHalf) {
    for (int iter = 0; iter < 32; ++iter) {
        RNG rng(iter);

        // Choose some random basic properties of the image.
        int width = 1 + rng.UniformUInt32(512);
        int height = 1 + rng.UniformUInt32(512);
        int nComps = 1 + rng.UniformUInt32(3);
        if (nComps == 0) ++nComps;
        bool asFloat = rng.UniformFloat() < 0.5;

        // Generate a buffer of random half values
        uint16_t *buf = new uint16_t[nComps * width * height];
        for (int i = 0; i < nComps * width * height; ++i)
            buf[i] = rng.UniformUInt32(65536);

        EXRImage image;
        InitEXRImage(&image);
        image.num_channels = nComps;
        // Random image compression method.
        switch (rng.UniformUInt32(3)) {
        case 0:
            image.compression = TINYEXR_COMPRESSIONTYPE_NONE;
            break;
        case 1:
            image.compression = TINYEXR_COMPRESSIONTYPE_ZIPS;
            break;
        case 2:
            image.compression = TINYEXR_COMPRESSIONTYPE_ZIP;
            break;
        }

        const char *channels[] = {"B", "G", "R", "A"};
        image.channel_names = channels;
        unsigned char *images[] = {(unsigned char *)buf,
                                   (unsigned char *)(buf + width * height),
                                   (unsigned char *)(buf + 2 * width * height),
                                   (unsigned char *)(buf + 3 * width * height)};
        image.images = images;
        int halfTypes[] = {TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF,
                           TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF};
        int floatTypes[] = {TINYEXR_PIXELTYPE_FLOAT, TINYEXR_PIXELTYPE_FLOAT,
                            TINYEXR_PIXELTYPE_FLOAT, TINYEXR_PIXELTYPE_FLOAT};
        image.pixel_types = halfTypes;
        // Write them out as either half or float values.
        image.requested_pixel_types = asFloat ? floatTypes : halfTypes;
        image.width = width;
        image.height = height;

        const char *err = nullptr;
        EXPECT_EQ(0, SaveMultiChannelEXRToFile(&image, "test.exr", &err))
            << err;

        EXRImage readImage;
        InitEXRImage(&readImage);
        // Unfortunately tinyexr hits an assert if we try to read a float
        // EXR and return half values.
        readImage.requested_pixel_types = asFloat ? floatTypes : halfTypes;
        EXPECT_EQ(0, LoadMultiChannelEXRFromFile(&readImage, "test.exr", &err))
            << err;

        // Make sure image basics are right.
        EXPECT_EQ(nComps, readImage.num_channels);
        EXPECT_EQ(width, readImage.width);
        EXPECT_EQ(height, readImage.height);
        for (int i = 0; i < nComps; ++i)
            EXPECT_EQ(
                asFloat ? TINYEXR_PIXELTYPE_FLOAT : TINYEXR_PIXELTYPE_HALF,
                readImage.pixel_types[i]);

        // Now make sure all pixel values match exactly.
        for (int c = 0; c < nComps; ++c)
            for (int i = 0; i < width * height; ++i)
                if (asFloat) {
                    FP16 half;
                    half.u = buf[c * width * height + i];
                    if (!std::isnan(half_to_float_full(half).f))
                        EXPECT_EQ(half_to_float_full(half).f,
                                  ((float *)readImage.images[c])[i]);
                } else
                    EXPECT_EQ(buf[c * width * height + i],
                              ((uint16_t *)readImage.images[c])[i]);

        delete[] buf;
    }
}

// Similar to RandomHalf, except generate random float values. These should
// come back exactly the same if a float on-disk format is used, and should
// be the closest half to the float otherwise.

TEST(EXR, RandomFloat) {
    for (int iter = 0; iter < 32; ++iter) {
        RNG rng(iter);

        // Choose some random basic properties of the image.
        int width = 1 + rng.UniformUInt32(512);
        int height = 1 + rng.UniformUInt32(512);
        int nComps = 1 + rng.UniformUInt32(3);
        if (nComps == 0) ++nComps;
        bool asFloat = rng.UniformFloat() < 0.5;

        // Generate a buffer of random float values
        float *buf = new float[nComps * width * height];
        for (int i = 0; i < nComps * width * height; ++i) {
          int exp = 8;
          Float logu = Lerp(rng.UniformFloat(), -exp, exp);
          buf[i] = std::pow(10, logu);
          if (rng.UniformFloat() < .5) buf[i] = -buf[i];
        }

        EXRImage image;
        InitEXRImage(&image);
        image.num_channels = nComps;
        // Random image compression method.
        switch (rng.UniformUInt32(3)) {
        case 0:
            image.compression = TINYEXR_COMPRESSIONTYPE_NONE;
            break;
        case 1:
            image.compression = TINYEXR_COMPRESSIONTYPE_ZIPS;
            break;
        case 2:
            image.compression = TINYEXR_COMPRESSIONTYPE_ZIP;
            break;
        }

        const char *channels[] = {"B", "G", "R", "A"};
        image.channel_names = channels;
        unsigned char *images[] = {(unsigned char *)buf,
                                   (unsigned char *)(buf + width * height),
                                   (unsigned char *)(buf + 2 * width * height),
                                   (unsigned char *)(buf + 3 * width * height)};
        image.images = images;
        int halfTypes[] = {TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF,
                           TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF};
        int floatTypes[] = {TINYEXR_PIXELTYPE_FLOAT, TINYEXR_PIXELTYPE_FLOAT,
                            TINYEXR_PIXELTYPE_FLOAT, TINYEXR_PIXELTYPE_FLOAT};
        image.pixel_types = floatTypes;
        // Write them out as either half or float values.
        image.requested_pixel_types = asFloat ? floatTypes : halfTypes;
        image.width = width;
        image.height = height;

        const char *err = nullptr;
        EXPECT_EQ(0, SaveMultiChannelEXRToFile(&image, "test.exr", &err))
            << err;

        EXRImage readImage;
        InitEXRImage(&readImage);

        readImage.requested_pixel_types = asFloat ? floatTypes : halfTypes;
        EXPECT_EQ(0, LoadMultiChannelEXRFromFile(&readImage, "test.exr", &err))
            << err;

        // Make sure image basics are right.
        EXPECT_EQ(nComps, readImage.num_channels);
        EXPECT_EQ(width, readImage.width);
        EXPECT_EQ(height, readImage.height);
        for (int i = 0; i < nComps; ++i)
            EXPECT_EQ(
                asFloat ? TINYEXR_PIXELTYPE_FLOAT : TINYEXR_PIXELTYPE_HALF,
                readImage.pixel_types[i]);

        // Now make sure all pixel values match.
        for (int c = 0; c < nComps; ++c) {
            for (int i = 0; i < width * height; ++i) {
                if (std::isnan(buf[c * width * height + i])) continue;
                if (asFloat)
                    EXPECT_EQ(buf[c * width * height + i],
                              ((float *)readImage.images[c])[i]);
                else {
                    int h = float_to_half(buf[c * width * height + i]);
                    EXPECT_EQ(h, ((int16_t *)readImage.images[c])[i]);
                }
            }
        }

        delete[] buf;
    }
}
