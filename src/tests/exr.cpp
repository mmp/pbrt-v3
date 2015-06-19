
#include "tests/gtest/gtest.h"
#include <cmath>

#include "pbrt.h"
#include "rng.h"
#include "ext/tinyexr.h"

union FP32 {
    uint u;
    float f;
    struct {
        uint Mantissa : 23;
        uint Exponent : 8;
        uint Sign : 1;
    };
};

union FP16 {
    unsigned short u;
    struct {
        uint Mantissa : 10;
        uint Exponent : 5;
        uint Sign : 1;
    };
};


static FP32 half_to_float_full(FP16 h)
{
  FP32 o = { 0 };

  // From ISPC ref code
  if (h.Exponent == 0 && h.Mantissa == 0) // (Signed) zero
    o.Sign = h.Sign;
  else {
    if (h.Exponent == 0) { // Denormal (will convert to normalized)
      // Adjust mantissa so it's normalized (and keep track of exp adjust)
      int e = -1;
      uint m = h.Mantissa;
      do {
        e++;
        m <<= 1;
      } while ((m & 0x400) == 0);

      o.Mantissa = (m & 0x3ff) << 13;
      o.Exponent = 127 - 15 - e;
      o.Sign = h.Sign;
    }
    else if (h.Exponent == 0x1f) { // Inf/NaN
      // NOTE: It's safe to treat both with the same code path by just
      // truncating lower Mantissa bits in NaNs (this is valid).
      o.Mantissa = h.Mantissa << 13;
      o.Exponent = 255;
      o.Sign = h.Sign;
    }
    else { // Normalized number
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

  int fint2 = FloatToBits(BitsToFloat(fint & round_mask) * BitsToFloat(magic)) -
      round_mask;
  // Clamp to signed infinity if overflowed
  fint2 = (fint2 > f16infty) ? f16infty : fint2;

  if (fint < f32infty)
    o = fint2 >> 13; // Take the bits!

  return (o | (sign >> 16));
}

static void CompareImages(const EXRImage &a, const EXRImage &b,
                          bool halfQuantize) {
   EXPECT_EQ(a.num_channels, b.num_channels);
   EXPECT_EQ(a.width, b.width);
   EXPECT_EQ(a.height, b.height);
   for (int i = 0; i < a.num_channels; ++i) {
     EXPECT_EQ(a.pixel_types[i], b.pixel_types[i]);
     EXPECT_EQ(std::string(a.channel_names[i]),
               std::string(b.channel_names[i]));
   }
   for (int i = 0; i < a.width * a.height; ++i) {
     for (int c = 0; c < a.num_channels; ++c) {
       float ap = ((float *)a.images[c])[i];
       float bp = ((float *)b.images[c])[i];
       if (std::isnan(ap) && std::isnan(bp))
         continue;
       if (halfQuantize) {
         int ha = float_to_half(ap);
         int hb = float_to_half(bp);
         EXPECT_EQ(ha, hb) <<  "offset " << i << ", channel " << c <<
             ", fa " << ap << ", fb " << bp;
       }
       else {
         EXPECT_EQ(ap, bp) << "offset " << i << ", channel " << c;
       }
     }
   }
}

TEST(EXR, BasicRoundTrip) {
   int width = 289;
   int height = 1 + 65536 / width;

   float *buf = new float[width * height];
   for (int i = 0; i < 65536; ++i) {
     FP16 half;
     half.u = i;
     buf[i] = half_to_float_full(half).f;
   }
   for (int i = 65536; i < width * height; ++i)
     buf[i] = 0;

   EXRImage image;
   image.num_channels = 1;
   const char *channels[] = { "R" };
   image.channel_names = channels;
   unsigned char *images[] = { (unsigned char *)buf };
   image.images = images;
   int pixel_types[] = { TINYEXR_PIXELTYPE_HALF };
   image.pixel_types = pixel_types;
   image.width = width;
   image.height = height;

   const char *err = nullptr;
   EXPECT_EQ(0, SaveMultiChannelEXRToFile(&image, "test.exr", &err)) << err;

   EXRImage readImage;
   EXPECT_EQ(0, LoadMultiChannelEXRFromFile(&readImage, "test.exr", &err))
       << err;

   CompareImages(image, readImage, false);
}

TEST(EXR, Randoms) {
   int width = 1024;
   int height = 1024;

   RNG rng;
   float *buf = new float[4 * width * height];
   for (int i = 0; i < 4 * width * height; ++i) {
     buf[i] = -20 + 20. * rng.UniformFloat();
   }

   EXRImage image;
   image.num_channels = 4;
   const char *channels[] = { "B", "G", "R", "A" };
   image.channel_names = channels;
   unsigned char *images[] = { (unsigned char *)buf,
                               (unsigned char *)(buf + width * height),
                               (unsigned char *)(buf + 2 * width * height),
                               (unsigned char *)(buf + 3 * width * height) };
   image.images = images;
   int pixel_types[] = { TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF,
                         TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF };
   image.pixel_types = pixel_types;
   image.width = width;
   image.height = height;

   const char *err = nullptr;
   EXPECT_EQ(0, SaveMultiChannelEXRToFile(&image, "test.exr", &err)) << err;

   EXRImage readImage;
   EXPECT_EQ(0, LoadMultiChannelEXRFromFile(&readImage, "test.exr", &err))
       << err;

   CompareImages(image, readImage, true);
}
