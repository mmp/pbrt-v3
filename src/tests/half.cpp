
#include "tests/gtest/gtest.h"

#include "pbrt.h"
#include "core/fp16.h"
#include "rng.h"

using namespace pbrt;

TEST(Half, Basics) {
  EXPECT_EQ(FloatToHalf(0.f), kHalfPositiveZero);
  EXPECT_EQ(FloatToHalf(-0.f), kHalfNegativeZero);
  EXPECT_EQ(FloatToHalf(Infinity), kHalfPositiveInfinity);
  EXPECT_EQ(FloatToHalf(-Infinity), kHalfNegativeInfinity);

  EXPECT_TRUE(HalfIsNaN(FloatToHalf(0.f / 0.f)));
  EXPECT_TRUE(HalfIsNaN(FloatToHalf(-0.f / 0.f)));
  EXPECT_FALSE(HalfIsNaN(kHalfPositiveInfinity));
}

TEST(Half, ExactConversions) {
  // Test round-trip conversion of integers that are perfectly
  // representable.
  for (int i = -2048; i <= 2048; ++i) {
    EXPECT_EQ(i, HalfToFloat(FloatToHalf(i)));
  }

  // Similarly for some well-behaved floats
  float limit = 1024, delta = 0.5;
  for (int i = 0; i < 10; ++i) {
    for (float f = -limit; f <= limit; f += delta)
      EXPECT_EQ(f, HalfToFloat(FloatToHalf(f)));
    limit /= 2;
    delta /= 2;
  }
}

TEST(Half, Randoms) {
  RNG rng;
  // Choose a bunch of random positive floats and make sure that they
  // convert to reasonable values.
  for (int i = 0; i < 1024; ++i) {
    float f = rng.UniformFloat() * 512;
    uint16_t h = FloatToHalf(f);
    float fh = HalfToFloat(h);
    if (fh == f) {
      // Very unlikely, but we happened to pick a value exactly
      // representable as a half.
      continue;
    }
    else {
      // The other half value that brackets the float.
      uint16_t hother;
      if (fh > f) {
        // The closest half was a bit bigger; therefore, the half before it
        // s the other one.
        hother = h - 1;
        if (hother > h) {
          // test for wrapping around zero
          continue;
        }
      } else {
        hother = h + 1;
        if (hother < h) {
          // test for wrapping around zero
          continue;
        }
      }

      // Make sure the two half values bracket the float.
      float fother = HalfToFloat(hother);
      float dh = std::abs(fh - f);
      float dother = std::abs(fother - f);
      if (fh > f)
        EXPECT_LT(fother, f);
      else
        EXPECT_GT(fother, f);

      // Make sure rounding to the other one of them wouldn't have given a
      // closer half.
      EXPECT_LE(dh, dother);
    }
  }
}
