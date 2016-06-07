
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "fileutil.h"
#include "spectrum.h"
#include "imageio.h"

static void TestRoundTrip(const char *fn, bool gamma, bool flipy) {
    Point2i res(16, 29);
    std::vector<Float> pixels(3 * res[0] * res[1]);
    for (int y = 0; y < res[1]; ++y)
        for (int x = 0; x < res[0]; ++x) {
            int offset = 3 * (y * res[0] + x);
            pixels[offset] = Float(x) / Float(res[0] - 1);
            pixels[offset + 1] = Float(y) / Float(res[1] - 1);
            pixels[offset + 2] = -1.5f;
        }

    WriteImage(fn, &pixels[0], Bounds2i({0, 0}, res), res);

    Point2i readRes;
    auto readPixels = ReadImage(fn, &readRes);
    ASSERT_TRUE(readPixels.get() != nullptr);
    EXPECT_EQ(readRes, res);

    for (int y = 0; y < res[1]; ++y)
        for (int x = 0; x < res[0]; ++x) {
            Float rgb[3];
            if (flipy)
                // flip in y :-(
                readPixels[(res[1] - 1 - y) * res[0] + x].ToRGB(rgb);
            else
                readPixels[y * res[0] + x].ToRGB(rgb);

            for (int c = 0; c < 3; ++c) {
                if (gamma)
                    rgb[c] = InverseGammaCorrect(rgb[c]);

                float wrote = pixels[3 * (y * res[0] + x) + c];
                float delta = wrote - rgb[c];
                if (HasExtension(fn, "pfm")) {
                    // Everything should come out exact.
                    EXPECT_EQ(0, delta) << fn << ":(" << x << ", " << y
                                        << ") c = " << c << " wrote " << wrote
                                        << ", read " << rgb[c]
                                        << ", delta = " << delta;
                } else if (HasExtension(fn, "exr")) {
                    if (c == 2)
                        // -1.5 is exactly representable as a float.
                        EXPECT_EQ(0, delta) << "(" << x << ", " << y
                                            << ") c = " << c << " wrote "
                                            << wrote << ", read " << rgb[c]
                                            << ", delta = " << delta;
                    else
                        EXPECT_LT(std::abs(delta), .001)
                            << fn << ":(" << x << ", " << y << ") c = " << c
                            << " wrote " << wrote << ", read " << rgb[c]
                            << ", delta = " << delta;
                } else {
                    // 8 bit format...
                    if (c == 2)
                        // -1.5 should be clamped to zero.
                        EXPECT_EQ(0, rgb[c]) << "(" << x << ", " << y
                                             << ") c = " << c << " wrote "
                                             << wrote << ", read " << rgb[c]
                                             << " (expected 0 back)";
                    else
                        // Allow a fair amount of slop, since there's an sRGB
                        // conversion before quantization to 8-bits...
                        EXPECT_LT(std::abs(delta), .02)
                            << fn << ":(" << x << ", " << y << ") c = " << c
                            << " wrote " << wrote << ", read " << rgb[c]
                            << ", delta = " << delta;
                }
            }
        }
}

TEST(ImageIO, RoundTripEXR) { TestRoundTrip("out.exr", false, false); }

TEST(ImageIO, RoundTripPFM) { TestRoundTrip("out.pfm", false, true); }

TEST(ImageIO, RoundTripTGA) { TestRoundTrip("out.tga", true, true); }

TEST(ImageIO, RoundTripPNG) { TestRoundTrip("out.png", true, true); }
