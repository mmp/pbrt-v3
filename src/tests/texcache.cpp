
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

// tests/texcache.cpp*
#include "tests/gtest/gtest.h"

#include <set>
#include "half.h"
#include "image.h"
#include "mipmap.h"
#include "parallel.h"
#include "pbrt.h"
#include "rng.h"
#include "texcache.h"

using namespace pbrt;

// Returns an image of the given format and resolution with random pixel
// values.
static Image GetImage(PixelFormat format, Point2i res) {
    RNG rng;

    auto v = [format, &rng]() -> Float {
        if (Is8Bit(format))
            return rng.UniformFloat();
        else if (Is16Bit(format))
            return Lerp(rng.UniformFloat(), -100., 100.);
        else {
            EXPECT_TRUE(Is32Bit(format));
            return Lerp(rng.UniformFloat(), -1e6f, 1e6f);
        }
    };

    Image image(format, res);
    for (int y = 0; y < res.y; ++y)
        for (int x = 0; x < res.x; ++x)
            for (int c = 0; c < image.nChannels(); ++c) {
                Float val = v();
                image.SetChannel({x, y}, c, val);
            }
    return image;
}

// Create a tiled texture with the given pixel format and tile size;
// then, add it to a texture cache and verify that all texels in all
// MIP levels exactly match the values in the original MIP chain.
static void TestFormat(PixelFormat format) {
    ParallelInit();

    std::unique_ptr<TextureCache> cache(new TextureCache);
    Image image = GetImage(format, {129, 60});

    char filename[64];
    sprintf(filename, "tx_fmt-%d", int(format));
    WrapMode wrapMode = WrapMode::Clamp;
    std::vector<Image> mips = image.GenerateMIPMap(wrapMode);
    int tileSize = TextureCache::TileSize(format);
    ASSERT_TRUE(TiledImagePyramid::Create(mips, filename, wrapMode, tileSize));

    int id = cache->AddTexture(filename);
    EXPECT_GE(id, 0);

    for (size_t level = 0; level < mips.size(); ++level) {
        EXPECT_EQ(cache->GetPixelFormat(id), format);
        EXPECT_EQ(cache->GetPixelFormat(id), mips[level].format);
        ASSERT_EQ(cache->GetLevelResolution(id, level), mips[level].resolution);

        for (int y = 0; y < mips[level].resolution.y; ++y)
            for (int x = 0; x < mips[level].resolution.x; ++x) {
                if (nChannels(format) == 1) {
                    Float val = cache->Texel<Float>(id, level, {x, y});
                    EXPECT_EQ(mips[level].GetChannel({x, y}, 0), val);
                } else {
                    ASSERT_EQ(3, nChannels(format));
                    Spectrum s = cache->Texel<Spectrum>(id, level, {x, y});
                    for (int c = 0; c < 3; ++c)
                        EXPECT_EQ(mips[level].GetChannel({x, y}, c), s[c]);
                }
            }
    }
    EXPECT_EQ(0, remove(filename));
    ParallelCleanup();
}

TEST(Texcache, SY8) { TestFormat(PixelFormat::SY8); }

TEST(Texcache, Y8) { TestFormat(PixelFormat::Y8); }

TEST(Texcache, Y16) { TestFormat(PixelFormat::Y16); }

TEST(Texcache, Y32) { TestFormat(PixelFormat::Y32); }

TEST(Texcache, SRGB8) { TestFormat(PixelFormat::SRGB8); }

TEST(Texcache, RGB8) { TestFormat(PixelFormat::RGB8); }

TEST(Texcache, RGB16) { TestFormat(PixelFormat::RGB16); }

TEST(Texcache, RGB32) { TestFormat(PixelFormat::RGB32); }

TEST(Texcache, ThreadInsanity) {
    ParallelInit();

    // Create a bunch of images with random sizes and contents.
    std::vector<std::vector<Image>> images;
    std::vector<std::string> filenames;
    RNG rng;
    for (int i = 0; i < 100; ++i) {
        Point2i res(2 + rng.UniformUInt32(1000), 2 + rng.UniformUInt32(1000));
        Image im = GetImage(PixelFormat::SRGB8, res);
        // Just do one level; save the time of creating MIP levels...
        std::vector<Image> mips{im};
        images.push_back(mips);
        std::string filename = StringPrintf("img-%d-%d.txp", res.x, res.y);
        filenames.push_back(filename);
        int tileSize = TextureCache::TileSize(im.format);
        ASSERT_TRUE(TiledImagePyramid::Create(mips, filename, WrapMode::Clamp,
                                              tileSize));
    }

    PbrtOptions.texCacheMB = 32;
    std::unique_ptr<TextureCache> cache(new TextureCache);
    ASSERT_TRUE(cache.get() != nullptr);

    // Supply textures to texture cache.
    std::vector<int> ids;
    for (const auto &fn : filenames) ids.push_back(cache->AddTexture(fn));

    // Have a bunch of threads hammer on it in parallel.
    ParallelFor(
        [&](int64_t chunk) {
            RNG rng(chunk);
            for (int i = 0; i < 10000; ++i) {
                // Choose a random texture and level.
                int texIndex = rng.UniformUInt32(ids.size());
                int texId = ids[texIndex];
                int level = rng.UniformUInt32(images[texIndex].size());
                Point2i res = cache->GetLevelResolution(texId, level);

                // Choose a random point in the texture.
                Point2i p(rng.UniformUInt32(res[0]), rng.UniformUInt32(res[1]));

                Spectrum v = cache->Texel<Spectrum>(texId, level, p);
                EXPECT_EQ(v, images[texIndex][level].GetSpectrum(p));
            }
        },
        1000);

    for (const auto &fn : filenames) EXPECT_EQ(0, remove(fn.c_str()));

    ParallelCleanup();
}
