
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

// core/imageio.cpp*
#include "imageio.h"
#include "geometry.h"
#include "image.h"
#include "fileutil.h"
#include "spectrum.h"
#include "fp16.h"

#include "ext/lodepng.h"
#include "ext/targa.h"
#include <ImfRgba.h>
#include <ImfRgbaFile.h>

namespace pbrt {

// ImageIO Local Declarations
static bool ReadImageEXR(const std::string &name, Image *image,
                         Bounds2i *dataWindow, Bounds2i *displayWindow);
static bool ReadImageTGA(const std::string &name, bool gamma, Image *image);
static bool ReadImagePNG(const std::string &name, bool gamma, Image *image);
static bool ReadImagePFM(const std::string &filename, Image *image);

// ImageIO Function Definitions
bool Image::Read(const std::string &name, Image *image, bool gamma,
                 Bounds2i *dataWindow, Bounds2i *displayWindow) {
    if (HasExtension(name, ".exr"))
        return ReadImageEXR(name, image, dataWindow, displayWindow);

    LOG_IF(ERROR, dataWindow != nullptr || displayWindow != nullptr) <<
        "Data window and display window not available for non-EXR images.";

    if (HasExtension(name, ".tga"))
        return ReadImageTGA(name, gamma, image);
    else if (HasExtension(name, ".png"))
        return ReadImagePNG(name, gamma, image);
    else if (HasExtension(name, ".pfm"))
        return ReadImagePFM(name, image);
    else {
        Error("%s: no support for reading images with this extension",
              name.c_str());
        return false;
    }
}

bool Image::Write(const std::string &name) const {
    return Write(name, Bounds2i({0, 0}, resolution), resolution);
}

bool Image::Write(const std::string &name, const Bounds2i &pixelBounds,
                  Point2i fullResolution) const {
    if (HasExtension(name, ".exr"))
        return WriteEXR(name, pixelBounds, fullResolution);
    else if (HasExtension(name, ".pfm"))
        return WritePFM(name);
    else if (HasExtension(name, ".png"))
        return WritePNG(name);
    else if (HasExtension(name, ".tga"))
        return WriteTGA(name);
    else {
        Error("Can't determine image file type from suffix of filename \"%s\"",
              name.c_str());
        return false;
    }
}

static bool ReadImageEXR(const std::string &name, Image *image,
                         Bounds2i *dataWindow, Bounds2i *displayWindow) {
    using namespace Imf;
    using namespace Imath;
    try {
        // TODO: handle single channel EXRs directly...
        RgbaInputFile file(name.c_str());
        Box2i dw = file.dataWindow();

        // OpenEXR uses inclusive pixel bounds; adjust to non-inclusive
        // (the convention pbrt uses) in the values returned.
        if (dataWindow)
            *dataWindow = {{dw.min.x, dw.min.y}, {dw.max.x + 1, dw.max.y + 1}};
        if (displayWindow) {
            Box2i dispw = file.displayWindow();
            *displayWindow = {{dispw.min.x, dispw.min.y},
                              {dispw.max.x + 1, dispw.max.y + 1}};
        }

        int width = dw.max.x - dw.min.x + 1;
        int height = dw.max.y - dw.min.y + 1;
        std::vector<Rgba> pixels(width * height);
        file.setFrameBuffer(&pixels[0] - dw.min.x - dw.min.y * width, 1, width);
        file.readPixels(dw.min.y, dw.max.y);

        std::vector<uint16_t> rgb(3 * width * height);
        for (int i = 0; i < width * height; ++i) {
            memcpy(&rgb[3 * i], &pixels[i].r, sizeof(half));
            memcpy(&rgb[3 * i + 1], &pixels[i].g, sizeof(half));
            memcpy(&rgb[3 * i + 2], &pixels[i].b, sizeof(half));
        }
        LOG(INFO) << StringPrintf("Read EXR image %s (%d x %d)", name.c_str(),
                                  width, height);
        *image =
            Image(std::move(rgb), PixelFormat::RGB16, Point2i(width, height));
        return true;
    } catch (const std::exception &e) {
        Error("Unable to read image file \"%s\": %s", name.c_str(), e.what());
    }

    return false;
}

bool Image::WriteEXR(const std::string &name, const Bounds2i &pixelBounds,
                     Point2i fullResolution) const {
    using namespace Imf;
    using namespace Imath;

    // FIXME: if we have RGB32, it seems like we should write out float32;
    // require the caller to explicitly downcast/ask for float16 if
    // desired?

    std::unique_ptr<Rgba[]> hrgba(new Rgba[resolution.x * resolution.y]);
    for (int y = 0; y < resolution.y; ++y)
        for (int x = 0; x < resolution.x; ++x)
            // TODO: skip the half -> float -> half round trip...
            hrgba[y * resolution.x + x] =
                Rgba(GetChannel({x, y}, 0), GetChannel({x, y}, 1),
                     GetChannel({x, y}, 2));

    Box2i displayWindow(V2i(0, 0),
                        V2i(fullResolution.x - 1, fullResolution.y - 1));
    Box2i dataWindow(V2i(pixelBounds.pMin.x, pixelBounds.pMin.y),
                     V2i(pixelBounds.pMax.x - 1, pixelBounds.pMax.y - 1));

    try {
        RgbaOutputFile file(name.c_str(), displayWindow, dataWindow,
                            WRITE_RGBA);
        file.setFrameBuffer(hrgba.get() - pixelBounds.pMin.x -
                            pixelBounds.pMin.y * resolution.x,
                            1, resolution.x);
        file.writePixels(resolution.y);
    } catch (const std::exception &exc) {
        Error("Error writing \"%s\": %s", name.c_str(), exc.what());
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
// PNG Function Definitions

static inline uint8_t FloatToSRGB(Float v) {
    return uint8_t(Clamp(255.f * GammaCorrect(v) + 0.5f, 0.f, 255.f));
}

static bool ReadImagePNG(const std::string &name, bool gamma, Image *image) {
    unsigned char *rgb;
    unsigned width, height;
    unsigned int error =
        lodepng_decode24_file(&rgb, &width, &height, name.c_str());
    if (error != 0) {
        Error("Error reading PNG \"%s\": %s", name.c_str(),
              lodepng_error_text(error));
        return false;
    }

    std::vector<uint8_t> rgb8(3 * width * height);
    unsigned char *src = rgb;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x, src += 3) {
            for (int c = 0; c < 3; ++c)
                rgb8[3 * ((y * width) + x) + c] = src[c];
        }
    }

    PixelFormat format = gamma ? PixelFormat::SRGB8 : PixelFormat::RGB8;
    *image = Image(std::move(rgb8), format, Point2i(width, height));
    free(rgb);
    return true;
}

bool Image::WritePNG(const std::string &name) const {
    unsigned int error = 0;
    switch (format) {
    case PixelFormat::SRGB8:
        error = lodepng_encode24_file(name.c_str(), &p8[0], resolution.x,
                                      resolution.y);
        break;
    case PixelFormat::SY8:
        error = lodepng_encode_file(name.c_str(), &p8[0], resolution.x,
                                    resolution.y, LCT_GREY, 8 /* bitdepth */);
        break;
    case PixelFormat::RGB8:
    case PixelFormat::RGB16:
    case PixelFormat::RGB32: {
        std::unique_ptr<uint8_t[]> rgb8(
            new uint8_t[3 * resolution.x * resolution.y]);
        for (int y = 0; y < resolution.y; ++y)
            for (int x = 0; x < resolution.x; ++x)
                for (int c = 0; c < 3; ++c)
                    rgb8[3 * (y * resolution.x + x) + c] =
                        FloatToSRGB(GetChannel({x, y}, c));

        error = lodepng_encode24_file(name.c_str(), rgb8.get(), resolution.x,
                                      resolution.y);
        break;
    }
    case PixelFormat::Y8:
    case PixelFormat::Y16:
    case PixelFormat::Y32: {
        std::unique_ptr<uint8_t[]> y8(new uint8_t[resolution.x * resolution.y]);
        for (int y = 0; y < resolution.y; ++y)
            for (int x = 0; x < resolution.x; ++x)
                y8[y * resolution.x + x] = FloatToSRGB(GetChannel({x, y}, 0));

        error = lodepng_encode_file(name.c_str(), y8.get(), resolution.x,
                                    resolution.y, LCT_GREY, 8 /* bitdepth */);
        break;
    }
    }

    if (error != 0) {
        Error("Error writing PNG \"%s\": %s", name.c_str(),
              lodepng_error_text(error));
        return false;
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////
// TGA Function Definitions

bool Image::WriteTGA(const std::string &name) const {
    int nc = nChannels();
    std::unique_ptr<uint8_t[]> outBuf(
        new uint8_t[nc * resolution.x * resolution.y]);
    uint8_t *dst = outBuf.get();
    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            switch (format) {
            case PixelFormat::SRGB8:
                // Reformat to 8-bit BGR layout.
                dst[0] = p8[3 * (y * resolution.x + x) + 2];
                dst[1] = p8[3 * (y * resolution.x + x) + 1];
                dst[2] = p8[3 * (y * resolution.x + x)];
                dst += 3;
                break;
            case PixelFormat::SY8:
                *dst++ = p8[x * resolution.x + x];
                break;
            case PixelFormat::Y8:
            case PixelFormat::Y16:
            case PixelFormat::Y32:
                *dst++ = FloatToSRGB(GetChannel({x, y}, 0));
                break;
            case PixelFormat::RGB8:
            case PixelFormat::RGB16:
            case PixelFormat::RGB32:
                // Again, reorder to BGR...
                dst[0] = FloatToSRGB(GetChannel({x, y}, 2));
                dst[1] = FloatToSRGB(GetChannel({x, y}, 1));
                dst[2] = FloatToSRGB(GetChannel({x, y}, 0));
                dst += 3;
                break;
            default:
                LOG(FATAL) << "Unhandled pixel format in WriteTGA";
            }
        }
    }

    tga_result result;
    if (nc == 1)
        result = tga_write_mono(name.c_str(), outBuf.get(), resolution.x,
                                resolution.y);
    else
        result = tga_write_bgr(name.c_str(), outBuf.get(), resolution.x,
                               resolution.y, 24);

    if (result != TGA_NOERR) {
        Error("Unable to write output file \"%s\" (%s)", name.c_str(),
              tga_error(result));
        return false;
    }
    return true;
}

static bool ReadImageTGA(const std::string &name, bool gamma, Image *image) {
    tga_image img;
    tga_result result;
    if ((result = tga_read(&img, name.c_str())) != TGA_NOERR) {
        Error("Unable to read from TGA file \"%s\" (%s)", name.c_str(),
              tga_error(result));
        return false;
    }

    if (tga_is_right_to_left(&img)) tga_flip_horiz(&img);
    if (tga_is_top_to_bottom(&img)) tga_flip_vert(&img);
    if (tga_is_colormapped(&img)) tga_color_unmap(&img);

    int nChannels = tga_is_mono(&img) ? 1 : 3;
    std::vector<uint8_t> pixels(img.width * img.height * nChannels);

    // "Unpack" the pixels (origin in the lower left corner).
    uint8_t *dst = &pixels[0];
    for (int y = 0; y < img.height; y++)
        for (int x = 0; x < img.width; x++) {
            uint8_t *src = tga_find_pixel(&img, x, y);
            if (nChannels == 1)
                *dst++ = *src;
            else {
                // TGA pixels are in BGRA format.
                *dst++ = src[2];
                *dst++ = src[1];
                *dst++ = src[0];
            }
        }

    LOG(INFO) << StringPrintf("Read TGA image %s (%d x %d)",
                              name.c_str(), img.width, img.height);

    PixelFormat format;
    if (nChannels == 3)
      format = gamma ? PixelFormat::SRGB8 : PixelFormat::RGB8;
    else {
        CHECK_EQ(1, nChannels);
        format = gamma ? PixelFormat::SY8 : PixelFormat::Y8;
    }
    *image = Image(std::move(pixels), format,
                   Point2i(img.width, img.height));

    tga_free_buffers(&img);
    return true;
}

///////////////////////////////////////////////////////////////////////////
// PFM Function Definitions

/*
 * PFM reader/writer code courtesy Jiawen "Kevin" Chen
 * (http://people.csail.mit.edu/jiawen/)
 */

static PBRT_CONSTEXPR bool hostLittleEndian =
#if defined(__BYTE_ORDER__)
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    true
  #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    false
  #else
    #error "__BYTE_ORDER__ defined but has unexpected value"
  #endif
#else
  #if defined(__LITTLE_ENDIAN__) || defined(__i386__) || defined(__x86_64__) || \
      defined(WIN32)
    true
  #elif defined(__BIG_ENDIAN__)
    false
  #elif defined(__sparc) || defined(__sparc__)
    false
  #else
    #error "Can't detect machine endian-ness at compile-time."
  #endif
#endif
    ;

#define BUFFER_SIZE 80

static inline int isWhitespace(char c) {
    return c == ' ' || c == '\n' || c == '\t';
}

// Reads a "word" from the fp and puts it into buffer and adds a null
// terminator.  i.e. it keeps reading until whitespace is reached.  Returns
// the number of characters read *not* including the whitespace, and
// returns -1 on an error.
static int readWord(FILE *fp, char *buffer, int bufferLength) {
    int n;
    int c;

    if (bufferLength < 1) return -1;

    n = 0;
    c = fgetc(fp);
    while (c != EOF && !isWhitespace(c) && n < bufferLength) {
        buffer[n] = c;
        ++n;
        c = fgetc(fp);
    }

    if (n < bufferLength) {
        buffer[n] = '\0';
        return n;
    }

    return -1;
}

static bool ReadImagePFM(const std::string &filename, Image *image) {
    std::vector<float> rgb32;
    char buffer[BUFFER_SIZE];
    unsigned int nFloats;
    int nChannels, width, height;
    float scale;
    bool fileLittleEndian;

    FILE *fp = fopen(filename.c_str(), "rb");
    if (!fp) goto fail;

    // read either "Pf" or "PF"
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;

    if (strcmp(buffer, "Pf") == 0)
        nChannels = 1;
    else if (strcmp(buffer, "PF") == 0)
        nChannels = 3;
    else
        goto fail;

    // read the rest of the header
    // read width
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    width = atoi(buffer);

    // read height
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    height = atoi(buffer);

    // read scale
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    sscanf(buffer, "%f", &scale);

    // read the data
    nFloats = nChannels * width * height;
    rgb32.resize(nFloats);
    for (int y = height - 1; y >= 0; --y)
      if (fread(&rgb32[nChannels * y * width], sizeof(float),
                nChannels * width, fp) != nChannels * width) goto fail;

    // apply endian conversian and scale if appropriate
    fileLittleEndian = (scale < 0.f);
    if (hostLittleEndian ^ fileLittleEndian) {
        uint8_t bytes[4];
        for (unsigned int i = 0; i < nFloats; ++i) {
            memcpy(bytes, &rgb32[i], 4);
            std::swap(bytes[0], bytes[3]);
            std::swap(bytes[1], bytes[2]);
            memcpy(&rgb32[i], bytes, 4);
        }
    }
    if (std::abs(scale) != 1.f)
        for (unsigned int i = 0; i < nFloats; ++i) rgb32[i] *= std::abs(scale);

    // create RGBs...
    *image = Image(std::move(rgb32),
                   nChannels == 1 ? PixelFormat::Y32 : PixelFormat::RGB32,
                   Point2i(width, height));
    fclose(fp);

    LOG(INFO) << StringPrintf("Read PFM image %s (%d x %d)",
                              filename.c_str(), width, height);
    return true;

fail:
    Error("Error reading PFM file \"%s\"", filename.c_str());
    if (fp) fclose(fp);
    return false;
}

bool Image::WritePFM(const std::string &filename) const {
    FILE *fp;
    float scale;

    fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        Error("Unable to open output PFM file \"%s\"", filename.c_str());
        return false;
    }

    std::unique_ptr<float[]> scanline(new float[3 * resolution.x]);

    // only write 3 channel PFMs here...
    if (fprintf(fp, "PF\n") < 0) goto fail;

    // write the width and height, which must be positive
    if (fprintf(fp, "%d %d\n", resolution.x, resolution.y) < 0) goto fail;

    // write the scale, which encodes endianness
    scale = hostLittleEndian ? -1.f : 1.f;
    if (fprintf(fp, "%f\n", scale) < 0) goto fail;

    // write the data from bottom left to upper right as specified by
    // http://netpbm.sourceforge.net/doc/pfm.html
    // The raster is a sequence of pixels, packed one after another, with no
    // delimiters of any kind. They are grouped by row, with the pixels in each
    // row ordered left to right and the rows ordered bottom to top.
    for (int y = resolution.y - 1; y >= 0; y--) {
        for (int x = 0; x < resolution.x; ++x) {
            Spectrum s = GetSpectrum({x, y});
            Float rgb[3];
            s.ToRGB(rgb);
            for (int c = 0; c < 3; ++c)
                // Assign element-wise in case Float is typedefed as 'double'.
                scanline[3 * x + c] = rgb[c];
        }
        if (fwrite(&scanline[0], sizeof(float), 3 * resolution.x, fp) <
            (size_t)(3 * resolution.x))
            goto fail;
    }

    fclose(fp);
    return true;

fail:
    Error("Error writing PFM file \"%s\"", filename.c_str());
    fclose(fp);
    return false;
}

}  // namespace pbrt
