
/*
    pbrt source code is Copyright(c) 1998-2015
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

#include "stdafx.h"

// core/imageio.cpp*
#include "imageio.h"
#include <string.h>
#include "spectrum.h"
#include "ext/tinyexr.h"
#include "ext/targa.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "ext/stb_image_write.h"

// ImageIO Local Declarations
static RGBSpectrum *ReadImageEXR(const std::string &name, int *width,
                                 int *height);
static void WriteImageEXR(const std::string &name, const Float *pixels,
                          int xRes, int yRes, int totalXRes, int totalYRes,
                          int xOffset, int yOffset);
static void WriteImageTGA(const std::string &name, const uint8_t *pixels,
                          int xRes, int yRes, int totalXRes, int totalYRes,
                          int xOffset, int yOffset);
static RGBSpectrum *ReadImageTGA(const std::string &name, int *w, int *h);
static bool WriteImagePFM(const std::string &filename, const Float *rgb,
                          int xres, int yres);
static RGBSpectrum *ReadImagePFM(const std::string &filename, int *xres,
                                 int *yres);

// ImageIO Function Definitions
std::unique_ptr<RGBSpectrum[]> ReadImage(const std::string &name,
                                         Point2i *resolution) {
    if (HasExtension(name, ".exr"))
        return std::unique_ptr<RGBSpectrum[]>(
            ReadImageEXR(name, &resolution->x, &resolution->y));
    else if (HasExtension(name, ".tga"))
        return std::unique_ptr<RGBSpectrum[]>(
            ReadImageTGA(name, &resolution->x, &resolution->y));
    else if (HasExtension(name, ".pfm"))
        return std::unique_ptr<RGBSpectrum[]>(
            ReadImagePFM(name, &resolution->x, &resolution->y));
    Error(
        "Unable to load image stored in format \"%s\" for filename \"%s\". "
        "Returning a constant gray image instead.",
        strrchr(name.c_str(), '.') ? (strrchr(name.c_str(), '.') + 1)
                                   : "(unknown)",
        name.c_str());
    RGBSpectrum *ret = new RGBSpectrum[1];
    ret[0] = RGBSpectrum(0.5f);
    resolution->x = resolution->y = 1;
    return std::unique_ptr<RGBSpectrum[]>(ret);
}

void WriteImage(const std::string &name, const Float *rgb,
                const Bounds2i &outputBounds, const Point2i &totalResolution) {
    Vector2i resolution = outputBounds.Diagonal();
    if (HasExtension(name, ".exr")) {
        WriteImageEXR(name, rgb, resolution.x, resolution.y, totalResolution.x,
                      totalResolution.y, outputBounds.pMin.x,
                      outputBounds.pMin.y);
    } else if (HasExtension(name, ".exr")) {
        WriteImagePFM(name, rgb, resolution.x, resolution.y);
    } else if (HasExtension(name, ".tga") || HasExtension(name, ".png")) {
        // 8-bit formats; apply gamma
        Vector2i resolution = outputBounds.Diagonal();
        std::unique_ptr<uint8_t[]> rgb8(
            new uint8_t[3 * resolution.x * resolution.y]);
        uint8_t *dst = rgb8.get();
        for (int y = 0; y < resolution.y; ++y) {
            for (int x = 0; x < resolution.x; ++x) {
#define TO_BYTE(v) (uint8_t) Clamp(255.f * GammaCorrect(v) + 0.5f, 0.f, 255.f)
                dst[0] = TO_BYTE(rgb[3 * (y * resolution.x + x) + 0]);
                dst[1] = TO_BYTE(rgb[3 * (y * resolution.x + x) + 1]);
                dst[2] = TO_BYTE(rgb[3 * (y * resolution.x + x) + 2]);
#undef TO_BYTE
                dst += 3;
            }
        }

        if (HasExtension(name, ".tga"))
            WriteImageTGA(
                name, rgb8.get(), outputBounds.pMax.x - outputBounds.pMin.x,
                outputBounds.pMax.y - outputBounds.pMin.y, totalResolution.x,
                totalResolution.y, outputBounds.pMin.x, outputBounds.pMin.y);
        else if (stbi_write_png(name.c_str(), resolution.x, resolution.y, 3,
                                rgb8.get(), 3 * resolution.x) == 0)
            Error("Error writing PNG \"%s\"", name.c_str());
    } else {
        Error("Can't determine image file type from suffix of filename \"%s\"",
              name.c_str());
    }
}

static Float convert(void *p, int offset, int type) {
    switch (type) {
    case TINYEXR_PIXELTYPE_UINT: {
        int32_t *pix = (int32_t *)p;
        return pix[offset];
    }
    case TINYEXR_PIXELTYPE_HALF:
    case TINYEXR_PIXELTYPE_FLOAT: {
        float *pix = (float *)p;
        return pix[offset];
    }
    default:
        Severe("Unexpected pixel type in EXR image");
        return 0;
    }
}

static RGBSpectrum *ReadImageEXR(const std::string &name, int *width,
                                 int *height) {
    EXRImage img;
    const char *err = nullptr;
    if (ParseMultiChannelEXRHeaderFromFile(&img, name.c_str(), &err) != 0) {
        Error("Unable to read \"%s\": %s", name.c_str(), err);
        return nullptr;
    }
    for (int i = 0; i < img.num_channels; ++i) {
        if (img.requested_pixel_types[i] == TINYEXR_PIXELTYPE_HALF)
            img.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
    }
    if (LoadMultiChannelEXRFromFile(&img, name.c_str(), &err) != 0) {
        Error("Unable to read \"%s\": %s", name.c_str(), err);
        return nullptr;
    }

    *width = img.width;
    *height = img.height;

    int idxR = -1, idxG = -1, idxB = -1;
    for (int c = 0; c < img.num_channels; c++) {
        if (strcmp(img.channel_names[c], "R") == 0) {
            idxR = c;
        } else if (strcmp(img.channel_names[c], "G") == 0) {
            idxG = c;
        } else if (strcmp(img.channel_names[c], "B") == 0) {
            idxB = c;
        }
    }

    RGBSpectrum *ret = new RGBSpectrum[img.width * img.height];
    int offset = 0;
    for (int y = 0; y < img.height; ++y) {
        for (int x = 0; x < img.width; ++x, ++offset) {
            if (img.num_channels == 1)
                ret[offset] =
                    convert(img.images[0], offset, img.pixel_types[0]);
            else {
                Float rgb[3] = {
                    convert(img.images[idxR], offset, img.pixel_types[idxR]),
                    convert(img.images[idxG], offset, img.pixel_types[idxG]),
                    convert(img.images[idxB], offset, img.pixel_types[idxB])};
                ret[offset] = RGBSpectrum::FromRGB(rgb);
            }
        }
    }
    FreeEXRImage(&img);

    return ret;
}

static void WriteImageEXR(const std::string &name, const Float *pixels,
                          int xRes, int yRes, int totalXRes, int totalYRes,
                          int xOffset, int yOffset) {
    EXRImage image;
    image.num_channels = 3;
    const char *channel_names[] = {"B", "G", "R"};
    image.channel_names = channel_names;
    int pixel_types[3] = {TINYEXR_PIXELTYPE_FLOAT, TINYEXR_PIXELTYPE_FLOAT,
                          TINYEXR_PIXELTYPE_FLOAT};
    image.pixel_types = pixel_types;
    int requestedPixelTypes[] = {TINYEXR_PIXELTYPE_HALF, TINYEXR_PIXELTYPE_HALF,
                                 TINYEXR_PIXELTYPE_HALF};
    image.requested_pixel_types = requestedPixelTypes;
    image.width = xRes;
    image.height = yRes;
    image.images = new unsigned char *[3];
    float *bgr = new float[3 * xRes * yRes];
    image.images[0] = (unsigned char *)bgr;
    image.images[1] = (unsigned char *)(bgr + 1 * xRes * yRes);
    image.images[2] = (unsigned char *)(bgr + 2 * xRes * yRes);

    int offset = 0;
    for (int y = 0; y < yRes; ++y) {
        for (int x = 0; x < xRes; ++x, ++offset) {
            ((float *)image.images[0])[offset] = pixels[3 * offset + 2];  // B
            ((float *)image.images[1])[offset] = pixels[3 * offset + 1];  // G
            ((float *)image.images[2])[offset] = pixels[3 * offset + 0];  // R
        }
    }

    const char *err;
    if (SaveMultiChannelEXRToFile(&image, name.c_str(), &err)) {
        Error("Error writing \"%s\": %s", name.c_str(), err);
    }

    delete[] bgr;
    delete[] image.images;
}

// TGA Function Definitions
void WriteImageTGA(const std::string &name, const uint8_t *pixels, int xRes,
                   int yRes, int totalXRes, int totalYRes, int xOffset,
                   int yOffset) {
    // Reformat to BGR layout.
    std::unique_ptr<uint8_t[]> outBuf(new uint8_t[3 * xRes * yRes]);
    uint8_t *dst = outBuf.get();
    const uint8_t *src = pixels;
    for (int y = 0; y < yRes; ++y) {
        for (int x = 0; x < xRes; ++x) {
            dst[0] = src[2];
            dst[1] = src[1];
            dst[2] = src[0];
            dst += 3;
            src += 3;
        }
    }

    tga_result result;
    if ((result = tga_write_bgr(name.c_str(), outBuf.get(), xRes, yRes, 24)) !=
        TGA_NOERR)
        Error("Unable to write output file \"%s\" (%s)", name.c_str(),
              tga_error(result));
}

static RGBSpectrum *ReadImageTGA(const std::string &name, int *width,
                                 int *height) {
    tga_image img;
    tga_result result;
    if ((result = tga_read(&img, name.c_str())) != TGA_NOERR) {
        Error("Unable to read from TGA file \"%s\" (%s)", name.c_str(),
              tga_error(result));
        return nullptr;
    }

    if (tga_is_right_to_left(&img)) tga_flip_horiz(&img);
    if (!tga_is_top_to_bottom(&img)) tga_flip_vert(&img);
    if (tga_is_colormapped(&img)) tga_color_unmap(&img);

    *width = img.width;
    *height = img.height;

    // "Unpack" the pixels (origin in the lower left corner).
    // TGA pixels are in BGRA format.
    RGBSpectrum *ret = new RGBSpectrum[*width * *height];
    RGBSpectrum *dst = ret;
    for (int y = *height - 1; y >= 0; y--)
        for (int x = 0; x < *width; x++) {
            uint8_t *src = tga_find_pixel(&img, x, y);
            if (tga_is_mono(&img))
                *dst++ = RGBSpectrum(*src / 255.f);
            else {
                Float c[3];
                c[2] = src[0] / 255.f;
                c[1] = src[1] / 255.f;
                c[0] = src[2] / 255.f;
                *dst++ = RGBSpectrum::FromRGB(c);
            }
        }

    tga_free_buffers(&img);
    Info("Read TGA image %s (%d x %d)", name.c_str(), *width, *height);

    return ret;
}

// PFM Function Definitions
/*
 * PFM reader/writer code courtesy Jiawen "Kevin" Chen
 * (http://people.csail.mit.edu/jiawen/)
 */

static bool hostLittleEndian =
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
    ;

#define BUFFER_SIZE 80

static inline int isWhitespace(char c) {
    return c == ' ' || c == '\n' || c == '\t';
}

// reads a "word" from the fp and puts it into buffer
// and adds a null terminator
// i.e. it keeps reading until a whitespace is reached
// returns the number of characters read
// *not* including the whitespace
// return -1 on an error
static int readWord(FILE *fp, char *buffer, int bufferLength) {
    int n;
    char c;

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

static RGBSpectrum *ReadImagePFM(const std::string &filename, int *xres,
                                 int *yres) {
    float *data = nullptr;
    RGBSpectrum *rgb = nullptr;
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
    *xres = width;

    // read height
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    height = atoi(buffer);
    *yres = height;

    // read scale
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    sscanf(buffer, "%f", &scale);

    // read the data
    nFloats = nChannels * width * height;
    data = new float[nFloats];
    if (fread(data, sizeof(float), nFloats, fp) != nFloats) goto fail;

    // apply endian conversian and scale if appropriate
    fileLittleEndian = (scale < 0.f);
    if (hostLittleEndian ^ fileLittleEndian) {
        uint8_t bytes[4];
        for (unsigned int i = 0; i < nFloats; ++i) {
            memcpy(bytes, &data[i], 4);
            std::swap(bytes[0], bytes[3]);
            std::swap(bytes[1], bytes[2]);
            memcpy(&data[i], bytes, 4);
        }
    }
    if (std::abs(scale) != 1.f)
        for (unsigned int i = 0; i < nFloats; ++i) data[i] *= std::abs(scale);

    // create RGBs...
    rgb = new RGBSpectrum[width * height];
    if (nChannels == 1) {
        for (int i = 0; i < width * height; ++i) rgb[i] = RGBSpectrum(data[i]);
    } else {
        for (int i = 0; i < width * height; ++i) {
            Float frgb[3] = {data[3 * i], data[3 * i + 1], data[3 * i + 2]};
            rgb[i] = RGBSpectrum::FromRGB(frgb);
        }
    }

    delete[] data;
    fclose(fp);
    return rgb;

fail:
    Error("Error reading PFM file \"%s\"", filename.c_str());
    fclose(fp);
    delete[] data;
    delete[] rgb;
    return nullptr;
}

static bool WriteImagePFM(const std::string &filename, const Float *rgb,
                          int width, int height) {
    FILE *fp;
    float scale;

    fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        Error("Unable to open output PFM file \"%s\"", filename.c_str());
        return false;
    }

    std::unique_ptr<float[]> scanline(new float[3 * width]);

    // only write 3 channel PFMs here...
    if (fprintf(fp, "PF\n") < 0) goto fail;

    // write the width and height, which must be positive
    if (fprintf(fp, "%d %d\n", width, height) < 0) goto fail;

    // write the scale, which encodes endianness
    scale = hostLittleEndian ? -1.f : 1.f;
    if (fprintf(fp, "%f\n", scale) < 0) goto fail;

    // write the data from bottom left to upper right as specified by
    // http://netpbm.sourceforge.net/doc/pfm.html
    // The raster is a sequence of pixels, packed one after another, with no
    // delimiters of any kind. They are grouped by row, with the pixels in each
    // row ordered left to right and the rows ordered bottom to top.
    for (int y = height - 1; y >= 0; y--) {
        // in case Float is 'double', copy into a staging buffer that's
        // definitely a 32-bit float...
        for (int x = 0; x < 3 * width; ++x)
            scanline[x] = rgb[y * width * 3 + x];
        if (fwrite(&scanline[0], sizeof(float), width * 3, fp) <
            (size_t)(width * 3))
            goto fail;
    }

    fclose(fp);
    return true;

fail:
    Error("Error writing PFM file \"%s\"", filename.c_str());
    fclose(fp);
    return false;
}
