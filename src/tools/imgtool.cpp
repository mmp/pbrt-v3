//
// imgtool.cpp
//
// Various useful operations on images.
//

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include "fileutil.h"
#include "imageio.h"
#include "mipmap.h"
#include "parallel.h"
#include "pbrt.h"
#include "spectrum.h"
extern "C" {
#include "ext/ArHosekSkyModel.h"
}
#include <glog/logging.h>

using namespace pbrt;

static void usage(const char *msg = nullptr, ...) {
    if (msg) {
        va_list args;
        va_start(args, msg);
        fprintf(stderr, "imgtool: ");
        vfprintf(stderr, msg, args);
        fprintf(stderr, "\n");
    }
    fprintf(stderr, R"(usage: imgtool <command> [options] <filenames...>

commands: assemble, cat, convert, diff, info, makesky, maketiled

assemble option:
    --outfile          Output image filename.

cat option:
    --sort             Sort output by pixel luminance.

convert options:
    --bloomiters <n>   Number of filtering iterations used to generate the bloom
                       image. Default: 5
    --bloomlevel <n>   Minimum RGB value for a pixel for it to contribute to bloom.
                       Default: Infinity (i.e., no bloom is applied)
    --bloomscale <s>   Amount by which the bloom image is scaled before being
                       added to the original image. Default: 0.3
    --bloomswidth <w>  Width of Gaussian used to generate bloom images.
                       Default: 15
    --despike <v>      For any pixels with a luminance value greater than <v>,
                       replace the pixel with the median of the 3x3 neighboring
                       pixels. Default: infinity (i.e., disabled).
    --flipy            Flip the image along the y axis
    --maxluminance <n> Luminance value mapped to white by tonemapping.
                       Default: 1
    --preservecolors   By default, out-of-gammut colors have each component
                       clamped to [0,1] when written to non-HDR formats. With
                       this option enabled, such colors are scaled by their
                       maximum component, which preserves the relative ratio
                       between RGB components.
    --repeatpix <n>    Repeat each pixel value n times in both directions
    --scale <scale>    Scale pixel values by given amount
    --tonemap          Apply tonemapping to the image (Reinhard et al.'s
                       photographic tone mapping operator)

diff options:
    --difftol <v>      Acceptable image difference percentage before differences
                       are reported. Default: 0
    --outfile <name>   Filename to use for saving an image that encodes the
                       absolute value of per-pixel differences.

makesky options:
    --albedo <a>       Albedo of ground-plane (range 0-1). Default: 0.5
    --elevation <e>    Elevation of the sun in degrees (range 0-90). Default: 10
    --outfile <name>   Filename to store latitude-longitude environment map in.
                       Default: "sky.exr"
    --turbidity <t>    Atmospheric turbidity (range 1.7-10). Default: 3
    --resolution <r>   Vertical resolution of generated environment map.
                       (Horizontal resolution is twice this value.)
                       Default: 2048

maketiled options:
    --wrapmode <mode>  Image wrap mode used for out-of-bounds texture accesses.
                       Options: "clamp", "repeat", "black". Default: "clamp".

)");
    exit(1);
}

int makesky(int argc, char *argv[]) {
    const char *outfile = "sky.exr";
    float albedo = 0.5;
    float turbidity = 3.;
    float elevation = Radians(10);
    int resolution = 2048;

    int i;
    auto parseArg = [&]() -> std::pair<std::string, double> {
        const char *ptr = argv[i];
        // Skip over a leading dash or two.
        CHECK_EQ(*ptr, '-');
        ++ptr;
        if (*ptr == '-') ++ptr;

        // Copy the flag name to the string.
        std::string flag;
        while (*ptr && *ptr != '=') flag += *ptr++;

        if (!*ptr && i + 1 == argc)
            usage("missing value after %s flag", argv[i]);
        const char *value = (*ptr == '=') ? (ptr + 1) : argv[++i];
        return {flag, atof(value)};
    };

    for (i = 0; i < argc; ++i) {
        if (!strcmp(argv[i], "--outfile") || !strcmp(argv[i], "-outfile")) {
            if (i + 1 == argc)
                usage("missing filename for %s parameter", argv[i]);
            outfile = argv[++i];
        } else if (!strncmp(argv[i], "--outfile=", 10)) {
            outfile = &argv[i][10];
        } else {
            auto arg = parseArg();
            if (std::get<0>(arg) == "albedo") {
                albedo = std::get<1>(arg);
                if (albedo < 0. || albedo > 1.)
                    usage("--albedo must be between 0 and 1");
            } else if (std::get<0>(arg) == "turbidity") {
                turbidity = std::get<1>(arg);
                if (turbidity < 1.7 || turbidity > 10.)
                    usage("--turbidity must be between 1.7 and 10.");
            } else if (std::get<0>(arg) == "elevation") {
                elevation = std::get<1>(arg);
                if (elevation < 0. || elevation > 90.)
                    usage("--elevation must be between 0. and 90.");
                elevation = Radians(elevation);
            } else if (std::get<0>(arg) == "resolution") {
                resolution = int(std::get<1>(arg));
                if (resolution < 1) usage("--resolution must be >= 1");
            } else
                usage();
        }
    }

    PBRT_CONSTEXPR int num_channels = 9;
    // Three wavelengths around red, three around green, and three around blue.
    double lambda[num_channels] = {630, 680, 710, 500, 530, 560, 460, 480, 490};

    ArHosekSkyModelState *skymodel_state[num_channels];
    for (int i = 0; i < num_channels; ++i) {
        skymodel_state[i] =
            arhosekskymodelstate_alloc_init(elevation, turbidity, albedo);
    }

    // Vector pointing at the sun. Note that elevation is measured from the
    // horizon--not the zenith, as it is elsewhere in pbrt.
    Vector3f sunDir(0., std::sin(elevation), std::cos(elevation));

    int nTheta = resolution, nPhi = 2 * nTheta;
    Image img(PixelFormat::RGB32, {nPhi, nTheta});

    ParallelInit();
    ParallelFor(
        [&](int64_t t) {
            Float theta = float(t + 0.5) / nTheta * Pi;
            if (theta > Pi / 2.) return;
            for (int p = 0; p < nPhi; ++p) {
                Float phi = float(p + 0.5) / nPhi * 2. * Pi;

                // Vector corresponding to the direction for this pixel.
                Vector3f v(std::cos(phi) * std::sin(theta), std::cos(theta),
                           std::sin(phi) * std::sin(theta));
                // Compute the angle between the pixel's direction and the sun
                // direction.
                Float gamma = std::acos(Clamp(Dot(v, sunDir), -1, 1));
                CHECK(gamma >= 0 && gamma <= Pi);

                Float rgb[3] = {Float(0), Float(0), Float(0)};
                for (int c = 0; c < num_channels; ++c) {
                    float val = arhosekskymodel_solar_radiance(
                        skymodel_state[c], theta, gamma, lambda[c]);
                    // For each of red, green, and blue, average the three
                    // values for the three wavelengths for the color.
                    // TODO: do a better spectral->RGB conversion.
                    rgb[c / 3] += val / 3.f;
                }
                for (int c = 0; c < 3; ++c)
                    img.SetChannel({p, (int)t}, c, rgb[c]);
            }
        },
        nTheta, 32);

    CHECK(img.Write(outfile));

    ParallelCleanup();
    return 0;
}

int assemble(int argc, char *argv[]) {
    if (argc == 0) usage("no filenames provided to \"assemble\"?");
    const char *outfile = nullptr;
    std::vector<const char *> infiles;
    for (int i = 0; i < argc; ++i) {
        if (!strcmp(argv[i], "--outfile") || !strcmp(argv[i], "-outfile")) {
            if (i + 1 == argc)
                usage("missing filename for %s parameter", argv[i]);
            outfile = argv[++i];
        } else if (!strncmp(argv[i], "--outfile=", 10)) {
            outfile = &argv[i][10];
        } else
            infiles.push_back(argv[i]);
    }

    if (!outfile) usage("--outfile not provided for \"assemble\"");

    Image fullImage;
    std::vector<bool> seenPixel;
    int seenMultiple = 0;
    Point2i fullRes;
    Bounds2i displayWindow;
    for (const char *file : infiles) {
        if (!HasExtension(file, ".exr"))
            usage(
                "only EXR images include the image bounding boxes that "
                "\"assemble\" needs.");

        Bounds2i dataWindow, dspw;
        Point2i res;
        Image img;
        if (!Image::Read(file, &img, true, &dataWindow, &dspw)) continue;

        if (fullImage.resolution == Point2i(0, 0)) {
            // First image read.
            fullRes = Point2i(dspw.pMax - dspw.pMin);
            fullImage = Image(img.format, fullRes);
            seenPixel.resize(fullRes.x * fullRes.y);
            displayWindow = dspw;
        } else {
            // Make sure that this image's info is compatible with the
            // first image's.
            if (dspw != displayWindow) {
                fprintf(stderr,
                        "%s: displayWindow (%d,%d) - (%d,%d) in EXR file "
                        "doesn't match the displayWindow of first EXR file "
                        "(%d,%d) - (%d,%d). "
                        "Ignoring this file.\n",
                        file, dspw.pMin.x, dspw.pMin.y, dspw.pMax.x,
                        dspw.pMax.y, displayWindow.pMin.x, displayWindow.pMin.y,
                        displayWindow.pMax.x, displayWindow.pMax.y);
                continue;
            }
            if (Union(dataWindow, displayWindow) != displayWindow) {
                fprintf(stderr,
                        "%s: dataWindow (%d,%d) - (%d,%d) in EXR file isn't "
                        "inside the displayWindow of first EXR file (%d,%d) - "
                        "(%d,%d). "
                        "Ignoring this file.\n",
                        file, dataWindow.pMin.x, dataWindow.pMin.y,
                        dataWindow.pMax.x, dataWindow.pMax.y,
                        displayWindow.pMin.x, displayWindow.pMin.y,
                        displayWindow.pMax.x, displayWindow.pMax.y);
                continue;
            }
            if (fullImage.nChannels() != img.nChannels()) {
                fprintf(stderr,
                        "%s: %d channel image; expecting %d channels.\n", file,
                        img.nChannels(), fullImage.nChannels());
                continue;
            }
        }

        // Copy pixels.
        for (int y = 0; y < res.y; ++y)
            for (int x = 0; x < res.x; ++x) {
                int fullOffset = (y + dataWindow.pMin.y) * fullRes.x +
                                 (x + dataWindow.pMin.x);
                if (seenPixel[fullOffset]) ++seenMultiple;
                seenPixel[fullOffset] = true;
                Point2i fullXY{x + dataWindow.pMin.x, y + dataWindow.pMin.y};
                for (int c = 0; c < fullImage.nChannels(); ++c)
                    fullImage.SetChannel(fullXY, c, img.GetChannel({x, y}, c));
            }
    }

    int unseenPixels = 0;
    for (int y = 0; y < fullRes.y; ++y)
        for (int x = 0; x < fullRes.x; ++x)
            if (!seenPixel[y * fullRes.x + x]) ++unseenPixels;

    if (seenMultiple > 0)
        fprintf(stderr, "%s: %d pixels present in multiple images.\n", outfile,
                seenMultiple);
    if (unseenPixels > 0)
        fprintf(stderr, "%s: %d pixels not present in any images.\n", outfile,
                unseenPixels);

    fullImage.Write(outfile);

    return 0;
}

int cat(int argc, char *argv[]) {
    if (argc == 0) usage("no filenames provided to \"cat\"?");
    bool sort = false;

    for (int i = 0; i < argc; ++i) {
        if (!strcmp(argv[i], "--sort") || !strcmp(argv[i], "-sort")) {
            sort = !sort;
            continue;
        }

        Image img;
        if (!Image::Read(argv[i], &img)) {
            fprintf(stderr, "imgtool: couldn't open \"%s\".\n", argv[i]);
            continue;
        }

        if (sort) {
            std::vector<std::tuple<int, int, std::array<Float, 3>>> sorted;
            sorted.reserve(img.resolution.x * img.resolution.y);
            for (int y = 0; y < img.resolution.y; ++y)
                for (int x = 0; x < img.resolution.x; ++x) {
                    Spectrum s = img.GetSpectrum({x, y});
                    std::array<Float, 3> rgb;
                    s.ToRGB(&rgb[0]);
                    sorted.push_back(std::make_tuple(x, y, rgb));
                }

            std::sort(sorted.begin(), sorted.end(),
                      [](const std::tuple<int, int, std::array<Float, 3>> &a,
                         const std::tuple<int, int, std::array<Float, 3>> &b) {
                          const std::array<Float, 3> ac = std::get<2>(a);
                          const std::array<Float, 3> bc = std::get<2>(b);
                          return (ac[0] + ac[1] + ac[2]) <
                                 (bc[0] + bc[1] + bc[2]);
                      });
            for (const auto &v : sorted) {
                const std::array<Float, 3> &rgb = std::get<2>(v);
                printf("(%d, %d): (%.9g %.9g %.9g)\n", std::get<0>(v),
                       std::get<1>(v), rgb[0], rgb[1], rgb[2]);
            }
        } else {
            for (int y = 0; y < img.resolution.y; ++y) {
                for (int x = 0; x < img.resolution.x; ++x) {
                    Spectrum s = img.GetSpectrum({x, y});
                    std::array<Float, 3> rgb;
                    s.ToRGB(&rgb[0]);
                    printf("(%d, %d): (%.9g %.9g %.9g)\n", x, y, rgb[0], rgb[1],
                           rgb[2]);
                }
            }
        }
    }
    return 0;
}

int diff(int argc, char *argv[]) {
    float tol = 0.;
    const char *outfile = nullptr;

    int i;
    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "--outfile") || !strcmp(argv[i], "-outfile") ||
            !strcmp(argv[i], "-o")) {
            if (i + 1 == argc)
                usage("missing filename after %s option", argv[i]);
            outfile = argv[++i];
        } else if (!strncmp(argv[i], "--outfile=", 10)) {
            outfile = &argv[i][10];
        } else if (!strcmp(argv[i], "--difftol") ||
                   !strcmp(argv[i], "-difftol") || !strcmp(argv[i], "-d")) {
            if (i + 1 == argc)
                usage("missing filename after %s option", argv[i]);
            ++i;
            if (!isdigit(argv[i][0]) && argv[i][0] != '.')
                usage("argument after %s doesn't look like a number", argv[i]);
            tol = atof(argv[i]);
        } else if (!strncmp(argv[i], "--difftol=", 10))
            tol = atof(&argv[i][10]);
        else
            usage("unknown \"diff\" option");
    }

    if (i >= argc)
        usage("missing filenames for \"diff\"");
    else if (i + 1 >= argc)
        usage("missing second filename for \"diff\"");
    else if (i + 2 < argc)
        usage("excess filenames provided to \"diff\"");

    const char *filename[2] = {argv[i], argv[i + 1]};
    Image img[2];
    for (int i = 0; i < 2; ++i)
        if (!Image::Read(filename[i], &img[i])) {
            fprintf(stderr, "%s: unable to read image\n", filename[i]);
            return 1;
        }

    if (img[0].resolution != img[1].resolution) {
        fprintf(stderr,
                "imgtool: image resolutions don't match \"%s\": (%d, %d) "
                "\"%s\": (%d, %d)\n",
                filename[0], img[0].resolution.x, img[0].resolution.y,
                filename[1], img[1].resolution.x, img[1].resolution.y);
        return 1;
    }

    Point2i res = img[0].resolution;
    Image diffImage(PixelFormat::RGB32, res);

    double sum[2] = {0., 0.};
    int smallDiff = 0, bigDiff = 0;
    double mse = 0.f;
    for (int y = 0; y < res.y; ++y) {
        for (int x = 0; x < res.x; ++x) {
            Spectrum s[2] = {img[0].GetSpectrum({x, y}),
                             img[1].GetSpectrum({x, y})};
            Spectrum diff;

            for (int c = 0; c < Spectrum::nSamples; ++c) {
                Float c0 = s[0][c], c1 = s[1][c];
                diff[c] = std::abs(c0 - c1);

                if (c0 == 0 && c1 == 0) continue;

                sum[0] += c0;
                sum[1] += c1;

                float d = std::abs(c0 - c1) / c0;
                mse += (c0 - c1) * (c0 - c1);
                if (d > .005) ++smallDiff;
                if (d > .05) ++bigDiff;
            }
            diffImage.SetSpectrum({x, y}, diff);
        }
    }

    double avg[2] = {sum[0] / (Spectrum::nSamples * res.x * res.y),
                     sum[1] / (Spectrum::nSamples * res.x * res.y)};
    double avgDelta = (avg[0] - avg[1]) / std::min(avg[0], avg[1]);
    if ((tol == 0. && (bigDiff > 0 || smallDiff > 0)) ||
        (tol > 0. && 100.f * std::abs(avgDelta) > tol)) {
        printf(
            "%s %s\n\tImages differ: %d big (%.2f%%), %d small (%.2f%%)\n"
            "\tavg 1 = %g, avg2 = %g (%f%% delta)\n"
            "\tMSE = %g, RMS = %.3f%%\n",
            filename[0], filename[1], bigDiff,
            100.f * float(bigDiff) / (3 * res.x * res.y), smallDiff,
            100.f * float(smallDiff) / (3 * res.x * res.y), avg[0], avg[1],
            100. * avgDelta, mse / (3. * res.x * res.y),
            100. * sqrt(mse / (3. * res.x * res.y)));
        if (outfile) {
            if (!diffImage.Write(outfile))
                fprintf(stderr, "imgtool: unable to write \"%s\": %s\n",
                        outfile, strerror(errno));
        }
        return 1;
    }

    return 0;
}

static void printImageStats(const char *name, const Image &image) {
    printf("%s: resolution %d, %d\n", name, image.resolution.x,
           image.resolution.y);
    Float min[3] = {Infinity, Infinity, Infinity};
    Float max[3] = {-Infinity, -Infinity, -Infinity};
    double sum[3] = {0., 0., 0.};
    double logYSum = 0.;
    int nNaN = 0, nInf = 0, nValid = 0;
    int nc = image.nChannels();
    CHECK_LE(nc, 3);  // fixed-sized arrays above...
    for (int y = 0; y < image.resolution.y; ++y)
        for (int x = 0; x < image.resolution.x; ++x) {
            Float lum = image.GetY({x, y});
            if (!std::isnan(lum) && !std::isinf(lum))
                logYSum += std::log(Float(1e-6) + lum);

            for (int c = 0; c < nc; ++c) {
                Float v = image.GetChannel({x, y}, c);
                if (std::isnan(v))
                    ++nNaN;
                else if (std::isinf(v))
                    ++nInf;
                else {
                    min[c] = std::min(min[c], v);
                    max[c] = std::max(max[c], v);
                    sum[c] += v;
                    ++nValid;
                }
            }
        }

    printf("%s: %d infinite pixel components, %d NaN, %d valid.\n", name, nInf,
           nNaN, nValid);
    printf("%s: log average luminance %f\n", name,
           std::exp(logYSum / (image.resolution.x * image.resolution.y)));
    printf("%s: min channel:", name);
    for (int c = 0; c < nc; ++c)
        printf(" %f%c", min[c], (c < nc - 1) ? ',' : ' ');
    printf("\n");
    printf("%s: max channel:", name);
    for (int c = 0; c < nc; ++c)
        printf(" %f%c", max[c], (c < nc - 1) ? ',' : ' ');
    printf("\n");
    printf("%s: avg channel:", name);
    for (int c = 0; c < nc; ++c)
        printf(" %f%c", sum[c] / nValid, (c < nc - 1) ? ',' : ' ');
    printf("\n");
}

int info(int argc, char *argv[]) {
    int err = 0;
    for (int i = 0; i < argc; ++i) {
        if (HasExtension(argv[i], "txp")) {
            std::unique_ptr<TextureCache> cache(new TextureCache);
            int id = cache->AddTexture(argv[i]);
            if (id < 0) {
                err = 1;
                continue;
            }
            printf("%s: wrap mode \"%s\"\n", argv[i],
                   WrapModeString(cache->GetWrapMode(id)));

            for (int level = 0; level < cache->Levels(id); ++level) {
                Image image = cache->GetLevelImage(id, level);
                printImageStats(
                    StringPrintf("%s-level%d", argv[i], level).c_str(), image);
            }
        } else {
            Point2i res;
            Image image;
            if (!Image::Read(argv[i], &image)) {
                fprintf(stderr, "%s: unable to load image.\n", argv[i]);
                err = 1;
                continue;
            }

            printImageStats(argv[i], image);
        }
    }
    return err;
}

Image bloom(Image image, Float level, int width, Float scale, int iters) {
    std::vector<Image> blurred;
    CHECK(image.nChannels() == 1 || image.nChannels() == 3);
    PixelFormat format =
        image.nChannels() == 1 ? PixelFormat::Y32 : PixelFormat::RGB32;

    // First, threshold the source image
    int nSurvivors = 0;
    Point2i res = image.resolution;
    int nc = image.nChannels();
    Image thresholdedImage(format, image.resolution);
    for (int y = 0; y < res.y; ++y) {
        for (int x = 0; x < res.x; ++x) {
            bool overThreshold = false;
            for (int c = 0; c < nc; ++c)
                if (image.GetChannel({x, y}, c) > level) overThreshold = true;
            if (overThreshold) {
                ++nSurvivors;
                for (int c = 0; c < nc; ++c)
                    thresholdedImage.SetChannel({x, y}, c,
                                                image.GetChannel({x, y}, c));
            } else
                for (int c = 0; c < nc; ++c)
                    thresholdedImage.SetChannel({x, y}, c, 0.f);
        }
    }
    if (nSurvivors == 0) {
        fprintf(stderr,
                "imgtool: warning: no pixels were above bloom threshold %f\n",
                level);
        return image;
    }
    blurred.push_back(std::move(thresholdedImage));

    if ((width % 2) == 0) {
        ++width;
        fprintf(
            stderr,
            "imgtool: bloom width must be an odd value. Rounding up to %d.\n",
            width);
    }
    int radius = width / 2;

    // Compute filter weights
    Float sigma = 2;  // TODO: make a parameter
    std::vector<Float> wts(width, Float(0));
    Float wtSum = 0;
    for (int i = 0; i < width; ++i) {
        Float v = std::abs(Float(i - radius)) / Float(radius);
        wts[i] = std::exp(-sigma * v);
        wtSum += wts[i];
    }
    // Normalize filter weights.
    for (int i = 0; i < width; ++i) wts[i] /= wtSum;

    // Now successively blur the thresholded image.
    Image blurx(format, res);
    for (int iter = 0; iter < iters; ++iter) {
        // Separable blur; first blur in x into blurx
        for (int y = 0; y < res.y; ++y) {
            for (int x = 0; x < res.x; ++x) {
                for (int c = 0; c < nc; ++c) {
                    Float result = 0;
                    for (int r = -radius; r <= radius; ++r)
                        result += wts[r + radius] *
                                  blurred.back().GetChannel({x + r, y}, c);
                    blurx.SetChannel({x, y}, c, result);
                }
            }
        }

        // Now blur in y from blur x to the result
        Image blury(format, res);
        for (int y = 0; y < res.y; ++y) {
            for (int x = 0; x < res.x; ++x) {
                for (int c = 0; c < nc; ++c) {
                    Float result = 0;
                    for (int r = -radius; r <= radius; ++r)
                        result +=
                            wts[r + radius] * blurx.GetChannel({x, y + r}, c);
                    blury.SetChannel({x, y}, c, result);
                }
            }
        }
        blurred.push_back(std::move(blury));
    }

    // Finally, add all of the blurred images, scaled, to the original.
    for (int y = 0; y < res.y; ++y) {
        for (int x = 0; x < res.x; ++x) {
            for (int c = 0; c < nc; ++c) {
                Float blurredSum = 0.f;
                // Skip the thresholded image, since it's already
                // present in the original; just add pixels from the
                // blurred ones.
                for (size_t j = 1; j < blurred.size(); ++j)
                    blurredSum += blurred[j].GetChannel({x, y}, c);
                image.SetChannel({x, y}, c, (scale / iters) * blurredSum);
            }
        }
    }

    return image;
}

int convert(int argc, char *argv[]) {
    float scale = 1.f;
    int repeat = 1;
    bool flipy = false;
    Float bloomLevel = Infinity;
    int bloomWidth = 15;
    Float bloomScale = .3;
    int bloomIters = 5;
    bool tonemap = false;
    Float maxY = 1.;
    Float despikeLimit = Infinity;
    bool preserveColors = false;

    int i;
    auto parseArg = [&]() -> std::pair<std::string, double> {
        const char *ptr = argv[i];
        // Skip over a leading dash or two.
        CHECK_EQ(*ptr, '-');
        ++ptr;
        if (*ptr == '-') ++ptr;

        // Copy the flag name to the string.
        std::string flag;
        while (*ptr && *ptr != '=') flag += *ptr++;

        if (!*ptr && i + 1 == argc)
            usage("missing value after %s flag", argv[i]);
        const char *value = (*ptr == '=') ? (ptr + 1) : argv[++i];
        return {flag, atof(value)};
    };

    std::pair<std::string, double> arg;
    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "--flipy") || !strcmp(argv[i], "-flipy"))
            flipy = !flipy;
        else if (!strcmp(argv[i], "--tonemap") || !strcmp(argv[i], "-tonemap"))
            tonemap = !tonemap;
        else if (!strcmp(argv[i], "--preservecolors") ||
                 !strcmp(argv[i], "-preservecolors"))
            preserveColors = !preserveColors;
        else {
            std::pair<std::string, double> arg = parseArg();
            if (std::get<0>(arg) == "maxluminance") {
                maxY = std::get<1>(arg);
                if (maxY <= 0)
                    usage("--maxluminance value must be greater than zero");
            } else if (std::get<0>(arg) == "repeatpix") {
                repeat = int(std::get<1>(arg));
                if (repeat <= 0)
                    usage("--repeatpix value must be greater than zero");
            } else if (std::get<0>(arg) == "scale") {
                scale = std::get<1>(arg);
                if (scale == 0) usage("--scale value must be non-zero");
            } else if (std::get<0>(arg) == "bloomlevel")
                bloomLevel = std::get<1>(arg);
            else if (std::get<0>(arg) == "bloomwidth")
                bloomWidth = int(std::get<1>(arg));
            else if (std::get<0>(arg) == "bloomscale")
                bloomScale = std::get<1>(arg);
            else if (std::get<0>(arg) == "bloomiters")
                bloomIters = int(std::get<1>(arg));
            else if (std::get<0>(arg) == "despike")
                despikeLimit = std::get<1>(arg);
            else
                usage();
        }
    }

    if (i + 1 >= argc)
        usage("missing second filename for \"convert\"");
    else if (i >= argc)
        usage("missing filenames for \"convert\"");

    const char *inFilename = argv[i], *outFilename = argv[i + 1];
    Image image;
    if (HasExtension(inFilename, "txp")) {
        std::unique_ptr<TextureCache> cache(new TextureCache);
        int id = cache->AddTexture(inFilename);
        if (id < 0) {
            fprintf(stderr, "%s: unable to read image\n", inFilename);
            return 1;
        }

        std::vector<Image> levelImages;
        int sumWidth = 0, maxHeight = 0;
        for (int level = 0; level < cache->Levels(id); ++level) {
            levelImages.push_back(cache->GetLevelImage(id, level));
            sumWidth += levelImages.back().resolution[0];
            maxHeight = std::max(maxHeight, levelImages.back().resolution[1]);
            if (level > 0)
                CHECK(levelImages[level].format == levelImages[0].format);
        }

        image = Image(levelImages[0].format, {sumWidth, maxHeight});
        int xStart = 0;
        int nc = image.nChannels();
        for (const auto &im : levelImages) {
            for (int y = 0; y < im.resolution[1]; ++y)
                for (int x = 0; x < im.resolution[0]; ++x)
                    for (int c = 0; c < nc; ++c)
                        image.SetChannel({x + xStart, y}, c,
                                         im.GetChannel({x, y}, c));
            xStart += im.resolution[0];
        }
    } else if (!Image::Read(inFilename, &image)) {
        fprintf(stderr, "%s: unable to read image\n", inFilename);
        return 1;
    }
    Point2i res = image.resolution;
    int nc = image.nChannels();

    // Convert to a 32-bit format for maximum accuracy in the following
    // processing.
    if (!Is32Bit(image.format)) {
        CHECK(nc == 1 || nc == 3);
        image = image.ConvertToFormat(nc == 1 ? PixelFormat::Y32
                                              : PixelFormat::RGB32);
    }

    for (int y = 0; y < res.y; ++y)
        for (int x = 0; x < res.x; ++x)
            for (int c = 0; c < nc; ++c)
                image.SetChannel({x, y}, c,
                                 scale * image.GetChannel({x, y}, c));

    if (despikeLimit < Infinity) {
        Image filteredImg = image;
        int despikeCount = 0;
        for (int y = 0; y < res.y; ++y) {
            for (int x = 0; x < res.x; ++x) {
                if (image.GetY({x, y}) < despikeLimit) continue;

                // Copy all of the valid neighbor pixels into neighbors[].
                ++despikeCount;
                int validNeighbors = 0;
                Spectrum neighbors[9];
                for (int dy = -1; dy <= 1; ++dy) {
                    if (y + dy < 0 || y + dy >= res.y) continue;
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (x + dx < 0 || x + dx > res.x) continue;
                        neighbors[validNeighbors++] =
                            image.GetSpectrum({x + dx, y + dy});
                    }
                }

                // Find the median of the neighbors, sorted by luminance.
                int mid = validNeighbors / 2;
                std::nth_element(
                    &neighbors[0], &neighbors[mid], &neighbors[validNeighbors],
                    [](const Spectrum &a, const Spectrum &b) -> bool {
                        return a.y() < b.y();
                    });
                filteredImg.SetSpectrum({x, y}, neighbors[mid]);
            }
        }
        std::swap(image, filteredImg);
        fprintf(stderr, "%s: despiked %d pixels\n", inFilename, despikeCount);
    }

    if (bloomLevel < Infinity)
        image = bloom(std::move(image), bloomLevel, bloomWidth, bloomScale,
                      bloomIters);

    if (tonemap) {
        for (int y = 0; y < res.y; ++y)
            for (int x = 0; x < res.x; ++x) {
                Float lum = image.GetY({x, y});
                // Reinhard et al. photographic tone mapping operator.
                Float scale = (1 + lum / (maxY * maxY)) / (1 + lum);
                for (int c = 0; c < nc; ++c)
                    image.SetChannel({x, y}, c,
                                     scale * image.GetChannel({x, y}, c));
            }
    }

    if (preserveColors) {
        for (int y = 0; y < res.y; ++y)
            for (int x = 0; x < res.x; ++x) {
                Float m = image.GetChannel({x, y}, 0);
                for (int c = 1; c < nc; ++c)
                    m = std::max(m, image.GetChannel({x, y}, c));
                if (m > 1) {
                    for (int c = 0; c < nc; ++c)
                        image.SetChannel({x, y}, c,
                                         image.GetChannel({x, y}, c) / m);
                }
            }
    }

    if (repeat > 1) {
        Image scaledImage(image.format,
                          Point2i(res.x * repeat, res.y * repeat));
        for (int y = 0; y < repeat * res.y; ++y) {
            int yy = y / repeat;
            for (int x = 0; x < repeat * res.x; ++x) {
                int xx = x / repeat;
                for (int c = 0; c < nc; ++c)
                    scaledImage.SetChannel({x, y}, c,
                                           image.GetChannel({xx, yy}, c));
            }
        }
        image = std::move(scaledImage);
        res = image.resolution;
    }

    if (flipy) image.FlipY();

    if (!image.Write(outFilename)) {
        fprintf(stderr, "imgtool: couldn't write to \"%s\".\n", outFilename);
        return 1;
    }

    return 0;
}

int maketiled(int argc, char *argv[]) {
    ParallelInit();
    WrapMode wrapMode = WrapMode::Clamp;

    int i;
    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "--wrapmode") || !strcmp(argv[i], "-wrapmode")) {
            if (i + 1 == argc)
                usage("missing wrap mode after %s option", argv[i]);
            ++i;
            if (!ParseWrapMode(argv[i], &wrapMode))
                usage(
                    "unknown wrap mode %s. Expected \"clamp\", \"repeat\", "
                    "or \"black\".",
                    argv[i]);
        } else if (!strncmp(argv[i], "--wrapmode=", 11)) {
            if (!ParseWrapMode(argv[i] + 11, &wrapMode))
                usage(
                    "unknown wrap mode %s. Expected \"clamp\", \"repeat\", "
                    "or \"black\".",
                    argv[i] + 11);
        } else
            usage("unknown \"maketiled\" option \"%s\"", argv[i]);
    }
    if (i + 2 != argc) {
        usage("expecting input and output filenames as arguments");
    }
    const char *infile = argv[i], *outfile = argv[i + 1];

    Image image;
    if (!Image::Read(infile, &image)) {
        fprintf(stderr, "imgtool: unable to read image \"%s\"\n", infile);
        return 1;
    }

    std::vector<Image> mips = image.GenerateMIPMap(wrapMode);
    int tileSize = TextureCache::TileSize(image.format);
    if (!TiledImagePyramid::Create(std::move(mips), outfile, wrapMode, tileSize)) {
        fprintf(stderr, "imgtool: unable to create tiled image \"%s\"\n",
                outfile);
        return 1;
    }
    ParallelCleanup();
    return 0;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 1;  // Warning and above.

    if (argc < 2) usage();

    if (!strcmp(argv[1], "assemble"))
        return assemble(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "cat"))
        return cat(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "convert"))
        return convert(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "diff"))
        return diff(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "info"))
        return info(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "makesky"))
        return makesky(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "maketiled"))
        return maketiled(argc - 2, argv + 2);
    else
        usage("unknown command \"%s\"", argv[1]);

    return 0;
}
