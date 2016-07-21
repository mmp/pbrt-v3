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
#include "pbrt.h"
#include "spectrum.h"
#include "parallel.h"
extern "C" {
#include "ext/ArHosekSkyModel.h"
}

static void usage(const char *msg = nullptr, ...) {
    if (msg) {
        va_list args;
        va_start(args, msg);
        fprintf(stderr, "imgtool: ");
        vfprintf(stderr, msg, args);
        fprintf(stderr, "\n");
    }
    fprintf(stderr, R"(usage: imgtool <command> [options] <filenames...>

commands: assemble, cat, convert, diff, info, makesky

assemble option:
    --outflie          Output image filename.

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
    --flipy            Flip the image along the y axis
    --maxluminance <n> Luminance value mapped to white by tonemapping.
                       Default: 1
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
        Assert(*ptr == '-');
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

    constexpr int num_channels = 9;
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
    std::vector<Float> img(3 * nTheta * nPhi, 0.f);
    ParallelFor([&](int64_t t) {
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
            Assert(gamma >= 0 && gamma <= Pi);

            for (int c = 0; c < num_channels; ++c) {
                float val = arhosekskymodel_solar_radiance(
                    skymodel_state[c], theta, gamma, lambda[c]);
                // For each of red, green, and blue, average the three
                // values for the three wavelengths for the color.
                // TODO: do a better spectral->RGB conversion.
                img[3 * (t * nPhi + p) + c / 3] += val / 3.f;
            }
        }
    }, nTheta, 32);

    WriteImage(outfile, (Float *)&img[0], Bounds2i({0, 0}, {nPhi, nTheta}),
               {nPhi, nTheta});
    TerminateWorkerThreads();
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

    std::unique_ptr<RGBSpectrum[]> fullImg;
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
        std::unique_ptr<RGBSpectrum[]> img(
            ReadImageEXR(file, &res.x, &res.y, &dataWindow, &dspw));
        if (!img) continue;

        if (!fullImg) {
            // First image read.
            fullRes = Point2i(dspw.pMax - dspw.pMin);
            fullImg.reset(new RGBSpectrum[fullRes.x * fullRes.y]);
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
        }

        // Copy pixels.
        for (int y = 0; y < res.y; ++y)
            for (int x = 0; x < res.x; ++x) {
                int fullOffset = (y + dataWindow.pMin.y) * fullRes.x +
                                 (x + dataWindow.pMin.x);
                fullImg[fullOffset] = img[y * res.x + x];
                if (seenPixel[fullOffset]) ++seenMultiple;
                seenPixel[fullOffset] = true;
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

    // TODO: fix yet another bad cast here.
    WriteImage(outfile, (Float *)fullImg.get(), displayWindow, fullRes);

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

        Point2i res;
        std::unique_ptr<RGBSpectrum[]> img = ReadImage(argv[i], &res);
        if (!img) continue;
        if (sort) {
            std::vector<std::pair<int, RGBSpectrum>> sorted;
            sorted.reserve(res.x * res.y);
            for (int y = 0; y < res.y; ++y) {
                for (int x = 0; x < res.x; ++x) {
                    int offset = y * res.x + x;
                    sorted.push_back({offset, img[offset]});
                }
            }
            std::sort(sorted.begin(), sorted.end(),
                      [](const std::pair<int, RGBSpectrum> &a,
                         const std::pair<int, RGBSpectrum> &b) {
                          return a.second.y() < b.second.y();
                      });
            for (const auto &v : sorted) {
                Float rgb[3];
                v.second.ToRGB(rgb);
                printf("(%d, %d): (%.9g %.9g %.9g)\n", v.first / res.x,
                       v.first % res.x, rgb[0], rgb[1], rgb[2]);
            }
        } else {
            for (int y = 0; y < res.y; ++y) {
                for (int x = 0; x < res.x; ++x) {
                    Float rgb[3];
                    img[y * res.x + x].ToRGB(rgb);
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
    Point2i res[2];
    std::unique_ptr<RGBSpectrum[]> imgs[2] = {ReadImage(filename[0], &res[0]),
                                              ReadImage(filename[1], &res[1])};
    if (!imgs[0]) {
        fprintf(stderr, "%s: unable to read image", filename[0]);
        return 1;
    }
    if (!imgs[1]) {
        fprintf(stderr, "%s: unable to read image", filename[1]);
        return 1;
    }
    if (res[0] != res[1]) {
        fprintf(stderr,
                "imgtool: image resolutions don't match \"%s\": (%d, %d) "
                "\"%s\": (%d, %d)\n",
                filename[0], res[0].x, res[0].y, filename[1], res[1].x,
                res[1].y);
        return 1;
    }

    std::unique_ptr<RGBSpectrum[]> diffImage;
    if (outfile) diffImage.reset(new RGBSpectrum[res[0].x * res[0].y]);

    double sum[2] = {0., 0.};
    int smallDiff = 0, bigDiff = 0;
    double mse = 0.f;
    for (int i = 0; i < res[0].x * res[0].y; ++i) {
        Float rgb[2][3];
        imgs[0][i].ToRGB(rgb[0]);
        imgs[1][i].ToRGB(rgb[1]);

        Float diffRGB[3];
        for (int c = 0; c < 3; ++c) {
            Float c0 = rgb[0][c], c1 = rgb[1][c];
            diffRGB[c] = std::abs(c0 - c1);

            if (c0 == 0 && c1 == 0) continue;

            sum[0] += c0;
            sum[1] += c1;

            float d = std::abs(c0 - c1) / c0;
            mse += (c0 - c1) * (c0 - c1);
            if (d > .005) ++smallDiff;
            if (d > .05) ++bigDiff;
        }
        if (diffImage) diffImage[i].FromRGB(diffRGB);
    }

    double avg[2] = {sum[0] / (3. * res[0].x * res[0].y),
                     sum[1] / (3. * res[0].x * res[0].y)};
    double avgDelta = (avg[0] - avg[1]) / std::min(avg[0], avg[1]);
    if ((tol == 0. && (bigDiff > 0 || smallDiff > 0)) ||
        (tol > 0. && 100.f * std::abs(avgDelta) > tol)) {
        printf(
            "%s %s\n\tImages differ: %d big (%.2f%%), %d small (%.2f%%)\n"
            "\tavg 1 = %g, avg2 = %g (%f%% delta)\n"
            "\tMSE = %g, RMS = %.3f%%\n",
            filename[0], filename[1], bigDiff,
            100.f * float(bigDiff) / (3 * res[0].x * res[0].y), smallDiff,
            100.f * float(smallDiff) / (3 * res[0].x * res[0].y), avg[0],
            avg[1], 100. * avgDelta, mse / (3. * res[0].x * res[0].y),
            100. * sqrt(mse / (3. * res[0].x * res[0].y)));
        if (outfile) {
            // FIXME: diffImage cast is bad.
            WriteImage(outfile, (Float *)diffImage.get(),
                       Bounds2i(Point2i(0, 0), res[0]), res[0]);
        }
        return 1;
    }

    return 0;
}

int info(int argc, char *argv[]) {
    int err = 0;
    for (int i = 0; i < argc; ++i) {
        Point2i res;
        std::unique_ptr<RGBSpectrum[]> image = ReadImage(argv[i], &res);
        if (!image) {
            fprintf(stderr, "%s: unable to load image.\n", argv[i]);
            err = 1;
            continue;
        }

        printf("%s: resolution %d, %d\n", argv[i], res.x, res.y);
        Float min[3] = {Infinity, Infinity, Infinity};
        Float max[3] = {-Infinity, -Infinity, -Infinity};
        double sum[3] = {0., 0., 0.};
        double logYSum = 0.;
        int nNaN = 0, nInf = 0, nValid = 0;
        for (int i = 0; i < res.x * res.y; ++i) {
            Float y = image[i].y();
            if (!std::isnan(y) && !std::isinf(y))
                logYSum += std::log(Float(1e-6) + y);

            Float rgb[3];
            image[i].ToRGB(rgb);
            for (int c = 0; c < 3; ++c) {
                if (std::isnan(rgb[c]))
                    ++nNaN;
                else if (std::isinf(rgb[c]))
                    ++nInf;
                else {
                    min[c] = std::min(min[c], rgb[c]);
                    max[c] = std::max(max[c], rgb[c]);
                    sum[c] += rgb[c];
                    ++nValid;
                }
            }
        }
        printf("%s: %d infinite pixel components, %d NaN, %d valid.\n", argv[i],
               nInf, nNaN, nValid);
        printf("%s: log average luminance %f\n", argv[i],
               std::exp(logYSum / (res.x * res.y)));
        printf("%s: min rgb (%f, %f, %f)\n", argv[i], min[0], min[1], min[2]);
        printf("%s: max rgb (%f, %f, %f)\n", argv[i], max[0], max[1], max[2]);
        printf("%s: avg rgb (%f, %f, %f)\n", argv[i], sum[0] / nValid,
               sum[1] / nValid, sum[2] / nValid);
    }
    return err;
}

std::unique_ptr<RGBSpectrum[]> bloom(std::unique_ptr<RGBSpectrum[]> image,
                                     const Point2i &res, Float level, int width,
                                     Float scale, int iters) {
    std::vector<std::unique_ptr<RGBSpectrum[]>> blurred;

    // First, threshold the source image
    int nSurvivors = 0;
    std::unique_ptr<RGBSpectrum[]> thresholded(new RGBSpectrum[res.x * res.y]);
    for (int i = 0; i < res.x * res.y; ++i) {
        Float rgb[3];
        image[i].ToRGB(rgb);
        if (rgb[0] > level || rgb[1] > level || rgb[2] > level) {
            ++nSurvivors;
            thresholded[i] = image[i];
        } else
            thresholded[i] = 0.f;
    }
    if (nSurvivors == 0) {
        fprintf(stderr,
                "imgtool: warning: no pixels were above bloom threshold %f\n",
                level);
        return image;
    }
    blurred.push_back(std::move(thresholded));

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

    auto getTexel = [&](const std::unique_ptr<RGBSpectrum[]> &img, Point2i p) {
        // Clamp at boundaries
        if (p.x < 0) p.x = 0;
        if (p.x >= res.x) p.x = res.x - 1;
        if (p.y < 0) p.y = 0;
        if (p.y >= res.y) p.y = res.y - 1;
        return img[p.y * res.x + p.x];
    };

    // Now successively blur the thresholded image.
    std::unique_ptr<RGBSpectrum[]> blurx(new RGBSpectrum[res.x * res.y]);
    for (int iter = 0; iter < iters; ++iter) {
        // Separable blur; first blur in x into blurx
        for (int y = 0; y < res.y; ++y) {
            for (int x = 0; x < res.x; ++x) {
                RGBSpectrum result = 0;
                for (int r = -radius; r <= radius; ++r)
                    result +=
                        wts[r + radius] * getTexel(blurred.back(), {x + r, y});
                blurx[y * res.x + x] = result;
            }
        }

        // Now blur in y from blur x to the result
        std::unique_ptr<RGBSpectrum[]> blury(new RGBSpectrum[res.x * res.y]);
        for (int y = 0; y < res.y; ++y) {
            for (int x = 0; x < res.x; ++x) {
                RGBSpectrum result = 0;
                for (int r = -radius; r <= radius; ++r)
                    result += wts[r + radius] * getTexel(blurx, {x, y + r});
                blury[y * res.x + x] = result;
            }
        }
        blurred.push_back(std::move(blury));
    }

    // Finally, add all of the blurred images, scaled, to the original.
    for (int i = 0; i < res.x * res.y; ++i) {
        RGBSpectrum blurredSum = 0.f;
        // Skip the thresholded image, since it's already present in the
        // original; just add pixels from the blurred ones.
        for (size_t j = 1; j < blurred.size(); ++j) blurredSum += blurred[j][i];
        image[i] += (scale / iters) * blurredSum;
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

    int i;
    auto parseArg = [&]() -> std::pair<std::string, double> {
        const char *ptr = argv[i];
        // Skip over a leading dash or two.
        Assert(*ptr == '-');
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
            else
                usage();
        }
    }

    if (i + 1 >= argc)
        usage("missing second filename for \"convert\"");
    else if (i >= argc)
        usage("missing filenames for \"convert\"");

    const char *inFilename = argv[i], *outFilename = argv[i + 1];
    Point2i res;
    std::unique_ptr<RGBSpectrum[]> image(ReadImage(inFilename, &res));
    if (!image) {
        fprintf(stderr, "%s: unable to read image", inFilename);
        return 1;
    }

    for (int i = 0; i < res.x * res.y; ++i) image[i] *= scale;

    if (bloomLevel < Infinity)
        image = bloom(std::move(image), res, bloomLevel, bloomWidth, bloomScale,
                      bloomIters);

    if (tonemap) {
        for (int i = 0; i < res.x * res.y; ++i) {
            Float y = image[i].y();
            // Reinhard et al. photographic tone mapping operator.
            Float scale = (1 + y / (maxY * maxY)) / (1 + y);
            image[i] *= scale;
        }
    }

    if (repeat > 1) {
        std::unique_ptr<RGBSpectrum[]> rscale(
            new RGBSpectrum[repeat * res.x * repeat * res.y]);
        RGBSpectrum *rsp = rscale.get();
        for (int y = 0; y < repeat * res.y; ++y) {
            int yy = y / repeat;
            for (int x = 0; x < repeat * res.x; ++x) {
                int xx = x / repeat;
                *rsp++ = image[yy * res.x + xx];
            }
        }
        res.x *= repeat;
        res.y *= repeat;
        image = std::move(rscale);
    }

    if (flipy) {
        for (int y = 0; y < res.y / 2; ++y) {
            int yo = res.y - 1 - y;
            for (int x = 0; x < res.x; ++x)
                std::swap(image[y * res.x + x], image[yo * res.x + x]);
        }
    }

    // FIXME: another bad RGBSpectrum -> Float cast.
    WriteImage(outFilename, (Float *)image.get(), Bounds2i(Point2i(0, 0), res),
               res);

    return 0;
}

int main(int argc, char *argv[]) {
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
    else
        usage("unknown command \"%s\"", argv[1]);

    return 0;
}
