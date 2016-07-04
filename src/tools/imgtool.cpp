//
// imgtool.cpp
//
// Various useful operations on images.
//

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
#include "pbrt.h"
#include "spectrum.h"
#include "imageio.h"

static void usage(const char *msg = nullptr, ...) {
    if (msg) {
        va_list args;
        va_start(args, msg);
        fprintf(stderr, "imgtool: ");
        vfprintf(stderr, msg, args);
        fprintf(stderr, "\n");
    }
    fprintf(stderr, R"(usage: imgtool <command> [options] <filenames...>

commands: convert, diff, info

convert options:
    --flipy
    --scale scale
    --repeatpix [count]
    --tonemap
    --maxluminance [luminance value mapped to white by tonemap]
    --bloomlevel [minimum RGB value for pixel for bloom]
    --bloomscale [bloom image scale]
    --bloomswidth [width of Gaussian used to generate bloom images]
    --bloomiters [filtering iterations to generate bloom images]

diff options:
    --outfile [outfile]
    --difftol [tolerance %%]

)");
    exit(1);
}

int diff(int argc, char *argv[]) {
    float tol = 0.;
    const char *outfile = nullptr;

    int i;
    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "--outfile") || !strcmp(argv[i], "-o")) {
            if (i + 1 == argc)
                usage("missing filename after %s option", argv[i]);
            outfile = argv[++i];
        } else if (!strcmp(argv[i], "--difftol") || !strcmp(argv[i], "-d")) {
            if (i + 1 == argc)
                usage("missing filename after %s option", argv[i]);
            ++i;
            if (!isnumber(argv[i][0]) && argv[i][0] != '.')
                usage("argument after %s doesn't look like a number",
                      argv[i - 1]);
            tol = atof(argv[i]);
        } else
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
    auto parseFloat = [&](const char *flag) -> Float {
        if (i + 1 == argc) usage("missing value after %s flag", flag);
        ++i;
        if (!isnumber(argv[i][0]) && argv[i][0] != '.')
            usage("non-numeric value found after %s", flag);
        Float value = atof(argv[i]);
        if (value == 0) usage("zero value for %s is invalid", flag);
        return value;
    };
    auto parseInt = [&](const char *flag) -> int {
        if (i + 1 == argc) usage("missing value after %s flag", flag);
        ++i;
        if (!isnumber(argv[i][0]))
            usage("non-numeric value found after %s", flag);
        int value = atof(argv[i]);
        if (value == 0) usage("zero value for %s is invalid", flag);
        return value;
    };

    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "-flipy"))
            flipy = !flipy;
        else if (!strcmp(argv[i], "-tonemap"))
            tonemap = !tonemap;
        else if (!strcmp(argv[i], "-maxluminance"))
            maxY = parseFloat("-maxluminance");
        else if (!strcmp(argv[i], "-repeatpix"))
            repeat = parseInt("-repeatpix");
        else if (!strcmp(argv[i], "-scale"))
            scale = parseFloat("-scale");
        else if (!strcmp(argv[i], "-bloomlevel"))
            bloomLevel = parseFloat("-bloomlevel");
        else if (!strcmp(argv[i], "-bloomwidth"))
            bloomWidth = parseInt("-bloomwidth");
        else if (!strcmp(argv[i], "-bloomscale"))
            bloomScale = parseFloat("-bloomscale");
        else if (!strcmp(argv[i], "-bloomiters"))
            bloomIters = parseInt("-bloomiters");
        else
            usage();
    }

    if (i + 1 >= argc)
        usage("missing second filename for \"convert\"");
    else if (i >= argc)
        usage("missing filenames for \"convert\"");

    const char *inFilename = argv[i], *outFilename = argv[i + 1];
    const char *filename[2] = {argv[i], argv[i + 1]};
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

    if (!strcmp(argv[1], "diff"))
        return diff(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "info"))
        return info(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "convert"))
        return convert(argc - 2, argv + 2);
    else
        usage("unknown command \"%s\"", argv[1]);

    return 0;
}
