//
// hdrtoldr.cpp
//
// Convert high dynamic range image formats to low dynamic range.
//

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "pbrt.h"
#include "spectrum.h"
#include "imageio.h"

static void usage() {
    fprintf(stderr, "usage: hdrtoldr [options] <input.exr> <output.png>\n");
    fprintf(stderr, "Supported options:\n");
    fprintf(stderr, "\t-scale scale\n");
    fprintf(stderr, "\t-repeatpix [count]\n");
    exit(1);
}

int main(int argc, char *argv[]) {
    float scale = 1.f;
    int repeat = 1;
    float rp = 1;

    int argNum = 1;
    while (argNum < argc && argv[argNum][0] == '-') {
#define ARG(name, var)                          \
    else if (!strcmp(argv[argNum], "-" name)) { \
        if (argNum + 1 == argc) usage();        \
        var = atof(argv[argNum + 1]);           \
        ++argNum;                               \
    }
        if (false) {
        }
        ARG("repeatpix", rp)
        ARG("scale", scale)
        else usage();
        ++argNum;
    }
    if (argNum + 2 > argc) usage();
    repeat = int(rp);

    char *inFile = argv[argNum], *outFile = argv[argNum + 1];

    Point2i res;
    std::unique_ptr<RGBSpectrum[]> image = ReadImage(inFile, &res);
    if (!image) return 1;

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

    for (int i = 0; i < res.x * res.y; ++i) image[i] *= scale;

    WriteImage(outFile, (Float *)image.get(), Bounds2i(Point2i(0, 0), res),
               res);

    return 0;
}
