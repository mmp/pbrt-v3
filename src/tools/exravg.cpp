
#include <stdio.h>
#include <stdlib.h>
#include "pbrt.h"
#include "spectrum.h"
#include "imageio.h"

static bool ReadEXR(const char *name, float **rgba, int *xRes, int *yRes);

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "usage: exravg [file1.exr] <file2.exr> ...\n");
        return 1;
    }

    float *rgba, *orig_rgba = NULL;
    int xRes = 0, yRes = 0;
    float a = 0;
    int file;

    for (file = 1; file < argc; file++) {
        if (ReadEXR(argv[file], &rgba, &xRes, &yRes)) {
            float *orig_rgba = rgba;
            a = 0;
            for (int i = 0; i < xRes * yRes; ++i) {
                for (int j = 0; j < 3; ++j) a += rgba[j];
                rgba += 4;
            }
            printf("%s: Average value %f\n", argv[file],
                   a / (3.f * xRes * yRes));
            delete[] orig_rgba;
        }
    }
    return 0;
}

static bool ReadEXR(const char *name, float **rgba, int *width, int *height) {
    Point2i res;
    std::unique_ptr<RGBSpectrum[]> image = ReadImage(name, &res);
    if (!image) return false;
    *width = res.x;
    *height = res.y;
    *rgba = new float[4 * *width * *height];
    for (int i = 0; i < *width * *height; ++i) {
        Float rgb[3];
        image[i].ToRGB(rgb);
        for (int c = 0; c < 3; ++c) {
            (*rgba)[4 * i + c] = rgb[c];
        }
        (*rgba)[4 * i + 3] = 1.;
    }
    return true;
}
