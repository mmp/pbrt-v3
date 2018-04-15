#ifndef IISPTPIXEL_H
#define IISPTPIXEL_H

namespace pbrt {

struct IisptPixel {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    int sample_count = 0;
};

}

#endif // IISPTPIXEL_H
