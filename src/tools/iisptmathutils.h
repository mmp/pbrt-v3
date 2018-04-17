#ifndef IISPTMATHUTILS_H
#define IISPTMATHUTILS_H

#include <cmath>

namespace pbrt {

namespace iispt {

static double gauss(
        double sigma,
        double x
        )
{
    double gauss_left =
            (1.0)
            /
            (
                std::sqrt(
                    2.0 * M_PI * sigma * sigma
                    )
            );
    double gauss_right =
            std::exp(
                -1.0 * (
                    (
                        x * x
                        )
                    /
                    (
                        2 * sigma * sigma
                        )
                    )
                );
    return gauss_left * gauss_right;
}

} // namespace iispt

} // namespace pbrt

#endif // IISPTMATHUTILS_H
