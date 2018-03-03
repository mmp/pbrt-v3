#ifndef DISTANCEFILM_H
#define DISTANCEFILM_H

#include <memory>

namespace pbrt {

class DistanceFilm
{

private:
    int width;
    int height;
    std::shared_ptr<Film> film;

public:

    // Constructor ============================================================
    DistanceFilm(
            int width,
            int height,
            std::string filename
            ) :
        width(width),
        height(height) {

        const Point2i resolution (width, height);
        const Bounds2f cropWindow (Point2f(0., 0.), Point2f(1., 1.));
        std::unique_ptr<Filter> filter (new GaussianFilter(Vector2f(2.f, 2.f), 2.f));
        Float scale = 1.;
        Float diagonal = 35.;
        Float maxSampleLuminance = Infinity;
        film = std::shared_ptr<Film>(
                    new Film(
                        resolution,
                        cropWindow,
                        std::move(filter),
                        diagonal,
                        filename,
                        scale,
                        maxSampleLuminance
                        )
                    );
    }

    // Set pixel ==============================================================

    // Get pixel ==============================================================

    // Write image ============================================================
};

} // namespace pbrt

#endif // DISTANCEFILM_H
