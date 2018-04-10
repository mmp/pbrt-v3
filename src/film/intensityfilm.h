#ifndef INTENSITYFILM_H
#define INTENSITYFILM_H

#include <memory>
#include <film/imagefilm.h>
#include "geometry.h"

namespace pbrt {

class IntensityFilm
{

private:

    std::shared_ptr<ImageFilm> film;

public:

    // Constructor ============================================================
    IntensityFilm(
            int width,
            int height
            )
    {
        film = std::shared_ptr<ImageFilm>(
                    new ImageFilm(
                        width, height, 3
                        )
                    );
    }

    // Set pixel ==============================================================
    void set(int x, int y, Float r, Float g, Float b);

    // Write image ============================================================
    void write(std::string filename);
};

} // namespace pbrt

#endif // INTENSITYFILM_H
