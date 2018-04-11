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

    virtual ~IntensityFilm() = default;

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

    // Get Image Film =========================================================

    std::shared_ptr<ImageFilm> get_image_film() {
        return film;
    }

    // Get pixel ==============================================================

    std::shared_ptr<PfmItem> get_image_coord(int x, int y);

    std::shared_ptr<PfmItem> get_camera_coord(int x, int y);

    std::shared_ptr<PfmItem> get_camera_coord_jacobian(int x, int y);
};

} // namespace pbrt

#endif // INTENSITYFILM_H
