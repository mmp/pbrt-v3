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

    void set_camera_coord(int cx, int cy, float r, float g, float b);

    // Write image ============================================================

    void write(std::string filename);

    void pbrt_write(std::string filename);

    // Get Image Film =========================================================

    std::shared_ptr<ImageFilm> get_image_film() {
        return film;
    }

    // Get pixel ==============================================================

    PfmItem get_image_coord(int x, int y);

    PfmItem get_camera_coord(int x, int y);

    PfmItem get_camera_coord_jacobian(int x, int y);

    // Populate from array ====================================================
    void populate_from_float_array(float* floatarray);
};

} // namespace pbrt

#endif // INTENSITYFILM_H