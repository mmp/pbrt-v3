#ifndef INTENSITYFILM_H
#define INTENSITYFILM_H

#include <memory>
#include <film/imagefilm.h>
#include "geometry.h"

namespace pbrt {

class IntensityFilm
{

private:

    // Fields =================================================================

    std::shared_ptr<ImageFilm> film;

    int width;
    int height;

    // CDF sampling data
    // All CDF data is in image coordinates
    bool cdf_computed = false;
    std::unique_ptr<std::vector<float>> pixel_cdfs;
    std::unique_ptr<std::vector<float>> row_cdfs;
    float probability_magnitude;

    // Private methods ========================================================
    float compute_row_cdf(
            int y
            );

    PfmItem importance_sample_row(
            int chosen_y,
            float rx,
            int* cx,
            float* prob
            )

public:

    virtual ~IntensityFilm() = default;

    // Constructor ============================================================

    IntensityFilm(
            int width,
            int height
            )
    {
        this->width = width;
        this->height = height;
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

    PfmItem get_image_coord_jacobian(int x, int y);

    // Populate from array ====================================================
    void populate_from_float_array(float* floatarray);

    // Compute CDFs ===========================================================
    void compute_cdfs();

    // Importance sampling ====================================================
    PfmItem importance_sample(
            float rx, // uniform random float
            float ry,
            int* cx, // sampled image-coordinate pixels
            int* cy,
            float* prob // probability
            );
};

} // namespace pbrt

#endif // INTENSITYFILM_H
