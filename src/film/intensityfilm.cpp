#include "intensityfilm.h"

#include <math.h>

namespace pbrt {

// ============================================================================

void IntensityFilm::set(int x, int y, Float r, Float g, Float b) {
    film->set(x, y, PfmItem(r, g, b));
}

void IntensityFilm::set_camera_coord(
        int cx,
        int cy,
        float r,
        float g,
        float b)
{
    film->set(cx, film->get_height() - 1 - cy, PfmItem(r, g, b));
}

// ============================================================================
void IntensityFilm::write(std::string filename) {
    film->write(filename);
}

// ============================================================================
void IntensityFilm::pbrt_write(std::string filename)
{
    film->pbrt_write_image(filename);
}

// ============================================================================

// Get a pixel
// Uses image coordinates: Y is from top to bottom
PfmItem IntensityFilm::get_image_coord(int x, int y) {
    return film->get(x, y);
}

// Get a pixel
// Uses camera coordinates: Y is from bottom to top
PfmItem IntensityFilm::get_camera_coord(int x, int y) {
    return film->get(x, film->get_height() - 1 - y);
}

// Get a pixel
// Uses camera coordinates: Y is from bottom to top
// Multiplies value by jacobian factor
PfmItem IntensityFilm::get_camera_coord_jacobian(int x, int y) {
    PfmItem pix = get_camera_coord(x, y);
    Float abs_vertical_value = ((Float) y) / film->get_height();
    Float polar_vertical_value = M_PI * abs_vertical_value;
    Float jacobian_factor = sin(polar_vertical_value);
    return pix.scalar_multiply(jacobian_factor);
}

// ============================================================================
// Populate from float array

void IntensityFilm::populate_from_float_array(float* floatarray) {
    film->populate_from_float_array(floatarray);
}

} // namespace pbrt
