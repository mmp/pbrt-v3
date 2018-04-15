#include "intensityfilm.h"

#include <math.h>
#include "film/rgbpfmitem.h"

namespace pbrt {

// ============================================================================

void IntensityFilm::set(int x, int y, Float r, Float g, Float b) {
    std::shared_ptr<PfmItem> item (
                new RgbPfmItem(r, g, b)
                );
    film->set(x, y, item);
}

void IntensityFilm::set_camera_coord(
        int cx,
        int cy,
        float r,
        float g,
        float b)
{
    std::shared_ptr<PfmItem> item (
                new RgbPfmItem(r, g, b)
                );
    film->set(cx, film->get_height() - 1 - cy, item);
}

// ============================================================================
void IntensityFilm::write(std::string filename) {
    film->write(filename);
}

// ============================================================================

// Get a pixel
// Uses image coordinates: Y is from top to bottom
std::shared_ptr<PfmItem> IntensityFilm::get_image_coord(int x, int y) {
    return film->get(x, y);
}

// Get a pixel
// Uses camera coordinates: Y is from bottom to top
std::shared_ptr<PfmItem> IntensityFilm::get_camera_coord(int x, int y) {
    return film->get(x, film->get_height() - 1 - y);
}

// Get a pixel
// Uses camera coordinates: Y is from bottom to top
// Multiplies value by jacobian factor
std::shared_ptr<PfmItem> IntensityFilm::get_camera_coord_jacobian(int x, int y) {
    std::shared_ptr<PfmItem> pix = get_camera_coord(x, y);
    Float abs_vertical_value = ((Float) y) / film->get_height();
    Float polar_vertical_value = M_PI * abs_vertical_value;
    Float jacobian_factor = sin(polar_vertical_value);
    return pix->scalar_multiply(jacobian_factor);
}

} // namespace pbrt
