#include "iisptfilmmonitor.h"

namespace pbrt {

// ============================================================================
IisptFilmMonitor::IisptFilmMonitor(
        Bounds2i film_bounds
        ) {

    this->film_bounds = film_bounds;

    // Initialize sampling density map
    Vector2i film_diagonal = film_bounds.Diagonal();
    for (int y = 0; y < film_diagonal.y; y++) {
        std::vector<IisptPixel> row;
        for (int x = 0; x < film_diagonal.x; x++) {
            IisptPixel pix;
            row.push_back(pix);
        }
        pixels.push_back(row);
    }
}

// ============================================================================

void IisptFilmMonitor::add_sample(Point2i pt, Spectrum s)
{
    execute_on_pixel([&](int fx, int fy) {
        IisptPixel pix = (pixels[fy])[fx];
        pix.sample_count += 1;

        float rgb[3];
        s.ToRGB(rgb);
        pix.r += rgb[0];
        pix.g += rgb[1];
        pix.b += rgb[2];

        (pixels[fy])[fx] = pix;
    }, pt.x, pt.y);
}

// ============================================================================

Bounds2i IisptFilmMonitor::get_film_bounds()
{
    return film_bounds;
}

// ============================================================================

void IisptFilmMonitor::execute_on_pixel(
        std::function<void (int, int)> func,
        int x,
        int y)
{
    int xstart = get_film_bounds().pMin.x;
    int ystart = get_film_bounds().pMin.y;
    int film_x = x - xstart;
    int film_y = y - ystart;
    func(film_x, film_y);
}

// ============================================================================

int IisptFilmMonitor::get_pixel_sampling_density(int x, int y)
{
    int res = 0;
    execute_on_pixel([&](int fx, int fy) {
        IisptPixel pix = (pixels[fy])[fx];
        res = pix.sample_count;
    }, x, y);
    return res;
}

} // namespace pbrt
