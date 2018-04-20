#include "iisptfilmmonitor.h"

namespace pbrt {

// ============================================================================
IisptFilmMonitor::IisptFilmMonitor(
        Bounds2i film_bounds
        ) {

    this->film_bounds = film_bounds;

    // Initialize sampling density map
    Vector2i film_diagonal = film_bounds.Diagonal();
    for (int y = 0; y <= film_diagonal.y; y++) {
        std::vector<IisptPixel> row;
        for (int x = 0; x <= film_diagonal.x; x++) {
            IisptPixel pix;
            row.push_back(pix);
        }
        pixels.push_back(row);
    }
}

// ============================================================================

void IisptFilmMonitor::add_sample(Point2i pt, Spectrum s, double weight)
{
    lock.lock();

    execute_on_pixel([&](int fx, int fy) {
        IisptPixel pix = (pixels[fy])[fx];
        pix.weight += weight;

        float rgb[3];
        s.ToRGB(rgb);
        pix.r += rgb[0];
        pix.g += rgb[1];
        pix.b += rgb[2];

        (pixels[fy])[fx] = pix;
    }, pt.x, pt.y);

    lock.unlock();
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

std::shared_ptr<IntensityFilm> IisptFilmMonitor::to_intensity_film_priv(
        bool reversed)
{
    Vector2i diagonal = film_bounds.Diagonal();
    int width = diagonal.x + 1;
    int height = diagonal.y + 1;
    std::cerr << "Width and height are ["<< width <<"] ["<< height <<"]" << std::endl;
    std::cerr << "Computed assuming exclusive pMax are ["<< (film_bounds.pMax.x - film_bounds.pMin.x) <<"] ["<< (film_bounds.pMax.y - film_bounds.pMin.y) <<"]" << std::endl;

    std::shared_ptr<IntensityFilm> intensity_film (
                new IntensityFilm(
                    width,
                    height
                    )
                );

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            IisptPixel pix = (pixels[y])[x];
            if (pix.weight > 0.0) {
                double r = pix.r / (pix.weight);
                double g = pix.g / (pix.weight);
                double b = pix.b / (pix.weight);
                if (reversed) {
                    intensity_film->set_camera_coord(
                                x,
                                y,
                                (float) r,
                                (float) g,
                                (float) b
                                );
                } else {
                    intensity_film->set(
                                x,
                                y,
                                (float) r,
                                (float) g,
                                (float) b
                                );
                }
            }
        }
    }

    return intensity_film;
}

// ============================================================================

std::shared_ptr<IntensityFilm> IisptFilmMonitor::to_intensity_film()
{
    lock.lock();

    std::shared_ptr<IntensityFilm> intensity_film =
            to_intensity_film_priv(false);

    lock.unlock();

    return intensity_film;
}

// ============================================================================
// Reverse intensity film
// If the current film is a camera film and output is for viewing

std::shared_ptr<IntensityFilm> IisptFilmMonitor::to_intensity_film_reversed()
{
    lock.lock();

    std::shared_ptr<IntensityFilm> intensity_film =
            to_intensity_film_priv(true);

    lock.unlock();

    return intensity_film;
}

} // namespace pbrt
