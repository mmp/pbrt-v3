#ifndef IISPTFILMMONITOR_H
#define IISPTFILMMONITOR_H

#include <memory>
#include <mutex>
#include <vector>
#include <functional>
#include "film.h"
#include "integrators/iisptfilmtile.h"
#include "integrators/iisptpixel.h"
#include "film/intensityfilm.h"

namespace pbrt {

// ============================================================================
class IisptFilmMonitor
{
private:

    // Fields -----------------------------------------------------------------

    std::recursive_mutex lock;

    std::vector<std::vector<IisptPixel>> pixels;

    Bounds2i film_bounds;

    // Private methods --------------------------------------------------------

    // <func> void function(int film_x, int film_y)
    void execute_on_pixel(
            std::function<void(int, int)> func,
            int x,
            int y
            );

    std::shared_ptr<IntensityFilm> to_intensity_film_priv(
            bool reversed);

public:

    // Constructor ------------------------------------------------------------
    IisptFilmMonitor(
            Bounds2i film_bounds
            );

    // Public methods ---------------------------------------------------------

    Bounds2i get_film_bounds();

    void add_sample(Point2i pt, Spectrum s, double weight);

    std::shared_ptr<IntensityFilm> to_intensity_film();

    std::shared_ptr<IntensityFilm> to_intensity_film_reversed();
};

} // namespace pbrt

#endif // IISPTFILMMONITOR_H
