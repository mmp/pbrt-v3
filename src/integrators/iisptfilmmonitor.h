#ifndef IISPTFILMMONITOR_H
#define IISPTFILMMONITOR_H

#include <memory>
#include <mutex>
#include <vector>
#include <functional>
#include "film.h"
#include "integrators/iisptfilmtile.h"
#include "integrators/iisptpixel.h"

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

public:

    // Constructor ------------------------------------------------------------
    IisptFilmMonitor(
            Bounds2i film_bounds
            );

    // Public methods ---------------------------------------------------------

    Bounds2i get_film_bounds();

    int get_pixel_sampling_density(int x, int y);

    void add_sample(Point2i pt, Spectrum s);
};

} // namespace pbrt

#endif // IISPTFILMMONITOR_H
