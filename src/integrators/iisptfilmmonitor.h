#ifndef IISPTFILMMONITOR_H
#define IISPTFILMMONITOR_H

#include <memory>
#include <mutex>
#include <vector>
#include "film.h"
#include "integrators/iisptfilmtile.h"

namespace pbrt {

// ============================================================================
class IisptFilmMonitor
{
private:

    // Fields -----------------------------------------------------------------

    std::recursive_mutex lock;

    // Full film
    Film* film;

    std::vector<std::vector<int>> sampling_density;

    // Private methods --------------------------------------------------------

    void record_density_point(Point2i pt);

    void absolute_to_density_array_coord(
            Point2i pt,
            int* x,
            int* y
            );

public:

    // Constructor ------------------------------------------------------------
    IisptFilmMonitor(Film* film);

    // Public methods ---------------------------------------------------------

    std::shared_ptr<IisptFilmTile> create_film_tile(
            int xc, int yc, float r
            );

    void merge_tile(std::shared_ptr<IisptFilmTile> tile);

    Bounds2i get_film_bounds();

    int get_pixel_sampling_density(int x, int y);
};

} // namespace pbrt

#endif // IISPTFILMMONITOR_H
