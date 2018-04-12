#ifndef IISPTFILMMONITOR_H
#define IISPTFILMMONITOR_H

#include <memory>
#include <mutex>
#include "film.h"

namespace pbrt {

// ============================================================================
class IisptFilmMonitor
{
private:

    // Fields -----------------------------------------------------------------

    std::recursive_mutex lock;

    // Full film
    Film* film;

public:

    // Constructor ------------------------------------------------------------
    IisptFilmMonitor(Film* film);

    // Public methods ---------------------------------------------------------

    std::unique_ptr<FilmTile> create_film_tile(
            int xc, int yc, float r
            );

    void merge_tile(std::unique_ptr<FilmTile> tile);
};

} // namespace pbrt

#endif // IISPTFILMMONITOR_H
