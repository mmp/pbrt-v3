#include "iisptfilmmonitor.h"

namespace pbrt {

// ============================================================================
IisptFilmMonitor::IisptFilmMonitor(Film *film) {
    this->film = film;
}

// ============================================================================
std::unique_ptr<FilmTile> IisptFilmMonitor::create_film_tile(
        int xc,
        int yc,
        float r)
{
    lock.lock();

    Bounds2i sample_bounds = film->GetSampleBounds();
    int rr = std::ceil(r);
    int x0 = std::max(sample_bounds.pMin.x, xc - rr);
    int x1 = std::min(sample_bounds.pMax.x, xc + rr);
    int y0 = std::max(sample_bounds.pMin.y, yc - rr);
    int y1 = std::min(sample_bounds.pMax.y, yc + rr);
    Bounds2i tile_bounds (
                Point2i(x0, y0),
                Point2i(x1, y1)
                );
    std::unique_ptr<FilmTile> res = film->GetFilmTile(tile_bounds);

    lock.unlock();

    return res;
}

// ============================================================================
void IisptFilmMonitor::merge_tile(std::unique_ptr<FilmTile> tile)
{
    lock.lock();

    film->MergeFilmTile(std::move(tile));

    lock.unlock();
}

} // namespace pbrt
