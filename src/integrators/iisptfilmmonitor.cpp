#include "iisptfilmmonitor.h"

namespace pbrt {

// ============================================================================
IisptFilmMonitor::IisptFilmMonitor(Film *film) {

    this->film = film;

    // Initialize sampling density map
    Vector2i film_diagonal = film->GetSampleBounds().Diagonal();
    for (int y = 0; y < film_diagonal.y; y++) {
        std::vector<int> row;
        for (int x = 0; x < film_diagonal.x; x++) {
            row.push_back(0);
        }
        sampling_density.push_back(row);
    }
}

// ============================================================================
std::shared_ptr<IisptFilmTile> IisptFilmMonitor::create_film_tile(
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
    std::shared_ptr<FilmTile> film_tile = film->GetFilmTileShared(tile_bounds);
    std::shared_ptr<IisptFilmTile> res (
                IisptFilmTile(film_tile)
                );

    lock.unlock();

    return res;
}

// ============================================================================
void IisptFilmMonitor::merge_tile(std::shared_ptr<IisptFilmTile> tile)
{
    lock.lock();

    film->MergeFilmTile(tile->get_tile());

    // Record sampled points to density map
    std::vector<Point2i>* sampled_points = tile->get_sampled_points();
    for (int i = 0; i < sampled_points->size(); i++) {
        Point2i a_point = sampled_points->operator[](i);
        record_density_point(a_point);
    }

    lock.unlock();
}

// ============================================================================

void IisptFilmMonitor::record_density_point(Point2i pt)
{
    int xstart = get_film_bounds().pMin.x;
    int ystart = get_film_bounds().pMin.y;
    int xeffective = pt.x - xstart;
    int yeffective = pt.y - ystart;
    (sampling_density[yeffective])[xeffective] += 1;
}

// ============================================================================

Bounds2i IisptFilmMonitor::get_film_bounds()
{
    return film->GetSampleBounds();
}

} // namespace pbrt
