#include "iisptfilmtile.h"

namespace pbrt {

// ============================================================================
IisptFilmTile::IisptFilmTile(std::shared_ptr<FilmTile> film_tile)
{
    this->film_tile = film_tile;
}

// ============================================================================
// Add sample
void IisptFilmTile::add_sample(
        Point2f p_film,
        Spectrum L
        )
{
    // Add to film
    film_tile->AddSample(p_film, L, 1.0);

    // Compute centre point
    Point2f p_film_discrete = p_film - Vector2f(0.5f, 0.5f);
    Point2i p_film_int = Point2i(
                std::round(p_film_discrete.x),
                std::round(p_film_discrete.y)
                );

    // Record sampled centre point
    sampled_points.push_back(p_film_int);
}

// ============================================================================
// Get film
std::shared_ptr<FilmTile> IisptFilmTile::get_tile()
{
    return this->film_tile;
}

// ============================================================================
// Get sampled points
std::vector<Point2i>* IisptFilmTile::get_sampled_points()
{
    return &sampled_points;
}

}
