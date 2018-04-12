#ifndef IISPTFILMTILE_H
#define IISPTFILMTILE_H

#include <vector>
#include "film.h"
#include "geometry.h"

namespace pbrt {

// ============================================================================
class IisptFilmTile
{
private:
    // Fields -----------------------------------------------------------------

    std::shared_ptr<FilmTile> film_tile;

    // List of pixel points to which samples have been added
    std::vector<Point2i> sampled_points;

public:

    // Public methods ---------------------------------------------------------
    IisptFilmTile(std::shared_ptr<FilmTile> film_tile);

    void add_sample(
            Point2f p_film,
            Spectrum L
            );

    std::shared_ptr<FilmTile> get_tile();

    std::vector<Point2i>* get_sampled_points();

};

}

#endif // IISPTFILMTILE_H
