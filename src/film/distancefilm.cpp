#include "distancefilm.h"

#include "film/scalarpfmitem.h"
#include "film/rgbpfmitem.h"

namespace pbrt {

// ============================================================================
void DistanceFilm::set(int x, int y, float val) {
    std::unique_ptr<PfmItem> item (new ScalarPfmItem(val));
    film->set(x, y, std::move(item));
}

// ============================================================================
void DistanceFilm::write(std::string filename) {
    film->write(filename);
}

// ============================================================================
void DistanceFilm::clear()
{
    std::unique_ptr<PfmItem> item (
                new RgbPfmItem(0.0, 0.0, 0.0)
                );
    film->set_all(std::move(item));
}

} // namespace pbrt
