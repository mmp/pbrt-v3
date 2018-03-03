#include "distancefilm.h"

#include "film/scalarpfmitem.h"

namespace pbrt {

// ============================================================================
void DistanceFilm::set(int x, int y, float val) {
    ScalarPfmItem item (val);
    film->set(x, y, item);
}

// ============================================================================
void DistanceFilm::write() {
    film->write(filename);
}

} // namespace pbrt
