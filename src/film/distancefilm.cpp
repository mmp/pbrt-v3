#include "distancefilm.h"

namespace pbrt {

// ============================================================================
void DistanceFilm::set(int x, int y, float val) {
    film->set(x, y, PfmItem(val));
}

// ============================================================================
void DistanceFilm::write(std::string filename) {
    film->write(filename);
}

// ============================================================================
void DistanceFilm::clear()
{
    film->set_all(PfmItem(0.0, 0.0, 0.0));
}

} // namespace pbrt
