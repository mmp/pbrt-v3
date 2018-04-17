#include "normalfilm.h"

namespace pbrt {

// ============================================================================
void NormalFilm::set(int x, int y, Normal3f n) {
    film->set(x, y, PfmItem(n.x, n.y, n.z));
}

// ============================================================================
void NormalFilm::write(std::string filename) {
    film->write(filename);
}

// ============================================================================
void NormalFilm::clear()
{
    film->set_all(PfmItem(0.0, 0.0, 0.0));
}

}// namespace pbrt
