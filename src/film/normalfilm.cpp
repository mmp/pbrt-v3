#include "normalfilm.h"

#include "film/rgbpfmitem.h"

namespace pbrt {

// ============================================================================
void NormalFilm::set(int x, int y, Normal3f n) {
    std::shared_ptr<PfmItem> item (new RgbPfmItem(n.x, n.y, n.z));
    film->set(x, y, item);
}

// ============================================================================
void NormalFilm::write(std::string filename) {
    film->write(filename);
}

}// namespace pbrt
