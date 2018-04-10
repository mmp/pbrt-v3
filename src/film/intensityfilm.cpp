#include "intensityfilm.h"

#include "film/rgbpfmitem.h"

namespace pbrt {

// ============================================================================
void IntensityFilm::set(int x, int y, Float r, Float g, Float b) {
    std::shared_ptr<PfmItem> item (
                new RgbPfmItem(r, g, b)
                );
    film->set(x, y, item);
}

// ============================================================================
void IntensityFilm::write(std::string filename) {
    film->write(filename);
}

} // namespace pbrt
