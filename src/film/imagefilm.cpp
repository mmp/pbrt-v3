#include "imagefilm.h"

namespace pbrt {


// ============================================================================
std::shared_ptr<std::vector<PfmItem>> get_row(int y) {

    return rows->operator[](y);

}

// ============================================================================
void ImageFilm::set(int x, int y, PfmItem pixel) {

    // Get the row
    std::shared_ptr<std::vector<PfmItem>> row = get_row(y);

    // Set the element
    row->operator[](x) = pixel;

}

// ============================================================================
PfmItem ImageFilm::get(int x, int y) {

    // Get the row
    std::shared_ptr<std::vector<PfmItem>> row = get_row(y);

    // Get the element
    return row->operator[](x);

}

// ============================================================================

} // namespace pbrt
