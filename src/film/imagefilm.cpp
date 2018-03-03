#include "imagefilm.h"

#include <fstream>

namespace pbrt {

// Utilities ==================================================================

// Writes a float, little endian
static void write_float_value(std::ofstream &ofs, float val) {
    ofs.write((char *)&val, sizeof(float));
}


// ============================================================================
void ImageFilm::set(int x, int y, std::shared_ptr<PfmItem> pixel) {

    (rows[y])[x] = pixel;

}

// ============================================================================
std::shared_ptr<PfmItem> ImageFilm::get(int x, int y) {

    return (rows[y])[x];

}

// ============================================================================
void ImageFilm::write(std::string filename) {

    std::ofstream ofs (filename, std::ios::binary);

    // Write type line
    if (num_components == 1) {
        ofs << "Pf\n";
    } else {
        ofs << "PF\n";
    }

    // Write [xres] [yres]
    ofs << width << " " << height << "\n";

    // Write byte order (little endian)
    ofs << "-1.0\n";

    // Write pixels
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            std::shared_ptr<PfmItem> pix = (rows[y])[x];
            if (num_components == 1) {
                float val = pix->get_single_component();
                write_float_value(ofs, val);
            } else {
                float r;
                float g;
                float b;
                pix->get_triple_component(r, g, b);
                write_float_value(ofs, r);
                write_float_value(ofs, g);
                write_float_value(ofs, b);
            }
        }
    }

    // Close
    ofs.close();
    std::cerr << "imagefilm.cpp: Finished writing " << filename << std::endl;

}

// ============================================================================

} // namespace pbrt
