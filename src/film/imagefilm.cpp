#include "imagefilm.h"

#include <fstream>

namespace pbrt {

// Utilities ==================================================================

// Writes a float, little endian
static void write_float_value(ofstream &ofs, float val) {
    ofs.write((char *)&val, sizeof(float));
}

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
void ImageFilm::write(std::string filename) {

    ofstream ofs (filename, ios::binary);

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
        std::shared_ptr<std::vector<PfmItem>> row = get_row(y);
        for (int x = 0; x < width; x++) {
            PfmItem pix = row->operator[](x);
            if (num_components == 1) {
                float val = pix.get_single_component();
                write_float_value(ofs, val);
            } else {
                float r;
                float g;
                float b;
                pix.get_triple_component(r, g, b);
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
