#include "imagefilm.h"

#include "geometry.h"
#include "imageio.h"
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
void ImageFilm::pbrt_write_image(std::string filename) {

    Bounds2i cropped_pixel_bounds (
                Point2i(0, 0),
                Point2i(width, height)
                );

    Point2i full_resolution (
                width, height
                );

    std::unique_ptr<Float[]> rgb (
                new Float[get_components() * cropped_pixel_bounds.Area()]
                );

    // Populate the array
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            std::shared_ptr<PfmItem> item = get(x, y);
            if (num_components == 1) {
                Float it = item->get_single_component();
                rgb[y*width + x] = it;
            } else {
                Float r, g, b;
                item->get_triple_component(r, g, b);
                int pix_index = y * width + x;
                int array_index = 3 * pix_index;
                rgb[array_index + 0] = r;
                rgb[array_index + 1] = g;
                rgb[array_index + 2] = b;
            }
        }
    }

    pbrt::WriteImage(filename, &rgb[0], cropped_pixel_bounds, full_resolution);

}

// ============================================================================

void ImageFilm::set_all(
        std::shared_ptr<PfmItem> pix
        )
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            set(x, y, pix);
        }
    }
}

} // namespace pbrt
