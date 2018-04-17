#ifndef IMAGEFILM_H
#define IMAGEFILM_H

#include <memory>
#include <vector>
#include <iostream>
#include <cstdlib>

#include "pfmitem.h"

namespace pbrt {

class ImageFilm
{

private:

    // Fields =================================================================

    int width;
    int height;
    int num_components;

    std::vector<PfmItem> data;

public:

    // Constructor ============================================================
    ImageFilm(
            int width,
            int height,
            int num_components
            ) :
        width(width),
        height(height),
        num_components(num_components)
    {
        if (!(num_components == 1 || num_components == 3)) {
            std::cerr << "imagefilm.h: num_components must be 1 or 3. Actual: "
                      << num_components << std::endl;
            exit(0);
        }

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                data.push_back(PfmItem(0.0, 0.0, 0.0));
            }
        }

    }

    // Set ====================================================================
    void set(int x, int y, PfmItem pixel);

    // Get ====================================================================
    PfmItem get(int x, int y);

    // Write ==================================================================

    // Write to PFM file
    void write(std::string filename);

    // Write using PBRT methods
    void pbrt_write_image(std::string filename);

    // Get Width ==============================================================
    int get_width() {
        return width;
    }

    // Get Height =============================================================
    int get_height() {
        return height;
    }

    // Get Components =========================================================
    int get_components() {
        return num_components;
    }

    // Set all pixels =========================================================
    void set_all(
            PfmItem pix
            );

};

}; // namespace pbrt

#endif // IMAGEFILM_H
