#ifndef IMAGEFILM_H
#define IMAGEFILM_H

#include <memory>
#include <vector>
#include <iostream>
#include <cstdlib>

#include "pfmitem.h"
#include "film/nullpfmitem.h"

namespace pbrt {

class ImageFilm
{

private:

    // Fields =================================================================

    int width;
    int height;
    int num_components;

    std::vector<std::vector<std::shared_ptr<PfmItem>>> rows;

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
            std::vector<std::shared_ptr<PfmItem>> a_row;
            for (int x = 0; x < width; x++) {
                std::shared_ptr<PfmItem> a_null_item (new NullPfmItem());
                a_row.push_back(
                    a_null_item
                );
            }
            rows.push_back(a_row);
        }

        if (rows.size() != height) {
            std::cerr << "imagefilm.h: construct ImageFilm: specified height is "
                      << height << " but actual height is " << rows.size() << std::endl;
            exit(1);
        }
    }

    // Set ====================================================================
    void set(int x, int y, std::shared_ptr<PfmItem> pixel);

    // Get ====================================================================
    std::shared_ptr<PfmItem> get(int x, int y);

    // Write ==================================================================
    // Write to PFM file
    void write(std::string filename);

};

}; // namespace pbrt

#endif // IMAGEFILM_H
