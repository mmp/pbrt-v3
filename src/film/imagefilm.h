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

    // Each row is a std::shared_ptr<std::vector<T>>
    std::shared_ptr<
        std::vector<
            std::shared_ptr<
                std::vector<
                    PfmItem
                >
            >
        >
    > rows;

    // Get row ================================================================
    std::shared_ptr<std::vector<PfmItem>> get_row(int y);

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

        rows = std::shared_ptr<std::vector<std::shared_ptr<std::vector<PfmItem>>>>(
                new std::vector<std::shared_ptr<std::vector<PfmItem>>> (height)
                );

        for (int y = 0; y < height; y++) {
            rows.push_back(
                    std::shared_ptr<std::vector<PfmItem>> (
                        new std::vector<PfmItem> (width, NullPfmItem())
                    )
            );
        }
    }

    // Set ====================================================================
    void set(int x, int y, PfmItem pixel);

    // Get ====================================================================
    PfmItem get(int x, int y);

    // Write ==================================================================
    // Write to PFM file
    void write(std::string filename);

};

}; // namespace pbrt

#endif // IMAGEFILM_H
