#ifndef IMAGEFILM_H
#define IMAGEFILM_H

#include <memory>
#include <vector>

#include "pfmitem.h"

namespace pbrt {

class ImageFilm
{

private:

    // Fields =================================================================

    int width;
    int height;

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
            int height
            ) :
        width(width),
        height(height)
    {
        rows = std::shared_ptr<std::vector<std::shared_ptr<std::vector<PfmItem>>>>(
                new std::vector<std::shared_ptr<std::vector<PfmItem>>> (height)
                );

        for (int y = 0; y < height; y++) {
            rows.push_back(
                    std::shared_ptr<std::vector<T>> (
                        new std::vector<T> (width)
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
