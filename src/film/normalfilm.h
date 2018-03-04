#ifndef NORMALFILM_H
#define NORMALFILM_H

#include <memory>
#include <film/imagefilm.h>
#include "geometry.h"

namespace pbrt {

class NormalFilm
{

private:

    // Fields =================================================================
    std::shared_ptr<ImageFilm> film;

public:

    // Constructor ============================================================
    NormalFilm(
            int width,
            int height
            )
    {
        film = std::shared_ptr<ImageFilm>(
                    new ImageFilm(
                        width, height, 3
                        )
                    );
    }

    // Set pixel ==============================================================
    void set(int x, int y, Normal3f n);

    // Write image ============================================================
    void write(std::string filename);
};

} // namespace pbrt

#endif // NORMALFILM_H
