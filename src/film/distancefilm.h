#ifndef DISTANCEFILM_H
#define DISTANCEFILM_H

#include <memory>
#include <film/imagefilm.h>

namespace pbrt {

class DistanceFilm
{

private:
    int width;
    int height;
    std::shared_ptr<ImageFilm> film;

public:

    virtual ~DistanceFilm() = default;

    // Constructor ============================================================
    DistanceFilm(
            int width,
            int height
            ) :
        width(width),
        height(height)
    {
        film = std::shared_ptr<ImageFilm>(
                new ImageFilm(
                    width,
                    height,
                    1
                )
        );
    }

    // Set pixel ==============================================================
    void set(int x, int y, float val);

    // Write image ============================================================
    void write(std::string filename);

    // Get Image Film =========================================================
    std::shared_ptr<ImageFilm> get_image_film() {
        return film;
    }
};

} // namespace pbrt

#endif // DISTANCEFILM_H
