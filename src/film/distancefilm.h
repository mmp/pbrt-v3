#ifndef DISTANCEFILM_H
#define DISTANCEFILM_H

#include <memory>

namespace pbrt {

class DistanceFilm
{

private:
    int width;
    int height;
    std::shared_ptr<ImageFilm> film;
    std::string filename;

public:

    // Constructor ============================================================
    DistanceFilm(
            int width,
            int height,
            std::string filename
            ) :
        width(width),
        height(height),
        filename(filename)
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
    void write();
};

} // namespace pbrt

#endif // DISTANCEFILM_H
