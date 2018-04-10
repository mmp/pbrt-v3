#include <cstdlib>
#include <iostream>
#include "iisptnnconnector.h"

namespace pbrt {

// ============================================================================
// Constructor
IisptNnConnector::IisptNnConnector() {

    // Get environment variable
    char* nn_py_path = getenv("IISPT_STDIO_NET_PY_PATH");
    if (nn_py_path == NULL) {
        std::cerr << "ERROR, environment variable IISPT_STDIO_NET_PY_PATH is not defined. Shutting down..." << std::endl;
        exit(1);
    }
    fprintf(stderr, "Got env variable %s\n", nn_py_path);

    char *const argv[] = {
        "python3",
        "-u",
        nn_py_path,
        NULL
    };

    child_process = std::unique_ptr<ChildProcess>(
                new ChildProcess(
                    std::string("python3"),
                    argv
                    )
                );

}

// ============================================================================
// Pipe image film
void IisptNnConnector::pipe_image_film(std::shared_ptr<ImageFilm> film) {
    if (film == NULL) {
        std::cerr << "Film is null!" << std::endl;
    }
    std::cerr << "Pipe image film start" << std::endl;
    int height = film->get_height();
    std::cerr << "Got the height" << std::endl;
    int width = film->get_width();
    int components = film->get_components();
    std::cerr << "Got the dimensions" << std::endl;

    // The input ImageFilm is assumed to already have the
    // correct Y axis direction

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            std::shared_ptr<PfmItem> pixel_item = film->get(x, y);
            if (components == 1) {
                Float val = pixel_item->get_single_component();
                child_process->write_float32(val);
            } else if (components == 3) {
                Float r;
                Float g;
                Float b;
                pixel_item->get_triple_component(r, g, b);
                child_process->write_float32(r);
                child_process->write_float32(g);
                child_process->write_float32(b);
            } else {
                std::cerr << "iisptnnconnector.cpp: Error, components is neither 1 nor 3. Stopping..." << std::endl;
                exit(1);
            }
        }
    }
}

// ============================================================================
// Communicate

std::shared_ptr<IntensityFilm> IisptNnConnector::communicate(
        std::shared_ptr<IntensityFilm> intensity,
        std::shared_ptr<DistanceFilm> distance,
        std::shared_ptr<NormalFilm> normals,
        Float intensity_normalization,
        Float distance_normalization
        )
{
    // Write rasters
    pipe_image_film(intensity->get_image_film());
    std::cerr << "Piping distance" << std::endl;
    pipe_image_film(distance->get_image_film());
    std::cerr << "Piping normals" << std::endl;
    pipe_image_film(normals->get_image_film());
    // Write normalization
    std::cerr << "Piping normalization values" << std::endl;
    child_process->write_float32(intensity_normalization);
    child_process->write_float32(distance_normalization);

    // Read output from child process
    // TODO: proper implementation
    std::cerr << "Piped image to child process. Follows stdout:" << std::endl;
    while (1) {
        char c = child_process->read_char();
        std::cerr << "["<< c <<"]" << std::endl;;
    }
}

}
