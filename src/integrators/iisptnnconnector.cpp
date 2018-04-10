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
    // TODO
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
    pipe_image_film(std::dynamic_pointer_cast<ImageFilm>(intensity));
    pipe_image_film(std::dynamic_pointer_cast<ImageFilm>(distance));
    pipe_image_film(std::dynamic_pointer_cast<ImageFilm>(normals));
    // Write normalization
    child_process->write_float32(intensity_normalization);
    child_process->write_float32(distance_normalization);

    // Read output from child process
    // TODO
}

}
