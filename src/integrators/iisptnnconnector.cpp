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
// Read image film

// <status> becomes 1 if errors occurred
//                  0 if all OK
std::shared_ptr<IntensityFilm> IisptNnConnector::read_image_film(
        int &status
        )
{
    std::cerr << "iisptnnconnector.cpp: read_image_film" << std::endl;
    int hemisize = PbrtOptions.iisptHemiSize;

    std::shared_ptr<IntensityFilm> film (
                new IntensityFilm(
                    hemisize,
                    hemisize
                    )
                );

    // Get raster
    for (int y = 0; y < hemisize; y++) {
        for (int x = 0; x < hemisize; x++) {
            Float r;
            Float g;
            Float b;

            int code = child_process->read_float32(&r);
            if (code) {
                std::cerr << "iisptnnconnector.cpp: Error when reading a float32" << std::endl;
                status = 1;
                return film;
            }

            code = child_process->read_float32(&g);
            if (code) {
                std::cerr << "iisptnnconnector.cpp: Error when reading a float32" << std::endl;
                status = 1;
                return film;
            }

            code = child_process->read_float32(&b);
            if (code) {
                std::cerr << "iisptnnconnector.cpp: Error when reading a float32" << std::endl;
                status = 1;
                return film;
            }

            film->set(x, y, r, g, b);
        }
    }

    // Check magic characters
    char c0 = child_process->read_char();
    char c1 = child_process->read_char();
    if (c0 == 'x' && c1 == '\n') {
        status = 0;
        return film;
    } else {
        std::cerr << "iisptnnconnector.cpp: magic characters don't match: ["<< c0 <<"] ["<< c1 <<"]" << std::endl;
        status = 1;
        return film;
    }

}

// ============================================================================
// Communicate

// <status> becomes 1 if an error occurred
//                  0 if all ok
std::shared_ptr<IntensityFilm> IisptNnConnector::communicate(
        std::shared_ptr<IntensityFilm> intensity,
        std::shared_ptr<DistanceFilm> distance,
        std::shared_ptr<NormalFilm> normals,
        Float intensity_normalization,
        Float distance_normalization,
        int &status
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
    int st = -1;
    std::shared_ptr<IntensityFilm> output_film = read_image_film(st);
    if (st) {
        std::cerr << "iisptnnconnector.cpp: An error occurred when reading output image" << std::endl;
        status = 1;
        return output_film;
    } else {
        std::cerr << "Obtained an output image. Saving to /tmp/da.pfm" << std::endl;
        output_film->write(std::string("/tmp/da.pfm"));
        std::cerr << "saved." << std::endl;
        status = 0;
        return output_film;
    }
}

}
