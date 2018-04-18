#include "intensityfilm.h"

#include <math.h>
#include <csignal>

namespace pbrt {

// ============================================================================

void IntensityFilm::set(int x, int y, Float r, Float g, Float b) {
    film->set(x, y, PfmItem(r, g, b));
}

void IntensityFilm::set_camera_coord(
        int cx,
        int cy,
        float r,
        float g,
        float b)
{
    film->set(cx, film->get_height() - 1 - cy, PfmItem(r, g, b));
}

// ============================================================================
void IntensityFilm::write(std::string filename) {
    film->write(filename);
}

// ============================================================================
void IntensityFilm::pbrt_write(std::string filename)
{
    film->pbrt_write_image(filename);
}

// ============================================================================

// Get a pixel
// Uses image coordinates: Y is from top to bottom
PfmItem IntensityFilm::get_image_coord(int x, int y) {
    return film->get(x, y);
}

// Get a pixel
// Uses camera coordinates: Y is from bottom to top
PfmItem IntensityFilm::get_camera_coord(int x, int y) {
    return film->get(x, film->get_height() - 1 - y);
}

// Get a pixel
// Uses camera coordinates: Y is from bottom to top
// Multiplies value by jacobian factor
PfmItem IntensityFilm::get_camera_coord_jacobian(int x, int y) {
    PfmItem pix = get_camera_coord(x, y);
    Float abs_vertical_value = ((Float) y) / film->get_height();
    Float polar_vertical_value = M_PI * abs_vertical_value;
    Float jacobian_factor = sin(polar_vertical_value);
    return pix.scalar_multiply(jacobian_factor);
}

PfmItem IntensityFilm::get_image_coord_jacobian(int x, int y) {
    PfmItem pix = get_image_coord(x, y);
    Float abs_vertical_value = ((Float) y) / film->get_height();
    Float polar_vertical_value = M_PI * abs_vertical_value;
    Float jacobian_factor = sin(polar_vertical_value);
    return pix.scalar_multiply(jacobian_factor);
}

// ============================================================================
// Populate from float array

void IntensityFilm::populate_from_float_array(float* floatarray) {
    film->populate_from_float_array(floatarray);
}

// ============================================================================
// Compute CDFs

void IntensityFilm::compute_cdfs()
{
    // Initialize data structures
    pixel_cdfs = std::unique_ptr<std::vector<float>>(
        new std::vector<float>(width * height));
    row_cdfs = std::unique_ptr<std::vector<float>>(
        new std::vector<float>(height));

    // Compute CDFs for each row
    float sum = 0.0;
    for (int y = 0; y < height; y++) {
        sum += compute_row_cdf(y);
        row_cdfs->operator[](y) = sum;
    }
    probability_magnitude = sum;

    std::cerr << "Computed CDFs. Row values are\n";
    for (int y = 0; y < height; y++) {
        float a_val = row_cdfs->operator[](y);
        std::cerr << a_val << std::endl;
    }

    cdf_computed = true;
}

// ============================================================================
// Compute row CDFs
// (private)
float IntensityFilm::compute_row_cdf(
        int y
        )
{
    int idx = y * width;
    float sum = 0.0;
    for (int x = 0; x < width; x++) {
        PfmItem item = get_image_coord_jacobian(x, y);
        sum += item.magnitude();
        pixel_cdfs->operator[](idx) = sum;
        idx++;
    }
    return sum;
}

// ============================================================================
// Importance sampling

PfmItem IntensityFilm::importance_sample(
        float rx, // uniform random float
        float ry,
        int* cx, // sampled image-coordinate pixels
        int* cy,
        float* prob // probability
        )
{
    if (!cdf_computed) {
        std::cerr << "intensityfilm.cpp: Error, calling importance_sample_camera_coord, but this object doesn't have compute_row_cdf yet\n";
        std::raise(SIGKILL);
    }

    // Scale ry by the maximum value in the row_cdfs
    ry *= row_cdfs->operator[](row_cdfs->size() - 1);

    // Iterate to find the row
    int chosen_y = -1;
    for (int y = 0; y < height; y++) {
        float a_row_cdf_val = row_cdfs->operator[](y);
        if (a_row_cdf_val > ry) {
            chosen_y = y - 1;
            break;
        }
    }
    if (chosen_y == -1) {
        chosen_y = height - 1;
    }
    *cy = chosen_y;
    return importance_sample_row(chosen_y, rx, cx, prob);
}

PfmItem IntensityFilm::importance_sample_camera_coord(
        float rx, // uniform random floats
        float ry,
        int* cx, // sampled camera-coordinate pixels
        int* cy,
        float* prob // probability
        )
{
    PfmItem res = importance_sample(rx, ry, cx, cy, prob);
    *cy = height - (*cy) - 1;
    return res;
}

// ============================================================================
PfmItem IntensityFilm::importance_sample_row(
        int chosen_y,
        float rx,
        int* cx,
        float* prob
        )
{
    // Scale rx by the maximum value in the current row
    int rowstart = chosen_y * width;
    rx *= pixel_cdfs->operator[](rowstart + width - 1);

    // Iterate to find the pixel
    int chosen_x = -1;
    for (int x = 0; x < width; x++) {
        int idx = rowstart + x;
        if (pixel_cdfs->operator[](idx) > rx) {
            chosen_x = x - 1;
            break;
        }
    }
    if (chosen_x == -1) {
        chosen_x = width - 1;
    }

    *cx = chosen_x;
    PfmItem res = get_image_coord_jacobian(chosen_x, chosen_y);
    float magnitude = res.magnitude();
    *prob = magnitude / probability_magnitude;
    return res;
}

} // namespace pbrt
