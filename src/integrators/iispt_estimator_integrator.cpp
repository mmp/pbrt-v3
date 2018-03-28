#include "iispt_estimator_integrator.h"
#include "camera.h"
#include "scene.h"

#include <math.h>
#include <iostream>

namespace pbrt {

// Utilities ==================================================================

Float tmp_raw_sum = 0.0;
int tmp_raw_n = 0;
Float tmp_raw_max = -1;

Float tmp_logd_sum = 0.0;
int tmp_logd_n = 0;
Float tmp_logd_max = -1;

// Logarithmic falloff normalization
Float adjust_logarithmic(Float r, Float g, Float b) {
    Float raw = std::max(r, std::max(g, b));
    Float logd = log(raw + 1.0);

    return logd;
}

// Square root falloff normalization
Float adjust_sqrt(Float d) {
    Float sd = std::sqrt(d);
    return sd;
}

// Estimate intensity =========================================================
void IISPTEstimatorIntegrator::estimate_intensity(
        const Scene &scene,
        Point2i pixel,
        std::shared_ptr<Sampler> sampler
        )
{
    // Clone sampler with seed
    std::unique_ptr<Sampler> tileSampler = sampler->Clone(pixel.x + pixel.y);

    {
        tileSampler->StartPixel(pixel);
    }

    // Allocate Memory Arena for tile
    MemoryArena arena;

    // Initialize camera sample
    CameraSample cameraSample =
            tileSampler->GetCameraSample(pixel);

    // Generate camera ray for current sample
    RayDifferential ray;
    Float rayWeight = camera->GenerateRayDifferential(cameraSample, &ray);
    // Scaling is fixed at 1 sample per pixel
    ray.ScaleDifferentials(1);

    // Evaluate radiance along camera ray
    Spectrum L (0.0);
    L = volpath->Li(ray, scene, *tileSampler, arena, 0);

    // Check for unexpected radiance values returned
    if (L.HasNaNs()) {
        L = Spectrum(0.0);
    } else if (L.y() < -1e-5) {
        L = Spectrum(0.0);
    } else if (std::isinf(L.y())) {
        L = Spectrum(0.0);
    }

    // Reset arena
    arena.Reset();

    // Postprocess sample
    Float rgb_values [3];
    L.ToRGB(&rgb_values[0]);
    Float logd = adjust_logarithmic(rgb_values[0], rgb_values[1], rgb_values[2]);

    // Record statistic
    if (max_logd_intensity == -1.0 || logd > max_logd_intensity) {
        max_logd_intensity = logd;
    }
}

// Estimate distance ==========================================================
void IISPTEstimatorIntegrator::estimate_distance(
        const Scene &scene,
        Point2i pixel,
        std::shared_ptr<Sampler> sampler
        )
{
    // Clone sampler with seed
    std::unique_ptr<Sampler> tileSampler = sampler->Clone(pixel.x + pixel.y);

    {
        tileSampler->StartPixel(pixel);
    }

    // Allocate Memory Arena for tile
    MemoryArena arena;

    // Initialize camera sample
    CameraSample cameraSample = tileSampler->GetCameraSample(pixel);

    // Generate camera ray for current sample
    RayDifferential ray;
    Float rayWeight = camera->GenerateRayDifferential(cameraSample, &ray);
    // Scaling is fixed at 1 sample per pixel
    ray.ScaleDifferentials(1);

    // Intersect
    SurfaceInteraction isect;
    bool foundIntersection = scene.Intersect(ray, &isect);
    if (foundIntersection) {
        // Record statistic
        Vector3f connecting_vector = isect.p - ray.o;
        float d2 = Dot(connecting_vector, connecting_vector);
        float d = sqrt(d2);
        float d_sqrt = adjust_sqrt(d);
        if (max_sqrt_distance == -1.0 || d_sqrt > max_sqrt_distance) {
            max_sqrt_distance = d_sqrt;
        }
    }
}

// Get max intensity ==========================================================
Float IISPTEstimatorIntegrator::get_max_intensity() {
    return max_logd_intensity;
}

// Get max distance ===========================================================
Float IISPTEstimatorIntegrator::get_max_distance() {
    return max_sqrt_distance;
}

}
