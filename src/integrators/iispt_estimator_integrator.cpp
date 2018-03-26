#include "iispt_estimator_integrator.h"
#include "camera.h"

#include <math.h>

namespace pbrt {

// Utilities ==================================================================

// Logarithmic falloff normalization
Float adjust_logarithmic(Float r, Float g, Float b) {
    Float raw = std::max(r, std::max(g, b));
    return log(raw + 1.0);
}

// Estimate intensity =========================================================
Float IISPTEstimatorIntegrator::estimate_intensity(
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
    return adjust_logarithmic(rgb_values[0], rgb_values[1], rgb_values[2]);
}

}
