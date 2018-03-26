#include "iispt_estimator_integrator.h"

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
        Scene &scene,
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
    L = Li(ray, scene, *tileSampler, arena);

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
    return adjust_logarithmic(L.c[0], L.c[1], L.c[2]);
}

}
