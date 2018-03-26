#ifndef IISPT_ESTIMATOR_INTEGRATOR_H
#define IISPT_ESTIMATOR_INTEGRATOR_H

#include "pbrt.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "integrators/volpath.h"

namespace pbrt {

class IISPTEstimatorIntegrator
{
private:

    std::shared_ptr<VolPathIntegrator> volpath;
    std::shared_ptr<const Camera> camera;

    Float max_logd_intensity = -1.0;
    Float max_sqrt_distance = -1.0;

public:
    IISPTEstimatorIntegrator(
            std::shared_ptr<VolPathIntegrator> volpath,
            std::shared_ptr<const Camera> camera,
            const Scene &scene,
            std::shared_ptr<Sampler> sampler
            ) :
        volpath(volpath),
        camera(camera)
    {
        volpath->Preprocess(scene, *sampler.get());
    }

    void estimate_intensity(
            const Scene &scene,
            Point2i pixel,
            std::shared_ptr<Sampler> sampler
            );

    void estimate_distance(
            const Scene &scene,
            Point2i pixel,
            std::shared_ptr<Sampler> sampler
            );

    Float get_max_intensity();

    Float get_max_distance();

};

} // namespace pbrt

#endif // IISPT_ESTIMATOR_INTEGRATOR_H
