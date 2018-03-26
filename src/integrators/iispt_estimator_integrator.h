#ifndef IISPT_ESTIMATOR_INTEGRATOR_H
#define IISPT_ESTIMATOR_INTEGRATOR_H

#include "pbrt.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "integrators/volpath.h"

namespace pbrt {

class IISPTEstimatorIntegrator : public SamplerIntegrator
{
private:

    std::shared_ptr<VolPathIntegrator> volpath;


public:
    IISPTEstimatorIntegrator(
            std::shared_ptr<VolPathIntegrator> volpath,
            const Scene &scene,
            Sampler &sampler
            ) :
        volpath(volpath)
    {
        volpath->Preprocess(scene, sampler);
    }

    Float estimate_intensity(
            Scene &scene,
            Point2i pixel,
            std::shared_ptr<Sampler> sampler
            );

};

} // namespace pbrt

#endif // IISPT_ESTIMATOR_INTEGRATOR_H
