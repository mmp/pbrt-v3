#ifndef IISPTRENDERRUNNER_H
#define IISPTRENDERRUNNER_H

#include "integrators/iispt.h"
#include "integrators/iisptfilmmonitor.h"
#include "integrators/iisptnnconnector.h"
#include "integrators/iisptschedulemonitor.h"
#include "integrators/iispt_d.h"
#include "rng.h"
#include "sampler.h"
#include "camera.h"

namespace pbrt {

// ============================================================================
class IisptRenderRunner
{
private:
    // Fields -----------------------------------------------------------------

    int thread_no;

    std::shared_ptr<Sampler> sampler;

    std::shared_ptr<IISPTIntegrator> iispt_integrator;

    std::shared_ptr<IisptScheduleMonitor> schedule_monitor;

    std::shared_ptr<IisptFilmMonitor> film_monitor;

    std::shared_ptr<Camera> dcamera;

    std::shared_ptr<const Camera> main_camera;

    std::shared_ptr<IISPTdIntegrator> d_integrator;

    std::shared_ptr<IisptNnConnector> nn_connector;

    std::shared_ptr<RNG> rng;

    Bounds2i pixel_bounds;

    // Private methods --------------------------------------------------------

    void generate_random_pixel(int* x, int* y);

public:

    // Constructor ------------------------------------------------------------
    IisptRenderRunner(
            std::shared_ptr<IISPTIntegrator> iispt_integrator,
            std::shared_ptr<IisptScheduleMonitor> schedule_monitor,
            std::shared_ptr<IisptFilmMonitor> film_monitor,
            std::shared_ptr<const Camera> main_camera,
            std::shared_ptr<Camera> dcamera,
            std::shared_ptr<Sampler> sampler,
            int thread_no,
            Bounds2i pixel_bounds
            );

    // Public methods ---------------------------------------------------------
    virtual void run();
};

}

#endif // IISPTRENDERRUNNER_H
