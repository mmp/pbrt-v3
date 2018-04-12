#ifndef IISPTRENDERRUNNER_H
#define IISPTRENDERRUNNER_H

#include "integrators/iispt.h"
#include "integrators/iisptfilmmonitor.h"
#include "integrators/iisptnnconnector.h"
#include "integrators/iisptschedulemonitor.h"
#include "integrators/iispt_d.h"
#include "rng.h"

namespace pbrt {

// ============================================================================
class IisptRenderRunner
{
private:
    // Fields -----------------------------------------------------------------

    std::shared_ptr<IISPTIntegrator> iispt_integrator;

    std::shared_ptr<IisptScheduleMonitor> schedule_monitor;

    std::shared_ptr<IisptFilmMonitor> film_monitor;

    std::unique_ptr<IISPTdIntegrator> d_integrator;

    std::unique_ptr<IisptNnConnector> nn_connector;

    std::unique_ptr<RNG> rng;

    // Private methods --------------------------------------------------------

    void generate_random_pixel(int* x, int* y);

public:

    // Constructor ------------------------------------------------------------
    IisptRenderRunner(
            std::shared_ptr<IISPTIntegrator> iispt_integrator,
            std::shared_ptr<IisptScheduleMonitor> schedule_monitor,
            std::shared_ptr<IisptFilmMonitor> film_monitor
            );

    // Public methods ---------------------------------------------------------
    virtual void run();
};

}

#endif // IISPTRENDERRUNNER_H
