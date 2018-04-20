#ifndef IISPTRENDERRUNNER_H
#define IISPTRENDERRUNNER_H

#include <climits>
#include <unordered_map>

#include "integrators/iispt.h"
#include "integrators/iisptfilmmonitor.h"
#include "integrators/iisptnnconnector.h"
#include "integrators/iisptschedulemonitor.h"
#include "integrators/iispt_d.h"
#include "rng.h"
#include "sampler.h"
#include "camera.h"
#include "cameras/hemispheric.h"
#include "tools/iisptmathutils.h"
#include "tools/iisptrng.h"
#include "tools/iisptpoint2i.h"

namespace pbrt {

// ============================================================================
class IisptRenderRunner
{
private:
    // Fields -----------------------------------------------------------------

    double HEMI_IMPORTANCE = 10.0;
    int HEMISPHERIC_IMPORTANCE_SAMPLES = 16;

    int thread_no;

    bool stop = false;

    Point2i sampler_pixel_counter = Point2i(0, 0);

    // Shared objects

    IISPTIntegrator* iispt_integrator;

    std::shared_ptr<IisptScheduleMonitor> schedule_monitor;

    std::shared_ptr<IisptFilmMonitor> film_monitor;

    std::shared_ptr<Camera> dcamera;

    std::shared_ptr<const Camera> main_camera;

    // Single objects

    std::shared_ptr<IISPTdIntegrator> d_integrator;

    std::unique_ptr<IisptNnConnector> nn_connector;

    std::unique_ptr<IisptRng> rng;

    std::unique_ptr<Sampler> sampler;

    Bounds2i pixel_bounds;

    // Private methods --------------------------------------------------------

    void generate_random_pixel(int* x, int* y);

    bool find_intersection(
            RayDifferential r,
            const Scene &scene,
            MemoryArena &arena,
            SurfaceInteraction* isect_out,
            RayDifferential* ray_out,
            Spectrum* beta_out,
            Spectrum* background_out
            );

    double compute_filter_weight(
            int cx, // Centre sampling pixel
            int cy,
            int fx, // Current filter pixel
            int fy,
            float radius, // Filter radius,
            double* scaling_factor // Scaling factor to obtain a gaussian curve
                                   // which has point X=0, Y=1
            );

    Spectrum sample_hemisphere(
            const Interaction &it,
            std::vector<float> &weights,
            std::vector<HemisphericCamera*> &cameras
            );

    Spectrum path_uniform_sample_one_light(
            Interaction &it,
            const Scene &scene,
            MemoryArena &arena,
            bool handleMedia,
            const Distribution1D* lightDistrib
            );

    Spectrum estimate_direct_lighting(
            Interaction &it,
            const Point2f &uScattering,
            const Light &light,
            const Point2f &uLight,
            const Scene &scene,
            MemoryArena &arena,
            bool handleMedia,
            bool specular
            );

    void sampler_next_pixel();

public:

    // Constructor ------------------------------------------------------------
    IisptRenderRunner(
            IISPTIntegrator* iispt_integrator,
            std::shared_ptr<IisptScheduleMonitor> schedule_monitor,
            std::shared_ptr<IisptFilmMonitor> film_monitor,
            std::shared_ptr<const Camera> main_camera,
            std::shared_ptr<Camera> dcamera,
            std::shared_ptr<Sampler> sampler,
            int thread_no,
            Bounds2i pixel_bounds
            );

    // Public methods ---------------------------------------------------------

    virtual void run(const Scene &scene);

};

}

#endif // IISPTRENDERRUNNER_H
