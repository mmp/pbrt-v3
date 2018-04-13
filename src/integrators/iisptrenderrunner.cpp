#include "iisptrenderrunner.h"

namespace pbrt {

// ============================================================================
IisptRenderRunner::IisptRenderRunner(
        std::shared_ptr<IISPTIntegrator> iispt_integrator,
        std::shared_ptr<IisptScheduleMonitor> schedule_monitor,
        std::shared_ptr<IisptFilmMonitor> film_monitor,
        std::shared_ptr<const Camera> main_camera,
        std::shared_ptr<Camera> dcamera,
        std::shared_ptr<Sampler> sampler,
        int thread_no,
        Bounds2i pixel_bounds
        )
{
    this->iispt_integrator = iispt_integrator;

    this->schedule_monitor = schedule_monitor;

    this->film_monitor = film_monitor;

    this->d_integrator = CreateIISPTdIntegrator(dcamera);

    this->nn_connector = std::shared_ptr<IisptNnConnector>(
                new IisptNnConnector()
                );

    // TODO remove fixed seed
    this->rng = std::shared_ptr<RNG>(
                new RNG(thread_no)
                );

    this->sampler = std::shared_ptr<Sampler>(
                sampler->Clone(thread_no)
                );

    this->dcamera = dcamera;

    this->thread_no = thread_no;

    this->pixel_bounds = pixel_bounds;

    this->main_camera = main_camera;
}

// ============================================================================
void IisptRenderRunner::run(const Scene &scene)
{
    while (1) {

        // --------------------------------------------------------------------
        //    * Obtain current __radius__ from the __ScheduleMonitor__. The ScheduleMonitor updates its internal count automatically
        float radius = schedule_monitor->get_current_radius();

        // --------------------------------------------------------------------
        //    * Use the __RNG__ to generate 2 random pixel samples. Look up the density of the samples and select the one that has lower density
        int pix1x;
        int pix1y;
        generate_random_pixel(&pix1x, &pix1y);

        int pix2x;
        int pix2y;
        generate_random_pixel(&pix2x, &pix2y);

        int pix1d = film_monitor->get_pixel_sampling_density(pix1x, pix1y);
        int pix2d = film_monitor->get_pixel_sampling_density(pix2x, pix2y);

        int x;
        int y;
        if (pix1d < pix2d) {
            x = pix1x;
            y = pix1y;
        } else {
            x = pix2x;
            y = pix2y;
        }
        Point2i pixel = Point2i(x, y);


        // --------------------------------------------------------------------
        //    * Obtain camera ray and shoot into scene. If no __intersection__ is found, evaluate infinite lights

        sampler->StartPixel(pixel);
        if (!InsideExclusive(pixel, pixel_bounds)) {
            continue;
        }

        CameraSample camera_sample =
                sampler->GetCameraSample(pixel);

        RayDifferential r;
        Float ray_weight =
                main_camera->GenerateRayDifferential(
                    camera_sample,
                    &r
                    );
        r.ScaleDifferentials(1.0); // Not scaling based on samples per
                                     // pixel here

        for (int bounces = 0;; bounces++) {
            std::cerr << "iisptrenderrunner.cpp::run. Pixel ["<< pixel <<"] Bounce " << bounces << std::endl;

            RayDifferential ray (r);

            SurfaceInteraction isect;
            bool found_intersection = scene.Intersect(ray, &isect);

            if (!found_intersection) {
                // No intersection
            }
        }

        // TODO
        // Need a function find_intersection
        // Takes the scene and the camera ray
        // Returns an intersection object (if any)
        // and a beta multiplier value


        // --------------------------------------------------------------------
        //    * Create __auxCamera__ and use the __dIntegrator__ to render a view

        // --------------------------------------------------------------------
        //    * Use the __NnConnector__ to obtain the predicted intensity

        // --------------------------------------------------------------------
        //    * Set the predicted intensity map on the __auxCamera__

        // --------------------------------------------------------------------
        //    * For all pixels within __radius__ and whose intersection and materials are compatible with the original intersection, evaluate __Li__ and update the filmTile

        // --------------------------------------------------------------------
        //    * Send the filmTile to the __filmMonitor__

    }
}

// ============================================================================

void IisptRenderRunner::generate_random_pixel(int *x, int *y)
{
    Bounds2i bounds = film_monitor->get_film_bounds();
    int xmin = bounds.pMin.x;
    int xmax = bounds.pMax.x;
    int ymin = bounds.pMin.y;
    int ymax = bounds.pMax.y;
    int width = xmax - xmin;
    int height = ymax - ymin;
    int randx = rng->UniformUInt32(width);
    int randy = rng->UniformUInt32(height);
    *x = randx + xmin;
    *y = randy + ymin;
}

// ============================================================================

// If intersection is found, a SurfaceInteraction is returned with a Beta value
// Otherwise, a background color is returned
// If beta_out is 0, then the current pixel always is black and doesn't need
// to be further evaluated
bool IisptRenderRunner::find_intersection(
        RayDifferential r,
        const Scene &scene,
        MemoryArena &arena,
        SurfaceInteraction* isect_out,
        Spectrum* beta_out,
        Spectrum* background_out
        )
{
    Spectrum beta (1.0);
    RayDifferential ray (r);

    for (bounces = 0; bounces < 24; ++bounces) {

        // Compute intersection
        SurfaceInteraction isect;
        bool found_intersection = scene.Intersect(ray, &isect);

        // If no intersection, returned beta-scaled background radiance
        if (!found_intersection) {
            Spectrum L (0.0);
            for (const auto &light : scene.infiniteLights) {
                L += beta * light->Le(ray);
                *background_out = L;
                return false;
            }
        }

        // Compute scattering functions
        isect.ComputeScatteringFunctions(ray, arena, true);
        if (!isect.bsdf) {
            // If BSDF is null, skip this intersection
            ray = isect.SpawnRay(ray.d);
            continue;
        }

        // Skip light sampling

        // Sample BSDF for new path direction
        Vector3f wo = -ray.d;
        Vector3f wi;
        Float pdf;
        BxDFType flags;
        Spectrum f =
                isect.bsdf->Sample_f(
                    wo,
                    &wi,
                    sampler->Get2D(),
                    &pdf,
                    BSDF_ALL,
                    &flags
                    );
        // If BSDF is black or contribution is null,
        // return a 0 beta
        if (f.IsBlack() || pdf == 0.f) {
            *beta_out = L(0.0);
            return true;
        }
        // Check for specular bounce
        bool specular_bounce = (flags & BSDF_SPECULAR) != 0;
        if (!specular_bounce) {
            // The current bounce is not specular, so we stop here
            // and let IISPT proceed from the current point
            // No need to update Beta here
            *isect_out = isect;
            *beta_out = beta;
            return true;
        }
        // Follow the specular bounce
        // Update beta value
        beta *= f * AbsDot(wi, isect.shading.n) / pdf;
        // Check for zero beta
        if (beta.y() < 0.f || isNaN(beta.y())) {
            *beta_out = L(0.0);
            return true;
        }
        // Spawn the new ray
        ray = isect.SpawnRay(wi);

        // Skip subsurface scattering

        // Skip RR termination

    }

    // Max depth reached, return 0 beta
    *beta = L(0.0);
    return true;
}

}
