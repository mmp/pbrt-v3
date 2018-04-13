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
    // Preprocess is called on run()

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
void IisptRenderRunner::run(const Scene &scene, MemoryArena &arena)
{
    this->d_integrator->Preprocess(scene);

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

        SurfaceInteraction isect;
        Spectrum beta;
        Spectrum background;
        RayDifferential ray;

        // Find intersection point
        bool intersection_found = find_intersection(
                    r,
                    scene,
                    arena,
                    &isect,
                    &ray,
                    &beta,
                    &background
                    );

        if (!intersection_found) {
            // Record background light
            film_monitor->add_sample(
                        pixel,
                        background
                        );
            continue;
        } else if (intersection_found && beta <= 0.0) {
            // Intersection found but black pixel
            film_monitor->add_sample(
                        pixel,
                        L(0.0)
                        );
            continue;
        }

        // The intersection object is isect

        // --------------------------------------------------------------------
        //    * Create __auxCamera__ and use the __dIntegrator__ to render a view

        // Invert normal if surface normal points inwards
        Normal3f surface_normal = isect.n;
        Vector3f sf_norm_vec = Vector3f(isect.n.x, isect.n.y, isect.n.z);
        Vector3f ray_vec = Vector3f(ray.d.x, ray.d.y, ray.d.z);
        if (Dot(sf_norm_vec, ray_vec) > 0.0) {
            surface_normal = Normal3f(
                        -isect.n.x,
                        -isect.n.y,
                        -isect.n.z
                        );
        }

        // aux_ray is centered at the intersection point
        // points towards the intersection surface normal
        Ray aux_ray = isect.SpawnRay(Vector3f(surface_normal));

        // Create aux camera
        std::shared_ptr<HemisphericCamera> aux_camera (
                    CreateHemisphericCamera(
                        PbrtOptions.iisptHemiSize,
                        PbrtOptions.iisptHemiSize,
                        dcamera->medium,
                        aux_ray.o,
                        Point3f(aux_ray.d.x, aux_ray.d.y, aux_ray.d.z),
                        pixel,
                        std::string("/tmp/null")
                        )
                    );

        // Run dintegrator render
        d_integrator->RenderView(scene, aux_camera);

        // --------------------------------------------------------------------
        //    * Use the __NnConnector__ to obtain the predicted intensity

        // Obtain intensity, normals, distance maps

        std::shared_ptr<IntensityFilm> aux_intensity =
                d_integrator->get_intensity_film(aux_camera);

        std::shared_ptr<NormalFilm> aux_normals =
                d_integrator->get_normal_film();

        std::shared_ptr<DistanceFilm> aux_distance =
                d_integrator->get_distance_film();

        // Use NN Connector

        int communicate_status = -1;
        std::shared_ptr<IntensityFilm> nn_film =
                nn_connector->communicate(
                    aux_intensity,
                    aux_distance,
                    aux_normals,
                    iispt_integrator->get_normalization_intensity(),
                    iispt_integrator->get_normalization_distance(),
                    communicate_status
                    );

        if (communicate_status) {
            std::cerr << "NN communication issue" << std::endl;
            raise(SIGKILL);
        }

        aux_camera->set_nn_film(nn_film);

        // --------------------------------------------------------------------
        //    * Set the predicted intensity map on the __auxCamera__

        // --------------------------------------------------------------------
        //    * For all pixels within __radius__ and whose intersection and materials are compatible with the original intersection, evaluate __Li__ and update the filmTile

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
// <ray_out> returns the ray used to find the returned intersection
bool IisptRenderRunner::find_intersection(
        RayDifferential r,
        const Scene &scene,
        MemoryArena &arena,
        SurfaceInteraction* isect_out,
        RayDifferential* ray_out,
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
            *ray_out = ray;
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
