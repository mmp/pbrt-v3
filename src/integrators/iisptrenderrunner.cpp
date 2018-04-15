#include "iisptrenderrunner.h"

namespace pbrt {

// ============================================================================
// Estimate direct (evaluate 1 hemisphere pixel)
static Spectrum estimate_direct(
        const Interaction &it,
        int hem_x,
        int hem_y,
        HemisphericCamera* auxCamera
        ) {

    bool specular = false; // Default value

    BxDFType bsdfFlags =
        specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Spectrum Ld(0.f);

    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 1.0 / 6.28;
    Float scatteringPdf = 0;
    VisibilityTester visibility;

    // Sample_Li with custom code to sample from hemisphere instead -----------
    // Writes into wi the vector towards the light source. Derived from hem_x and hem_y
    // For the hemisphere, lightPdf would be a constant (probably 1/(2pi))
    // We don't need to have a visibility object

    // Get jacobian-adjusted sample, camera coordinates
    Spectrum Li = auxCamera->get_light_sample_nn(hem_x, hem_y, &wi);

    // Combine incoming light, BRDF and viewing direction ---------------------
    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample

        Spectrum f;

        if (it.IsSurfaceInteraction()) {

            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;

            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) * AbsDot(wi, isect.shading.n);

            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);

        } else {

            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;

        }

        if (!f.IsBlack()) {
            // Compute effect of visibility for light source sample
            // Always unoccluded visibility using hemispherical map

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                Ld += f * Li / lightPdf;
            }
        }
    }

    // Skipping sampling BSDF with multiple importance sampling
    // because we gather all information from lights (hemisphere)

    return Ld;

}

// ============================================================================
// Sample hemisphere
static Spectrum sample_hemisphere(
        const Interaction &it,
        HemisphericCamera* auxCamera
        ) {
    Spectrum L(0.f);

    // Loop for every pixel in the hemisphere
    for (int hemi_x = 0; hemi_x < PbrtOptions.iisptHemiSize; hemi_x++) {
        for (int hemi_y = 0; hemi_y < PbrtOptions.iisptHemiSize; hemi_y++) {
            L += estimate_direct(it, hemi_x, hemi_y, auxCamera);
        }
    }

    int n_samples = PbrtOptions.iisptHemiSize * PbrtOptions.iisptHemiSize;

    return L / n_samples;
}

// ============================================================================
IisptRenderRunner::IisptRenderRunner(
        IISPTIntegrator* iispt_integrator,
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
    int loop_count = 0;

    while (1) {


        if (stop) {
            return;
        }

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

        std::cerr << "Getting camera sample...\n";
        CameraSample camera_sample =
                sampler->GetCameraSample(pixel);
        std::cerr << "Got the camera sample\n";

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
        } else if (intersection_found && beta.y() <= 0.0) {
            // Intersection found but black pixel
            film_monitor->add_sample(
                        pixel,
                        Spectrum(0.0)
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

        // --------------------------------------------------------------------
        //    * Set the predicted intensity map on the __auxCamera__

        aux_camera->set_nn_film(nn_film);

        // --------------------------------------------------------------------
        //    * For all pixels within __radius__ and whose intersection and materials are compatible with the original intersection, evaluate __Li__ and update the filmTile

        // TODO move from a square area to a circular area

        // TODO add special case for very small radius

        int filter_start_x = std::max(
                    film_monitor->get_film_bounds().pMin.x,
                    (int) std::round(((float) x) - radius)
                    );

        int filter_end_x = std::min(
                    film_monitor->get_film_bounds().pMax.x,
                    (int) std::round(((float) x) + radius)
                    );

        int filter_start_y = std::max(
                    film_monitor->get_film_bounds().pMin.y,
                    (int) std::round(((float) y) - radius)
                    );

        int filter_end_y = std::min(
                    film_monitor->get_film_bounds().pMax.y,
                    (int) std::round(((float) y) + radius)
                    );

        for (int fy = filter_start_y; fy <= filter_end_y; fy++) {
            for (int fx = filter_start_x; fx <= filter_end_x; fx++) {

                Point2i f_pixel = Point2i(fx, fy);
                sampler->StartPixel(f_pixel);

                CameraSample f_camera_sample =
                        sampler->GetCameraSample(f_pixel);

                RayDifferential f_r;
                Float f_ray_weight =
                        main_camera->GenerateRayDifferential(
                            f_camera_sample,
                            &f_r
                            );
                f_r.ScaleDifferentials(1.0);

                SurfaceInteraction f_isect;
                Spectrum f_beta;
                Spectrum f_background;
                RayDifferential f_ray;

                // Find intersection point
                bool f_intersection_found = find_intersection(
                            f_r,
                            scene,
                            arena,
                            &f_isect,
                            &f_ray,
                            &f_beta,
                            &f_background
                            );

                if (!f_intersection_found) {
                    // No intersection found, nothing to do
                    continue;
                } else if (f_intersection_found && f_beta.y() <= 0.0) {
                    // Intersection found but black pixel
                    // Nothing to do
                    continue;
                }

                // Valid intersection found

                // TODO check if intersection is within valid range
                // and that has similar material and normal facing

                // Compute scattering functions for surface interaction
                f_isect.ComputeScatteringFunctions(f_ray, arena);
                if (!isect.bsdf) {
                    // This should not be possible, because find_intersection()
                    // would have skipped the intersection
                    // so do nothing
                    continue;
                }

                // wo is vector towards viewer, from intersection
                Vector3f wo = f_isect.wo;
                Float wo_length = Dot(wo, wo);
                if (wo_length == 0) {
                    std::cerr << "iisptrenderrunner.cpp: Detected a 0 length wo" << std::endl;
                    raise(SIGKILL);
                    exit(1);
                }

                Spectrum L (0.0);

                // Compute emitted light if ray hit an area light source
                L += f_isect.Le(wo);

                // Compute hemispheric contribution
                L += sample_hemisphere(
                            f_isect,
                            aux_camera.get()
                            );

                // Record sample
                film_monitor->add_sample(f_pixel, L);

            }
        }

        loop_count++;
        if (loop_count > 20) {
            stop = true;
        }

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

    for (int bounces = 0; bounces < 24; ++bounces) {

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
            *beta_out = Spectrum(0.0);
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
            *beta_out = Spectrum(0.0);
            return true;
        }
        // Spawn the new ray
        ray = isect.SpawnRay(wi);

        // Skip subsurface scattering

        // Skip RR termination

    }

    // Max depth reached, return 0 beta
    *beta_out = Spectrum(0.0);
    return true;
}

}
