#include "iisptrenderrunner.h"
#include "lightdistrib.h"

#include <chrono>
#include <csignal>

namespace pbrt {

// ============================================================================
// Estimate direct (evaluate 1 hemisphere pixel)
// Output is scaled by 1/pp(x)
//        which is the probability of sampling the specific
//        pp is not a 0-1 probability but it's 1-centered
//        for a uniform distribution
// See intensityfilm.cpp for more detail
static Spectrum estimate_direct(
        const Interaction &it,
        float rx, // input uniform random floats
        float ry,
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
    float pp_prob;
    Spectrum Li = auxCamera->get_light_sample_nn_importance(
                rx,
                ry,
                &wi,
                &pp_prob
                );
    if (pp_prob <= 1e-5) {
        return Spectrum(0.0);
    }

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

    return Ld / pp_prob;

}

// ============================================================================
// Sample hemisphere with multiple cameras and weights
Spectrum IisptRenderRunner::sample_hemisphere(
        const Interaction &it,
        std::vector<float> &weights,
        std::vector<HemisphericCamera*> &cameras
        )
{
    Spectrum L(0.f);

    int samples_taken = 0;

    for (int i = 0; i < cameras.size(); i++) {
        HemisphericCamera* a_camera = cameras[i];
        if (a_camera != NULL) {
            a_camera->compute_cdfs();
        }
        float a_weight = weights[i];

        // Attempt HEMISPHERIC_IMPORTANCE_SAMPLES to sample this camera
        // The expected number of samples across all the cameras will be
        // HEMISPHERIC_IMPORTANCE_SAMPLES
        for (int j = 0; j < HEMISPHERIC_IMPORTANCE_SAMPLES; j++) {
            float rr = rng->uniform_float();
            if (rr < a_weight) {
                samples_taken++;
                if (a_camera != NULL) {
                    float rx = rng->uniform_float();
                    float ry = rng->uniform_float();
                    L += estimate_direct(it, rx, ry, a_camera);
                }
            }
        }
    }

    if (samples_taken > 0) {
        return L / samples_taken;
    } else {
        return Spectrum(0.0);
    }
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

    std::cerr << "iisptrenderrunner.cpp: Creating NN connector\n";
    this->nn_connector = std::unique_ptr<IisptNnConnector>(
                new IisptNnConnector()
                );
    std::cerr << "iisptrenderrunner.cpp: NN connector created\n";

    // TODO remove fixed seed
    this->rng = std::unique_ptr<IisptRng>(
                new IisptRng(thread_no)
                );

    this->sampler = std::unique_ptr<Sampler>(
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
    std::cerr << "iisptrenderrunner.cpp: New tiled renderer\n";
    d_integrator->Preprocess(scene);
    int loop_count = 0;
    std::unique_ptr<LightDistribution> lightDistribution =
            CreateLightSampleDistribution(std::string("spatial"), scene);

    std::cerr << "iisptrenderrunner.cpp: start render loop\n";

    while (1) {

        loop_count++;
        std::cerr << "iisptrenderrunner.cpp loop count " << loop_count << std::endl;

        if (loop_count > 10) {
            return;
        }

        MemoryArena arena;

        // Obtain the current task
        IisptScheduleMonitorTask sm_task = schedule_monitor->next_task();
        // sm_task end points are exclusive
        std::cerr << "Obtained new task: ["<< sm_task.x0 <<"]["<< sm_task.y0 <<"]-["<< sm_task.x1 <<"]["<< sm_task.y1 <<"] tilesize ["<< sm_task.tilesize <<"]\n";

        // Use a HashMap to store the hemi points
        std::unordered_map<
                IisptPoint2i,
                std::shared_ptr<HemisphericCamera>
                > hemi_points;

        // Check the iteration space of the tiles
        int tile_x = sm_task.x0;
        int tile_y = sm_task.y0;
        while (1) {
            // Process current tile
            std::cerr << "Hemi point ["<< tile_x <<"] ["<< tile_y <<"]\n";
            IisptPoint2i hemi_key;
            hemi_key.x = tile_x;
            hemi_key.y = tile_y;

            Point2i pixel (tile_x, tile_y);

            // Obtain camera ray and shoot into scene.

            sampler_next_pixel();

            CameraSample camera_sample =
                    sampler->GetCameraSample(pixel);

            RayDifferential r;
            main_camera->GenerateRayDifferential(
                        camera_sample,
                        &r
                        );
            r.ScaleDifferentials(1.0);

            // Find intersection

            SurfaceInteraction isect;
            Spectrum beta;
            Spectrum background;
            RayDifferential ray;

            bool intersection_found = find_intersection(
                        r,
                        scene,
                        arena,
                        &isect,
                        &ray,
                        &beta,
                        &background
                        );

            if (!intersection_found || beta.y() <= 0.0) {

                // Set a black hemi
                hemi_points[hemi_key] = nullptr;

            } else {

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
                std::unique_ptr<HemisphericCamera> aux_camera (
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
                d_integrator->RenderView(scene, aux_camera.get());

                // Use NN Connector

                // Obtain intensity, normals, distance maps

                std::unique_ptr<IntensityFilm> aux_intensity =
                        d_integrator->get_intensity_film(aux_camera.get());

                NormalFilm* aux_normals =
                        d_integrator->get_normal_film();

                DistanceFilm* aux_distance =
                        d_integrator->get_distance_film();

                int communicate_status = -1;
                std::shared_ptr<IntensityFilm> nn_film =
                        nn_connector->communicate(
                            aux_intensity.get(),
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

                hemi_points[hemi_key] = std::move(aux_camera);

            }

            // Advance to the next tile
            bool advance_tile_y = false;
            if (tile_x == sm_task.x1 - 1) {
                // This was the last tile of the row, go to next row
                tile_x = sm_task.x0;
                advance_tile_y = true;
            } else if (tile_x >= sm_task.x1) {
                std::cerr << "iisptrenderrunner: ERROR tile has gone past the end\n";
                std::raise(SIGKILL);
            } else {
                // Advance x only
                tile_x = std::min(
                            tile_x + sm_task.tilesize,
                            sm_task.x1 - 1
                            );
            }

            if (advance_tile_y) {
                if (tile_y == sm_task.y1 - 1) {
                    // This was the last row,
                    // complete the loop
                    break;
                } else {
                    // Advance to the next row
                    tile_y = std::min(
                                tile_y + sm_task.tilesize,
                                sm_task.y1 - 1
                                );
                }
            }
        }

        // Check neighbour hemi points for each pixel in the task
        // Neigh:
        //     S - top left
        //     R - top right
        //     B - bottom left
        //     E - bottom right
        for (int fy = sm_task.y0; fy < sm_task.y1; fy++) {
            for (int fx = sm_task.x0; fx < sm_task.x1; fx++) {

                Point2i f_pixel (fx, fy);

                Point2i neigh_s (
                            fx - (fx % sm_task.tilesize),
                            fy - (fy % sm_task.tilesize)
                            );


                Point2i neigh_e (
                            std::min(
                                neigh_s.x + sm_task.tilesize,
                                sm_task.x1 - 1
                                ),
                            std::min(
                                neigh_s.y + sm_task.tilesize,
                                sm_task.y1 - 1
                                )
                            );

                Point2i neigh_r (
                            neigh_e.x,
                            neigh_s.y
                            );

                Point2i neigh_b (
                            neigh_s.x,
                            neigh_e.y
                            );

                // Compute distances from the neighbours
                float dist_s = iispt::points_distance(f_pixel, neigh_s);
                float dist_r = iispt::points_distance(f_pixel, neigh_r);
                float dist_b = iispt::points_distance(f_pixel, neigh_b);
                float dist_e = iispt::points_distance(f_pixel, neigh_e);

                // Normalize the distances
                float dist_sum = dist_s + dist_r + dist_b + dist_e;
                if (dist_sum <= 0.0) {
                    // Degenerate tile
                    continue;
                }

                dist_s /= dist_sum;
                dist_r /= dist_sum;
                dist_b /= dist_sum;
                dist_e /= dist_sum;

                // Invert
                dist_s = 1.0 - dist_s;
                dist_r = 1.0 - dist_r;
                dist_b = 1.0 - dist_b;
                dist_e = 1.0 - dist_e;

                // Constructor vectors for sampling
                std::vector<float> hemi_sampling_weights (4);
                hemi_sampling_weights[0] = dist_s;
                hemi_sampling_weights[1] = dist_r;
                hemi_sampling_weights[2] = dist_b;
                hemi_sampling_weights[3] = dist_e;

                std::vector<HemisphericCamera*> hemi_sampling_cameras (4);
                auto hemi_point_get = [&](Point2i pt) {
                    IisptPoint2i pt_key;
                    pt_key.x = pt.x;
                    pt_key.y = pt.y;
                    std::shared_ptr<HemisphericCamera> a_cmr =
                            hemi_points.at(pt_key);
                    if (a_cmr == nullptr) {
                        return (HemisphericCamera*) NULL;
                    } else {
                        return a_cmr.get();
                    }
                };

                hemi_sampling_cameras[0] = hemi_point_get(neigh_s);
                hemi_sampling_cameras[1] = hemi_point_get(neigh_r);
                hemi_sampling_cameras[2] = hemi_point_get(neigh_b);
                hemi_sampling_cameras[3] = hemi_point_get(neigh_e);

                sampler_next_pixel();
                CameraSample f_camera_sample =
                        sampler->GetCameraSample(f_pixel);

                RayDifferential f_r;
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
                    // No intersection found, record background
                    film_monitor->add_sample(
                                f_pixel,
                                f_background,
                                1.0
                                );
                    continue;
                } else if (f_intersection_found && f_beta.y() <= 0.0) {
                    // Intersection found but black pixel
                    // Nothing to do
                    continue;
                }

                // Valid intersection found

                // Compute scattering functions for surface interaction
                f_isect.ComputeScatteringFunctions(f_ray, arena);
                if (!f_isect.bsdf) {
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

                // Sample one direct lighting
                const Distribution1D* distribution = lightDistribution->Lookup(f_isect.p);
                L += path_uniform_sample_one_light(
                            f_isect,
                            scene,
                            arena,
                            false,
                            distribution
                            );

                // Compute emitted light if ray hit an area light source
                L += f_isect.Le(wo);

                // Compute hemispheric contribution
                L += sample_hemisphere(
                            f_isect,
                            hemi_sampling_weights,
                            hemi_sampling_cameras
                            );

                // Record sample
                film_monitor->add_sample(
                            f_pixel,
                            f_beta * L,
                            1.0);


            }
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
    int randx = rng->uniform_uint32(width);
    int randy = rng->uniform_uint32(height);
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
            }
            *background_out = L;
            return false;
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

// ============================================================================
// Compute weights
// Gaussian filtering weight
double IisptRenderRunner::compute_filter_weight(
        int cx, // Centre sampling pixel
        int cy,
        int fx, // Current filter pixel
        int fy,
        float radius, // Filter radius,
        double* scaling_factor // Scaling factor to obtain a gaussian curve
                               // which has point X=0, Y=1
        )
{
    double sigma = radius / 3.0;

    // Compute distance
    double dx2 = (double) (cx - fx);
    dx2 = dx2 * dx2;
    double dy2 = (double) (cy - fy);
    dy2 = dy2 * dy2;
    double distance = std::sqrt(
                dx2 + dy2
                );

    // Compute gaussian weight
    double gaussian_weight = iispt::gauss(sigma, distance);
    *scaling_factor = 1.0 / iispt::gauss(sigma, 0.0);
    return gaussian_weight;
}

// ============================================================================

Spectrum IisptRenderRunner::path_uniform_sample_one_light(
        Interaction &it,
        const Scene &scene,
        MemoryArena &arena,
        bool handleMedia,
        const Distribution1D* lightDistrib
        )
{
    // Randomly choose a single light to sample
    int nLights = int(scene.lights.size());
    if (nLights == 0) {
        return Spectrum(0.0);
    }

    int lightNum;
    float lightPdf;
    if (lightDistrib) {
        lightNum = lightDistrib->SampleDiscrete(sampler->Get1D(), &lightPdf);
        if (lightPdf == 0) {
            return Spectrum(0.0);
        }
    } else {
        lightNum = std::min((int)(sampler->Get1D() * nLights), nLights - 1);
        lightPdf = Float(1) / nLights;
    }

    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    Point2f uLight = sampler->Get2D();
    Point2f uScattering = sampler->Get2D();
    return estimate_direct_lighting(it, uScattering, *light, uLight,
                          scene, arena, handleMedia, false) / lightPdf;
}

// ============================================================================

Spectrum IisptRenderRunner::estimate_direct_lighting(
        Interaction &it,
        const Point2f &uScattering,
        const Light &light,
        const Point2f &uLight,
        const Scene &scene,
        MemoryArena &arena,
        bool handleMedia,
        bool specular
        )
{
    BxDFType bsdfFlags =
        specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Spectrum Ld(0.f);
    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester visibility;
    Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample
        Spectrum f;
        if (it.IsSurfaceInteraction()) {
            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) *
                AbsDot(wi, isect.shading.n);
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
            if (handleMedia) {
                Li *= visibility.Tr(scene, *sampler);
            } else {
              if (!visibility.Unoccluded(scene)) {
                Li = Spectrum(0.f);
              }
            }

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                if (IsDeltaLight(light.flags)) {
                    Ld += f * Li / lightPdf;
                } else {
                    Float weight =
                        PowerHeuristic(1, lightPdf, 1, scatteringPdf);
                    Ld += f * Li * weight / lightPdf;
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if (!IsDeltaLight(light.flags)) {
        Spectrum f;
        bool sampledSpecular = false;
        if (it.IsSurfaceInteraction()) {
            // Sample scattered direction for surface interactions
            BxDFType sampledType;
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
                                     bsdfFlags, &sampledType);
            f *= AbsDot(wi, isect.shading.n);
            sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
        } else {
            // Sample scattered direction for medium interactions
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->Sample_p(mi.wo, &wi, uScattering);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        if (!f.IsBlack() && scatteringPdf > 0) {
            // Account for light contributions along sampled direction _wi_
            Float weight = 1;
            if (!sampledSpecular) {
                lightPdf = light.Pdf_Li(it, wi);
                if (lightPdf == 0) {
                    return Ld;
                }
                weight = PowerHeuristic(1, scatteringPdf, 1, lightPdf);
            }

            // Find intersection and compute transmittance
            SurfaceInteraction lightIsect;
            Ray ray = it.SpawnRay(wi);
            Spectrum Tr(1.f);
            bool foundSurfaceInteraction =
                handleMedia ? scene.IntersectTr(ray, *sampler, &lightIsect, &Tr)
                            : scene.Intersect(ray, &lightIsect);

            // Add light contribution from material sampling
            Spectrum Li(0.f);
            if (foundSurfaceInteraction) {
                if (lightIsect.primitive->GetAreaLight() == &light) {
                    Li = lightIsect.Le(-wi);
                }
            } else {
                Li = light.Le(ray);
            }
            if (!Li.IsBlack()) {
                Ld += f * Li * Tr * weight / scatteringPdf;
            }
        }
    }
    return Ld;
}

// ============================================================================
// This method increments the sampler pixel count to make sure that we
// never use two pixel values more than once, increasing sampling diversity

void IisptRenderRunner::sampler_next_pixel()
{

    int x = sampler_pixel_counter.x;
    int y = sampler_pixel_counter.y;
    if (x == INT_MAX) {
        x = -1;
        y++;
    }
    x++;
    sampler_pixel_counter = Point2i(x, y);
    sampler->StartPixel(sampler_pixel_counter);

}

}
