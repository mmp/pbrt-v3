
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

// integrators/iispt.cpp*
#include "integrators/iispt.h"
#include "integrators/iispt_d.h"
#include "bssrdf.h"
#include "camera.h"
#include "film.h"
#include "interaction.h"
#include "paramset.h"
#include "scene.h"
#include "stats.h"
#include "progressreporter.h"
#include "cameras/hemispheric.h"
#include "pbrt.h"
#include "samplers/sobol.h"
#include "integrators/path.h"
#include "integrators/volpath.h"
#include "integrators/iispt_estimator_integrator.h"
#include "integrators/iisptnnconnector.h"
#include "film/intensityfilm.h"
#include "integrators/iisptschedulemonitor.h"
#include "integrators/iisptfilmmonitor.h"
#include "integrators/iisptrenderrunner.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <fstream>

namespace pbrt {

STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIBUTION("Integrator/Path length", pathLength);

// Utilities ==================================================================

// Get sample extent in pixels ------------------------------------------------
static Vector2i get_sample_extent(std::shared_ptr<const Camera> camera) {
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    return sampleExtent;
}

// Generate reference file name -----------------------------------------------
// extension must start with a .
static std::string generate_reference_name(std::string identifier_type, Point2i pixel, std::string extension) {
    std::string generated_path =
            IISPT_REFERENCE_DIRECTORY +
            identifier_type +
            "_" +
            std::to_string(pixel.x) +
            "_" +
            std::to_string(pixel.y) +
            extension;
    return generated_path;
}

// Create auxiliary path integrator for reference mode
static std::shared_ptr<PathIntegrator> create_aux_path_integrator(
        int path_pixel_samples,
        std::string output_filename,
        std::shared_ptr<Camera> dcamera,
        Ray auxRay,
        Point2i pixel
        )
{
    std::shared_ptr<HemisphericCamera> pathCamera (
                CreateHemisphericCamera(
                    PbrtOptions.iisptHemiSize,
                    PbrtOptions.iisptHemiSize,
                    dcamera->medium,
                    auxRay.o,
                    Point3f(auxRay.d.x, auxRay.d.y, auxRay.d.z),
                    pixel,
                    output_filename
                    )
                );

    const Bounds2i path_sample_bounds (
                Point2i(0, 0),
                Point2i(PbrtOptions.iisptHemiSize, PbrtOptions.iisptHemiSize)
                );
    std::shared_ptr<Sampler> path_sobol_sampler (CreateSobolSampler(path_sample_bounds, path_pixel_samples));
    int path_max_depth = IISPT_REFERENCE_PATH_MAX_DEPTH;
    Float path_rr_threshold = 1.0;
    std::string path_light_strategy = "spatial";

    std::shared_ptr<PathIntegrator> path_integrator
            (CreatePathIntegrator(path_sobol_sampler,
                                  pathCamera,
                                  path_max_depth,
                                  path_sample_bounds,
                                  path_rr_threshold,
                                  path_light_strategy
                                  ));
    return path_integrator;
}

static std::shared_ptr<VolPathIntegrator> create_aux_volpath_integrator(
        int pixel_samples,
        std::string output_filename,
        std::shared_ptr<Camera> dcamera,
        Ray auxRay,
        Point2i pixel
        )
{
    std::shared_ptr<HemisphericCamera> pathCamera (
                CreateHemisphericCamera(
                    PbrtOptions.iisptHemiSize,
                    PbrtOptions.iisptHemiSize,
                    dcamera->medium,
                    auxRay.o,
                    Point3f(auxRay.d.x, auxRay.d.y, auxRay.d.z),
                    pixel,
                    output_filename
                    )
                );

    const Bounds2i path_sample_bounds (
                Point2i(0, 0),
                Point2i(PbrtOptions.iisptHemiSize, PbrtOptions.iisptHemiSize)
                );
    std::shared_ptr<Sampler> path_sobol_sampler (
                CreateSobolSampler(path_sample_bounds, pixel_samples));
    int path_max_depth = IISPT_REFERENCE_PATH_MAX_DEPTH;
    Float path_rr_threshold = 0.5;
    std::string path_light_strategy = "spatial";

    std::shared_ptr<VolPathIntegrator> path_integrator
            (CreateVolPathIntegrator(path_sobol_sampler,
                                  pathCamera,
                                  path_max_depth,
                                  path_sample_bounds,
                                  path_rr_threshold,
                                  path_light_strategy
                                  ));
    return path_integrator;
}

static std::shared_ptr<VolPathIntegrator> create_aux_volpath_integrator_perspective(
        std::shared_ptr<const Camera> camera
        )
{
    const Bounds2i path_sample_bounds = camera->film->GetSampleBounds();

    std::shared_ptr<Sampler> path_sobol_sampler (
                CreateSobolSampler(
                    path_sample_bounds, 1
                    )
                );

    int path_max_depth = IISPT_REFERENCE_PATH_MAX_DEPTH;
    Float path_rr_threshold = 0.5;
    std::string path_light_strategy = "spatial";

    std::shared_ptr<VolPathIntegrator> path_integrator (
                CreateVolPathIntegrator(
                    path_sobol_sampler,
                    camera,
                    path_max_depth,
                    path_sample_bounds,
                    path_rr_threshold,
                    path_light_strategy
                    )
                );

    return path_integrator;
}

// Check if file exists
static inline bool file_exists(std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

// Utility to execute a lambda only if the file doesn't exist already
template<typename Func>
static void exec_if_not_exists(std::string file_path, Func f) {
    if (PbrtOptions.referenceResume == 0) {
        f();
        return;
    }
    if (!file_exists(file_path)) {
        f();
    }
}

// Utility to execute a lambda only if at least one of the files doesn't
// exist
template<typename Func>
static void exec_if_one_not_exists(std::vector<std::string> &file_paths, Func f) {
    if (PbrtOptions.referenceResume == 0) {
        f();
        return;
    }
    for (std::string fp : file_paths) {
        if (!file_exists(fp)) {
            f();
            return;
        }
    }
}

// IISPTIntegrator Method Definitions =========================================

// Constructor
IISPTIntegrator::IISPTIntegrator(int maxDepth,
                               std::shared_ptr<const Camera> camera,
                               std::shared_ptr<Sampler> sampler,
                               const Bounds2i &pixelBounds,
                               std::shared_ptr<Camera> dcamera,
                               Float rrThreshold,
                               const std::string &lightSampleStrategy
) :
    SamplerIntegrator(camera, sampler, pixelBounds),
    sampler(sampler),
    maxDepth(maxDepth),
    rrThreshold(rrThreshold),
    lightSampleStrategy(lightSampleStrategy),
    dcamera(dcamera)
{

}

void IISPTIntegrator::Preprocess(const Scene &scene) {

    LOG(INFO) << "IISPTIntegrator preprocess";

}

Spectrum IISPTIntegrator::SpecularTransmit(
        const RayDifferential &ray,
        const SurfaceInteraction &isect,
        const Scene &scene,
        Sampler &sampler,
        MemoryArena &arena,
        int depth,
        Point2i pixel
        ) const {
    Vector3f wo = isect.wo, wi;
    Float pdf;
    const Point3f &p = isect.p;
    const Normal3f &ns = isect.shading.n;
    const BSDF &bsdf = *isect.bsdf;
    Spectrum f = bsdf.Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                               BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
    Spectrum L = Spectrum(0.f);
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f) {
        // Compute ray differential _rd_ for specular transmission
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = p + isect.dpdx;
            rd.ryOrigin = p + isect.dpdy;

            Float eta = bsdf.eta;
            Vector3f w = -wo;
            if (Dot(wo, ns) < 0) eta = 1.f / eta;

            Normal3f dndx = isect.shading.dndu * isect.dudx +
                            isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy +
                            isect.shading.dndv * isect.dvdy;

            Vector3f dwodx = -ray.rxDirection - wo,
                     dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);

            Float mu = eta * Dot(w, ns) - Dot(wi, ns);
            Float dmudx =
                (eta - (eta * eta * Dot(w, ns)) / Dot(wi, ns)) * dDNdx;
            Float dmudy =
                (eta - (eta * eta * Dot(w, ns)) / Dot(wi, ns)) * dDNdy;

            rd.rxDirection =
                wi + eta * dwodx - Vector3f(mu * dndx + dmudx * ns);
            rd.ryDirection =
                wi + eta * dwody - Vector3f(mu * dndy + dmudy * ns);
        }
        L = f * Li_direct(rd, scene, sampler, arena, depth + 1, pixel) * AbsDot(wi, ns) / pdf;
    }
    return L;
}

Spectrum IISPTIntegrator::SpecularReflect(
        const RayDifferential &ray,
        const SurfaceInteraction &isect,
        const Scene &scene,
        Sampler &sampler,
        MemoryArena &arena,
        int depth,
        Point2i pixel
        ) const {
    // Compute specular reflection direction _wi_ and BSDF value
    Vector3f wo = isect.wo, wi;
    Float pdf;
    BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
    Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, type);

    // Return contribution of specular reflection
    const Normal3f &ns = isect.shading.n;
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f) {
        // Compute ray differential _rd_ for specular reflection
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = isect.p + isect.dpdx;
            rd.ryOrigin = isect.p + isect.dpdy;
            // Compute differential reflected directions
            Normal3f dndx = isect.shading.dndu * isect.dudx +
                            isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy +
                            isect.shading.dndv * isect.dvdy;
            Vector3f dwodx = -ray.rxDirection - wo,
                     dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);
            rd.rxDirection =
                wi - dwodx + 2.f * Vector3f(Dot(wo, ns) * dndx + dDNdx * ns);
            rd.ryDirection =
                wi - dwody + 2.f * Vector3f(Dot(wo, ns) * dndy + dDNdy * ns);
        }
        return f * Li_direct(rd, scene, sampler, arena, depth + 1, pixel) * AbsDot(wi, ns) /
               pdf;
    } else
        return Spectrum(0.f);
}

static bool is_debug_pixel(Point2i pixel) {
    return (pixel.x % 100 == 0) && (pixel.y % 100 == 0);
}

/*
Reimplementing pixel estimation using Direct Illumination and rendered hemisphere:

in integrator.cpp: UniformSampleAllLights
    for each light, calls EstimateDirect
    Instead of using the number of samples for the light source,
    I do the for loop for every pixel in my hemisphere

in integrator.cpp: EstimateDirect
    here is Multiple Importance Sampling
    light.Sample_Li is sampling the illumination. I can replace this to sample from my hemisphere.
*/

// Write info file ============================================================
void IISPTIntegrator::write_info_file(std::string out_filename) {

    rapidjson::Document jd;
    jd.SetObject();

    auto& allocator = jd.GetAllocator();

    jd.AddMember(
                rapidjson::Value("normalization_intensity", allocator).Move(),
                rapidjson::Value().SetDouble(max_intensity),
            allocator);

    jd.AddMember(
                rapidjson::Value("normalization_distance", allocator).Move(),
                rapidjson::Value().SetDouble(max_distance),
                allocator
                );

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer (buffer);
    jd.Accept(writer);

    std::ofstream out_file (out_filename);
    out_file << buffer.GetString();
    out_file.close();
}

// Estimate intensity normalization ===========================================
void IISPTIntegrator::estimate_normalization_values(
        const Scene &scene,
        Vector2i sample_extent
        )
{
    // Create RNG
    std::shared_ptr<RNG> rng (new RNG());

    // Create auxiliary estimation path tracer
    std::shared_ptr<VolPathIntegrator> aux_volpath =
            create_aux_volpath_integrator_perspective(
                camera
                );

    std::shared_ptr<IISPTEstimatorIntegrator> estimator_integrator (
                new IISPTEstimatorIntegrator(
                    aux_volpath,
                    camera,
                    scene,
                    sampler
                    )
                );

    // Loop to get the samples
    for (int i = 0; i < IISPT_NORMALIZATION_ESTIMATION_SAMPLES; i++) {
        int x = rng->UniformUInt32(sample_extent.x);
        int y = rng->UniformUInt32(sample_extent.y);
        estimator_integrator->estimate_intensity(
                    scene,
                    Point2i(x, y),
                    sampler
                    );
        estimator_integrator->estimate_distance(
                    scene,
                    Point2i(x, y),
                    sampler
                    );
    }

    // Print statistics
    max_intensity = estimator_integrator->get_max_intensity();
    max_distance = estimator_integrator->get_max_distance();
    std::cerr << "Max intensity ["<< max_intensity <<"] Max distance ["<< max_distance <<"]" << std::endl;

    // Write info file
    write_info_file(IISPT_REFERENCE_DIRECTORY + IISPT_REFERENCE_TRAIN_INFO);
}

// Estimate normalization values ==============================================
void IISPTIntegrator::estimate_normalization(const Scene &scene) {
    std::cerr << "Start estimate_normalization" << std::endl;
    Vector2i sample_extent = get_sample_extent(camera);
    estimate_normalization_values(scene, sample_extent);
}

// Render =====================================================================
void IISPTIntegrator::Render(const Scene &scene) {
    if (PbrtOptions.referenceTiles <= 0) {
        // Normal render of the scene
        std::cerr << "Starting normal 2 render" << std::endl;
        render_normal_2(scene);
        // render_normal(scene);
    } else {
        // Render reference training views
        std::cerr << "Starting reference render" << std::endl;
        render_reference(scene);
    }
}

// ============================================================================
// Render normal 2
void IISPTIntegrator::render_normal_2(const Scene &scene) {

    Preprocess(scene);

    estimate_normalization(scene);

    // Create: IisptScheduleMonitor, IisptFilmMonitor,
    // IisptRenderRunner

    std::shared_ptr<IisptScheduleMonitor> schedule_monitor (
                new IisptScheduleMonitor(camera->film->GetSampleBounds())
                );

    std::shared_ptr<IisptFilmMonitor> film_monitor (
                new IisptFilmMonitor(
                    camera->film->GetSampleBounds()
                    )
                );

    std::shared_ptr<IisptRenderRunner> render_runner (
                new IisptRenderRunner(
                    this,
                    schedule_monitor,
                    film_monitor,
                    camera,
                    dcamera,
                    sampler,
                    0,
                    camera->film->GetSampleBounds()
                    )
                );

    render_runner->run(scene);

    std::shared_ptr<IntensityFilm> output_film =
            film_monitor->to_intensity_film();

    output_film->write("/tmp/iispt.pfm");
    output_film->pbrt_write("/tmp/iispt.exr");
}

// Render reference ===========================================================
void IISPTIntegrator::render_reference(const Scene &scene) {

    Preprocess(scene);

    estimate_normalization(scene);

    // Create the auxiliary integrator for intersection-view
    this->dintegrator = std::shared_ptr<IISPTdIntegrator>(CreateIISPTdIntegrator(dcamera));
    // Preprocess on auxiliary integrator
    dintegrator->Preprocess(scene);

    // Compute number of tiles
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    int reference_tiles = PbrtOptions.referenceTiles;
    int reference_tile_interval_x = sampleExtent.x / reference_tiles; // Pixels
    int reference_tile_interval_y = sampleExtent.y / reference_tiles; // Pixels

    if (reference_tile_interval_x == 0 || reference_tile_interval_y == 0) {
        fprintf(stderr, "Reference tile interval too small. Image resolution could be too small or reference tiles too many\n");
        return;
    }

    // Initialize sampler and arena
    MemoryArena arena;
    std::unique_ptr<Sampler> tile_sampler = sampler->Clone(0);

    for (int px_y = 0; px_y < sampleExtent.y; px_y += reference_tile_interval_y) {
        for (int px_x = 0; px_x < sampleExtent.x; px_x += reference_tile_interval_x) {

            std::cerr << "Current pixel ["<< px_x <<"] ["<< px_y <<"]" << std::endl;

            CameraSample current_sample;
            current_sample.pFilm = Point2f(px_x, px_y);
            current_sample.time = 0;

            std::cerr << "Camera sample is " << current_sample << std::endl;

            // Render IISPTd views and Reference views
            RayDifferential ray;
            Float rayWeight = camera->GenerateRayDifferential(current_sample, &ray);
            // It's a single pass per pixel, so we don't scale the differential
            ray.ScaleDifferentials(1);
            // The Li method, in reference mode, will automatically save the reference images
            // to the out/ directory
            Li(ray, scene, *tile_sampler, arena, 0, Point2i(px_x, px_y));


        }
    }

}

// Estimate direct ============================================================
static Spectrum IISPTEstimateDirect(
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

    //Spectrum Li = auxCamera->getLightSample(hem_x, hem_y, &wi);
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

// Sample hemisphere ==========================================================
static Spectrum IISPTSampleHemisphere(
        const Interaction &it,
        const Scene &scene,
        MemoryArena &arena,
        Sampler &sampler,
        HemisphericCamera* auxCamera
        ) {
    ProfilePhase p(Prof::DirectLighting);
    Spectrum L(0.f);

    // Loop for every pixel in the hemisphere
    // TODO adjust jacobian
    for (int hemi_x = 0; hemi_x < PbrtOptions.iisptHemiSize; hemi_x++) {
        for (int hemi_y = 0; hemi_y < PbrtOptions.iisptHemiSize; hemi_y++) {
            L += IISPTEstimateDirect(it, hemi_x, hemi_y, auxCamera);
        }
    }

    std::cerr << "Sum of all IISPTEstimateDirect is " << L << std::endl;

    int n_samples = PbrtOptions.iisptHemiSize * PbrtOptions.iisptHemiSize;

    return L / n_samples;
}

// Disabled version ===========================================================
Spectrum IISPTIntegrator::Li(const RayDifferential &r,
                             const Scene &scene,
                             Sampler &sampler,
                             MemoryArena &arena,
                             int depth
                             ) const {
    fprintf(stderr, "ERROR IISPTIntegrator::Li, overridden version, is not defined in this debug version.");
    exit(1);
}

// Direct version used by specular and transmit ===============================
Spectrum IISPTIntegrator::Li_direct(
        const RayDifferential &ray,
        const Scene &scene,
        Sampler &sampler,
        MemoryArena &arena,
        int depth,
        Point2i pixel
        ) const {
    Spectrum L (0.f);
    return L;
}

// New version ================================================================
Spectrum IISPTIntegrator::Li(const RayDifferential &ray,
                             const Scene &scene,
                             Sampler &sampler,
                             MemoryArena &arena,
                             int depth,
                             Point2i pixel
                             ) const {

    std::cerr << "Debug trace start" << std::endl;

    ProfilePhase p(Prof::SamplerIntegratorLi);
    Spectrum L (0.f);
    Spectrum beta (1.f);

    // Find closest ray intersection or return background radiance
    SurfaceInteraction isect;
    if (!scene.Intersect(ray, &isect)) {
        if (PbrtOptions.referenceTiles > 0) {
            std::cerr << "No intersection" << std::endl;
        }
        for (const auto &light : scene.infiniteLights) {
            L += beta * light->Le(ray);
        }
        return L;
    }

    // Compute the hemisphere -------------------------------------------------

    // Invert normal if the surface's normal was pointing inwards
    Normal3f surfNormal = isect.n;
    if (Dot(Vector3f(isect.n.x, isect.n.y, isect.n.z), Vector3f(ray.d.x, ray.d.y, ray.d.z)) > 0.0) {
        surfNormal = Normal3f(-isect.n.x, -isect.n.y, -isect.n.z);
    }

    // auxRay is centered at the intersection point, and points towards the intersection
    // surface normal
    Ray auxRay = isect.SpawnRay(Vector3f(surfNormal));

    // testCamera is used for the hemispheric rendering
    std::string reference_d_name = generate_reference_name("d", pixel, ".pfm");
    std::shared_ptr<HemisphericCamera> auxCamera (
                CreateHemisphericCamera(
                    PbrtOptions.iisptHemiSize,
                    PbrtOptions.iisptHemiSize,
                    dcamera->medium,
                    auxRay.o,
                    Point3f(auxRay.d.x, auxRay.d.y, auxRay.d.z),
                    pixel,
                    reference_d_name
                    )
                );

    // In Reference mode, save the rendered view ------------------------------
    if (PbrtOptions.referenceTiles > 0) {
        std::vector<std::string> direct_reference_names;
        direct_reference_names.push_back(reference_d_name);
        std::string reference_z_name = generate_reference_name("z", pixel, ".pfm");
        direct_reference_names.push_back(reference_z_name);
        std::string reference_n_name = generate_reference_name("n", pixel, ".pfm");
        direct_reference_names.push_back(reference_n_name);
        exec_if_one_not_exists(direct_reference_names, [&]() {
            // Start rendering the hemispherical view
            this->dintegrator->RenderView(scene, auxCamera.get());
            dintegrator->save_reference(
                        auxCamera,
                        reference_z_name, // distance map
                        reference_n_name  // normal map
                        );
        });
    } else {
        // Normal mode --------------------------------------------------------
        // Start rendering the hemispherical view
        std::cerr << "Normal mode, starting hemispheric render" << std::endl;
        this->dintegrator->RenderView(scene, auxCamera.get());
        std::cerr << "hemispheric render obtained. Getting intensity image" << std::endl;
        std::unique_ptr<IntensityFilm> dcamera_intensity = dintegrator->get_intensity_film(auxCamera.get());
        std::cerr << "Got the intensity image. Saving to /tmp/int.pfm" << std::endl;
        dcamera_intensity->write(std::string("/tmp/int.pfm"));
        std::cerr << "Saved." << std::endl;

        // Get normals and distance films
        NormalFilm* dcamera_normal = dintegrator->get_normal_film();
        DistanceFilm* dcamera_distance = dintegrator->get_distance_film();

        // Create the IISPT NN Connector
        std::cerr << "Creating NN connector..." << std::endl;
        std::unique_ptr<IisptNnConnector> nn_connector (
                    new IisptNnConnector()
                    );
        std::cerr << "Calling communicate" << std::endl;
        int comm_status = -1;
        std::shared_ptr<IntensityFilm> nn_film = nn_connector->communicate(
                    dcamera_intensity.get(),
                    dcamera_distance,
                    dcamera_normal,
                    max_intensity,
                    max_distance,
                    comm_status
                    );

        // Save the neural network produced film in the camera
        auxCamera->set_nn_film(nn_film);
    }

    if (PbrtOptions.referenceTiles > 0) {
        std::string reference_b_name = generate_reference_name("p", pixel, ".pfm");
        exec_if_not_exists(reference_b_name, [&]() {
            std::shared_ptr<VolPathIntegrator> volpath =
                    create_aux_volpath_integrator(
                        PbrtOptions.referencePixelSamples,
                        reference_b_name,
                        auxCamera,
                        auxRay,
                        pixel
                        );
            // Start rendering the ground truth
            // The render method will automatically save the image
            volpath->Render(scene);
        });
    }

    // Compute scattering functions for surface interaction
    isect.ComputeScatteringFunctions(ray, arena);
    if (!isect.bsdf) {
        return Li(isect.SpawnRay(ray.d), scene, sampler, arena, depth);
    }

    // wo should be the vector towards camera, from intersection
    Vector3f wo = isect.wo;
    Float woLength = Dot(wo, wo);
    if (woLength == 0) {
        fprintf(stderr, "iispt.cpp: Detected a 0 length wo");
        exit(1);
    }

    // Compute emitted light if ray hit an area light source
    L += isect.Le(wo);
    if (scene.lights.size() > 0) {
        // Compute direct lighting using hemisphere information TODO
        L += IISPTSampleHemisphere(isect, scene, arena, sampler, auxCamera.get());
    }

    return L;

}

// Creator ====================================================================
IISPTIntegrator *CreateIISPTIntegrator(const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera,
    std::shared_ptr<Camera> dcamera
) {
    LOG(INFO) << "CreateIISPTIntegrator: in";

    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    Float rrThreshold = params.FindOneFloat("rrthreshold", 1.);
    std::string lightStrategy =
        params.FindOneString("lightsamplestrategy", "spatial");
    return new IISPTIntegrator(maxDepth, camera, sampler, pixelBounds,
        dcamera, rrThreshold, lightStrategy);
}

}  // namespace pbrt
