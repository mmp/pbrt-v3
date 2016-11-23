// Kevin Egan

#include <stdio.h>
#include <stdlib.h>

#include "pbrt.h"
#include "reflection.h"
#include "sampling.h"
#include "memory.h"
#include "api.h"
#include "paramset.h"
#include "shapes/disk.h"

using namespace pbrt;

static MemoryArena arena;
static RNG rng;

// extract the red channel from a Spectrum class
double spectrumRedValue(const Spectrum& s) { return s[0]; }

typedef void (*CreateBSDFFunc)(BSDF* bsdf);

void createLambertian(BSDF* bsdf);
void createOrenNayar0(BSDF* bsdf);
void createOrenNayar20(BSDF* bsdf);
void createMicrofacet(BSDF* bsdf, bool beckmann, bool samplevisible,
                      float roughx, float roughy);
void createMicrofacet30and0(BSDF* bsdf);
void createFresnelBlend(BSDF* bsdf, bool beckmann, bool samplevisible,
                        float roughx, float roughy);

typedef void (*GenSampleFunc)(BSDF* bsdf, const Vector3f& wo, Vector3f* wi,
                              Float* pdf, Spectrum* f);

void Gen_Sample_f(BSDF* bsdf, const Vector3f& wo, Vector3f* wi, Float* pdf,
                  Spectrum* f);
void Gen_CosHemisphere(BSDF* bsdf, const Vector3f& wo, Vector3f* wi, Float* pdf,
                       Spectrum* f);
void Gen_UniformHemisphere(BSDF* bsdf, const Vector3f& wo, Vector3f* wi,
                           Float* pdf, Spectrum* f);

int main(int argc, char* argv[]) {
    Options opt;
    pbrtInit(opt);

    // number of monte carlo estimates
    // const int estimates = 1;
    const int estimates = 10000000;

    // radiance of uniform environment map
    const double environmentRadiance = 1.0;

    fprintf(stderr,
            "outgoing radiance from a surface viewed\n"
            "straight on with uniform lighting\n\n"
            "    uniform incoming radiance = %.3f\n"
            "    monte carlo samples = %d\n\n\n",
            environmentRadiance, estimates);

    CreateBSDFFunc BSDFFuncArray[] = {
        createLambertian,
        createOrenNayar0,
        createOrenNayar20,
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, true, 0.5, 0.5); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, true, 0.5, 0.5); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, true, 0.2, 0.1); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, true, 0.2, 0.1); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, true, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, true, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, true, 0.33, 0.033); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, true, 0.33, 0.033); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, false, 0.5, 0.5); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, false, 0.5, 0.5); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, false, 0.2, 0.1); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, false, 0.2, 0.1); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, false, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, false, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, true, false, 0.33, 0.033); },
        [](BSDF* bsdf) -> void
        { createMicrofacet(bsdf, false, false, 0.33, 0.033); },
        [](BSDF* bsdf) -> void
        { createFresnelBlend(bsdf, true, true, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createFresnelBlend(bsdf, false, true, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createFresnelBlend(bsdf, true, false, 0.15, 0.25); },
        [](BSDF* bsdf) -> void
        { createFresnelBlend(bsdf, false, false, 0.15, 0.25); },
    };

    const char* BSDFFuncDescripArray[] = {
        "Lambertian",
        "Oren Nayar (sigma 0)",
        "Oren Nayar (sigma 20)",
        "Beckmann (roughness 0.5, sample visible mf area)",
        "Trowbridge-Reitz (roughness 0.5, sample visible mf area)",
        "Beckmann (roughness 0.2/0.1, sample visible mf area)",
        "Trowbridge-Reitz (roughness 0.2/0.1, sample visible mf area)",
        "Beckmann (roughness 0.15/0.25, sample visible mf area)",
        "Trowbridge-Reitz (roughness 0.15/0.25, sample visible mf area)",
        "Beckmann (roughness 0.33/0.033, sample visible mf area)",
        "Trowbridge-Reitz (roughness 0.33/0.033, sample visible mf area)",
        "Beckmann (roughness 0.5, traditional sample wh)",
        "Trowbridge-Reitz (roughness 0.5, traditional sample wh)",
        "Beckmann (roughness 0.2/0.1, traditional sample wh)",
        "Trowbridge-Reitz (roughness 0.2/0.1, traditional sample wh)",
        "Beckmann (roughness 0.15/0.25, traditional sample wh)",
        "Trowbridge-Reitz (roughness 0.15/0.25, traditional sample wh)",
        "Beckmann (roughness 0.33/0.033, traditional sample wh)",
        "Trowbridge-Reitz (roughness 0.33/0.033, traditional sample wh)",
        "Fresnel Blend Beckmann (roughness 0.15/0.25, sample visible mf area)",
        "Fresnel Blend Trowbridge-Reitz (roughness 0.15/0.25, sample visible mf area)",
        "Fresnel Blend Beckmann (roughness 0.15/0.25, traditional sample wh)",
        "Fresnel Blend Trowbridge-Reitz (roughness 0.15/0.25, traditional sample wh)",
    };

    GenSampleFunc SampleFuncArray[] = {
        Gen_Sample_f,
        // CO        Gen_CosHemisphere,
        // CO        Gen_UniformHemisphere,
    };

    const char* SampleFuncDescripArray[] = {
        "BSDF Importance Sampling",
        // CO        "Cos Hemisphere",
        // CO        "Uniform Hemisphere",
    };

    int numModels = sizeof(BSDFFuncArray) / sizeof(BSDFFuncArray[0]);
    int numModelsDescrip =
        sizeof(BSDFFuncDescripArray) / sizeof(BSDFFuncDescripArray[0]);
    int numGenerators = sizeof(SampleFuncArray) / sizeof(SampleFuncArray[0]);
    int numGeneratorsDescrip =
        sizeof(SampleFuncDescripArray) / sizeof(SampleFuncDescripArray[0]);

    if (numModels != numModelsDescrip) {
        fprintf(stderr,
                "BSDFFuncArray and BSDFFuncDescripArray out of sync!\n");
        exit(1);
    }

    if (numGenerators != numGeneratorsDescrip) {
        fprintf(stderr,
                "SampleFuncArray and SampleFuncDescripArray out of sync!\n");
        exit(1);
    }

    // for each bsdf model
    for (int model = 0; model < numModels; model++) {
        BSDF* bsdf;

        // create BSDF which requires creating a Shape, casting a Ray
        // that hits the shape to get a SurfaceInteraction object.
        {
            Transform t = RotateX(-90);
            bool reverseOrientation = false;
            ParamSet p;

            std::shared_ptr<Shape> disk(
                new Disk(new Transform(t), new Transform(Inverse(t)),
                         reverseOrientation, 0., 1., 0, 360.));
            Point3f origin(
                0.1, 1, 0);  // offset slightly so we don't hit center of disk
            Vector3f direction(0, -1, 0);
            Float tHit;
            Ray r(origin, direction);
            SurfaceInteraction isect;
            disk->Intersect(r, &tHit, &isect);
            bsdf = ARENA_ALLOC(arena, BSDF)(isect);
            (BSDFFuncArray[model])(bsdf);
        }

        // facing directly at normal
        Vector3f woL = Normalize(Vector3f(0, 0, 1));
        Vector3f wo = bsdf->LocalToWorld(woL);
        // was bsdf->dgShading.nn
        const Normal3f n = Normal3f(bsdf->LocalToWorld(Vector3f(0, 0, 1)));

        // for each method of generating samples over the hemisphere
        for (int gen = 0; gen < numGenerators; gen++) {
            double redSum = 0.0;

            const int numHistoBins = 10;
            double histogram[numHistoBins][numHistoBins];
            for (int i = 0; i < numHistoBins; i++) {
                for (int j = 0; j < numHistoBins; j++) {
                    histogram[i][j] = 0;
                }
            }
            int badSamples = 0;
            int outsideSamples = 0;

            int warningTarget = 1;
            for (int sample = 0; sample < estimates; sample++) {
                Vector3f wi;
                Float pdf;
                Spectrum f;

                // sample hemisphere around bsdf, wo is fixed
                (SampleFuncArray[gen])(bsdf, wo, &wi, &pdf, &f);

                double redF = spectrumRedValue(f);

                // add hemisphere sample to histogram
                Vector3f wiL = bsdf->WorldToLocal(wi);
                float x = Clamp(wiL.x, -1.f, 1.f);
                float y = Clamp(wiL.y, -1.f, 1.f);
                float wiPhi = (atan2(y, x) + Pi) / (2.0 * Pi);
                float wiCosTheta = wiL.z;
                bool validSample = (wiCosTheta > 1e-7);
                if (wiPhi < -0.0001 || wiPhi > 1.0001 || wiCosTheta > 1.0001) {
                    // wiCosTheta can be less than 0
                    fprintf(stderr, "bad wi! %.3f %.3f %.3f, (%.3f %.3f)\n",
                            wiL[0], wiL[1], wiL[2], wiPhi, wiCosTheta);
                } else if (validSample) {
                    int histoPhi = (int)(wiPhi * numHistoBins);
                    if (histoPhi == numHistoBins)
                      --histoPhi;
                    int histoCosTheta = (int)(wiCosTheta * numHistoBins);
                    if (histoCosTheta == numHistoBins)
                      --histoCosTheta;
                    assert(histoPhi >= 0 && histoPhi < numHistoBins);
                    assert(histoCosTheta >= 0 && histoCosTheta < numHistoBins);
                    histogram[histoCosTheta][histoPhi] += 1.0 / pdf;
                }

                if (!validSample) {
                    outsideSamples++;
                } else if (pdf == 0.f || std::isnan(pdf) || redF < 0 ||
                           std::isnan(redF)) {
                    if (badSamples == warningTarget) {
                        fprintf(stderr,
                                "warning %d, bad sample %d! "
                                "pdf: %.3f, redF: %.3f\n",
                                warningTarget, sample, pdf, redF);
                        warningTarget *= 10;
                    }
                    badSamples++;
                } else {
                    // outgoing radiance estimate =
                    //   bsdf * incomingRadiance * cos(wi) / pdf
                    redSum += redF * environmentRadiance * AbsDot(wi, n) / pdf;
                }
            }
            int goodSamples = estimates - badSamples;

            // print results
            fprintf(stderr,
                    "*** BRDF: '%s', Samples: '%s'\n\n"
                    "wi histogram showing the relative weight in each bin\n"
                    "  all entries should be close to 2pi = %.5f:\n"
                    "  (%d bad samples, %d outside samples)\n\n"
                    "                          phi bins\n",
                    BSDFFuncDescripArray[model], SampleFuncDescripArray[gen],
                    Pi * 2.0, badSamples, outsideSamples);
            double totalSum = 0.0;
            for (int i = 0; i < numHistoBins; i++) {
                fprintf(stderr, "  cos(theta) bin %02d:", i);
                for (int j = 0; j < numHistoBins; j++) {
                    fprintf(stderr, " %5.2f", histogram[i][j] * numHistoBins *
                                                  numHistoBins / goodSamples);
                    totalSum += histogram[i][j];
                }
                fprintf(stderr, "\n");
            }
            fprintf(stderr,
                    "\n  final average :  %.5f (error %.5f)\n\n"
                    "  radiance = %.5f\n\n",
                    totalSum / goodSamples, totalSum / goodSamples - Pi * 2.0,
                    redSum / goodSamples);
        }
    }

    pbrtCleanup();
    return 0;
}

void Gen_Sample_f(BSDF* bsdf, const Vector3f& wo, Vector3f* wi, Float* pdf,
                  Spectrum* f) {
    // only glossy or diffuse reflections (no specular reflections)
    BxDFType inflags = BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE | BSDF_GLOSSY);
    BxDFType outflags;
    Point2f sample {rng.UniformFloat(), rng.UniformFloat()};
    *f = bsdf->Sample_f(wo, wi, sample, pdf, inflags, &outflags);

    // double check bsdf->Pdf() gives us the same answer
    Vector3f wiL = bsdf->WorldToLocal(*wi);
    float wiCosTheta = wiL.z;
    bool validSample = (wiCosTheta > 1e-7);

    if (validSample) {
        float verifyPdf = bsdf->Pdf(wo, *wi, inflags);
        float relErr = std::abs(verifyPdf - *pdf) / *pdf;
        if (relErr > 1e-3) {
            fprintf(stderr,
                    "BSDF::Pdf() doesn't match BSDF::Sample_f() !\n"
                    "  Sample_f pdf %.3f, Pdf pdf %.3f (rel error %f)\n"
                    "  wo %.3f %.3f %.3f, wi %.3f %.3f %.3f\n",
                    *pdf, verifyPdf, relErr, wo[0], wo[1], wo[2], (*wi)[0],
                    (*wi)[1], (*wi)[2]);
            fprintf(
                stderr,
                "blah! validSample %d, wiCosTheta %.3f, wiL %.3f %.3f %.3f\n",
                validSample, wiCosTheta, wiL[0], wiL[1], wiL[2]);
        }
    }
}

void Gen_CosHemisphere(BSDF* bsdf, const Vector3f& wo, Vector3f* wi, Float* pdf,
                       Spectrum* f) {
    float u1 = rng.UniformFloat();
    float u2 = rng.UniformFloat();
    Vector3f wiL = CosineSampleHemisphere(Point2f(u1, u2));
    *wi = bsdf->LocalToWorld(wiL);
    float cosTheta = wiL.z;
    *pdf = CosineHemispherePdf(cosTheta);

    *f = bsdf->f(wo, *wi);
}

void Gen_UniformHemisphere(BSDF* bsdf, const Vector3f& wo, Vector3f* wi,
                           Float* pdf, Spectrum* f) {
    float u1 = rng.UniformFloat();
    float u2 = rng.UniformFloat();
    Vector3f wiL = UniformSampleHemisphere(Point2f(u1, u2));
    *wi = bsdf->LocalToWorld(wiL);
    *pdf = UniformHemispherePdf();

    *f = bsdf->f(wo, *wi);
}

void createLambertian(BSDF* bsdf) {
    Spectrum Kd(1);
    bsdf->Add(ARENA_ALLOC(arena, LambertianReflection)(Kd));
}

void createMicrofacet(BSDF* bsdf, bool beckmann, bool samplevisible,
                      float roughx, float roughy) {
    Spectrum Ks(1);
    MicrofacetDistribution *distrib;
    if (beckmann) {
      Float alphax = BeckmannDistribution::RoughnessToAlpha(roughx);
      Float alphay = BeckmannDistribution::RoughnessToAlpha(roughy);
      distrib = ARENA_ALLOC(arena, BeckmannDistribution)(alphax, alphay,
                                                        samplevisible);
    }
    else {
      Float alphax = TrowbridgeReitzDistribution::RoughnessToAlpha(roughx);
      Float alphay = TrowbridgeReitzDistribution::RoughnessToAlpha(roughy);
      distrib = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(alphax, alphay,
                                                               samplevisible);
    }
    Fresnel* fresnel = ARENA_ALLOC(arena, FresnelNoOp)();
    BxDF* bxdf = ARENA_ALLOC(arena, MicrofacetReflection)(Ks, distrib, fresnel);
    bsdf->Add(bxdf);
}

void createFresnelBlend(BSDF* bsdf, bool beckmann, bool samplevisible,
                        float roughx, float roughy)
{
    Spectrum d(0.5);
    Spectrum s(0.5);
    MicrofacetDistribution *distrib;
    if (beckmann) {
      Float alphax = BeckmannDistribution::RoughnessToAlpha(roughx);
      Float alphay = BeckmannDistribution::RoughnessToAlpha(roughy);
      distrib = ARENA_ALLOC(arena, BeckmannDistribution)(alphax, alphay,
                                                        samplevisible);
    }
    else {
      Float alphax = TrowbridgeReitzDistribution::RoughnessToAlpha(roughx);
      Float alphay = TrowbridgeReitzDistribution::RoughnessToAlpha(roughy);
      distrib = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(alphax, alphay,
                                                               samplevisible);
    }
    BxDF* bxdf = ARENA_ALLOC(arena, FresnelBlend)(d, s, distrib);
    bsdf->Add(bxdf);
}

void createMicrofacet30and0(BSDF* bsdf, bool beckmann) {
    Spectrum Ks(0.5);
    MicrofacetDistribution *distrib1, *distrib2;
    if (beckmann) {
      Float alphax = BeckmannDistribution::RoughnessToAlpha(0.8);
      Float alphay = BeckmannDistribution::RoughnessToAlpha(0.8);
      distrib1 = ARENA_ALLOC(arena, BeckmannDistribution)(alphax, alphay);

      alphax = BeckmannDistribution::RoughnessToAlpha(0.01);
      alphay = BeckmannDistribution::RoughnessToAlpha(0.01);
      distrib2 = ARENA_ALLOC(arena, BeckmannDistribution)(alphax, alphay);
    }
    else {
      Float alphax = TrowbridgeReitzDistribution::RoughnessToAlpha(0.8);
      Float alphay = TrowbridgeReitzDistribution::RoughnessToAlpha(0.8);
      distrib1 = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(alphax, alphay);

      alphax = TrowbridgeReitzDistribution::RoughnessToAlpha(0.01);
      alphay = TrowbridgeReitzDistribution::RoughnessToAlpha(0.01);
      distrib2 = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(alphax, alphay);
    }

    Fresnel* fresnel = ARENA_ALLOC(arena, FresnelNoOp)();
    BxDF* bxdf1 =
        ARENA_ALLOC(arena, MicrofacetReflection)(Ks, distrib1, fresnel);
    bsdf->Add(bxdf1);
    BxDF* bxdf2 =
        ARENA_ALLOC(arena, MicrofacetReflection)(Ks, distrib2, fresnel);
    bsdf->Add(bxdf2);
}

void createOrenNayar0(BSDF* bsdf) {
    Spectrum Kd(1);
    float sigma = 0.0;
    BxDF* bxdf = ARENA_ALLOC(arena, OrenNayar)(Kd, sigma);
    bsdf->Add(bxdf);
}

void createOrenNayar20(BSDF* bsdf) {
    Spectrum Kd(1);
    float sigma = 20.0;
    BxDF* bxdf = ARENA_ALLOC(arena, OrenNayar)(Kd, sigma);
    bsdf->Add(bxdf);
}
