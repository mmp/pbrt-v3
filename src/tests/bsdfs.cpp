
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <functional>

#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "reflection.h"
#include "sampling.h"
#include "memory.h"
#include "api.h"
#include "paramset.h"
#include "shapes/disk.h"

/* The null hypothesis will be rejected when the associated
   p-value is below the significance level specified here. */
#define CHI2_SLEVEL 0.01

/* Resolution of the frequency table discretization. The azimuthal
   resolution is twice this value. */
#define CHI2_THETA_RES 10
#define CHI2_PHI_RES 2 * CHI2_THETA_RES

/* Number of MC samples to compute the observed frequency table */
#define CHI2_SAMPLECOUNT 1000000

/* Minimum expected bin frequency. The chi^2 test does not
   work reliably when the expected frequency in a cell is
   low (e.g. less than 5), because normality assumptions
   break down in this case. Therefore, the implementation
   will merge such low-frequency cells when they fall below
   the threshold specified here. */
#define CHI2_MINFREQ 5

/* Each provided BSDF will be tested for a few different
   incident directions. The value specified here determines
   how many tests will be executed per BSDF */
#define CHI2_RUNS 5

/// Regularized lower incomplete gamma function (based on code from Cephes)
double RLGamma(double a, double x) {
    const double epsilon = 0.000000000000001;
    const double big = 4503599627370496.0;
    const double bigInv = 2.22044604925031308085e-16;
    if (a < 0 || x < 0)
        throw std::runtime_error("LLGamma: invalid arguments range!");

    if (x == 0) return 0.0f;

    double ax = (a * std::log(x)) - x - std::lgamma(a);
    if (ax < -709.78271289338399) return a < x ? 1.0 : 0.0;

    if (x <= 1 || x <= a) {
        double r2 = a;
        double c2 = 1;
        double ans2 = 1;

        do {
            r2 = r2 + 1;
            c2 = c2 * x / r2;
            ans2 += c2;
        } while ((c2 / ans2) > epsilon);

        return std::exp(ax) * ans2 / a;
    }

    int c = 0;
    double y = 1 - a;
    double z = x + y + 1;
    double p3 = 1;
    double q3 = x;
    double p2 = x + 1;
    double q2 = z * x;
    double ans = p2 / q2;
    double error;

    do {
        c++;
        y += 1;
        z += 2;
        double yc = y * c;
        double p = (p2 * z) - (p3 * yc);
        double q = (q2 * z) - (q3 * yc);

        if (q != 0) {
            double nextans = p / q;
            error = std::abs((ans - nextans) / nextans);
            ans = nextans;
        } else {
            // zero div, skip
            error = 1;
        }

        // shift
        p3 = p2;
        p2 = p;
        q3 = q2;
        q2 = q;

        // normalize fraction when the numerator becomes large
        if (std::abs(p) > big) {
            p3 *= bigInv;
            p2 *= bigInv;
            q3 *= bigInv;
            q2 *= bigInv;
        }
    } while (error > epsilon);

    return 1.0 - (std::exp(ax) * ans);
}

/// Chi^2 distribution cumulative distribution function
double Chi2CDF(double x, int dof) {
    if (dof < 1 || x < 0) {
        return 0.0;
    } else if (dof == 2) {
        return 1.0 - std::exp(-0.5 * x);
    } else {
        return (Float)RLGamma(0.5 * dof, 0.5 * x);
    }
}

/// Adaptive Simpson integration over an 1D interval
Float AdaptiveSimpson(const std::function<Float(Float)>& f, Float x0, Float x1,
                      Float eps = 1e-6f, int depth = 6) {
    int count = 0;
    /* Define an recursive lambda function for integration over subintervals */
    std::function<Float(Float, Float, Float, Float, Float, Float, Float, Float,
                        int)> integrate = [&](Float a, Float b, Float c,
                                              Float fa, Float fb, Float fc,
                                              Float I, Float eps, int depth) {
        /* Evaluate the function at two intermediate points */
        Float d = 0.5f * (a + b), e = 0.5f * (b + c), fd = f(d), fe = f(e);

        /* Simpson integration over each subinterval */
        Float h = c - a, I0 = (Float)(1.0 / 12.0) * h * (fa + 4 * fd + fb),
              I1 = (Float)(1.0 / 12.0) * h * (fb + 4 * fe + fc), Ip = I0 + I1;
        ++count;

        /* Stopping criterion from J.N. Lyness (1969)
          "Notes on the adaptive Simpson quadrature routine" */
        if (depth <= 0 || std::abs(Ip - I) < 15 * eps) {
            // Richardson extrapolation
            return Ip + (Float)(1.0 / 15.0) * (Ip - I);
        }

        return integrate(a, d, b, fa, fd, fb, I0, .5f * eps, depth - 1) +
               integrate(b, e, c, fb, fe, fc, I1, .5f * eps, depth - 1);
    };
    Float a = x0, b = 0.5f * (x0 + x1), c = x1;
    Float fa = f(a), fb = f(b), fc = f(c);
    Float I = (c - a) * (Float)(1.0 / 6.0) * (fa + 4 * fb + fc);
    return integrate(a, b, c, fa, fb, fc, I, eps, depth);
}

/// Nested adaptive Simpson integration over a 2D rectangle
Float AdaptiveSimpson2D(const std::function<Float(Float, Float)>& f, Float x0,
                        Float y0, Float x1, Float y1, Float eps = 1e-6f,
                        int depth = 6) {
    /* Lambda function that integrates over the X axis */
    auto integrate = [&](Float y) {
        return AdaptiveSimpson(std::bind(f, std::placeholders::_1, y), x0, x1,
                               eps, depth);
    };
    Float value = AdaptiveSimpson(integrate, y0, y1, eps, depth);
    return value;
}

/// Generate a histogram of the BSDF density function via MC sampling
void FrequencyTable(const BSDF* bsdf, const Vector3f& wo, RNG& rng,
                    int sampleCount, int thetaRes, int phiRes, Float* target) {
    memset(target, 0, thetaRes * phiRes * sizeof(Float));

    Float factorTheta = thetaRes / Pi, factorPhi = phiRes / (2 * Pi);

    BxDFType flags;
    Vector3f wi;
    Float pdf;

    for (int i = 0; i < sampleCount; ++i) {
        Point2f sample(rng.UniformFloat(), rng.UniformFloat());

        Spectrum f = bsdf->Sample_f(wo, &wi, sample, &pdf, BSDF_ALL, &flags);

        if (f == Spectrum() || (flags & BSDF_SPECULAR)) continue;

        Vector3f wiL = bsdf->WorldToLocal(wi);

        Point2f coords(std::acos(wiL.z) * factorTheta,
                       std::atan2(wiL.y, wiL.x) * factorPhi);

        if (coords.y < 0) coords.y += 2 * Pi * factorPhi;

        int thetaBin =
            std::min(std::max(0, (int)std::floor(coords.x)), thetaRes - 1);
        int phiBin =
            std::min(std::max(0, (int)std::floor(coords.y)), phiRes - 1);

        target[thetaBin * phiRes + phiBin] += 1;
    }
}

// Numerically integrate the probability density function over rectangles in
// spherical coordinates.
void IntegrateFrequencyTable(const BSDF* bsdf, const Vector3f& wo,
                             int sampleCount, int thetaRes, int phiRes,
                             Float* target) {
    memset(target, 0, thetaRes * phiRes * sizeof(Float));

    Float factorTheta = Pi / thetaRes, factorPhi = (2 * Pi) / phiRes;

    for (int i = 0; i < thetaRes; ++i) {
        for (int j = 0; j < phiRes; ++j) {
            *target++ = sampleCount *
                        AdaptiveSimpson2D(
                            [&](Float theta, Float phi) -> Float {
                                Float cosTheta = std::cos(theta),
                                      sinTheta = std::sin(theta);
                                Float cosPhi = std::cos(phi),
                                      sinPhi = std::sin(phi);
                                Vector3f wiL(sinTheta * cosPhi,
                                             sinTheta * sinPhi, cosTheta);
                                return bsdf->Pdf(wo, bsdf->LocalToWorld(wiL),
                                                 BSDF_ALL) *
                                       sinTheta;
                            },
                            i* factorTheta, j* factorPhi, (i + 1) * factorTheta,
                            (j + 1) * factorPhi);
        }
    }
}

/// Write the frequency tables to disk in a format that is nicely plottable by
/// Octave and MATLAB
void DumpTables(const Float* frequencies, const Float* expFrequencies,
                int thetaRes, int phiRes, const char* filename) {
    std::ofstream f(filename);

    f << "frequencies = [ ";
    for (int i = 0; i < thetaRes; ++i) {
        for (int j = 0; j < phiRes; ++j) {
            f << frequencies[i * phiRes + j];
            if (j + 1 < phiRes) f << ", ";
        }
        if (i + 1 < thetaRes) f << "; ";
    }
    f << " ];" << std::endl << "expFrequencies = [ ";
    for (int i = 0; i < thetaRes; ++i) {
        for (int j = 0; j < phiRes; ++j) {
            f << expFrequencies[i * phiRes + j];
            if (j + 1 < phiRes) f << ", ";
        }
        if (i + 1 < thetaRes) f << "; ";
    }
    f << " ];" << std::endl
      << "colormap(jet);" << std::endl
      << "clf; subplot(2,1,1);" << std::endl
      << "imagesc(frequencies);" << std::endl
      << "title('Observed frequencies');" << std::endl
      << "axis equal;" << std::endl
      << "subplot(2,1,2);" << std::endl
      << "imagesc(expFrequencies);" << std::endl
      << "axis equal;" << std::endl
      << "title('Expected frequencies');" << std::endl;
    f.close();
}

/// Run A Chi^2 test based on the given frequency tables
std::pair<bool, std::string> Chi2Test(const Float* frequencies,
                                      const Float* expFrequencies, int thetaRes,
                                      int phiRes, int sampleCount,
                                      Float minExpFrequency,
                                      Float significanceLevel, int numTests) {
    struct Cell {
        Float expFrequency;
        size_t index;
    };

    /* Sort all cells by their expected frequencies */
    std::vector<Cell> cells(thetaRes * phiRes);
    for (size_t i = 0; i < cells.size(); ++i) {
        cells[i].expFrequency = expFrequencies[i];
        cells[i].index = i;
    }
    std::sort(cells.begin(), cells.end(), [](const Cell& a, const Cell& b) {
        return a.expFrequency < b.expFrequency;
    });

    /* Compute the Chi^2 statistic and pool cells as necessary */
    Float pooledFrequencies = 0, pooledExpFrequencies = 0, chsq = 0;
    int pooledCells = 0, dof = 0;

    for (const Cell& c : cells) {
        if (expFrequencies[c.index] == 0) {
            if (frequencies[c.index] > sampleCount * 1e-5f) {
                /* Uh oh: samples in a c that should be completely empty
                   according to the probability density function. Ordinarily,
                   even a single sample requires immediate rejection of the null
                   hypothesis. But due to finite-precision computations and
                   rounding
                   errors, this can occasionally happen without there being an
                   actual bug. Therefore, the criterion here is a bit more
                   lenient. */

                char result[256];
                sprintf(result,
                        "Encountered %f samples in a c with expected "
                        "frequency 0. Rejecting the null hypothesis!",
                        frequencies[c.index]);
                return std::make_pair(false, std::string(result));
            }
        } else if (expFrequencies[c.index] < minExpFrequency) {
            /* Pool cells with low expected frequencies */
            pooledFrequencies += frequencies[c.index];
            pooledExpFrequencies += expFrequencies[c.index];
            pooledCells++;
        } else if (pooledExpFrequencies > 0 &&
                   pooledExpFrequencies < minExpFrequency) {
            /* Keep on pooling cells until a sufficiently high
               expected frequency is achieved. */
            pooledFrequencies += frequencies[c.index];
            pooledExpFrequencies += expFrequencies[c.index];
            pooledCells++;
        } else {
            Float diff = frequencies[c.index] - expFrequencies[c.index];
            chsq += (diff * diff) / expFrequencies[c.index];
            ++dof;
        }
    }

    if (pooledExpFrequencies > 0 || pooledFrequencies > 0) {
        Float diff = pooledFrequencies - pooledExpFrequencies;
        chsq += (diff * diff) / pooledExpFrequencies;
        ++dof;
    }

    /* All parameters are assumed to be known, so there is no
       additional DF reduction due to model parameters */
    dof -= 1;

    if (dof <= 0) {
        char result[256];
        sprintf(result, "The number of degrees of freedom %d is too low!", dof);
        return std::make_pair(false, std::string(result));
    }

    /* Probability of obtaining a test statistic at least
       as extreme as the one observed under the assumption
       that the distributions match */
    Float pval = 1 - (Float)Chi2CDF(chsq, dof);

    /* Apply the Sidak correction term, since we'll be conducting multiple
       independent
       hypothesis tests. This accounts for the fact that the probability of a
       failure
       increases quickly when several hypothesis tests are run in sequence. */
    Float alpha = 1.0f - std::pow(1.0f - significanceLevel, 1.0f / numTests);

    if (pval < alpha || !std::isfinite(pval)) {
        char result[512];
        sprintf(result,
                "Rejected the null hypothesis (p-value = %f, "
                "significance level = %f",
                pval, alpha);
        return std::make_pair(false, std::string(result));
    } else {
        return std::make_pair(true, std::string(""));
    }
}

void TestBSDF(void (*createBSDF)(BSDF*, MemoryArena&),
              const char* description) {
    MemoryArena arena;

    Options opt;
    pbrtInit(opt);

    const int thetaRes = CHI2_THETA_RES;
    const int phiRes = CHI2_PHI_RES;
    const int sampleCount = CHI2_SAMPLECOUNT;
    Float* frequencies = new Float[thetaRes * phiRes];
    Float* expFrequencies = new Float[thetaRes * phiRes];
    RNG rng;

    int index = 0;
    std::cout.precision(3);

    // Create BSDF, which requires creating a Shape, casting a Ray that
    // hits the shape to get a SurfaceInteraction object.
    BSDF* bsdf = nullptr;
    {
        Transform t = RotateX(-90);
        bool reverseOrientation = false;
        ParamSet p;

        std::shared_ptr<Shape> disk(
            new Disk(new Transform(t), new Transform(Inverse(t)),
                     reverseOrientation, 0., 1., 0, 360.));
        Point3f origin(0.1, 1,
                       0);  // offset slightly so we don't hit center of disk
        Vector3f direction(0, -1, 0);
        Float tHit;
        Ray r(origin, direction);
        SurfaceInteraction isect;
        disk->Intersect(r, &tHit, &isect);
        bsdf = ARENA_ALLOC(arena, BSDF)(isect);
        createBSDF(bsdf, arena);
    }

    for (int k = 0; k < CHI2_RUNS; ++k) {
        /* Randomly pick an outgoing direction on the hemisphere */
        Point2f sample(rng.UniformFloat(), rng.UniformFloat());
        Vector3f woL = CosineSampleHemisphere(sample);
        Vector3f wo = bsdf->LocalToWorld(woL);

        FrequencyTable(bsdf, wo, rng, sampleCount, thetaRes, phiRes,
                       frequencies);

        IntegrateFrequencyTable(bsdf, wo, sampleCount, thetaRes, phiRes,
                                expFrequencies);

        char filename[256];
        snprintf(filename, sizeof(filename), "/tmp/chi2test_%s_%03i.m",
                 description, ++index);
        DumpTables(frequencies, expFrequencies, thetaRes, phiRes, filename);

        auto result =
            Chi2Test(frequencies, expFrequencies, thetaRes, phiRes, sampleCount,
                     CHI2_MINFREQ, CHI2_SLEVEL, CHI2_RUNS);
        EXPECT_TRUE(result.first) << result.second << ", iteration " << k;
    }

    delete[] frequencies;
    delete[] expFrequencies;

    pbrtCleanup();
}

void createLambertian(BSDF* bsdf, MemoryArena& arena) {
    Spectrum Kd(1);
    bsdf->Add(ARENA_ALLOC(arena, LambertianReflection)(Kd));
}

void createMicrofacet(BSDF* bsdf, MemoryArena& arena, bool beckmann,
                      bool samplevisible, float roughx, float roughy) {
    Spectrum Ks(1);
    MicrofacetDistribution* distrib;
    if (beckmann) {
        Float alphax = BeckmannDistribution::RoughnessToAlpha(roughx);
        Float alphay = BeckmannDistribution::RoughnessToAlpha(roughy);
        distrib = ARENA_ALLOC(arena, BeckmannDistribution)(alphax, alphay,
                                                           samplevisible);
    } else {
        Float alphax = TrowbridgeReitzDistribution::RoughnessToAlpha(roughx);
        Float alphay = TrowbridgeReitzDistribution::RoughnessToAlpha(roughy);
        distrib = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(
            alphax, alphay, samplevisible);
    }
    Fresnel* fresnel = ARENA_ALLOC(arena, FresnelNoOp)();
    BxDF* bxdf = ARENA_ALLOC(arena, MicrofacetReflection)(Ks, distrib, fresnel);
    bsdf->Add(bxdf);
}

void createFresnelBlend(BSDF* bsdf, MemoryArena& arena, bool beckmann,
                        bool samplevisible, float roughx, float roughy) {
    Spectrum d(0.5);
    Spectrum s(0.5);
    MicrofacetDistribution* distrib;
    if (beckmann) {
        Float alphax = BeckmannDistribution::RoughnessToAlpha(roughx);
        Float alphay = BeckmannDistribution::RoughnessToAlpha(roughy);
        distrib = ARENA_ALLOC(arena, BeckmannDistribution)(alphax, alphay,
                                                           samplevisible);
    } else {
        Float alphax = TrowbridgeReitzDistribution::RoughnessToAlpha(roughx);
        Float alphay = TrowbridgeReitzDistribution::RoughnessToAlpha(roughy);
        distrib = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(
            alphax, alphay, samplevisible);
    }
    BxDF* bxdf = ARENA_ALLOC(arena, FresnelBlend)(d, s, distrib);
    bsdf->Add(bxdf);
}

TEST(BSDFSampling, Lambertian) { TestBSDF(createLambertian, "Lambertian"); }

TEST(BSDFSampling, Beckmann_VA_0p5) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, true, true, 0.5, 0.5);
    }, "Beckmann, visible area sample, alpha = 0.5");
}

TEST(BSDFSampling, TR_VA_0p5) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, false, true, 0.5, 0.5);
    }, "Trowbridge-Reitz, visible area sample, alpha = 0.5");
}

TEST(BSDFSampling, Beckmann_std_0p5) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, true, false, 0.5, 0.5);
    }, "Beckmann, std sample, alpha = 0.5");
}

TEST(BSDFSampling, TR_std_0p5) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, false, false, 0.5, 0.5);
    }, "Trowbridge-Reitz, std sample, alpha = 0.5");
}

TEST(BSDFSampling, Beckmann_VA_0p2_0p1) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, true, true, 0.2, 0.1);
    }, "Beckmann, visible area sample, alpha = 0.2/0.1");
}

TEST(BSDFSampling, TR_VA_0p3_0p15) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, false, true, 0.3, 0.15);
    }, "Trowbridge-Reitz, visible area sample, alpha = 0.3/0.15");
}

TEST(BSDFSampling, Beckmann_std_0p2_0p1) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, true, false, 0.2, 0.1);
    }, "Beckmann, std sample, alpha = 0.2/0.1");
}

TEST(BSDFSampling, TR_std_0p2_0p1) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createMicrofacet(bsdf, arena, false, false, 0.2, 0.1);
    }, "Trowbridge-Reitz, std sample, alpha = 0.2/0.1");
}

TEST(BSDFSampling, Beckmann_VA_0p4_0p3) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createFresnelBlend(bsdf, arena, true, true, 0.4, 0.3);
    }, "Fresnel blend Beckmann, visible area sample, alpha = 0.4/0.3");
}

TEST(BSDFSampling, TR_VA_0p3) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createFresnelBlend(bsdf, arena, false, true, 0.3, 0.3);
    }, "Fresnel blend Trowbridge-Reitz, visible area sample, alpha = 0.3");
}

TEST(BSDFSampling, Beckmann_std_0p2) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createFresnelBlend(bsdf, arena, true, false, 0.2, 0.2);
    }, "Fresnel blend Beckmann, std sample, alpha = 0.2");
}

TEST(BSDFSampling, TR_std_0p05_0p1) {
    TestBSDF([](BSDF* bsdf, MemoryArena& arena) -> void {
        createFresnelBlend(bsdf, arena, false, false, 0.05, 0.1);
    }, "Fresnel blend Trowbridge-Reitz, std sample, alpha = 0.05/0.1");
}
