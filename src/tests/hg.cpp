
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "rng.h"
#include "sampling.h"
#include "medium.h"

using namespace pbrt;

TEST(HenyeyGreenstein, SamplingMatch) {
    RNG rng;
    for (float g = -.75; g <= 0.75; g += 0.25) {
        HenyeyGreenstein hg(g);
        for (int i = 0; i < 100; ++i) {
            Vector3f wo =
                UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
            Vector3f wi;
            Point2f u {rng.UniformFloat(), rng.UniformFloat()};
            Float p0 = hg.Sample_p(wo, &wi, u);
            // Phase function is normalized, and the sampling method should be
            // exact.
            EXPECT_NEAR(p0, hg.p(wo, wi), 1e-4f) << "Failure with g = " << g;
        }
    }
}

TEST(HenyeyGreenstein, SamplingOrientationForward) {
    RNG rng;

    HenyeyGreenstein hg(0.95);
    Vector3f wo(-1, 0, 0);
    int nForward = 0, nBackward = 0;
    for (int i = 0; i < 100; ++i) {
        Point2f u {rng.UniformFloat(), rng.UniformFloat()};
        Vector3f wi;
        hg.Sample_p(wo, &wi, u);
        if (wi.x > 0)
            ++nForward;
        else
            ++nBackward;
    }
    // With g = 0.95, almost all of the samples should have wi.x > 0.
    EXPECT_GE(nForward, 10 * nBackward);
}

TEST(HenyeyGreenstein, SamplingOrientationBackward) {
    RNG rng;

    HenyeyGreenstein hg(-0.95);
    Vector3f wo(-1, 0, 0);
    int nForward = 0, nBackward = 0;
    for (int i = 0; i < 100; ++i) {
        Point2f u {rng.UniformFloat(), rng.UniformFloat()};
        Vector3f wi;
        hg.Sample_p(wo, &wi, u);
        if (wi.x > 0)
            ++nForward;
        else
            ++nBackward;
    }
    // With g = -0.95, almost all of the samples should have wi.x < 0.
    EXPECT_GE(nBackward, 10 * nForward);
}

TEST(HenyeyGreenstein, Normalized) {
    RNG rng;
    for (float g = -.75; g <= 0.75; g += 0.25) {
        HenyeyGreenstein hg(g);
        Vector3f wo =
            UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
        Float sum = 0;
        int nSamples = 100000;
        for (int i = 0; i < nSamples; ++i) {
            Vector3f wi =
                UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
            sum += hg.p(wo, wi);
        }
        // Phase function should integrate to 1/4pi.
        EXPECT_NEAR(sum / nSamples, 1. / (4. * Pi), 1e-3f);
    }
}
