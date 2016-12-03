
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "rng.h"
#include "spectrum.h"

using namespace pbrt;

TEST(Spectrum, LinearUpsampleSimple) {
    // Linear SPD where val(lambda) = 1 + 2 * lambda.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = 5;

    // Resample at exactly the same points; should give back exactly the
    // original values.
    Float newVal[nIn];
    ResampleLinearSpectrum(lambda, val, nIn, lambda[0] /* lambdaMin */,
                     lambda[nIn - 1] /* lambdaMax */, nIn /* nOut */, newVal);

    for (int i = 0; i < 5; ++i) EXPECT_EQ(val[i], newVal[i]);
}

TEST(Spectrum, LinearUpsampleSubset) {
    // Linear SPD where val(lambda) = 1 + 2 * lambda.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = 5;

    // Resample at exactly the same rate, but over a subset of the input
    // samples; should give back exactly the original values for them.
    const int nOut = 3;
    Float newVal[nOut];
    ResampleLinearSpectrum(lambda, val, nIn, lambda[1] /* lambdaMin */,
                     lambda[3] /* lambdaMax */, nOut, newVal);

    for (int i = 0; i < 3; ++i) EXPECT_EQ(val[i + 1], newVal[i]);
}

TEST(Spectrum, LinearUpsample2x) {
    // Linear SPD where val(lambda) = 1 + 2 * lambda.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = 5;

    // Resample at 2x the rate, but same endpoints. Should exactly
    // reproduce the linear function.
    const int nOut = 9;
    Float newVal[nOut];
    ResampleLinearSpectrum(lambda, val, nIn, lambda[0], lambda[nIn - 1], nOut,
                     newVal);

    for (int i = 0; i < nOut; ++i) EXPECT_EQ(i + 1, newVal[i]);
}

TEST(Spectrum, LinearUpsampleHigher) {
    // Linear SPD where val(lambda) = 1 + 2 * lambda.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = 5;

    // Higher sampling rate, at subset of lambdas; should reproduce the
    // linear function (modulo floating-point roundoff error).
    const int nOut = 20;
    Float newVal[nOut];
    ResampleLinearSpectrum(lambda, val, nIn, lambda[1] /* lambdaMin */,
                     lambda[3] /* lambdaMax */, nOut, newVal);

    for (int i = 0; i < nOut; ++i) {
        Float t = i / Float(nOut - 1);
        EXPECT_FLOAT_EQ(Lerp(t, val[1], val[3]), newVal[i]);
    }
}

TEST(Spectrum, LinearUpsampling) {
    // Linear SPD where val(lambda) = 1 + 2 * lambda.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = 5;

    // Higher sampling rate, subset of lambdas; endpoints don't exactly
    // match.
    const int nOut = 40;
    Float newVal[nOut];
    const Float lambdaMin = 1.5, lambdaMax = 3.75;
    ResampleLinearSpectrum(lambda, val, nIn, lambdaMin, lambdaMax, nOut, newVal);

    for (int i = 0; i < nOut; ++i) {
        Float t = i / Float(nOut - 1);
        EXPECT_FLOAT_EQ(Lerp(t, 1 + 2 * lambdaMin, 1 + 2 * lambdaMax),
                        newVal[i]);
    }
}

TEST(Spectrum, LinearIrregularResample) {
    // Irregularly-sampled SPD, where f(lambda) = lambda^2.
    Float lambdaIrreg[] = {-1.5, -.5, .01, .6, 1,   2,     2.1, 3.4, 4.6,
                           5.7,  7,   8.2, 9,  9.8, 11.11, 12,  13,  14.7};
    const int nIn = sizeof(lambdaIrreg) / sizeof(lambdaIrreg[0]);
    Float valIrreg[nIn];
    for (int i = 0; i < nIn; ++i) valIrreg[i] = lambdaIrreg[i] * lambdaIrreg[i];

    // Resample it over a subset of the wavelengths.
    const int nOut = 30;
    const Float lambdaMin = -.5, lambdaMax = 14;
    Float newVal[nOut];
    ResampleLinearSpectrum(lambdaIrreg, valIrreg, nIn, lambdaMin, lambdaMax, nOut,
                     newVal);

    // The result should be generally close to lambda^2, though there will
    // be some differences due to the fact that in the places where we are
    // downsampling, we're averaging over a range of the quadratic
    // function.
    for (int i = 0; i < nOut; ++i) {
        Float t = i / Float(nOut - 1);
        Float lambda = Lerp(t, lambdaMin, lambdaMax);
        EXPECT_LT(std::abs(lambda * lambda - newVal[i]), .75);
    }
}

TEST(Spectrum, LinearDownsampleBasic) {
    // Another linear SPD.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = sizeof(lambda) / sizeof(lambda[0]);

    // Resample it with the same endpoints but at a lower sampling rate.
    const int nOut = 3;
    Float newVal[nOut];
    const Float lambdaMin = 0, lambdaMax = 4;
    ResampleLinearSpectrum(lambda, val, nIn, lambdaMin, lambdaMax, nOut, newVal);

    // We expect the computed values to be the averages over lambda=[-1,1],
    // then [1,3], then [3,5]. For the endpoints, recall that we model the
    // SPD beyond the specified endpoints as constant.
    EXPECT_FLOAT_EQ(1.5, newVal[0]);
    EXPECT_FLOAT_EQ(5, newVal[1]);
    EXPECT_FLOAT_EQ(8.5, newVal[2]);
}

TEST(Spectrum, LinearDownsampleOffset) {
    // Another linear SPD.
    Float lambda[5] = {0, 1, 2, 3, 4};
    Float val[5] = {1, 3, 5, 7, 9};
    const int nIn = sizeof(lambda) / sizeof(lambda[0]);

    const int nOut = 4;
    Float newVal[nOut];
    const Float lambdaMin = 0.5, lambdaMax = 3.5;
    ResampleLinearSpectrum(lambda, val, nIn, lambdaMin, lambdaMax, nOut, newVal);

    // The spacing between samples in the destination SPD is 1, so we expect
    // to get back averages over [0,1], [1,2], [2,3], and [3,4].
    EXPECT_FLOAT_EQ(2, newVal[0]);
    EXPECT_FLOAT_EQ(4, newVal[1]);
    EXPECT_FLOAT_EQ(6, newVal[2]);
    EXPECT_FLOAT_EQ(8, newVal[3]);
}

TEST(Spectrum, LinearDownsampleIrreg) {
    // Generate a very irregular set of lambda values starting at -25, where
    // the SPD is f(lambda) = lambda^2.
    RNG rng;
    std::vector<Float> lambdaIrreg, valIrreg;
    lambdaIrreg.push_back(-25);
    for (int i = 0; i < 100; ++i)
        lambdaIrreg.push_back(lambdaIrreg.back() + rng.UniformFloat());
    for (size_t i = 0; i < lambdaIrreg.size(); ++i)
        valIrreg.push_back(lambdaIrreg[i] * lambdaIrreg[i]);

    // Resample over a subset of the wavelengths.
    const int nOut = 10;
    Float newVal[nOut];
    const Float lambdaMin = -5, lambdaMax = 20;
    ResampleLinearSpectrum(lambdaIrreg.data(), valIrreg.data(), lambdaIrreg.size(),
                     lambdaMin, lambdaMax, nOut, newVal);

    // As with the IrregularResample test, we need a enough of an error
    // tolerance so that we account for the averaging over ranges of the
    // quadratic function.
    for (int i = 0; i < nOut; ++i) {
        Float t = Float(i) / Float(nOut - 1);
        Float lambda = Lerp(t, lambdaMin, lambdaMax);
        EXPECT_LT(std::abs(lambda * lambda - newVal[i]), .8);
    }
}
