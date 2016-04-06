
#include "tests/gtest/gtest.h"
#include <cmath>

#include "pbrt.h"
#include "rng.h"
#include "efloat.h"
#include "parallel.h"

static float GetFloat(RNG &rng) {
    float f;
    do {
        f = BitsToFloat(rng.UniformUInt32());
    } while (std::isnan(f));
    return f;
}

static double GetDouble(RNG &rng) {
    double d;
    do {
        d = BitsToFloat(uint64_t(rng.UniformUInt32()) |
                        (uint64_t(rng.UniformUInt32()) << 32));
    } while (std::isnan(d));
    return d;
}

TEST(FloatingPoint, NextUpDownFloat) {
    EXPECT_GT(NextFloatUp(-0.f), 0.f);
    EXPECT_LT(NextFloatDown(0.f), 0.f);

    EXPECT_EQ(NextFloatUp((float)Infinity), (float)Infinity);
    EXPECT_LT(NextFloatDown((float)Infinity), (float)Infinity);

    EXPECT_EQ(NextFloatDown(-(float)Infinity), -(float)Infinity);
    EXPECT_GT(NextFloatUp(-(float)Infinity), -(float)Infinity);

    RNG rng;
    for (int i = 0; i < 100000; ++i) {
        float f = GetFloat(rng);
        if (std::isinf(f)) continue;

        EXPECT_EQ(std::nextafter(f, (float)Infinity), NextFloatUp(f));
        EXPECT_EQ(std::nextafter(f, -(float)Infinity), NextFloatDown(f));
    }
}

TEST(FloatingPoint, NextUpDownDouble) {
    EXPECT_GT(NextFloatUp(-0.), 0.);
    EXPECT_LT(NextFloatDown(0.), 0.);

    EXPECT_EQ(NextFloatUp((double)Infinity), (double)Infinity);
    EXPECT_LT(NextFloatDown((double)Infinity), (double)Infinity);

    EXPECT_EQ(NextFloatDown(-(double)Infinity), -(double)Infinity);
    EXPECT_GT(NextFloatUp(-(double)Infinity), -(double)Infinity);

    RNG rng(3);
    for (int i = 0; i < 100000; ++i) {
        double d = GetDouble(rng);
        if (std::isinf(d)) continue;

        EXPECT_EQ(std::nextafter(d, (double)Infinity), NextFloatUp(d));
        EXPECT_EQ(std::nextafter(d, -(double)Infinity), NextFloatDown(d));
    }
}

TEST(FloatingPoint, FloatBits) {
    RNG rng(1);
    for (int i = 0; i < 100000; ++i) {
        uint32_t ui = rng.UniformUInt32();
        float f = BitsToFloat(ui);
        if (std::isnan(f)) continue;

        EXPECT_EQ(ui, FloatToBits(f));
    }
}

TEST(FloatingPoint, DoubleBits) {
    RNG rng(2);
    for (int i = 0; i < 100000; ++i) {
        uint64_t ui = (uint64_t(rng.UniformUInt32()) |
                       (uint64_t(rng.UniformUInt32()) << 32));
        double f = BitsToFloat(ui);

        if (std::isnan(f)) continue;

        EXPECT_EQ(ui, FloatToBits(f));
    }
}

TEST(FloatingPoint, AtomicFloat) {
    AtomicFloat af(0);
    Float f = 0.;
    EXPECT_EQ(f, af);
    af.Add(1.0251);
    f += 1.0251;
    EXPECT_EQ(f, af);
    af.Add(2.);
    f += 2.;
    EXPECT_EQ(f, af);
}

///////////////////////////////////////////////////////////////////////////
// EFloat tests

// Return an exponentially-distributed floating-point value.
static EFloat getFloat(RNG &rng, Float minExp = -6., Float maxExp = 6.) {
    Float logu = Lerp(rng.UniformFloat(), minExp, maxExp);
    Float val = std::pow(10, logu);

    // Choose a random error bound.
    Float err = 0;
    switch (rng.UniformUInt32(4)) {
    case 0:
        // no error
        break;
    case 1: {
        // small typical/reasonable error
        uint32_t ulpError = rng.UniformUInt32(1024);
        Float offset = BitsToFloat(FloatToBits(val) + ulpError);
        err = std::abs(offset - val);
        break;
    }
    case 2: {
        // bigger ~reasonable error
        uint32_t ulpError = rng.UniformUInt32(1024 * 1024);
        Float offset = BitsToFloat(FloatToBits(val) + ulpError);
        err = std::abs(offset - val);
        break;
    }
    case 3: {
        err = (4 * rng.UniformFloat()) * std::abs(val);
    }
    }
    Float sign = rng.UniformFloat() < .5 ? -1. : 1.;
    return EFloat(sign * val, err);
}

// Given an EFloat covering some range, choose a double-precision "precise"
// value that is in the EFloat's range.
static double getPrecise(const EFloat &ef, RNG &rng) {
    switch (rng.UniformUInt32(3)) {
    // 2/3 of the time, pick a value that is right at the end of the range;
    // this is a maximally difficult / adversarial choice, so should help
    // ferret out any bugs.
    case 0:
        return ef.LowerBound();
    case 1:
        return ef.UpperBound();
    case 2: {
        // Otherwise choose a value uniformly inside the EFloat's range.
        Float t = rng.UniformFloat();
        double p = (1 - t) * ef.LowerBound() + t * ef.UpperBound();
        if (p > ef.UpperBound()) p = ef.UpperBound();
        if (p < ef.LowerBound()) p = ef.LowerBound();
        return p;
    }
    }
    return (Float)ef;  // NOTREACHED
}

static const int kEFloatIters = 1000000;

TEST(EFloat, Abs) {
    for (int trial = 0; trial < kEFloatIters; ++trial) {
        RNG rng(trial);

        EFloat ef = getFloat(rng);
        double precise = getPrecise(ef, rng);

        EFloat efResult = abs(ef);
        double preciseResult = std::abs(precise);

        EXPECT_GE(preciseResult, efResult.LowerBound());
        EXPECT_LE(preciseResult, efResult.UpperBound());
    }
}

TEST(EFloat, Sqrt) {
    for (int trial = 0; trial < kEFloatIters; ++trial) {
        RNG rng(trial);

        EFloat ef = getFloat(rng);
        double precise = getPrecise(ef, rng);

        // If the error starts to get too big such that the interval is
        // relatively close to zero w.r.t. the center value, we can't
        // compute error bounds for sqrt; skip these.
        if (ef.GetAbsoluteError() > .25 * std::abs(ef.LowerBound())) continue;

        EFloat efResult = sqrt(abs(ef));
        double preciseResult = std::sqrt(std::abs(precise));

        EXPECT_GE(preciseResult, efResult.LowerBound());
        EXPECT_LE(preciseResult, efResult.UpperBound());
    }
}

TEST(EFloat, Add) {
    for (int trial = 0; trial < kEFloatIters; ++trial) {
        RNG rng(trial);

        EFloat ef[2] = {getFloat(rng), getFloat(rng)};
        double precise[2] = {getPrecise(ef[0], rng), getPrecise(ef[1], rng)};

        EFloat efResult = ef[0] + ef[1];
        float preciseResult = precise[0] + precise[1];

        EXPECT_GE(preciseResult, efResult.LowerBound());
        EXPECT_LE(preciseResult, efResult.UpperBound());
    }
}

TEST(EFloat, Sub) {
    for (int trial = 0; trial < kEFloatIters; ++trial) {
        RNG rng(trial);

        EFloat ef[2] = {getFloat(rng), getFloat(rng)};
        double precise[2] = {getPrecise(ef[0], rng), getPrecise(ef[1], rng)};

        EFloat efResult = ef[0] - ef[1];
        float preciseResult = precise[0] - precise[1];

        EXPECT_GE(preciseResult, efResult.LowerBound());
        EXPECT_LE(preciseResult, efResult.UpperBound());
    }
}

TEST(EFloat, Mul) {
    for (int trial = 0; trial < kEFloatIters; ++trial) {
        RNG rng(trial);

        EFloat ef[2] = {getFloat(rng), getFloat(rng)};
        double precise[2] = {getPrecise(ef[0], rng), getPrecise(ef[1], rng)};

        EFloat efResult = ef[0] * ef[1];
        float preciseResult = precise[0] * precise[1];

        EXPECT_GE(preciseResult, efResult.LowerBound());
        EXPECT_LE(preciseResult, efResult.UpperBound());
    }
}

TEST(EFloat, Div) {
    for (int trial = 0; trial < kEFloatIters; ++trial) {
        RNG rng(trial);

        EFloat ef[2] = {getFloat(rng), getFloat(rng)};
        double precise[2] = {getPrecise(ef[0], rng), getPrecise(ef[1], rng)};

        // As with sqrt, things get messy if the denominator's interval
        // straddles zero or is too close to zero w.r.t. its center value.
        if (ef[1].LowerBound() * ef[1].UpperBound() < 0. ||
            ef[1].GetAbsoluteError() > .25 * std::abs(ef[1].LowerBound()))
            continue;

        EFloat efResult = ef[0] / ef[1];
        float preciseResult = precise[0] / precise[1];

        EXPECT_GE(preciseResult, efResult.LowerBound());
        EXPECT_LE(preciseResult, efResult.UpperBound());
    }
}
