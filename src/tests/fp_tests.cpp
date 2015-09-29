
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

TEST(FloatingPoint, EFloat) {
    for (int trial = 0; trial < 1000; ++trial) {
        RNG rng(trial);
        EFloat ef = -10000 + 20000 * rng.UniformFloat();
        long double ld = (float)ef;

        for (int iter = 0; iter < 100; ++iter) {
            EXPECT_GE(ld, ef.LowerBound());
            EXPECT_LE(ld, ef.UpperBound());

            EFloat ef2 = 0;
            float f = -10000 + 20000 * rng.UniformFloat();
            switch (rng.UniformUInt32() % 3) {
            case 0:
                ef2 = f;
                break;
            case 1:
                ef2 = ef;
                break;
            case 3:
                ef2 = EFloat(f, std::abs(rng.UniformFloat() * .001 * f));
                break;
            }
            long double ld2 = (float)ef2;

            int op = rng.UniformUInt32() % 7;
            switch (op) {
            case 0:
                ef = -ef;
                ld = -ld;
                break;
            case 1:
                ef = ef + ef2;
                ld = ld + ld2;
                break;
            case 2:
                ef = ef - ef2;
                ld = ld - ld2;
                break;
            case 3:
                ef = ef * ef2;
                ld = ld * ld2;
                break;
            case 4:
                if (ef.LowerBound() * ef.UpperBound() > 0 &&
                    ef2.LowerBound() * ef2.UpperBound() > 0) {
                    ef = ef / ef2;
                    ld = ld / ld2;
                }
                break;
            case 5:
                if (ld >= 0 && ef.LowerBound() > 0) {
                    ef = sqrt(ef);
                    ld = std::sqrt(ld);
                }
                break;
            case 6:
                ef = abs(ef);
                ld = (ld < 0) ? -ld : ld;
                break;
            }

            if (std::isnan((float)ef) || std::isnan(ld) ||
                std::isinf((float)ef) || std::isinf(ld) ||
                std::isinf(ef.GetAbsoluteError()))
                break;
        }
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
