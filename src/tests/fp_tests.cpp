
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
    for (int trial = 0; trial < 100000; ++trial) {
        RNG rng(trial);
        // Return an exponentially-distributed floating-point value.
        auto getFloat = [&rng](Float minExp = -4., Float maxExp = 4.) {
            Float logu = Lerp(rng.UniformFloat(), minExp, maxExp);
            Float sign = rng.UniformFloat() < .5 ? -1. : 1.;
            return sign * std::pow(10, logu);
        };

        // We'll track a value in an EFloat and a precise version of the
        // same value. As we perform random arithmetic operations to ef,
        // we'll perform the same operations on efPrecise and make sure
        // that efPrecise remains within the error bounds that ef has
        // computed..
        EFloat ef = getFloat();
        long double efPrecise = (float)ef;

        for (int iter = 0; iter < 100; ++iter) {
            // Exit out if we've gone off to a bad place.
            if (std::isnan((float)ef) || std::isnan(efPrecise) ||
                std::isinf((float)ef) || std::isinf(efPrecise) ||
                std::isinf(ef.GetAbsoluteError()))
                break;

            // Make sure that the precise value is inside the bounds of the
            // EFloat's interval.
            EXPECT_GE(efPrecise, ef.LowerBound()) << "trial " << trial
                                                  << ", iter " << iter;
            EXPECT_LE(efPrecise, ef.UpperBound()) << "trial " << trial
                                                  << ", iter " << iter;

            // Choose a second value to (maybe) use as an operand below.
            EFloat ef2 = 0;
            long double efPrecise2;
            switch (rng.UniformUInt32(3)) {
            case 0: {
                // Choose a new random floating-point value; assume it is
                // fully precise.
                Float f = getFloat();
                ef2 = f;
                efPrecise2 = f;
                break;
            }
            case 1:
                // Use the same value as ef.
                ef2 = ef;
                efPrecise2 = efPrecise;
                break;
            case 2: {
                // Choose a random float and make up a small interval around it.
                Float f = getFloat();
                Float err = std::abs(getFloat(-8., -2.) * f);
                ef2 = EFloat(f, err);
                // Now compute a 'precise' value that's not equal to f, but
                // is instead somewhere within the error bounds.
                Float offset = rng.UniformFloat();
                efPrecise2 = (1. - offset) * ef2.LowerBound() +
                             offset * ef2.UpperBound();
                // Sometimes the result isn't actually inside the bounds,
                // due to round-off error.
                if (efPrecise2 >= ef2.UpperBound() ||
                    efPrecise2 <= ef2.LowerBound())
                    efPrecise2 = f;
#ifndef NDEBUG
                ef2 = EFloat(f, efPrecise2, err);
#else
                ef2 = EFloat(f, err);
#endif
                break;
            }
            }

            // Now do a random operation, upading the separate precise
            // value in the same manner.
            switch (rng.UniformUInt32(7)) {
            case 0:
                // Negation.
                ef = -ef;
                efPrecise = -efPrecise;
                break;
            case 1:
                // Addition.
                ef = ef + ef2;
                efPrecise = efPrecise + efPrecise2;
                break;
            case 2:
                // Subtraction.
                ef = ef - ef2;
                efPrecise = efPrecise - efPrecise2;
                break;
            case 3:
                // Multiplication.
                ef = ef * ef2;
                efPrecise = efPrecise * efPrecise2;
                break;
            case 4:
                // Division.
                if (ef.LowerBound() * ef.UpperBound() > 0 &&
                    ef2.LowerBound() * ef2.UpperBound() > 0) {
                    ef = ef / ef2;
                    efPrecise = efPrecise / efPrecise2;
                }
                break;
            case 5:
                // Sqrt.
                if (efPrecise >= 0 && ef.LowerBound() > 0) {
                    ef = sqrt(ef);
                    efPrecise = std::sqrt(efPrecise);
                }
                break;
            case 6:
                // Abs.
                ef = abs(ef);
                efPrecise = (efPrecise < 0) ? -efPrecise : efPrecise;
                break;
            }
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
