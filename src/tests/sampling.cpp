
#include "tests/gtest/gtest.h"
#include <stdint.h>
#include <algorithm>
#include "pbrt.h"
#include "sampling.h"
#include "lowdiscrepancy.h"

TEST(LowDiscrepancy, RadicalInverse) {
    for (int a = 0; a < 1024; ++a) {
        EXPECT_EQ(ReverseBits32(a) * 2.3283064365386963e-10f,
                  RadicalInverse(0, a));
    }
}

TEST(LowDiscrepancy, GeneratorMatrix) {
    uint32_t C[32];
    // Identity matrix, column-wise
    for (int i = 0; i < 32; ++i) C[i] = 1 << i;

    for (int a = 0; a < 128; ++a) {
        // Make sure identity generator matrix matches van der Corput
        EXPECT_EQ(a, MultiplyGenerator(C, a));
        EXPECT_EQ(RadicalInverse(0, a), ReverseBits32(MultiplyGenerator(C, a)) *
                                            2.3283064365386963e-10f);
    }

    for (int a = 0; a < 16; ++a) {
        EXPECT_EQ(ReverseBits32(a), ReverseMultiplyGenerator(C, a));
        EXPECT_EQ(RadicalInverse(0, a),
                  ReverseMultiplyGenerator(C, a) * 2.3283064365386963e-10f);
        EXPECT_EQ(RadicalInverse(0, a), SampleGeneratorMatrix(C, a));
    }
}

TEST(LowDiscrepancy, GrayCodeSample) {
    uint32_t C[32];
    // Identity matrix, column-wise
    for (int i = 0; i < 32; ++i) C[i] = 1 << i;

    std::vector<Float> v(64, (Float)0);
    GrayCodeSample(C, v.size(), 0, &v[0]);

    for (int a = 0; a < (int)v.size(); ++a) {
        Float u = ReverseMultiplyGenerator(C, a) * 2.3283064365386963e-10f;
        EXPECT_NE(v.end(), std::find(v.begin(), v.end(), u));
    }
}
