
#include "tests/gtest/gtest.h"
#include <stdint.h>
#include "pbrt.h"

TEST(Log2, Basics) {
    for (int i = 0; i < 32; ++i) {
        uint32_t ui = 1u << i;
        EXPECT_EQ(i, Log2Int(ui));
    }

    for (int i = 1; i < 31; ++i) {
        uint32_t ui = 1u << i;
        EXPECT_EQ(i, Log2Int(ui + 1));
    }
}

TEST(Pow2, Basics) {
    for (int i = 0; i < 32; ++i) {
        uint32_t ui = 1u << i;
        EXPECT_EQ(true, IsPowerOf2(ui));
        if (ui > 1) {
            EXPECT_EQ(false, IsPowerOf2(ui + 1));
        }
        if (ui > 2) {
            EXPECT_EQ(false, IsPowerOf2(ui - 1));
        }
    }
}

TEST(CountTrailing, Basics) {
    for (int i = 0; i < 32; ++i) {
        uint32_t ui = 1u << i;
        EXPECT_EQ(i, CountTrailingZeros(ui));
    }
}

TEST(RoundUpPow2, Basics) {
    EXPECT_EQ(RoundUpPow2(7), 8);
    for (int i = 1; i < (1 << 24); ++i)
        if (IsPowerOf2(i))
            EXPECT_EQ(RoundUpPow2(i), i);
        else
            EXPECT_EQ(RoundUpPow2(i), 1 << (Log2Int(i) + 1));

    for (int64_t i = 1; i < (1 << 24); ++i)
        if (IsPowerOf2(i))
            EXPECT_EQ(RoundUpPow2(i), i);
        else
            EXPECT_EQ(RoundUpPow2(i), 1 << (Log2Int(i) + 1));

    for (int i = 0; i < 30; ++i) {
        int v = 1 << i;
        EXPECT_EQ(RoundUpPow2(v), v);
        if (v > 2) EXPECT_EQ(RoundUpPow2(v - 1), v);
        EXPECT_EQ(RoundUpPow2(v + 1), 2 * v);
    }

    for (int i = 0; i < 62; ++i) {
        int64_t v = 1ll << i;
        EXPECT_EQ(RoundUpPow2(v), v);
        if (v > 2) EXPECT_EQ(RoundUpPow2(v - 1), v);
        EXPECT_EQ(RoundUpPow2(v + 1), 2 * v);
    }
}
