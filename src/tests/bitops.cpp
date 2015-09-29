
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include <stdint.h>

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
