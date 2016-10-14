
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "geometry.h"

TEST(Bounds2, IteratorBasic) {
    Bounds2i b{{0, 1}, {2, 3}};
    Point2i e[] = { { 0, 1 }, { 1, 1 }, { 0, 2 }, { 1, 2 } };
    int offset = 0;
    for (auto p : b) {
        EXPECT_LT(offset, sizeof(e) / sizeof(e[0]));
        EXPECT_EQ(e[offset], p) << "offset = " << offset;
        ++offset;
    }
}

TEST(Bounds2, IteratorDegenerate) {
    Bounds2i b{{0, 0}, {0, 10}};
    for (auto p : b) {
        // This loop should never run.
        bool reached = true;
        EXPECT_FALSE(reached) << "p = " << p;
        break;
    }

    Bounds2i b2{{0, 0}, {4, 0}};
    for (auto p : b2) {
        // This loop should never run.
        bool reached = true;
        EXPECT_FALSE(reached) << "p = " << p;
        break;
    }

    Bounds2i b3;
    for (auto p : b3) {
        // This loop should never run.
        bool reached = true;
        EXPECT_FALSE(reached) << "p = " << p;
        break;
    }
}

