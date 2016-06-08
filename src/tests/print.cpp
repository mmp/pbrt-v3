
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "geometry.h"
#include "quaternion.h"
#include "transform.h"
#include <sstream>

TEST(StringPrintf, Basics) {
    EXPECT_EQ(StringPrintf("Hello, world"), "Hello, world");
    EXPECT_EQ(StringPrintf("x = %d", 5), "x = 5");
    EXPECT_EQ(StringPrintf("%f, %f, %f", 1., 1.5, -8.125),
              "1.000000, 1.500000, -8.125000");
}

TEST(OperatorLeftShiftPrint, Basics) {
    {
        std::ostringstream os;
        os << Point2f(105.5, -12.8);
        EXPECT_EQ(os.str(), "[ 105.5000000000, -12.8000001907 ]");
    }
    {
        std::ostringstream os;
        os << Point2i(-9, 5);
        EXPECT_EQ(os.str(), "[ -9, 5 ]");
    }
    {
        std::ostringstream os;
        os << Point3f(0., 1.25, -9.25);
        EXPECT_EQ(os.str(), "[ 0.0000000000, 1.2500000000, -9.2500000000 ]");
    }
    {
        std::ostringstream os;
        os << Point3i(7, -10, 4);
        EXPECT_EQ(os.str(), "[ 7, -10, 4 ]");
    }
    {
        std::ostringstream os;
        os << Vector2f(105.5, -12.8);
        EXPECT_EQ(os.str(), "[ 105.5000000000, -12.8000001907 ]");
    }
    {
        std::ostringstream os;
        os << Vector2i(-9, 5);
        EXPECT_EQ(os.str(), "[ -9, 5 ]");
    }
    {
        std::ostringstream os;
        os << Vector3f(0., 1.25, -9.25);
        EXPECT_EQ(os.str(), "[ 0.0000000000, 1.2500000000, -9.2500000000 ]");
    }
    {
        std::ostringstream os;
        os << Vector3i(7, -10, 4);
        EXPECT_EQ(os.str(), "[ 7, -10, 4 ]");
    }
    {
        std::ostringstream os;
        os << Normal3f(0., 1.25, -9.25);
        EXPECT_EQ(os.str(), "[ 0.0000000000, 1.2500000000, -9.2500000000 ]");
    }
    {
        std::ostringstream os;
        Quaternion q;
        q.v = {1.25, -8.3, 14.75};
        q.w = -0.5;
        os << q;
        EXPECT_EQ(
            os.str(),
            "[1.2500000000, -8.3000001907, 14.7500000000, -0.5000000000]");
    }
    {
        std::ostringstream os;
        Ray r(Point3f(-5.5, 2.75, 0.), Vector3f(1.0, -8.75, 2.25), 10000.f,
              0.25);
        os << r;
        EXPECT_EQ(os.str(),
                  "[o=[ -5.5000000000, 2.7500000000, 0.0000000000 ], "
                  "d=[ 1.0000000000, -8.7500000000, 2.2500000000 ], "
                  "tMax=10000, time=0.25]");
    }
    {
        std::ostringstream os;
        Bounds2f b(Point2f(2, -5), Point2f(-8, 3));
        os << b;
        EXPECT_EQ(os.str(),
                  "[ [ -8.0000000000, -5.0000000000 ] - [ 2.0000000000, "
                  "3.0000000000 ] ]");
    }
    {
        std::ostringstream os;
        Bounds3f b(Point3f(2, -5, .125), Point3f(-8, 3, -128.5));
        os << b;
        EXPECT_EQ(os.str(),
                  "[ [ -8.0000000000, -5.0000000000, -128.5000000000 ] - "
                  "[ 2.0000000000, 3.0000000000, 0.1250000000 ] ]");
    }
    {
        std::ostringstream os;
        Matrix4x4 m(0., -1., 2., -3.5, 4.5, 5.5, -6.5, -7.5, 8., 9.25, 10.75,
                    -11, 12, 13.25, 14.5, -15.875);
        os << m;
        EXPECT_EQ(
            os.str(),
            "[ [ 0.0000000000, -1.0000000000, 2.0000000000, -3.5000000000 ] "
            "[ 4.5000000000, 5.5000000000, -6.5000000000, -7.5000000000 ] "
            "[ 8.0000000000, 9.2500000000, 10.7500000000, -11.0000000000 ] "
            "[ 12.0000000000, 13.2500000000, 14.5000000000, -15.8750000000 ] "
            "]");
    }
    {
        std::ostringstream os;
        Transform t =
            Translate(Vector3f(-1.25, 3.5, 7.875)) * Scale(2., -3., -4.75);
        os << t;
        EXPECT_EQ(
            os.str(),
            "t=[ [ 2.0000000000, 0.0000000000, 0.0000000000, -1.2500000000 ] "
            "[ 0.0000000000, -3.0000000000, 0.0000000000, 3.5000000000 ] "
            "[ 0.0000000000, 0.0000000000, -4.7500000000, 7.8750000000 ] "
            "[ 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000 ] ], "
            "inv=[ [ 0.5000000000, 0.0000000000, 0.0000000000, 0.6250000000 ] "
            "[ 0.0000000000, -0.3333333433, 0.0000000000, 1.1666667461 ] "
            "[ 0.0000000000, 0.0000000000, -0.2105263174, 1.6578947306 ] "
            "[ 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000 ] ]");
    }
    // then clang-format...
}
