
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "stringprint.h"
#include "geometry.h"
#include "quaternion.h"
#include "transform.h"
#include <sstream>

TEST(StringPrintf, Basics) {
    EXPECT_EQ(StringPrintf("Hello, world"), "Hello, world");
    EXPECT_EQ(StringPrintf("x = %d", 5), "x = 5");
    EXPECT_EQ(StringPrintf("%f, %f, %f", 1., 1.5, -8.125),
              "1, 1.5, -8.125");
#ifndef NDEBUG
    EXPECT_DEATH(StringPrintf("not enough %s"), "Assertion.*failed.*line");
#endif
}

TEST(OperatorLeftShiftPrint, Basics) {
    {
        std::ostringstream os;
        os << Point2f(105.5, -12.8);
        EXPECT_EQ(os.str(), "[ 105.5, -12.8000002 ]");
    }
    {
        std::ostringstream os;
        os << Point2i(-9, 5);
        EXPECT_EQ(os.str(), "[ -9, 5 ]");
    }
    {
        std::ostringstream os;
        os << Point3f(0., 1.25, -9.25);
        EXPECT_EQ(os.str(), "[ 0, 1.25, -9.25 ]");
    }
    {
        std::ostringstream os;
        os << Point3i(7, -10, 4);
        EXPECT_EQ(os.str(), "[ 7, -10, 4 ]");
    }
    {
        std::ostringstream os;
        os << Vector2f(105.5, -12.8);
        EXPECT_EQ(os.str(), "[ 105.5, -12.8000002 ]");
    }
    {
        std::ostringstream os;
        os << Vector2i(-9, 5);
        EXPECT_EQ(os.str(), "[ -9, 5 ]");
    }
    {
        std::ostringstream os;
        os << Vector3f(0., 1.25, -9.25);
        EXPECT_EQ(os.str(), "[ 0, 1.25, -9.25 ]");
    }
    {
        std::ostringstream os;
        os << Vector3i(7, -10, 4);
        EXPECT_EQ(os.str(), "[ 7, -10, 4 ]");
    }
    {
        std::ostringstream os;
        os << Normal3f(0., 1.25, -9.25);
        EXPECT_EQ(os.str(), "[ 0, 1.25, -9.25 ]");
    }
    {
        std::ostringstream os;
        Quaternion q;
        q.v = {1.25, -8.3, 14.75};
        q.w = -0.5;
        os << q;
        EXPECT_EQ(
            os.str(),
            "[ 1.25, -8.30000019, 14.75, -0.5 ]");
    }
    {
        std::ostringstream os;
        Ray r(Point3f(-5.5, 2.75, 0.), Vector3f(1.0, -8.75, 2.25), 10000.f,
              0.25);
        os << r;
        EXPECT_EQ(os.str(),
                  "[o=[ -5.5, 2.75, 0 ], d=[ 1, -8.75, 2.25 ], tMax=10000, time=0.25]");
    }
    {
        std::ostringstream os;
        Bounds2f b(Point2f(2, -5), Point2f(-8, 3));
        os << b;
        EXPECT_EQ(os.str(),
                  "[ [ -8, -5 ] - [ 2, 3 ] ]");
    }
    {
        std::ostringstream os;
        Bounds3f b(Point3f(2, -5, .125), Point3f(-8, 3, -128.5));
        os << b;
        EXPECT_EQ(os.str(),
                  "[ [ -8, -5, -128.5 ] - [ 2, 3, 0.125 ] ]");
    }
    {
        std::ostringstream os;
        Matrix4x4 m(0., -1., 2., -3.5, 4.5, 5.5, -6.5, -7.5, 8., 9.25, 10.75,
                    -11, 12, 13.25, 14.5, -15.875);
        os << m;
        EXPECT_EQ(
            os.str(),
            "[ [ 0, -1, 2, -3.5 ] "
            "[ 4.5, 5.5, -6.5, -7.5 ] "
            "[ 8, 9.25, 10.75, -11 ] "
            "[ 12, 13.25, 14.5, -15.875 ] "
            "]");
    }
    {
        std::ostringstream os;
        Transform t =
            Translate(Vector3f(-1.25, 3.5, 7.875)) * Scale(2., -3., -4.75);
        os << t;
        EXPECT_EQ(
            os.str(),
            "t=[ [ 2, 0, 0, -1.25 ] "
            "[ 0, -3, 0, 3.5 ] "
            "[ 0, 0, -4.75, 7.875 ] "
            "[ 0, 0, 0, 1 ] ], "
            "inv=[ [ 0.5, 0, 0, 0.625 ] "
            "[ 0, -0.333333343, 0, 1.16666675 ] "
            "[ 0, 0, -0.210526317, 1.65789473 ] "
            "[ 0, 0, 0, 1 ] ]");
    }
}
