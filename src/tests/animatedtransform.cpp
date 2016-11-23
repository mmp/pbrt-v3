
#include "tests/gtest/gtest.h"
#include "pbrt.h"
#include "rng.h"
#include "sampling.h"
#include "transform.h"

using namespace pbrt;

static Transform RandomTransform(RNG &rng) {
    Transform t;
    auto r = [&rng]() { return -10. + 20. * rng.UniformFloat(); };
    for (int i = 0; i < 10; ++i) {
        switch (rng.UniformUInt32(3)) {
        case 0:
            t = t * Scale(std::abs(r()), std::abs(r()), std::abs(r()));
            break;
        case 1:
            t = t * Translate(Vector3f(r(), r(), r()));
            break;
        case 2:
            t = t *
                Rotate(r() * 20., UniformSampleSphere(Point2f(
                                      rng.UniformFloat(), rng.UniformFloat())));
            break;
        }
    }
    return t;
}

TEST(AnimatedTransform, Randoms) {
    RNG rng;
    auto r = [&rng]() { return -10. + 20. * rng.UniformFloat(); };

    for (int i = 0; i < 200; ++i) {
        // Generate a pair of random transformation matrices.
        Transform t0 = RandomTransform(rng);
        Transform t1 = RandomTransform(rng);
        AnimatedTransform at(&t0, 0., &t1, 1.);

        for (int j = 0; j < 5; ++j) {
            // Generate a random bounding box and find the bounds of its motion.
            Bounds3f bounds(Point3f(r(), r(), r()), Point3f(r(), r(), r()));
            Bounds3f motionBounds = at.MotionBounds(bounds);

            for (Float t = 0.; t <= 1.; t += 1e-3 * rng.UniformFloat()) {
                // Now, interpolate the transformations at a bunch of times
                // along the time range and then transform the bounding box
                // with the result.
                Transform tr;
                at.Interpolate(t, &tr);
                Bounds3f tb = tr(bounds);

                // Add a little slop to allow for floating-point round-off
                // error in computing the motion extrema times.
                tb.pMin += (Float)1e-4 * tb.Diagonal();
                tb.pMax -= (Float)1e-4 * tb.Diagonal();

                // Now, the transformed bounds should be inside the motion
                // bounds.
                EXPECT_GE(tb.pMin.x, motionBounds.pMin.x);
                EXPECT_LE(tb.pMax.x, motionBounds.pMax.x);
                EXPECT_GE(tb.pMin.y, motionBounds.pMin.y);
                EXPECT_LE(tb.pMax.y, motionBounds.pMax.y);
                EXPECT_GE(tb.pMin.z, motionBounds.pMin.z);
                EXPECT_LE(tb.pMax.z, motionBounds.pMax.z);
            }
        }
    }
}
