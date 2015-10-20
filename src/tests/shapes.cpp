
#include "tests/gtest/gtest.h"
#include <cmath>
#include "pbrt.h"
#include "rng.h"
#include "shape.h"
#include "sampling.h"
#include "shapes/triangle.h"

static Float p(RNG &rng) {
    Float logu = Lerp(rng.UniformFloat(), -8., 8);
    return std::pow(10, logu);
}

TEST(Triangle, Reintersect) {
    for (int i = 0; i < 1000; ++i) {
        RNG rng(i);
        // Triangle vertices
        Point3f v[3];
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) v[j][k] = p(rng);

        // Create the corresponding Triangle.
        Transform identity;
        int indices[3] = {0, 1, 2};
        std::vector<std::shared_ptr<Shape>> triVec =
            CreateTriangleMesh(&identity, &identity, false, 1, indices, 3, v,
                               nullptr, nullptr, nullptr, nullptr, nullptr);
        EXPECT_EQ(1, triVec.size());
        std::shared_ptr<Shape> tri = triVec[0];

        // Sample a point on the triangle surface to shoot the ray toward.
        Point2f u;
        u[0] = rng.UniformFloat();
        u[1] = rng.UniformFloat();
        Interaction pTri = tri->Sample(u);

        // Choose a ray origin.
        Point3f o;
        for (int j = 0; j < 3; ++j) o[j] = p(rng);

        // Intersect the ray with the triangle.
        Ray r(o, pTri.p - o);
        Float tHit;
        SurfaceInteraction isect;
        if (!tri->Intersect(r, &tHit, &isect, false))
            // We should almost always find an intersection, but rarely
            // miss, due to round-off error. Just do another go-around in
            // this case.
            continue;

        // Now trace a bunch of rays leaving the intersection point.
        for (int j = 0; j < 10000; ++j) {
            // Random direction leaving the intersection point.
            Point2f u;
            u[0] = rng.UniformFloat();
            u[1] = rng.UniformFloat();
            Vector3f w = UniformSampleSphere(u);
            Ray rOut = isect.SpawnRay(w);
            EXPECT_FALSE(tri->IntersectP(rOut));

            SurfaceInteraction spawnIsect;
            Float tHit;
            EXPECT_FALSE(tri->Intersect(rOut, &tHit, &isect, false));

            // Choose a random point to trace rays to.
            Point3f p2;
            for (int k = 0; k < 3; ++k) p2[k] = p(rng);
            rOut = isect.SpawnRayTo(p2);

            EXPECT_FALSE(tri->IntersectP(rOut));
            EXPECT_FALSE(tri->Intersect(rOut, &tHit, &isect, false));
        }
    }
}
