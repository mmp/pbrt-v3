
#include "tests/gtest/gtest.h"
#include <cmath>
#include "pbrt.h"
#include "rng.h"
#include "shape.h"
#include "sampling.h"
#include "shapes/sphere.h"
#include "shapes/triangle.h"

static Float p(RNG &rng, Float exp = 8.) {
    Float logu = Lerp(rng.UniformFloat(), -exp, exp);
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
TEST(ParialSphere, Normal) {
    for (int i = 0; i < 1000; ++i) {
        RNG rng(i);
        Transform identity;
        Float radius = p(rng, 4);
        Float zMin = rng.UniformFloat() < 0.5
                         ? -radius
                         : Lerp(rng.UniformFloat(), -radius, radius);
        Float zMax = rng.UniformFloat() < 0.5
                         ? radius
                         : Lerp(rng.UniformFloat(), -radius, radius);
        Float phiMax =
            rng.UniformFloat() < 0.5 ? 360. : rng.UniformFloat() * 360.;
        Sphere sphere(&identity, &identity, false, radius, zMin, zMax, phiMax);

        // Ray origin
        Point3f o;
        for (int c = 0; c < 3; ++c) o[c] = p(rng);

        // Destination: a random point in the shape's bounding box.
        Bounds3f bbox = sphere.WorldBound();
        Point3f t;
        for (int c = 0; c < 3; ++c) t[c] = rng.UniformFloat();
        Point3f p2 = bbox.Lerp(t);

        // Ray to intersect with the shape.
        Ray r(o, p2 - o);
        if (rng.UniformFloat() < .5) r.d = Normalize(r.d);

        // We should usually (but not always) find an intersection.
        SurfaceInteraction isect;
        Float tHit;
        if (!sphere.Intersect(r, &tHit, &isect, false)) continue;

        Float dot = Dot(Normalize(isect.n), Normalize(Vector3f(isect.p)));
        EXPECT_FLOAT_EQ(1., dot);
    }
}

