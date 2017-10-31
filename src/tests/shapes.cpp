
#include "tests/gtest/gtest.h"
#include <cmath>
#include <functional>
#include "pbrt.h"
#include "rng.h"
#include "shape.h"
#include "lowdiscrepancy.h"
#include "sampling.h"
#include "shapes/cone.h"
#include "shapes/cylinder.h"
#include "shapes/disk.h"
#include "shapes/paraboloid.h"
#include "shapes/sphere.h"
#include "shapes/triangle.h"

using namespace pbrt;

static Float pExp(RNG &rng, Float exp = 8.) {
    Float logu = Lerp(rng.UniformFloat(), -exp, exp);
    return std::pow(10, logu);
}

static Float pUnif(RNG &rng, Float range = 10.) {
    return Lerp(rng.UniformFloat(), -range, range);
}

TEST(Triangle, Watertight) {
    RNG rng(12111);
    int nTheta = 16, nPhi = 16;
    ASSERT_GE(nTheta, 3);
    ASSERT_GE(nPhi, 4);

    // Make a triangle mesh representing a triangulated sphere (with
    // vertices randomly offset along their normal), centered at the
    // origin.
    int nVertices = nTheta * nPhi;
    std::vector<Point3f> vertices;
    for (int t = 0; t < nTheta; ++t) {
        Float theta = Pi * (Float)t / (Float)(nTheta - 1);
        Float cosTheta = std::cos(theta);
        Float sinTheta = std::sin(theta);
        for (int p = 0; p < nPhi; ++p) {
            Float phi = 2 * Pi * (Float)p / (Float)(nPhi - 1);
            Float radius = 1;
            // Make sure all of the top and bottom vertices are coincident.
            if (t == 0)
                vertices.push_back(Point3f(0, 0, radius));
            else if (t == nTheta - 1)
                vertices.push_back(Point3f(0, 0, -radius));
            else if (p == nPhi - 1)
                // Close it up exactly at the end
                vertices.push_back(vertices[vertices.size() - (nPhi - 1)]);
            else {
                radius += 5 * rng.UniformFloat();
                vertices.push_back(
                    Point3f(0, 0, 0) +
                    radius * SphericalDirection(sinTheta, cosTheta, phi));
            }
        }
    }
    EXPECT_EQ(nVertices, vertices.size());

    std::vector<int> indices;
    // fan at top
    auto offset = [nPhi](int t, int p) { return t * nPhi + p; };
    for (int p = 0; p < nPhi - 1; ++p) {
        indices.push_back(offset(0, 0));
        indices.push_back(offset(1, p));
        indices.push_back(offset(1, p + 1));
    }

    // quads in the middle rows
    for (int t = 1; t < nTheta - 2; ++t) {
        for (int p = 0; p < nPhi - 1; ++p) {
            indices.push_back(offset(t, p));
            indices.push_back(offset(t + 1, p));
            indices.push_back(offset(t + 1, p + 1));

            indices.push_back(offset(t, p));
            indices.push_back(offset(t + 1, p + 1));
            indices.push_back(offset(t, p + 1));
        }
    }

    // fan at bottom
    for (int p = 0; p < nPhi - 1; ++p) {
        indices.push_back(offset(nTheta - 1, 0));
        indices.push_back(offset(nTheta - 2, p));
        indices.push_back(offset(nTheta - 2, p + 1));
    }

    Transform identity;
    std::vector<std::shared_ptr<Shape>> tris = CreateTriangleMesh(
        &identity, &identity, false, indices.size() / 3, &indices[0], nVertices,
        &vertices[0], nullptr, nullptr, nullptr, nullptr, nullptr);

    for (int i = 0; i < 100000; ++i) {
        RNG rng(i);
        // Choose a random point in sphere of radius 0.5 around the origin.
        Point2f u;
        u[0] = rng.UniformFloat();
        u[1] = rng.UniformFloat();
        Point3f p = Point3f(0, 0, 0) + Float(0.5) * UniformSampleSphere(u);

        // Choose a random direction.
        u[0] = rng.UniformFloat();
        u[1] = rng.UniformFloat();
        Ray r(p, UniformSampleSphere(u));
        int nHits = 0;
        for (const auto &tri : tris) {
            Float tHit;
            SurfaceInteraction isect;
            if (tri->Intersect(r, &tHit, &isect, false)) ++nHits;
        }
        EXPECT_GE(nHits, 1);

        // Now tougher: shoot directly at a vertex.
        Point3f pVertex = vertices[rng.UniformUInt32(vertices.size())];
        r.d = pVertex - r.o;
        nHits = 0;
        for (const auto &tri : tris) {
            Float tHit;
            SurfaceInteraction isect;
            if (tri->Intersect(r, &tHit, &isect, false)) ++nHits;
        }
        EXPECT_GE(nHits, 1) << pVertex;
    }
}

std::shared_ptr<Triangle> GetRandomTriangle(std::function<Float()> value) {
    // Triangle vertices
    Point3f v[3];
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) v[j][k] = value();

    if ((Cross(v[1] - v[0], v[2] - v[0]).LengthSquared()) < 1e-20)
        // Don't into trouble with ~degenerate triangles.
        return nullptr;

    // Create the corresponding Triangle.
    static Transform identity;
    int indices[3] = {0, 1, 2};
    std::vector<std::shared_ptr<Shape>> triVec =
        CreateTriangleMesh(&identity, &identity, false, 1, indices, 3, v,
                           nullptr, nullptr, nullptr, nullptr, nullptr);
    EXPECT_EQ(1, triVec.size());
    std::shared_ptr<Triangle> tri =
        std::dynamic_pointer_cast<Triangle>(triVec[0]);
    EXPECT_NE(tri.get(), nullptr);
    return tri;
}

TEST(Triangle, Reintersect) {
    for (int i = 0; i < 1000; ++i) {
        RNG rng(i);
        std::shared_ptr<Triangle> tri =
            GetRandomTriangle([&]() { return pExp(rng); });
        if (!tri) continue;

        // Sample a point on the triangle surface to shoot the ray toward.
        Point2f u;
        u[0] = rng.UniformFloat();
        u[1] = rng.UniformFloat();
        Float pdf;
        Interaction pTri = tri->Sample(u, &pdf);

        // Choose a ray origin.
        Point3f o;
        for (int j = 0; j < 3; ++j) o[j] = pExp(rng);

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
            for (int k = 0; k < 3; ++k) p2[k] = pExp(rng);
            rOut = isect.SpawnRayTo(p2);

            EXPECT_FALSE(tri->IntersectP(rOut));
            EXPECT_FALSE(tri->Intersect(rOut, &tHit, &isect, false));
        }
    }
}

// Computes the projected solid angle subtended by a series of random
// triangles both using uniform spherical sampling as well as
// Triangle::Sample(), in order to verify Triangle::Sample().
TEST(Triangle, Sampling) {
    for (int i = 0; i < 30; ++i) {
        const Float range = 10;
        RNG rng(i);
        std::shared_ptr<Triangle> tri =
            GetRandomTriangle([&]() { return pUnif(rng, range); });
        if (!tri) continue;

        // Ensure that the reference point isn't too close to the
        // triangle's surface (which makes the Monte Carlo stuff have more
        // variance, thus requiring more samples).
        Point3f pc{pUnif(rng, range), pUnif(rng, range), pUnif(rng, range)};
        pc[rng.UniformUInt32() % 3] =
            rng.UniformFloat() > .5 ? (-range - 3) : (range + 3);

        // Compute reference value using Monte Carlo with uniform spherical
        // sampling.
        const int count = 512 * 1024;
        int hits = 0;
        for (int j = 0; j < count; ++j) {
            Point2f u{RadicalInverse(0, j), RadicalInverse(1, j)};
            Vector3f w = UniformSampleSphere(u);
            if (tri->IntersectP(Ray(pc, w))) ++hits;
        }
        double unifEstimate = hits / double(count * UniformSpherePdf());

        // Now use Triangle::Sample()...
        Interaction ref(pc, Normal3f(), Vector3f(), Vector3f(0, 0, 1), 0,
                        MediumInterface{});
        double triSampleEstimate = 0;
        for (int j = 0; j < count; ++j) {
            Point2f u{RadicalInverse(0, j), RadicalInverse(1, j)};
            Float pdf;
            Interaction pTri = tri->Sample(ref, u, &pdf);
            EXPECT_GT(pdf, 0);
            triSampleEstimate += 1. / (count * pdf);
        }

        // Now make sure that the two computed solid angle values are
        // fairly close.
        // Absolute error for small solid angles, relative for large.
        auto error = [](Float a, Float b) {
            if (std::abs(a) < 1e-4 || std::abs(b) < 1e-4)
                return std::abs(a - b);
            return std::abs((a - b) / b);
        };

        // Don't compare really small triangles, since uniform sampling
        // doesn't get a good estimate for them.
        if (triSampleEstimate > 1e-3)
            // The error tolerance is fairly large so that we can use a
            // reasonable number of samples.  It has been verified that for
            // larger numbers of Monte Carlo samples, the error continues to
            // tighten.
            EXPECT_LT(error(triSampleEstimate, unifEstimate), .1)
                << "Unif sampling: " << unifEstimate
                << ", triangle sampling: " << triSampleEstimate
                << ", tri index " << i;
    }
}

// Checks the closed-form solid angle computation for triangles against a
// Monte Carlo estimate of it.
TEST(Triangle, SolidAngle) {
    for (int i = 0; i < 50; ++i) {
        const Float range = 10;
        RNG rng(100 +
                i);  // Use different triangles than the Triangle/Sample test.
        std::shared_ptr<Triangle> tri =
            GetRandomTriangle([&]() { return pUnif(rng, range); });
        if (!tri) continue;

        // Ensure that the reference point isn't too close to the
        // triangle's surface (which makes the Monte Carlo stuff have more
        // variance, thus requiring more samples).
        Point3f pc{pUnif(rng, range), pUnif(rng, range), pUnif(rng, range)};
        pc[rng.UniformUInt32() % 3] =
            rng.UniformFloat() > .5 ? (-range - 3) : (range + 3);

        // Compute a reference value using Triangle::Sample()
        const int count = 64 * 1024;
        Interaction ref(pc, Normal3f(), Vector3f(), Vector3f(0, 0, 1), 0,
                        MediumInterface{});
        double triSampleEstimate = 0;
        for (int j = 0; j < count; ++j) {
            Point2f u{RadicalInverse(0, j), RadicalInverse(1, j)};
            Float pdf;
            (void)tri->Sample(ref, u, &pdf);
            EXPECT_GT(pdf, 0);
            triSampleEstimate += 1. / (count * pdf);
        }

        auto error = [](Float a, Float b) {
            if (std::abs(a) < 1e-4 || std::abs(b) < 1e-4)
                return std::abs(a - b);
            return std::abs((a - b) / b);
        };

        // Now compute the subtended solid angle of the triangle in closed
        // form.
        Float sphericalArea = tri->SolidAngle(pc);

        EXPECT_LT(error(sphericalArea, triSampleEstimate), .015)
            << "spherical area: " << sphericalArea
            << ", tri sampling: " << triSampleEstimate << ", pc = " << pc
            << ", tri index " << i;
    }
}

// Use Quasi Monte Carlo with uniform sphere sampling to esimate the solid
// angle subtended by the given shape from the given point.
static Float mcSolidAngle(const Point3f &p, const Shape &shape, int nSamples) {
    int nHits = 0;
    for (int i = 0; i < nSamples; ++i) {
        Point2f u{RadicalInverse(0, i), RadicalInverse(1, i)};
        Vector3f w = UniformSampleSphere(u);
        if (shape.IntersectP(Ray(p, w), false)) ++nHits;
    }
    return nHits / (UniformSpherePdf() * nSamples);
}

TEST(Sphere, SolidAngle) {
    Transform tr = Translate(Vector3f(1, .5, -.8)) * RotateX(30);
    Transform trInv = Inverse(tr);
    Sphere sphere(&tr, &trInv, false, 1, -1, 1, 360);

    // Make sure we get a subtended solid angle of 4pi for a point
    // inside the sphere.
    Point3f pInside(1, .9, -.8);
    const int nSamples = 128 * 1024;
    EXPECT_LT(std::abs(mcSolidAngle(pInside, sphere, nSamples) - 4 * Pi), .01);
    EXPECT_LT(std::abs(sphere.SolidAngle(pInside, nSamples) - 4 * Pi), .01);

    // Now try a point outside the sphere
    Point3f p(-.25, -1, .8);
    Float mcSA = mcSolidAngle(p, sphere, nSamples);
    Float sphereSA = sphere.SolidAngle(p, nSamples);
    EXPECT_LT(std::abs(mcSA - sphereSA), .001);
}

TEST(Cylinder, SolidAngle) {
    Transform tr = Translate(Vector3f(1, .5, -.8)) * RotateX(30);
    Transform trInv = Inverse(tr);
    Cylinder cyl(&tr, &trInv, false, .25, -1, 1, 360.);

    Point3f p(.5, .25, .5);
    const int nSamples = 128 * 1024;
    Float solidAngle = mcSolidAngle(p, cyl, nSamples);
    EXPECT_LT(std::abs(solidAngle - cyl.SolidAngle(p, nSamples)), .001);
}

TEST(Disk, SolidAngle) {
    Transform tr = Translate(Vector3f(1, .5, -.8)) * RotateX(30);
    Transform trInv = Inverse(tr);
    Disk disk(&tr, &trInv, false, 0, 1.25, 0, 360);

    Point3f p(.5, -.8, .5);
    const int nSamples = 128 * 1024;
    Float solidAngle = mcSolidAngle(p, disk, nSamples);
    EXPECT_LT(std::abs(solidAngle - disk.SolidAngle(p, nSamples)), .001);
}

// Check for incorrect self-intersection: assumes that the shape is convex,
// such that if the dot product of an outgoing ray and the surface normal
// at a point is positive, then a ray leaving that point in that direction
// should never intersect the shape.
static void TestReintersectConvex(Shape &shape, RNG &rng) {
    // Ray origin
    Point3f o;
    for (int c = 0; c < 3; ++c) o[c] = pExp(rng);

    // Destination: a random point in the shape's bounding box.
    Bounds3f bbox = shape.WorldBound();
    Point3f t;
    for (int c = 0; c < 3; ++c) t[c] = rng.UniformFloat();
    Point3f p2 = bbox.Lerp(t);

    // Ray to intersect with the shape.
    Ray r(o, p2 - o);
    if (rng.UniformFloat() < .5) r.d = Normalize(r.d);

    // We should usually (but not always) find an intersection.
    SurfaceInteraction isect;
    Float tHit;
    if (!shape.Intersect(r, &tHit, &isect, false)) return;

    // Now trace a bunch of rays leaving the intersection point.
    for (int j = 0; j < 10000; ++j) {
        // Random direction leaving the intersection point.
        Point2f u;
        u[0] = rng.UniformFloat();
        u[1] = rng.UniformFloat();
        Vector3f w = UniformSampleSphere(u);
        // Make sure it's in the same hemisphere as the surface normal.
        w = Faceforward(w, isect.n);
        Ray rOut = isect.SpawnRay(w);
        EXPECT_FALSE(shape.IntersectP(rOut, false));

        SurfaceInteraction spawnIsect;
        Float spawnTHit;
        EXPECT_FALSE(shape.Intersect(rOut, &spawnTHit, &spawnIsect, false));

        // Choose a random point to trace rays to.
        Point3f p2;
        for (int c = 0; c < 3; ++c) p2[c] = pExp(rng);
        // Make sure that the point we're tracing rays toward is in the
        // hemisphere about the intersection point's surface normal.
        w = p2 - isect.p;
        w = Faceforward(w, isect.n);
        p2 = isect.p + w;
        rOut = isect.SpawnRayTo(p2);

        EXPECT_FALSE(shape.IntersectP(rOut, false));
        EXPECT_FALSE(shape.Intersect(rOut, &tHit, &isect, false));
    }
}

TEST(FullSphere, Reintersect) {
    for (int i = 0; i < 100; ++i) {
        RNG rng(i);
        Transform identity;
        Float radius = pExp(rng, 4);
        Float zMin = -radius;
        Float zMax = radius;
        Float phiMax = 360;
        Sphere sphere(&identity, &identity, false, radius, zMin, zMax, phiMax);

        TestReintersectConvex(sphere, rng);
    }
}

TEST(ParialSphere, Normal) {
    for (int i = 0; i < 100; ++i) {
        RNG rng(i);
        Transform identity;
        Float radius = pExp(rng, 4);
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
        for (int c = 0; c < 3; ++c) o[c] = pExp(rng);

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

TEST(PartialSphere, Reintersect) {
    for (int i = 0; i < 100; ++i) {
        RNG rng(i);
        Transform identity;
        Float radius = pExp(rng, 4);
        Float zMin = rng.UniformFloat() < 0.5
                         ? -radius
                         : Lerp(rng.UniformFloat(), -radius, radius);
        Float zMax = rng.UniformFloat() < 0.5
                         ? radius
                         : Lerp(rng.UniformFloat(), -radius, radius);
        Float phiMax =
            rng.UniformFloat() < 0.5 ? 360. : rng.UniformFloat() * 360.;
        Sphere sphere(&identity, &identity, false, radius, zMin, zMax, phiMax);

        TestReintersectConvex(sphere, rng);
    }
}

TEST(Cylinder, Reintersect) {
    for (int i = 0; i < 100; ++i) {
        RNG rng(i);
        Transform identity;
        Float radius = pExp(rng, 4);
        Float zMin = pExp(rng, 4) * (rng.UniformFloat() < 0.5 ? -1 : 1);
        Float zMax = pExp(rng, 4) * (rng.UniformFloat() < 0.5 ? -1 : 1);
        Float phiMax =
            rng.UniformFloat() < 0.5 ? 360. : rng.UniformFloat() * 360.;
        Cylinder cyl(&identity, &identity, false, radius, zMin, zMax, phiMax);

        TestReintersectConvex(cyl, rng);
    }
}

#if 0
TEST(Cone, Reintersect) {
    for (int i = 0; i < 1000; ++i) {
        RNG rng(i);
        Transform identity;
        Float height = p(rng, 4);
        Float radius = p(rng, 4);
        Float phiMax = 360;
        Cone cone(&identity, &identity, false, height, radius, phiMax);

        TestReintersectConvex(cone, rng);
    }
}

TEST(Paraboloid, Reintersect) {
    for (int i = 0; i < 1000; ++i) {
        RNG rng(i);
        Transform identity;
        Float radius = p(rng, 4);
        Float z0 = p(rng, 4);
        Float z1 = p(rng, 4);
        Float phiMax = 360;
        Paraboloid paraboloid(&identity, &identity, false, radius,
                              z0, z1, phiMax);

        TestReintersectConvex(paraboloid, rng);
    }
}
#endif
