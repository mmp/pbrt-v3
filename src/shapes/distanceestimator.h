#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCE_ESTIMATOR_H
#define PBRT_SHAPES_DISTANCE_ESTIMATOR_H

#include "shape.h"

namespace pbrt {

// Distance Estimator Parameters.
struct DistanceEstimatorParams {
    int maxIters = 1000;               // Number of steps along the ray until we give up (default 1000).
    Float hitEpsilon = 1e-5;           // How close to the surface we must be before we say we "hit" it.
    Float rayEpsilonMultiplier = 5;    // How much we multiply hitEpsilon by to get pError.
    Float normalEpsilon = 1e-5;        // The epsilon we send to CalculateNormal().
};

// Distance Estimator Declarations.
class DistanceEstimator : public Shape {
    public:
        // Distance Estimator Public Methods.
        DistanceEstimator(
            const Transform *o2w,
            const Transform *w2o,
            bool reverseOrientation,
            const DistanceEstimatorParams &params)
            : Shape(o2w, w2o, reverseOrientation),
            params(params) {}
        // Non Pure Virtual.
        virtual bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const;
        virtual Interaction Sample(const Point2f &u, Float *pdf) const;
        virtual Vector3f CalculateNormal(const Point3f &pos, float eps, const Vector3f &defaultNormal) const;

        // Pure Virtual.
        virtual Bounds3f ObjectBound() const = 0;
        virtual Float Area() const = 0;
        virtual Float Evaluate(const Point3f &p) const = 0;

    private:
        // Distance Estimator Private Data.
        const DistanceEstimatorParams params;
};

} // namespace pbrt

#endif // PBRT_SHAPES_DISTANCE_ESTIMATOR
