#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_SPHERE_DE
#define PBRT_SHAPES_SPHERE_DE

#include "distanceestimator.h"

namespace pbrt {

class SphereDE : public DistanceEstimator {
    public:
        // SphereDE Public Methods.
        SphereDE(
            const Transform *o2w,
            const Transform *w2o,
            bool reverseOrientation,
            const DistanceEstimatorParams &params,
            Float radius)
            : DistanceEstimator(o2w, w2o, reverseOrientation, params),
            radius(radius) {}
        Bounds3f ObjectBound() const;
        Float Area() const;
        Float Evaluate(const Point3f &p) const;
    private:
        // SphereDE Private Data.
        const Float radius;
};

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w,
                                           const Transform *w2o,
                                           bool reverseOrientation,
                                           const ParamSet &params);

} // namespace pbrt

#endif // PBRT_SHAPES_SPHERE_DE
