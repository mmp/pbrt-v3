#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCE_ESTIMATOR_H
#define PBRT_SHAPES_DISTANCE_ESTIMATOR_H

#include "shape.h"

namespace pbrt {

// Distance Estimator Declarations.
class DistanceEstimator : public Shape {
    public:
        // Distance Estimator Public Methods.
        DistanceEstimator(const Transform *o2w, const Transform *w2o, bool reverseOrientation, Float radius, Float zMin, Float zMax, Float phiMax)
            : Shape(o2w, w2o, reverseOrientation),
            radius(radius),
            zMin(Clamp(std::min(zMin, zMax), -radius, radius)),
            zMax(Clamp(std::max(zMin, zMax), -radius, radius)),
            thetaMin(std::acos(Clamp(std::min(zMin, zMax) / radius, -1, 1))),
            thetaMax(std::acos(Clamp(std::max(zMin, zMax) / radius, -1, 1))),
            phiMax(Radians(Clamp(phiMax, 0, 360))) {}
        Bounds3f ObjectBound() const;
        bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const;
        Float Area() const;
        Interaction Sample(const Point2f &u, Float *pdf) const;

    private:
        // Distance Estimator Private Data.
        const Float radius;
        const Float zMin, zMax;
        const Float thetaMin, thetaMax, phiMax;
};

std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params);

} // namespace pbrt

#endif // PBRT_SHAPES_DISTANCE_ESTIMATOR
