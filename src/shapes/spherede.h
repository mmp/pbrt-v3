/*
 An extension version of the PBRT source code for CS348B assignment 2: step 2
 
 Zheng Lyu, 2019 Spring
 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCEESTIMATOR_SPHEREDE_H
#define PBRT_SHAPES_DISTANCEESTIMATOR_SPHEREDE_H

// distanceestimator/SphereDE.h*
#include "distanceestimator.h"

namespace pbrt {

struct DistanceEstimatorParams {
    int maxIters = 1000; // Number of steps along the ray until we give up (default 1000)
    Float hitEpsilon = 1e-5; // how close to the surface we must be before we say we "hit" it
    Float rayEpsilonMultiplier = 10; // how much we multiply hitEpsilon by to get pError
    Float normalEpsilon = 1e-5; // The epsilon we send to CalculateNormal()
};
    
class SphereDE : public DistanceEstimator{
    public:
        SphereDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
                 bool reverseOrientation, Float radius, DistanceEstimatorParams DEparams):
                DistanceEstimator(ObjectToWorld,WorldToObject,
                                  reverseOrientation),
                radius(radius),
                DEparams(DEparams){}
    
    
        virtual Float Evaluate(const Point3f& p) const;
        virtual Float Area() const;
        virtual Bounds3f ObjectBound() const;
    
    protected:
        const Float radius;
        const DistanceEstimatorParams DEparams;
};

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params);
    
}  // namespace pbrt
#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
