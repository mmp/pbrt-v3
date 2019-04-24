/*
 An extension version of the PBRT source code for CS348B assignment 2: step 2
 
 Zheng Lyu, 2019 Spring
 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_SPHEREDE_H
#define PBRT_SHAPES_SPHEREDE_H

// distanceestimator/SphereDE.h*
#include "shapes/distanceestimator.h"

namespace pbrt {
    
class SphereDE : public DistanceEstimator{
    public:
        SphereDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
                 bool reverseOrientation, DistanceEstimatorParams DEparams, Float radius):
                DistanceEstimator(ObjectToWorld,WorldToObject,
                                  reverseOrientation, DEparams),
                radius(radius){}
    
        virtual Float Evaluate(const Point3f& p) const;
        virtual Float Area() const;
        virtual Bounds3f ObjectBound() const;
    
    protected:
        const Float radius;

};

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params);
    
}  // namespace pbrt
#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
