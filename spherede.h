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

// shapes/distanceestimator.h*
#include "distanceestimator.h"

namespace pbrt {

class SphereDE : public DistanceEstimator{
    public:
    SphereDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
             bool reverseOrientation, Float radius, DistanceEstimatorParams DEparams):
            DistanceEstimator(ObjectToWorld,WorldToObject,
                              reverseOrientation, radius,DEparams){}
    
    
    virtual Float Evaluate(const Point3f& p) const;
    virtual Float Area() const;
    virtual Bounds3f ObjectBound() const;
};
    
}
