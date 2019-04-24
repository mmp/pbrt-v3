/*
 An extension version of the PBRT source code for CS348B assignment 2: step 2
 
 Zheng Lyu, 2019 Spring
 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_INFINITESPHEREGRIDDE_H
#define PBRT_SHAPES_INFINITESPHEREGRIDDE_H

// distanceestimator/SphereDE.h*
#include "shapes/distanceestimator.h"

namespace pbrt {
    

    
class InfiniteSphereGridDE : public DistanceEstimator{
public:
    InfiniteSphereGridDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
             bool reverseOrientation, DistanceEstimatorParams& DEparams, Float cellSize):
        DistanceEstimator(ObjectToWorld,WorldToObject,
                      reverseOrientation, DEparams),
        cellSize(cellSize){}

    
    virtual Float Evaluate(const Point3f& p) const;
    virtual Float Area() const;
    virtual Bounds3f ObjectBound() const;
    
protected:
    const Float cellSize;
    
};

std::shared_ptr<Shape> CreateInfiniteSphereGridDEShape(const Transform *o2w,
                                           const Transform *w2o,
                                           bool reverseOrientation,
                                           const ParamSet &params);
    
}  // namespace pbrt
#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
