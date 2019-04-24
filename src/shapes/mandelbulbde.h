/*
 An extension version of the PBRT source code for CS348B assignment 2: step 2
 
 Zheng Lyu, 2019 Spring
 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_MANDELBULBDE_H
#define PBRT_SHAPES_MANDELBULBDE_H

// distanceestimator/MandelbulbDE.h*
#include "shapes/distanceestimator.h"

namespace pbrt {
    
class MandelbulbDE : public DistanceEstimator{
public:
    MandelbulbDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
                         bool reverseOrientation, DistanceEstimatorParams DEparams, int fractalIters, int mandelbulbPower):
    DistanceEstimator(ObjectToWorld,WorldToObject,
                      reverseOrientation, DEparams),
    fractalIters(fractalIters),
    mandelbulbPower(mandelbulbPower){}
    
    virtual Float Evaluate(const Point3f& p) const;
    virtual Float Area() const;
    virtual Bounds3f ObjectBound() const;
    
protected:
    const int fractalIters, mandelbulbPower;
    
};

std::shared_ptr<Shape> CreateMandelbulbDEShape(const Transform *o2w,
                                                       const Transform *w2o,
                                                       bool reverseOrientation,
                                                       const ParamSet &params);
    
}  // namespace pbrt
#endif // PBRT_SHAPES_MANDELBULBDE_H
