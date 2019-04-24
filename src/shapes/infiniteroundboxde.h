/*
 An extension version of the PBRT source code for CS348B assignment 2: step 7
 
 Zheng Lyu, 2019 Spring
 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_INFINITEROUNDBOXDE_H
#define PBRT_SHAPES_INFINITEROUNDBOXDE_H

// distanceestimator/SphereDE.h*
#include "shapes/distanceestimator.h"

namespace pbrt {
    
class InfiniteRoundBoxDE : public DistanceEstimator{
    public:
        InfiniteRoundBoxDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
                             bool reverseOrientation, DistanceEstimatorParams& DEparams, Vector3f box ,Float radius, Float cellSize):
        DistanceEstimator(ObjectToWorld,WorldToObject,
                          reverseOrientation, DEparams),
        box(box),
        radius(radius),
        cellSize(cellSize){}
    
    
        virtual Float Evaluate(const Point3f& p) const;
        virtual Float Area() const;
        virtual Bounds3f ObjectBound() const;
        virtual Vector3f MaxVector3fSelf(const Point3f& p1, const Point3f& p2) const;
        virtual Vector3f MinVector3fSelf(const Point3f& p1, const Point3f& p2) const;
    
    protected:
        const Vector3f box;
        const Float radius;
        const Float cellSize;
    
};

std::shared_ptr<Shape> CreateInfiniteRoundBoxDEShape(const Transform *o2w,
                                                       const Transform *w2o,
                                                       bool reverseOrientation,
                                                       const ParamSet &params);
    
}  // namespace pbrt
#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
