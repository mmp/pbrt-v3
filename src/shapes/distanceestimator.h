
/*
  An extension version of the PBRT source code for CS348B assignment 2: step 2

  Zheng Lyu, 2019 Spring
*/

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCEESTIMATOR_H
#define PBRT_SHAPES_DISTANCEESTIMATOR_H

// shapes/distanceestimator.h*
#include "shape.h"

namespace pbrt {
    

    
// DistanceEstimator Declarations
class DistanceEstimator : public Shape {
  public:
    // DistanceEstimator Public Methods
    DistanceEstimator(const Transform *ObjectToWorld, const Transform *WorldToObject,
		      bool reverseOrientation)
				: Shape(ObjectToWorld, WorldToObject, reverseOrientation)
                    {}
    virtual Bounds3f ObjectBound() const = 0;
    virtual bool Intersect(const Ray& ray, Float *tHit, SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const = 0;
    virtual Float Area() const = 0;
    virtual Interaction Sample(const Point2f &u, Float *pdf) const;
	
	virtual Float Evaluate(const Point3f& p) const = 0;
	virtual Vector3f CalculateNormal(const Point3f& pos, float eps, 
       const Vector3f& defaultNormal) const;
    
 //protected:
 //   const DistanceEstimatorParams DEparams;
	
};

//std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
//                                           const Transform *w2o,
//                                           bool reverseOrientation,
//                                           const ParamSet &params);
    
}  // namespace pbrt

#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
