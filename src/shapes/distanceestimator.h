
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
    
struct DistanceEstimatorParams {
    int maxIters = 1000; // Number of steps along the ray until we give up (default 1000)
    float hitEpsilon = 1e-5; // how close to the surface we must be before we say we "hit" it
    float rayEpsilonMultiplier = 5; // how much we multiply hitEpsilon by to get rayEpsilon
    float normalEpsilon = 1e-5; // The epsilon we send to CalculateNormal()
};
    
// DistanceEstimator Declarations
class DistanceEstimator : public Shape {
  public:
    // DistanceEstimator Public Methods
    DistanceEstimator(const Transform *ObjectToWorld, const Transform *WorldToObject,
		      bool reverseOrientation, DistanceEstimatorParams& DEparams)
				: Shape(ObjectToWorld, WorldToObject, reverseOrientation)
                {params = DEparams;}
    virtual Bounds3f ObjectBound() const = 0;
    virtual bool Intersect(const Ray& ray, Float *tHit, SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const;
    virtual Float Area() const = 0;
    virtual Interaction Sample(const Point2f &u, Float *pdf) const;
	
	virtual Float Evaluate(const Point3f& p) const = 0;
	virtual Vector3f CalculateNormal(const Point3f& pos, float eps, 
       const Vector3f& defaultNormal) const;
    
 protected:
    DistanceEstimatorParams params;
	
};

}  // namespace pbrt

#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
