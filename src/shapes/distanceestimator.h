
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
		      bool reverseOrientation, Float radius, Float zMin, Float zMax,
           Float phiMax)
        : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
          radius(radius),
          zMin(Clamp(std::min(zMin, zMax), -radius, radius)),
          zMax(Clamp(std::max(zMin, zMax), -radius, radius)),
          thetaMin(std::acos(Clamp(std::min(zMin, zMax) / radius, -1, 1))),
          thetaMax(std::acos(Clamp(std::max(zMin, zMax) / radius, -1, 1))),
          phiMax(Radians(Clamp(phiMax, 0, 360))) {}
    Bounds3f ObjectBound() const;
    bool Intersect(const Ray& ray, Float *tHit, SurfaceInteraction *isect,
		   bool testAlphaTexture = true) const;
    Float Area() const;
    Interaction Sample(const Point2f &u, Float *pdf) const;

  protected:
    // DistanceEstimator Protected Data
    //DistanceEstimatorParams params;
    Float radius;
    Float zMin, zMax;
    Float thetaMin, thetaMax, phiMax;
};

std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
								const Transform *w2o,
								bool reverseOrientation,
								const ParamSet &params);
}  // namespace pbrt

#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H
