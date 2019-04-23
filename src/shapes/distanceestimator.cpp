/*
  DistanceEstimator .cpp scripts implementation

  Zheng Lyu, Spring 2019.
*/

// shapes/distanceestimator.cpp*
#include "shapes/distanceestimator.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt{
  
// DistanceEstimator Method Definitions


Interaction DistanceEstimator::Sample(const Point2f &u, Float *pdf) const {
	return Interaction();
}

Vector3f DistanceEstimator::CalculateNormal(const Point3f& pos, float eps, 
       const Vector3f& defaultNormal) const {
const Vector3f v1 = Vector3f( 1.0,-1.0,-1.0);
const Vector3f v2 = Vector3f(-1.0,-1.0, 1.0);
const Vector3f v3 = Vector3f(-1.0, 1.0,-1.0);
const Vector3f v4 = Vector3f( 1.0, 1.0, 1.0);

const Vector3f normal = v1 * Evaluate( pos + v1*eps ) +
             v2 * Evaluate( pos + v2*eps ) +
             v3 * Evaluate( pos + v3*eps ) +
             v4 * Evaluate( pos + v4*eps );
const Float length = normal.Length();

return length > 0 ? (normal/length) : defaultNormal;
}



//std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
//                                         const Transform *w2o,
//                                         bool reverseOrientation,
//                                         const ParamSet &params)   {
//  DistanceEstimatorParams DEparams;
//  DEparams.maxIters = params.FindOneInt("maxIters", 1000000);
//  DEparams.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-5);
//  DEparams.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
//  DEparams.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5);
//  return std::make_shared<DistanceEstimator>(o2w, w2o, reverseOrientation, DEparams);
//}

}  // namespace pbrt
