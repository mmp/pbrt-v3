/*
 DistanceEstimator .cpp scripts implementation
 
 Zheng Lyu, Spring 2019.
 */

// shapes/distanceestimator.cpp*
#include "shapes/spherede.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt{
    
// DistanceEstimator Method Definitions
Bounds3f SphereDE::ObjectBound() const {
    return Bounds3f(Point3f(-radius, -radius, -radius),
                    Point3f(radius, radius, radius));
}



Float SphereDE::Area() const { return 4 * 3.1415926 * radius * radius; }

// Newly implemented functions
Float SphereDE::Evaluate(const Point3f& p) const {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - radius;
}
    

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params)   {
    Float radius = params.FindOneFloat("radius", 1.f);
    DistanceEstimatorParams DEparams;
    DEparams.maxIters = params.FindOneInt("maxIters", 1000000);
    DEparams.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-5);
    DEparams.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
    DEparams.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5);
    return std::make_shared<SphereDE>(o2w, w2o, reverseOrientation, DEparams, radius);
}
    
}  // namespace pbrt
