/*
 DistanceEstimator .cpp scripts implementation
 
 Zheng Lyu, Spring 2019.
 */

// shapes/distanceestimator.cpp*
#include "shapes/infinitespheregridde.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt{
    
// DistanceEstimator Method Definitions
Bounds3f InfiniteSphereGridDE::ObjectBound() const {
    Point3f lower = Point3f(-1e5, -1e5, -1e5);
    Point3f upper = Point3f(1e5, 1e5, 1e5);
    return Bounds3f(lower,
                    upper);
}
    

    
Float InfiniteSphereGridDE::Area() const { return std::numeric_limits<float>::infinity();}
    
// Newly implemented functions
Float InfiniteSphereGridDE::Evaluate(const Point3f& p) const {
    Point3f pGrid = Point3f(remainder(p.x, cellSize),
                          remainder(p.y, cellSize),
                          remainder(p.z, cellSize));
    return std::sqrt(pGrid.x * pGrid.x + pGrid.y * pGrid.y + pGrid.z * pGrid.z) - 1.0;
}
    
    
std::shared_ptr<Shape> CreateInfiniteSphereGridDEShape(const Transform *o2w,
                                           const Transform *w2o,
                                           bool reverseOrientation,
                                           const ParamSet &params)   {
    Float cellSize = params.FindOneFloat("cellSize", 1.f);
    DistanceEstimatorParams DEparams;
    DEparams.maxIters = params.FindOneInt("maxiters", 1000000);
    DEparams.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-5);
    DEparams.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
    DEparams.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5);
    return std::make_shared<InfiniteSphereGridDE>(o2w, w2o, reverseOrientation, DEparams, cellSize);
}
    
}  // namespace pbrt
