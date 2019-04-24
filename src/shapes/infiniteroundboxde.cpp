/*
 DistanceEstimator .cpp scripts implementation
 
 Zheng Lyu, Spring 2019.
 */

// shapes/distanceestimator.cpp*
#include "shapes/infiniteroundboxde.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"
namespace pbrt{
    
    // DistanceEstimator Method Definitions
    Bounds3f InfiniteRoundBoxDE::ObjectBound() const {
        Point3f lower = Point3f(-1e5, -1e5, -1e5);
        Point3f upper = Point3f(1e5, 1e5, 1e5);
        return Bounds3f(lower,
                        upper);
    }
    
    
    
    Float InfiniteRoundBoxDE::Area() const { return std::numeric_limits<float>::infinity();}
    
    // Newly implemented functions
    Float InfiniteRoundBoxDE::Evaluate(const Point3f& p) const {
        Point3f pGrid = Abs(Point3f(remainder(p.x, cellSize),
                                remainder(p.y, cellSize),
                                remainder(p.z, cellSize)));
        
        Point3f d = Point3f(pGrid.x - box.x, pGrid.y - box.y, pGrid.z - box.z);
        
        return (MaxVector3fSelf(d, Point3f(0, 0, 0))).Length() - radius + std::min<Float>(std::max(d.x, std::max(d.y, d.z)), 0);
    }
    
    Vector3f InfiniteRoundBoxDE::MaxVector3fSelf(const Point3f& p1, const Point3f& p2) const {
        return Vector3f(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
    }

    Vector3f InfiniteRoundBoxDE::MinVector3fSelf(const Point3f& p1, const Point3f& p2) const {
        return Vector3f(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
    }
    
    
    std::shared_ptr<Shape> CreateInfiniteRoundBoxDEShape(const Transform *o2w,
                                                           const Transform *w2o,
                                                           bool reverseOrientation,
                                                           const ParamSet &params)   {
        Float radius = params.FindOneFloat("radius", 1.f);
        Float cellSize = params.FindOneFloat("cellSize", 1.f);
        Vector3f box = params.FindOneVector3f("box", Vector3f(1.5, 1.5, 1.5));
        
        DistanceEstimatorParams DEparams;
        DEparams.maxIters = params.FindOneInt("maxiters", 1000000);
        DEparams.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-5);
        DEparams.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
        DEparams.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5);
        return std::make_shared<InfiniteRoundBoxDE>(o2w, w2o, reverseOrientation, DEparams, box, radius, cellSize);
    }
    
}  // namespace pbrt
