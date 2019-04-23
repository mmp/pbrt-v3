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

bool SphereDE::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const {
    // Initialize the parameters
    Float hitEpsilon = DEparams.hitEpsilon;
    int maxIters = DEparams.maxIters;
    Float rayEpsilonMultiplier = DEparams.rayEpsilonMultiplier;
    Float normalEpsilon = DEparams.normalEpsilon;
    
    ProfilePhase p(Prof::ShapeIntersect);
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);
    
    int step = 0;
    Float t =0, dist = 0;
    
    pHit = ray(t);
    while((dist = Evaluate(pHit)) > hitEpsilon) {
        t += (dist / (ray.d).Length());
        if (t > ray.tMax || ++step > maxIters || std::isinf(t)) {
            return false;
        }
        pHit = ray(t);
    }
    
    *tHit = t;
    
    // Setup parameters for isect
    Vector3f pError = Vector3f(rayEpsilonMultiplier * hitEpsilon, rayEpsilonMultiplier * hitEpsilon, rayEpsilonMultiplier * hitEpsilon);
    Point2f uv = Point2f(0, 0);
    Vector3f wo = -ray.d;
    Normal3f dndu = Normal3f(0, 0, 0);
    Normal3f dndv = Normal3f(0, 0, 0);
    
    
    // Calculate dpdu and dpdv
    Vector3f n = CalculateNormal(pHit, normalEpsilon, Vector3f(1, 1, 1));
    Vector3f dpdu, dpdv;
    if (std::abs(n.x) > std::abs(n.y))
        dpdu = Vector3f(-n.z, 0, n.x) / std::sqrt(n.x * n.x + n.z * n.z);
    else
        dpdu = Vector3f(0, n.z, -n.y) / std::sqrt(n.y * n.y + n.z * n.z);
    dpdv = Cross(n, dpdu);
    
    
    // Initialize _SurfaceInteraction_ from parametric information
    
    *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, uv, wo, dpdu, dpdv, dndu, dndv, ray.time,
                                                 this));
    
    return true;
    
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
    SphereDEParams DEparams;
    DEparams.maxIters = params.FindOneInt("maxIters", 1000000);
    DEparams.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-5);
    DEparams.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
    DEparams.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5);
    return std::make_shared<SphereDE>(o2w, w2o, reverseOrientation, DEparams, radius);
}
    
}  // namespace pbrt
