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

bool DistanceEstimator::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const {
    // Initialize the parameters
    Float hitEpsilon = params.hitEpsilon;
    int maxIters = params.maxIters;
    Float rayEpsilonMultiplier = params.rayEpsilonMultiplier;
    Float normalEpsilon = params.normalEpsilon;
    
    ProfilePhase p(Prof::ShapeIntersect);
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);
    
    int step = 0;
    Float t = 0, dist = 0;
    
    pHit = ray(t);
    while((dist = Evaluate(pHit)) > hitEpsilon) {
        t += (dist / (ray.d).Length());
        if (t > ray.tMax || ++step > maxIters || std::isinf(t)) {
            return false;
        }
        pHit = ray(t);
    }
    
    if (tHit != NULL)
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
    
    if (isect != NULL)
        *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, uv, wo, dpdu, dpdv, dndu, dndv, ray.time,
                                                     this));
    
    return true;
    
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

}  // namespace pbrt
