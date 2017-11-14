#include "shapes/distanceestimator.h"
#include "paramset.h"
#include "stats.h"

namespace pbrt {

// Distance Estimator Method Definitions.

bool DistanceEstimator::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const {
    ProfilePhase p(Prof::ShapeIntersect);
    Point3f pHit;

    Ray ray = (*WorldToObject)(r);

    Float t = 0.0;
    int i = 0;
    while (t < ray.tMax && i < params.maxIters) {
        Point3f p = ray(t);
        Float d = Evaluate(p);

        if (d < params.hitEpsilon) {
            pHit = p;
            tHit = &t;

            Vector3f dpdu = Vector3f();
            Vector3f dpdv = Vector3f();
            Normal3f dndu = Normal3f();
            Normal3f dndv = Normal3f();

            Vector3f normal = CalculateNormal(p, params.normalEpsilon, -ray.d);
            CoordinateSystem(normal, &dpdu, &dpdv);

            // Compute error bounds for intersection.
            Float error = 10 * params.hitEpsilon;
            Vector3f pError = Vector3f(error, error, error);

            *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(), -ray.d, dpdu, dpdv, dndu, dndv, ray.time, this));

            return true;
        }

        t += d;
        ++i;
    }

    return false;
}

Interaction DistanceEstimator::Sample(const Point2f &u, Float *pdf) const {
    LOG(FATAL) << "DistanceEstimator::Sample not implemented.";
    return Interaction();
}

Vector3f DistanceEstimator::CalculateNormal(const Point3f &pos, float eps, const Vector3f &defaultNormal) const {
    const Vector3f v1 = Vector3f( 1.0,-1.0,-1.0);
    const Vector3f v2 = Vector3f(-1.0,-1.0, 1.0);
    const Vector3f v3 = Vector3f(-1.0, 1.0,-1.0);
    const Vector3f v4 = Vector3f( 1.0, 1.0, 1.0);

    const Vector3f normal =
        v1 * Evaluate( pos + v1*eps ) +
        v2 * Evaluate( pos + v2*eps ) +
        v3 * Evaluate( pos + v3*eps ) +
        v4 * Evaluate( pos + v4*eps );

    const Float length = normal.Length();

    return length > 0 ? (normal/length) : defaultNormal;
}

} // namespace pbrt
