#include "shapes/distanceestimator.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

// Distance Estimator Method Definitions.
Bounds3f DistanceEstimator::ObjectBound() const {
    return Bounds3f(Point3f(-radius, -radius, -radius),
                    Point3f(radius, radius, radius));
}

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

Float DistanceEstimator::Area() const {
    return 4 * Pi * radius * radius;
}

Interaction DistanceEstimator::Sample(const Point2f &u, Float *pdf) const {
    LOG(FATAL) << "DistanceEstimator::Sample not implemented.";
    return Interaction();
}

Float DistanceEstimator::Evaluate(const Point3f &p) const {
    return Distance(p, Point3f()) - radius;
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

std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
                                               const Transform *w2o,
                                               bool reverseOrientation,
                                               const ParamSet &params) {
    Float radius = params.FindOneFloat("radius", 1.0);

    DistanceEstimatorParams deParams;
    deParams.maxIters = params.FindOneInt("maxiters", 1000);
    deParams.hitEpsilon = params.FindOneFloat("hitepsilon", 1e-5);
    deParams.rayEpsilonMultiplier = params.FindOneFloat("rayepsilonmultiplier", 5);
    deParams.normalEpsilon = params.FindOneFloat("normalepsilon", 1e-5);

    return std::make_shared<DistanceEstimator>(o2w, w2o, reverseOrientation, radius, deParams);
}

} // namespace pbrt
