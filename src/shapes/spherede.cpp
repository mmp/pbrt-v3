#include "shapes/spherede.h"
#include "paramset.h"

namespace pbrt {

// SphereDE Method Definitions.

Bounds3f SphereDE::ObjectBound() const {
    return Bounds3f(Point3f(-radius, -radius, -radius),
                    Point3f(radius, radius, radius));
}

Float SphereDE::Area() const {
    return 4 * Pi * radius * radius;
}

Float SphereDE::Evaluate(const Point3f &p) const {
    return Distance(p, Point3f()) - radius;
}

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w,
                                           const Transform *w2o,
                                           bool reverseOrientation,
                                           const ParamSet &params) {
    Float radius = params.FindOneFloat("radius", 1.0);

    DistanceEstimatorParams deParams;
    deParams.maxIters = params.FindOneInt("maxiters", 1000);
    deParams.hitEpsilon = params.FindOneFloat("hitepsilon", 1e-5);
    deParams.rayEpsilonMultiplier = params.FindOneFloat("rayepsilonmultiplier", 5);
    deParams.normalEpsilon = params.FindOneFloat("normalepsilon", 1e-5);

    return std::make_shared<SphereDE>(o2w, w2o, reverseOrientation, deParams, radius);
}

} // namespace pbrt
