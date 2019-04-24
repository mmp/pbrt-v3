/*
 DistanceEstimator .cpp scripts implementation
 
 Zheng Lyu, Spring 2019.
 */

// shapes/distanceestimator.cpp*
#include "shapes/mandelbulbde.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt{
    
// DistanceEstimator Method Definitions
Bounds3f MandelbulbDE::ObjectBound() const {
    Point3f lower = Point3f(-1, -1, -1);
    Point3f upper = Point3f(1, 1, 1);
    return Bounds3f(lower,
                    upper);
}
    
    
    
Float MandelbulbDE::Area() const { return std::numeric_limits<float>::infinity();}
    
// Newly implemented functions
Float MandelbulbDE::Evaluate(const Point3f& p) const {
    const float bailout = 2.0f;
    const float Power = (float)mandelbulbPower;
    Point3f z = p;
    float dr = 1.0;
    float r = 0.0;
    for (int i = 0; i < fractalIters; i++) {
        r = (z-Point3f(0,0,0)).Length();
        if (r>bailout) break;
        
        // convert to polar coordinates
        float theta = acos(z.z/r);
        float phi = atan2(z.y,z.x);
        dr =  pow( r, Power-1.0)*Power*dr + 1.0;
        
        // scale and rotate the point
        float zr = pow( r,Power);
        theta = theta*Power;
        phi = phi*Power;
        
        // convert back to cartesian coordinates
        z = zr*Point3f(sin(theta)*cos(phi), sin(phi)*sin(theta), cos(theta));
        z += p;
    }
    return 0.5*log(r)*r/dr;
}
    
    
std::shared_ptr<Shape> CreateMandelbulbDEShape(const Transform *o2w,
                                                       const Transform *w2o,
                                                       bool reverseOrientation,
                                                       const ParamSet &params)   {

    DistanceEstimatorParams DEparams;
    DEparams.maxIters = params.FindOneInt("maxiters", 1000);
    DEparams.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-5);
    DEparams.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 10);
    DEparams.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5);
    
    int fractalIters = params.FindOneInt("fractalIters", 1000);
    int mandelbulbPower = params.FindOneInt("mandelbulbPower", 8);
    return std::make_shared<MandelbulbDE>(o2w, w2o, reverseOrientation, DEparams, fractalIters, mandelbulbPower);
}
    
}  // namespace pbrt
