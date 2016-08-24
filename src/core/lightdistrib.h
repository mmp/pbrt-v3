
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef PBRT_CORE_LIGHTDISTRIB_H
#define PBRT_CORE_LIGHTDISTRIB_H

#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>
#include "geometry.h"
#include "pbrt.h"
#include "sampling.h"

// LightDistribution defines a general interface for classes that provide
// probability distributions for sampling light sources at a given point in
// space.
class LightDistribution {
  public:
    virtual ~LightDistribution();

    // Given a point |p| in space, this method returns a (hopefully
    // effective) sampling distribution for light sources at that point.
    virtual const Distribution1D *Lookup(const Point3f &p) const = 0;
};

std::unique_ptr<LightDistribution> CreateLightSampleDistribution(
    const std::string &name, const Scene &scene);

// The simplest possible implementation of LightDistribution: this returns
// a uniform distribution over all light sources, ignoring the provided
// point. This approach works well for very simple scenes, but is quite
// ineffective for scenes with more than a handful of light sources. (This
// was the sampling method originally used for the PathIntegrator and the
// VolPathIntegrator in the printed book, though without the
// UniformLightDistribution class.)
class UniformLightDistribution : public LightDistribution {
  public:
    UniformLightDistribution(const Scene &scene);
    const Distribution1D *Lookup(const Point3f &p) const;

  private:
    std::unique_ptr<Distribution1D> distrib;
};

// PowerLightDistribution returns a distribution with sampling probability
// proportional to the total emitted power for each light. (It also ignores
// the provided point |p|.)  This approach works well for scenes where
// there the most powerful lights are also the most important contributors
// to lighting in the scene, but doesn't do well if there are many lights
// and if different lights are relatively important in some areas of the
// scene and unimportant in others. (This was the default sampling method
// used for the BDPT integrator and MLT integrator in the printed book,
// though also without the PowerLightDistribution class.)
class PowerLightDistribution : public LightDistribution {
  public:
    PowerLightDistribution(const Scene &scene);
    const Distribution1D *Lookup(const Point3f &p) const;

  private:
    std::unique_ptr<Distribution1D> distrib;
};

// Helper struct to compute a hash function of an integer 3D point.
//
// TODO: This can almost certainly be be improved. Review
// e.g. https://github.com/aappleby/smhasher and
// http://stackoverflow.com/questions/8513911/how-to-create-a-good-hash-combine-with-64-bit-output-inspired-by-boosthash-co.
struct Point3iHash {
    size_t operator()(const Point3i &p) const {
        std::hash<int> hasher;
        size_t h = hasher(p.x);
        h ^= hasher(p.y ^ 0xfa60a2bc);
        h ^= hasher(p.z ^ 0x7051259b);
        return h;
    }
};

// A spatially-varying light distribution that adjusts the probability of
// sampling a light source based on an estimate of its contribution to a
// region of space.  A fixed voxel grid is imposed over the scene bounds
// and a sampling distribution is computed as needed for each voxel.
class SpatialLightDistribution : public LightDistribution {
  public:
    SpatialLightDistribution(const Scene &scene, int maxVoxels = 64);
    ~SpatialLightDistribution();
    const Distribution1D *Lookup(const Point3f &p) const;

  private:
    static const int nBuckets = 32;
#ifndef PBRT_IS_MSVC2013
    static_assert(IsPowerOf2(nBuckets),
                  "nBuckets must be an exact power of two");
#endif // !PBRT_IS_MSVC2013

    const Scene &scene;
    int nVoxels[3];

    // The main challenge with the implementation of this approach is good
    // scalability with multiple threads--we only compute sampling
    // distributions for a voxel when needed, but then want to make them
    // available to all threads. Therefore, we have implemented a two-level
    // hash table: the first hash table is of fixed size, nBuckets. Each
    // bucket in it is protected by the corresponding mutex in mutexes. In
    // turn, each hash table bucket is itself a hash table that goes from
    // integer voxel coordinates to light sampling distributions.
    mutable std::mutex mutexes[nBuckets];
    using BucketHash =
        std::unordered_map<Point3i, std::unique_ptr<Distribution1D>,
                           Point3iHash>;
    mutable BucketHash voxelDistribution[nBuckets];
};

#endif  // PBRT_CORE_LIGHTDISTRIB_H
