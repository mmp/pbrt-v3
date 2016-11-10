
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

// TODO (maybe): have integrators pre-prime the cache by rendering a very
// low res image first?

#include "lightdistrib.h"
#include "lowdiscrepancy.h"
#include "parallel.h"
#include "scene.h"
#include "stats.h"
#include <numeric>

LightDistribution::~LightDistribution() {}

std::unique_ptr<LightDistribution> CreateLightSampleDistribution(
    const std::string &name, const Scene &scene) {
    if (name == "uniform" || scene.lights.size() == 1)
        return std::unique_ptr<LightDistribution>{
            new UniformLightDistribution(scene)};
    else if (name == "power")
        return std::unique_ptr<LightDistribution>{
            new PowerLightDistribution(scene)};
    else if (name == "spatial")
        return std::unique_ptr<LightDistribution>{
            new SpatialLightDistribution(scene)};
    else {
        Error(
            "Light sample distribution type \"%s\" unknown. Using \"spatial\".",
            name.c_str());
        return std::unique_ptr<LightDistribution>{
            new SpatialLightDistribution(scene)};
    }
}

UniformLightDistribution::UniformLightDistribution(const Scene &scene) {
    std::vector<Float> prob(scene.lights.size(), Float(1));
    distrib.reset(new Distribution1D(&prob[0], prob.size()));
}

const Distribution1D *UniformLightDistribution::Lookup(const Point3f &p) const {
    return distrib.get();
}

PowerLightDistribution::PowerLightDistribution(const Scene &scene)
    : distrib(ComputeLightPowerDistribution(scene)) {}

const Distribution1D *PowerLightDistribution::Lookup(const Point3f &p) const {
    return distrib.get();
}

///////////////////////////////////////////////////////////////////////////
// SpatialLightDistribution

STAT_COUNTER("SpatialLightDistribution/Distributions created", nCreated);
STAT_INT_DISTRIBUTION("SpatialLightDistribution/Hash bucket load",
                      hashBucketLoad);
STAT_RATIO("SpatialLightDistribution/Lookups per distribution", nLookups, nDistributions);

SpatialLightDistribution::SpatialLightDistribution(const Scene &scene,
                                                   int maxVoxels)
    : scene(scene) {
    // Compute the number of voxels so that the widest scene bounding box
    // dimension has maxVoxels voxels and the other dimensions have a number
    // of voxels so that voxels are roughly cube shaped.
    Bounds3f b = scene.WorldBound();
    Vector3f diag = b.Diagonal();
    Float bmax = diag[b.MaximumExtent()];
    for (int i = 0; i < 3; ++i)
        nVoxels[i] = std::max(1, int(std::round(diag[i] / bmax * maxVoxels)));

    LOG(INFO) << "SpatialLightDistribution: scene bounds " << b <<
        ", voxel res (" << nVoxels[0] << ", " << nVoxels[1] << ", " <<
        nVoxels[2] << ")";

    // It's important to pre-size the localVoxelDistributions vector, to
    // avoid race conditions with one thread resizing the vector while
    // another is reading from it.
    localVoxelDistributions.resize(MaxThreadIndex());
}

SpatialLightDistribution::~SpatialLightDistribution() {
    // Gather statistics about how well the computed distributions are across
    // the buckets.
    // This is slightly ugly: we are depending on the destructor running
    // before statistics are reported (which is currently the case at least).
    for (size_t i = 0; i < nBuckets; ++i) {
        LOG(INFO) << "Bucket " << i << ", size " << voxelDistribution[i].size()
                  << ", bucket count " << voxelDistribution[i].bucket_count()
                  << ", load factor " << voxelDistribution[i].load_factor()
                  << ", max load factor " << voxelDistribution[i].max_load_factor();
        ReportValue(hashBucketLoad, voxelDistribution[i].size());
    }
}

const Distribution1D *SpatialLightDistribution::Lookup(const Point3f &p) const {
    ProfilePhase _(Prof::LightDistribLookup);
    ++nLookups;

    // Compute integer voxel coordinates for the given point |p| with
    // respect to the overall voxel grid.
    Vector3f offset = scene.WorldBound().Offset(p);  // offset in [0,1].
    Point3i pi;
    for (int i = 0; i < 3; ++i) pi[i] = int(offset[i] * nVoxels[i]);

    // Create the per-thread cache of sampling distributions if needed.
    LocalBucketHash *localVoxelDistribution =
        localVoxelDistributions[ThreadIndex].get();
    if (!localVoxelDistribution) {
        LOG(INFO) << "Created per-thread SpatialLightDistribution for thread" <<
            ThreadIndex;
        localVoxelDistribution = new LocalBucketHash;
        localVoxelDistributions[ThreadIndex].reset(localVoxelDistribution);
    }
    else {
        // Otherwise see if we have a sampling distribution for the voxel
        // that |p| is in already available in the local cache.
        auto iter = localVoxelDistribution->find(pi);
        if (iter != localVoxelDistribution->end())
            return iter->second;
    }

    // Now we need to either get the distribution from the shared hash
    // table (if another thread has already created it), or create it
    // ourselves and add it to the shared table.
    ProfilePhase __(Prof::LightDistribLookupL2);

    // First, compute a hash into the first-level hash table.
    size_t hash = std::hash<int>{}(pi[0] + nVoxels[0] * pi[1] +
                                   nVoxels[0] * nVoxels[1] * pi[2]);
    hash &= (nBuckets - 1);

    // Acquire the lock for the corresponding second-level hash table.
    std::lock_guard<std::mutex> lock(mutexes[hash]);
    // See if we can find it.
    auto iter = voxelDistribution[hash].find(pi);
    if (iter != voxelDistribution[hash].end()) {
        // Success. Add the pointer to the thread-specific hash table so
        // that we can look up this distribution more efficiently in the
        // future.
        (*localVoxelDistribution)[pi] = iter->second.get();
        return iter->second.get();
    }

    // We need to compute a new sampling distriibution for this voxel. Note
    // that we're holding the lock for the first-level hash table bucket
    // throughout the following; in general, we'd like to do the following
    // quickly so that other threads don't get held up waiting for that
    // lock (for this or other voxels that share it).
    ProfilePhase ___(Prof::LightDistribCreation);
    ++nCreated;
    ++nDistributions;

    // Compute the world-space bounding box of the voxel.
    Point3f p0(Float(pi[0]) / Float(nVoxels[0]),
               Float(pi[1]) / Float(nVoxels[1]),
               Float(pi[2]) / Float(nVoxels[2]));
    Point3f p1(Float(pi[0] + 1) / Float(nVoxels[0]),
               Float(pi[1] + 1) / Float(nVoxels[1]),
               Float(pi[2] + 1) / Float(nVoxels[2]));
    Bounds3f voxelBounds(scene.WorldBound().Lerp(p0),
                         scene.WorldBound().Lerp(p1));

    // Compute the sampling distribution. Sample a number of points inside
    // voxelBounds using a 3D Halton sequence; at each one, sample each
    // light source and compute a weight based on Li/pdf for the light's
    // sample (ignoring visibility between the point in the voxel and the
    // point on the light source) as an approximation to how much the light
    // is likely to contribute to illumination in the voxel.
    int nSamples = 128;
    std::vector<Float> lightContrib(scene.lights.size(), Float(0));
    for (int i = 0; i < nSamples; ++i) {
        Point3f po = voxelBounds.Lerp(Point3f(
            RadicalInverse(0, i), RadicalInverse(1, i), RadicalInverse(2, i)));
        Interaction intr(po, Normal3f(), Vector3f(), Vector3f(1, 0, 0),
                         0 /* time */, MediumInterface());

        // Use the next two Halton dimensions to sample a point on the
        // light source.
        Point2f u(RadicalInverse(3, i), RadicalInverse(4, i));
        for (size_t j = 0; j < scene.lights.size(); ++j) {
            Float pdf;
            Vector3f wi;
            VisibilityTester vis;
            Spectrum Li = scene.lights[j]->Sample_Li(intr, u, &wi, &pdf, &vis);
            if (pdf > 0) {
                // TODO: look at tracing shadow rays / computing beam
                // transmittance.  Probably shouldn't give those full weight
                // but instead e.g. have an occluded shadow ray scale down
                // the contribution by 10 or something.
                lightContrib[j] += Li.y() / pdf;
            }
        }
    }

    // We don't want to leave any lights with a zero probability; it's
    // possible that a light contributes to points in the voxel even though
    // we didn't find such a point when sampling above.  Therefore, compute
    // a minimum (small) weight and ensure that all lights are given at
    // least the corresponding probability.
    Float sumContrib =
        std::accumulate(lightContrib.begin(), lightContrib.end(), Float(0));
    Float avgContrib = sumContrib / (nSamples * lightContrib.size());
    Float minContrib = (avgContrib > 0) ? .001 * avgContrib : 1;
    for (size_t i = 0; i < lightContrib.size(); ++i) {
        VLOG(2) << "Voxel pi = " << pi << ", light " << i << " contrib = "
                << lightContrib[i];
        lightContrib[i] = std::max(lightContrib[i], minContrib);
    }
    LOG(INFO) << "Initialized light distribution in voxel pi= " <<  pi <<
        ", avgContrib = " << avgContrib;

    // Compute a sampling distribution from the accumulated
    // contributions.
    std::unique_ptr<Distribution1D> distrib(
        new Distribution1D(&lightContrib[0], lightContrib.size()));

    // Store a pointer to it in the per-thread cache for the future.
    (*localVoxelDistribution)[pi] = distrib.get();

    // Store the canonical unique_ptr for it in the global hash table so
    // other threads can use it.
    voxelDistribution[hash][pi] = std::move(distrib);

    return (*localVoxelDistribution)[pi];
}
