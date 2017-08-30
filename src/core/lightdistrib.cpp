
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

namespace pbrt {

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
    distrib.reset(new Distribution1D(&prob[0], int(prob.size())));
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
STAT_RATIO("SpatialLightDistribution/Lookups per distribution", nLookups, nDistributions);
STAT_INT_DISTRIBUTION("SpatialLightDistribution/Hash probes per lookup", nProbesPerLookup);

// Voxel coordinates are packed into a uint64_t for hash table lookups;
// 10 bits are allocated to each coordinate.  invalidPackedPos is an impossible
// packed coordinate value, which we use to represent
static const uint64_t invalidPackedPos = 0xffffffffffffffff;

SpatialLightDistribution::SpatialLightDistribution(const Scene &scene,
                                                   int maxVoxels)
    : scene(scene) {
    // Compute the number of voxels so that the widest scene bounding box
    // dimension has maxVoxels voxels and the other dimensions have a number
    // of voxels so that voxels are roughly cube shaped.
    Bounds3f b = scene.WorldBound();
    Vector3f diag = b.Diagonal();
    Float bmax = diag[b.MaximumExtent()];
    for (int i = 0; i < 3; ++i) {
        nVoxels[i] = std::max(1, int(std::round(diag[i] / bmax * maxVoxels)));
        // In the Lookup() method, we require that 20 or fewer bits be
        // sufficient to represent each coordinate value. It's fairly hard
        // to imagine that this would ever be a problem.
        CHECK_LT(nVoxels[i], 1 << 20);
    }

    hashTableSize = 4 * nVoxels[0] * nVoxels[1] * nVoxels[2];
    hashTable.reset(new HashEntry[hashTableSize]);
    for (int i = 0; i < hashTableSize; ++i) {
        hashTable[i].packedPos.store(invalidPackedPos);
        hashTable[i].distribution.store(nullptr);
    }

    LOG(INFO) << "SpatialLightDistribution: scene bounds " << b <<
        ", voxel res (" << nVoxels[0] << ", " << nVoxels[1] << ", " <<
        nVoxels[2] << ")";
}

SpatialLightDistribution::~SpatialLightDistribution() {
    // Gather statistics about how well the computed distributions are across
    // the buckets.
    for (size_t i = 0; i < hashTableSize; ++i) {
        HashEntry &entry = hashTable[i];
        if (entry.distribution.load())
            delete entry.distribution.load();
    }
}

const Distribution1D *SpatialLightDistribution::Lookup(const Point3f &p) const {
    ProfilePhase _(Prof::LightDistribLookup);
    ++nLookups;

    // First, compute integer voxel coordinates for the given point |p|
    // with respect to the overall voxel grid.
    Vector3f offset = scene.WorldBound().Offset(p);  // offset in [0,1].
    Point3i pi;
    for (int i = 0; i < 3; ++i)
        // The clamp should almost never be necessary, but is there to be
        // robust to computed intersection points being slightly outside
        // the scene bounds due to floating-point roundoff error.
        pi[i] = Clamp(int(offset[i] * nVoxels[i]), 0, nVoxels[i] - 1);

    // Pack the 3D integer voxel coordinates into a single 64-bit value.
    uint64_t packedPos = (uint64_t(pi[0]) << 40) | (uint64_t(pi[1]) << 20) | pi[2];
    CHECK_NE(packedPos, invalidPackedPos);

    // Compute a hash value from the packed voxel coordinates.  We could
    // just take packedPos mod the hash table size, but since packedPos
    // isn't necessarily well distributed on its own, it's worthwhile to do
    // a little work to make sure that its bits values are individually
    // fairly random. For details of and motivation for the following, see:
    // http://zimbry.blogspot.ch/2011/09/better-bit-mixing-improving-on.html
    uint64_t hash = packedPos;
    hash ^= (hash >> 31);
    hash *= 0x7fb5d329728ea185;
    hash ^= (hash >> 27);
    hash *= 0x81dadef4bc2dd44d;
    hash ^= (hash >> 33);
    hash %= hashTableSize;
    CHECK_GE(hash, 0);

    // Now, see if the hash table already has an entry for the voxel. We'll
    // use quadratic probing when the hash table entry is already used for
    // another value; step stores the square root of the probe step.
    int step = 1;
    int nProbes = 0;
    while (true) {
        ++nProbes;
        HashEntry &entry = hashTable[hash];
        // Does the hash table entry at offset |hash| match the current point?
        uint64_t entryPackedPos = entry.packedPos.load(std::memory_order_acquire);
        if (entryPackedPos == packedPos) {
            // Yes! Most of the time, there should already by a light
            // sampling distribution available.
            Distribution1D *dist = entry.distribution.load(std::memory_order_acquire);
            if (dist == nullptr) {
                // Rarely, another thread will have already done a lookup
                // at this point, found that there isn't a sampling
                // distribution, and will already be computing the
                // distribution for the point.  In this case, we spin until
                // the sampling distribution is ready.  We assume that this
                // is a rare case, so don't do anything more sophisticated
                // than spinning.
                ProfilePhase _(Prof::LightDistribSpinWait);
                while ((dist = entry.distribution.load(std::memory_order_acquire)) ==
                       nullptr)
                    // spin :-(. If we were fancy, we'd have any threads
                    // that hit this instead help out with computing the
                    // distribution for the voxel...
                    ;
            }
            // We have a valid sampling distribution.
            ReportValue(nProbesPerLookup, nProbes);
            return dist;
        } else if (entryPackedPos != invalidPackedPos) {
            // The hash table entry we're checking has already been
            // allocated for another voxel. Advance to the next entry with
            // quadratic probing.
            hash += step * step;
            if (hash >= hashTableSize)
                hash %= hashTableSize;
            ++step;
        } else {
            // We have found an invalid entry. (Though this may have
            // changed since the load into entryPackedPos above.)  Use an
            // atomic compare/exchange to try to claim this entry for the
            // current position.
            uint64_t invalid = invalidPackedPos;
            if (entry.packedPos.compare_exchange_weak(invalid, packedPos)) {
                // Success; we've claimed this position for this voxel's
                // distribution. Now compute the sampling distribution and
                // add it to the hash table. As long as packedPos has been
                // set but the entry's distribution pointer is nullptr, any
                // other threads looking up the distribution for this voxel
                // will spin wait until the distribution pointer is
                // written.
                Distribution1D *dist = ComputeDistribution(pi);
                entry.distribution.store(dist, std::memory_order_release);
                ReportValue(nProbesPerLookup, nProbes);
                return dist;
            }
        }
    }
}

Distribution1D *
SpatialLightDistribution::ComputeDistribution(Point3i pi) const {
    ProfilePhase _(Prof::LightDistribCreation);
    ++nCreated;
    ++nDistributions;

    // Compute the world-space bounding box of the voxel corresponding to
    // |pi|.
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

    // Compute a sampling distribution from the accumulated contributions.
    return new Distribution1D(&lightContrib[0], int(lightContrib.size()));
}

}  // namespace pbrt
