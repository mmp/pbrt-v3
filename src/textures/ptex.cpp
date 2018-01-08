
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

// textures/ptex.cpp*
#include "textures/ptex.h"

#include "error.h"
#include "interaction.h"
#include "paramset.h"
#include "stats.h"

#include <Ptexture.h>

namespace pbrt {

namespace {

// Reference count for the cache. Note: we assume that PtexTextures aren't
// being created/destroyed concurrently by multiple threads.
int nActiveTextures;
Ptex::PtexCache *cache;

STAT_COUNTER("Texture/Ptex lookups", nLookups);
STAT_COUNTER("Texture/Ptex files accessed", nFilesAccessed);
STAT_COUNTER("Texture/Ptex block reads", nBlockReads);
STAT_MEMORY_COUNTER("Memory/Ptex peak memory used", peakMemoryUsed);

struct : public PtexErrorHandler {
    void reportError(const char *error) override { Error("%s", error); }
} errorHandler;

}  // anonymous namespace

// PtexTexture Method Definitions
template <typename T>
PtexTexture<T>::PtexTexture(const std::string &filename, Float gamma)
    : filename(filename), gamma(gamma) {
    if (!cache) {
        CHECK_EQ(nActiveTextures, 0);
        int maxFiles = 100;
        size_t maxMem = 1ull << 32;  // 4GB
        bool premultiply = true;

        cache = Ptex::PtexCache::create(maxFiles, maxMem, premultiply, nullptr,
                                        &errorHandler);
        // TODO? cache->setSearchPath(...);
    }
    ++nActiveTextures;

    // Issue an error if the texture doesn't exist or has an unsupported
    // number of channels.
    valid = false;
    Ptex::String error;
    Ptex::PtexTexture *texture = cache->get(filename.c_str(), error);
    if (!texture)
        Error("%s", error.c_str());
    else {
        if (texture->numChannels() != 1 && texture->numChannels() != 3)
            Error("%s: only one and three channel ptex textures are supported",
                  filename.c_str());
        else {
            valid = true;
            LOG(INFO) << filename << ": added ptex texture";
        }
        texture->release();
    }
}

template <typename T>
PtexTexture<T>::~PtexTexture() {
    if (--nActiveTextures == 0) {
        LOG(INFO) << "Releasing ptex cache";
        Ptex::PtexCache::Stats stats;
        cache->getStats(stats);
        nFilesAccessed += stats.filesAccessed;
        nBlockReads += stats.blockReads;
        peakMemoryUsed = stats.peakMemUsed;

        cache->release();
        cache = nullptr;
    }
}

template <typename T>
inline T fromResult(int nc, float *result) {
    return T::unimplemented;
}

template <>
inline Float fromResult<Float>(int nc, float *result) {
    if (nc == 1)
        return result[0];
    else
        return (result[0] + result[1] + result[2]) / 3;
}

template <>
inline Spectrum fromResult<Spectrum>(int nc, float *result) {
    if (nc == 1)
        return Spectrum(result[0]);
    else {
        Float rgb[3] = { result[0], result[1], result[2] };
        return Spectrum::FromRGB(rgb);
    }
}

template <typename T>
T PtexTexture<T>::Evaluate(const SurfaceInteraction &si) const {
    ProfilePhase _(Prof::TexFiltPtex);

    if (!valid) return T{};

    ++nLookups;
    Ptex::String error;
    Ptex::PtexTexture *texture = cache->get(filename.c_str(), error);
    CHECK_NOTNULL(texture);
    // TODO: make the filter an option?
    Ptex::PtexFilter::Options opts(Ptex::PtexFilter::FilterType::f_bspline);
    Ptex::PtexFilter *filter = Ptex::PtexFilter::getFilter(texture, opts);
    int nc = texture->numChannels();

    float result[3];
    int firstChan = 0;
    filter->eval(result, firstChan, nc, si.faceIndex, si.uv[0],
                 si.uv[1], si.dudx, si.dvdx, si.dudy, si.dvdy);
    filter->release();
    texture->release();

    if (gamma != 1)
        for (int i = 0; i < nc; ++i)
            if (result[i] >= 0 && result[i] <= 1)
                // FIXME: should use something more efficient here
                result[i] = std::pow(result[i], gamma);

    return fromResult<T>(nc, result);
}

PtexTexture<Float> *CreatePtexFloatTexture(const Transform &tex2world,
                                           const TextureParams &tp) {
    std::string filename = tp.FindFilename("filename");
    Float gamma = tp.FindFloat("gamma", 2.2);
    return new PtexTexture<Float>(filename, gamma);
}

PtexTexture<Spectrum> *CreatePtexSpectrumTexture(const Transform &tex2world,
                                                 const TextureParams &tp) {
    std::string filename = tp.FindFilename("filename");
    Float gamma = tp.FindFloat("gamma", 2.2);
    return new PtexTexture<Spectrum>(filename, gamma);
}

}  // namespace pbrt
