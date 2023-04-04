
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_FILM_H
#define PBRT_CORE_FILM_H

#define filterTableWidth 16

// core/film.h*
#include "pbrt.h"
#include "geometry.h"
#include "spectrum.h"
#include "filter.h"
#include "stats.h"
#include "parallel.h"

namespace pbrt {

// FilmTilePixel Declarations
struct FilmTilePixel {
    Spectrum contribSum = 0.f;
    Float filterWeightSum = 0.f;
    FilmTilePixel() {}
    FilmTilePixel(Spectrum&& contrib, Float weight_sum): contribSum(contrib), filterWeightSum(weight_sum) {}
};

/**
 * Note that, for simplicity, transient component is directly embedded into the film
 * I don't want to distinguish TransientFilm and film currently, since the lack of time and efficiency
 * is really a pain in the ass.
 */

// Film Declarations
class Film {
  public:
    // Film Public Methods
    Film(const Point2i &resolution, const Bounds2f &cropWindow,
         std::unique_ptr<Filter> filter, Float diagonal,
         const std::string &filename, Float scale,
         Float maxSampleLuminance = Infinity, Float min_t = 0., 
         Float max_t = 20., Float interval = 0.0625, int sample_cnt = 256);
    Bounds2i GetSampleBounds() const;
    Bounds2f GetPhysicalExtent() const;
    std::unique_ptr<FilmTile> GetFilmTile(const Bounds2i &sampleBounds);
    void MergeFilmTile(std::unique_ptr<FilmTile> tile);
    void SetImage(const Spectrum *img) const;
    void AddSplat(const Point2f &p, Spectrum v, Float cur_time = -1.0);
    void WriteImage(Float splatScale = 1);
    void WriteTransient();
    void Clear();

    // Film Public Data
    const Point2i fullResolution;
    const Float diagonal;
    std::unique_ptr<Filter> filter;
    const std::string filename;
    Bounds2i croppedPixelBounds;

    const Float min_time;
    const Float max_time;
    const Float interval;
    int sample_cnt;

  private:
    // Film Private Data
    struct Pixel {
        Pixel() { xyz[0] = xyz[1] = xyz[2] = filterWeightSum = 0; }
        Float xyz[3];
        Float filterWeightSum;
        AtomicFloat splatXYZ[3];
        Float pad;
    };

    struct TransientPixel {
        AtomicFloat xyz[3];
        AtomicFloat weight_sum;

        void export_xyz(Float* const arr) const {
            for (int i = 0; i < 3; i++) arr[i] = xyz[i];
        }
    };

    // We might need an extra field, which is called time bins, time bins should contain two fields
    // time_bin: h, w, weight_sum (h*w*sample_cnt)
    std::unique_ptr<TransientPixel[]> time_bins;

    std::unique_ptr<Pixel[]> pixels;
    Float filterTable[filterTableWidth * filterTableWidth];
    std::mutex mutex;
    const Float scale;
    const Float maxSampleLuminance;

    // Film Private Methods
    Pixel &GetPixel(const Point2i &p) {
        CHECK(InsideExclusive(p, croppedPixelBounds));
        int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
        int offset = (p.x - croppedPixelBounds.pMin.x) +
                     (p.y - croppedPixelBounds.pMin.y) * width;
        return pixels[offset];
    }

    /** Using spatial and temporal index to locate a time bin and return its reference */
    TransientPixel& IndexTimeBin(const Point2i &p, int index) {
        CHECK(InsideExclusive(p, croppedPixelBounds));
        int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
        int offset = (p.x - croppedPixelBounds.pMin.x) +
                     (p.y - croppedPixelBounds.pMin.y) * width;
        // The temporal bound check should be done out side of this function
        int temporal_off = offset * sample_cnt + index;
        return time_bins[temporal_off];
    }

    TransientPixel& GetTimeBin(const Point2i &p, Float cur_time) {
        int index = int(std::floor((cur_time - min_time) / interval));
        return IndexTimeBin(p, index);
    }

    TransientPixel* transient_ptr(const Point2i &p) {
        CHECK(InsideExclusive(p, croppedPixelBounds));
        int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
        int offset = (p.x - croppedPixelBounds.pMin.x) +
                     (p.y - croppedPixelBounds.pMin.y) * width;
        return &time_bins.get()[offset * sample_cnt];
    }
};

// This might be the most difficult part
class FilmTile {
  using LocalTransients = std::unique_ptr<FilmTilePixel []>;
  public:
    // FilmTile Public Methods
    FilmTile(const Bounds2i &pixelBounds, const Vector2f &filterRadius,
             const Float *filterTable, int filterTableSize, Float maxSampleLuminance, 
             Float min_t = 0., Float max_t = 20., Float interval = 0.0625, int sample_cnt = 256
        ): pixelBounds(pixelBounds),
          filterRadius(filterRadius),
          invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
          filterTable(filterTable),
          filterTableSize(filterTableSize),
          maxSampleLuminance(maxSampleLuminance), min_time(min_t), max_time(max_t),
          interval(interval), sample_cnt(sample_cnt) 
        {
        pixels = std::vector<FilmTilePixel>(std::max(0, pixelBounds.Area()));
        if (sample_cnt > 0) {
            transients.resize(std::max(0, pixelBounds.Area()));
            for (std::vector<FilmTilePixel>& transient: transients)       // initialize with empty ones
                transient.resize(sample_cnt, FilmTilePixel());
        }
    }

    void AddSample(const Point2f &pFilm, Spectrum L,
                   Float sampleWeight = 1., LocalTransients lts = nullptr) {
        // Note that local storage have size: sample_cnt (time is converted to index beforehand)
        // add sample in the transient bins
        ProfilePhase _(Prof::AddFilmSample);
        if (L.y() > maxSampleLuminance)
            L *= maxSampleLuminance / L.y();
        // Compute sample's raster bounds, this has something to do with image filtering
        Point2f pFilmDiscrete = pFilm - Vector2f(0.5f, 0.5f);
        Point2i p0 = (Point2i)Ceil(pFilmDiscrete - filterRadius);
        Point2i p1 =
            (Point2i)Floor(pFilmDiscrete + filterRadius) + Point2i(1, 1);
        // Find all the pixels that (1) in the filter area and (2) in the tile bound 
        p0 = Max(p0, pixelBounds.pMin);
        p1 = Min(p1, pixelBounds.pMax);

        // Loop over filter support and add sample to pixel arrays

        // Precompute $x$ and $y$ filter table offsets
        int *ifx = ALLOCA(int, p1.x - p0.x);
        for (int x = p0.x; x < p1.x; ++x) {
            Float fx = std::abs((x - pFilmDiscrete.x) * invFilterRadius.x *
                                filterTableSize);
            ifx[x - p0.x] = std::min((int)std::floor(fx), filterTableSize - 1);
        }
        int *ify = ALLOCA(int, p1.y - p0.y);
        for (int y = p0.y; y < p1.y; ++y) {
            Float fy = std::abs((y - pFilmDiscrete.y) * invFilterRadius.y *
                                filterTableSize);
            ify[y - p0.y] = std::min((int)std::floor(fy), filterTableSize - 1);
        }
        // traverse all the valid pixel positions
        for (int y = p0.y; y < p1.y; ++y) {
            for (int x = p0.x; x < p1.x; ++x) {
                // Evaluate filter value at $(x,y)$ pixel
                int offset = ify[y - p0.y] * filterTableSize + ifx[x - p0.x];
                Float filterWeight = filterTable[offset];

                // Update pixel values with filtered sample contribution
                FilmTilePixel &pixel = GetPixel(Point2i(x, y));
                Float weight = sampleWeight * filterWeight;
                pixel.contribSum += L * weight;
                pixel.filterWeightSum += filterWeight;
                if (sample_cnt > 0 && lts) {          // local transient storage exists and we are actually using transient rendering
                    std::vector<FilmTilePixel>& dump = GetTransient(Point2i(x, y));
                    std::transform(dump.begin(), dump.end(), lts.get(), dump.begin(), 
                        [weight, filterWeight](const FilmTilePixel& p1, const FilmTilePixel& p2) {
                        return FilmTilePixel(p1.contribSum + p2.contribSum * weight, p1.filterWeightSum + filterWeight * p2.filterWeightSum);
                    });
                }
            }
        }
    }

    FilmTilePixel &GetPixel(const Point2i &p) {
        CHECK(InsideExclusive(p, pixelBounds));
        int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
        int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
        return pixels[offset];
    }

    const FilmTilePixel &GetPixel(const Point2i &p) const {
        CHECK(InsideExclusive(p, pixelBounds));
        int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
        int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
        return pixels[offset];
    }

    /** Since if const-ness is only distinguished by its callee, we can't call IndexTransient directly */
    std::vector<FilmTilePixel>& GetTransient(const Point2i &p) {
        CHECK(InsideExclusive(p, pixelBounds));
        int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
        int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
        return transients[offset];
    }

    const std::vector<FilmTilePixel>& GetTransient(const Point2i &p) const {
        CHECK(InsideExclusive(p, pixelBounds));
        int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
        int offset = (p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
        return transients[offset];
    }

    Bounds2i GetPixelBounds() const { return pixelBounds; }

  private:
    // FilmTile Private Data
    const int sample_cnt;
    const Float max_time;
    const Float min_time;
    const Float interval;

    const Bounds2i pixelBounds;
    const Vector2f filterRadius, invFilterRadius;
    const Float *filterTable;
    const int filterTableSize;

    std::vector<FilmTilePixel> pixels;
    std::vector<std::vector<FilmTilePixel>> transients;
    
    const Float maxSampleLuminance;
    friend class Film;
};

Film *CreateFilm(const ParamSet &params, std::unique_ptr<Filter> filter);

}  // namespace pbrt

#endif  // PBRT_CORE_FILM_H
