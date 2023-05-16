
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


// core/film.cpp*
#include <memory>
#include <iomanip>
#include <experimental/filesystem>

#include "film.h"
#include "paramset.h"
#include "imageio.h"
#include "stats.h"

#define AREA_THRESHOLD 262144 

// TODO: the whole file (and some related APIs, like creating film and passing parameters) should be modified

namespace pbrt {

STAT_MEMORY_COUNTER("Memory/Film pixels", filmPixelMemory);

// Film Method Definitions
Film::Film(const Point2i &resolution, const Bounds2f &cropWindow,
           std::unique_ptr<Filter> filt, Float diagonal,
           const std::string &filename, Float scale, Float maxSampleLuminance, 
           Float min_t, Float max_t, Float interval, int sample_cnt
    ): fullResolution(resolution),
      diagonal(diagonal * .001),
      filter(std::move(filt)),
      filename(filename),
      scale(scale),
      maxSampleLuminance(maxSampleLuminance),
      min_time(min_t), max_time(max_t), interval(interval), sample_cnt(sample_cnt)
     {
    // Compute film image bounds
    croppedPixelBounds =
        Bounds2i(Point2i(std::ceil(fullResolution.x * cropWindow.pMin.x),
                         std::ceil(fullResolution.y * cropWindow.pMin.y)),
                 Point2i(std::ceil(fullResolution.x * cropWindow.pMax.x),
                         std::ceil(fullResolution.y * cropWindow.pMax.y)));
    LOG(INFO) << "Created film with full resolution " << resolution <<
        ". Crop window of " << cropWindow << " -> croppedPixelBounds " <<
        croppedPixelBounds;

    // Allocate film image storage
    int area = croppedPixelBounds.Area();
    pixels = std::make_unique<Pixel[]>(area);

    // Allocate transient film image storage
    if (area <= AREA_THRESHOLD && sample_cnt > 0) {     // maximum image area allocated, bigger than this figure might introduce memory overhead that is too high
        int64_t memory2use = area * sizeof(TransientPixel) * 2 * sample_cnt;     // estimated, since FilmTile might double the consumption
        filmPixelMemory += memory2use;
        time_bins = std::make_unique<TransientPixel[]>(area * sample_cnt);
        printf("Transient blocks will be allocated. Film area = %d, estimated memory: %f MB\n", area, float(memory2use) / (1024 * 1024));
        printf("Transient params: min_t = %.5f, max_t = %.5f, interval = %.5f, sample_cnt = %d\n", min_time, max_time, interval, sample_cnt);
    } else {
        sample_cnt = 0; // area greater than threshold is not allowed
    }

    filmPixelMemory += area * sizeof(Pixel);

    // Precompute filter weight table
    int offset = 0;
    for (int y = 0; y < filterTableWidth; ++y) {
        for (int x = 0; x < filterTableWidth; ++x, ++offset) {
            Point2f p;
            p.x = (x + 0.5f) * filter->radius.x / filterTableWidth;
            p.y = (y + 0.5f) * filter->radius.y / filterTableWidth;
            filterTable[offset] = filter->Evaluate(p);
        }
    }
}

Bounds2i Film::GetSampleBounds() const {
    Bounds2f floatBounds(Floor(Point2f(croppedPixelBounds.pMin) +
                               Vector2f(0.5f, 0.5f) - filter->radius),
                         Ceil(Point2f(croppedPixelBounds.pMax) -
                              Vector2f(0.5f, 0.5f) + filter->radius));
    return (Bounds2i)floatBounds;
}

Bounds2f Film::GetPhysicalExtent() const {
    Float aspect = (Float)fullResolution.y / (Float)fullResolution.x;
    Float x = std::sqrt(diagonal * diagonal / (1 + aspect * aspect));
    Float y = aspect * x;
    return Bounds2f(Point2f(-x / 2, -y / 2), Point2f(x / 2, y / 2));
}

std::unique_ptr<FilmTile> Film::GetFilmTile(const Bounds2i &sampleBounds) {
    // Bound image pixels that samples in _sampleBounds_ contribute to
    Vector2f halfPixel = Vector2f(0.5f, 0.5f);
    Bounds2f floatBounds = (Bounds2f)sampleBounds;
    Point2i p0 = (Point2i)Ceil(floatBounds.pMin - halfPixel - filter->radius);
    Point2i p1 = (Point2i)Floor(floatBounds.pMax - halfPixel + filter->radius) +
                 Point2i(1, 1);
    Bounds2i tilePixelBounds = Intersect(Bounds2i(p0, p1), croppedPixelBounds);
    return std::make_unique<FilmTile>(
        tilePixelBounds, filter->radius, filterTable, filterTableWidth,
        maxSampleLuminance, min_time, max_time, interval, sample_cnt);
}

void Film::Clear() {
    for (Point2i p : croppedPixelBounds) {
        Pixel &pixel = GetPixel(p);
        for (int c = 0; c < 3; ++c)
            pixel.splatXYZ[c] = pixel.xyz[c] = 0;
        pixel.filterWeightSum = 0;
    }
    if (time_bins) {
        for (Point2i p : croppedPixelBounds) {
            for (int time_i = 0; time_i < sample_cnt; time_i++) {
                TransientPixel &pixel = IndexTimeBin(p, time_i);
                for (int c = 0; c < 3; ++c) 
                    pixel.xyz[c] = 0;
                pixel.weight_sum = 0;
            }
        }
    }
}

void Film::MergeFilmTile(std::unique_ptr<FilmTile> tile) {
    // TODO: transient film tile merge
    ProfilePhase p(Prof::MergeFilmTile);
    VLOG(1) << "Merging film tile " << tile->pixelBounds;
    std::lock_guard<std::mutex> lock(mutex);
    for (Point2i pixel : tile->GetPixelBounds()) {
        // Merge _pixel_ into _Film::pixels_
        const FilmTilePixel &tilePixel = tile->GetPixel(pixel);
        Pixel &mergePixel = GetPixel(pixel);
        Float xyz[3];
        tilePixel.contribSum.ToXYZ(xyz);
        for (int i = 0; i < 3; ++i) mergePixel.xyz[i] += xyz[i];
        mergePixel.filterWeightSum += tilePixel.filterWeightSum;
        if (time_bins && tile->transients.empty() == false) {
            TransientPixel* const t_ptr = transient_ptr(pixel);
            const std::vector<FilmTilePixel>& transient = tile->GetTransient(pixel);
            for (int i = 0; i < sample_cnt; i++) {
                const FilmTilePixel& p1 = transient[i];
                TransientPixel& p2 = t_ptr[i];
                p1.contribSum.ToXYZ(xyz);
                for (int chid = 0; chid < 3; ++chid) p2.xyz[chid].Add(xyz[chid]);
                p2.weight_sum.Add(p1.filterWeightSum);
            }
        }
    }
}

void Film::SetImage(const Spectrum *img) const {
    /** This function is not used in BDPT, therefore NOT modified */
    int nPixels = croppedPixelBounds.Area();
    for (int i = 0; i < nPixels; ++i) {
        Pixel &p = pixels[i];
        img[i].ToXYZ(p.xyz);
        p.filterWeightSum = 1;
        p.splatXYZ[0] = p.splatXYZ[1] = p.splatXYZ[2] = 0;
    }
}

void Film::AddSplat(const Point2f &p, Spectrum v, Float cur_time) {
    ProfilePhase pp(Prof::SplatFilm);

    if (v.HasNaNs()) {
        LOG(ERROR) << StringPrintf("Ignoring splatted spectrum with NaN values "
                                   "at (%f, %f)", p.x, p.y);
        return;
    } else if (v.y() < 0.) {
        LOG(ERROR) << StringPrintf("Ignoring splatted spectrum with negative "
                                   "luminance %f at (%f, %f)", v.y(), p.x, p.y);
        return;
    } else if (std::isinf(v.y())) {
        LOG(ERROR) << StringPrintf("Ignoring splatted spectrum with infinite "
                                   "luminance at (%f, %f)", p.x, p.y);
        return;
    }

    Point2i pi = Point2i(Floor(p));
    if (!InsideExclusive(pi, croppedPixelBounds)) return;
    if (v.y() > maxSampleLuminance)
        v *= maxSampleLuminance / v.y();
    Float xyz[3];
    v.ToXYZ(xyz);
    Pixel &pixel = GetPixel(pi);
    // AddSplat is Atomic op, so we also need to use this operation
    for (int i = 0; i < 3; ++i) pixel.splatXYZ[i].Add(xyz[i]);
    if (time_bins && cur_time > min_time && cur_time < max_time) {
        TransientPixel& tpix = GetTimeBin(pi, cur_time);
        for (int i = 0; i < 3; ++i) tpix.xyz[i].Add(xyz[i]);
        // Qianyue He: When doing add splat op, we do not consider image filtering, therefore 1.0 is used as weight
        tpix.weight_sum.Add(1.0);
    }
}

void Film::WriteImage(Float splatScale) {
    // Convert image to RGB and compute final pixel values
    LOG(INFO) <<
        "Converting image to RGB and computing final weighted pixel values";
    std::string folder_path = filename.substr(0, filename.find_last_of("/") + 1);
    namespace fs = std::experimental::filesystem;
    if (time_bins && !fs::exists(folder_path)) {
        fs::create_directories(folder_path);
        printf("Path <%s> does not exist, creating folder...\n", folder_path.c_str());
    }
    auto rgb = std::make_unique<Float[]>(3 * croppedPixelBounds.Area());
    int offset = 0;
    for (Point2i p : croppedPixelBounds) {
        // Convert pixel XYZ color to RGB
        Pixel &pixel = GetPixel(p);
        XYZToRGB(pixel.xyz, &rgb[3 * offset]);

        // Normalize pixel with weight sum
        Float filterWeightSum = pixel.filterWeightSum;
        if (filterWeightSum != 0) {
            Float invWt = (Float)1 / filterWeightSum;
            rgb[3 * offset] = std::max((Float)0, rgb[3 * offset] * invWt);
            rgb[3 * offset + 1] =
                std::max((Float)0, rgb[3 * offset + 1] * invWt);
            rgb[3 * offset + 2] =
                std::max((Float)0, rgb[3 * offset + 2] * invWt);
        }

        // Add splat value at pixel
        Float splatRGB[3];
        Float splatXYZ[3] = {pixel.splatXYZ[0], pixel.splatXYZ[1],
                             pixel.splatXYZ[2]};
        XYZToRGB(splatXYZ, splatRGB);

        // TODO: splatRGB is not scaled by how many splat RGB values are added, but SPP
        // This is somewhat different from my calculation method: one splat, filterSum add 1.0
        // For transient samples, this might be true. However, this should be noted and tested.
        rgb[3 * offset] += splatScale * splatRGB[0];
        rgb[3 * offset + 1] += splatScale * splatRGB[1];
        rgb[3 * offset + 2] += splatScale * splatRGB[2];

        // Scale pixel value by _scale_
        rgb[3 * offset] *= scale;
        rgb[3 * offset + 1] *= scale;
        rgb[3 * offset + 2] *= scale;
        ++offset;
    }

    // Write RGB image
    LOG(INFO) << "Writing image " << filename << " with bounds " <<
        croppedPixelBounds;
    pbrt::WriteImage(filename, &rgb[0], croppedPixelBounds, fullResolution);
}

void Film::WriteTransient() {
    // Convert image to RGB and compute final pixel values
    
    if (time_bins) {            // transient images are valid
        printf("Exporting transient profiles...\n");
        LOG(INFO) << "Converting transient to RGB and computing final weighted transient profile";
        std::string prefix = filename.substr(0, filename.find_last_of("."));
        // exporting transient might take time, therefore I opt for multi-threading
        for (size_t ti = 0; ti < sample_cnt; ti++) {
            
            std::ostringstream str;
            str << std::setw(4) << std::setfill('0') << ti;
            std::string output_name = prefix + "_" + str.str() + ".exr";
            auto rgb = std::make_unique<Float[]>(3 * croppedPixelBounds.Area());
            int offset = 0;
            for (Point2i p : croppedPixelBounds) {
                // Convert pixel XYZ color to RGB
                const TransientPixel &pixel = IndexTimeBin(p, ti);

                Float xyz[3];
                pixel.export_xyz(xyz);
                XYZToRGB(xyz, &rgb[3 * offset]);

                // Normalize pixel with weight sum
                Float filterWeightSum = pixel.weight_sum;
                if (filterWeightSum != 0) {
                    Float invWt = (Float)1 / filterWeightSum;
                    rgb[3 * offset] = std::max((Float)0, rgb[3 * offset] * invWt);
                    rgb[3 * offset + 1] =
                        std::max((Float)0, rgb[3 * offset + 1] * invWt);
                    rgb[3 * offset + 2] =
                        std::max((Float)0, rgb[3 * offset + 2] * invWt);
                }

                // Scale pixel value by _scale_
                rgb[3 * offset] *= scale;
                rgb[3 * offset + 1] *= scale;
                rgb[3 * offset + 2] *= scale;
                ++offset;
            }
            pbrt::WriteImage(output_name, &rgb[0], croppedPixelBounds, fullResolution);
        }
    } else {
        LOG(INFO) << "Time bin is empty, not exporting transient profile.";
    }
}

Film *CreateFilm(const ParamSet &params, std::unique_ptr<Filter> filter) {
    std::string filename;
    if (PbrtOptions.imageFile != "") {
        filename = PbrtOptions.imageFile;
        std::string paramsFilename = params.FindOneString("filename", "");
        if (paramsFilename != "")
            Warning(
                "Output filename supplied on command line, \"%s\" is overriding "
                "filename provided in scene description file, \"%s\".",
                PbrtOptions.imageFile.c_str(), paramsFilename.c_str());
    } else
        filename = params.FindOneString("filename", "pbrt.exr");

    int xres = params.FindOneInt("xresolution", 1280);
    int yres = params.FindOneInt("yresolution", 720);
    if (PbrtOptions.quickRender) xres = std::max(1, xres / 4);
    if (PbrtOptions.quickRender) yres = std::max(1, yres / 4);
    Bounds2f crop;
    int cwi;
    const Float *cr = params.FindFloat("cropwindow", &cwi);
    if (cr && cwi == 4) {
        crop.pMin.x = Clamp(std::min(cr[0], cr[1]), 0.f, 1.f);
        crop.pMax.x = Clamp(std::max(cr[0], cr[1]), 0.f, 1.f);
        crop.pMin.y = Clamp(std::min(cr[2], cr[3]), 0.f, 1.f);
        crop.pMax.y = Clamp(std::max(cr[2], cr[3]), 0.f, 1.f);
    } else if (cr)
        Error("%d values supplied for \"cropwindow\". Expected 4.", cwi);
    else
        crop = Bounds2f(Point2f(Clamp(PbrtOptions.cropWindow[0][0], 0, 1),
                                Clamp(PbrtOptions.cropWindow[1][0], 0, 1)),
                        Point2f(Clamp(PbrtOptions.cropWindow[0][1], 0, 1),
                                Clamp(PbrtOptions.cropWindow[1][1], 0, 1)));

    Float scale = params.FindOneFloat("scale", 1.);
    Float diagonal = params.FindOneFloat("diagonal", 35.);
    Float maxSampleLuminance = params.FindOneFloat("maxsampleluminance",
                                                   Infinity);
    Float min_time = params.FindOneFloat("tmin_time", 0.0);
    Float interval = params.FindOneFloat("t_interval", 0.0625);
    int sample_cnt = params.FindOneInt("t_samplecnt", 0);
    Float max_time = min_time + Float(sample_cnt) * interval;
    return new Film(Point2i(xres, yres), crop, std::move(filter), diagonal,
                    filename, scale, maxSampleLuminance, min_time, max_time, interval, sample_cnt);
}

}  // namespace pbrt
