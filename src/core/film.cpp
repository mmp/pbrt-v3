
/*
    pbrt source code is Copyright(c) 1998-2015
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

#include "stdafx.h"

// core/film.cpp*
#include "film.h"
#include "paramset.h"
#include "imageio.h"

// Film Method Definitions
Film::Film(const Point2i &resolution, const Bounds2f &cropWindow,
           Filter *filter, Float diagonal, const std::string &filename,
           Float scale, Float gamma)
    : fullResolution(resolution),
      diagonal(diagonal * .001),
      filter(filter),
      filename(filename),
      scale(scale),
      gamma(gamma) {
    // Compute film image bounds
    croppedPixelBounds =
        Bounds2i(Point2i(std::ceil(fullResolution.x * cropWindow.pMin.x),
                         std::ceil(fullResolution.y * cropWindow.pMin.y)),
                 Point2i(std::ceil(fullResolution.x * cropWindow.pMax.x),
                         std::ceil(fullResolution.y * cropWindow.pMax.y)));

    // Allocate film image storage
    pixels = std::unique_ptr<Pixel[]>(new Pixel[croppedPixelBounds.Area()]);

    // Precompute filter weight table
    int offset = 0;
    for (int y = 0; y < filterTableWidth; ++y) {
        for (int x = 0; x < filterTableWidth; ++x, ++offset) {
            Point2f p(x + 0.5f, y + 0.5f);
            p.x *= filter->radius.x / filterTableWidth;
            p.y *= filter->radius.y / filterTableWidth;
            filterTable[offset] = filter->Evaluate(p);
        }
    }
}

Bounds2i Film::GetSampleBounds() const {
    Bounds2f floatBounds(Floor(Point2f(croppedPixelBounds.pMin) +
                               Vector2f(0.5f, 0.5f) - filter->radius),
                         Ceil(Point2f(croppedPixelBounds.pMax) -
                              Vector2f(0.5f, 0.5f) + filter->radius));
#if defined(PBRT_IS_MSVC) && (__MWKM__)
	// VS2015_mwkm: vs2015 not smart :(
	return (Bounds2i)(Point2i(floatBounds.pMin), Point2i(floatBounds.pMax));
#else
	return (Bounds2i)floatBounds;
#endif
}

Point2f Film::GetPhysicalSize() const {
    Float aspect = (Float)fullResolution.y / (Float)fullResolution.x;
    Float x = std::sqrt(diagonal * diagonal / (1.f + aspect * aspect));
    Float y = aspect * x;
    return Point2f(x, y);
}

std::unique_ptr<FilmTile> Film::GetFilmTile(const Bounds2i &sampleBounds) {
    // Bound image pixels that samples in _SampleBounds_ contribute to
    Vector2f halfPixel = Vector2f(0.5f, 0.5f);
#if defined(PBRT_IS_MSVC) && (__MWKM__)
	// VS2015_mwkm: vs2015 not smart :(
	Bounds2f floatBounds = (Bounds2f)(Point2f(sampleBounds.pMin), Point2f(sampleBounds.pMax));
#else
	Bounds2f floatBounds = (Bounds2f)sampleBounds;
#endif

    Point2i p0 = (Point2i)Ceil(floatBounds.pMin - halfPixel - filter->radius);
    Point2i p1 = (Point2i)Floor(floatBounds.pMax - halfPixel + filter->radius) +
                 Point2i(1, 1);
    Bounds2i tilePixelBounds = Intersect(Bounds2i(p0, p1), croppedPixelBounds);
    return std::unique_ptr<FilmTile>(new FilmTile(
        tilePixelBounds, filter->radius, filterTable, filterTableWidth));
}

void Film::MergeFilmTile(std::unique_ptr<FilmTile> tile) {
    std::lock_guard<std::mutex> lock(mutex);
    for (Point2i pixel : tile->GetPixelBounds()) {
        // Merge _pixel_ into _Film::pixels_
        const FilmTilePixel &unnormalizedPixel = tile->GetPixel(pixel);
        Pixel &mergePixel = GetPixel(pixel);
        Float xyz[3];
        unnormalizedPixel.contribSum.ToXYZ(xyz);
        for (int i = 0; i < 3; ++i) mergePixel.xyz[i] += xyz[i];
        mergePixel.filterWeightSum += unnormalizedPixel.filterWeightSum;
    }
}

void Film::Splat(const Point2f &pFilm, const Spectrum &L) {
    if (L.HasNaNs()) {
        Warning("Film ignoring splatted spectrum with NaN values");
        return;
    }
    Point2i pi = (Point2i)Floor(pFilm);
    if (!InsideExclusive(pi, croppedPixelBounds)) return;
    Float xyz[3];
    L.ToXYZ(xyz);
    Pixel &pixel = GetPixel(pi);
#if 0
    // XXX rethink this -- it's killing performance for BDPT
    std::lock_guard<std::mutex> lock(mutex);
    for (int i = 0; i < 3; ++i)
        pixel.splatXYZ[i] += xyz[i];
#else
    /// Workaround for now (?) using atomic FP adds
    for (int i = 0; i < 3; ++i) AtomicAdd(&pixel.splatXYZ[i], xyz[i]);
#endif
}

void Film::SetImage(const Spectrum *img) const {
    int nPixels = croppedPixelBounds.Area();
    for (int i = 0; i < nPixels; ++i) {
        Pixel &p = pixels[i];
        img[i].ToXYZ(p.xyz);
        p.filterWeightSum = 1.f;
        p.splatXYZ[0] = p.splatXYZ[1] = p.splatXYZ[2] = 0.f;
    }
}

void Film::WriteImage(Float splatScale) {
    // Convert image to RGB and compute final pixel values
    std::vector<Float> rgb(3 * croppedPixelBounds.Area());
    int offset = 0;
    for (Point2i p : croppedPixelBounds) {
        // Convert pixel XYZ color to RGB
        Pixel &pixel = GetPixel(p);
        XYZToRGB(pixel.xyz, &rgb[3 * offset]);

        // Normalize pixel with weight sum
        Float filterWeightSum = pixel.filterWeightSum;
        if (filterWeightSum != 0.f) {
            Float invWt = (Float)1. / filterWeightSum;
            rgb[3 * offset] = std::max((Float)0., rgb[3 * offset] * invWt);
            rgb[3 * offset + 1] =
                std::max((Float)0., rgb[3 * offset + 1] * invWt);
            rgb[3 * offset + 2] =
                std::max((Float)0., rgb[3 * offset + 2] * invWt);
        }

        // Add splat value at pixel
        Float splatRGB[3];
        Float splatXYZ[3] = {pixel.splatXYZ[0], pixel.splatXYZ[1],
                             pixel.splatXYZ[2]};
        XYZToRGB(splatXYZ, splatRGB);
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
    ::WriteImage(filename, &rgb[0], croppedPixelBounds, fullResolution, gamma);
}

Film *CreateFilm(const ParamSet &params, Filter *filter) {
    // Intentionally use FindOneString() rather than FindOneFilename() here
    // so that the rendered image is left in the working directory, rather
    // than the directory the scene file lives in.
    std::string filename = params.FindOneString("filename", "");
    if (PbrtOptions.imageFile != "") {
        if (filename != "") {
            Warning(
                "Output filename supplied on command line, \"%s\", ignored "
                "due to filename provided in scene description file, \"%s\".",
                PbrtOptions.imageFile.c_str(), filename.c_str());
        } else
            filename = PbrtOptions.imageFile;
    }
    if (filename == "") filename = "pbrt.exr";

    int xres = params.FindOneInt("xresolution", 1280);
    int yres = params.FindOneInt("yresolution", 720);
    if (PbrtOptions.quickRender) xres = std::max(1, xres / 4);
    if (PbrtOptions.quickRender) yres = std::max(1, yres / 4);
    Bounds2f crop(Point2f(0, 0), Point2f(1, 1));
    int cwi;
    const Float *cr = params.FindFloat("cropwindow", &cwi);
    if (cr && cwi == 4) {
        crop.pMin.x = Clamp(std::min(cr[0], cr[1]), 0.f, 1.f);
        crop.pMax.x = Clamp(std::max(cr[0], cr[1]), 0.f, 1.f);
        crop.pMin.y = Clamp(std::min(cr[2], cr[3]), 0.f, 1.f);
        crop.pMax.y = Clamp(std::max(cr[2], cr[3]), 0.f, 1.f);
    } else if (cr)
        Error("%d values supplied for \"cropwindow\". Expected 4.", cwi);

    Float scale = params.FindOneFloat("scale", 1.);
    Float gamma = params.FindOneFloat("gamma", 2.2);
    Float diagonal = params.FindOneFloat("diagonal", 35.);
    return new Film(Point2i(xres, yres), crop, filter, diagonal, filename,
                    scale, gamma);
}
