
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


// textures/imagemap.cpp*
#include "textures/imagemap.h"
#include "imageio.h"
#include "stats.h"

namespace pbrt {

// ImageTexture Method Definitions
template <typename T>
ImageTexture<T>::ImageTexture(
    std::unique_ptr<TextureMapping2D> mapping, const std::string &filename,
    const std::string &filter, Float maxAniso, WrapMode wrapMode, Float scale,
    bool gamma)
    : mapping(std::move(mapping)), scale(scale) {
    mipmap =
        GetTexture(filename, filter, maxAniso, wrapMode, gamma);
}

template <typename T>
MIPMap *ImageTexture<T>::GetTexture(
    const std::string &filename, const std::string &filter, Float maxAniso,
    WrapMode wrap, bool gamma) {
    // Return _MIPMap_ from texture cache if present
    TexInfo texInfo(filename, filter, maxAniso, wrap, gamma);
    if (textures.find(texInfo) != textures.end())
        return textures[texInfo].get();

    // Create _MIPMap_ for _filename_
    ProfilePhase _(Prof::TextureLoading);
    MIPMapFilterOptions options;
    options.maxAnisotropy = maxAniso;
    if (!ParseFilter(filter, &options.filter))
        Warning("%s: filter function unknown", filter.c_str());
    std::unique_ptr<MIPMap> mipmap =
        MIPMap::CreateFromFile(filename, options, wrap, gamma);
    if (mipmap) {
        textures[texInfo] = std::move(mipmap);
        return textures[texInfo].get();
    } else
        return nullptr;
}

template <typename T>
std::map<TexInfo, std::unique_ptr<MIPMap>> ImageTexture<T>::textures;

ImageTexture<Float> *CreateImageFloatTexture(const Transform &tex2world,
                                             const TextureParams &tp) {
    // Initialize 2D texture mapping _map_ from _tp_
    std::unique_ptr<TextureMapping2D> map;
    std::string type = tp.FindString("mapping", "uv");
    if (type == "uv") {
        Float su = tp.FindFloat("uscale", 1.);
        Float sv = tp.FindFloat("vscale", 1.);
        Float du = tp.FindFloat("udelta", 0.);
        Float dv = tp.FindFloat("vdelta", 0.);
        map.reset(new UVMapping2D(su, sv, du, dv));
    } else if (type == "spherical")
        map.reset(new SphericalMapping2D(Inverse(tex2world)));
    else if (type == "cylindrical")
        map.reset(new CylindricalMapping2D(Inverse(tex2world)));
    else if (type == "planar")
        map.reset(new PlanarMapping2D(tp.FindVector3f("v1", Vector3f(1, 0, 0)),
                                      tp.FindVector3f("v2", Vector3f(0, 1, 0)),
                                      tp.FindFloat("udelta", 0.f),
                                      tp.FindFloat("vdelta", 0.f)));
    else {
        Error("2D texture mapping \"%s\" unknown", type.c_str());
        map.reset(new UVMapping2D);
    }

    // Initialize _ImageTexture_ parameters
    Float maxAniso = tp.FindFloat("maxanisotropy", 8.f);
    std::string filter = tp.FindString("filter", "bilinear");
    std::string wrap = tp.FindString("wrap", "repeat");
    WrapMode wrapMode;
    std::string wrapString = tp.FindString("wrap", "repeat");
    if (!ParseWrapMode(wrapString.c_str(), &wrapMode))
        Warning("%s: wrap mode unknown", wrapString.c_str());
    Float scale = tp.FindFloat("scale", 1.f);
    std::string filename = tp.FindFilename("filename");
    bool gamma = tp.FindBool("gamma", HasExtension(filename, ".tga") ||
                                          HasExtension(filename, ".png"));
    return new ImageTexture<Float>(std::move(map), filename, filter,
                                   maxAniso, wrapMode, scale, gamma);
}

ImageTexture<Spectrum> *CreateImageSpectrumTexture(
    const Transform &tex2world, const TextureParams &tp) {
    // Initialize 2D texture mapping _map_ from _tp_
    std::unique_ptr<TextureMapping2D> map;
    std::string type = tp.FindString("mapping", "uv");
    if (type == "uv") {
        Float su = tp.FindFloat("uscale", 1.);
        Float sv = tp.FindFloat("vscale", 1.);
        Float du = tp.FindFloat("udelta", 0.);
        Float dv = tp.FindFloat("vdelta", 0.);
        map.reset(new UVMapping2D(su, sv, du, dv));
    } else if (type == "spherical")
        map.reset(new SphericalMapping2D(Inverse(tex2world)));
    else if (type == "cylindrical")
        map.reset(new CylindricalMapping2D(Inverse(tex2world)));
    else if (type == "planar")
        map.reset(new PlanarMapping2D(tp.FindVector3f("v1", Vector3f(1, 0, 0)),
                                      tp.FindVector3f("v2", Vector3f(0, 1, 0)),
                                      tp.FindFloat("udelta", 0.f),
                                      tp.FindFloat("vdelta", 0.f)));
    else {
        Error("2D texture mapping \"%s\" unknown", type.c_str());
        map.reset(new UVMapping2D);
    }

    // Initialize _ImageTexture_ parameters
    Float maxAniso = tp.FindFloat("maxanisotropy", 8.f);
    std::string filter = tp.FindString("filter", "bilinear");
    std::string wrap = tp.FindString("wrap", "repeat");
    WrapMode wrapMode;
    std::string wrapString = tp.FindString("wrap", "repeat");
    if (!ParseWrapMode(wrapString.c_str(), &wrapMode))
        Warning("%s: wrap mode unknown", wrapString.c_str());
    Float scale = tp.FindFloat("scale", 1.f);
    std::string filename = tp.FindFilename("filename");
    bool gamma = tp.FindBool("gamma", HasExtension(filename, ".tga") ||
                                          HasExtension(filename, ".png"));
    return new ImageTexture<Spectrum>(
        std::move(map), filename, filter, maxAniso, wrapMode, scale, gamma);
}

}  // namespace pbrt
