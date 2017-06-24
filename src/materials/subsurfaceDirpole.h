
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

#ifndef PBRT_MATERIALS_SUBSURFACE_DIRPOLE_H
#define PBRT_MATERIALS_SUBSURFACE_DIRPOLE_H

// materials/subsurfaceDirpole.h*
#include "pbrt.h"
#include "material.h"
#include "reflection.h"
#include "bssrdf.h"

namespace pbrt {

// SubsurfaceDirpoleMaterial Declarations
class SubsurfaceDirpoleMaterial : public Material {
  public:
    // SubsurfaceDirpoleMaterial Public Methods
    SubsurfaceDirpoleMaterial(Float scale,
                       const std::shared_ptr<Texture<Spectrum>> &Kr,
                       const std::shared_ptr<Texture<Spectrum>> &Kt,
                       const std::shared_ptr<Texture<Spectrum>> &sigma_a,
                       const std::shared_ptr<Texture<Spectrum>> &sigma_s,
                       Float g, Float eta,
                       const std::shared_ptr<Texture<Float>> &uRoughness,
                       const std::shared_ptr<Texture<Float>> &vRoughness,
                       const std::shared_ptr<Texture<Float>> &bumpMap,
                       bool remapRoughness)
        : scale(scale),
          Kr(Kr),
          Kt(Kt),
          sigma_a(sigma_a),
          sigma_s(sigma_s),
          uRoughness(uRoughness),
          vRoughness(vRoughness),
          bumpMap(bumpMap),
          eta(eta),
          g(g),
          remapRoughness(remapRoughness),
          table(100, 64) {
        ComputeDirpoleBSSRDF(g, eta, &table);
    }
    void ComputeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                    TransportMode mode,
                                    bool allowMultipleLobes) const;

  private:
    // SubsurfaceDirpoleMaterial Private Data
    const Float scale;
    std::shared_ptr<Texture<Spectrum>> Kr, Kt, sigma_a, sigma_s;
    std::shared_ptr<Texture<Float>> uRoughness, vRoughness;
    std::shared_ptr<Texture<Float>> bumpMap;
    const Float eta;
    const Float g;
    const bool remapRoughness;
    BSSRDFTable table;
};

SubsurfaceDirpoleMaterial *CreateSubsurfaceDirpoleMaterial(
        const TextureParams &mp);

}  // namespace pbrt

#endif  // PBRT_MATERIALS_SUBSURFACE_DIRPOLE_H
