
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

// materials/kdsubsurface.cpp*
#include "materials/kdsubsurface.h"
#include "textures/constant.h"
#include "spectrum.h"
#include "texture.h"
#include "paramset.h"
#include "interaction.h"

// KdSubsurfaceMaterial Method Definitions
void KdSubsurfaceMaterial::ComputeScatteringFunctions(
    SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
    bool allowMultipleLobes) const {
    // Perform bump mapping with _bumpMap_, if present
    if (bumpMap) Bump(bumpMap, si);
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si, eta);
    Spectrum R = Kr->Evaluate(*si).Clamp();
    Spectrum T = Kt->Evaluate(*si).Clamp();

    if (allowMultipleLobes && (!R.IsBlack() || !T.IsBlack()))
        si->bsdf->Add(
            ARENA_ALLOC(arena, FresnelSpecular)(1., 1., 1.f, eta, mode));
    else {
        if (!R.IsBlack())
            si->bsdf->Add(ARENA_ALLOC(arena, SpecularReflection)(
                R, ARENA_ALLOC(arena, FresnelDielectric)(1.f, eta)));
        if (!T.IsBlack())
            si->bsdf->Add(
                ARENA_ALLOC(arena, SpecularTransmission)(T, 1.f, eta, mode));
    }

    Spectrum sig_t = scale * sigma_t->Evaluate(*si).Clamp();

    Spectrum kd = Kd->Evaluate(*si).Clamp();
    Spectrum sig_a, sig_s;
    SubsurfaceFromDiffuse(table, kd, sig_t, &sig_a, &sig_s);
    si->bssrdf = ARENA_ALLOC(arena, TabulatedBSSRDF)(*si, this, table, sig_a,
                                                     sig_s, eta, mode);
}

KdSubsurfaceMaterial *CreateKdSubsurfaceMaterial(const TextureParams &mp) {
    Float Kd[3] = {.5, .5, .5};
    std::shared_ptr<Texture<Spectrum>> kd =
        mp.GetSpectrumTexture("Kd", Spectrum::FromRGB(Kd));
    std::shared_ptr<Texture<Spectrum>> sigma_t =
        mp.GetSpectrumTexture("sigma_t", Spectrum::FromRGB(Kd));
    std::shared_ptr<Texture<Spectrum>> kr =
        mp.GetSpectrumTexture("Kr", Spectrum(1.f));
    std::shared_ptr<Texture<Spectrum>> kt =
        mp.GetSpectrumTexture("Kt", Spectrum(1.f));
    std::shared_ptr<Texture<Float>> bumpMap =
        mp.GetFloatTextureOrNull("bumpmap");
    Float eta = mp.FindFloat("eta", 1.33f);
    Float scale = mp.FindFloat("scale", 1.0f);
    Float g = mp.FindFloat("g", 0.0f);
    return new KdSubsurfaceMaterial(scale, kd, kr, kt, sigma_t, g, eta,
                                    bumpMap);
}
