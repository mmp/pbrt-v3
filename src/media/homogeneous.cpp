
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

// media/homogeneous.cpp*
#include "media/homogeneous.h"
#include "sampler.h"
#include "interaction.h"
#include "paramset.h"

// HomogeneousMedium Method Definitions
Spectrum HomogeneousMedium::Tr(const Ray &ray, Sampler &sampler) const {
    return Exp(-sigma_t * std::min(ray.tMax * ray.d.Length(), MaxFloat));
}

Spectrum HomogeneousMedium::Sample(const Ray &ray, Sampler &sampler,
                                   MemoryArena &arena,
                                   MediumInteraction *mi) const {
    Point2f u = sampler.Get2D();
    // Sample a channel and distance along the ray
    int channel =
        std::min((int)(u[0] * Spectrum::nSamples), Spectrum::nSamples - 1);
    Float t = std::min(
        -std::log(1 - u[1]) / (sigma_t[channel] * ray.d.Length()), ray.tMax);
    bool sampledMedium = t < ray.tMax;
    if (sampledMedium) {
        PhaseFunction *phase = ARENA_ALLOC(arena, HenyeyGreenstein)(g);
        *mi = MediumInteraction(ray(t), -ray.d, ray.time, this, phase);
    }

    // Compute the transmittance and sampling density
    Spectrum Tr = Exp(-sigma_t * std::min(t, MaxFloat) * ray.d.Length());
    Spectrum density = (sampledMedium ? sigma_t : Spectrum(1.f)) * Tr;
    Float pdf = 0.f;
    for (int i = 0; i < Spectrum::nSamples; ++i) pdf += density[i];
    pdf *= 1 / (Float)Spectrum::nSamples;
    return Tr * (sampledMedium ? sigma_s : Spectrum(1.f)) / pdf;
}

Float HomogeneousMedium::Pdf(const Ray &ray, const Interaction &it) const {
    Spectrum density =
        Exp(-sigma_t * (it.p - ray.o).Length()) *
        (it.IsMediumInteraction() ? (sigma_t * ray.d.Length()) : Spectrum(1.0));
    Float pdf = 0;
    for (int i = 0; i < Spectrum::nSamples; ++i) pdf += density[i];
    return pdf / Spectrum::nSamples;
}
