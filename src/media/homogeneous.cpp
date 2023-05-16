
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


// media/homogeneous.cpp*
#include "media/homogeneous.h"
#include "sampler.h"
#include "interaction.h"
#include "paramset.h"
#include "stats.h"

namespace pbrt {

// HomogeneousMedium Method Definitions
Spectrum HomogeneousMedium::Tr(const Ray &ray, Sampler &sampler) const {
    ProfilePhase _(Prof::MediumTr);
    return Exp(-sigma_t * std::min(ray.tMax * ray.d.Length(), MaxFloat));
}

Spectrum HomogeneousMedium::Sample(const Ray &ray, Sampler &sampler,
                                   MemoryArena &arena, MediumInteraction *mi,
                                   const GuidedSamplingInfo* guide_info) const {
    ProfilePhase _(Prof::MediumSample);
    Spectrum scatter_t = sigma_t;
    if (guide_info != nullptr) {
      // If there is guiding info, we can rescale sigma_t for distance sampling
      Float w_z = Dot(ray.d / ray.d.Length(), guide_info->normal);
      scatter_t *= (1.f - w_z / guide_info->nu);
    }

    // Sample a channel and distance along the ray
    int channel = std::min((int)(sampler.Get1D() * Spectrum::nSamples),
                           Spectrum::nSamples - 1);
    Float dist = -std::log(1 - sampler.Get1D()) / scatter_t[channel];
    Float t = std::min(dist / ray.d.Length(), ray.tMax);
    bool sampledMedium = t < ray.tMax;
    if (sampledMedium)
        *mi = MediumInteraction(ray(t), -ray.d, ray.time + t, this,       // TODO: add t will suffice Qianyue He: account for travelling time
                                ARENA_ALLOC(arena, HenyeyGreenstein)(g));

    // Compute the transmittance and sampling density
    // Note that we can not modify the sigma_t here, since it's calculating contribution
    Float actual_d = std::min(t, MaxFloat) * ray.d.Length();
    Spectrum Tr = Exp(-sigma_t * actual_d);

    // Return weighting factor for scattering from homogeneous medium
    Spectrum density;
    if (guide_info == nullptr) {
      density = sampledMedium ? (sigma_t * Tr) : Tr;
    } else {
      // When guiding information is used, we should use scaled u_t to calculate all parts of pdf
      // Since pdf should be in accordance with sampling method, otherwise it introduces bias
      Spectrum Tr_pdf = Exp(-scatter_t * actual_d);
      density = sampledMedium ? (scatter_t * Tr_pdf) : Tr_pdf;
    }
    Float pdf = 0;
    
    for (int i = 0; i < Spectrum::nSamples; ++i) pdf += density[i];
    pdf *= 1 / (Float)Spectrum::nSamples;
    if (pdf == 0) {
        CHECK(Tr.IsBlack());
        pdf = 1;
    }
    return sampledMedium ? (Tr * sigma_s / pdf) : (Tr / pdf);
}

}  // namespace pbrt
