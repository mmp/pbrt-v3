
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_MEDIA_HOMOGENEOUS_H
#define PBRT_MEDIA_HOMOGENEOUS_H
#include "stdafx.h"

// media/homogeneous.h*
#include "medium.h"

// HomogeneousMedium Declarations
class HomogeneousMedium : public Medium {
  public:
    // HomogeneousMedium Public Methods
    HomogeneousMedium(const Spectrum &sigma_a, const Spectrum &sigma_s, Float g)
        : sigma_a(sigma_a),
          sigma_s(sigma_s),
          sigma_t(sigma_s + sigma_a),
          g(g) {}
    Spectrum Tr(const Ray &ray, Sampler &sampler) const;
    Spectrum Sample(const Ray &ray, Sampler &sampler, MemoryArena &arena,
                    MediumInteraction *mi) const;

  private:
    // HomogeneousMedium Private Data
    const Spectrum sigma_a, sigma_s, sigma_t;
    const Float g;
};

#endif  // PBRT_MEDIA_HOMOGENEOUS_H
