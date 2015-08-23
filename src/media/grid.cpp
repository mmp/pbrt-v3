
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

// media/grid.cpp*
#include "media/grid.h"
#include "paramset.h"
#include "sampler.h"
#include "interaction.h"

// GridDensityMedium Method Definitions
Float GridDensityMedium::Density(const Point3f &p) const {
    // Compute voxel coordinates and offsets for _p_
    Point3f pSamples(p.x * nx - .5f, p.y * ny - .5f, p.z * nz - .5f);
    Point3i pi = (Point3i)Floor(pSamples);
    Vector3f d = pSamples - (Point3f)pi;

    // Trilinearly interpolate density values to compute local density
    Float d00 = Lerp(d.x, D(pi), D(pi + Vector3i(1, 0, 0)));
    Float d10 = Lerp(d.x, D(pi + Vector3i(0, 1, 0)), D(pi + Vector3i(1, 1, 0)));
    Float d01 = Lerp(d.x, D(pi + Vector3i(0, 0, 1)), D(pi + Vector3i(1, 0, 1)));
    Float d11 = Lerp(d.x, D(pi + Vector3i(0, 1, 1)), D(pi + Vector3i(1, 1, 1)));
    Float d0 = Lerp(d.y, d00, d10);
    Float d1 = Lerp(d.y, d01, d11);
    return Lerp(d.z, d0, d1);
}

Spectrum GridDensityMedium::Tr(const Ray &_ray, Sampler &sampler) const {
    // Transform the ray into local coordinates and determine overlap interval
    // [_tMin, tMax_]
    const Bounds3f dataBounds(Point3f(0.f, 0.f, 0.f), Point3f(1.f, 1.f, 1.f));
    Ray ray = WorldToMedium(
        Ray(_ray.o, Normalize(_ray.d), _ray.tMax * _ray.d.Length()));
    Float tMin, tMax;
    if (!dataBounds.IntersectP(ray, &tMin, &tMax)) return Spectrum(1.f);
    tMin = std::max(tMin, (Float)0);
    tMax = std::min(tMax, ray.tMax);
    if (tMin >= tMax) return Spectrum(1.f);
    Float tr = 1.f;
    // Perform ratio tracking to estimate the transmittance value
    Float t = tMin;
    while (true) {
        t -= std::log(1 - sampler.Get1D()) * invMaxDensity;
        if (t >= tMax) break;
        Float density = Density(ray(t));
        tr *= 1.f - std::max((Float)0, sigma_t * density * invMaxDensity);
    }
    return Spectrum(tr);
}

Spectrum GridDensityMedium::Sample(const Ray &_ray, Sampler &sampler,
                                   MemoryArena &arena,
                                   MediumInteraction *mi) const {
    // Transform the ray into local coordinates and determine overlap interval
    // [_tMin, tMax_]
    const Bounds3f dataBounds(Point3f(0.f, 0.f, 0.f), Point3f(1.f, 1.f, 1.f));
    Ray ray = WorldToMedium(
        Ray(_ray.o, Normalize(_ray.d), _ray.tMax * _ray.d.Length()));
    Float tMin, tMax;
    if (!dataBounds.IntersectP(ray, &tMin, &tMax)) return Spectrum(1.f);
    tMin = std::max(tMin, (Float)0);
    tMax = std::min(tMax, ray.tMax);
    if (tMin >= tMax) return Spectrum(1.f);

    // Run Delta-Tracking iterations to sample a medium interaction
    Float t = tMin;
    while (true) {
        t -= std::log(1 - sampler.Get1D()) * invMaxDensity;
        if (t >= tMax) break;
        Float density = Density(ray(t));
        if (density * invMaxDensity * sigma_t > sampler.Get1D()) {
            // Populate _mi_ with medium interaction information and return
            PhaseFunction *phase = ARENA_ALLOC(arena, HenyeyGreenstein)(g);
            *mi = MediumInteraction(_ray(t), -_ray.d, _ray.time, this, phase);
            return sigma_s / sigma_t;
        }
    }
    return Spectrum(1.0f);
}

Float GridDensityMedium::Pdf(const Ray &ray, const Interaction &it) const {
    Error("GridDensityMedium::Pdf() is unimplemented");
    return 0;
}
