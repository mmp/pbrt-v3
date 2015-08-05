
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

// core/bssrdf.cpp*
#include "bssrdf.h"
#include "interpolation.h"
#include "scene.h"

// BSSRDF Utility Functions
Float FresnelMoment1(Float eta) {
    Float eta2 = eta * eta, eta3 = eta2 * eta, eta4 = eta3 * eta,
          eta5 = eta4 * eta;
    if (eta < 1)
        return 0.45966f - 1.73965f * eta + 3.37668f * eta2 - 3.904945 * eta3 +
               2.49277f * eta4 - 0.68441f * eta5;
    else
        return -4.61686f + 11.1136f * eta - 10.4646f * eta2 + 5.11455f * eta3 -
               1.27198f * eta4 + 0.12746f * eta5;
}

Float FresnelMoment2(Float eta) {
    Float eta2 = eta * eta, eta3 = eta2 * eta, eta4 = eta3 * eta,
          eta5 = eta4 * eta;
    if (eta < 1) {
        return 0.27614f - 0.87350f * eta + 1.12077f * eta2 - 0.65095f * eta3 +
               0.07883f * eta4 + 0.04860f * eta5;
    } else {
        Float r_eta = 1 / eta, r_eta2 = r_eta * r_eta, r_eta3 = r_eta2 * r_eta;
        return -547.033f + 45.3087f * r_eta3 - 218.725f * r_eta2 +
               458.843f * r_eta + 404.557f * eta - 189.519f * eta2 +
               54.9327f * eta3 - 9.00603f * eta4 + 0.63942f * eta5;
    }
}

Float BeamDiffusionMS(Float sig_s, Float sig_a, Float g, Float eta, Float r) {
    // Compute reduced scattering coefficients and albedo
    Float sigp_s = sig_s * (1 - g);
    Float sigp_t = sig_a + sigp_s;
    Float alphap = sigp_s / sigp_t;

    // Compute diffusion and transport coefficients
    Float D_g = (2 * sig_a + sigp_s) / (3 * sigp_t * sigp_t);
    Float sig_tr = std::sqrt(sig_a / D_g);

    // Determine boundary conditions
    Float fm1 = FresnelMoment1(eta), fm2 = FresnelMoment2(eta);

    // Determine the position of the extrapolated boundary
    Float zb = 2 * D_g * (1 + 3 * fm2) / (1 - 2 * fm1);

    // Determine fluence and vector irradiance weights
    Float c_phi = .25f * (1 - 2 * fm1), c_E = .5f * (1 - 3 * fm2);
    Float integral = 0.0f;
    const int nSamples = 100;
    for (int i = 0; i < nSamples; ++i) {
        // Evaluate dipole integrand and add to _integral_
        Float zr = -std::log(1 - (i + .5f) / nSamples) / sigp_t,
              zv = zr + 2 * zb, dr = std::sqrt(r * r + zr * zr),
              dv = std::sqrt(r * r + zv * zv);

        // Compute fluence _phi_ and vector irradiance _E\_n_ due to the dipole
        Float phi = alphap * Inv4Pi / D_g *
                    (std::exp(-sig_tr * dr) / dr - std::exp(-sig_tr * dv) / dv);
        Float E_n =
            alphap * Inv4Pi *
            (zr * (1 + sig_tr * dr) * std::exp(-sig_tr * dr) / (dr * dr * dr) +
             (zr + 2 * zb) * (1 + sig_tr * dv) * std::exp(-sig_tr * dv) /
                 (dv * dv * dv));

        // Compute empirical correction factor _kappa_
        Float kappa = 1 - std::exp(-2 * sigp_t * (dr + zr));
        integral += kappa * alphap * (phi * c_phi + E_n * c_E);
    }
    return integral / nSamples;
}

Float BeamDiffusionSS(Float sig_s, Float sig_a, Float g, Float eta, Float r) {
    Float sig_t = sig_a + sig_s, rho = sig_s / sig_t;
    // Find minimum $t$ below the critical angle
    Float tCrit = r * std::sqrt(eta * eta - 1);
    Float integral = 0.0f;
    const int nSamples = 100;
    for (int i = 0; i < nSamples; ++i) {
        // Evaluate single scattering integrand and add to _integral_
        Float t = tCrit - std::log(1 - (i + .5f) / nSamples) / sig_t;

        // Determine length of connecting segment
        Float d = std::sqrt(r * r + t * t);

        // Determine angle cosine for single scattering contribution
        Float cosTheta = t / d;

        // Add contribution of single scattering at depth $t$
        integral += PhaseHG(-cosTheta, g) * std::exp(-sig_t * d) * cosTheta /
                    (d * d) * (1 - FrDielectric(-cosTheta, 1, eta));
    }
    // Return result of single scattering integral
    return integral * rho * std::exp(-tCrit * sig_t) / nSamples;
}

void ComputeBeamDiffusionBSSRDF(Float g, Float eta, BSSRDFTable *t) {
    // Choose radii of the diffusion profile disretization
    t->radiusSamples[0] = 0;
    t->radiusSamples[1] = 2.5e-3f;
    for (int i = 2; i < t->nRadiusSamples; ++i)
        t->radiusSamples[i] = t->radiusSamples[i - 1] * 1.2f;
    ParallelFor([&](int i) {
        // Compute the diffusion profile for the _i_-th albedo sample
        Float rho = (1 - std::exp(-8 * i / (Float)(t->nRhoSamples - 1))) /
                    (1 - std::exp(-8));

        // Compute profile for chosen $\rho$
        t->profile[i * t->nRadiusSamples] = 0.0f;
        for (int j = 0; j < t->nRadiusSamples; ++j)
            t->profile[i * t->nRadiusSamples + j] =
                2 * Pi * t->radiusSamples[j] *
                (BeamDiffusionSS(rho, 1 - rho, g, eta, t->radiusSamples[j]) +
                 BeamDiffusionMS(rho, 1 - rho, g, eta, t->radiusSamples[j]));

        // Compute $\rho_{\roman{eff}}$ and importance sampling CDF
        t->rhoSamples[i] = rho;
        t->rhoEff[i] =
            IntegrateCatmullRom(t->nRadiusSamples, t->radiusSamples.get(),
                                &t->profile[i * t->nRadiusSamples],
                                &t->profileCDF[i * t->nRadiusSamples]);
    }, t->nRhoSamples);
}

void SubsurfaceFromDiffuse(const BSSRDFTable &table, const Spectrum &Kd,
                           const Spectrum &sigma_t, Spectrum *sigma_a,
                           Spectrum *sigma_s) {
    for (int c = 0; c < Spectrum::nSamples; ++c) {
        Float rho = InvertCatmullRom(table.nRhoSamples, table.rhoSamples.get(),
                                     table.rhoEff.get(), Kd[c]);
        (*sigma_s)[c] = rho * sigma_t[c];
        (*sigma_a)[c] = (1 - rho) * sigma_t[c];
    }
}

// BSSRDF Method Definitions
Spectrum SeparableBSSRDF::Sp(const SurfaceInteraction &pi) const {
    return Sr(Distance(po.p, pi.p));
}

BSSRDFTable::BSSRDFTable(int nRhoSamples, int nRadiusSamples)
    : nRhoSamples(nRhoSamples),
      nRadiusSamples(nRadiusSamples),
      rhoSamples(new Float[nRhoSamples]),
      radiusSamples(new Float[nRadiusSamples]),
      profile(new Float[nRadiusSamples * nRhoSamples]),
      rhoEff(new Float[nRhoSamples]),
      profileCDF(new Float[nRadiusSamples * nRhoSamples]) {}

Spectrum TabulatedBSSRDF::Sr(Float r) const {
    Spectrum fs(0.f);
    for (int ch = 0; ch < Spectrum::nSamples; ++ch) {
        // Convert $r$ into unitless optical radius $r_{\roman{optical}}$
        Float rOptical = r * sigma_t[ch];

        // Compute spline weights to interpolate BSSRDF on channel _ch_
        int rhoOffset, radiusOffset;
        Float rhoWeights[4], radiusWeights[4];
        if (!CatmullRomWeights(table.nRhoSamples, table.rhoSamples.get(),
                               rho[ch], &rhoOffset, rhoWeights) ||
            !CatmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(),
                               rOptical, &radiusOffset, radiusWeights))
            continue;

        // Set BSSRDF value _fs[ch]_ using tensor spline interpolation
        Float lookup = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                Float weight = rhoWeights[i] * radiusWeights[j];
                if (weight != 0)
                    lookup +=
                        weight *
                        table.EvalProfile(rhoOffset + i, radiusOffset + j);
            }
        }

        // Cancel marginal PDF factor from tabulated BSSRDF profile
        if (rOptical != 0) lookup /= 2 * Pi * rOptical;
        fs[ch] = lookup;
    }
    // Transform BSSRDF value into world space units
    fs *= sigma_t * sigma_t;
    return fs.Clamp();
}

Spectrum SeparableBSSRDF::Sample_S(const Scene &scene, Float sample1,
                                   const Point2f &sample2, MemoryArena &arena,
                                   SurfaceInteraction *si, Float *pdf) const {
    Spectrum result = Sample_Sp(scene, sample1, sample2, arena, si, pdf);
    if (!result.IsBlack()) {
        // Initialize material model at sampled surface interaction
        si->bsdf = ARENA_ALLOC(arena, BSDF)(*si);
        si->bsdf->Add(ARENA_ALLOC(arena, SeparableBSSRDFAdapter)(this));
        si->wo = Vector3f(si->shading.n);
    }
    return result;
}

Spectrum SeparableBSSRDF::Sample_Sp(const Scene &scene, Float sample1,
                                    const Point2f &sample2, MemoryArena &arena,
                                    SurfaceInteraction *pi, Float *pdf) const {
    // Choose projection axis for BSSRDF sampling
    Vector3f axis;
    if (sample1 < .25f) {
        axis = ss;
        sample1 *= 4;
    } else if (sample1 < .5f) {
        axis = ts;
        sample1 = (sample1 - .25f) * 4;
    } else {
        axis = Vector3f(ns);
        sample1 = (sample1 - .5f) * 2;
    }

    // Choose spectral channel for BSSRDF sampling
    int ch =
        Clamp((int)(sample1 * Spectrum::nSamples), 0, Spectrum::nSamples - 1);
    sample1 = sample1 * Spectrum::nSamples - ch;

    // Sample BSSRDF profile in polar coordinates
    Float r = Sample_Sr(ch, sample2.x);
    if (r < 0) return Spectrum(0.f);
    Float phi = 2 * Pi * sample2.y;

    // Compute BSSRDF profile bounds and intersection height
    Float rMax = Sample_Sr(ch, 0.999f);
    if (r > rMax) return Spectrum(0.f);
    Float l = 2 * std::sqrt(rMax * rMax - r * r);

    // Compute BSSRDF sampling ray segment
    Vector3f v0, v1;
    CoordinateSystem(axis, &v0, &v1);
    Interaction base;
    base.p =
        po.p + r * (v0 * std::cos(phi) + v1 * std::sin(phi)) - l * axis * 0.5f;
    base.time = po.time;
    Point3f target = base.p + l * axis;

    // Intersect BSSRDF sampling ray against the scene geometry
    struct IntersectionChain {
        SurfaceInteraction si;
        IntersectionChain *next;
    };
    IntersectionChain *chain = ARENA_ALLOC(arena, IntersectionChain)(),
                      *ptr = chain;
    int nFound = 0;
    while (true) {
        if (!scene.Intersect(base.SpawnRayTo(target), &ptr->si)) break;
        base = ptr->si;
        // Append admissible intersection to _IntersectionChain_
        if (ptr->si.primitive->GetMaterial() == material) {
            IntersectionChain *next = ARENA_ALLOC(arena, IntersectionChain)();
            ptr->next = next;
            ptr = next;
            nFound++;
        }
    }

    // Randomly choose one of several intersections during BSSRDF sampling
    if (nFound == 0) return Spectrum(0.0f);
    int selected = Clamp((int)(sample1 * nFound), 0, nFound - 1);
    while (selected-- > 0) chain = chain->next;
    *pi = chain->si;

    // Compute sample PDF and return the BSSRDF value
    *pdf = Pdf_Sp(*pi) / nFound;
    return Sp(*pi);
}

Float SeparableBSSRDF::Pdf_Sp(const SurfaceInteraction &pi) const {
    // Express $\pti-\pto$ and $N_i$ with respect to local coordinates at $\pto$
    Vector3f d = po.p - pi.p;
    Vector3f dLocal(Dot(ss, d), Dot(ts, d), Dot(ns, d));
    Normal3f nLocal(Dot(ss, pi.n), Dot(ts, pi.n), Dot(ns, pi.n));

    // Compute BSSRDF profile radius under projection along each axis
    Float rProj[3] = {std::sqrt(dLocal.y * dLocal.y + dLocal.z * dLocal.z),
                      std::sqrt(dLocal.z * dLocal.z + dLocal.x * dLocal.x),
                      std::sqrt(dLocal.x * dLocal.x + dLocal.y * dLocal.y)};

    // Return combined probability from all BSSRDF sampling strategies
    Float result = 0, axisProb[3] = {.25f, .25f, .5f},
          chProb = 1 / (Float)Spectrum::nSamples;
    for (int axis = 0; axis < 3; ++axis)
        for (int ch = 0; ch < Spectrum::nSamples; ++ch)
            result += Pdf_Sr(ch, rProj[axis]) * std::abs(nLocal[axis]) *
                      chProb * axisProb[axis];
    return result;
}

Float TabulatedBSSRDF::Sample_Sr(int ch, Float sample) const {
    if (sigma_t[ch] == 0) return -1;
    return SampleCatmullRom2D(table.nRhoSamples, table.nRadiusSamples,
                              table.rhoSamples.get(), table.radiusSamples.get(),
                              table.profile.get(), table.profileCDF.get(),
                              rho[ch], sample) /
           sigma_t[ch];
}

Float TabulatedBSSRDF::Pdf_Sr(int ch, Float r) const {
    // Convert $r$ into unitless optical radius $r_{\roman{optical}}$
    Float rOptical = r * sigma_t[ch];

    // Compute spline weights to interpolate BSSRDF density on channel _ch_
    int rhoOffset, radiusOffset;
    Float rhoWeights[4], radiusWeights[4];
    if (!CatmullRomWeights(table.nRhoSamples, table.rhoSamples.get(), rho[ch],
                           &rhoOffset, rhoWeights) ||
        !CatmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(),
                           rOptical, &radiusOffset, radiusWeights))
        return 0.f;

    // Return BSSRDF profile density for channel _ch_
    Float lookup = 0.f, rhoEff = 0;
    for (int i = 0; i < 4; ++i) {
        if (rhoWeights[i] == 0) continue;
        rhoEff += table.rhoEff[rhoOffset + i] * rhoWeights[i];
        for (int j = 0; j < 4; ++j) {
            if (radiusWeights[j] == 0) continue;
            lookup += table.EvalProfile(rhoOffset + i, radiusOffset + j) *
                      rhoWeights[i] * radiusWeights[j];
        }
    }

    // Cancel marginal PDF factor from tabulated BSSRDF profile
    if (rOptical != 0) lookup /= 2 * Pi * rOptical;
    return std::max((Float)0, lookup * sigma_t[ch] * sigma_t[ch] / rhoEff);
}
