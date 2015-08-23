
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

#ifndef PBRT_CORE_BSSRDF_H
#define PBRT_CORE_BSSRDF_H
#include "stdafx.h"

// core/bssrdf.h*
#include "interaction.h"
#include "reflection.h"
#include "stats.h"

// BSSRDF Declarations
Float FresnelMoment1(Float eta);
Float FresnelMoment2(Float eta);
class BSSRDF {
  public:
    // BSSRDF Public Methods
    BSSRDF(const SurfaceInteraction &po, Float eta) : po(po), eta(eta) {}

    // BSSRDF Interface
    virtual Spectrum S(const SurfaceInteraction &pi, const Vector3f &wi) = 0;
    virtual Spectrum Sample_S(const Scene &scene, Float sample1,
                              const Point2f &sample2, MemoryArena &arena,
                              SurfaceInteraction *si, Float *pdf) const = 0;

  protected:
    // BSSRDF Protected Data
    const SurfaceInteraction &po;
    Float eta;
};

class SeparableBSSRDF : public BSSRDF {
    friend class SeparableBSSRDFAdapter;

  public:
    // SeparableBSSRDF Public Methods
    SeparableBSSRDF(const SurfaceInteraction &po, Float eta,
                    const Material *material, TransportMode mode)
        : BSSRDF(po, eta),
          ns(po.shading.n),
          ss(Normalize(po.shading.dpdu)),
          ts(Cross(ns, ss)),
          material(material),
          mode(mode) {}
    Spectrum S(const SurfaceInteraction &pi, const Vector3f &wi) {
        ProfilePhase pp(Prof::BSSRDFEvaluation);
        Float Ft = (1 - FrDielectric(CosTheta(po.wo), 1.f, eta));
        return Ft * Sp(pi) * Sw(wi);
    }
    Spectrum Sw(const Vector3f &w) const {
        Float c = 1 - 2 * FresnelMoment1(1 / eta);
        return InvPi * (1 - FrDielectric(CosTheta(w), 1.f, eta)) / c;
    }
    Spectrum Sp(const SurfaceInteraction &si) const;
    Spectrum Sample_S(const Scene &scene, Float sample1, const Point2f &sample2,
                      MemoryArena &arena, SurfaceInteraction *si,
                      Float *pdf) const;
    Spectrum Sample_Sp(const Scene &scene, Float sample1,
                       const Point2f &sample2, MemoryArena &arena,
                       SurfaceInteraction *si, Float *pdf) const;
    Float Pdf_Sp(const SurfaceInteraction &si) const;

    // SeparableBSSRDF Interface
    virtual Spectrum Sr(Float d) const = 0;
    virtual Float Sample_Sr(int ch, Float sample) const = 0;
    virtual Float Pdf_Sr(int ch, Float r) const = 0;

  private:
    // SeparableBSSRDF Private Data
    Normal3f ns;
    Vector3f ss, ts;
    const Material *material;
    const TransportMode mode;
};

class TabulatedBSSRDF : public SeparableBSSRDF {
  public:
    // TabulatedBSSRDF Public Methods
    TabulatedBSSRDF(const SurfaceInteraction &po, const Material *material,
                    TransportMode mode, Float eta, const Spectrum &sigma_a,
                    const Spectrum &sigma_s, const BSSRDFTable &table)
        : SeparableBSSRDF(po, eta, material, mode), table(table) {
        sigma_t = sigma_a + sigma_s;
        for (int c = 0; c < Spectrum::nSamples; ++c)
            rho[c] = sigma_t[c] != 0 ? (sigma_s[c] / sigma_t[c]) : 0.f;
    }
    Spectrum Sr(Float distance) const;
    Float Pdf_Sr(int ch, Float distance) const;
    Float Sample_Sr(int ch, Float sample) const;

  private:
    // TabulatedBSSRDF Private Data
    const BSSRDFTable &table;
    Spectrum sigma_t, rho;
};

struct BSSRDFTable {
    // BSSRDFTable Public Data
    const int nRhoSamples, nRadiusSamples;
    std::unique_ptr<Float[]> rhoSamples, radiusSamples;
    std::unique_ptr<Float[]> profile;
    std::unique_ptr<Float[]> rhoEff;
    std::unique_ptr<Float[]> profileCDF;

    // BSSRDFTable Public Methods
    BSSRDFTable(int nRhoSamples, int nRadiusSamples);
    inline Float EvalProfile(int rhoIndex, int radiusIndex) const {
        return profile[rhoIndex * nRadiusSamples + radiusIndex];
    }
};

class SeparableBSSRDFAdapter : public BxDF {
  public:
    SeparableBSSRDFAdapter(const SeparableBSSRDF *bssrdf)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), bssrdf(bssrdf) {}
    Spectrum f(const Vector3f &, const Vector3f &wi) const {
        Spectrum value(bssrdf->Sw(wi));
        // Update BSSRDF transmission term to account for adjoint light
        // transport
        if (bssrdf->mode == TransportMode::Radiance)
            value *= bssrdf->eta * bssrdf->eta;
        return value;
    }

  private:
    const SeparableBSSRDF *bssrdf;
};

Float BeamDiffusionSS(Float sig_s, Float sig_a, Float g, Float eta, Float r);
Float BeamDiffusionMS(Float sig_s, Float sig_a, Float g, Float eta, Float r);
void ComputeBeamDiffusionBSSRDF(Float g, Float eta, BSSRDFTable *t);
void SubsurfaceFromDiffuse(const BSSRDFTable &table, const Spectrum &Kd,
                           const Spectrum &sigma_t, Spectrum *sigma_a,
                           Spectrum *sigma_s);

#endif  // PBRT_CORE_BSSRDF_H
