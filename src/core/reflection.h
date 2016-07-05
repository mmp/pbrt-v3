
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

#ifndef PBRT_CORE_REFLECTION_H
#define PBRT_CORE_REFLECTION_H

// core/reflection.h*
#include "pbrt.h"
#include "geometry.h"
#include "microfacet.h"
#include "shape.h"
#include "spectrum.h"

// Reflection Declarations
Float FrDielectric(Float cosThetaI, Float etaI, Float etaT);
Spectrum FrConductor(Float cosThetaI, const Spectrum &etaI,
                     const Spectrum &etaT, const Spectrum &k);

// BSDF Inline Functions
inline Float CosTheta(const Vector3f &w) { return w.z; }
inline Float Cos2Theta(const Vector3f &w) { return w.z * w.z; }
inline Float AbsCosTheta(const Vector3f &w) { return std::abs(w.z); }
inline Float Sin2Theta(const Vector3f &w) {
    return std::max((Float)0, (Float)1 - Cos2Theta(w));
}

inline Float SinTheta(const Vector3f &w) { return std::sqrt(Sin2Theta(w)); }

inline Float TanTheta(const Vector3f &w) { return SinTheta(w) / CosTheta(w); }

inline Float Tan2Theta(const Vector3f &w) {
    return Sin2Theta(w) / Cos2Theta(w);
}

inline Float CosPhi(const Vector3f &w) {
    Float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 1 : Clamp(w.x / sinTheta, -1, 1);
}

inline Float SinPhi(const Vector3f &w) {
    Float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 0 : Clamp(w.y / sinTheta, -1, 1);
}

inline Float Cos2Phi(const Vector3f &w) { return CosPhi(w) * CosPhi(w); }

inline Float Sin2Phi(const Vector3f &w) { return SinPhi(w) * SinPhi(w); }

inline Float CosDPhi(const Vector3f &wa, const Vector3f &wb) {
    return Clamp(
        (wa.x * wb.x + wa.y * wb.y) / std::sqrt((wa.x * wa.x + wa.y * wa.y) *
                                                (wb.x * wb.x + wb.y * wb.y)),
        -1, 1);
}

inline Vector3f Reflect(const Vector3f &wo, const Vector3f &n) {
    return -wo + 2 * Dot(wo, n) * n;
}

inline bool Refract(const Vector3f &wi, const Normal3f &n, Float eta,
                    Vector3f *wt) {
    // Compute $\cos \theta_\roman{t}$ using Snell's law
    Float cosThetaI = Dot(n, wi);
    Float sin2ThetaI = std::max(Float(0), Float(1 - cosThetaI * cosThetaI));
    Float sin2ThetaT = eta * eta * sin2ThetaI;

    // Handle total internal reflection for transmission
    if (sin2ThetaT >= 1) return false;
    Float cosThetaT = std::sqrt(1 - sin2ThetaT);
    *wt = eta * -wi + (eta * cosThetaI - cosThetaT) * Vector3f(n);
    return true;
}

inline bool SameHemisphere(const Vector3f &w, const Vector3f &wp) {
    return w.z * wp.z > 0;
}

inline bool SameHemisphere(const Vector3f &w, const Normal3f &wp) {
    return w.z * wp.z > 0;
}

// BSDF Declarations
enum BxDFType {
    BSDF_REFLECTION = 1 << 0,
    BSDF_TRANSMISSION = 1 << 1,
    BSDF_DIFFUSE = 1 << 2,
    BSDF_GLOSSY = 1 << 3,
    BSDF_SPECULAR = 1 << 4,
    BSDF_ALL = BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR | BSDF_REFLECTION |
               BSDF_TRANSMISSION,
};

struct FourierBSDFTable {
    // FourierBSDFTable Public Data
    Float eta;
    int mMax;
    int nChannels;
    int nMu;
    Float *mu;
    int *m;
    int *aOffset;
    Float *a;
    Float *a0;
    Float *cdf;
    Float *recip;

    // FourierBSDFTable Public Methods
    static bool Read(const std::string &filename, FourierBSDFTable *table);
    const Float *GetAk(int offsetI, int offsetO, int *mptr) const {
        *mptr = m[offsetO * nMu + offsetI];
        return a + aOffset[offsetO * nMu + offsetI];
    }
    bool GetWeightsAndOffset(Float cosTheta, int *offset,
                             Float weights[4]) const;
};

class BSDF {
  public:
    // BSDF Public Methods
    BSDF(const SurfaceInteraction &si, Float eta = 1)
        : eta(eta),
          ns(si.shading.n),
          ng(si.n),
          ss(Normalize(si.shading.dpdu)),
          ts(Cross(ns, ss)) {}
    void Add(BxDF *b) {
        Assert(nBxDFs < MaxBxDFs);
        bxdfs[nBxDFs++] = b;
    }
    int NumComponents(BxDFType flags = BSDF_ALL) const;
    Vector3f WorldToLocal(const Vector3f &v) const {
        return Vector3f(Dot(v, ss), Dot(v, ts), Dot(v, ns));
    }
    Vector3f LocalToWorld(const Vector3f &v) const {
        return Vector3f(ss.x * v.x + ts.x * v.y + ns.x * v.z,
                        ss.y * v.x + ts.y * v.y + ns.y * v.z,
                        ss.z * v.x + ts.z * v.y + ns.z * v.z);
    }
    Spectrum f(const Vector3f &woW, const Vector3f &wiW,
               BxDFType flags = BSDF_ALL) const;
    Spectrum rho(int nSamples, const Point2f *samples1, const Point2f *samples2,
                 BxDFType flags = BSDF_ALL) const;
    Spectrum rho(const Vector3f &wo, int nSamples, const Point2f *samples,
                 BxDFType flags = BSDF_ALL) const;
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType type = BSDF_ALL,
                      BxDFType *sampledType = nullptr) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi,
              BxDFType flags = BSDF_ALL) const;
    std::string ToString() const;

    // BSDF Public Data
    const Float eta;

  private:
    // BSDF Private Methods
    ~BSDF() {}

    // BSDF Private Data
    const Normal3f ns, ng;
    const Vector3f ss, ts;
    int nBxDFs = 0;
    static PBRT_CONSTEXPR int MaxBxDFs = 8;
    BxDF *bxdfs[MaxBxDFs];
    friend class MixMaterial;
};

// BxDF Declarations
class BxDF {
  public:
    // BxDF Interface
    virtual ~BxDF() {}
    BxDF(BxDFType type) : type(type) {}
    bool MatchesFlags(BxDFType t) const { return (type & t) == type; }
    virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const = 0;
    virtual Spectrum Sample_f(const Vector3f &wo, Vector3f *wi,
                              const Point2f &sample, Float *pdf,
                              BxDFType *sampledType = nullptr) const;
    virtual Spectrum rho(const Vector3f &wo, int nSamples,
                         const Point2f *samples) const;
    virtual Spectrum rho(int nSamples, const Point2f *samples1,
                         const Point2f *samples2) const;
    virtual Float Pdf(const Vector3f &wi, const Vector3f &wo) const;
    virtual std::string ToString() const = 0;

    // BxDF Public Data
    const BxDFType type;
};

class ScaledBxDF : public BxDF {
  public:
    // ScaledBxDF Public Methods
    ScaledBxDF(BxDF *bxdf, const Spectrum &scale)
        : BxDF(BxDFType(bxdf->type)), bxdf(bxdf), scale(scale) {}
    Spectrum rho(const Vector3f &w, int nSamples,
                 const Point2f *samples) const {
        return scale * bxdf->rho(w, nSamples, samples);
    }
    Spectrum rho(int nSamples, const Point2f *samples1,
                 const Point2f *samples2) const {
        return scale * bxdf->rho(nSamples, samples1, samples2);
    }
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample,
                      Float *pdf, BxDFType *sampledType) const;
    std::string ToString() const;

  private:
    BxDF *bxdf;
    Spectrum scale;
};

class Fresnel {
  public:
    // Fresnel Interface
    virtual ~Fresnel();
    virtual Spectrum Evaluate(Float cosI) const = 0;
    virtual std::string ToString() const = 0;
};

class FresnelConductor : public Fresnel {
  public:
    // FresnelConductor Public Methods
    Spectrum Evaluate(Float cosThetaI) const;
    FresnelConductor(const Spectrum &etaI, const Spectrum &etaT,
                     const Spectrum &k)
        : etaI(etaI), etaT(etaT), k(k) {}
    std::string ToString() const;

  private:
    Spectrum etaI, etaT, k;
};

class FresnelDielectric : public Fresnel {
  public:
    // FresnelDielectric Public Methods
    Spectrum Evaluate(Float cosThetaI) const;
    FresnelDielectric(Float etaI, Float etaT) : etaI(etaI), etaT(etaT) {}
    std::string ToString() const;

  private:
    Float etaI, etaT;
};

class FresnelNoOp : public Fresnel {
  public:
    Spectrum Evaluate(Float) const { return Spectrum(1.); }
    std::string ToString() const { return "[ FresnelNoOp ]"; }
};

class SpecularReflection : public BxDF {
  public:
    // SpecularReflection Public Methods
    SpecularReflection(const Spectrum &R, Fresnel *fresnel)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_SPECULAR)),
          R(R),
          fresnel(fresnel) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const {
        return Spectrum(0.f);
    }
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const { return 0; }
    std::string ToString() const;

  private:
    // SpecularReflection Private Data
    const Spectrum R;
    const Fresnel *fresnel;
};

class SpecularTransmission : public BxDF {
  public:
    // SpecularTransmission Public Methods
    SpecularTransmission(const Spectrum &T, Float etaA, Float etaB,
                         TransportMode mode)
        : BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR)),
          T(T),
          etaA(etaA),
          etaB(etaB),
          fresnel(etaA, etaB),
          mode(mode) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const {
        return Spectrum(0.f);
    }
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const { return 0; }
    std::string ToString() const;

  private:
    // SpecularTransmission Private Data
    const Spectrum T;
    const Float etaA, etaB;
    const FresnelDielectric fresnel;
    const TransportMode mode;
};

class FresnelSpecular : public BxDF {
  public:
    // FresnelSpecular Public Methods
    FresnelSpecular(const Spectrum &R, const Spectrum &T, Float etaA,
                    Float etaB, TransportMode mode)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_SPECULAR)),
          R(R),
          T(T),
          etaA(etaA),
          etaB(etaB),
          fresnel(etaA, etaB),
          mode(mode) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const {
        return Spectrum(0.f);
    }
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const { return 0; }
    std::string ToString() const;

  private:
    // FresnelSpecular Private Data
    const Spectrum R, T;
    const Float etaA, etaB;
    const FresnelDielectric fresnel;
    const TransportMode mode;
};

class LambertianReflection : public BxDF {
  public:
    // LambertianReflection Public Methods
    LambertianReflection(const Spectrum &R)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum rho(const Vector3f &, int, const Point2f *) const { return R; }
    Spectrum rho(int, const Point2f *, const Point2f *) const { return R; }
    std::string ToString() const;

  private:
    // LambertianReflection Private Data
    const Spectrum R;
};

class LambertianTransmission : public BxDF {
  public:
    // LambertianTransmission Public Methods
    LambertianTransmission(const Spectrum &T)
        : BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_DIFFUSE)), T(T) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum rho(const Vector3f &, int, const Point2f *) const { return T; }
    Spectrum rho(int, const Point2f *, const Point2f *) const { return T; }
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const;
    std::string ToString() const;

  private:
    // LambertianTransmission Private Data
    Spectrum T;
};

class OrenNayar : public BxDF {
  public:
    // OrenNayar Public Methods
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    OrenNayar(const Spectrum &R, Float sigma)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {
        sigma = Radians(sigma);
        Float sigma2 = sigma * sigma;
        A = 1.f - (sigma2 / (2.f * (sigma2 + 0.33f)));
        B = 0.45f * sigma2 / (sigma2 + 0.09f);
    }
    std::string ToString() const;

  private:
    // OrenNayar Private Data
    const Spectrum R;
    Float A, B;
};

class MicrofacetReflection : public BxDF {
  public:
    // MicrofacetReflection Public Methods
    MicrofacetReflection(const Spectrum &R,
                         MicrofacetDistribution *distribution, Fresnel *fresnel)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_GLOSSY)),
          R(R),
          distribution(distribution),
          fresnel(fresnel) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const;
    std::string ToString() const;

  private:
    // MicrofacetReflection Private Data
    const Spectrum R;
    const MicrofacetDistribution *distribution;
    const Fresnel *fresnel;
};

class MicrofacetTransmission : public BxDF {
  public:
    // MicrofacetTransmission Public Methods
    MicrofacetTransmission(const Spectrum &T,
                           MicrofacetDistribution *distribution, Float etaA,
                           Float etaB, TransportMode mode)
        : BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_GLOSSY)),
          T(T),
          distribution(distribution),
          etaA(etaA),
          etaB(etaB),
          fresnel(etaA, etaB),
          mode(mode) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const;
    std::string ToString() const;

  private:
    // MicrofacetTransmission Private Data
    const Spectrum T;
    const MicrofacetDistribution *distribution;
    const Float etaA, etaB;
    const FresnelDielectric fresnel;
    const TransportMode mode;
};

class FresnelBlend : public BxDF {
  public:
    // FresnelBlend Public Methods
    FresnelBlend(const Spectrum &Rd, const Spectrum &Rs,
                 MicrofacetDistribution *distrib);
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum SchlickFresnel(Float cosTheta) const {
        auto pow5 = [](Float v) { return (v * v) * (v * v) * v; };
        return Rs + pow5(1 - cosTheta) * (Spectrum(1.) - Rs);
    }
    Spectrum Sample_f(const Vector3f &wi, Vector3f *sampled_f, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wi, const Vector3f &wo) const;
    std::string ToString() const;

  private:
    // FresnelBlend Private Data
    const Spectrum Rd, Rs;
    MicrofacetDistribution *distribution;
};

class FourierBSDF : public BxDF {
  public:
    // FourierBSDF Public Methods
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    FourierBSDF(const FourierBSDFTable &bsdfTable, TransportMode mode)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_GLOSSY)),
          bsdfTable(bsdfTable),
          mode(mode) {}
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wi, const Vector3f &wo) const;
    std::string ToString() const;

  private:
    // FourierBSDF Private Data
    const FourierBSDFTable &bsdfTable;
    const TransportMode mode;
};

// BSDF Inline Method Definitions
inline int BSDF::NumComponents(BxDFType flags) const {
    int num = 0;
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(flags)) ++num;
    return num;
}

#endif  // PBRT_CORE_REFLECTION_H
