
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

// core/reflection.cpp*
#include "reflection.h"
#include "spectrum.h"
#include "sampler.h"
#include "sampling.h"
#include "interpolation.h"
#include "scene.h"
#include "interaction.h"
#include <stdarg.h>

// BxDF Utility Functions
Float FrDielectric(Float cosThetaI, Float etai, Float etat) {
    cosThetaI = Clamp(cosThetaI, -1, 1);
    // Potentially swap indices of refraction
    bool entering = cosThetaI > 0.f;
    if (!entering) {
        std::swap(etai, etat);
        cosThetaI = std::abs(cosThetaI);
    }

    // Compute _cosThetaT_ using Snell's law
    Float sinThetaI =
        std::sqrt(std::max((Float)0, 1.f - cosThetaI * cosThetaI));
    Float sinThetaT = etai / etat * sinThetaI;

    // Handle total internal reflection
    if (sinThetaT >= 1.f) return 1.f;
    Float cosThetaT =
        std::sqrt(std::max((Float)0, 1.f - sinThetaT * sinThetaT));
    Float Rparl = ((etat * cosThetaI) - (etai * cosThetaT)) /
                  ((etat * cosThetaI) + (etai * cosThetaT));
    Float Rperp = ((etai * cosThetaI) - (etat * cosThetaT)) /
                  ((etai * cosThetaI) + (etat * cosThetaT));
    return (Rparl * Rparl + Rperp * Rperp) / 2.f;
}

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
Spectrum FrConductor(Float cosThetaI, const Spectrum &etai,
                     const Spectrum &etat, const Spectrum &k) {
    Spectrum eta = etat / etai;
    Spectrum etak = k / etai;

    Float cosThetaI2 = cosThetaI * cosThetaI;
    Float sinThetaI2 = 1. - cosThetaI2;
    Spectrum eta2 = eta * eta;
    Spectrum etak2 = etak * etak;

    Spectrum t0 = eta2 - etak2 - sinThetaI2;
    Spectrum a2plusb2 = Sqrt(t0 * t0 + 4 * eta2 * etak2);
    Spectrum t1 = a2plusb2 + cosThetaI2;
    Spectrum a = Sqrt(0.5f * (a2plusb2 + t0));
    Spectrum t2 = (Float)2 * cosThetaI * a;
    Spectrum Rs = (t1 - t2) / (t1 + t2);

    Spectrum t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
    Spectrum t4 = t2 * sinThetaI2;
    Spectrum Rp = Rs * (t3 - t4) / (t3 + t4);

    return 0.5 * (Rp + Rs);
}

// BxDF Method Definitions
Spectrum ScaledBxDF::f(const Vector3f &wo, const Vector3f &wi) const {
    return scale * bxdf->f(wo, wi);
}

Spectrum ScaledBxDF::Sample_f(const Vector3f &wo, Vector3f *wi,
                              const Point2f &sample, Float *pdf,
                              BxDFType *sampledType) const {
    Spectrum f = bxdf->Sample_f(wo, wi, sample, pdf, sampledType);
    return scale * f;
}

Fresnel::~Fresnel() {}
Spectrum FresnelConductor::Evaluate(Float cosThetaI) const {
    return FrConductor(std::abs(cosThetaI), etai, etat, k);
}

Spectrum FresnelDielectric::Evaluate(Float cosThetaI) const {
    return FrDielectric(cosThetaI, etaI, etaT);
}

Spectrum SpecularReflection::Sample_f(const Vector3f &wo, Vector3f *wi,
                                      const Point2f &sample, Float *pdf,
                                      BxDFType *sampledType) const {
    // Compute perfect specular reflection direction
    *wi = Vector3f(-wo.x, -wo.y, wo.z);
    *pdf = 1.f;
    return fresnel->Evaluate(CosTheta(wo)) * R / AbsCosTheta(*wi);
}

Spectrum SpecularTransmission::Sample_f(const Vector3f &wo, Vector3f *wi,
                                        const Point2f &sample, Float *pdf,
                                        BxDFType *sampledType) const {
    // Figure out which $\eta$ is incident and which is transmitted
    bool entering = CosTheta(wo) > 0.f;
    Float etai = entering ? etaa : etab;
    Float etat = entering ? etab : etaa;

    // Compute ray direction for specular transmission
    if (!Refract(wo, Faceforward(Normal3f(0, 0, 1), wo), etai / etat, wi))
        return 0.;
    *pdf = 1.f;
    Spectrum ft = T * (Spectrum(1.) - fresnel.Evaluate(CosTheta(*wi)));
    // Account for non-symmetry with transmission to different medium
    if (mode == TransportMode::Radiance) ft *= (etai * etai) / (etat * etat);
    return ft / AbsCosTheta(*wi);
}

Spectrum LambertianReflection::f(const Vector3f &wo, const Vector3f &wi) const {
    return R * InvPi;
}

Spectrum LambertianTransmission::f(const Vector3f &wo,
                                   const Vector3f &wi) const {
    return T * InvPi;
}

Spectrum OrenNayar::f(const Vector3f &wo, const Vector3f &wi) const {
    Float sinThetaI = SinTheta(wi);
    Float sinThetaO = SinTheta(wo);
    // Compute cosine term of Oren-Nayar model
    Float maxCos = 0.f;
    if (sinThetaI > 1e-4 && sinThetaO > 1e-4) {
        Float sinPhiI = SinPhi(wi), cosPhiI = CosPhi(wi);
        Float sinPhiO = SinPhi(wo), cosPhiO = CosPhi(wo);
        Float dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
        maxCos = std::max((Float)0., dCos);
    }

    // Compute sine and tangent terms of Oren-Nayar model
    Float sinAlpha, tanBeta;
    if (AbsCosTheta(wi) > AbsCosTheta(wo)) {
        sinAlpha = sinThetaO;
        tanBeta = sinThetaI / AbsCosTheta(wi);
    } else {
        sinAlpha = sinThetaI;
        tanBeta = sinThetaO / AbsCosTheta(wo);
    }
    return R * InvPi * (A + B * maxCos * sinAlpha * tanBeta);
}

Spectrum MicrofacetReflection::f(const Vector3f &wo, const Vector3f &wi) const {
    Float cosThetaO = AbsCosTheta(wo), cosThetaI = AbsCosTheta(wi);
    if (cosThetaI == 0.f || cosThetaO == 0.f) return Spectrum(0.);
    Vector3f wh = wi + wo;
    if (wh.x == 0.f && wh.y == 0.f && wh.z == 0.f) return Spectrum(0.);
    wh = Normalize(wh);
    Float cosThetaH = Dot(wi, wh);
    Spectrum F = fresnel->Evaluate(cosThetaH);
    return R * distribution->D(wh) * distribution->G(wo, wi) * F /
           (4.f * cosThetaI * cosThetaO);
}

Spectrum MicrofacetTransmission::f(const Vector3f &wo,
                                   const Vector3f &wi) const {
    if (SameHemisphere(wo, wi)) return 0.f;  // transmission only

    Float cosThetaO = CosTheta(wo);
    Float cosThetaI = CosTheta(wi);
    if (cosThetaI == 0.f || cosThetaO == 0.f) return Spectrum(0.f);

    // Compute $\wh$ from $\wo$ and $\wi$ for microfacet transmission
    Float eta = CosTheta(wo) > 0 ? (etaInterior / etaExterior)
                                 : (etaExterior / etaInterior);
    Vector3f wh = Normalize(wo + wi * eta);
    if (wh.z < 0) wh = -wh;

    Spectrum F = fresnel.Evaluate(Dot(wo, wh));

    Float sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
    Float factor = (mode == TransportMode::Radiance) ? (1.f / eta) : 1.f;

    return (Spectrum(1.f) - F) * T *
           std::abs(distribution->D(wh) * distribution->G(wo, wi) * eta * eta *
                    AbsDot(wi, wh) * AbsDot(wo, wh) * factor * factor /
                    (cosThetaI * cosThetaO * sqrtDenom * sqrtDenom));
}

FresnelBlend::FresnelBlend(const Spectrum &Rd, const Spectrum &Rs,
                           MicrofacetDistribution *distribution)
    : BxDF(BxDFType(BSDF_REFLECTION | BSDF_GLOSSY)),
      Rd(Rd),
      Rs(Rs),
      distribution(distribution) {}

Spectrum FresnelBlend::f(const Vector3f &wo, const Vector3f &wi) const {
    Spectrum diffuse = (28.f / (23.f * Pi)) * Rd * (Spectrum(1.f) - Rs) *
                       (1.f - std::pow(1.f - .5f * AbsCosTheta(wi), 5)) *
                       (1.f - std::pow(1.f - .5f * AbsCosTheta(wo), 5));
    Vector3f wh = wi + wo;
    if (wh.x == 0.f && wh.y == 0.f && wh.z == 0.f) return Spectrum(0.f);
    wh = Normalize(wh);
    Spectrum specular =
        distribution->D(wh) /
        (4.f * AbsDot(wi, wh) * std::max(AbsCosTheta(wi), AbsCosTheta(wo))) *
        SchlickFresnel(Dot(wi, wh));
    return diffuse + specular;
}

Spectrum KajiyaKay::f(const Vector3f &wo, const Vector3f &wi) const {
    Spectrum diffuse(0.f), specular(0.f);
    if (!Ks.IsBlack()) {
        // Compute specular Kajiya-Kay term
        Vector3f wh = wi + wo;
        if (!(wh.x == 0.f && wh.y == 0.f && wh.z == 0.f)) {
            wh = Normalize(wh);
#if 0
            Float costhetah = Dot(wo, wh);
            Float sinthetah = std::sqrt(std::max((Float)0., (Float)1. - costhetah * costhetah));
            Float costhetao = CosTheta(wo), sinthetao = SinTheta(wo);
            Float spec = std::pow(costhetao * costhetah + sinthetao * sinthetah,
                                  exponent);
#else
            Float tdoth = wh.x;
            Float spec = std::pow(
                std::sqrt(std::max((Float)0., (Float)1. - tdoth * tdoth)),
                exponent);
#endif
            specular = spec * Ks;
        }
    }
    // Compute diffuse Kajiya-Kay term
    diffuse = Kd * std::sqrt(std::max((Float)0., (Float)1. - wi.x * wi.x));
    return (InvPi / AbsCosTheta(wi)) * (diffuse + specular);
}

/*
  File format description:

  This is the file format generated by the material designer of the paper

  'A Comprehensive Framework for Rendering Layered Materials' by
  Wenzel Jakob, Eugene D'Eon, Otto Jakob and Steve Marschner
  Transactions on Graphics (Proceedings of SIGGRAPH 2014)

  This format specifies an isotropic BSDF expressed in a Spline x Fourier
  directional basis. It begins with a header of the following type:

 struct Header {
     uint8_t identifier[7];     // Set to 'SCATFUN'
     uint8_t version;           // Currently version is 1
     uint32_t flags;            // 0x01: file contains a BSDF, 0x02: uses
 harmonic extrapolation
     int nMu;                   // Number of samples in the elevational
 discretization

     int nCoeffs;               // Total number of Fourier series coefficients
 stored in the file
     int mMax;                  // Coeff. count for the longest series occurring
 in the file
     int nChannels;             // Number of color channels (usually 1 or 3)
     int nBases;                // Number of BSDF basis functions (relevant for
 texturing)

     int nMetadataBytes;        // Size of descriptive metadata that follows the
 BSDF data
     int nParameters;           // Number of textured material parameters
     int nParameterValues;      // Total number of BSDF samples for all textured
 parameters
     float eta;                 // Relative IOR through the material
 (eta(bottom) / eta(top))

     float alpha[2];            // Beckmann-equiv. roughness on the top (0) and
 bottom (1) side
     float unused[2];           // Unused fields to pad the header to 64 bytes
 };

  XXX explain remainder of file in textual form somewhere?

  Due to space constraints, two features are not currently implemented texturing
 and
  harmonic extrapolation, though it would be straightforward to port them from
 Mitsuba.
*/

inline bool IsBigEndian() {
    uint32_t i = 0x01020304;
    char c[4];
    memcpy(c, &i, 4);
    return (c[0] == 1);
}

bool FourierBSDFTable::Read(const std::string &filename,
                            FourierBSDFTable *bsdfTable) {
    bsdfTable->mu = bsdfTable->cdf = bsdfTable->a = nullptr;
    bsdfTable->aOffset = bsdfTable->m = nullptr;

    FILE *f = fopen(filename.c_str(), "rb");

    if (!f) {
        Error("Unable to open tabulated BSDF file \"%s\"", filename.c_str());
        return false;
    }

    auto read32 = [&](void *target, size_t count) -> bool {
        if (fread(target, sizeof(int), count, f) != count) return false;
        if (IsBigEndian()) {
            int32_t *tmp = (int32_t *)target;
            for (size_t i = 0; i < count; ++i) {
#if defined(__GNUC__)
                tmp[i] = __builtin_bswap32(tmp[i]);
#else
                tmp[i] = _byteswap_ulong(tmp[i]);
#endif
            }
        }
        return true;
    };
    auto readfloat = [&](Float *target, size_t count) -> bool {
        if (sizeof(*target) == sizeof(float)) return read32(target, count);

        std::unique_ptr<float[]> buf(new float[count]);
        bool ret = read32(buf.get(), count);
        for (size_t i = 0; i < count; ++i) target[i] = buf[i];
        return ret;
    };

    const char header_exp[8] = {'S', 'C', 'A', 'T', 'F', 'U', 'N', '\x01'};
    char header[8];
    int *offsetAndLength;

    if (fread(header, 1, 8, f) != 8 || memcmp(header, header_exp, 8) != 0)
        goto fail;

    int flags, nCoeffs, nBases, unused[4];

    if (!read32(&flags, 1) || !read32(&bsdfTable->nMu, 1) ||
        !read32(&nCoeffs, 1) || !read32(&bsdfTable->mMax, 1) ||
        !read32(&bsdfTable->nChannels, 1) || !read32(&nBases, 1) ||
        !read32(unused, 3) || !readfloat(&bsdfTable->eta, 1) ||
        !read32(&unused, 4))
        goto fail;

    /* Only a subset of BSDF files are supported for simplicity, in particular:
       monochromatic and
       RGB files with uniform (i.e. non-textured) material properties */
    if (flags != 1 ||
        (bsdfTable->nChannels != 1 && bsdfTable->nChannels != 3) || nBases != 1)
        goto fail;

    bsdfTable->mu = new Float[bsdfTable->nMu];
    bsdfTable->cdf = new Float[bsdfTable->nMu * bsdfTable->nMu];
    bsdfTable->avg = new Float[bsdfTable->nMu * bsdfTable->nMu];
    offsetAndLength = new int[bsdfTable->nMu * bsdfTable->nMu * 2];
    bsdfTable->aOffset = new int[bsdfTable->nMu * bsdfTable->nMu];
    bsdfTable->m = new int[bsdfTable->nMu * bsdfTable->nMu];
    bsdfTable->a = new Float[nCoeffs];

    if (!readfloat(bsdfTable->mu, bsdfTable->nMu) ||
        !readfloat(bsdfTable->cdf, bsdfTable->nMu * bsdfTable->nMu) ||
        !read32(offsetAndLength, bsdfTable->nMu * bsdfTable->nMu * 2) ||
        !readfloat(bsdfTable->a, nCoeffs))
        goto fail;

    for (int i = 0; i < bsdfTable->nMu * bsdfTable->nMu; ++i) {
        int offset = offsetAndLength[2 * i],
            length = offsetAndLength[2 * i + 1];

        bsdfTable->aOffset[i] = offset;
        bsdfTable->m[i] = length;

        bsdfTable->avg[i] = length > 0 ? bsdfTable->a[offset] : (Float)0;
    }
    delete[] offsetAndLength;

    bsdfTable->recip = new Float[bsdfTable->mMax];
    for (int i = 0; i < bsdfTable->mMax; ++i)
        bsdfTable->recip[i] = 1 / (Float)i;

    fclose(f);
    return true;
fail:
    fclose(f);
    Error(
        "Tabulated BSDF file \"%s\" has an incompatible file format or "
        "version.",
        filename.c_str());
    return false;
}

Spectrum FourierBSDF::f(const Vector3f &wo, const Vector3f &wi) const {
    // Find the zenith angle cosines and azimuth difference angle
    Float cosThetaI = CosTheta(-wi), cosThetaO = CosTheta(wo);
    Float cosPhi = CosDPhi(-wi, wo);

    // Compute Fourier coefficients $a_k$ for $(\mui, \muo)$

    // Determine offsets and weights for $\mui$ and $\muo$
    int offsetI, offsetO;
    Float weightsI[4], weightsO[4];
    if (!bsdfTable.GetWeightsAndOffset(cosThetaI, &offsetI, weightsI) ||
        !bsdfTable.GetWeightsAndOffset(cosThetaO, &offsetO, weightsO))
        return Spectrum(0.f);

    // Allocate storage to accumulate _ak_ coefficients
    Float *ak = ALLOCA(Float, bsdfTable.mMax * bsdfTable.nChannels);
    memset(ak, 0, bsdfTable.mMax * bsdfTable.nChannels * sizeof(Float));

    // Accumulate weighted sums of nearby $a_k$ coefficients
    int mMax = 0;
    for (int b = 0; b < 4; ++b) {
        for (int a = 0; a < 4; ++a) {
            // Add contribution of _(a, b)_ to $a_k$ values
            Float weight = weightsI[a] * weightsO[b];
            if (weight != 0.) {
                int m;
                const Float *ap = bsdfTable.GetAk(offsetI + a, offsetO + b, &m);
                mMax = std::max(mMax, m);
                for (int c = 0; c < bsdfTable.nChannels; ++c)
                    for (int k = 0; k < m; ++k)
                        ak[c * bsdfTable.mMax + k] += weight * ap[c * m + k];
            }
        }
    }

    // Evaluate Fourier expansion for angle $\phi$
    Float Y = std::max((Float)0, Fourier(ak, mMax, cosPhi));
    Float scale = cosThetaI != 0 ? (1 / std::abs(cosThetaI)) : (Float)0;

    // Update _scale_ to account for adjoint light transport
    if (mode == TransportMode::Radiance && cosThetaI * cosThetaO > 0) {
        float eta = cosThetaI > 0 ? 1 / bsdfTable.eta : bsdfTable.eta;
        scale *= eta * eta;
    }
    if (bsdfTable.nChannels == 1)
        return Spectrum(Y * scale);
    else {
        // Compute and return RGB colors for tabulated BSDF
        Float R = Fourier(ak + 1 * bsdfTable.mMax, mMax, cosPhi);
        Float B = Fourier(ak + 2 * bsdfTable.mMax, mMax, cosPhi);
        Float G = 1.39829f * Y - 0.100913f * B - 0.297375f * R;
        Float rgb[3] = {R * scale, G * scale, B * scale};
        return Spectrum::FromRGB(rgb).Clamp();
    }
}

bool FourierBSDFTable::GetWeightsAndOffset(Float cosTheta, int *offset,
                                           Float weights[4]) const {
    return CatmullRomWeights(nMu, mu, cosTheta, offset, weights);
}

Spectrum BxDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample,
                        Float *pdf, BxDFType *sampledType) const {
    // Cosine-sample the hemisphere, flipping the direction if necessary
    *wi = CosineSampleHemisphere(sample);
    if (wo.z < 0.) wi->z *= -1.f;
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float BxDF::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    return SameHemisphere(wo, wi) ? AbsCosTheta(wi) * InvPi : 0.f;
}

Spectrum LambertianTransmission::Sample_f(const Vector3f &wo, Vector3f *wi,
                                          const Point2f &sample, Float *pdf,
                                          BxDFType *sampledType) const {
    *wi = CosineSampleHemisphere(sample);
    if (wo.z > 0.f) wi->z *= -1.f;
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float LambertianTransmission::Pdf(const Vector3f &wo,
                                  const Vector3f &wi) const {
    return !SameHemisphere(wo, wi) ? AbsCosTheta(wi) * InvPi : 0.f;
}

Spectrum MicrofacetReflection::Sample_f(const Vector3f &wo, Vector3f *wi,
                                        const Point2f &sample, Float *pdf,
                                        BxDFType *sampledType) const {
    Vector3f wh = distribution->Sample_wh(wo, sample);
    *wi = Reflect(wo, wh);
    if (!SameHemisphere(wo, *wi)) return Spectrum(0.f);
    *pdf = distribution->Pdf(wo, *wi, wh) / (4.f * Dot(wo, wh));
    return f(wo, *wi);
}

Float MicrofacetReflection::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (!SameHemisphere(wo, wi)) return 0.f;
    Vector3f wh = Normalize(wo + wi);
    return distribution->Pdf(wo, wi, wh) / (4.f * Dot(wo, wh));
}

Spectrum MicrofacetTransmission::Sample_f(const Vector3f &wo, Vector3f *wi,
                                          const Point2f &sample, Float *pdf,
                                          BxDFType *sampledType) const {
    Vector3f wh = distribution->Sample_wh(wo, sample);
    Float eta = CosTheta(wo) > 0 ? (etaExterior / etaInterior)
                                 : (etaInterior / etaExterior);
    if (!Refract(wo, (Normal3f)wh, eta, wi)) return 0.f;  // TIR

    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float MicrofacetTransmission::Pdf(const Vector3f &wo,
                                  const Vector3f &wi) const {
    if (SameHemisphere(wo, wi)) return 0.f;

    // Compute $\wh$ from $\wo$ and $\wi$ for microfacet transmission
    Float eta = CosTheta(wo) > 0 ? (etaInterior / etaExterior)
                                 : (etaExterior / etaInterior);
    Vector3f wh = Normalize(wo + wi * eta);

    /* Jacobian of the half-direction mapping */
    Float sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
    Float dwh_dwi = (eta * eta * Dot(wi, wh)) / (sqrtDenom * sqrtDenom);

    Float pdf_wh = distribution->Pdf(wo, wi, wh);
    return std::abs(pdf_wh * dwh_dwi);
}

Spectrum FresnelBlend::Sample_f(const Vector3f &wo, Vector3f *wi,
                                const Point2f &origSample, Float *pdf,
                                BxDFType *sampledType) const {
    Point2f sample = origSample;
    if (sample.x < .5) {
        sample.x = 2.f * sample.x;
        // Cosine-sample the hemisphere, flipping the direction if necessary
        *wi = CosineSampleHemisphere(sample);
        if (wo.z < 0.) wi->z *= -1.f;
    } else {
        sample.x = 2.f * (sample.x - .5f);
        Vector3f wh = distribution->Sample_wh(wo, sample);
        *wi = Reflect(wo, wh);
        if (!SameHemisphere(wo, *wi)) return Spectrum(0.f);
    }
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float FresnelBlend::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (!SameHemisphere(wo, wi)) return 0.f;
    Vector3f wh = Normalize(wo + wi);
    Float pdf_wh = distribution->Pdf(wo, wi, wh);
    return .5f * (AbsCosTheta(wi) * InvPi + pdf_wh / (4. * Dot(wo, wh)));
}

Spectrum FresnelSpecular::Sample_f(const Vector3f &wo, Vector3f *wi,
                                   const Point2f &sample, Float *pdf,
                                   BxDFType *sampledType) const {
    Float F = FrDielectric(CosTheta(wo), etaa, etab);
    if (sample.x < F) {
        // Compute perfect specular reflection direction
        *wi = Vector3f(-wo.x, -wo.y, wo.z);
        if (sampledType)
            *sampledType = BxDFType(BSDF_SPECULAR | BSDF_REFLECTION);
        *pdf = F;
        return F * R / AbsCosTheta(*wi);
    } else {
        // Figure out which $\eta$ is incident and which is transmitted
        bool entering = CosTheta(wo) > 0.f;
        Float etai = entering ? etaa : etab;
        Float etat = entering ? etab : etaa;

        // Compute ray direction for specular transmission
        if (!Refract(wo, Faceforward(Normal3f(0, 0, 1), wo), etai / etat, wi))
            return 0.;
        Spectrum ft = T * (1. - F);
        // Account for non-symmetry with transmission to different medium
        if (mode == TransportMode::Radiance)
            ft *= (etai * etai) / (etat * etat);
        if (sampledType)
            *sampledType = BxDFType(BSDF_SPECULAR | BSDF_TRANSMISSION);
        *pdf = 1. - F;
        return ft / AbsCosTheta(*wi);
    }
}

Float FourierBSDF::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    // Find the zenith angle cosines and azimuth difference angle
    Float cosThetaI = CosTheta(-wi), cosThetaO = CosTheta(wo);
    Float cosPhi = CosDPhi(-wi, wo);

    // Compute luminance Fourier coefficients $a_k$ for $(\mui, \muo)$
    int offsetI, offsetO;
    Float weightsI[4], weightsO[4];
    if (!bsdfTable.GetWeightsAndOffset(cosThetaI, &offsetI, weightsI) ||
        !bsdfTable.GetWeightsAndOffset(cosThetaO, &offsetO, weightsO))
        return 0.f;
    Float *ak = ALLOCA(Float, bsdfTable.mMax * bsdfTable.nChannels);
    memset(ak, 0, bsdfTable.mMax * bsdfTable.nChannels * sizeof(Float));
    int mMax = 0;
    for (int o = 0; o < 4; ++o) {
        for (int i = 0; i < 4; ++i) {
            Float weight = weightsI[i] * weightsO[o];
            if (weight == 0) continue;

            int order;
            const Float *coeffs =
                bsdfTable.GetAk(offsetI + i, offsetO + o, &order);
            mMax = std::max(mMax, order);

            for (int k = 0; k < order; ++k) ak[k] += *coeffs++ * weight;
        }
    }

    // Evaluate probability of sampling _wi_
    Float total = 0.0f;
    for (int i = 0; i < 4; ++i) {
        if (weightsO[i] == 0) continue;
        total +=
            weightsO[i] *
            bsdfTable.cdf[(offsetO + i) * bsdfTable.nMu + bsdfTable.nMu - 1];
    }
    Float value = std::max((Float)0, Fourier(ak, mMax, cosPhi));
    return total > 0.f ? (value / (2 * Pi * total)) : 0.f;
}

Spectrum FourierBSDF::Sample_f(const Vector3f &wo, Vector3f *wi,
                               const Point2f &sample, Float *pdf,
                               BxDFType *sampledType) const {
    // Sample zenith angle component
    Float cosThetaO = CosTheta(wo);
    Float pdfMu;
    Float cosThetaI = SampleCatmullRom2D(
        bsdfTable.nMu, bsdfTable.nMu, bsdfTable.mu, bsdfTable.mu, bsdfTable.avg,
        bsdfTable.cdf, cosThetaO, sample[1], nullptr, &pdfMu);

    // Compute Fourier coefficients $a_k$ for $(\mui, \muo)$

    // Determine offsets and weights for $\mui$ and $\muo$
    int offsetI, offsetO;
    Float weightsI[4], weightsO[4];
    if (!bsdfTable.GetWeightsAndOffset(cosThetaI, &offsetI, weightsI) ||
        !bsdfTable.GetWeightsAndOffset(cosThetaO, &offsetO, weightsO))
        return Spectrum(0.f);

    // Allocate storage to accumulate _ak_ coefficients
    Float *ak = ALLOCA(Float, bsdfTable.mMax * bsdfTable.nChannels);
    memset(ak, 0, bsdfTable.mMax * bsdfTable.nChannels * sizeof(Float));

    // Accumulate weighted sums of nearby $a_k$ coefficients
    int mMax = 0;
    for (int b = 0; b < 4; ++b) {
        for (int a = 0; a < 4; ++a) {
            // Add contribution of _(a, b)_ to $a_k$ values
            Float weight = weightsI[a] * weightsO[b];
            if (weight != 0.) {
                int m;
                const Float *ap = bsdfTable.GetAk(offsetI + a, offsetO + b, &m);
                mMax = std::max(mMax, m);
                for (int c = 0; c < bsdfTable.nChannels; ++c)
                    for (int k = 0; k < m; ++k)
                        ak[c * bsdfTable.mMax + k] += weight * ap[c * m + k];
            }
        }
    }

    // Importance sample the luminance Fourier expansion
    Float phi, pdfPhi;
    Float Y =
        SampleFourier(ak, bsdfTable.recip, mMax, sample[0], &pdfPhi, &phi);
    *pdf = std::max((Float)0, pdfPhi * pdfMu);

    // Compute the scattered direction
    Float norm = std::sqrt((1 - cosThetaI * cosThetaI) /
                           (wo.x * wo.x + wo.y * wo.y)),
          sinPhiD = std::sin(phi), cosPhiD = std::cos(phi);
    *wi = -Vector3f(norm * (cosPhiD * wo.x - sinPhiD * wo.y),
                    norm * (sinPhiD * wo.x + cosPhiD * wo.y), cosThetaI);

    // Evaluate remaining Fourier expansions for angle $\phi$
    Float scale = cosThetaI != 0 ? (1 / std::abs(cosThetaI)) : (Float)0;
    if (mode == TransportMode::Radiance && cosThetaI * cosThetaO > 0) {
        float eta = cosThetaI > 0 ? 1 / bsdfTable.eta : bsdfTable.eta;
        scale *= eta * eta;
    }

    if (bsdfTable.nChannels == 1) return Spectrum(Y * scale);
    Float cosPhi = std::cos(phi);
    Float R = Fourier(ak + 1 * bsdfTable.mMax, mMax, cosPhi);
    Float B = Fourier(ak + 2 * bsdfTable.mMax, mMax, cosPhi);
    Float G = 1.39829f * Y - 0.100913f * B - 0.297375f * R;
    Float rgb[3] = {R * scale, G * scale, B * scale};
    return Spectrum::FromRGB(rgb).Clamp();
}

Spectrum BxDF::rho(const Vector3f &w, int nSamples,
                   const Point2f *samples) const {
    Spectrum r = 0.;
    for (int i = 0; i < nSamples; ++i) {
        // Estimate one term of $\rho_\roman{hd}$
        Vector3f wi;
        Float pdf = 0.f;
        Spectrum f = Sample_f(w, &wi, samples[i], &pdf);
        if (pdf > 0.f) r += f * AbsCosTheta(wi) / pdf;
    }
    return r / Float(nSamples);
}

Spectrum BxDF::rho(int nSamples, const Point2f *samples1,
                   const Point2f *samples2) const {
    Spectrum r = 0.f;
    for (int i = 0; i < nSamples; ++i) {
        // Estimate one term of $\rho_\roman{hh}$
        Vector3f wo, wi;
        wo = UniformSampleHemisphere(samples1[i]);
        Float pdf_o = Inv2Pi, pdf_i = 0.f;
        Spectrum f = Sample_f(wo, &wi, samples2[i], &pdf_i);
        if (pdf_i > 0.f)
            r += f * AbsCosTheta(wi) * AbsCosTheta(wo) / (pdf_o * pdf_i);
    }
    return r / (Pi * nSamples);
}

// BSDF Method Definitions
Spectrum BSDF::f(const Vector3f &woW, const Vector3f &wiW,
                 BxDFType flags) const {
    Vector3f wi = WorldToLocal(wiW), wo = WorldToLocal(woW);
    bool reflect = Dot(wiW, ng) * Dot(woW, ng) > 0;
    Spectrum f = 0.f;
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(flags) &&
            ((reflect && (bxdfs[i]->type & BSDF_REFLECTION)) ||
             (!reflect && (bxdfs[i]->type & BSDF_TRANSMISSION))))
            f += bxdfs[i]->f(wo, wi);
    return f;
}

Spectrum BSDF::rho(RNG &rng, int nSamples, const Point2f *samples1,
                   const Point2f *samples2, BxDFType flags) const {
    Spectrum ret(0.f);
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(flags))
            ret += bxdfs[i]->rho(nSamples, samples1, samples2);
    return ret;
}

Spectrum BSDF::rho(const Vector3f &wo, int nSamples, const Point2f *samples,
                   BxDFType flags) const {
    Spectrum ret(0.f);
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(flags))
            ret += bxdfs[i]->rho(wo, nSamples, samples);
    return ret;
}

Spectrum BSDF::Sample_f(const Vector3f &woW, Vector3f *wiW,
                        const Point2f &sample, Float *pdf, BxDFType flags,
                        BxDFType *sampledType) const {
    // Choose which _BxDF_ to sample
    int matchingComps = NumComponents(flags);
    if (matchingComps == 0) {
        *pdf = 0.f;
        if (sampledType) *sampledType = BxDFType(0);
        return Spectrum(0.f);
    }
    int which =
        std::min((int)std::floor(sample.x * matchingComps), matchingComps - 1);
    Point2f remappedSample(sample.x * matchingComps - which, sample.y);
    BxDF *bxdf = nullptr;
    int count = which;
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(flags) && count-- == 0) {
            bxdf = bxdfs[i];
            break;
        }
    Assert(bxdf);

    // Sample chosen _BxDF_
    Vector3f wo = WorldToLocal(woW);
    Vector3f wi;
    *pdf = 0.f;
    if (sampledType) *sampledType = bxdf->type;
    Spectrum f = bxdf->Sample_f(wo, &wi, remappedSample, pdf, sampledType);
    if (*pdf == 0.f) {
        if (sampledType) *sampledType = BxDFType(0);
        return 0.f;
    }
    *wiW = LocalToWorld(wi);

    // Compute overall PDF with all matching _BxDF_s
    if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1)
        for (int i = 0; i < nBxDFs; ++i)
            if (bxdfs[i] != bxdf && bxdfs[i]->MatchesFlags(flags))
                *pdf += bxdfs[i]->Pdf(wo, wi);
    if (matchingComps > 1) *pdf /= matchingComps;

    // Compute value of BSDF for sampled direction
    if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1) {
        bool reflect = Dot(*wiW, ng) * Dot(woW, ng) > 0;
        f = 0.;
        for (int i = 0; i < nBxDFs; ++i)
            if (bxdfs[i]->MatchesFlags(flags) &&
                ((reflect && (bxdfs[i]->type & BSDF_REFLECTION)) ||
                 (!reflect && (bxdfs[i]->type & BSDF_TRANSMISSION))))
                f += bxdfs[i]->f(wo, wi);
    }
    return f;
}

Float BSDF::Pdf(const Vector3f &woW, const Vector3f &wiW,
                BxDFType flags) const {
    if (nBxDFs == 0.f) return 0.f;
    Vector3f wo = WorldToLocal(woW), wi = WorldToLocal(wiW);
    Float pdf = 0.f;
    int matchingComps = 0;
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(flags)) {
            ++matchingComps;
            pdf += bxdfs[i]->Pdf(wo, wi);
        }
    Float v = matchingComps > 0 ? pdf / matchingComps : 0.f;
    return v;
}

// BSSRDF Utility Functions
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
    Float c_phi = 0.25f * (1 - 2 * fm1), c_E = 0.5f * (1 - 3 * fm2);
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

Float BeamDiffusionSS(Float sig_s, Float sig_a, Float g, Float eta, Float r) {
    Float sig_t = sig_a + sig_s, alpha = sig_s / sig_t;
    // Find minimum $t$ below the critical angle
    Float critT = r * std::sqrt(1 - 1 / (eta * eta)) * eta;
    Float integral = 0.0f;
    const int nSamples = 100;
    for (int i = 0; i < nSamples; ++i) {
        // Evaluate single scattering integrand and add to _integral_
        Float t = critT - std::log(1 - (i + .5f) / nSamples) / sig_t,
              d = std::sqrt(r * r + t * t);

        // Determine angle cosines at the scattering and exit points
        Float cosThetaS = 1 / std::sqrt(1 + (r * r) / (t * t));
        Float cosThetaE = t / d;

        integral += alpha * PhaseHG(cosThetaS, g) * std::exp(-sig_t * d) *
                    cosThetaE / (d * d) *
                    (1 - FrDielectric(-cosThetaE, 1, eta));
    }
    return integral * std::exp(-critT * sig_t) / nSamples;
}

void ComputeBeamDiffusionBSSRDF(Float g, Float eta, BSSRDFTable *t) {
    // Choose radii of the diffusion profile disretization
    t->radiusSamples[0] = 0;
    t->radiusSamples[1] = 2.5e-3f;
    for (int i = 2; i < t->nRadiusSamples; ++i)
        t->radiusSamples[i] = t->radiusSamples[i - 1] * 1.2f;
#if defined(PBRT_IS_MSVC)
	// VS2015_mwkm: ParallelFor ambiguous call
	ParallelFor( (const std::function<void(int)>)[&](int i) {
#else
	ParallelFor([&](int i) {
#endif
		// Compute the _i_-th diffusion profile
        Float albedo = (1 - std::exp(-8 * i / (Float)(t->nAlbedoSamples - 1))) /
                       (1 - std::exp(-8));

        // Compute profile for chosen _albedo_
        t->profile[i * t->nRadiusSamples] = 0.0f;
        for (int j = 0; j < t->nRadiusSamples; ++j)
            t->profile[i * t->nRadiusSamples + j] =
                2 * Pi * t->radiusSamples[j] *
                (BeamDiffusionSS(albedo, 1 - albedo, g, eta,
                                 t->radiusSamples[j]) +
                 BeamDiffusionMS(albedo, 1 - albedo, g, eta,
                                 t->radiusSamples[j]));

        // Compute multiple scattering albedo and importance sampling CDF
        t->albedoSamples[i] = albedo;
        t->profileAlbedo[i] =
            IntegrateCatmullRom(t->nRadiusSamples, t->radiusSamples.get(),
                                &t->profile[i * t->nRadiusSamples],
                                &t->profileCDF[i * t->nRadiusSamples]);
    }, t->nAlbedoSamples);
}

void SubsurfaceFromDiffuse(const BSSRDFTable &table, const Spectrum &Kd,
                           const Spectrum &sigma_t, Spectrum *sigma_a,
                           Spectrum *sigma_s) {
    for (int c = 0; c < Spectrum::nSamples; ++c) {
        Float ssAlbedo =
            InvertCatmullRom(table.nAlbedoSamples, table.albedoSamples.get(),
                             table.profileAlbedo.get(), Kd[c]);
        (*sigma_s)[c] = ssAlbedo * sigma_t[c];
        (*sigma_a)[c] = (1 - ssAlbedo) * sigma_t[c];
    }
}

// BSSRDF Method Definitions
Spectrum TabulatedBSSRDF::f(const SurfaceInteraction &isect_out,
                            const SurfaceInteraction &isect_in) const {
    Spectrum fs = 0.f;
    for (int c = 0; c < Spectrum::nSamples; ++c) {
        // Compute BSSRDF sample weights for spectrum component _c_
        Float distance = Distance(isect_out.p, isect_in.p);
        int albedoOffset, radiusOffset;
        Float albedoWeights[4], radiusWeights[4];
        if (!CatmullRomWeights(table.nAlbedoSamples, table.albedoSamples.get(),
                               albedo[c], &albedoOffset, albedoWeights) ||
            !CatmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(),
                               distance * sigma_t[c], &radiusOffset,
                               radiusWeights))
            continue;

        // Compute weighted BSSRDF value using weights
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                Float weight = albedoWeights[i] * radiusWeights[j];
                if (weight != 0)
                    fs[c] +=
                        table.evalProfile(albedoOffset + i, radiusOffset + j) *
                        weight;
            }
    }
    return (fs * sigma_t).Clamp();
}

bool IntersectFacing(const Scene &scene, Ray ray, const Vector3f &axis,
                     SurfaceInteraction *its) {
    while (scene.Intersect(ray, its)) {
        if (Dot(its->n, axis) > 0) return true;
        ray = its->SpawnRay(ray.d);
    }
    return false;
}

Spectrum TabulatedBSSRDF::Sample_f(const SurfaceInteraction &isect,
                                   const Scene &scene, Float time,
                                   const BSSRDFSample &bssrdfSample,
                                   MemoryArena &arena,
                                   SurfaceInteraction *isect_in,
                                   Float *pdf) const {
    Vector3f axis;

    bool offsetRayOrigin = false;
    Float discrSample = bssrdfSample.uDiscrete;
    if (discrSample < .25f) {
        axis = sn;
        discrSample *= 4;
    } else if (discrSample < .5f) {
        axis = tn;
        discrSample = (discrSample - .25f) * 4.f;
    } else {
        axis = Vector3f(nn);
        discrSample = (discrSample - .5f) * 2.f;
        offsetRayOrigin = true;
    }

    if (discrSample < .5f) {
        discrSample *= 2.f;
    } else {
        discrSample = 2.f * (discrSample - .5f);
        axis = -axis;
    }

    int ch = Clamp((int)(discrSample * Spectrum::nSamples), 0,
                   Spectrum::nSamples - 1);
    if (sigma_t[ch] == 0) return Spectrum(0.f);

    Float phi = 2 * Pi * bssrdfSample.pos.x;
    Float r =
        SampleCatmullRom2D(table.nAlbedoSamples, table.nRadiusSamples,
                           table.albedoSamples.get(), table.radiusSamples.get(),
                           table.profile.get(), table.profileCDF.get(),
                           albedo[ch], bssrdfSample.pos.y) /
        sigma_t[ch];

    Vector3f v0, v1;
    CoordinateSystem(axis, &v0, &v1);
    Point3f origin = isect.p + (v0 * std::cos(phi) + v1 * std::sin(phi)) * r;

    if (offsetRayOrigin)
        origin = OffsetRayOrigin(origin, isect.pError * 10, isect.n, axis);

    Ray ray1(origin, axis, Infinity, time);
    Ray ray2(origin, -axis, Infinity, time);

    SurfaceInteraction its1, its2;
    bool its1Valid = IntersectFacing(scene, ray1, axis, &its1);
    bool its2Valid = IntersectFacing(scene, ray2, axis, &its2);
    its1Valid = its1Valid && its1.primitive->GetMaterial() == material;
    its2Valid = its2Valid && its2.primitive->GetMaterial() == material;

    SurfaceInteraction *its;
    if (its1Valid && its2Valid)
        its = (its1.p - origin).LengthSquared() <
                      (its2.p - origin).LengthSquared()
                  ? &its1
                  : &its2;
    else if (its1Valid)
        its = &its1;
    else if (its2Valid)
        its = &its2;
    else
        return Spectrum(0.0f);

    // XXX is this still needed? (we just need the shading frame to be computed
    // for this intersection)
    its->ComputeScatteringFunctions(RayDifferential(its == &its1 ? ray1 : ray2),
                                    arena, true);

    its->bsdf = ARENA_ALLOC(arena, BSDF)(*its);
    its->bsdf->Add(ARENA_ALLOC(arena, LambertianReflection)(Spectrum(1.0f)));

    *pdf = Pdf(isect, *its);
    *isect_in = *its;

    if (*pdf == 0.f) return Spectrum(0.f);

    return f(isect, *its) *
           (mode == TransportMode::Radiance ? (eta * eta) : 1.f) / *pdf;
}

Float TabulatedBSSRDF::Pdf(Float distance) const {
    Float result = 0.f;
    for (int c = 0; c < Spectrum::nSamples; ++c) {
        int albedoOffset, radiusOffset;
        Float albedoWeights[4], radiusWeights[4];

        if (!CatmullRomWeights(table.nAlbedoSamples, table.albedoSamples.get(),
                               albedo[c], &albedoOffset, albedoWeights) ||
            !CatmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(),
                               distance * sigma_t[c], &radiusOffset,
                               radiusWeights))
            continue;

        Float lookup = 0.f, finalAlbedo = 0;
        for (int i = 0; i < 4; ++i) {
            if (albedoWeights[i] == 0) continue;
            finalAlbedo +=
                table.profileAlbedo[albedoOffset + i] * albedoWeights[i];

            for (int j = 0; j < 4; ++j) {
                if (radiusWeights[j] == 0) continue;
                lookup +=
                    table.evalProfile(albedoOffset + i, radiusOffset + j) *
                    albedoWeights[i] * radiusWeights[j];
            }
        }

        result += std::max((Float)0, lookup * sigma_t[c] / finalAlbedo);
    }

    return result * (1.f / Spectrum::nSamples);
}

Float TabulatedBSSRDF::Pdf(const SurfaceInteraction &isect,
                           const SurfaceInteraction &isect_in) const {
    Vector3f d = isect.p - isect_in.p;
    Vector3f dlocal = Vector3f(Dot(sn, d), Dot(tn, d), Dot(nn, d));
    Normal3f nlocal(Dot(sn, isect_in.n), Dot(tn, isect_in.n),
                    Dot(nn, isect_in.n));

    Float axisProb[3] = {.25f, .25f, .5f};
    Float distSqr[3] = {dlocal.y * dlocal.y + dlocal.z * dlocal.z,
                        dlocal.z * dlocal.z + dlocal.x * dlocal.x,
                        dlocal.x * dlocal.x + dlocal.y * dlocal.y};

    Float result = 0;
    for (int i = 0; i < 3; ++i)
        result += 0.5f * Pdf(std::sqrt(distSqr[i])) * axisProb[i] *
                  std::abs(nlocal[i]);
    return result;
}
