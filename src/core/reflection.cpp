
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
            Float cosThetaH = Dot(wo, wh);
            Float sinThetaH = std::sqrt(std::max((Float)0, (Float)1 - cosThetaH * cosThetaH));
            Float cosThetaO = CosTheta(wo), sinThetaO = SinTheta(wo);
            Float spec = std::pow(cosThetao * cosThetah + sinThetaO * sinThetaH,
                                  exponent);
#else
            Float tdoth = wh.x;
            Float spec = std::pow(
                std::sqrt(std::max((Float)0, (Float)1 - tdoth * tdoth)),
                exponent);
#endif
            specular = spec * Ks;
        }
    }
    // Compute diffuse Kajiya-Kay term
    diffuse = Kd * std::sqrt(std::max((Float)0., (Float)1. - wi.x * wi.x));
    return (InvPi / AbsCosTheta(wi)) * (diffuse + specular);
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

Spectrum BxDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                        Float *pdf, BxDFType *sampledType) const {
    // Cosine-sample the hemisphere, flipping the direction if necessary
    *wi = CosineSampleHemisphere(u);
    if (wo.z < 0.) wi->z *= -1.f;
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float BxDF::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    return SameHemisphere(wo, wi) ? AbsCosTheta(wi) * InvPi : 0.f;
}

Spectrum LambertianTransmission::Sample_f(const Vector3f &wo, Vector3f *wi,
                                          const Point2f &u, Float *pdf,
                                          BxDFType *sampledType) const {
    *wi = CosineSampleHemisphere(u);
    if (wo.z > 0.f) wi->z *= -1.f;
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float LambertianTransmission::Pdf(const Vector3f &wo,
                                  const Vector3f &wi) const {
    return !SameHemisphere(wo, wi) ? AbsCosTheta(wi) * InvPi : 0.f;
}

Spectrum MicrofacetReflection::Sample_f(const Vector3f &wo, Vector3f *wi,
                                        const Point2f &u, Float *pdf,
                                        BxDFType *sampledType) const {
    Vector3f wh = distribution->Sample_wh(wo, u);
    *wi = Reflect(wo, wh);
    if (!SameHemisphere(wo, *wi)) return Spectrum(0.f);
    // Compute PDF of _wi_ for microfacet reflection
    *pdf = distribution->Pdf(wo, wh) / (4 * Dot(wo, wh));
    return f(wo, *wi);
}

Float MicrofacetReflection::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (!SameHemisphere(wo, wi)) return 0.f;
    Vector3f wh = Normalize(wo + wi);
    return distribution->Pdf(wo, wh) / (4 * Dot(wo, wh));
}

Spectrum MicrofacetTransmission::Sample_f(const Vector3f &wo, Vector3f *wi,
                                          const Point2f &u, Float *pdf,
                                          BxDFType *sampledType) const {
    Vector3f wh = distribution->Sample_wh(wo, u);
    Float eta = CosTheta(wo) > 0 ? (etaExterior / etaInterior)
                                 : (etaInterior / etaExterior);
    if (!Refract(wo, (Normal3f)wh, eta, wi)) return 0;
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

    // Compute change of variables _dwh\_dwi_ for microfacet transmission
    Float sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
    Float dwh_dwi =
        std::abs((eta * eta * Dot(wi, wh)) / (sqrtDenom * sqrtDenom));
    return distribution->Pdf(wo, wh) * dwh_dwi;
}

Spectrum FresnelBlend::Sample_f(const Vector3f &wo, Vector3f *wi,
                                const Point2f &uOrig, Float *pdf,
                                BxDFType *sampledType) const {
    Point2f u = uOrig;
    if (u[0] < .5) {
        u[0] = 2 * u[0];
        // Cosine-sample the hemisphere, flipping the direction if necessary
        *wi = CosineSampleHemisphere(u);
        if (wo.z < 0.) wi->z *= -1.f;
    } else {
        u[0] = 2 * (u[0] - .5f);
        Vector3f wh = distribution->Sample_wh(wo, u);
        *wi = Reflect(wo, wh);
        if (!SameHemisphere(wo, *wi)) return Spectrum(0.f);
    }
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float FresnelBlend::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (!SameHemisphere(wo, wi)) return 0.f;
    Vector3f wh = Normalize(wo + wi);
    Float pdf_wh = distribution->Pdf(wo, wh);
    return .5f * (AbsCosTheta(wi) * InvPi + pdf_wh / (4 * Dot(wo, wh)));
}

Spectrum FresnelSpecular::Sample_f(const Vector3f &wo, Vector3f *wi,
                                   const Point2f &u, Float *pdf,
                                   BxDFType *sampledType) const {
    Float F = FrDielectric(CosTheta(wo), etaa, etab);
    if (u[0] < F) {
        // Compute specular reflection for _FresnelSpecular_

        // Compute perfect specular reflection direction
        *wi = Vector3f(-wo.x, -wo.y, wo.z);
        if (sampledType)
            *sampledType = BxDFType(BSDF_SPECULAR | BSDF_REFLECTION);
        *pdf = F;
        return F * R / AbsCosTheta(*wi);
    } else {
        // Compute specular transmission for _FresnelSpecular_

        // Figure out which $\eta$ is incident and which is transmitted
        bool entering = CosTheta(wo) > 0.f;
        Float etai = entering ? etaa : etab;
        Float etat = entering ? etab : etaa;

        // Compute ray direction for specular transmission
        if (!Refract(wo, Faceforward(Normal3f(0, 0, 1), wo), etai / etat, wi))
            return 0.;
        Spectrum ft = T * (1 - F);

        // Account for non-symmetry with transmission to different medium
        if (mode == TransportMode::Radiance)
            ft *= (etai * etai) / (etat * etat);
        if (sampledType)
            *sampledType = BxDFType(BSDF_SPECULAR | BSDF_TRANSMISSION);
        *pdf = 1 - F;
        return ft / AbsCosTheta(*wi);
    }
}

Spectrum FourierBSDF::Sample_f(const Vector3f &wo, Vector3f *wi,
                               const Point2f &u, Float *pdf,
                               BxDFType *sampledType) const {
    // Sample zenith angle component for _FourierBSDF_
    Float cosThetaO = CosTheta(wo);
    Float pdfMu;
    Float cosThetaI = SampleCatmullRom2D(
        bsdfTable.nMu, bsdfTable.nMu, bsdfTable.mu, bsdfTable.mu, bsdfTable.avg,
        bsdfTable.cdf, cosThetaO, u[1], nullptr, &pdfMu);

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
    Float Y = SampleFourier(ak, bsdfTable.recip, mMax, u[0], &pdfPhi, &phi);
    *pdf = std::max((Float)0, pdfPhi * pdfMu);

    // Compute the scattered direction for _FourierBSDF_
    Float sin2ThetaI = 1 - cosThetaI * cosThetaI;
    Float norm = std::sqrt(sin2ThetaI / Sin2Theta(wo));
    Float sinPhiD = std::sin(phi), cosPhiD = std::cos(phi);
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
    Float total = 0;
    for (int i = 0; i < 4; ++i)
        if (weightsO[i] != 0)
            total +=
                weightsO[i] *
                bsdfTable
                    .cdf[(offsetO + i) * bsdfTable.nMu + bsdfTable.nMu - 1];
    Float value = std::max((Float)0, Fourier(ak, mMax, cosPhi));
    return total > 0 ? (value / (2 * Pi * total)) : 0;
}

Spectrum BxDF::rho(const Vector3f &w, int nSamples, const Point2f *u) const {
    Spectrum r(0.);
    for (int i = 0; i < nSamples; ++i) {
        // Estimate one term of $\rho_\roman{hd}$
        Vector3f wi;
        Float pdf = 0.f;
        Spectrum f = Sample_f(w, &wi, u[i], &pdf);
        if (pdf > 0.f) r += f * AbsCosTheta(wi) / pdf;
    }
    return r / Float(nSamples);
}

Spectrum BxDF::rho(int nSamples, const Point2f *u1, const Point2f *u2) const {
    Spectrum r(0.f);
    for (int i = 0; i < nSamples; ++i) {
        // Estimate one term of $\rho_\roman{hh}$
        Vector3f wo, wi;
        wo = UniformSampleHemisphere(u1[i]);
        Float pdfo = UniformHemispherePdf(), pdfi = 0;
        Spectrum f = Sample_f(wo, &wi, u2[i], &pdfi);
        if (pdfi > 0)
            r += f * AbsCosTheta(wi) * AbsCosTheta(wo) / (pdfo * pdfi);
    }
    return r / (Pi * nSamples);
}

// BSDF Method Definitions
Spectrum BSDF::f(const Vector3f &woW, const Vector3f &wiW,
                 BxDFType flags) const {
    Vector3f wi = WorldToLocal(wiW), wo = WorldToLocal(woW);
    bool reflect = Dot(wiW, ng) * Dot(woW, ng) > 0;
    Spectrum f(0.f);
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

Spectrum BSDF::Sample_f(const Vector3f &woW, Vector3f *wiW, const Point2f &u,
                        Float *pdf, BxDFType type,
                        BxDFType *sampledType) const {
    // Choose which _BxDF_ to sample
    int matchingComps = NumComponents(type);
    if (matchingComps == 0) {
        *pdf = 0;
        if (sampledType) *sampledType = BxDFType(0);
        return Spectrum(0);
    }
    int comp =
        std::min((int)std::floor(u[0] * matchingComps), matchingComps - 1);

    // Get _BxDF_ pointer for chosen component
    BxDF *bxdf = nullptr;
    int count = comp;
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->MatchesFlags(type) && count-- == 0) {
            bxdf = bxdfs[i];
            break;
        }
    Assert(bxdf);

    // Sample chosen _BxDF_
    Point2f uRemapped(u[0] * matchingComps - comp, u[1]);
    Vector3f wo = WorldToLocal(woW);
    Vector3f wi;
    *pdf = 0;
    if (sampledType) *sampledType = bxdf->type;
    Spectrum f = bxdf->Sample_f(wo, &wi, uRemapped, pdf, sampledType);
    if (*pdf == 0) {
        if (sampledType) *sampledType = BxDFType(0);
        return 0;
    }
    *wiW = LocalToWorld(wi);

    // Compute overall PDF with all matching _BxDF_s
    if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1)
        for (int i = 0; i < nBxDFs; ++i)
            if (bxdfs[i] != bxdf && bxdfs[i]->MatchesFlags(type))
                *pdf += bxdfs[i]->Pdf(wo, wi);
    if (matchingComps > 1) *pdf /= matchingComps;

    // Compute value of BSDF for sampled direction
    if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1) {
        bool reflect = Dot(*wiW, ng) * Dot(woW, ng) > 0;
        f = 0.;
        for (int i = 0; i < nBxDFs; ++i)
            if (bxdfs[i]->MatchesFlags(type) &&
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
        si->wo = Vector3f(-si->shading.n);
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
    Float rMax = std::max(r, Sample_Sr(ch, 0.9999));
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
        SurfaceInteraction isect;
        IntersectionChain *next;
    };
    IntersectionChain *isects = ARENA_ALLOC(arena, IntersectionChain)(),
                      *ptr = isects;
    int nFound = 0;
    while (true) {
        if (!scene.Intersect(base.SpawnRayTo(target), &ptr->isect)) break;
        base = ptr->isect;
        // Append admissible intersection to _IntersectionChain_
        if (ptr->isect.primitive->GetMaterial() == material) {
            IntersectionChain *next = ARENA_ALLOC(arena, IntersectionChain)();
            ptr->next = next;
            ptr = next;
            nFound++;
        }
    }

    // Randomly choose one of several intersections during BSSRDF sampling
    if (nFound == 0) return Spectrum(0.0f);
    int selected = Clamp((int)(sample1 * nFound), 0, nFound - 1);
    while (selected-- > 0) isects = isects->next;
    *pi = isects->isect;

    // Compute position sampling weight and set output arguments
    *pdf = Pdf_Sp(*pi);
    Spectrum value = nFound * Sp(*pi);

    // Update BSSRDF sampling weight to account for adjoint light transport
    if (mode == TransportMode::Radiance) value *= eta * eta;
    return value;
}

Float SeparableBSSRDF::Pdf_Sp(const SurfaceInteraction &pi) const {
    Vector3f d = po.p - pi.p;
    Vector3f dLocal = Vector3f(Dot(ss, d), Dot(ts, d), Dot(ns, d));
    // Compute cosines and distances for each projection axis
    Normal3f nLocal(Dot(ss, pi.n), Dot(ts, pi.n), Dot(ns, pi.n));
    Float distSqr[3] = {dLocal.y * dLocal.y + dLocal.z * dLocal.z,
                        dLocal.z * dLocal.z + dLocal.x * dLocal.x,
                        dLocal.x * dLocal.x + dLocal.y * dLocal.y};

    // Return combined probability from all strategies
    Float result = 0, axisProb[3] = {.25f, .25f, .5f};
    for (int i = 0; i < 3; ++i)
        result +=
            Pdf_Sr(std::sqrt(distSqr[i])) * axisProb[i] * std::abs(nLocal[i]);
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

Float TabulatedBSSRDF::Pdf_Sr(Float r) const {
    Float result = 0.f;
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

        // Add PDF term due to channel _ch_
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
        result +=
            std::max((Float)0, lookup * sigma_t[ch] * sigma_t[ch] / rhoEff);
    }
    return result / Spectrum::nSamples;
}
