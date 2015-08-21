
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
#include "stats.h"
#include <stdarg.h>

// BxDF Utility Functions
Float FrDielectric(Float cosThetaI, Float etaI, Float etaT) {
    cosThetaI = Clamp(cosThetaI, -1, 1);
    // Potentially swap indices of refraction
    bool entering = cosThetaI > 0.f;
    if (!entering) {
        std::swap(etaI, etaT);
        cosThetaI = std::abs(cosThetaI);
    }

    // Compute _cosThetaT_ using Snell's law
    Float sinThetaI = std::sqrt(std::max((Float)0, 1 - cosThetaI * cosThetaI));
    Float sinThetaT = etaI / etaT * sinThetaI;

    // Handle total internal reflection
    if (sinThetaT >= 1) return 1;
    Float cosThetaT = std::sqrt(std::max((Float)0, 1 - sinThetaT * sinThetaT));
    Float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
                  ((etaT * cosThetaI) + (etaI * cosThetaT));
    Float Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
                  ((etaI * cosThetaI) + (etaT * cosThetaT));
    return (Rparl * Rparl + Rperp * Rperp) / 2;
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
    return FrConductor(std::abs(cosThetaI), etaI, etaT, k);
}

Spectrum FresnelDielectric::Evaluate(Float cosThetaI) const {
    return FrDielectric(cosThetaI, etaI, etaT);
}

Spectrum SpecularReflection::Sample_f(const Vector3f &wo, Vector3f *wi,
                                      const Point2f &sample, Float *pdf,
                                      BxDFType *sampledType) const {
    // Compute perfect specular reflection direction
    *wi = Vector3f(-wo.x, -wo.y, wo.z);
    *pdf = 1;
    return fresnel->Evaluate(CosTheta(*wi)) * R / AbsCosTheta(*wi);
}

Spectrum SpecularTransmission::Sample_f(const Vector3f &wo, Vector3f *wi,
                                        const Point2f &sample, Float *pdf,
                                        BxDFType *sampledType) const {
    // Figure out which $\eta$ is incident and which is transmitted
    bool entering = CosTheta(wo) > 0;
    Float etaI = entering ? etaA : etaB;
    Float etaT = entering ? etaB : etaA;

    // Compute ray direction for specular transmission
    if (!Refract(wo, Faceforward(Normal3f(0, 0, 1), wo), etaI / etaT, wi))
        return 0;
    *pdf = 1;
    Spectrum ft = T * (Spectrum(1.) - fresnel.Evaluate(CosTheta(*wi)));
    // Account for non-symmetry with transmission to different medium
    if (mode == TransportMode::Radiance) ft *= (etaI * etaI) / (etaT * etaT);
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
    Float maxCos = 0;
    if (sinThetaI > 1e-4 && sinThetaO > 1e-4) {
        Float sinPhiI = SinPhi(wi), cosPhiI = CosPhi(wi);
        Float sinPhiO = SinPhi(wo), cosPhiO = CosPhi(wo);
        Float dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
        maxCos = std::max((Float)0, dCos);
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
    Vector3f wh = wi + wo;
    // Handle degenerate cases for microfacet reflection
    if (cosThetaI == 0 || cosThetaO == 0) return Spectrum(0.);
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0.);
    wh = Normalize(wh);
    Spectrum F = fresnel->Evaluate(Dot(wi, wh));
    return R * distribution->D(wh) * distribution->G(wo, wi) * F /
           (4 * cosThetaI * cosThetaO);
}

Spectrum MicrofacetTransmission::f(const Vector3f &wo,
                                   const Vector3f &wi) const {
    if (SameHemisphere(wo, wi)) return 0;  // transmission only

    Float cosThetaO = CosTheta(wo);
    Float cosThetaI = CosTheta(wi);
    if (cosThetaI == 0 || cosThetaO == 0) return Spectrum(0);

    // Compute $\wh$ from $\wo$ and $\wi$ for microfacet transmission
    Float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    Vector3f wh = Normalize(wo + wi * eta);
    if (wh.z < 0) wh = -wh;

    Spectrum F = fresnel.Evaluate(Dot(wo, wh));

    Float sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
    Float factor = (mode == TransportMode::Radiance) ? (1 / eta) : 1;

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
    auto pow5 = [](Float v) { return (v * v) * (v * v) * v; };
    Spectrum diffuse = (28.f / (23.f * Pi)) * Rd * (Spectrum(1.f) - Rs) *
                       (1 - pow5(1 - .5f * AbsCosTheta(wi))) *
                       (1 - pow5(1 - .5f * AbsCosTheta(wo)));
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0);
    wh = Normalize(wh);
    Spectrum specular =
        distribution->D(wh) /
        (4 * AbsDot(wi, wh) * std::max(AbsCosTheta(wi), AbsCosTheta(wo))) *
        SchlickFresnel(Dot(wi, wh));
    return diffuse + specular;
}

Spectrum KajiyaKay::f(const Vector3f &wo, const Vector3f &wi) const {
    Spectrum diffuse(0.f), specular(0.f);
    if (!Ks.IsBlack()) {
        // Compute specular Kajiya-Kay term
        Vector3f wh = wi + wo;
        if (!(wh.x == 0 && wh.y == 0 && wh.z == 0)) {
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
            if (weight != 0) {
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
    Float eta = CosTheta(wo) > 0 ? (etaA / etaB) : (etaB / etaA);
    if (!Refract(wo, (Normal3f)wh, eta, wi)) return 0;
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float MicrofacetTransmission::Pdf(const Vector3f &wo,
                                  const Vector3f &wi) const {
    if (SameHemisphere(wo, wi)) return 0.f;
    // Compute $\wh$ from $\wo$ and $\wi$ for microfacet transmission
    Float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
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
    Float F = FrDielectric(CosTheta(wo), etaA, etaB);
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
        bool entering = CosTheta(wo) > 0;
        Float etaI = entering ? etaA : etaB;
        Float etaT = entering ? etaB : etaA;

        // Compute ray direction for specular transmission
        if (!Refract(wo, Faceforward(Normal3f(0, 0, 1), wo), etaI / etaT, wi))
            return 0;
        Spectrum ft = T * (1 - F);

        // Account for non-symmetry with transmission to different medium
        if (mode == TransportMode::Radiance)
            ft *= (etaI * etaI) / (etaT * etaT);
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
            if (weight != 0) {
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
    ProfilePhase pp(Prof::BSDFEvaluation);
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
    ProfilePhase pp(Prof::BSDFEvaluation);
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
