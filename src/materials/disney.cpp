
/*
    pbrt source code is Copyright(c) 1998-2017
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

/*

Implementation of the Disney BSDF with Subsurface Scattering, as described in:
http://blog.selfshadow.com/publications/s2015-shading-course/burley/s2015_pbs_disney_bsdf_notes.pdf.

That model is based on the Disney BRDF, described in:
https://disney-animation.s3.amazonaws.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf

Many thanks for Brent Burley and Karl Li for answering many questions about
the details of the implementation.

The initial implementation of the BRDF was adapted from
https://github.com/wdas/brdf/blob/master/src/brdfs/disney.brdf, which is
licensed under a slightly-modified Apache 2.0 license.

*/

// materials/disney.cpp*
#include "materials/disney.h"
#include "bssrdf.h"
#include "interaction.h"
#include "paramset.h"
#include "reflection.h"
#include "stats.h"
#include "stringprint.h"
#include "texture.h"
#include "rng.h"

namespace pbrt {

inline Float sqr(Float x) { return x * x; }

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
//
// The Schlick Fresnel approximation is:
//
// R = R(0) + (1 - R(0)) (1 - cos theta)^5,
//
// where R(0) is the reflectance at normal indicence.
inline Float SchlickWeight(Float cosTheta) {
    Float m = Clamp(1 - cosTheta, 0, 1);
    return (m * m) * (m * m) * m;
}

inline Float FrSchlick(Float R0, Float cosTheta) {
    return Lerp(SchlickWeight(cosTheta), R0, 1);
}

inline Spectrum FrSchlick(const Spectrum &R0, Float cosTheta) {
    return Lerp(SchlickWeight(cosTheta), R0, Spectrum(1.));
}

// For a dielectric, R(0) = (eta - 1)^2 / (eta + 1)^2, assuming we're
// coming from air..
inline Float SchlickR0FromEta(Float eta) { return sqr(eta - 1) / sqr(eta + 1); }

///////////////////////////////////////////////////////////////////////////
// DisneyDiffuse

class DisneyDiffuse : public BxDF {
  public:
    DisneyDiffuse(const Spectrum &R)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum rho(const Vector3f &, int, const Point2f *) const { return R; }
    Spectrum rho(int, const Point2f *, const Point2f *) const { return R; }
    std::string ToString() const;

  private:
    Spectrum R;
};

Spectrum DisneyDiffuse::f(const Vector3f &wo, const Vector3f &wi) const {
    Float Fo = SchlickWeight(AbsCosTheta(wo)),
          Fi = SchlickWeight(AbsCosTheta(wi));

    // Diffuse fresnel - go from 1 at normal incidence to .5 at grazing.
    // Burley 2015, eq (4).
    return R * InvPi * (1 - Fo / 2) * (1 - Fi / 2);
}

std::string DisneyDiffuse::ToString() const {
    return StringPrintf("[ DisneyDiffuse R: %s ]", R.ToString().c_str());
}

///////////////////////////////////////////////////////////////////////////
// DisneyFakeSS

// "Fake" subsurface scattering lobe, based on the Hanrahan-Krueger BRDF
// approximation of the BSSRDF.
class DisneyFakeSS : public BxDF {
  public:
    DisneyFakeSS(const Spectrum &R, Float roughness)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)),
          R(R),
          roughness(roughness) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum rho(const Vector3f &, int, const Point2f *) const { return R; }
    Spectrum rho(int, const Point2f *, const Point2f *) const { return R; }
    std::string ToString() const;

  private:
    Spectrum R;
    Float roughness;
};

Spectrum DisneyFakeSS::f(const Vector3f &wo, const Vector3f &wi) const {
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0.);
    wh = Normalize(wh);
    Float cosThetaD = Dot(wi, wh);

    // Fss90 used to "flatten" retroreflection based on roughness
    Float Fss90 = cosThetaD * cosThetaD * roughness;
    Float Fo = SchlickWeight(AbsCosTheta(wo)),
          Fi = SchlickWeight(AbsCosTheta(wi));
    Float Fss = Lerp(Fo, 1.0, Fss90) * Lerp(Fi, 1.0, Fss90);
    // 1.25 scale is used to (roughly) preserve albedo
    Float ss =
        1.25f * (Fss * (1 / (AbsCosTheta(wo) + AbsCosTheta(wi)) - .5f) + .5f);

    return R * InvPi * ss;
}

std::string DisneyFakeSS::ToString() const {
    return StringPrintf("[ DisneyFakeSS R: %s roughness: %f ]",
                        R.ToString().c_str(), roughness);
}

///////////////////////////////////////////////////////////////////////////
// DisneyRetro

class DisneyRetro : public BxDF {
  public:
    DisneyRetro(const Spectrum &R, Float roughness)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)),
          R(R),
          roughness(roughness) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum rho(const Vector3f &, int, const Point2f *) const { return R; }
    Spectrum rho(int, const Point2f *, const Point2f *) const { return R; }
    std::string ToString() const;

  private:
    Spectrum R;
    Float roughness;
};

Spectrum DisneyRetro::f(const Vector3f &wo, const Vector3f &wi) const {
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0.);
    wh = Normalize(wh);
    Float cosThetaD = Dot(wi, wh);

    Float Fo = SchlickWeight(AbsCosTheta(wo)),
          Fi = SchlickWeight(AbsCosTheta(wi));
    Float Rr = 2 * roughness * cosThetaD * cosThetaD;

    // Burley 2015, eq (4).
    return R * InvPi * Rr * (Fo + Fi + Fo * Fi * (Rr - 1));
}

std::string DisneyRetro::ToString() const {
    return StringPrintf("[ DisneyRetro R: %s roughness: %f ]",
                        R.ToString().c_str(), roughness);
}

///////////////////////////////////////////////////////////////////////////
// DisneySheen

class DisneySheen : public BxDF {
  public:
    DisneySheen(const Spectrum &R)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum rho(const Vector3f &, int, const Point2f *) const { return R; }
    Spectrum rho(int, const Point2f *, const Point2f *) const { return R; }
    std::string ToString() const;

  private:
    Spectrum R;
};

Spectrum DisneySheen::f(const Vector3f &wo, const Vector3f &wi) const {
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0.);
    wh = Normalize(wh);
    Float cosThetaD = Dot(wi, wh);

    return R * SchlickWeight(cosThetaD);
}

std::string DisneySheen::ToString() const {
    return StringPrintf("[ DisneySheen R: %s]", R.ToString().c_str());
}

///////////////////////////////////////////////////////////////////////////
// DisneyClearcoat

class DisneyClearcoat : public BxDF {
  public:
    DisneyClearcoat(Float weight, Float gloss)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_GLOSSY)),
          weight(weight),
          gloss(gloss) {}
    Spectrum f(const Vector3f &wo, const Vector3f &wi) const;
    Spectrum Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                      Float *pdf, BxDFType *sampledType) const;
    Float Pdf(const Vector3f &wo, const Vector3f &wi) const;
    std::string ToString() const;

  private:
    Float weight, gloss;
};

inline Float GTR1(Float cosTheta, Float alpha) {
    Float alpha2 = alpha * alpha;
    return (alpha2 - 1) /
           (Pi * std::log(alpha2) * (1 + (alpha2 - 1) * cosTheta * cosTheta));
}

// Smith masking/shadowing term.
inline Float smithG_GGX(Float cosTheta, Float alpha) {
    Float alpha2 = alpha * alpha;
    Float cosTheta2 = cosTheta * cosTheta;
    return 1 / (cosTheta + sqrt(alpha2 + cosTheta2 - alpha2 * cosTheta2));
}

Spectrum DisneyClearcoat::f(const Vector3f &wo, const Vector3f &wi) const {
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0.);
    wh = Normalize(wh);

    // Clearcoat has ior = 1.5 hardcoded -> F0 = 0.04. It then uses the
    // GTR1 distribution, which has even fatter tails than Trowbridge-Reitz
    // (which is GTR2).
    Float Dr = GTR1(AbsCosTheta(wh), gloss);
    Float Fr = FrSchlick(.04, Dot(wo, wh));
    // The geometric term always based on alpha = 0.25.
    Float Gr =
        smithG_GGX(AbsCosTheta(wo), .25) * smithG_GGX(AbsCosTheta(wi), .25);

    return weight * Gr * Fr * Dr / 4;
}

Spectrum DisneyClearcoat::Sample_f(const Vector3f &wo, Vector3f *wi,
                                   const Point2f &u, Float *pdf,
                                   BxDFType *sampledType) const {
    // TODO: double check all this: there still seem to be some very
    // occasional fireflies with clearcoat; presumably there is a bug
    // somewhere.
    if (wo.z == 0) return 0.;

    Float alpha2 = gloss * gloss;
    Float cosTheta = std::sqrt(
        std::max(Float(0), (1 - std::pow(alpha2, 1 - u[0])) / (1 - alpha2)));
    Float sinTheta = std::sqrt(std::max((Float)0, 1 - cosTheta * cosTheta));
    Float phi = 2 * Pi * u[1];
    Vector3f wh = SphericalDirection(sinTheta, cosTheta, phi);
    if (!SameHemisphere(wo, wh)) wh = -wh;

    *wi = Reflect(wo, wh);
    if (!SameHemisphere(wo, *wi)) return Spectrum(0.f);

    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

Float DisneyClearcoat::Pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (!SameHemisphere(wo, wi)) return 0;

    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return 0;
    wh = Normalize(wh);

    // The sampling routine samples wh exactly from the GTR1 distribution.
    // Thus, the final value of the PDF is just the value of the
    // distribution for wh converted to a mesure with respect to the
    // surface normal.
    Float Dr = GTR1(AbsCosTheta(wh), gloss);
    return Dr * AbsCosTheta(wh) / (4 * Dot(wo, wh));
}

std::string DisneyClearcoat::ToString() const {
    return StringPrintf("[ DisneyClearcoat weight: %f gloss: %f ]", weight,
                        gloss);
}

///////////////////////////////////////////////////////////////////////////
// DisneyFresnel

// Specialized Fresnel function used for the specular component, based on
// a mixture between dielectric and the Schlick Fresnel approximation.
class DisneyFresnel : public Fresnel {
  public:
    DisneyFresnel(const Spectrum &R0, Float metallic, Float eta)
        : R0(R0), metallic(metallic), eta(eta) {}
    Spectrum Evaluate(Float cosI) const {
        return Lerp(metallic, Spectrum(FrDielectric(cosI, 1, eta)),
                    FrSchlick(R0, cosI));
    }
    std::string ToString() const {
        return StringPrintf("[ DisneyFresnel R0: %s metallic: %f eta: %f ]",
                            R0.ToString().c_str(), metallic, eta);
    }

  private:
    const Spectrum R0;
    const Float metallic, eta;
};

///////////////////////////////////////////////////////////////////////////
// DisneyMicrofacetDistribution

class DisneyMicrofacetDistribution : public TrowbridgeReitzDistribution {
public:
    DisneyMicrofacetDistribution(Float alphax, Float alphay)
        : TrowbridgeReitzDistribution(alphax, alphay) {}

    Float G(const Vector3f &wo, const Vector3f &wi) const {
        // Disney uses the separable masking-shadowing model.
        return G1(wo) * G1(wi);
    }
};

///////////////////////////////////////////////////////////////////////////
// DisneyBSSRDF

// Implementation of the empirical BSSRDF described in "Extending the
// Disney BRDF to a BSDF with integrated subsurface scattering" (Brent
// Burley) and "Approximate Reflectance Profiles for Efficient Subsurface
// Scattering (Christensen and Burley).
class DisneyBSSRDF : public SeparableBSSRDF {
  public:
    DisneyBSSRDF(const Spectrum &R, const Spectrum &d,
                 const SurfaceInteraction &po, Float eta,
                 const Material *material, TransportMode mode)
        // 0.2 factor comes from personal communication from Brent Burley
        // and Matt Chiang.
        : SeparableBSSRDF(po, eta, material, mode), R(R), d(0.2 * d) {}

    Spectrum S(const SurfaceInteraction &pi, const Vector3f &wi);
    Spectrum Sr(Float d) const;
    Float Sample_Sr(int ch, Float u) const;
    Float Pdf_Sr(int ch, Float r) const;

  private:
    Spectrum R, d;
};

// We need to override BSSRDF::S() so that we can have access to the full
// hit information in order to modulate based on surface normal
// orientations..
Spectrum DisneyBSSRDF::S(const SurfaceInteraction &pi, const Vector3f &wi) {
    ProfilePhase pp(Prof::BSSRDFEvaluation);
    // Fade based on relative orientations of the two surface normals to
    // better handle surface cavities. (Details via personal communication
    // from Brent Burley; these details aren't published in the course
    // notes.)
    //
    // TODO: test
    // TODO: explain
    Vector3f a = Normalize(pi.p - po.p);
    Float fade = 1;
    Vector3f n = Vector3f(po.shading.n);
    Float cosTheta = Dot(a, n);
    if (cosTheta > 0) {
        // Point on or above surface plane
        Float sinTheta = std::sqrt(std::max(Float(0), 1 - cosTheta * cosTheta));
        Vector3f a2 = n * sinTheta - (a - n * cosTheta) * cosTheta / sinTheta;
        fade = std::max(Float(0), Dot(pi.shading.n, a2));
    }

    Float Fo = SchlickWeight(AbsCosTheta(po.wo)),
          Fi = SchlickWeight(AbsCosTheta(wi));
    return fade * (1 - Fo / 2) * (1 - Fi / 2) * Sp(pi) / Pi;
}

// Diffusion profile from Burley 2015, eq (5).
Spectrum DisneyBSSRDF::Sr(Float r) const {
    ProfilePhase pp(Prof::BSSRDFEvaluation);
    if (r < 1e-6f) r = 1e-6f;  // Avoid singularity at r == 0.
    return R * (Exp(-Spectrum(r) / d) + Exp(-Spectrum(r) / (3 * d))) /
           (8 * Pi * d * r);
}

Float DisneyBSSRDF::Sample_Sr(int ch, Float u) const {
    // The good news is that diffusion profile implemented in Sr is
    // normalized---integrating in polar coordinates, we have:
    //
    // int_0^2pi int_0^Infinity Sr(r) r dr dphi == 1.
    //
    // The CDF can be found in closed-form. It is:
    //
    // 1 - e^(-x/d) / 4 - (3 / 4) e^(-x / (3d)).
    //
    // Unfortunately, inverting the CDF requires solving a cubic, which
    // would be nice to sidestep. Therefore, following Christensen and
    // Burley's suggestion (section 6), we will sample from each of the two
    // exponential terms individually (which can be done directly) and then
    // compute an overall PDF using MIS.  There are a few details to work
    // through...
    //
    // For the first exponential term, we can find:
    // normalized PDF: e^(-r/d) / (2 Pi d r)
    // CDF: 1 - e^(-r/d)
    // sampling recipe: r = d log(1 / (1 - u))
    //
    // For the second:
    // PDF: e^(-r/(3d)) / (6 Pi d r)
    // CDF: 1 - e^(-r/(3d))
    // sampling: r = 3 d log(1 / (1 - u))
    //
    // The last question is what fraction of samples to use for each
    // technique.  The second exponential has 3x the contribution to the
    // final value as the first does, so therefore we'll take three samples
    // from that for every one sample we take from the first.
    if (u < .25f) {
        // Sample the first exponential
        u = std::min<Float>(u * 4, OneMinusEpsilon);  // renormalize to [0,1)
        return d[ch] * std::log(1 / (1 - u));
    } else {
        // Second exponenital
        u = std::min<Float>((u - .25f) / .75f, OneMinusEpsilon);  // normalize to [0,1)
        return 3 * d[ch] * std::log(1 / (1 - u));
    }
}

Float DisneyBSSRDF::Pdf_Sr(int ch, Float r) const {
    if (r < 1e-6f) r = 1e-6f;  // Avoid singularity at r == 0.

    // Weight the two individual PDFs as per the sampling frequency in
    // Sample_Sr().
    return (.25f * std::exp(-r / d[ch]) / (2 * Pi * d[ch] * r) +
            .75f * std::exp(-r / (3 * d[ch])) / (6 * Pi * d[ch] * r));
}

///////////////////////////////////////////////////////////////////////////
// DisneyMaterial

// DisneyMaterial Method Definitions
void DisneyMaterial::ComputeScatteringFunctions(SurfaceInteraction *si,
                                                MemoryArena &arena,
                                                TransportMode mode,
                                                bool allowMultipleLobes) const {
    // Perform bump mapping with _bumpMap_, if present
    if (bumpMap) Bump(bumpMap, si);

    // Evaluate textures for _DisneyMaterial_ material and allocate BRDF
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si);

    // Diffuse
    Spectrum c = color->Evaluate(*si).Clamp();
    Float metallicWeight = metallic->Evaluate(*si);
    Float e = eta->Evaluate(*si);
    Float strans = specTrans->Evaluate(*si);
    Float diffuseWeight = (1 - metallicWeight) * (1 - strans);
    Float dt = diffTrans->Evaluate(*si) /
               2;  // 0: all diffuse is reflected -> 1, transmitted
    Float rough = roughness->Evaluate(*si);
    Float lum = c.y();
    // normalize lum. to isolate hue+sat
    Spectrum Ctint = lum > 0 ? (c / lum) : Spectrum(1.);

    Float sheenWeight = sheen->Evaluate(*si);
    Spectrum Csheen;
    if (sheenWeight > 0) {
        Float stint = sheenTint->Evaluate(*si);
        Csheen = Lerp(stint, Spectrum(1.), Ctint);
    }

    if (diffuseWeight > 0) {
        if (thin) {
            Float flat = flatness->Evaluate(*si);
            // Blend between DisneyDiffuse and fake subsurface based on
            // flatness.  Additionally, weight using diffTrans.
            si->bsdf->Add(ARENA_ALLOC(arena, DisneyDiffuse)(
                diffuseWeight * (1 - flat) * (1 - dt) * c));
            si->bsdf->Add(ARENA_ALLOC(arena, DisneyFakeSS)(
                diffuseWeight * flat * (1 - dt) * c, rough));
        } else {
            Spectrum sd = scatterDistance->Evaluate(*si);
            if (sd.IsBlack())
                // No subsurface scattering; use regular (Fresnel modified)
                // diffuse.
                si->bsdf->Add(
                    ARENA_ALLOC(arena, DisneyDiffuse)(diffuseWeight * c));
            else {
                // Use a BSSRDF instead.
                si->bsdf->Add(ARENA_ALLOC(arena, SpecularTransmission)(
                    1.f, 1.f, e, mode));
                si->bssrdf = ARENA_ALLOC(arena, DisneyBSSRDF)(
                    c * diffuseWeight, sd, *si, e, this, mode);
            }
        }

        // Retro-reflection.
        si->bsdf->Add(
            ARENA_ALLOC(arena, DisneyRetro)(diffuseWeight * c, rough));

        // Sheen (if enabled)
        if (sheenWeight > 0)
            si->bsdf->Add(ARENA_ALLOC(arena, DisneySheen)(
                diffuseWeight * sheenWeight * Csheen));
    }

    // Create the microfacet distribution for metallic and/or specular
    // transmission.
    Float aspect = std::sqrt(1 - anisotropic->Evaluate(*si) * .9);
    Float ax = std::max(Float(.001), sqr(rough) / aspect);
    Float ay = std::max(Float(.001), sqr(rough) * aspect);
    MicrofacetDistribution *distrib =
        ARENA_ALLOC(arena, DisneyMicrofacetDistribution)(ax, ay);

    // Specular is Trowbridge-Reitz with a modified Fresnel function.
    Float specTint = specularTint->Evaluate(*si);
    Spectrum Cspec0 =
        Lerp(metallicWeight,
             SchlickR0FromEta(e) * Lerp(specTint, Spectrum(1.), Ctint), c);
    Fresnel *fresnel =
        ARENA_ALLOC(arena, DisneyFresnel)(Cspec0, metallicWeight, e);
    si->bsdf->Add(
        ARENA_ALLOC(arena, MicrofacetReflection)(c, distrib, fresnel));

    // Clearcoat
    Float cc = clearcoat->Evaluate(*si);
    if (cc > 0) {
        si->bsdf->Add(ARENA_ALLOC(arena, DisneyClearcoat)(
            cc, Lerp(clearcoatGloss->Evaluate(*si), .1, .001)));
    }

    // BTDF
    if (strans > 0) {
        // Walter et al's model, with the provided transmissive term scaled
        // by sqrt(color), so that after two refractions, we're back to the
        // provided color.
        Spectrum T = strans * Sqrt(c);
        if (thin) {
            // Scale roughness based on IOR (Burley 2015, Figure 15).
            Float rscaled = (0.65f * e - 0.35f) * rough;
            Float ax = std::max(Float(.001), sqr(rscaled) / aspect);
            Float ay = std::max(Float(.001), sqr(rscaled) * aspect);
            MicrofacetDistribution *scaledDistrib =
                ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(ax, ay);
            si->bsdf->Add(ARENA_ALLOC(arena, MicrofacetTransmission)(
                T, scaledDistrib, 1., e, mode));
        } else
            si->bsdf->Add(ARENA_ALLOC(arena, MicrofacetTransmission)(
                T, distrib, 1., e, mode));
    }
    if (thin) {
        // Lambertian, weighted by (1 - diffTrans)
        si->bsdf->Add(ARENA_ALLOC(arena, LambertianTransmission)(dt * c));
    }
}

DisneyMaterial *CreateDisneyMaterial(const TextureParams &mp) {
    std::shared_ptr<Texture<Spectrum>> color =
        mp.GetSpectrumTexture("color", Spectrum(0.5f));
    std::shared_ptr<Texture<Float>> metallic =
        mp.GetFloatTexture("metallic", 0.f);
    std::shared_ptr<Texture<Float>> eta = mp.GetFloatTexture("eta", 1.5f);
    std::shared_ptr<Texture<Float>> roughness =
        mp.GetFloatTexture("roughness", .5f);
    std::shared_ptr<Texture<Float>> specularTint =
        mp.GetFloatTexture("speculartint", 0.f);
    std::shared_ptr<Texture<Float>> anisotropic =
        mp.GetFloatTexture("anisotropic", 0.f);
    std::shared_ptr<Texture<Float>> sheen = mp.GetFloatTexture("sheen", 0.f);
    std::shared_ptr<Texture<Float>> sheenTint =
        mp.GetFloatTexture("sheentint", .5f);
    std::shared_ptr<Texture<Float>> clearcoat =
        mp.GetFloatTexture("clearcoat", 0.f);
    std::shared_ptr<Texture<Float>> clearcoatGloss =
        mp.GetFloatTexture("clearcoatgloss", 1.f);
    std::shared_ptr<Texture<Float>> specTrans =
        mp.GetFloatTexture("spectrans", 0.f);
    std::shared_ptr<Texture<Spectrum>> scatterDistance =
        mp.GetSpectrumTexture("scatterdistance", Spectrum(0.));
    bool thin = mp.FindBool("thin", false);
    std::shared_ptr<Texture<Float>> flatness =
        mp.GetFloatTexture("flatness", 0.f);
    std::shared_ptr<Texture<Float>> diffTrans =
        mp.GetFloatTexture("difftrans", 1.f);
    std::shared_ptr<Texture<Float>> bumpMap =
        mp.GetFloatTextureOrNull("bumpmap");
    return new DisneyMaterial(color, metallic, eta, roughness, specularTint,
                              anisotropic, sheen, sheenTint, clearcoat,
                              clearcoatGloss, specTrans, scatterDistance, thin,
                              flatness, diffTrans, bumpMap);
}

}  // namespace pbrt
