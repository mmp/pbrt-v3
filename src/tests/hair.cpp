
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

// tests/hair.cpp*
#include "materials/hair.h"
#include "pbrt.h"
#include "rng.h"
#include "sampling.h"
#include "tests/gtest/gtest.h"

// Hair Tests
#if 0
TEST(Hair, Reciprocity) {
  RNG rng;
  for (int i = 0; i < 10; ++i) {
    Hair h(-1 + 2 * rng.UniformFloat(), 1.55,
           HairBSDF::SigmaAFromConcentration(.3 + 7.7 * rng.UniformFloat()),
           .1 + .9 * rng.UniformFloat(),
           .1 + .9 * rng.UniformFloat());
    Vector3f wi = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
    Vector3f wo = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
    Spectrum a = h.f(wi, wo) * AbsCosTheta(wo);
    Spectrum b = h.f(wo, wi) * AbsCosTheta(wi);
    EXPECT_EQ(a.y(), b.y()) << h << ", a = " << a << ", b = " << b << ", wi = " << wi
                    << ", wo = " << wo;
  }
}

#endif
TEST(Hair, Pow) {
    EXPECT_EQ(Pow<0>(2.f), 1 << 0);
    EXPECT_EQ(Pow<1>(2.f), 1 << 1);
    EXPECT_EQ(Pow<2>(2.f), 1 << 2);
    // Test remainder of pow template powers to 29
    EXPECT_EQ(Pow<3>(2.f), 1 << 3);
    EXPECT_EQ(Pow<4>(2.f), 1 << 4);
    EXPECT_EQ(Pow<5>(2.f), 1 << 5);
    EXPECT_EQ(Pow<6>(2.f), 1 << 6);
    EXPECT_EQ(Pow<7>(2.f), 1 << 7);
    EXPECT_EQ(Pow<8>(2.f), 1 << 8);
    EXPECT_EQ(Pow<9>(2.f), 1 << 9);
    EXPECT_EQ(Pow<10>(2.f), 1 << 10);
    EXPECT_EQ(Pow<11>(2.f), 1 << 11);
    EXPECT_EQ(Pow<12>(2.f), 1 << 12);
    EXPECT_EQ(Pow<13>(2.f), 1 << 13);
    EXPECT_EQ(Pow<14>(2.f), 1 << 14);
    EXPECT_EQ(Pow<15>(2.f), 1 << 15);
    EXPECT_EQ(Pow<16>(2.f), 1 << 16);
    EXPECT_EQ(Pow<17>(2.f), 1 << 17);
    EXPECT_EQ(Pow<18>(2.f), 1 << 18);
    EXPECT_EQ(Pow<19>(2.f), 1 << 19);
    EXPECT_EQ(Pow<20>(2.f), 1 << 20);
    EXPECT_EQ(Pow<21>(2.f), 1 << 21);
    EXPECT_EQ(Pow<22>(2.f), 1 << 22);
    EXPECT_EQ(Pow<23>(2.f), 1 << 23);
    EXPECT_EQ(Pow<24>(2.f), 1 << 24);
    EXPECT_EQ(Pow<25>(2.f), 1 << 25);
    EXPECT_EQ(Pow<26>(2.f), 1 << 26);
    EXPECT_EQ(Pow<27>(2.f), 1 << 27);
    EXPECT_EQ(Pow<28>(2.f), 1 << 28);
    EXPECT_EQ(Pow<29>(2.f), 1 << 29);
}

TEST(Hair, WhiteFurnace) {
    RNG rng;
    Vector3f wo = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
    for (Float beta_m = .1; beta_m < 1; beta_m += .2) {
        for (Float beta_n = .1; beta_n < 1; beta_n += .2) {
            // Estimate reflected uniform incident radiance from hair
            Spectrum sum = 0.f;
            int count = 300000;
            for (int i = 0; i < count; ++i) {
                Float h = -1 + 2. * rng.UniformFloat();
                Spectrum sigma_a = 0.f;
                HairBSDF hair(h, 1.55, sigma_a, beta_m, beta_n, 0.f);
                Vector3f wi = UniformSampleSphere(
                    {rng.UniformFloat(), rng.UniformFloat()});
                sum += hair.f(wo, wi) * AbsCosTheta(wi);
            }
            Float avg = sum.y() / (count * UniformSpherePdf());
            EXPECT_TRUE(avg >= .95 && avg <= 1.05);
        }
    }
}

TEST(Hair, WhiteFurnaceSampled) {
    RNG rng;
    Vector3f wo = UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
    for (Float beta_m = .1; beta_m < 1; beta_m += .2) {
        for (Float beta_n = .1; beta_n < 1; beta_n += .2) {
            Spectrum sum = 0.f;
            int count = 300000;
            for (int i = 0; i < count; ++i) {
                Float h = -1 + 2. * rng.UniformFloat();
                Spectrum sigma_a = 0.f;
                HairBSDF hair(h, 1.55, sigma_a, beta_m, beta_n, 0.f);

                Vector3f wi;
                Float pdf;
                Point2f u = {rng.UniformFloat(), rng.UniformFloat()};
                Spectrum f = hair.Sample_f(wo, &wi, u, &pdf, nullptr);
                if (pdf > 0) sum += f * AbsCosTheta(wi) / pdf;
            }
            Float avg = sum.y() / count;
            EXPECT_TRUE(avg >= .99 && avg <= 1.01) << avg;
        }
    }
}

TEST(Hair, SamplingWeights) {
    RNG rng;
    for (Float beta_m = .1; beta_m < 1; beta_m += .2)
        for (Float beta_n = .4; beta_n < 1; beta_n += .2) {
            int count = 10000;
            for (int i = 0; i < count; ++i) {
                // Check _HairBSDF::Sample\_f()_ sample weight
                Float h = -1 + 2 * rng.UniformFloat();
                Spectrum sigma_a = 0;
                HairBSDF hair(h, 1.55, sigma_a, beta_m, beta_n, 0.f);
                Vector3f wo = UniformSampleSphere(
                    {rng.UniformFloat(), rng.UniformFloat()});
                Vector3f wi;
                Float pdf;
                Point2f u = {rng.UniformFloat(), rng.UniformFloat()};
                Spectrum f = hair.Sample_f(wo, &wi, u, &pdf, nullptr);
                if (pdf > 0) {
                    // Verify that hair BSDF sample weight is close to 1 for
                    // _wi_
                    EXPECT_GT(f.y() * AbsCosTheta(wi) / pdf, 0.999);
                    EXPECT_LT(f.y() * AbsCosTheta(wi) / pdf, 1.001);
                }
            }
        }
}

TEST(Hair, SamplingConsistency) {
    RNG rng;
    for (Float beta_m = .2; beta_m < 1; beta_m += .2)
        for (Float beta_n = .4; beta_n < 1; beta_n += .2) {
            // Declare variables for hair sampling test
            const int count = 64 * 1024;
            Spectrum sigma_a = .25;
            Vector3f wo =
                UniformSampleSphere({rng.UniformFloat(), rng.UniformFloat()});
            auto Li = [](const Vector3f &w) -> Spectrum { return w.z * w.z; };
            Spectrum fImportance = 0, fUniform = 0;
            for (int i = 0; i < count; ++i) {
                // Compute estimates of scattered radiance for hair sampling
                // test
                Float h = -1 + 2 * rng.UniformFloat();
                HairBSDF hair(h, 1.55, sigma_a, beta_m, beta_n, 0.f);
                Vector3f wi;
                Float pdf;
                Point2f u = {rng.UniformFloat(), rng.UniformFloat()};
                Spectrum f = hair.Sample_f(wo, &wi, u, &pdf, nullptr);
                if (pdf > 0)
                    fImportance += f * Li(wi) * AbsCosTheta(wi) / (count * pdf);
                wi = UniformSampleSphere(u);
                fUniform += hair.f(wo, wi) * Li(wi) * AbsCosTheta(wi) /
                            (count * UniformSpherePdf());
            }
            // Verify consistency of estimated hair reflected radiance values
            Float err = std::abs(fImportance.y() - fUniform.y()) / fUniform.y();
            EXPECT_LT(err, 0.05);
        }
}
