
#include "tests/gtest/gtest.h"
#include <stdint.h>
#include <algorithm>
#include "pbrt.h"
#include "sampling.h"
#include "lowdiscrepancy.h"
#include "samplers/maxmin.h"
#include "samplers/sobol.h"
#include "samplers/zerotwosequence.h"

TEST(LowDiscrepancy, RadicalInverse) {
    for (int a = 0; a < 1024; ++a) {
        EXPECT_EQ(ReverseBits32(a) * 2.3283064365386963e-10f,
                  RadicalInverse(0, a));
    }
}

TEST(LowDiscrepancy, GeneratorMatrix) {
    uint32_t C[32];
    // Identity matrix, column-wise
    for (int i = 0; i < 32; ++i) C[i] = 1 << i;

    for (int a = 0; a < 128; ++a) {
        // Make sure identity generator matrix matches van der Corput
        EXPECT_EQ(a, MultiplyGenerator(C, a));
        EXPECT_EQ(RadicalInverse(0, a), ReverseBits32(MultiplyGenerator(C, a)) *
                                            2.3283064365386963e-10f);
    }

    for (int a = 0; a < 16; ++a) {
        EXPECT_EQ(ReverseBits32(a), ReverseMultiplyGenerator(C, a));
        EXPECT_EQ(RadicalInverse(0, a),
                  ReverseMultiplyGenerator(C, a) * 2.3283064365386963e-10f);
        EXPECT_EQ(RadicalInverse(0, a), SampleGeneratorMatrix(C, a));
    }
}

TEST(LowDiscrepancy, GrayCodeSample) {
    uint32_t C[32];
    // Identity matrix, column-wise
    for (int i = 0; i < 32; ++i) C[i] = 1 << i;

    std::vector<Float> v(64, (Float)0);
    GrayCodeSample(C, v.size(), 0, &v[0]);

    for (int a = 0; a < (int)v.size(); ++a) {
        Float u = ReverseMultiplyGenerator(C, a) * 2.3283064365386963e-10f;
        EXPECT_NE(v.end(), std::find(v.begin(), v.end(), u));
    }
}

// Make sure samplers that are supposed to generate a single sample in
// each of the elementary intervals actually do so.
// TODO: check Halton (where the elementary intervals are (2^i, 3^j)).
TEST(LowDiscrepancy, ElementaryIntervals) {
  auto checkSampler = [](const char *name, std::unique_ptr<Sampler> sampler,
                         int logSamples) {
        // Get all of the samples for a pixel.
        sampler->StartPixel(Point2i(0, 0));
        std::vector<Point2f> samples;
        do {
            samples.push_back(sampler->Get2D());
        } while (sampler->StartNextSample());

        for (int i = 0; i <= logSamples; ++i) {
            // Check one set of elementary intervals: number of intervals
            // in each dimension.
            int nx = 1 << i, ny = 1 << (logSamples - i);

            std::vector<int> count(1 << logSamples,  0);
            for (const Point2f &s : samples) {
                // Map the sample to an interval
                Float x = nx * s.x, y = ny * s.y;
                EXPECT_GE(x, 0);
                EXPECT_LT(x, nx);
                EXPECT_GE(y, 0);
                EXPECT_LT(y, ny);
                int index = (int)std::floor(y) * nx + (int)std::floor(x);
                EXPECT_GE(index, 0);
                EXPECT_LT(index, count.size());

                // This should be the first time a sample has landed in its
                // interval.
                EXPECT_EQ(0, count[index]) << "Sampler " << name;
                ++count[index];
            }
        }
    };

    for (int logSamples = 2; logSamples <= 10; ++logSamples) {
      checkSampler("MaxMinDistSampler", std::unique_ptr<Sampler>(
          new MaxMinDistSampler(1 << logSamples, 2)), logSamples);
      checkSampler("ZeroTwoSequenceSampler", std::unique_ptr<Sampler>(
          new ZeroTwoSequenceSampler(1 << logSamples, 2)), logSamples);
      checkSampler("Sobol", std::unique_ptr<Sampler>(
          new SobolSampler(1 << logSamples,
                           Bounds2i(Point2i(0, 0), Point2i(10, 10)))),
                   logSamples);
    }
}

TEST(MaxMinDist, MinDist) {
    // We use a silly O(n^2) distance check below, so don't go all the way up
    // to 2^16 samples.
    for (int logSamples = 2; logSamples <= 10; ++logSamples) {
        // Store a pixel's worth of samples in the vector s.
        MaxMinDistSampler mm(1 << logSamples, 2);
        mm.StartPixel(Point2i(0, 0));
        std::vector<Point2f> s;
        do {
            s.push_back(mm.Get2D());
        } while (mm.StartNextSample());

        // Distance with toroidal topology
        auto dist = [](const Point2f &p0, const Point2f &p1) {
            Vector2f d = Abs(p1 - p0);
            if (d.x > 0.5) d.x = 1. - d.x;
            if (d.y > 0.5) d.y = 1. - d.y;
            return d.Length();
        };

        Float minDist = Infinity;
        for (size_t i = 0; i < s.size(); ++i) {
          for (size_t j = 0; j < s.size(); ++j) {
            if (i == j)
              continue;
            minDist = std::min(minDist, dist(s[i], s[j]));
          }
        }

        // Expected minimum distances from Gruenschloss et al.'s paper.
        Float expectedMinDist[17] = {
          0., /* not checked */
          0., /* not checked */
          0.35355,
          0.35355,
          0.22534,
          0.16829,
          0.11267,
          0.07812,
          0.05644,
          0.03906,
          0.02816,
          0.01953,
          0.01408,
          0.00975,
          0.00704,
          0.00486,
          0.00352,
        };
        // Increase the tolerance by a small slop factor.
        EXPECT_GT(minDist, 0.99 * expectedMinDist[logSamples]);
    }
}
