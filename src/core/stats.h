
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

#ifndef PBRT_CORE_STATS_H
#define PBRT_CORE_STATS_H
#include "stdafx.h"

// core/stats.h*
#include "pbrt.h"
#include <map>
#include <chrono>
#include <string>
#include <functional>

// Statistics Declarations
class StatsAccumulator;
class StatRegisterer {
  public:
    // StatRegisterer Public Methods
    StatRegisterer(std::function<void(StatsAccumulator &)> func) {
        if (!funcs)
            funcs = new std::vector<std::function<void(StatsAccumulator &)>>;
        funcs->push_back(func);
    }
    static void CallCallbacks(StatsAccumulator &accum);

  private:
    // StatRegisterer Private Data
    static std::vector<std::function<void(StatsAccumulator &)>> *funcs;
};

void PrintStats(FILE *dest);
void ReportThreadStats();

class StatsAccumulator {
  public:
    // StatsAccumulator Public Methods
    void ReportCounter(const std::string &name, int64_t val) {
        counters[name] += val;
    }
    void ReportMemoryCounter(const std::string &name, int64_t val) {
        memoryCounters[name] += val;
    }
    void ReportIntDistribution(const std::string &name, int64_t sum,
                               int64_t count, int64_t min, int64_t max) {
        intDistributionSums[name] += sum;
        intDistributionCounts[name] += count;
        if (intDistributionMins.find(name) == intDistributionMins.end())
            intDistributionMins[name] = min;
        else
            intDistributionMins[name] =
                std::min(intDistributionMins[name], min);
        if (intDistributionMaxs.find(name) == intDistributionMaxs.end())
            intDistributionMaxs[name] = max;
        else
            intDistributionMaxs[name] =
                std::max(intDistributionMaxs[name], max);
    }
    void ReportFloatDistribution(const std::string &name, double sum,
                                 int64_t count, double min, double max) {
        floatDistributionSums[name] += sum;
        floatDistributionCounts[name] += count;
        if (floatDistributionMins.find(name) == floatDistributionMins.end())
            floatDistributionMins[name] = min;
        else
            floatDistributionMins[name] =
                std::min(floatDistributionMins[name], min);
        if (floatDistributionMaxs.find(name) == floatDistributionMaxs.end())
            floatDistributionMaxs[name] = max;
        else
            floatDistributionMaxs[name] =
                std::max(floatDistributionMaxs[name], max);
    }
    void ReportPercentage(const std::string &name, int64_t num, int64_t denom) {
        percentages[name].first += num;
        percentages[name].second += denom;
    }
    void ReportRatio(const std::string &name, int64_t num, int64_t denom) {
        ratios[name].first += num;
        ratios[name].second += denom;
    }
    void ReportTimer(const std::string &name, int64_t val) {
        timers[name] += val;
    }

    void Print(FILE *file);

  private:
    // StatsAccumulator Private Data
    std::map<std::string, int64_t> counters;
    std::map<std::string, int64_t> memoryCounters;
    std::map<std::string, int64_t> intDistributionSums;
    std::map<std::string, int64_t> intDistributionCounts;
    std::map<std::string, int64_t> intDistributionMins;
    std::map<std::string, int64_t> intDistributionMaxs;
    std::map<std::string, double> floatDistributionSums;
    std::map<std::string, int64_t> floatDistributionCounts;
    std::map<std::string, double> floatDistributionMins;
    std::map<std::string, double> floatDistributionMaxs;
    std::map<std::string, std::pair<int64_t, int64_t>> percentages;
    std::map<std::string, std::pair<int64_t, int64_t>> ratios;
    std::map<std::string, int64_t> timers;
};

enum class Prof {
    IntegratorRender,
    SamplerIntegratorLi,
    DirectLighting,
    AccelIntersect,
    AccelIntersectP,
    TriIntersect,
    // Remainder of _Prof_ _enum_ entries
    TriIntersectP,
    ComputeScatteringFuncs,
    GenerateCameraRay,
    BSDFEvaluation,
    BSSRDFEvaluation,
    MergeFilmTile,
    SplatFilm,
    StartPixel,
    TexFiltTrilerp,
    TexFiltEWA,
    NumProfEvents
};

static const char *ProfNames[] = {
    "Integrator::Render()",
    "SamplerIntegrator::Li()",
    "Direct lighting",
    "Accelerator::Intersect()",
    "Accelerator::IntersectP()",
    "Triangle::Intersect()",
    "Triangle::IntersectP()",
    "Material::ComputeScatteringFunctions()",
    "Camera::GenerateRay[Differential]()",
    "BSDF::f()",
    "BSSRDF::f()",
    "Film::MergeTile()",
    "Film::AddSplat()",
    "Sampler::StartPixelSample()",
    "MIPMap::Lookup() (trilinear)",
    "MIPMap::Lookup() (EWA)",
};

extern thread_local uint32_t profilerState;
inline uint32_t CurrentProfilerState() { return profilerState; }
class ProfilePhase {
  public:
    // ProfilePhase Public Methods
    ProfilePhase(Prof p) {
        categoryBit = (1 << (int)p);
        reset = (profilerState & categoryBit) == 0;
        profilerState |= categoryBit;
    }
    ~ProfilePhase() {
        if (reset) profilerState &= ~categoryBit;
    }
    ProfilePhase(const ProfilePhase &) = delete;
    ProfilePhase &operator=(const ProfilePhase &) = delete;

  private:
    // ProfilePhase Private Data
    bool reset;
    uint32_t categoryBit;
};

void InitProfiler();
void ReportProfilerResults(FILE *dest);

// Statistics Macros
#define STAT_COUNTER(title, var)                           \
    static thread_local int64_t var;                       \
    static void STATS_FUNC##var(StatsAccumulator &accum) { \
        accum.ReportCounter(title, var);                   \
        var = 0;                                           \
    }                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)
#define STAT_MEMORY_COUNTER(title, var)                    \
    static thread_local int64_t var;                       \
    static void STATS_FUNC##var(StatsAccumulator &accum) { \
        accum.ReportMemoryCounter(title, var);             \
        var = 0;                                           \
    }                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

// Work around lack of support for constexpr in VS2013 and earlier.
#if defined(PBRT_IS_MSVC) && (_MSC_VER <= 1800)
#define STATS_INT64_T_MIN LLONG_MAX
#define STATS_INT64_T_MAX _I64_MIN
#define STATS_DBL_T_MIN DBL_MAX
#define STATS_DBL_T_MAX -DBL_MAX
#else
#define STATS_INT64_T_MIN std::numeric_limits<int64_t>::max()
#define STATS_INT64_T_MAX std::numeric_limits<int64_t>::lowest()
#define STATS_DBL_T_MIN std::numeric_limits<double>::max()
#define STATS_DBL_T_MAX std::numeric_limits<double>::lowest()
#endif

#define STAT_INT_DISTRIBUTION(title, var)                                  \
    static thread_local int64_t var##sum;                                  \
    static thread_local int64_t var##count;                                \
    static thread_local int64_t var##min = (STATS_INT64_T_MIN);            \
    static thread_local int64_t var##max = (STATS_INT64_T_MAX);            \
    static void STATS_FUNC##var(StatsAccumulator &accum) {                 \
        accum.ReportIntDistribution(title, var##sum, var##count, var##min, \
                                    var##max);                             \
        var##sum = 0;                                                      \
        var##count = 0;                                                    \
        var##min = std::numeric_limits<int64_t>::max();                    \
        var##max = std::numeric_limits<int64_t>::lowest();                 \
    }                                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

#define STAT_FLOAT_DISTRIBUTION(title, var)                                  \
    static thread_local double var##sum;                                     \
    static thread_local int64_t var##count;                                  \
    static thread_local double var##min = (STATS_DBL_T_MIN);                 \
    static thread_local double var##max = (STATS_DBL_T_MAX);                 \
    static void STATS_FUNC##var(StatsAccumulator &accum) {                   \
        accum.ReportFloatDistribution(title, var##sum, var##count, var##min, \
                                      var##max);                             \
        var##sum = 0;                                                        \
        var##count = 0;                                                      \
        var##min = std::numeric_limits<double>::max();                       \
        var##max = std::numeric_limits<double>::lowest();                    \
    }                                                                        \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

#define ReportValue(var, value)                                   \
    do {                                                          \
        var##sum += value;                                        \
        var##count += 1;                                          \
        var##min = std::min(var##min, decltype(var##min)(value)); \
        var##max = std::max(var##max, decltype(var##min)(value)); \
    } while (0)

#define STAT_PERCENT(title, numVar, denomVar)                 \
    static thread_local int64_t numVar, denomVar;             \
    static void STATS_FUNC##numVar(StatsAccumulator &accum) { \
        accum.ReportPercentage(title, numVar, denomVar);      \
        numVar = denomVar = 0;                                \
    }                                                         \
    static StatRegisterer STATS_REG##numVar(STATS_FUNC##numVar)

#define STAT_RATIO(title, numVar, denomVar)                   \
    static thread_local int64_t numVar, denomVar;             \
    static void STATS_FUNC##numVar(StatsAccumulator &accum) { \
        accum.ReportRatio(title, numVar, denomVar);           \
        numVar = denomVar = 0;                                \
    }                                                         \
    static StatRegisterer STATS_REG##numVar(STATS_FUNC##numVar)

class StatTimer {
  public:
    StatTimer(uint64_t *sns) {
        sumNS = sns;
        startTime = std::chrono::high_resolution_clock::now();
    };
    ~StatTimer() {
        time_point endTime = std::chrono::high_resolution_clock::now();
        *sumNS += std::chrono::duration_cast<std::chrono::nanoseconds>(
                      endTime - startTime)
                      .count();
    }

    typedef std::chrono::high_resolution_clock::time_point time_point;
    time_point startTime;
    uint64_t *sumNS;
};

#define STAT_TIMER(title, var)                             \
    static thread_local uint64_t var;                      \
    static void STATS_FUNC##var(StatsAccumulator &accum) { \
        accum.ReportTimer(title, var);                     \
        var = 0;                                           \
    }                                                      \
    static StatRegisterer STATS_REG##var(STATS_FUNC##var)

#endif  // PBRT_CORE_STATS_H
