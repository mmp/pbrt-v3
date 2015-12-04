
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

// core/stats.cpp*
#include "stats.h"
#include <algorithm>
#include <mutex>
#include <cinttypes>
#include <type_traits>
#include <atomic>
#include <signal.h>
#ifndef PBRT_IS_WINDOWS
#include <sys/time.h>
#endif  // !PBRT_IS_WINDOWS

// Statistics Local Variables
std::vector<std::function<void(StatsAccumulator &)>> *StatRegisterer::funcs;
static StatsAccumulator statsAccumulator;
static std::unique_ptr<std::atomic<uint64_t>[]> profileSamples;
#ifndef PBRT_IS_WINDOWS
static void ReportProfileSample(int, siginfo_t *, void *);
#endif  // !PBRT_IS_WINDOWS

// Statistics Definitions
void ReportThreadStats() {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    StatRegisterer::CallCallbacks(statsAccumulator);
}

void StatRegisterer::CallCallbacks(StatsAccumulator &accum) {
    for (auto func : *funcs) func(accum);
}

void PrintStats(FILE *dest) { statsAccumulator.Print(dest); }

static void getCategoryAndTitle(const std::string &str, std::string *category,
                                std::string *title) {
    const char *s = str.c_str();
    const char *slash = strchr(s, '/');
    if (!slash)
        *title = str;
    else {
        *category = std::string(s, slash - s);
        *title = std::string(slash + 1);
    }
}

void StatsAccumulator::Print(FILE *dest) {
    fprintf(dest, "Statistics:\n");
    std::map<std::string, std::vector<std::string>> toPrint;

    char buf[512];
    for (auto &counter : counters) {
        if (counter.second == 0) continue;
        std::string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        sprintf(buf, "%-42s               %12" PRIu64, title.c_str(),
                counter.second);
        toPrint[category].push_back(buf);
    }
    for (auto &counter : memoryCounters) {
        if (counter.second == 0) continue;
        std::string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        double mib = (double)counter.second / (1024. * 1024.);
        sprintf(buf, "%-42s                  %9.2f MiB", title.c_str(), mib);
        toPrint[category].push_back(buf);
    }
    for (auto &distributionSum : intDistributionSums) {
        const std::string &name = distributionSum.first;
        if (intDistributionCounts[name] == 0) continue;
        std::string category, title;
        getCategoryAndTitle(name, &category, &title);
        double avg = (double)distributionSum.second /
                     (double)intDistributionCounts[name];
        sprintf(buf, "%-42s                      %.3f avg [range %" PRIu64
                     " - %" PRIu64 "]",
                title.c_str(), avg, intDistributionMins[name],
                intDistributionMaxs[name]);
        toPrint[category].push_back(buf);
    }
    for (auto &distributionSum : floatDistributionSums) {
        const std::string &name = distributionSum.first;
        if (floatDistributionCounts[name] == 0) continue;
        std::string category, title;
        getCategoryAndTitle(name, &category, &title);
        double avg = (double)distributionSum.second /
                     (double)floatDistributionCounts[name];
        sprintf(buf, "%-42s                      %.3f avg [range %f - %f]",
                title.c_str(), avg, floatDistributionMins[name],
                floatDistributionMaxs[name]);
        toPrint[category].push_back(buf);
    }
    for (auto &percentage : percentages) {
        if (percentage.second.second == 0) continue;
        int64_t num = percentage.second.first;
        int64_t denom = percentage.second.second;
        std::string category, title;
        getCategoryAndTitle(percentage.first, &category, &title);
        sprintf(buf, "%-42s%12" PRIu64 " / %12" PRIu64 " (%.2f%%)",
                title.c_str(), num, denom, (100.f * num) / denom);
        toPrint[category].push_back(buf);
    }
    for (auto &ratio : ratios) {
        if (ratio.second.second == 0) continue;
        int64_t num = ratio.second.first;
        int64_t denom = ratio.second.second;
        std::string category, title;
        getCategoryAndTitle(ratio.first, &category, &title);
        sprintf(buf, "%-42s%12" PRIu64 " / %12" PRIu64 " (%.2fx)",
                title.c_str(), num, denom, (double)num / (double)denom);
        toPrint[category].push_back(buf);
    }
    for (auto &timer : timers) {
        if (timer.second == 0) continue;
        uint64_t ns = timer.second;
        double seconds = (double)ns / 1e9;
        std::string category, title;
        getCategoryAndTitle(timer.first, &category, &title);
        sprintf(buf, "%-42s                  %9.3f s", title.c_str(), seconds);
        toPrint[category].push_back(buf);
    }

    for (auto &categories : toPrint) {
        fprintf(dest, "  %s\n", categories.first.c_str());
        for (auto &item : categories.second)
            fprintf(dest, "    %s\n", item.c_str());
    }
}

static constexpr int NumProfEvents = (int)Prof::NumProfEvents;
thread_local uint32_t profilerState;

#ifdef PBRT_IS_OSX
#include <execinfo.h>
#endif
void InitProfiler() {
    static_assert(NumProfEvents == sizeof(ProfNames) / sizeof(ProfNames[0]),
                  "ProfNames[] array and Prof enumerant have different "
                  "number of entries!");
    profileSamples.reset(new std::atomic<uint64_t>[1 << NumProfEvents]);
    for (int i = 0; i < (1 << NumProfEvents); ++i) profileSamples[i] = 0;
// Set timer to periodically interrupt the system for profiling
#ifndef PBRT_IS_WINDOWS
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = ReportProfileSample;
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPROF, &sa, NULL);

    static struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 1000000 / 100;  // 100 Hz sampling
    timer.it_value = timer.it_interval;

    if (setitimer(ITIMER_PROF, &timer, NULL) != 0)
        Error("Timer could not be initialized");
#endif
}

#ifndef PBRT_IS_WINDOWS
static void ReportProfileSample(int, siginfo_t *, void *) {
#if 0
    // Print stack trace if context is unknown
#if 0 && defined(PBRT_IS_OSX)
    static std::atomic<int> foo(20);
    if (profilerState == 0 && --foo == 0) {
        void* callstack[128];
        int i, frames = backtrace(callstack, 128);
        char** strs = backtrace_symbols(callstack, frames);
        for (i = 0; i < frames; ++i) {
            printf("%s\n", strs[i]);
        }
        free(strs);
        foo = 20;
    }
#endif
#endif
    if (profileSamples) profileSamples[profilerState]++;
}

#endif  // !PBRT_IS_WINDOWS
void ReportProfilerResults(FILE *dest) {
#ifndef PBRT_IS_WINDOWS
    fprintf(dest, "  Profile\n");

    uint64_t overallCount = 0;
    uint64_t eventCount[NumProfEvents] = {0};
    for (int i = 0; i < 1 << NumProfEvents; ++i) {
        uint64_t count = profileSamples[i].load();
        overallCount += count;
        for (int b = 0; b < NumProfEvents; ++b) {
            if (i & (1 << b)) eventCount[b] += count;
        }
    }

    std::map<std::string, uint64_t> hierarchicalResults;
    for (int i = 0; i < 1 << NumProfEvents; ++i) {
        uint64_t count = profileSamples[i].load();
        if (count == 0) continue;
        std::string s;
        for (int b = 0; b < NumProfEvents; ++b) {
            if (i & (1 << b)) {
                if (s.size() > 0) {
                    // contribute to the parents...
                    hierarchicalResults[s] += count;
                    s += "/";
                }
                s += ProfNames[b];
            }
        }
        if (s == "") s = "Startup and scene construction";
        hierarchicalResults[s] = count;
    }
    for (const auto &r : hierarchicalResults) {
        float pct = (100.f * r.second) / overallCount;
        int indent = 4;
        int slashIndex = r.first.find_last_of("/");
        if (slashIndex == std::string::npos)
            slashIndex = -1;
        else
            indent += 2 * std::count(r.first.begin(), r.first.end(), '/');
        const char *toPrint = r.first.c_str() + slashIndex + 1;
        fprintf(dest, "%*c%s%*c %5.2f %%\n", indent, ' ', toPrint,
                std::max(0, int(67 - strlen(toPrint) - indent)), ' ', pct);
    }
    fprintf(dest, "\n");
#endif
}
