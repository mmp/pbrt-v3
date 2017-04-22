
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

// core/stats.cpp*
#include "stats.h"
#include <signal.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <functional>
#include <mutex>
#include <type_traits>
#include "parallel.h"
#include "stringprint.h"
#ifdef PBRT_HAVE_ITIMER
#include <sys/time.h>
#endif  // PBRT_HAVE_ITIMER

namespace pbrt {

// Statistics Local Variables
std::vector<std::function<void(StatsAccumulator &)>> *StatRegisterer::funcs;
static StatsAccumulator statsAccumulator;

// For a given profiler state (i.e., a set of "on" bits corresponding to
// profiling categories that are active), ProfileSample stores a count of
// the number of times that state has been active when the timer interrupt
// to record a profiling sample has fired.
struct ProfileSample {
    std::atomic<uint64_t> profilerState{0};
    std::atomic<uint64_t> count{0};
};

// We use a hash table to keep track of the profiler state counts. Because
// we can't do dynamic memory allocation in a signal handler (and because
// the counts are updated in a signal handler), we can't easily use
// std::unordered_map.  We therefore allocate a fixed size hash table and
// use linear probing if there's a conflict.
static const int profileHashSize = 256;
static std::array<ProfileSample, profileHashSize> profileSamples;

static std::chrono::system_clock::time_point profileStartTime;

#ifdef PBRT_HAVE_ITIMER
static void ReportProfileSample(int, siginfo_t *, void *);
#endif  // PBRT_HAVE_ITIMER

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

void ClearStats() { statsAccumulator.Clear(); }

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

    for (auto &counter : counters) {
        if (counter.second == 0) continue;
        std::string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        toPrint[category].push_back(StringPrintf(
            "%-42s               %12" PRIu64, title.c_str(), counter.second));
    }
    for (auto &counter : memoryCounters) {
        if (counter.second == 0) continue;
        std::string category, title;
        getCategoryAndTitle(counter.first, &category, &title);
        double kb = (double)counter.second / 1024.;
        if (kb < 1024.)
            toPrint[category].push_back(StringPrintf(
                "%-42s                  %9.2f kB", title.c_str(), kb));
        else {
            float mib = kb / 1024.;
            if (mib < 1024.)
                toPrint[category].push_back(StringPrintf(
                    "%-42s                  %9.2f MiB", title.c_str(), mib));
            else {
                float gib = mib / 1024.;
                toPrint[category].push_back(StringPrintf(
                    "%-42s                  %9.2f GiB", title.c_str(), gib));
            }
        }
    }
    for (auto &distributionSum : intDistributionSums) {
        const std::string &name = distributionSum.first;
        if (intDistributionCounts[name] == 0) continue;
        std::string category, title;
        getCategoryAndTitle(name, &category, &title);
        double avg = (double)distributionSum.second /
                     (double)intDistributionCounts[name];
        toPrint[category].push_back(
            StringPrintf("%-42s                      %.3f avg [range %" PRIu64
                         " - %" PRIu64 "]",
                         title.c_str(), avg, intDistributionMins[name],
                         intDistributionMaxs[name]));
    }
    for (auto &distributionSum : floatDistributionSums) {
        const std::string &name = distributionSum.first;
        if (floatDistributionCounts[name] == 0) continue;
        std::string category, title;
        getCategoryAndTitle(name, &category, &title);
        double avg = (double)distributionSum.second /
                     (double)floatDistributionCounts[name];
        toPrint[category].push_back(
            StringPrintf("%-42s                      %.3f avg [range %f - %f]",
                         title.c_str(), avg, floatDistributionMins[name],
                         floatDistributionMaxs[name]));
    }
    for (auto &percentage : percentages) {
        if (percentage.second.second == 0) continue;
        int64_t num = percentage.second.first;
        int64_t denom = percentage.second.second;
        std::string category, title;
        getCategoryAndTitle(percentage.first, &category, &title);
        toPrint[category].push_back(
            StringPrintf("%-42s%12" PRIu64 " / %12" PRIu64 " (%.2f%%)",
                         title.c_str(), num, denom, (100.f * num) / denom));
    }
    for (auto &ratio : ratios) {
        if (ratio.second.second == 0) continue;
        int64_t num = ratio.second.first;
        int64_t denom = ratio.second.second;
        std::string category, title;
        getCategoryAndTitle(ratio.first, &category, &title);
        toPrint[category].push_back(StringPrintf(
            "%-42s%12" PRIu64 " / %12" PRIu64 " (%.2fx)", title.c_str(), num,
            denom, (double)num / (double)denom));
    }

    for (auto &categories : toPrint) {
        fprintf(dest, "  %s\n", categories.first.c_str());
        for (auto &item : categories.second)
            fprintf(dest, "    %s\n", item.c_str());
    }
}

void StatsAccumulator::Clear() {
    counters.clear();
    memoryCounters.clear();
    intDistributionSums.clear();
    intDistributionCounts.clear();
    intDistributionMins.clear();
    intDistributionMaxs.clear();
    floatDistributionSums.clear();
    floatDistributionCounts.clear();
    floatDistributionMins.clear();
    floatDistributionMaxs.clear();
    percentages.clear();
    ratios.clear();
}

PBRT_THREAD_LOCAL uint64_t ProfilerState;
static std::atomic<bool> profilerRunning{false};

void InitProfiler() {
    CHECK(!profilerRunning);

    // Access the per-thread ProfilerState variable now, so that there's no
    // risk of its first access being in the signal handler (which in turn
    // would cause dynamic memory allocation, which is illegal in a signal
    // handler).
    ProfilerState = ProfToBits(Prof::SceneConstruction);

    ClearProfiler();

    profileStartTime = std::chrono::system_clock::now();
// Set timer to periodically interrupt the system for profiling
#ifdef PBRT_HAVE_ITIMER
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

    CHECK_EQ(setitimer(ITIMER_PROF, &timer, NULL), 0)
        << "Timer could not be initialized: " << strerror(errno);
#endif
    profilerRunning = true;
}

static std::atomic<int> profilerSuspendCount{0};

void SuspendProfiler() { ++profilerSuspendCount; }

void ResumeProfiler() { CHECK_GE(--profilerSuspendCount, 0); }

void ProfilerWorkerThreadInit() {
#ifdef PBRT_HAVE_ITIMER
    // The per-thread initialization in the worker threads has to happen
    // *before* the profiling signal handler is installed.
    CHECK(!profilerRunning || profilerSuspendCount > 0);

    // ProfilerState is a thread-local variable that is accessed in the
    // profiler signal handler. It's important to access it here, which
    // causes the dynamic memory allocation for the thread-local storage to
    // happen now, rather than in the signal handler, where this isn't
    // allowed.
    ProfilerState = ProfToBits(Prof::SceneConstruction);
#endif  // PBRT_HAVE_ITIMER
}

void ClearProfiler() {
    for (ProfileSample &ps : profileSamples) {
        ps.profilerState = 0;
        ps.count = 0;
    }
}

void CleanupProfiler() {
    CHECK(profilerRunning);
#ifdef PBRT_HAVE_ITIMER
    static struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    timer.it_value = timer.it_interval;

    CHECK_EQ(setitimer(ITIMER_PROF, &timer, NULL), 0)
        << "Timer could not be disabled: " << strerror(errno);
#endif  // PBRT_HAVE_ITIMER
    profilerRunning = false;
}

#ifdef PBRT_HAVE_ITIMER
static void ReportProfileSample(int, siginfo_t *, void *) {
    if (profilerSuspendCount > 0) return;
    if (ProfilerState == 0) return;  // A ProgressReporter thread, most likely.

    uint64_t h = std::hash<uint64_t>{}(ProfilerState) % (profileHashSize - 1);
    int count = 0;
    while (count < profileHashSize &&
           profileSamples[h].profilerState != ProfilerState &&
           profileSamples[h].profilerState != 0) {
        // Wrap around to the start if we hit the end.
        if (++h == profileHashSize) h = 0;
        ++count;
    }
    CHECK_NE(count, profileHashSize) << "Profiler hash table filled up!";
    profileSamples[h].profilerState = ProfilerState;
    ++profileSamples[h].count;
}
#endif  // PBRT_HAVE_ITIMER

static std::string timeString(float pct, std::chrono::system_clock::time_point now) {
    pct /= 100.;  // remap passed value to to [0,1]
    int64_t ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - profileStartTime).count();
    // milliseconds for this category
    int64_t ms = int64_t(ns * pct / 1000000.);
    // Peel off hours, minutes, seconds, and remaining milliseconds.
    int h = ms / (3600 * 1000);
    ms -= h * 3600 * 1000;
    int m = ms / (60 * 1000);
    ms -= m * (60 * 1000);
    int s = ms / 1000;
    ms -= s * 1000;
    ms /= 10;  // only printing 2 digits of fractional seconds
    return StringPrintf("%4d:%02d:%02d.%02d", h, m, s, ms);
}

void ReportProfilerResults(FILE *dest) {
#ifdef PBRT_HAVE_ITIMER
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    PBRT_CONSTEXPR int NumProfCategories = (int)Prof::NumProfCategories;
    uint64_t overallCount = 0;
    uint64_t eventCount[NumProfCategories] = {0};
    int used = 0;
    for (const ProfileSample &ps : profileSamples) {
        if (ps.count > 0) {
            overallCount += ps.count;
            ++used;
            for (int b = 0; b < NumProfCategories; ++b)
                if (ps.profilerState & (1ull << b)) eventCount[b] += ps.count;
        }
    }
    LOG(INFO) << "Used " << used << " / " << profileHashSize
              << " entries in profiler hash table";

    std::map<std::string, uint64_t> flatResults;
    std::map<std::string, uint64_t> hierarchicalResults;
    for (const ProfileSample &ps : profileSamples) {
        if (ps.count == 0) continue;

        std::string s;
        for (int b = 0; b < NumProfCategories; ++b) {
            if (ps.profilerState & (1ull << b)) {
                if (s.size() > 0) {
                    // contribute to the parents...
                    hierarchicalResults[s] += ps.count;
                    s += "/";
                }
                s += ProfNames[b];
            }
        }
        hierarchicalResults[s] += ps.count;

        int nameIndex = Log2Int(ps.profilerState);
        DCHECK_LT(nameIndex, NumProfCategories);
        flatResults[ProfNames[nameIndex]] += ps.count;
    }

    fprintf(dest, "  Profile\n");
    for (const auto &r : hierarchicalResults) {
        float pct = (100.f * r.second) / overallCount;
        int indent = 4;
        int slashIndex = r.first.find_last_of("/");
        if (slashIndex == std::string::npos)
            slashIndex = -1;
        else
            indent += 2 * std::count(r.first.begin(), r.first.end(), '/');
        const char *toPrint = r.first.c_str() + slashIndex + 1;
        fprintf(dest, "%*c%s%*c %5.2f%% (%s)\n", indent, ' ', toPrint,
                std::max(0, int(67 - strlen(toPrint) - indent)), ' ', pct,
                timeString(pct, now).c_str());
    }

    // Sort the flattened ones by time, longest to shortest.
    std::vector<std::pair<std::string, uint64_t>> flatVec;
    for (const auto &r : flatResults)
        flatVec.push_back(std::make_pair(r.first, r.second));
    std::sort(
        flatVec.begin(), flatVec.end(),
        [](std::pair<std::string, uint64_t> a,
           std::pair<std::string, uint64_t> b) { return a.second > b.second; });

    fprintf(dest, "  Profile (flattened)\n");
    for (const auto &r : flatVec) {
        float pct = (100.f * r.second) / overallCount;
        int indent = 4;
        const char *toPrint = r.first.c_str();
        fprintf(dest, "%*c%s%*c %5.2f%% (%s)\n", indent, ' ', toPrint,
                std::max(0, int(67 - strlen(toPrint) - indent)), ' ', pct,
                timeString(pct, now).c_str());
    }
    fprintf(dest, "\n");
#endif
}

}  // namespace pbrt
