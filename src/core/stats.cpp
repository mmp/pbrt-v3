
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

// Statistics Local Variables
std::vector<void (*)(StatsAccumulator &)> *StatRegisterer::funcs;
static StatsAccumulator statsAccumulator;

// Statistics Definitions
void ReportThreadStats() {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    StatRegisterer::CallCallbacks(statsAccumulator);
}

void StatRegisterer::CallCallbacks(StatsAccumulator &accum) {
    for (auto func : *funcs) func(accum);
}

void PrintStats(FILE *dest) {
    ReportThreadStats();
    statsAccumulator.Print(dest);
}

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
