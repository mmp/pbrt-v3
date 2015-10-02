
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

#ifndef PBRT_CORE_PROGRESSREPORTER_H
#define PBRT_CORE_PROGRESSREPORTER_H
#include "stdafx.h"

// core/progressreporter.h*
#include "pbrt.h"
#include <mutex>
#include <atomic>

// ProgressReporter Declarations
class ProgressReporter {
  public:
    // ProgressReporter Public Methods
    ProgressReporter(int64_t totalWork, const std::string &title);
    void Update(int64_t num = 1);
    Float ElapsedMS() const {
        std::chrono::system_clock::time_point now =
            std::chrono::system_clock::now();
        int64_t elapsedMS =
            std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                  startTime)
                .count();
        return (Float)elapsedMS;
    }
    void Done();

  private:
    // ProgressReporter Private Data
    const int64_t totalWork;
    int64_t workDone;
    int plussesPrinted, totalPlusses;
    std::chrono::system_clock::time_point startTime;
    FILE *outFile;
    std::unique_ptr<char[]> buf;
    char *curSpace;
    std::mutex mutex;
};

extern int TerminalWidth();

#endif  // PBRT_CORE_PROGRESSREPORTER_H
