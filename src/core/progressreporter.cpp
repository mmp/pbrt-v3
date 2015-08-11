
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

// core/progressreporter.cpp*
#include "progressreporter.h"
#include <chrono>
#include "parallel.h"
#ifdef PBRT_IS_WINDOWS
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#endif  // !PBRT_IS_WINDOWS

// ProgressReporter Method Definitions
ProgressReporter::ProgressReporter(int64_t totalWork, const std::string &title)
    : totalWork(totalWork) {
    int barLength = TerminalWidth() - 28;
    totalPlusses = std::max(2, barLength - (int)title.size());
    plussesPrinted = 0;
    workDone = 0;
    startTime = std::chrono::system_clock::now();
    outFile = stdout;
    // Initialize progress string
    const int bufLen = title.size() + totalPlusses + 64;
    buf.reset(new char[bufLen]);
    snprintf(buf.get(), bufLen, "\r%s: [", title.c_str());
    curSpace = buf.get() + strlen(buf.get());
    char *s = curSpace;
    for (int i = 0; i < totalPlusses; ++i) *s++ = ' ';
    *s++ = ']';
    *s++ = ' ';
    *s++ = '\0';
    if (!PbrtOptions.quiet) {
        fputs(buf.get(), outFile);
        fflush(outFile);
    }
}

void ProgressReporter::Update(int64_t num) {
    if (num == 0 || PbrtOptions.quiet) return;
    std::lock_guard<std::mutex> lock(mutex);
    workDone += num;
    Float percentDone = Float(workDone) / Float(totalWork);
    int plussesNeeded = std::round(totalPlusses * percentDone);
    if (plussesNeeded > totalPlusses) plussesNeeded = totalPlusses;
    while (plussesPrinted < plussesNeeded) {
        *curSpace++ = '+';
        ++plussesPrinted;
    }
    fputs(buf.get(), outFile);
    // Update elapsed time and estimated time to completion
    Float seconds = ElapsedMS() / 1000.f;
    Float estRemaining = seconds / percentDone - seconds;
    if (percentDone == 1.f)
        fprintf(outFile, " (%.1fs)       ", seconds);
    else
        fprintf(outFile, " (%.1fs|%.1fs)  ", seconds,
                std::max((Float)0., estRemaining));
    fflush(outFile);
}

void ProgressReporter::Done() {
    if (PbrtOptions.quiet) return;
    std::lock_guard<std::mutex> lock(mutex);
    while (plussesPrinted++ < totalPlusses) *curSpace++ = '+';
    fputs(buf.get(), outFile);
    Float seconds = ElapsedMS() / 1000.f;
    fprintf(outFile, " (%.1fs)       \n", seconds);
    fflush(outFile);
}

int TerminalWidth() {
#ifdef PBRT_IS_WINDOWS
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h == INVALID_HANDLE_VALUE || !h) {
        fprintf(stderr, "GetStdHandle() call failed");
        return 80;
    }
    CONSOLE_SCREEN_BUFFER_INFO bufferInfo = {0};
    GetConsoleScreenBufferInfo(h, &bufferInfo);
    return bufferInfo.dwSize.X;
#else
    struct winsize w;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) < 0) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            fprintf(stderr, "Error in ioctl() in TerminalWidth(): %d\n", errno);
        }
        return 80;
    }
    return w.ws_col;
#endif  // PBRT_IS_WINDOWS
}
