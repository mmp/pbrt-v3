
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
    : totalWork(std::max((int64_t)1, totalWork)),
      title(title),
      startTime(std::chrono::system_clock::now()) {
    workDone = 0;
    exitThread = false;
    // Launch thread to periodically update progress bar
    if (!PbrtOptions.quiet)
        updateThread = std::thread([this]() { PrintBar(); });
}

ProgressReporter::~ProgressReporter() {
    if (!PbrtOptions.quiet) {
        workDone = totalWork;
        exitThread = true;
        updateThread.join();
        printf("\n");
    }
}

void ProgressReporter::PrintBar() {
    int barLength = TerminalWidth() - 28;
    int totalPlusses = std::max(2, barLength - (int)title.size());
    int plussesPrinted = 0;

    // Initialize progress string
    const int bufLen = title.size() + totalPlusses + 64;
    std::unique_ptr<char[]> buf(new char[bufLen]);
    snprintf(buf.get(), bufLen, "\r%s: [", title.c_str());
    char *curSpace = buf.get() + strlen(buf.get());
    char *s = curSpace;
    for (int i = 0; i < totalPlusses; ++i) *s++ = ' ';
    *s++ = ']';
    *s++ = ' ';
    *s++ = '\0';
    fputs(buf.get(), stdout);
    fflush(stdout);

    std::chrono::milliseconds sleepDuration(250);
    int iterCount = 0;
    while (!exitThread) {
        std::this_thread::sleep_for(sleepDuration);

        // Periodically increase sleepDuration to reduce overhead of
        // updates.
        ++iterCount;
        if (iterCount == 10)
            // Up to 0.5s after ~2.5s elapsed
            sleepDuration *= 2;
        else if (iterCount == 70)
            // Up to 1s after an additional ~30s have elapsed.
            sleepDuration *= 2;

        Float percentDone = Float(workDone) / Float(totalWork);
        int plussesNeeded = std::round(totalPlusses * percentDone);
        while (plussesPrinted < plussesNeeded) {
            *curSpace++ = '+';
            ++plussesPrinted;
        }
        fputs(buf.get(), stdout);

        // Update elapsed time and estimated time to completion
        Float seconds = ElapsedMS() / 1000.f;
        Float estRemaining = seconds / percentDone - seconds;
        if (percentDone == 1.f)
            printf(" (%.1fs)       ", seconds);
        else if (!std::isinf(estRemaining))
            printf(" (%.1fs|%.1fs)  ", seconds,
                   std::max((Float)0., estRemaining));
        else
            printf(" (%.1fs|?s)  ", seconds);
        fflush(stdout);
    }
}

void ProgressReporter::Done() {
    workDone = totalWork;
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
        // ENOTTY is fine and expected, e.g. if output is being piped to a file.
        if (errno != ENOTTY) {
            static bool warned = false;
            if (!warned) {
                warned = true;
                fprintf(stderr, "Error in ioctl() in TerminalWidth(): %d\n",
                        errno);
            }
        }
        return 80;
    }
    return w.ws_col;
#endif  // PBRT_IS_WINDOWS
}
