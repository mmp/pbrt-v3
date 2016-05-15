
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


// core/parallel.cpp*
#include "parallel.h"
#include "memory.h"
#include "stats.h"
#include <list>
#include <thread>
#include <condition_variable>

// Parallel Local Definitions
static std::vector<std::thread> threads;
static bool shutdownThreads = false;
class ParallelForLoop;
static ParallelForLoop *workList = nullptr;
static std::mutex workListMutex;
class ParallelForLoop {
  public:
    // ParallelForLoop Public Methods
    ParallelForLoop(std::function<void(int64_t)> func1D, int64_t maxIndex,
                    int chunkSize, int profilerState)
        : func1D(std::move(func1D)),
          maxIndex(maxIndex),
          chunkSize(chunkSize),
          profilerState(profilerState) {}
    ParallelForLoop(const std::function<void(Point2i)> &f, const Point2i &count,
                    int profilerState)
        : func2D(f),
          maxIndex(count.x * count.y),
          chunkSize(1),
          profilerState(profilerState) {
        nX = count.x;
    }

  public:
    // ParallelForLoop Private Data
    std::function<void(int64_t)> func1D;
    std::function<void(Point2i)> func2D;
    const int64_t maxIndex;
    const int chunkSize, profilerState;
    int64_t nextIndex = 0;
    int activeWorkers = 0;
    ParallelForLoop *next = nullptr;
    int nX = -1;

    // ParallelForLoop Private Methods
    bool Finished() const {
        return nextIndex >= maxIndex && activeWorkers == 0;
    }
};

static std::condition_variable workListCondition;
static void workerThreadFunc(int tIndex) {
    ThreadIndex = tIndex;
    std::unique_lock<std::mutex> lock(workListMutex);
    while (!shutdownThreads) {
        if (!workList) {
            // Sleep until there are more tasks to run
            workListCondition.wait(lock);
        } else {
            // Get work from _workList_ and run loop iterations
            ParallelForLoop &loop = *workList;

            // Run a chunk of loop iterations for _loop_

            // Find the set of loop iterations to run next
            int64_t indexStart = loop.nextIndex;
            int64_t indexEnd =
                std::min(indexStart + loop.chunkSize, loop.maxIndex);

            // Update _loop_ to reflect iterations this thread will run
            loop.nextIndex = indexEnd;
            if (loop.nextIndex == loop.maxIndex) workList = loop.next;
            loop.activeWorkers++;

            // Run loop indices in _[indexStart, indexEnd)_
            lock.unlock();
            for (int64_t index = indexStart; index < indexEnd; ++index) {
                int oldState = profilerState;
                profilerState = loop.profilerState;
                if (loop.func1D) {
                    loop.func1D(index);
                }
                // Handle other types of loops
                else {
                    Assert(loop.func2D);
                    loop.func2D(Point2i(index % loop.nX, index / loop.nX));
                }
                profilerState = oldState;
            }
            lock.lock();

            // Update _loop_ to reflect completion of iterations
            loop.activeWorkers--;
            if (loop.Finished()) workListCondition.notify_all();
        }
    }
    // Report thread statistics at worker thread exit
    ReportThreadStats();
}

// Parallel Definitions
void ParallelFor(const std::function<void(int64_t)> &func, int64_t count,
                 int chunkSize) {
    // Run iterations immediately if not using threads or if _count_ is small
    if (PbrtOptions.nThreads == 1 || count < chunkSize) {
        for (int64_t i = 0; i < count; ++i) func(i);
        return;
    }

    // Launch worker threads if needed
    if (threads.size() == 0) {
        Assert(PbrtOptions.nThreads != 1);
        ThreadIndex = 0;
        for (int i = 0; i < NumSystemCores() - 1; ++i)
            threads.push_back(std::thread(workerThreadFunc, i + 1));
    }

    // Create and enqueue _ParallelForLoop_ for this loop
    ParallelForLoop loop(func, count, chunkSize, CurrentProfilerState());
    workListMutex.lock();
    loop.next = workList;
    workList = &loop;
    workListMutex.unlock();

    // Notify worker threads of work to be done
    std::unique_lock<std::mutex> lock(workListMutex);
    workListCondition.notify_all();

    // Help out with parallel loop iterations in the current thread
    while (!loop.Finished()) {
        // Run a chunk of loop iterations for _loop_

        // Find the set of loop iterations to run next
        int64_t indexStart = loop.nextIndex;
        int64_t indexEnd = std::min(indexStart + loop.chunkSize, loop.maxIndex);

        // Update _loop_ to reflect iterations this thread will run
        loop.nextIndex = indexEnd;
        if (loop.nextIndex == loop.maxIndex) workList = loop.next;
        loop.activeWorkers++;

        // Run loop indices in _[indexStart, indexEnd)_
        lock.unlock();
        for (int64_t index = indexStart; index < indexEnd; ++index) {
            int oldState = profilerState;
            profilerState = loop.profilerState;
            if (loop.func1D) {
                loop.func1D(index);
            }
            // Handle other types of loops
            else {
                Assert(loop.func2D);
                loop.func2D(Point2i(index % loop.nX, index / loop.nX));
            }
            profilerState = oldState;
        }
        lock.lock();

        // Update _loop_ to reflect completion of iterations
        loop.activeWorkers--;
    }
}

PBRT_THREAD_LOCAL int ThreadIndex;
int MaxThreadIndex() {
    if (PbrtOptions.nThreads != 1) {
        // Launch worker threads if needed
        if (threads.size() == 0) {
            Assert(PbrtOptions.nThreads != 1);
            ThreadIndex = 0;
            for (int i = 0; i < NumSystemCores() - 1; ++i)
                threads.push_back(std::thread(workerThreadFunc, i + 1));
        }
    }
    return 1 + threads.size();
}

void ParallelFor2D(std::function<void(Point2i)> func, const Point2i &count) {
    if (PbrtOptions.nThreads == 1) {
        for (int y = 0; y < count.y; ++y)
            for (int x = 0; x < count.x; ++x) func(Point2i(x, y));
        return;
    }
    // Launch worker threads if needed
    if (threads.size() == 0) {
        Assert(PbrtOptions.nThreads != 1);
        ThreadIndex = 0;
        for (int i = 0; i < NumSystemCores() - 1; ++i)
            threads.push_back(std::thread(workerThreadFunc, i + 1));
    }

    ParallelForLoop loop(std::move(func), count, CurrentProfilerState());
    {
        std::lock_guard<std::mutex> lock(workListMutex);
        loop.next = workList;
        workList = &loop;
    }

    std::unique_lock<std::mutex> lock(workListMutex);
    workListCondition.notify_all();

    // Help out with parallel loop iterations in the current thread
    while (!loop.Finished()) {
        // Run a chunk of loop iterations for _loop_

        // Find the set of loop iterations to run next
        int64_t indexStart = loop.nextIndex;
        int64_t indexEnd = std::min(indexStart + loop.chunkSize, loop.maxIndex);

        // Update _loop_ to reflect iterations this thread will run
        loop.nextIndex = indexEnd;
        if (loop.nextIndex == loop.maxIndex) workList = loop.next;
        loop.activeWorkers++;

        // Run loop indices in _[indexStart, indexEnd)_
        lock.unlock();
        for (int64_t index = indexStart; index < indexEnd; ++index) {
            int oldState = profilerState;
            profilerState = loop.profilerState;
            if (loop.func1D) {
                loop.func1D(index);
            }
            // Handle other types of loops
            else {
                Assert(loop.func2D);
                loop.func2D(Point2i(index % loop.nX, index / loop.nX));
            }
            profilerState = oldState;
        }
        lock.lock();

        // Update _loop_ to reflect completion of iterations
        loop.activeWorkers--;
    }
}

int NumSystemCores() {
    if (PbrtOptions.nThreads > 0) return PbrtOptions.nThreads;
    return std::max(1u, std::thread::hardware_concurrency());
}

void TerminateWorkerThreads() {
    if (threads.size() == 0) return;

    {
        std::lock_guard<std::mutex> lock(workListMutex);
        shutdownThreads = true;
        workListCondition.notify_all();
    }

    for (std::thread &thread : threads) thread.join();
    threads.erase(threads.begin(), threads.end());
    shutdownThreads = false;
}
