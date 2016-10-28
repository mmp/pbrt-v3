
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_PARALLEL_H
#define PBRT_CORE_PARALLEL_H

// core/parallel.h*
#include "pbrt.h"
#include "geometry.h"
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

// Parallel Declarations
class AtomicFloat {
  public:
    // AtomicFloat Public Methods
    explicit AtomicFloat(Float v = 0) { bits = FloatToBits(v); }
    operator Float() const { return BitsToFloat(bits); }
    Float operator=(Float v) {
        bits = FloatToBits(v);
        return v;
    }
    void Add(Float v) {
#ifdef PBRT_FLOAT_AS_DOUBLE
        uint64_t oldBits = bits, newBits;
#else
        uint32_t oldBits = bits, newBits;
#endif
        do {
            newBits = FloatToBits(BitsToFloat(oldBits) + v);
        } while (!bits.compare_exchange_weak(oldBits, newBits));
    }

  private:
// AtomicFloat Private Data
#ifdef PBRT_FLOAT_AS_DOUBLE
    std::atomic<uint64_t> bits;
#else
    std::atomic<uint32_t> bits;
#endif
};

// Simple one-use barrier; ensures that multiple threads all reach a
// particular point of execution before allowing any of them to proceed
// past it.
//
// Note: this should be heap allocated and managed with a shared_ptr, where
// all threads that use it are passed the shared_ptr. This ensures that
// memory for the Barrier won't be freed until all threads have
// successfully cleared it.
class Barrier {
  public:
    Barrier(int count) : count(count) { CHECK_GT(count, 0); }
    ~Barrier() { CHECK_EQ(count, 0); }
    void Wait();

  private:
    std::mutex mutex;
    std::condition_variable cv;
    int count;
};

void ParallelFor(std::function<void(int64_t)> func, int64_t count,
                 int chunkSize = 1);
extern PBRT_THREAD_LOCAL int ThreadIndex;
void ParallelFor2D(std::function<void(Point2i)> func, const Point2i &count);
int MaxThreadIndex();
int NumSystemCores();

void ParallelInit();
void ParallelCleanup();
void MergeWorkerThreadStats();

#endif  // PBRT_CORE_PARALLEL_H
