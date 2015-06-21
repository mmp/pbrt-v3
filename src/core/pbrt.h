
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

#ifndef PBRT_CORE_PBRT_H
#define PBRT_CORE_PBRT_H
#include "stdafx.h"

// core/pbrt.h*
#if defined(_WIN32) || defined(_WIN64)
#define PBRT_IS_WINDOWS
#elif defined(__linux__)
#define PBRT_IS_LINUX
#elif defined(__APPLE__)
#define PBRT_IS_OSX
#elif defined(__OpenBSD__)
#define PBRT_IS_OPENBSD
#endif
#if defined(_MSC_VER)
#define PBRT_IS_MSVC
#elif defined(__MINGW32__) // Defined for both 32 bit/64 bit MinGW
#define PBRT_IS_MINGW
#endif

// Global Include Files
#include <type_traits>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <limits>
#include <memory>
#include <cinttypes>
#include "error.h"
#if !defined(PBRT_IS_OSX) && !defined(PBRT_IS_OPENBSD)
#include <malloc.h>  // for _alloca, memalign
#endif
#if !defined(PBRT_IS_WINDOWS) && !defined(PBRT_IS_OSX) && \
    !defined(PBRT_IS_OPENBSD)
#include <alloca.h>
#endif
#include <assert.h>
#include <string.h>

// Platform-specific definitions
#include <stdint.h>
#if defined(PBRT_IS_MSVC)
#include <float.h>
#pragma warning(disable : 4305)  // double constant assigned to float
#pragma warning(disable : 4244)  // int -> float conversion
#pragma warning(disable : 4267)  // size_t -> unsigned int conversion
#endif

// Global Macros
#define ALLOCA(TYPE, COUNT) (TYPE *) alloca((COUNT) * sizeof(TYPE))
#ifdef MSC_VER
#define THREAD_LOCAL thread_local
#else
#define THREAD_LOCAL __thread
#endif

// Global Forward Declarations
class Scene;
class Integrator;
class SamplerIntegrator;
template <typename T>
class Vector2;
template <typename T>
class Vector3;
template <typename T>
class Point3;
template <typename T>
class Point2;
template <typename T>
class Normal3;
class Ray;
class RayDifferential;
template <typename T>
class Bounds2;
template <typename T>
class Bounds3;
class Transform;
struct Interaction;
class SurfaceInteraction;
class Shape;
class Primitive;
class GeometricPrimitive;
template <int nSpectrumSamples>
class CoefficientSpectrum;
class RGBSpectrum;
class SampledSpectrum;
typedef RGBSpectrum Spectrum;
// typedef SampledSpectrum Spectrum;
class Camera;
struct CameraSample;
class ProjectiveCamera;
class Sampler;
class Filter;
class Film;
class FilmTile;
class BxDF;
class BRDF;
class BTDF;
class BSDF;
class Material;
template <typename T>
class Texture;
class Medium;
class MediumInteraction;
struct MediumInterface;
class BSSRDF;
struct BSSRDFSample;
struct BSSRDFTable;
class Light;
class VisibilityTester;
class AreaLight;
struct Distribution1D;
class Distribution2D;
#ifdef PBRT_FLOAT_AS_DOUBLE
typedef double Float;
#else
typedef float Float;
#endif  // PBRT_FLOAT_AS_DOUBLE
class RNG;
class ProgressReporter;
class MemoryArena;
template <typename T, int logBlockSize = 2>
class BlockedArray;
struct Matrix4x4;
class ParamSet;
template <typename T>
struct ParamSetItem;
struct Options {
    Options() {
        nCores = 0;
        quickRender = quiet = openWindow = verbose = false;
        imageFile = "";
    }
    int nCores;
    bool quickRender;
    bool quiet, verbose;
    bool openWindow;
    std::string imageFile;
};

extern Options PbrtOptions;
class TextureParams;

// Global Constants
#ifdef _MSC_VER
#define MaxFloat std::numeric_limits<Float>::max()
#define Infinity std::numeric_limits<Float>::infinity()
#else
static constexpr Float MaxFloat = std::numeric_limits<Float>::max();
static constexpr Float Infinity = std::numeric_limits<Float>::infinity();
#endif
#ifdef _MSC_VER
#define MachineEpsilon (std::numeric_limits<Float>::epsilon() * 0.5)
#else
static constexpr Float MachineEpsilon =
    std::numeric_limits<Float>::epsilon() * 0.5;
#endif
const Float ShadowEpsilon = 0.0001f;
#ifdef M_PI
#undef M_PI
#endif
static const Float Pi = 3.14159265358979323846;
static const Float InvPi = 0.31830988618379067154;
static const Float Inv2Pi = 0.15915494309189533577;
static const Float Inv4Pi = 0.07957747154594766788;
static const Float PiOver2 = 1.57079632679489661923;
static const Float PiOver4 = 0.785398163397448309616;
#if defined(PBRT_IS_MSVC)
#define alloca _alloca
#endif
#ifndef PBRT_L1_CACHE_LINE_SIZE
#define PBRT_L1_CACHE_LINE_SIZE 64
#endif

// Global Inline Functions
inline uint32_t FloatToBits(float f) {
    uint32_t ui;
    memcpy(&ui, &f, sizeof(float));
    return ui;
}

inline float BitsToFloat(uint32_t ui) {
    float f;
    memcpy(&f, &ui, sizeof(uint32_t));
    return f;
}

inline uint64_t FloatToBits(double f) {
    uint64_t ui;
    memcpy(&ui, &f, sizeof(double));
    return ui;
}

inline double BitsToFloat(uint64_t ui) {
    double f;
    memcpy(&f, &ui, sizeof(uint64_t));
    return f;
}

inline float NextFloatUp(float v) {
    // Handle infinity and negative zero for _NextFloatUp()_
    if (std::isinf(v) && v > 0.) return v;
    if (v == -0.f) v = 0.f;

    // Advance _v_ to next higher float
    uint32_t ui = FloatToBits(v);
    if (v >= 0.)
        ++ui;
    else
        --ui;
    return BitsToFloat(ui);
}

inline float NextFloatDown(float v) {
    // Handle infinity and positive zero for _NextFloatDown()_
    if (std::isinf(v) && v < 0.) return v;
    if (v == 0.f) v = -0.f;
    uint32_t ui = FloatToBits(v);
    if (v > 0.)
        --ui;
    else
        ++ui;
    return BitsToFloat(ui);
}

inline double NextFloatUp(double v, int delta = 1) {
    if (std::isinf(v) && v > 0.) return v;
    if (v == -0.f) v = 0.f;
    uint64_t ui = FloatToBits(v);
    if (v >= 0.)
        ui += delta;
    else
        ui -= delta;
    return BitsToFloat(ui);
}

inline double NextFloatDown(double v, int delta = 1) {
    if (std::isinf(v) && v < 0.) return v;
    if (v == 0.f) v = -0.f;
    uint64_t ui = FloatToBits(v);
    if (v > 0.)
        ui -= delta;
    else
        ui += delta;
    return BitsToFloat(ui);
}

inline Float gamma(int n) {
    return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
}

inline bool AtomicCompareAndExchange(volatile int32_t *v, int32_t newValue,
                                     int32_t oldValue) {
#if defined(PBRT_IS_MSVC)
    return _InterlockedCompareExchange(reinterpret_cast<volatile long *>(v),
                                       newValue, oldValue) == oldValue;
#else
    return __sync_bool_compare_and_swap(v, oldValue, newValue);
#endif
}

inline bool AtomicCompareAndExchange(volatile int64_t *v, int64_t newValue,
                                     int64_t oldValue) {
#if defined(PBRT_IS_MSVC)
    return _InterlockedCompareExchange64(
               reinterpret_cast<volatile __int64 *>(v), newValue, oldValue) ==
           oldValue;
#else
    return __sync_bool_compare_and_swap(v, oldValue, newValue);
#endif
}

inline float AtomicAdd(volatile float *dst, float delta) {
    union bits {
        float f;
        int32_t i;
    };
    bits oldVal, newVal;
    do {
#if defined(__i386__) || defined(__amd64__)
        __asm__ __volatile__("pause\n");
#endif
        oldVal.f = *dst;
        newVal.f = oldVal.f + delta;
    } while (
        !AtomicCompareAndExchange((volatile int32_t *)dst, newVal.i, oldVal.i));
    return newVal.f;
}

inline double AtomicAdd(volatile double *dst, double delta) {
    union bits {
        double f;
        int64_t i;
    };
    bits oldVal, newVal;
    do {
#if defined(__i386__) || defined(__amd64__)
        __asm__ __volatile__("pause\n");
#endif
        oldVal.f = *dst;
        newVal.f = oldVal.f + delta;
    } while (
        !AtomicCompareAndExchange((volatile int64_t *)dst, newVal.i, oldVal.i));
    return newVal.f;
}

template <typename T, typename U, typename V>
inline T Clamp(T val, U low, V high) {
    if (val < low)
        return low;
    else if (val > high)
        return high;
    else
        return val;
}

template <typename T>
inline T Mod(T a, T b) {
    T result = a - (a / b) * b;
    return (T)((result < 0) ? result + b : result);
}

template <>
inline Float Mod(Float a, Float b) {
    return std::fmod(a, b);
}

inline Float Radians(Float deg) { return (Pi / (Float)180) * deg; }

inline Float Degrees(Float rad) { return ((Float)180 / Pi) * rad; }

inline Float Log2(Float x) {
    const Float invLog2 = 1.442695040888963387004650940071;
    return std::log(x) * invLog2;
}

inline int Log2Int(uint32_t v) {
#if defined(_MSC_VER)
    DWORD lz = 0;
    if (_BitScanReverse(&lz, v)) return lz;
    return 0;
#else
    return 31 - __builtin_clz(v);
#endif
}

template <typename T>
inline bool IsPowerOf2(T v) {
    return v && !(v & (v - 1));
}

inline int32_t RoundUpPow2(int32_t v) {
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}

inline int64_t RoundUpPow2(int64_t v) {
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    return v + 1;
}

#if defined(_MSC_VER)
inline int CountTrailingZeros(uint32_t v) {
    unsigned long index;
    if (_BitScanForward(&index, v))
        return index;
    else
        return 32;
}

#else
inline int CountTrailingZeros(uint32_t v) { return __builtin_ctz(v); }

#endif
template <typename Predicate>
int FindInterval(int size, const Predicate &pred) {
    int first = 0, len = size;
    while (len > 0) {
        int half = len >> 1, middle = first + half;
        // Bisect range based on value of _pred_ at _middle_
        if (pred(middle)) {
            first = middle + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }
    return Clamp(first - 1, 0, size - 2);
}

#ifdef NDEBUG
#define Assert(expr) ((void)0)
#else
#define Assert(expr)                                                     \
    ((expr) ? (void)0 : Severe("Assertion \"%s\" failed in %s, line %d", \
                               #expr, __FILE__, __LINE__))
#endif  // NDEBUG
inline Float Lerp(Float t, Float v1, Float v2) { return (1 - t) * v1 + t * v2; }

inline bool Quadratic(Float a, Float b, Float c, Float *t0, Float *t1) {
    // Find quadratic discriminant
    double discrim = (double)b * (double)b - 4. * (double)a * (double)c;
    if (discrim < 0.) return false;
    double rootDiscrim = std::sqrt(discrim);

    // Compute quadratic _t_ values
    double q;
    if (b < 0)
        q = -.5 * (b - rootDiscrim);
    else
        q = -.5 * (b + rootDiscrim);
    *t0 = q / a;
    *t1 = c / q;
    if (*t0 > *t1) std::swap(*t0, *t1);
    return true;
}

#endif  // PBRT_CORE_PBRT_H
