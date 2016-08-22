
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

#ifndef PBRT_CORE_PBRT_H
#define PBRT_CORE_PBRT_H

// core/pbrt.h*
#include "port.h"

// Global Include Files
#include <type_traits>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "error.h"
#ifdef PBRT_HAVE_MALLOC_H
#include <malloc.h>  // for _alloca, memalign
#endif
#ifdef PBRT_HAVE_ALLOCA_H
#include <alloca.h>
#endif
#include <assert.h>
#include <string.h>

// Platform-specific definitions
#include <stdint.h>
#if defined(PBRT_IS_MSVC)
#include <float.h>
#include <intrin.h>
#pragma warning(disable : 4305)  // double constant assigned to float
#pragma warning(disable : 4244)  // int -> float conversion
#pragma warning(disable : 4843)  // double -> float conversion
#endif

// Global Macros
#define ALLOCA(TYPE, COUNT) (TYPE *) alloca((COUNT) * sizeof(TYPE))

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
class SeparableBSSRDF;
class TabulatedBSSRDF;
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
    int nThreads = 0;
    bool quickRender = false;
    bool quiet = false, verbose = false;
    bool cat = false, toPly = false;
    std::string imageFile;
};

extern Options PbrtOptions;
class TextureParams;

// Global Constants
#ifdef _MSC_VER
#define MaxFloat std::numeric_limits<Float>::max()
#define Infinity std::numeric_limits<Float>::infinity()
#else
static PBRT_CONSTEXPR Float MaxFloat = std::numeric_limits<Float>::max();
static PBRT_CONSTEXPR Float Infinity = std::numeric_limits<Float>::infinity();
#endif
#ifdef _MSC_VER
#define MachineEpsilon (std::numeric_limits<Float>::epsilon() * 0.5)
#else
static PBRT_CONSTEXPR Float MachineEpsilon =
    std::numeric_limits<Float>::epsilon() * 0.5;
#endif
const Float ShadowEpsilon = 0.0001f;
static const Float Pi = 3.14159265358979323846;
static const Float InvPi = 0.31830988618379067154;
static const Float Inv2Pi = 0.15915494309189533577;
static const Float Inv4Pi = 0.07957747154594766788;
static const Float PiOver2 = 1.57079632679489661923;
static const Float PiOver4 = 0.78539816339744830961;
static const Float Sqrt2 = 1.41421356237309504880;
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
    if (v >= 0)
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
    if (v > 0)
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

inline Float GammaCorrect(Float value) {
    if (value <= 0.0031308f) return 12.92f * value;
    return 1.055f * std::pow(value, (Float)(1.f / 2.4f)) - 0.055f;
}

inline Float InverseGammaCorrect(Float value) {
    if (value <= 0.04045f) return value * 1.f / 12.92f;
    return std::pow((value + 0.055f) * 1.f / 1.055f, (Float)2.4f);
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

inline Float Radians(Float deg) { return (Pi / 180) * deg; }

inline Float Degrees(Float rad) { return (180 / Pi) * rad; }

inline Float Log2(Float x) {
    const Float invLog2 = 1.442695040888963387004650940071;
    return std::log(x) * invLog2;
}

inline int Log2Int(uint32_t v) {
#if defined(PBRT_IS_MSVC)
    unsigned long lz = 0;
    if (_BitScanReverse(&lz, v)) return lz;
    return 0;
#else
    return 31 - __builtin_clz(v);
#endif
}

template <typename T>
inline constexpr bool IsPowerOf2(T v) {
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

inline int CountTrailingZeros(uint32_t v) {
#if defined(PBRT_IS_MSVC)
    unsigned long index;
    if (_BitScanForward(&index, v))
        return index;
    else
        return 32;
#else
    return __builtin_ctz(v);
#endif
}

template <typename Predicate>
int FindInterval(int size, const Predicate &pred) {
    int first = 0, len = size;
    while (len > 0) {
        int half = len >> 1, middle = first + half;
        // Bisect range based on value of _pred_ at _middle_
        if (pred(middle)) {
            first = middle + 1;
            len -= half + 1;
        } else
            len = half;
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
    double discrim = (double)b * (double)b - 4 * (double)a * (double)c;
    if (discrim < 0) return false;
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

inline Float ErfInv(Float x) {
    Float w, p;
    x = Clamp(x, -.99999f, .99999f);
    w = -std::log((1 - x) * (1 + x));
    if (w < 5) {
        w = w - 2.5f;
        p = 2.81022636e-08f;
        p = 3.43273939e-07f + p * w;
        p = -3.5233877e-06f + p * w;
        p = -4.39150654e-06f + p * w;
        p = 0.00021858087f + p * w;
        p = -0.00125372503f + p * w;
        p = -0.00417768164f + p * w;
        p = 0.246640727f + p * w;
        p = 1.50140941f + p * w;
    } else {
        w = std::sqrt(w) - 3;
        p = -0.000200214257f;
        p = 0.000100950558f + p * w;
        p = 0.00134934322f + p * w;
        p = -0.00367342844f + p * w;
        p = 0.00573950773f + p * w;
        p = -0.0076224613f + p * w;
        p = 0.00943887047f + p * w;
        p = 1.00167406f + p * w;
        p = 2.83297682f + p * w;
    }
    return p * x;
}

inline Float Erf(Float x) {
    // constants
    Float a1 = 0.254829592f;
    Float a2 = -0.284496736f;
    Float a3 = 1.421413741f;
    Float a4 = -1.453152027f;
    Float a5 = 1.061405429f;
    Float p = 0.3275911f;

    // Save the sign of x
    int sign = 1;
    if (x < 0) sign = -1;
    x = std::abs(x);

    // A&S formula 7.1.26
    Float t = 1 / (1 + p * x);
    Float y =
        1 -
        (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * std::exp(-x * x);

    return sign * y;
}

#endif  // PBRT_CORE_PBRT_H
