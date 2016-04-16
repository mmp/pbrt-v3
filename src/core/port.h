
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

#ifndef PBRT_CORE_PORT_H
#define PBRT_CORE_PORT_H
#include "stdafx.h"

#if defined(_WIN32) || defined(_WIN64)
  #define PBRT_IS_WINDOWS
  #if defined(__MINGW32__)  // Defined for both 32 bit/64 bit MinGW
    #define PBRT_IS_MINGW
  #elif defined(_MSC_VER)
    #define PBRT_IS_MSVC
  #endif
#elif defined(__linux__)
  #define PBRT_IS_LINUX
#elif defined(__APPLE__)
  #define PBRT_IS_OSX
#elif defined(__OpenBSD__)
  #define PBRT_IS_OPENBSD
#elif defined(__FreeBSD__)
  #define PBRT_IS_FREEBSD
#endif

#if defined(_MSC_VER) && _MSC_VER == 1900
   #define PBRT_IS_MSVC2015
#endif

#if defined(_MSC_VER) && _MSC_VER == 1800
  #define PBRT_IS_MSVC2013
#endif

///////////////////////////////////////////////////////////////////////////
// Now, use what we've figured out to do #defines for features and to do
// various target-specific workarounds.

#if !defined(PBRT_IS_MSVC)
  #define PBRT_HAVE_HEX_FP_CONSTANTS
  #define PBRT_HAVE_BITVEC_CONSTANTS
#endif

#if defined(PBRT_IS_LINUX) || defined(PBRT_IS_WINDOWS)
  #define PBRT_HAVE_MALLOC_H
#endif

#ifdef PBRT_IS_LINUX
  #define PBRT_HAVE_ALLOCA_H
#endif

#if !defined(PBRT_IS_MSVC)
  #define PBRT_THREAD_LOCAL __thread
#else
  #define PBRT_THREAD_LOCAL thread_local
#endif

#ifdef PBRT_IS_MSVC2013
  #define PBRT_CONSTEXPR const
#else
  #define PBRT_CONSTEXPR constexpr
#endif

#if !defined(PBRT_IS_MSVC2013)
  #define PBRT_HAVE_ALIGNAS
#endif

#ifdef PBRT_IS_MSVC2013
  #define snprintf _snprintf
#endif

#endif // PBRT_CORE_PORT_H
