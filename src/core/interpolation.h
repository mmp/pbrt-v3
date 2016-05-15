
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

#ifndef PBRT_CORE_INTERPOLATION_H
#define PBRT_CORE_INTERPOLATION_H

// core/interpolation.h*
#include "pbrt.h"

// Spline Interpolation Declarations
Float CatmullRom(int size, const Float *nodes, const Float *values, Float x);
bool CatmullRomWeights(int size, const Float *nodes, Float x, int *offset,
                       Float *weights);
Float SampleCatmullRom(int size, const Float *nodes, const Float *f,
                       const Float *cdf, Float sample, Float *fval = nullptr,
                       Float *pdf = nullptr);
Float SampleCatmullRom2D(int size1, int size2, const Float *nodes1,
                         const Float *nodes2, const Float *values,
                         const Float *cdf, Float alpha, Float sample,
                         Float *fval = nullptr, Float *pdf = nullptr);
Float IntegrateCatmullRom(int n, const Float *nodes, const Float *values,
                          Float *cdf);
Float InvertCatmullRom(int n, const Float *x, const Float *values, Float u);

// Fourier Interpolation Declarations
Float Fourier(const Float *a, int m, double cosPhi);
Float SampleFourier(const Float *ak, const Float *recip, int m, Float u,
                    Float *pdf, Float *phiPtr);

#endif  // PBRT_CORE_INTERPOLATION_H
