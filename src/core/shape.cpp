
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

// core/shape.cpp*
#include "shape.h"
#include "stats.h"

// Shape Method Definitions
Shape::~Shape() {}

STAT_COUNTER("Scene/Shapes created", nShapesCreated);
Shape::Shape(const Transform *ObjectToWorld, const Transform *WorldToObject,
             bool ReverseOrientation)
    : ObjectToWorld(ObjectToWorld),
      WorldToObject(WorldToObject),
      ReverseOrientation(ReverseOrientation),
      TransformSwapsHandedness(ObjectToWorld->SwapsHandedness()) {
    ++nShapesCreated;
}

Bounds3f Shape::WorldBound() const { return (*ObjectToWorld)(ObjectBound()); }

Float Shape::Pdf(const Interaction &ref, const Vector3f &wi) const {
    // Intersect sample ray with area light geometry
    SurfaceInteraction isectLight;
    Ray ray = ref.SpawnRay(wi);
    Float tHit;
    if (!Intersect(ray, &tHit, &isectLight, false)) return 0;

    // Convert light sample weight to solid angle measure
    Float pdf = DistanceSquared(ref.p, isectLight.p) /
                (AbsDot(isectLight.n, -wi) * Area());
    if (std::isinf(pdf)) pdf = 0.f;
    return pdf;
}
