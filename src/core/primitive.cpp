
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

// core/primitive.cpp*
#include "primitive.h"
#include "light.h"
#include "interaction.h"
#include "stats.h"

// Primitive Method Definitions
Primitive::~Primitive() {}
const AreaLight *Aggregate::GetAreaLight() const {
    Severe(
        "Aggregate::GetAreaLight() method"
        "called; should have gone to GeometricPrimitive");
    return nullptr;
}

const Material *Aggregate::GetMaterial() const {
    Severe(
        "Aggregate::GetMaterial() method"
        "called; should have gone to GeometricPrimitive");
    return nullptr;
}

void Aggregate::ComputeScatteringFunctions(SurfaceInteraction *isect,
                                           MemoryArena &arena,
                                           TransportMode mode,
                                           bool allowMultipleLobes) const {
    Severe(
        "Aggregate::ComputeScatteringFunctions() method"
        "called; should have gone to GeometricPrimitive");
}

// TransformedPrimitive Method Definitions
bool TransformedPrimitive::Intersect(const Ray &r,
                                     SurfaceInteraction *isect) const {
    Transform InterpolatedPrimToWorld;
    PrimitiveToWorld.Interpolate(r.time, &InterpolatedPrimToWorld);
    Ray ray = Inverse(InterpolatedPrimToWorld)(r);
    if (!primitive->Intersect(ray, isect)) return false;
    r.tMax = ray.tMax;
    // Transform instance's intersection data to world space
    if (!InterpolatedPrimToWorld.IsIdentity()) {
        *isect = InterpolatedPrimToWorld(*isect);
    }
    Assert(Dot(isect->n, isect->shading.n) >= 0.);
    return true;
}

bool TransformedPrimitive::IntersectP(const Ray &r) const {
    Transform InterpolatedPrimToWorld;
    PrimitiveToWorld.Interpolate(r.time, &InterpolatedPrimToWorld);
    Transform InterpolatedWorldToPrim = Inverse(InterpolatedPrimToWorld);
    return primitive->IntersectP(InterpolatedWorldToPrim(r));
}

// GeometricPrimitive Method Definitions
Bounds3f GeometricPrimitive::WorldBound() const { return shape->WorldBound(); }

bool GeometricPrimitive::IntersectP(const Ray &r) const {
    return shape->IntersectP(r);
}

bool GeometricPrimitive::Intersect(const Ray &r, SurfaceInteraction *si) const {
    Float tHit;
    if (!shape->Intersect(r, &tHit, si)) return false;
    r.tMax = tHit;
    si->primitive = this;
    Assert(Dot(si->n, si->shading.n) >= 0.);
    // Initialize _SurfaceInteraction::mediumInterface_ after _Shape_
    // intersection
    if (mediumInterface.IsMediumTransition())
        si->mediumInterface = mediumInterface;
    else
        si->mediumInterface = MediumInterface(r.medium);
    return true;
}

const AreaLight *GeometricPrimitive::GetAreaLight() const {
    return areaLight.get();
}

const Material *GeometricPrimitive::GetMaterial() const {
    return material.get();
}

void GeometricPrimitive::ComputeScatteringFunctions(
    SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
    bool allowMultipleLobes) const {
    ProfilePhase p(Prof::ComputeScatteringFuncs);
    if (material)
        material->ComputeScatteringFunctions(isect, arena, mode,
                                             allowMultipleLobes);
    Assert(Dot(isect->n, isect->shading.n) >= 0.);
}
