
#ifndef BILINEAR_PATCH_H
#define BILINEAR_PATCH_H

#include "pbrt.h"
#include "shape.h"

#include <memory>
#include <vector>

namespace pbrt {

class BilinearPatch;

class BilinearPatchMesh {
  public:
    BilinearPatchMesh(const Transform &worldFromObject, bool reverseOrientation,
                      std::vector<int> vertexIndices, std::vector<Point3f> p,
                      std::vector<Normal3f> N, std::vector<Point2f> uv);

    static std::vector<std::shared_ptr<Shape>> Create(const Transform *worldFromObject,
                                                      const Transform *objectFromWorld,
                                                      bool reverseOrientation,
                                                      const ParamSet &dict);

    SurfaceInteraction InteractionFromIntersection(int patchIndex, const Point2f &uvHit,
                                                   Float time, const Vector3f &wo, const BilinearPatch *patch,
                                                   Transform *worldFromInstance = nullptr) const;

    bool reverseOrientation, transformSwapsHandedness;
    int nPatches, nVertices;
    std::vector<int> vertexIndices;
    std::vector<Point3f> p;
    std::vector<Normal3f> n;
    std::vector<Point2f> uv;

    static std::vector<const BilinearPatchMesh *> allMeshes;
};

struct BilinearIntersection {
    Point2f uv;
    Float t;
};

inline Vector3f Lerp(Float t, Vector3f a, Vector3f b) {
    return (1.f - t) * a + t * b;
}

class BilinearPatch : public Shape {
  public:
    BilinearPatch(const Transform *ObjectToWorld, const Transform *WorldToObject,
                  bool reverseOrientation, int meshIndex, int blpIndex);

    Bounds3f WorldBound() const;
    Bounds3f ObjectBound() const;
    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlpha = true) const;
    bool IntersectP(const Ray &ray, bool testAlpha = true) const;
    Float Area() const;

    Interaction Sample(const Point2f &u, Float *pdf) const;
    Float Pdf(const Interaction &) const;
    Interaction Sample(const Interaction &ref, const Point2f &u,
                       Float *pdf) const;
    Float Pdf(const Interaction &ref, const Vector3f &wi) const;

    static bool Intersect(const Ray &ray,
                          const Point3f &p00, const Point3f &p10,
                          const Point3f &p01, const Point3f &p11,
                          BilinearIntersection *bi);

private:
    std::array<std::array<Float, 3>, 3> biquadraticBSDFWeights(const Interaction &ref) const;

    bool IsQuad() const;

    const BilinearPatchMesh *&GetMesh() const {
        return BilinearPatchMesh::allMeshes[meshIndex];
    }

    // BilinearPatch Private Data
    int meshIndex, blpIndex;
    Float area;
};

};

#endif
