
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

// accelerators/kdtreeaccel.cpp*
#include "accelerators/kdtreeaccel.h"
#include "paramset.h"
#include "interaction.h"
#include <algorithm>

// KdTreeAccel Local Declarations
struct KdAccelNode {
    // KdAccelNode Methods
    void InitLeaf(int *primNums, int np, std::vector<int> *primitiveIndices);
    void InitInterior(int axis, int ac, Float s) {
        split = s;
        flags = axis;
        aboveChild |= (ac << 2);
    }
    Float SplitPos() const { return split; }
    int nPrimitives() const { return nPrims >> 2; }
    int SplitAxis() const { return flags & 3; }
    bool IsLeaf() const { return (flags & 3) == 3; }
    int AboveChild() const { return aboveChild >> 2; }
    union {
        Float split;                 // Interior
        int onePrimitive;            // Leaf
        int primitiveIndicesOffset;  // Leaf
    };

  private:
    union {
        int flags;       // Both
        int nPrims;      // Leaf
        int aboveChild;  // Interior
    };
};

enum class EdgeType { Start, End };
struct BoundEdge {
    // BoundEdge Public Methods
    BoundEdge() {}
    BoundEdge(Float t, int primNum, bool starting) : t(t), primNum(primNum) {
        type = starting ? EdgeType::Start : EdgeType::End;
    }
    bool operator<(const BoundEdge &e) const {
        if (t == e.t)
            return (int)type < (int)e.type;
        else
            return t < e.t;
    }
    Float t;
    int primNum;
    EdgeType type;
};

// KdTreeAccel Method Definitions
KdTreeAccel::KdTreeAccel(const std::vector<std::shared_ptr<Primitive>> &p,
                         int isectCost, int traversalCost, Float emptyBonus,
                         int maxPrims, int maxDepth)
    : isectCost(isectCost),
      traversalCost(traversalCost),
      maxPrims(maxPrims),
      emptyBonus(emptyBonus),
      primitives(p) {
    // Build kd-tree for accelerator
    nextFreeNode = nAllocedNodes = 0;
    if (maxDepth <= 0)
        maxDepth = std::round(8 + 1.3f * Log2Int(primitives.size()));

    // Compute bounds for kd-tree construction
    std::vector<Bounds3f> primBounds;
    primBounds.reserve(primitives.size());
    for (const std::shared_ptr<Primitive> &prim : primitives) {
        Bounds3f b = prim->WorldBound();
        bounds = Union(bounds, b);
        primBounds.push_back(b);
    }

    // Allocate working memory for kd-tree construction
    BoundEdge *edges[3];
    for (int i = 0; i < 3; ++i) edges[i] = new BoundEdge[2 * primitives.size()];
    std::unique_ptr<int[]> prims0(new int[primitives.size()]);
    std::unique_ptr<int[]> prims1(new int[(maxDepth + 1) * primitives.size()]);

    // Initialize _primNums_ for kd-tree construction
    std::unique_ptr<int[]> primNums(new int[primitives.size()]);
    for (size_t i = 0; i < primitives.size(); ++i) primNums[i] = i;

    // Start recursive construction of kd-tree
    buildTree(0, bounds, primBounds, primNums.get(), primitives.size(),
              maxDepth, edges, prims0.get(), prims1.get());

    // Free working memory for kd-tree construction
    for (int i = 0; i < 3; ++i) delete[] edges[i];
}

void KdAccelNode::InitLeaf(int *primNums, int np,
                           std::vector<int> *primitiveIndices) {
    flags = 3;
    nPrims |= (np << 2);
    // Store primitive ids for leaf node
    if (np == 0)
        onePrimitive = 0;
    else if (np == 1)
        onePrimitive = primNums[0];
    else {
        primitiveIndicesOffset = primitiveIndices->size();
        for (int i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
    }
}

KdTreeAccel::~KdTreeAccel() { FreeAligned(nodes); }

void KdTreeAccel::buildTree(int nodeNum, const Bounds3f &nodeBounds,
                            const std::vector<Bounds3f> &allPrimBounds,
                            int *primNums, int nPrimitives, int depth,
                            BoundEdge *edges[3], int *prims0, int *prims1,
                            int badRefines) {
    Assert(nodeNum == nextFreeNode);
    // Get next free node from _nodes_ array
    if (nextFreeNode == nAllocedNodes) {
        int nAlloc = std::max(2 * nAllocedNodes, 512);
        KdAccelNode *n = AllocAligned<KdAccelNode>(nAlloc);
        if (nAllocedNodes > 0) {
            memcpy(n, nodes, nAllocedNodes * sizeof(KdAccelNode));
            FreeAligned(nodes);
        }
        nodes = n;
        nAllocedNodes = nAlloc;
    }
    ++nextFreeNode;

    // Initialize leaf node if termination criteria met
    if (nPrimitives <= maxPrims || depth == 0) {
        nodes[nodeNum].InitLeaf(primNums, nPrimitives, &primitiveIndices);
        return;
    }

    // Initialize interior node and continue recursion

    // Choose split axis position for interior node
    int bestAxis = -1, bestOffset = -1;
    Float bestCost = Infinity;
    Float oldCost = isectCost * Float(nPrimitives);
    Float totalSA = nodeBounds.SurfaceArea();
    Float invTotalSA = 1.f / totalSA;
    Vector3f d = nodeBounds.pMax - nodeBounds.pMin;

    // Choose which axis to split along
    int axis = nodeBounds.MaximumExtent();
    int retries = 0;
retrySplit:

    // Initialize edges for _axis_
    for (int i = 0; i < nPrimitives; ++i) {
        int pn = primNums[i];
        const Bounds3f &bounds = allPrimBounds[pn];
        edges[axis][2 * i] = BoundEdge(bounds.pMin[axis], pn, true);
        edges[axis][2 * i + 1] = BoundEdge(bounds.pMax[axis], pn, false);
    }
    std::sort(&edges[axis][0], &edges[axis][2 * nPrimitives]);

    // Compute cost of all splits for _axis_ to find best
    int nBelow = 0, nAbove = nPrimitives;
    for (int i = 0; i < 2 * nPrimitives; ++i) {
        if (edges[axis][i].type == EdgeType::End) --nAbove;
        Float edget = edges[axis][i].t;
        if (edget > nodeBounds.pMin[axis] && edget < nodeBounds.pMax[axis]) {
            // Compute cost for split at _i_th edge

            // Compute child surface areas for split at _edget_
            int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
            Float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                 (edget - nodeBounds.pMin[axis]) *
                                     (d[otherAxis0] + d[otherAxis1]));
            Float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                 (nodeBounds.pMax[axis] - edget) *
                                     (d[otherAxis0] + d[otherAxis1]));
            Float pBelow = belowSA * invTotalSA;
            Float pAbove = aboveSA * invTotalSA;
            Float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0.f;
            Float cost =
                traversalCost +
                isectCost * (1.f - eb) * (pBelow * nBelow + pAbove * nAbove);

            // Update best split if this is lowest cost so far
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestOffset = i;
            }
        }
        if (edges[axis][i].type == EdgeType::Start) ++nBelow;
    }
    Assert(nBelow == nPrimitives && nAbove == 0);

    // Create leaf if no good splits were found
    if (bestAxis == -1 && retries < 2) {
        ++retries;
        axis = (axis + 1) % 3;
        goto retrySplit;
    }
    if (bestCost > oldCost) ++badRefines;
    if ((bestCost > 4.f * oldCost && nPrimitives < 16) || bestAxis == -1 ||
        badRefines == 3) {
        nodes[nodeNum].InitLeaf(primNums, nPrimitives, &primitiveIndices);
        return;
    }

    // Classify primitives with respect to split
    int n0 = 0, n1 = 0;
    for (int i = 0; i < bestOffset; ++i)
        if (edges[bestAxis][i].type == EdgeType::Start)
            prims0[n0++] = edges[bestAxis][i].primNum;
    for (int i = bestOffset + 1; i < 2 * nPrimitives; ++i)
        if (edges[bestAxis][i].type == EdgeType::End)
            prims1[n1++] = edges[bestAxis][i].primNum;

    // Recursively initialize children nodes
    Float tsplit = edges[bestAxis][bestOffset].t;
    Bounds3f bounds0 = nodeBounds, bounds1 = nodeBounds;
    bounds0.pMax[bestAxis] = bounds1.pMin[bestAxis] = tsplit;
    buildTree(nodeNum + 1, bounds0, allPrimBounds, prims0, n0, depth - 1, edges,
              prims0, prims1 + nPrimitives, badRefines);
    int aboveChild = nextFreeNode;
    nodes[nodeNum].InitInterior(bestAxis, aboveChild, tsplit);
    buildTree(aboveChild, bounds1, allPrimBounds, prims1, n1, depth - 1, edges,
              prims0, prims1 + nPrimitives, badRefines);
}

bool KdTreeAccel::Intersect(const Ray &ray, SurfaceInteraction *si) const {
    // Compute initial parametric range of ray inside kd-tree extent
    Float tMin, tMax;
    if (!bounds.IntersectP(ray, &tMin, &tMax)) {
        return false;
    }

    // Prepare to traverse kd-tree for ray
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    constexpr int maxTodo = 64;
    KdToDo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    bool hit = false;
    const KdAccelNode *node = &nodes[0];
    while (node != nullptr) {
        // Bail out if we found a hit closer than the current node
        if (ray.tMax < tMin) break;
        if (!node->IsLeaf()) {
            // Process kd-tree interior node

            // Compute parametric distance along ray to split plane
            int axis = node->SplitAxis();
            Float tPlane = (node->SplitPos() - ray.o[axis]) * invDir[axis];

            // Get node children pointers for ray
            const KdAccelNode *firstChild, *secondChild;
            int belowFirst =
                (ray.o[axis] < node->SplitPos()) ||
                (ray.o[axis] == node->SplitPos() && ray.d[axis] <= 0);
            if (belowFirst) {
                firstChild = node + 1;
                secondChild = &nodes[node->AboveChild()];
            } else {
                firstChild = &nodes[node->AboveChild()];
                secondChild = node + 1;
            }

            // Advance to next child node, possibly enqueue other child
            if (tPlane > tMax || tPlane <= 0)
                node = firstChild;
            else if (tPlane < tMin)
                node = secondChild;
            else {
                // Enqueue _secondChild_ in todo list
                todo[todoPos].node = secondChild;
                todo[todoPos].tMin = tPlane;
                todo[todoPos].tMax = tMax;
                ++todoPos;
                node = firstChild;
                tMax = tPlane;
            }
        } else {
            // Check for intersections inside leaf node
            int nPrimitives = node->nPrimitives();
            if (nPrimitives == 1) {
                const std::shared_ptr<Primitive> &p =
                    primitives[node->onePrimitive];
                // Check one primitive inside leaf node
                if (p->Intersect(ray, si)) hit = true;
            } else {
                for (int i = 0; i < nPrimitives; ++i) {
                    int index =
                        primitiveIndices[node->primitiveIndicesOffset + i];
                    const std::shared_ptr<Primitive> &p = primitives[index];
                    // Check one primitive inside leaf node
                    if (p->Intersect(ray, si)) hit = true;
                }
            }

            // Grab next node to process from todo list
            if (todoPos > 0) {
                --todoPos;
                node = todo[todoPos].node;
                tMin = todo[todoPos].tMin;
                tMax = todo[todoPos].tMax;
            } else
                break;
        }
    }
    return hit;
}

bool KdTreeAccel::IntersectP(const Ray &ray) const {
    // Compute initial parametric range of ray inside kd-tree extent
    Float tMin, tMax;
    if (!bounds.IntersectP(ray, &tMin, &tMax)) {
        return false;
    }

    // Prepare to traverse kd-tree for ray
    Vector3f invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    constexpr int maxTodo = 64;
    KdToDo todo[maxTodo];
    int todoPos = 0;
    const KdAccelNode *node = &nodes[0];
    while (node != nullptr) {
        if (node->IsLeaf()) {
            // Check for shadow ray intersections inside leaf node
            int nPrimitives = node->nPrimitives();
            if (nPrimitives == 1) {
                const std::shared_ptr<Primitive> &p =
                    primitives[node->onePrimitive];
                if (p->IntersectP(ray)) {
                    return true;
                }
            } else {
                for (int i = 0; i < nPrimitives; ++i) {
                    int primitiveIndex =
                        primitiveIndices[node->primitiveIndicesOffset + i];
                    const std::shared_ptr<Primitive> &prim =
                        primitives[primitiveIndex];
                    if (prim->IntersectP(ray)) {
                        return true;
                    }
                }
            }

            // Grab next node to process from todo list
            if (todoPos > 0) {
                --todoPos;
                node = todo[todoPos].node;
                tMin = todo[todoPos].tMin;
                tMax = todo[todoPos].tMax;
            } else
                break;
        } else {
            // Process kd-tree interior node

            // Compute parametric distance along ray to split plane
            int axis = node->SplitAxis();
            Float tPlane = (node->SplitPos() - ray.o[axis]) * invDir[axis];

            // Get node children pointers for ray
            const KdAccelNode *firstChild, *secondChild;
            int belowFirst =
                (ray.o[axis] < node->SplitPos()) ||
                (ray.o[axis] == node->SplitPos() && ray.d[axis] <= 0);
            if (belowFirst) {
                firstChild = node + 1;
                secondChild = &nodes[node->AboveChild()];
            } else {
                firstChild = &nodes[node->AboveChild()];
                secondChild = node + 1;
            }

            // Advance to next child node, possibly enqueue other child
            if (tPlane > tMax || tPlane <= 0)
                node = firstChild;
            else if (tPlane < tMin)
                node = secondChild;
            else {
                // Enqueue _secondChild_ in todo list
                todo[todoPos].node = secondChild;
                todo[todoPos].tMin = tPlane;
                todo[todoPos].tMax = tMax;
                ++todoPos;
                node = firstChild;
                tMax = tPlane;
            }
        }
    }
    return false;
}

std::shared_ptr<KdTreeAccel> CreateKdTreeAccelerator(
    const std::vector<std::shared_ptr<Primitive>> &prims, const ParamSet &ps) {
    int isectCost = ps.FindOneInt("intersectcost", 80);
    int travCost = ps.FindOneInt("traversalcost", 1);
    Float emptyBonus = ps.FindOneFloat("emptybonus", 0.5f);
    int maxPrims = ps.FindOneInt("maxprims", 1);
    int maxDepth = ps.FindOneInt("maxdepth", -1);
    return std::make_shared<KdTreeAccel>(prims, isectCost, travCost, emptyBonus,
                                         maxPrims, maxDepth);
}
