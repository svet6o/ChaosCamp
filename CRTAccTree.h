#pragma once
#include <vector>
#include <stack>
#include <limits>
#include <algorithm>

#include "AABB.h"
#include "CRTTriangle.h"
#include "CRTRay.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTSettings.h"
#include "CRTColor.h"

// -----------------------------------------------------
// Hit info
// -----------------------------------------------------
struct HitInfo {
    bool hit = false;
    float t  = std::numeric_limits<float>::infinity();
    float u  = 0.0f, v = 0.0f;
    int   triIndex = -1;
    CRTVector normal;
    CRTVector uv;
};

// -----------------------------------------------------
// Helpers
// -----------------------------------------------------
inline float getComp(const CRTVector& v, int axis) {
    return (axis == 0) ? v.getX() : (axis == 1) ? v.getY() : v.getZ();
}

inline CRTVector setComp(const CRTVector& v, int axis, float value) {
    if (axis == 0) return CRTVector(value, v.getY(), v.getZ());
    if (axis == 1) return CRTVector(v.getX(), value, v.getZ());
    return CRTVector(v.getX(), v.getY(), value);
}

inline AABB splitAABB(const AABB& box, int axis, bool takeLowerHalf) {
    float mn = getComp(box.min, axis);
    float mx = getComp(box.max, axis);
    float split = 0.5f * (mn + mx);

    AABB out = box;
    if (takeLowerHalf) {
        out.max = setComp(out.max, axis, split);
    } else {
        out.min = setComp(out.min, axis, split);
    }
    return out;
}

inline bool intersectAABB_AABB(const AABB& a, const AABB& b) {
    if (a.min.getX() > b.max.getX() || a.max.getX() < b.min.getX()) return false;
    if (a.min.getY() > b.max.getY() || a.max.getY() < b.min.getY()) return false;
    if (a.min.getZ() > b.max.getZ() || a.max.getZ() < b.min.getZ()) return false;
    return true;
}

inline AABB computeTriangleAABB_BVH(const CRTTriangle& tri) {
    const CRTVector& v0 = tri.getVertex(0);
    const CRTVector& v1 = tri.getVertex(1);
    const CRTVector& v2 = tri.getVertex(2);

    CRTVector minV(
        std::min({v0.getX(), v1.getX(), v2.getX()}),
        std::min({v0.getY(), v1.getY(), v2.getY()}),
        std::min({v0.getZ(), v1.getZ(), v2.getZ()})
    );
    CRTVector maxV(
        std::max({v0.getX(), v1.getX(), v2.getX()}),
        std::max({v0.getY(), v1.getY(), v2.getY()}),
        std::max({v0.getZ(), v1.getZ(), v2.getZ()})
    );
    return AABB(minV, maxV);
}

inline AABB computeSceneAABB_BVH(const std::vector<CRTTriangle>& tris) {
    CRTVector minV(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()
    );
    CRTVector maxV(
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()
    );

    for (const auto& tri : tris) {
        for (int i = 0; i < 3; ++i) {
            const CRTVector& v = tri.getVertex(i);
            minV = CRTVector(
                std::min(minV.getX(), v.getX()),
                std::min(minV.getY(), v.getY()),
                std::min(minV.getZ(), v.getZ())
            );
            maxV = CRTVector(
                std::max(maxV.getX(), v.getX()),
                std::max(maxV.getY(), v.getY()),
                std::max(maxV.getZ(), v.getZ())
            );
        }
    }
    return AABB(minV, maxV);
}

// -----------------------------------------------------
// Acceleration Tree Node
// -----------------------------------------------------
struct CRTAccTreeNode {
    AABB aabb;
    int  children[2] = {-1, -1};
    int  parent      = -1;
    std::vector<int> triIndices; // индекси към глобалния масив триъгълници

    CRTAccTreeNode() = default;
    CRTAccTreeNode(const AABB& box, int parentIdx)
        : aabb(box), parent(parentIdx) {}
};

// -----------------------------------------------------
// Acceleration Tree (BVH-like)
// -----------------------------------------------------
class CRTAccTree {
public:
    void build(const std::vector<CRTTriangle>& triangles,
               int maxDepth = 32,
               int maxBoxTrianglesCount = 8)
    {
        triRefs.clear();
        triRefs.reserve(triangles.size());
        triAABBs.clear();
        triAABBs.reserve(triangles.size());

        for (int i = 0; i < (int)triangles.size(); ++i) {
            triRefs.push_back(i);
            triAABBs.push_back(computeTriangleAABB_BVH(triangles[i]));
        }

        nodes.clear();
        AABB rootBox = computeSceneAABB_BVH(triangles);
        int rootIdx = addNode(rootBox, -1);
        nodes[rootIdx].triIndices = triRefs;

        buildRecursive(triangles, rootIdx, 0, maxDepth, maxBoxTrianglesCount);
    }

    // достъп
    int root() const { return 0; }
    int size() const { return (int)nodes.size(); }

    inline int getRootIndex() const { return root(); }

    const CRTAccTreeNode& getNode(int idx) const { return nodes[idx]; }
    CRTAccTreeNode&       getNode(int idx)       { return nodes[idx]; }

    // Най-близък удар (DFS)
    void intersectDFS(
        const CRTRay& ray,
        const std::vector<CRTTriangle>& triangles,
        HitInfo& outHit,
        float tMax = std::numeric_limits<float>::infinity()
    ) const
    {
        std::stack<int> st;
        st.push(root());

        while (!st.empty()) {
            int idx = st.top();
            st.pop();

            float tNear, tFar;
            if (!nodes[idx].aabb.intersect(ray, tNear, tFar) || tNear > tMax)
                continue;

            if (!nodes[idx].triIndices.empty()) {
                for (int triId : nodes[idx].triIndices) {
                    float t, u, v;
                    CRTVector n, uv;
                    if (triangles[triId].intersect(ray, t, u, v, n, uv)) {
                        if (t > 0.0f && t < outHit.t && t < tMax) {
                            outHit.hit      = true;
                            outHit.t        = t;
                            outHit.u        = u;
                            outHit.v        = v;
                            outHit.triIndex = triId;
                            outHit.normal   = n;
                            outHit.uv       = uv;
                        }
                    }
                }
            } else {
                if (nodes[idx].children[0] != -1) st.push(nodes[idx].children[0]);
                if (nodes[idx].children[1] != -1) st.push(nodes[idx].children[1]);
            }
        }
    }

    // Shadow DFS – early out
    bool shadowDFS(
        const CRTRay& ray,
        const std::vector<CRTTriangle>& triangles,
        float maxDist
    ) const
    {
        std::stack<int> st;
        st.push(root());

        while (!st.empty()) {
            int idx = st.top(); st.pop();

            float tNear, tFar;
            if (!nodes[idx].aabb.intersect(ray, tNear, tFar) || tNear > maxDist)
                continue;

            if (!nodes[idx].triIndices.empty()) {
                for (int triId : nodes[idx].triIndices) {
                    float t, u, v;
                    CRTVector n, uv;
                    if (triangles[triId].intersect(ray, t, u, v, n, uv)) {
                        if (t > 0.0f && t < maxDist) {
                            return true;
                        }
                    }
                }
            } else {
                if (nodes[idx].children[0] != -1) st.push(nodes[idx].children[0]);
                if (nodes[idx].children[1] != -1) st.push(nodes[idx].children[1]);
            }
        }
        return false;
    }

private:
    std::vector<CRTAccTreeNode> nodes;
    std::vector<int>            triRefs;
    std::vector<AABB>           triAABBs;

    int addNode(const AABB& box, int parentIdx) {
        nodes.emplace_back(box, parentIdx);
        return (int)nodes.size() - 1;
    }

    void buildRecursive(const std::vector<CRTTriangle>& triangles,
                        int nodeIdx,
                        int depth,
                        int maxDepth,
                        int maxBoxTrianglesCount)
    {
        auto& node = nodes[nodeIdx];

        if (depth >= maxDepth || (int)node.triIndices.size() <= maxBoxTrianglesCount)
            return;

        int axis = depth % 3;

        AABB leftBox  = splitAABB(node.aabb, axis, true);
        AABB rightBox = splitAABB(node.aabb, axis, false);

        std::vector<int> leftTris;
        std::vector<int> rightTris;
        leftTris.reserve(node.triIndices.size());
        rightTris.reserve(node.triIndices.size());

        for (int triId : node.triIndices) {
            const AABB& ta = triAABBs[triId];
            if (intersectAABB_AABB(ta, leftBox))  leftTris.push_back(triId);
            if (intersectAABB_AABB(ta, rightBox)) rightTris.push_back(triId);
        }

        node.triIndices.clear();

        if (!leftTris.empty()) {
            int leftIdx = addNode(leftBox, nodeIdx);
            nodes[nodeIdx].children[0] = leftIdx;
            nodes[leftIdx].triIndices  = std::move(leftTris);
            buildRecursive(triangles, leftIdx, depth + 1, maxDepth, maxBoxTrianglesCount);
        }

        if (!rightTris.empty()) {
            int rightIdx = addNode(rightBox, nodeIdx);
            nodes[nodeIdx].children[1] = rightIdx;
            nodes[rightIdx].triIndices = std::move(rightTris);
            buildRecursive(triangles, rightIdx, depth + 1, maxDepth, maxBoxTrianglesCount);
        }
    }
};

// -----------------------------------------------------
// Удобни wrapper-и
// -----------------------------------------------------
inline bool closestHit_BVH(
    const CRTRay& ray,
    const CRTAccTree& acc,
    const std::vector<CRTTriangle>& tris,
    HitInfo& outHit,
    float tMax = std::numeric_limits<float>::infinity()
) {
    outHit = HitInfo{};
    acc.intersectDFS(ray, tris, outHit, tMax);
    return outHit.hit;
}

inline bool isOccluded_BVH(
    const CRTRay& ray,
    const CRTAccTree& acc,
    const std::vector<CRTTriangle>& tris,
    float maxDist
) {
    return acc.shadowDFS(ray, tris, maxDist);
}
