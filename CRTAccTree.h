#pragma once
#include <vector>
#include <stack>
#include <limits>
#include <algorithm>

#include "AABB.h"
#include "CRTTriangle.h"
#include "CRTRay.h"

// Hit info (same as original)
struct HitInfo {
    bool hit = false;
    float t  = std::numeric_limits<float>::infinity();
    float u  = 0.0f, v = 0.0f;
    int   triIndex = -1;
    CRTVector normal;
    CRTVector uv;
};

// Simple helper functions (cleaned up from original)
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
    if (tris.empty()) return AABB();
    
    AABB result = computeTriangleAABB_BVH(tris[0]);
    for (size_t i = 1; i < tris.size(); ++i) {
        result.expand(computeTriangleAABB_BVH(tris[i]));
    }
    return result;
}

// Simple AABB splitting
inline AABB splitAABB(const AABB& box, int axis, bool takeLowerHalf) {
    CRTVector center = box.center();
    AABB result = box;
    
    if (takeLowerHalf) {
        if (axis == 0) result.max = CRTVector(center.getX(), result.max.getY(), result.max.getZ());
        else if (axis == 1) result.max = CRTVector(result.max.getX(), center.getY(), result.max.getZ());
        else result.max = CRTVector(result.max.getX(), result.max.getY(), center.getZ());
    } else {
        if (axis == 0) result.min = CRTVector(center.getX(), result.min.getY(), result.min.getZ());
        else if (axis == 1) result.min = CRTVector(result.min.getX(), center.getY(), result.min.getZ());
        else result.min = CRTVector(result.min.getX(), result.min.getY(), center.getZ());
    }
    return result;
}

inline bool intersectAABB_AABB(const AABB& a, const AABB& b) {
    if (a.min.getX() > b.max.getX() || a.max.getX() < b.min.getX()) return false;
    if (a.min.getY() > b.max.getY() || a.max.getY() < b.min.getY()) return false;
    if (a.min.getZ() > b.max.getZ() || a.max.getZ() < b.min.getZ()) return false;
    return true;
}

// Acceleration Tree Node (same structure as original)
struct CRTAccTreeNode {
    AABB aabb;
    int  children[2] = {-1, -1};
    int  parent      = -1;
    std::vector<int> triIndices;

    CRTAccTreeNode() = default;
    CRTAccTreeNode(const AABB& box, int parentIdx)
        : aabb(box), parent(parentIdx) {}
};

// Simplified Acceleration Tree
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

        // Initialize triangle references and AABBs
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

    // Basic accessors
    int root() const { return 0; }
    const CRTAccTreeNode& getNode(int idx) const { return nodes[idx]; }

    // Main intersection function - simplified but efficient
    void intersectDFS(
        const CRTRay& ray,
        const std::vector<CRTTriangle>& triangles,
        HitInfo& outHit,
        float tMax = std::numeric_limits<float>::infinity()
    ) const
    {
        if (nodes.empty()) return;
        
        std::stack<int> nodeStack;
        nodeStack.push(root());

        while (!nodeStack.empty()) {
            int nodeIdx = nodeStack.top();
            nodeStack.pop();

            const CRTAccTreeNode& node = nodes[nodeIdx];

            // Test ray against node's AABB
            float tNear, tFar;
            if (!node.aabb.intersect(ray, tNear, tFar) || tNear > tMax)
                continue;

            // If this is a leaf node, test triangles
            if (!node.triIndices.empty()) {
                for (int triId : node.triIndices) {
                    float t, u, v;
                    CRTVector n, uv;
                    if (triangles[triId].intersect(ray, t, u, v, n, uv)) {
                        if (t > 1e-4f && t < outHit.t && t < tMax) {
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
                // Internal node - add children to stack
                if (node.children[0] != -1) nodeStack.push(node.children[0]);
                if (node.children[1] != -1) nodeStack.push(node.children[1]);
            }
        }
    }

    // Shadow ray test - optimized for speed
    bool shadowDFS(
        const CRTRay& ray,
        const std::vector<CRTTriangle>& triangles,
        float maxDist
    ) const
    {
        if (nodes.empty()) return false;
        
        std::stack<int> nodeStack;
        nodeStack.push(root());

        while (!nodeStack.empty()) {
            int nodeIdx = nodeStack.top();
            nodeStack.pop();

            const CRTAccTreeNode& node = nodes[nodeIdx];

            // Test ray against node's AABB
            float tNear, tFar;
            if (!node.aabb.intersect(ray, tNear, tFar) || tNear > maxDist)
                continue;

            // If this is a leaf node, test triangles
            if (!node.triIndices.empty()) {
                for (int triId : node.triIndices) {
                    float t;
                    // Use the faster shadow intersection
                    if (triangles[triId].intersectShadow(ray, t, maxDist)) {
                        return true; // Early exit on first hit
                    }
                }
            } else {
                // Internal node - add children to stack
                if (node.children[0] != -1) nodeStack.push(node.children[0]);
                if (node.children[1] != -1) nodeStack.push(node.children[1]);
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
        CRTAccTreeNode& node = nodes[nodeIdx];

        // Stop recursion if we've reached limits
        if (depth >= maxDepth || (int)node.triIndices.size() <= maxBoxTrianglesCount)
            return;

        // Choose split axis (simple round-robin)
        int axis = depth % 3;

        // Split the node's AABB
        AABB leftBox  = splitAABB(node.aabb, axis, true);
        AABB rightBox = splitAABB(node.aabb, axis, false);

        // Distribute triangles to left and right
        std::vector<int> leftTris;
        std::vector<int> rightTris;
        leftTris.reserve(node.triIndices.size());
        rightTris.reserve(node.triIndices.size());

        for (int triId : node.triIndices) {
            const AABB& triAABB = triAABBs[triId];
            if (intersectAABB_AABB(triAABB, leftBox))  leftTris.push_back(triId);
            if (intersectAABB_AABB(triAABB, rightBox)) rightTris.push_back(triId);
        }

        // Clear parent's triangle list (it's now an internal node)
        node.triIndices.clear();

        // Create left child if it has triangles
        if (!leftTris.empty()) {
            int leftIdx = addNode(leftBox, nodeIdx);
            nodes[nodeIdx].children[0] = leftIdx;
            nodes[leftIdx].triIndices  = std::move(leftTris);
            buildRecursive(triangles, leftIdx, depth + 1, maxDepth, maxBoxTrianglesCount);
        }

        // Create right child if it has triangles
        if (!rightTris.empty()) {
            int rightIdx = addNode(rightBox, nodeIdx);
            nodes[nodeIdx].children[1] = rightIdx;
            nodes[rightIdx].triIndices = std::move(rightTris);
            buildRecursive(triangles, rightIdx, depth + 1, maxDepth, maxBoxTrianglesCount);
        }
    }
};

// Simple wrapper functions (same as original)
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
