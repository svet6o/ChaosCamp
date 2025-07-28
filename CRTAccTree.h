#pragma once

#include "CRTAABB.h"
#include "CRTTriangle.h"
#include "CRTRay.h"
#include "CRTMaterial.h"
#include <vector>
#include <thread>
#include <future>
#include <algorithm>
#include <limits>
#include <numeric>

// Forward declaration for intersection data
struct IntersectionData {
    int triangleIndex = -1;
    float u = 0.0f, v = 0.0f;
    CRTVector normal;
    CRTVector uv;
};

/// Node for the acceleration tree structure,
/// holding a box for the sub space the node represents and indices for the children sub spaces represented by nodes
struct CRTAccTreeNode {
    CRTAccTreeNode(const CRTAABB& CRTAABB, const int parentIdx, const int leftNodeIdx, const int rightNodeIdx, const std::vector<int>& triangles)
        : CRTAABB(CRTAABB), parent(parentIdx), triangles(triangles) {
        children[0] = leftNodeIdx;
        children[1] = rightNodeIdx;
    }

    /// Intersect the given ray with the triangles in the box, meaningful only for leaf nodes, the others do not store triangles
    void intersect(
        const CRTRay& ray,
        const float maxT,
        const std::vector<CRTTriangle>& sceneTriangles,
        const std::vector<CRTMaterial>& materials,
        float& t,
        float& minT,
        IntersectionData& data
    ) const {
        // Only leaf nodes have triangles to intersect
        if (triangles.empty()) return;
        
        for (int triIdx : triangles) {
            float hitT, hitU, hitV;
            CRTVector hitNormal, hitUV;
            
            if (sceneTriangles[triIdx].intersect(ray, hitT, hitU, hitV, hitNormal, hitUV)) {
                if (hitT > 1e-4f && hitT < maxT && hitT < minT) {
                    minT = hitT;
                    t = hitT;
                    data.triangleIndex = triIdx;
                    data.u = hitU;
                    data.v = hitV;
                    data.normal = hitNormal;
                    data.uv = hitUV;
                }
            }
        }
    }

    std::vector<int> triangles; ///< In case of a leaf node a list with the triangles for intersection in the box for the node, empty otherwise
    CRTAABB CRTAABB; ///< Axis aligned bounding box for the sub space this node represents
    int children[2]; ///< The left and right indices for the node's children, indexing in the big list for the tree with all the nodes
    int parent; ///< The index of the parent node for this node and its sub space
};

class CRTAccTree {
private:
    std::vector<CRTAccTreeNode> nodes;
    std::vector<CRTTriangle> sceneTriangles;
    int rootIdx;
    
    // Build parameters
    static constexpr int maxBoxTrianglesCount = 8;
    static constexpr int maxDepth = 20;
    static constexpr int axisCount = 3;
    
    // Threading parameters
    static constexpr int MIN_TRIANGLES_FOR_THREADING = 1000;
    static constexpr int MAX_DEPTH_FOR_THREADING = 4;

public:
    CRTAccTree() : rootIdx(-1) {}

        /// Add node to the tree and return its index
    int addNode(const CRTAABB& CRTAABB, int parentIdx, int child0Idx, int child1Idx, const std::vector<int>& triangles) {
        nodes.emplace_back(CRTAABB, parentIdx, child0Idx, child1Idx, triangles);
        return static_cast<int>(nodes.size() - 1);
    }
    
    /// Build acceleration tree from triangles following the specified algorithm
    void build(const std::vector<CRTTriangle>& triangles, int maxLeafSize = 8, int maxTreeDepth = 20) {
        sceneTriangles = triangles;
        if (triangles.empty()) return;
        
        // Gather all triangles in an array
        std::vector<int> allTriangles(triangles.size());
        std::iota(allTriangles.begin(), allTriangles.end(), 0);
        
        // Create CRTAABB for the scene
        CRTAABB sceneCRTAABB = CRTAABB::fromTriangles(sceneTriangles, allTriangles);
        
        // Create root node
        rootIdx = addNode(sceneCRTAABB, -1, -1, -1,std::vector<int>());
        
        // Recursively build the acceleration tree
        buildAccTree(rootIdx, 0, allTriangles);
    }
    

    
    /// Traverse tree and find closest intersection
    bool intersect(const CRTRay& ray, float& closestT, int& hitTriangleIdx, 
                   float& hitU, float& hitV, CRTVector& hitNormal, CRTVector& hitUV) const {
        if (rootIdx == -1 || nodes.empty()) return false;
        
        closestT = std::numeric_limits<float>::infinity();
        IntersectionData data;
        
        traverseNode(rootIdx, ray, closestT, data);
        
        if (data.triangleIndex != -1) {
            hitTriangleIdx = data.triangleIndex;
            hitU = data.u;
            hitV = data.v;
            hitNormal = data.normal;
            hitUV = data.uv;
            return true;
        }
        
        return false;
    }
    
    /// Fast shadow ray intersection (stops at first hit)
    bool intersectAny(const CRTRay& ray, float maxDistance) const {
        if (rootIdx == -1 || nodes.empty()) return false;
        return traverseNodeAny(rootIdx, ray, maxDistance);
    }
    
    /// Get triangle by index
    const CRTTriangle& getTriangle(int index) const {
        return sceneTriangles[index];
    }

private:
    /// Recursively build the acceleration tree following the specified algorithm
    void buildAccTree(int parentIdx, int depth, const std::vector<int>& triangles) {
        // If maximum depth reached or triangles.count <= maxBoxTrianglesCount:
        if (depth >= maxDepth || triangles.size() <= maxBoxTrianglesCount) {
            // accTree[parentIdx].triangles = triangles
            nodes[parentIdx].triangles = triangles;
            // Stop the recursion!
            return;
        }
        
        // Split the parent CRTAABB box in two halves, alternating the split axis:
        int splitAxis = depth % axisCount;
        auto [child0CRTAABB, child1CRTAABB] = splitCRTAABB(nodes[parentIdx].CRTAABB, splitAxis);
        
        std::vector<int> child0triangles, child1triangles;
        
        // For each triangle in triangles:
        for (int triangleIdx : triangles) {
            CRTAABB triangleCRTAABB = CRTAABB::fromTriangle(sceneTriangles[triangleIdx]);
            
            // If triangle.CRTAABB intersects child0CRTAABB:
            if (CRTAABBIntersects(triangleCRTAABB, child0CRTAABB)) {
                // child0triangles += triangle
                child0triangles.push_back(triangleIdx);
            }
            // If triangle.CRTAABB intersects child1CRTAABB:
            if (CRTAABBIntersects(triangleCRTAABB, child1CRTAABB)) {
                // child1triangles += triangle
                child1triangles.push_back(triangleIdx);
            }
        }
        
        // If child0Triangles.count > 0:
        if (!child0triangles.empty()) {
            // child0Idx = accTree.addNode(child0CRTAABB, -1, -1, [])
            int child0Idx = addNode(child0CRTAABB, parentIdx, -1, -1, std::vector<int>());
            // accTree[parentIdx].child[0] = child0Idx
            nodes[parentIdx].children[0] = child0Idx;
            // buildAccTree(child0Idx, depth + 1, child0Triangles)
            
            bool shouldThread = depth < MAX_DEPTH_FOR_THREADING && 
                               child0triangles.size() > MIN_TRIANGLES_FOR_THREADING;
            
            if (shouldThread && !child1triangles.empty()) {
                // Build child0 in parallel
                auto future = std::async(std::launch::async, [&]() {
                    buildAccTree(child0Idx, depth + 1, child0triangles);
                });
                
                // Build child1 on current thread
                if (!child1triangles.empty()) {
                    int child1Idx = addNode(child1CRTAABB, parentIdx, -1, -1, std::vector<int>());
                    nodes[parentIdx].children[1] = child1Idx;
                    buildAccTree(child1Idx, depth + 1, child1triangles);
                }
                
                future.wait(); // Wait for child0 to complete
            } else {
                // Build sequentially
                buildAccTree(child0Idx, depth + 1, child0triangles);
                
                // If child1Triangles.count > 0:
                if (!child1triangles.empty()) {
                    // child1Idx = accTree.addNode(child1CRTAABB, -1, -1, [])
                    int child1Idx = addNode(child1CRTAABB, parentIdx, -1, -1, std::vector<int>());
                    // accTree[parentIdx].child[1] = child1Idx
                    nodes[parentIdx].children[1] = child1Idx;
                    // buildAccTree(child1Idx, depth + 1, child1Triangles)
                    buildAccTree(child1Idx, depth + 1, child1triangles);
                }
            }
        } else if (!child1triangles.empty()) {
            // Only child1 has triangles
            int child1Idx = addNode(child1CRTAABB, parentIdx, -1, -1, std::vector<int>());
            nodes[parentIdx].children[1] = child1Idx;
            buildAccTree(child1Idx, depth + 1, child1triangles);
        }
    }
    
    /// CRTAABB split algorithm following the specification
    std::pair<CRTAABB, CRTAABB> splitCRTAABB(const CRTAABB& CRTAABBToSplit, int splitAxisIdx) const {
        // Calculate the middle point for the splitting:
        // mid = (CRTAABBToSplit.max[AASplitAxisIdx] - CRTAABBToSplit.min[AASplitAxisIdx]) / 2
        float axisMin, axisMax;
        if (splitAxisIdx == 0) {
            axisMin = CRTAABBToSplit.minBounds.getX();
            axisMax = CRTAABBToSplit.maxBounds.getX();
        } else if (splitAxisIdx == 1) {
            axisMin = CRTAABBToSplit.minBounds.getY();
            axisMax = CRTAABBToSplit.maxBounds.getY();
        } else {
            axisMin = CRTAABBToSplit.minBounds.getZ();
            axisMax = CRTAABBToSplit.maxBounds.getZ();
        }
        
        float mid = (axisMax - axisMin) / 2.0f;
        // splitPlaneCoordinate = CRTAABBToSplit.min[AASplitAxisIdx] + mid
        float splitPlaneCoordinate = axisMin + mid;
        
        // Create A and B to be the same as CRTAABBToSplit:
        CRTAABB A = CRTAABBToSplit; // A = CRTAABBToSplit
        CRTAABB B = CRTAABBToSplit; // B = CRTAABBToSplit
        
        // Update the maximum component of A for the splitting axis:
        // A.max[AASplitAxisIdx] = splitPlaneCoordinate
        if (splitAxisIdx == 0) {
            A.maxBounds = CRTVector(splitPlaneCoordinate, A.maxBounds.getY(), A.maxBounds.getZ());
        } else if (splitAxisIdx == 1) {
            A.maxBounds = CRTVector(A.maxBounds.getX(), splitPlaneCoordinate, A.maxBounds.getZ());
        } else {
            A.maxBounds = CRTVector(A.maxBounds.getX(), A.maxBounds.getY(), splitPlaneCoordinate);
        }
        
        // Update the minimum component of B for the splitting axis:
        // B.min[AASplitAxisIdx] = splitPlaneCoordinate
        if (splitAxisIdx == 0) {
            B.minBounds = CRTVector(splitPlaneCoordinate, B.minBounds.getY(), B.minBounds.getZ());
        } else if (splitAxisIdx == 1) {
            B.minBounds = CRTVector(B.minBounds.getX(), splitPlaneCoordinate, B.minBounds.getZ());
        } else {
            B.minBounds = CRTVector(B.minBounds.getX(), B.minBounds.getY(), splitPlaneCoordinate);
        }
        
        return {A, B};
    }
    
    /// Check if two CRTAABBs intersect
    bool CRTAABBIntersects(const CRTAABB& a, const CRTAABB& b) const {
        return (a.minBounds.getX() <= b.maxBounds.getX() && a.maxBounds.getX() >= b.minBounds.getX()) &&
               (a.minBounds.getY() <= b.maxBounds.getY() && a.maxBounds.getY() >= b.minBounds.getY()) &&
               (a.minBounds.getZ() <= b.maxBounds.getZ() && a.maxBounds.getZ() >= b.minBounds.getZ());
    }
    
    /// Traverse node for closest intersection
    void traverseNode(int nodeIdx, const CRTRay& ray, float& closestT, IntersectionData& data) const {
        if (nodeIdx == -1) return;
        
        const CRTAccTreeNode& node = nodes[nodeIdx];
        
        // Check if ray intersects node's bounding box
        float tMin, tMax;
        if (!node.CRTAABB.intersect(ray, tMin, tMax) || tMin > closestT) {
            return;
        }
        
        // If leaf node (has triangles)
        if (!node.triangles.empty()) {
            float minT = closestT;
            node.intersect(ray, closestT, sceneTriangles, std::vector<CRTMaterial>(), closestT, minT, data);
            closestT = minT;
        } else {
            // Traverse children
            // Test closer child first for early termination
            if (node.children[0] != -1 && node.children[1] != -1) {
                float leftTMin, leftTMax, rightTMin, rightTMax;
                bool leftHit = nodes[node.children[0]].CRTAABB.intersect(ray, leftTMin, leftTMax);
                bool rightHit = nodes[node.children[1]].CRTAABB.intersect(ray, rightTMin, rightTMax);
                
                if (leftHit && rightHit) {
                    if (leftTMin <= rightTMin) {
                        traverseNode(node.children[0], ray, closestT, data);
                        traverseNode(node.children[1], ray, closestT, data);
                    } else {
                        traverseNode(node.children[1], ray, closestT, data);
                        traverseNode(node.children[0], ray, closestT, data);
                    }
                } else if (leftHit) {
                    traverseNode(node.children[0], ray, closestT, data);
                } else if (rightHit) {
                    traverseNode(node.children[1], ray, closestT, data);
                }
            } else {
                // Traverse existing children
                if (node.children[0] != -1) traverseNode(node.children[0], ray, closestT, data);
                if (node.children[1] != -1) traverseNode(node.children[1], ray, closestT, data);
            }
        }
    }
    
    /// Fast traversal for shadow rays (any intersection)
    bool traverseNodeAny(int nodeIdx, const CRTRay& ray, float maxDistance) const {
        if (nodeIdx == -1) return false;
        
        const CRTAccTreeNode& node = nodes[nodeIdx];
        
        // Check if ray intersects node's bounding box
        if (!node.CRTAABB.intersectFast(ray)) {
            return false;
        }
        
        // If leaf node (has triangles)
        if (!node.triangles.empty()) {
            for (int triIdx : node.triangles) {
                float t, u, v;
                CRTVector normal, uv;
                
                if (sceneTriangles[triIdx].intersect(ray, t, u, v, normal, uv) && 
                    t > 1e-4f && t < maxDistance) {
                    return true;
                }
            }
            return false;
        } else {
            // Traverse children
            return (node.children[0] != -1 && traverseNodeAny(node.children[0], ray, maxDistance)) ||
                   (node.children[1] != -1 && traverseNodeAny(node.children[1], ray, maxDistance));
        }
    }
};
