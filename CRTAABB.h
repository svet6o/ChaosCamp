#pragma once

#include "CRTVector.h"
#include "CRTRay.h"
#include "CRTTriangle.h"
#include <vector>
#include <algorithm>
#include <limits>

class CRTAABB {
public:
    CRTVector minBounds;
    CRTVector maxBounds;

    CRTAABB() 
        : minBounds(std::numeric_limits<float>::max(), 
                   std::numeric_limits<float>::max(), 
                   std::numeric_limits<float>::max()),
          maxBounds(std::numeric_limits<float>::lowest(), 
                   std::numeric_limits<float>::lowest(), 
                   std::numeric_limits<float>::lowest()) {}

    CRTAABB(const CRTVector& min, const CRTVector& max) 
        : minBounds(min), maxBounds(max) {}

    // Fast ray-box intersection using slab method
    inline bool intersect(const CRTRay& ray, float& tMin, float& tMax) const {
        const CRTVector& origin = ray.getOrigin();
        const CRTVector& direction = ray.getDirection();
        
        float t1 = (minBounds.getX() - origin.getX()) / direction.getX();
        float t2 = (maxBounds.getX() - origin.getX()) / direction.getX();
        if (t1 > t2) std::swap(t1, t2);
        
        tMin = t1;
        tMax = t2;
        
        t1 = (minBounds.getY() - origin.getY()) / direction.getY();
        t2 = (maxBounds.getY() - origin.getY()) / direction.getY();
        if (t1 > t2) std::swap(t1, t2);
        
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        
        t1 = (minBounds.getZ() - origin.getZ()) / direction.getZ();
        t2 = (maxBounds.getZ() - origin.getZ()) / direction.getZ();
        if (t1 > t2) std::swap(t1, t2);
        
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        
        return tMax >= tMin && tMax > 0.0f;
    }

    // Fast ray-box intersection check without distance calculation
    inline bool intersectFast(const CRTRay& ray) const {
        const CRTVector& origin = ray.getOrigin();
        const CRTVector& direction = ray.getDirection();
        
        float t1, t2, tMin, tMax;
        
        t1 = (minBounds.getX() - origin.getX()) / direction.getX();
        t2 = (maxBounds.getX() - origin.getX()) / direction.getX();
        if (t1 > t2) std::swap(t1, t2);
        tMin = t1; tMax = t2;
        
        t1 = (minBounds.getY() - origin.getY()) / direction.getY();
        t2 = (maxBounds.getY() - origin.getY()) / direction.getY();
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMax < tMin) return false;
        
        t1 = (minBounds.getZ() - origin.getZ()) / direction.getZ();
        t2 = (maxBounds.getZ() - origin.getZ()) / direction.getZ();
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        
        return tMax >= tMin && tMax > 0.0f;
    }

    // Expand bounding box to include a point
    // This implements: boxMin[c] = min(boxMin[c], point[c])
    //                 boxMax[c] = max(boxMax[c], point[c])
    void expand(const CRTVector& point) {
        minBounds = minBounds.componentMin(point); // min for each component
        maxBounds = maxBounds.componentMax(point); // max for each component
    }

    // Expand bounding box to include another CRTAABB
    void expand(const CRTAABB& other) {
        minBounds = minBounds.componentMin(other.minBounds);
        maxBounds = maxBounds.componentMax(other.maxBounds);
    }

    // Create CRTAABB from triangle - follows the exact algorithm you described
    static CRTAABB fromTriangle(const CRTTriangle& triangle) {
        CRTAABB box; // Initialized with min = maxFloat, max = minFloat
        
        // For each vertex (3 vertices in triangle)
        box.expand(triangle.getVertex(0)); // boxMin[c] = min(boxMin[c], vertex[c])
        box.expand(triangle.getVertex(1)); // boxMax[c] = max(boxMax[c], vertex[c])
        box.expand(triangle.getVertex(2));
        
        return box;
    }

    // Create CRTAABB from list of triangles
    static CRTAABB fromTriangles(const std::vector<CRTTriangle>& triangles, 
                             const std::vector<int>& indices) {
        CRTAABB box;
        for (int idx : indices) {
            box.expand(CRTAABB::fromTriangle(triangles[idx]));
        }
        return box;
    }

    // Get surface area (used for SAH)
    float getSurfaceArea() const {
        CRTVector extent = maxBounds - minBounds;
        return 2.0f * (extent.getX() * extent.getY() + 
                      extent.getY() * extent.getZ() + 
                      extent.getZ() * extent.getX());
    }

    // Get center point
    CRTVector getCenter() const {
        return (minBounds + maxBounds) * 0.5f;
    }

    // Get extent (size)
    CRTVector getExtent() const {
        return maxBounds - minBounds;
    }

    // Get longest axis (0=X, 1=Y, 2=Z)
    int getLongestAxis() const {
        CRTVector extent = getExtent();
        if (extent.getX() >= extent.getY() && extent.getX() >= extent.getZ()) return 0;
        if (extent.getY() >= extent.getZ()) return 1;
        return 2;
    }

    // Check if box is valid
    bool isValid() const {
        return minBounds.getX() <= maxBounds.getX() &&
               minBounds.getY() <= maxBounds.getY() &&
               minBounds.getZ() <= maxBounds.getZ();
    }
};
