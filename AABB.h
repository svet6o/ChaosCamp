#pragma once
#include "CRTVector.h"
#include "CRTRay.h"
#include <limits>
#include <algorithm>

struct AABB {
    CRTVector min, max;

    AABB() = default;
    AABB(const CRTVector& mn, const CRTVector& mx) : min(mn), max(mx) {}

    // Fixed ray-AABB intersection using slab method (main bug fix)
    bool intersect(const CRTRay& ray, float& tNear, float& tFar) const {
        const CRTVector& orig = ray.getOrigin();
        const CRTVector& dir = ray.getDirection();
        
        // X slab
        float t1 = (min.getX() - orig.getX()) / dir.getX();
        float t2 = (max.getX() - orig.getX()) / dir.getX();
        
        if (t1 > t2) std::swap(t1, t2);
        tNear = t1;
        tFar = t2;
        
        // Y slab
        t1 = (min.getY() - orig.getY()) / dir.getY();
        t2 = (max.getY() - orig.getY()) / dir.getY();
        
        if (t1 > t2) std::swap(t1, t2);
        
        if (tNear > t2 || t1 > tFar) return false;
        tNear = std::max(tNear, t1);
        tFar = std::min(tFar, t2);
        
        // Z slab
        t1 = (min.getZ() - orig.getZ()) / dir.getZ();
        t2 = (max.getZ() - orig.getZ()) / dir.getZ();
        
        if (t1 > t2) std::swap(t1, t2);
        
        if (tNear > t2 || t1 > tFar) return false;
        tNear = std::max(tNear, t1);
        tFar = std::min(tFar, t2);
        
        return tFar > 1e-4f; // Consistent epsilon
    }

    // Simple intersection test when you don't need t values
    bool intersects(const CRTRay& ray) const {
        float tNear, tFar;
        return intersect(ray, tNear, tFar);
    }

    // Expand AABB to include a point
    void expand(const CRTVector& point) {
        min = CRTVector(
            std::min(min.getX(), point.getX()),
            std::min(min.getY(), point.getY()),
            std::min(min.getZ(), point.getZ())
        );
        max = CRTVector(
            std::max(max.getX(), point.getX()),
            std::max(max.getY(), point.getY()),
            std::max(max.getZ(), point.getZ())
        );
    }

    // Expand AABB to include another AABB
    void expand(const AABB& other) {
        expand(other.min);
        expand(other.max);
    }

    // Get center point
    CRTVector center() const {
        return CRTVector(
            (min.getX() + max.getX()) * 0.5f,
            (min.getY() + max.getY()) * 0.5f,
            (min.getZ() + max.getZ()) * 0.5f
        );
    }

    // Surface area (useful for BVH construction)
    float surfaceArea() const {
        float dx = max.getX() - min.getX();
        float dy = max.getY() - min.getY();
        float dz = max.getZ() - min.getZ();
        return 2.0f * (dx * dy + dx * dz + dy * dz);
    }
};
