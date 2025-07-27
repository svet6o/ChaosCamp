#pragma once
#include "CRTVector.h"
#include "CRTRay.h"
#include <limits>

struct AABB {
    CRTVector min, max;

    AABB() = default;
    AABB(const CRTVector& mn, const CRTVector& mx) : min(mn), max(mx) {}

    // Fast ray-AABB using precomputed invDir & sign
    bool intersect(const CRTRay& ray, float& tNear, float& tFar) const {
        const auto& orig = ray.getOrigin();
        const auto& dir  = ray.getDirection();
        float invDir[3] = {1.0f/dir.getX(), 1.0f/dir.getY(), 1.0f/dir.getZ()};
        int sign[3]     = {invDir[0] < 0, invDir[1] < 0, invDir[2] < 0};

        const float bounds[2][3] = {{min.getX(), min.getY(), min.getZ()},
                                    {max.getX(), max.getY(), max.getZ()}};

        tNear = (bounds[sign[0]][0] - orig.getX()) * invDir[0];
        tFar  = (bounds[1-sign[0]][0] - orig.getX()) * invDir[0];

        for (int i = 1; i < 3; ++i) {
            float t1 = (bounds[sign[i]][i] - (i==1?orig.getY():orig.getZ())) * invDir[i];
            float t2 = (bounds[1-sign[i]][i] - (i==1?orig.getY():orig.getZ())) * invDir[i];
            if ((tNear > t2) || (t1 > tFar)) return false;
            tNear = t1 > tNear ? t1 : tNear;
            tFar  = t2 < tFar  ? t2 : tFar;
        }
        return tFar > 0;
    }
};
