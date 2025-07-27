#pragma once
#include "CRTVector.h"
#include "CRTCamera.h"

class CRTRay {
public:
    CRTRay() : origin(), direction() {}

    CRTRay(const CRTVector& origin, const CRTVector& direction) 
        : origin(origin), direction(direction.normalize()) {}

    CRTRay(const CRTRay& other) 
        : origin(other.origin), direction(other.direction) {}

    // Basic accessors (same as original)
    const CRTVector& getOrigin() const { return origin; }
    const CRTVector& getDirection() const { return direction; }

    void setOrigin(const CRTVector& newOrigin) { origin = newOrigin; }
    void setDirection(const CRTVector& newDirection) { direction = newDirection.normalize(); }

    // Point along ray (same as original)
    CRTVector pointAtParameter(float t) const {
        return origin + direction * t;
    }

    // Primary ray generation (same as original, just cleaner)
    static CRTRay generatePrimaryRay(int x, int y, int imageWidth, int imageHeight, const CRTCamera& camera) {
        // Convert pixel coordinates to normalized device coordinates
        float pixelCenterX = static_cast<float>(x) + 0.5f;
        float pixelCenterY = static_cast<float>(y) + 0.5f;

        float ndcX = pixelCenterX / static_cast<float>(imageWidth);
        float ndcY = pixelCenterY / static_cast<float>(imageHeight);

        // Convert to screen space [-1, 1]
        float screenX = 2.0f * ndcX - 1.0f;
        float screenY = 1.0f - 2.0f * ndcY;

        // Apply aspect ratio
        float aspectRatio = static_cast<float>(imageWidth) / static_cast<float>(imageHeight);
        screenX *= aspectRatio;

        // Create ray direction
        CRTVector direction(screenX, screenY, -1.0f);
        direction = direction.normalize();

        return CRTRay(camera.getPosition(), direction);
    }

    // Simple utility: check if ray hits sphere (useful for debugging)
    bool intersectSphere(const CRTVector& center, float radius, float& t) const {
        CRTVector oc = origin - center;
        float a = direction.dot(direction);
        float b = 2.0f * oc.dot(direction);
        float c = oc.dot(oc) - radius * radius;
        float discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        float sqrtDisc = std::sqrt(discriminant);
        float t1 = (-b - sqrtDisc) / (2 * a);
        float t2 = (-b + sqrtDisc) / (2 * a);
        
        t = (t1 > 1e-4f) ? t1 : t2;
        return t > 1e-4f;
    }

private:
    CRTVector origin;    
    CRTVector direction; 
};
