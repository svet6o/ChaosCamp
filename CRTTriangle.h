#pragma once

#include "CRTVector.h"
#include "CRTRay.h"

class CRTTriangle {
private:
    static const int vertexCount = 3;
    CRTVector vertices[vertexCount];
    CRTVector normals[vertexCount];
    CRTVector uvCoords[vertexCount]; // UV coords: x=u, y=v
    CRTVector faceNormal;
    CRTVector edge1;
    CRTVector edge2;
    float areaValue;
    float invDenominator;
    int materialIndex;

public:
    CRTTriangle() = default;
    
    // Constructor with optional normals; new parameters uv0, uv1, uv2
    CRTTriangle(const CRTVector& v0, const CRTVector& v1, const CRTVector& v2,
                const CRTVector& n0 = CRTVector(), 
                const CRTVector& n1 = CRTVector(),
                const CRTVector& n2 = CRTVector(),
                const CRTVector& uv0 = CRTVector(0,0,0),
                const CRTVector& uv1 = CRTVector(1,0,0),
                const CRTVector& uv2 = CRTVector(0,1,0),
                int matIdx = 0)
    {
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
        normals[0] = n0;
        normals[1] = n1;
        normals[2] = n2;
        uvCoords[0] = uv0;
        uvCoords[1] = uv1;
        uvCoords[2] = uv2;
        materialIndex = matIdx;
        
        // Precompute edges and face normal
        edge1 = v1 - v0;
        edge2 = v2 - v0;
        CRTVector crossProd = edge1.cross(edge2);
        faceNormal = crossProd.normalize();
        areaValue = 0.5f * crossProd.length();
        invDenominator = 1.0f / (2.0f * areaValue);
        
        // If vertex normals not provided, use face normal
        if (normals[0].length() < 1e-6f) {
            normals[0] = normals[1] = normals[2] = faceNormal;
        }
    }

    int getMaterialIndex() const { return materialIndex; }
    void setMaterialIndex(int idx) { materialIndex = idx; }

    // Möller-Trumbore intersection with barycentric coords and UV
    bool intersect(const CRTRay& ray, float& t, float& u, float& v, CRTVector& hitNormal, CRTVector& hitUV) const {
        const float EPSILON = 1e-4f;
        CRTVector h = ray.getDirection().cross(edge2);
        float a = edge1.dot(h);

        if (a > -EPSILON && a < EPSILON) 
            return false;

        float f = 1.0f / a;
        CRTVector s = ray.getOrigin() - vertices[0];
        u = f * s.dot(h);

        if (u < 0.0f || u > 1.0f)
            return false;

        CRTVector q = s.cross(edge1);
        v = f * ray.getDirection().dot(q);

        if (v < 0.0f || u + v > 1.0f)
            return false;

        t = f * edge2.dot(q);

        if (t <= EPSILON)
            return false;

        // Interpolate normal
        float w = 1.0f - u - v;
        hitNormal = (normals[0] * w + normals[1] * u + normals[2] * v).normalize();

        // Interpolate UV coords
        hitUV = uvCoords[0] * w + uvCoords[1] * u + uvCoords[2] * v;

        return true;
    }

    // Accessors
    const CRTVector& getVertex(int index) const { 
        return vertices[index]; 
    }
    
    const CRTVector& getFaceNormal() const { 
        return faceNormal; 
    }
    
    const CRTVector& getNormal(int vertexIndex) const {
        return normals[vertexIndex];
    }
    
    const CRTVector* getVertices() const {
        return vertices;
    }
    
    const CRTVector* getNormals() const {
        return normals;
    }
}; 