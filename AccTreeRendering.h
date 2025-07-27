#pragma once

#include "CRTVector.h"
#include "CRTRay.h"
#include "CRTMaterial.h"
#include "CRTLight.h"
#include "CRTSettings.h"
#include "CRTColor.h"
#include "CRTAccTree.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <thread>
#include <fstream>

// Helper functions (same as original, just consistent epsilon)
static float FresnelSchlick(const CRTVector& I, const CRTVector& N, float ior);
static bool Refract(const CRTVector& I, const CRTVector& N, float eta1, float eta2,
                    CRTVector& refracted);

// Main ray tracing function - simplified and optimized
inline CRTColor traceRayBVH(
    const CRTRay& ray,
    const std::vector<CRTTriangle>& scene,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTAccTree& accTree,
    int depth,
    bool isShadowRay = false
) {
    // Consistent depth limit
    if (depth > 5) {
        return settings.backgroundColor;
    }

    // Find closest intersection using BVH
    HitInfo hit;
    closestHit_BVH(ray, accTree, scene, hit);
    if (!hit.hit) {
        return settings.backgroundColor;
    }

    // Compute hit point and material info
    CRTVector P = ray.pointAtParameter(hit.t);
    const CRTMaterial& mat = materials[scene[hit.triIndex].getMaterialIndex()];
    CRTVector N = mat.smoothShading ? hit.normal : scene[hit.triIndex].getFaceNormal();

    // Sample material color (texture or albedo)
    CRTColor baseColor = (mat.texType != CRTMaterial::TexType::NONE)
                       ? mat.sampleTexture(hit.uv, hit.u, hit.v)
                       : mat.albedo;

    // Material shading
    switch (mat.type) {
        case CRTMaterial::Type::DIFFUSE: {
            CRTColor result(0, 0, 0);
            const float shadowBias = 1e-4f;
            
            for (const auto& light : lights) {
                CRTVector L = (light.getPosition() - P).normalize();
                float lightDist = (light.getPosition() - P).length();
                
                // Shadow test - use optimized shadow rays
                CRTRay shadowRay(P + N * shadowBias, L);
                if (isOccluded_BVH(shadowRay, accTree, scene, lightDist)) {
                    continue; // In shadow
                }

                // Compute lighting
                float NdotL = std::max(0.0f, N.dot(L));
                float attenuation = light.getIntensity() / (4.0f * 3.14159265f * lightDist * lightDist + 1e-4f);
                result += baseColor * (attenuation * NdotL);
            }
            return result;
        }

        case CRTMaterial::Type::REFLECTIVE: {
            // Reflection
            CRTVector I = ray.getDirection().normalize();
            CRTVector R = I - N * (2.0f * I.dot(N));
            CRTRay reflRay(P + N * 1e-4f, R.normalize());
            CRTColor reflected = traceRayBVH(reflRay, scene, materials, lights, settings, accTree, depth + 1);

            // Simple diffuse + reflection blend
            CRTColor diffuse(0, 0, 0);
            const float shadowBias = 1e-4f;
            
            for (const auto& light : lights) {
                CRTVector L = (light.getPosition() - P).normalize();
                float lightDist = (light.getPosition() - P).length();
                
                CRTRay shadowRay(P + N * shadowBias, L);
                if (isOccluded_BVH(shadowRay, accTree, scene, lightDist)) {
                    continue;
                }

                float NdotL = std::max(0.0f, N.dot(L));
                float attenuation = light.getIntensity() / (4.0f * 3.14159265f * lightDist * lightDist + 1e-4f);
                diffuse += baseColor * (attenuation * NdotL);
            }

            // Simple blend: more reflection for brighter materials
            float reflectiveness = (baseColor.r + baseColor.g + baseColor.b) / (3.0f * 255.0f);
            return diffuse * (1.0f - reflectiveness) + reflected * reflectiveness;
        }

        case CRTMaterial::Type::REFRACTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector n = N;
            float eta1 = 1.0f, eta2 = mat.ior;
            
            // Check if we're inside the object
            if (I.dot(n) > 0.0f) { 
                n = -n; 
                std::swap(eta1, eta2); 
            }

            // Reflection
            CRTVector R = I - n * (2.0f * I.dot(n));
            CRTRay reflRay(P + n * 1e-4f, R.normalize());
            CRTColor C_reflect = traceRayBVH(reflRay, scene, materials, lights, settings, accTree, depth + 1);

            // Refraction
            CRTColor C_refract(0, 0, 0);
            CRTVector T;
            if (Refract(I, n, eta1, eta2, T)) {
                CRTRay refrRay(P - n * 1e-4f, T.normalize());
                C_refract = traceRayBVH(refrRay, scene, materials, lights, settings, accTree, depth + 1, true);
            }

            // Simple Fresnel blend
            if (isShadowRay) {
                return (C_refract.r || C_refract.g || C_refract.b) ? C_refract : C_reflect;
            }

            float kr = (C_refract.r || C_refract.g || C_refract.b)
                     ? FresnelSchlick(I, n, mat.ior)
                     : 1.0f;
            return C_reflect * kr + C_refract * (1.0f - kr);
        }

        case CRTMaterial::Type::CONSTANT:
        default:
            return baseColor;
    }
}

// Simplified parallel rendering - same structure as original but cleaner
inline void renderTriangleSceneWithBVH(
    const std::vector<CRTTriangle>& scene,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTCamera& camera,
    const CRTAccTree& accTree,
    const std::string& filename,
    unsigned desiredThreads = 0
) {
    const int W = settings.resolutionWidth;
    const int H = settings.resolutionHeight;

    // Image buffer
    std::vector<CRTColor> buffer(W * H, settings.backgroundColor);

    // Thread setup
    unsigned numThreads = desiredThreads;
    if (numThreads == 0) {
        numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) numThreads = 4; // fallback
    }

    // Simple row-based work distribution
    const int rowsPerThread = (H + numThreads - 1) / numThreads;
    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    // Worker function
    auto renderRows = [&](int y0, int y1) {
        for (int y = y0; y < y1 && y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                CRTRay primary = CRTRay::generatePrimaryRay(x, y, W, H, camera);
                buffer[y * W + x] = traceRayBVH(primary, scene, materials, lights, 
                                               settings, accTree, 0);
            }
        }
    };

    // Launch threads
    for (unsigned i = 0; i < numThreads; ++i) {
        int y0 = i * rowsPerThread;
        int y1 = std::min(H, y0 + rowsPerThread);
        if (y0 >= H) break;
        threads.emplace_back(renderRows, y0, y1);
    }

    // Wait for completion
    for (auto& t : threads) {
        t.join();
    }

    // Save result
    std::ofstream ppm(filename);
    ppm << "P3\n" << W << " " << H << "\n255\n";
    for (const auto& c : buffer) {
        ppm << int(c.r) << ' ' << int(c.g) << ' ' << int(c.b) << ' ';
    }
}
