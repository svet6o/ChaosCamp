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

static float FresnelSchlick(const CRTVector& I, const CRTVector& N, float ior);
static bool Refract(const CRTVector& I, const CRTVector& N, float eta1, float eta2,
                    CRTVector& refracted);

// Основен traceRay с BVH accel tree
inline CRTColor traceRayBVH(
    const CRTRay& ray,
    const std::vector<CRTTriangle>& scene,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTAccTree& accTree,
    int depth,
    bool isShadowRay     = false,
    float shadowBias     = 1e-4f,
    float refractionBias = 1e-4f
) {
    if (depth > 5) {
        return settings.backgroundColor;
    }

    // Най-близък удар с BVH
    HitInfo hit;
    closestHit_BVH(ray, accTree, scene, hit);
    if (!hit.hit) {
        return settings.backgroundColor;
    }

    // Точка и нормал на удара
    CRTVector P = ray.pointAtParameter(hit.t);
    const CRTMaterial& mat = materials[ scene[hit.triIndex].getMaterialIndex() ];
    CRTVector N = mat.smoothShading
                ? hit.normal
                : scene[hit.triIndex].getFaceNormal();

    // Базов цвят: текстура или албедо
    CRTColor baseColor = (mat.texType != CRTMaterial::TexType::NONE)
                       ? mat.sampleTexture(hit.uv, hit.u, hit.v)
                       : mat.albedo;

    switch (mat.type) {
        case CRTMaterial::Type::DIFFUSE: {
            CRTColor accum(0,0,0);
            for (const auto& light : lights) {
                CRTVector L = (light.getPosition() - P).normalize();
                CRTRay shadowRay(P + N * shadowBias, L);
                float lightDist = (light.getPosition() - P).length();
                if (isOccluded_BVH(shadowRay, accTree, scene, lightDist))
                    continue;

                float NdotL = std::max(0.0f, N.dot(L));
                float att   = light.getIntensity() / (4.0f * 3.14159265f * lightDist * lightDist + 1e-4f);
                accum += baseColor * (att * NdotL);
            }
            return accum;
        }
        case CRTMaterial::Type::REFLECTIVE: {
    // Нормализирани вектори
    CRTVector I = ray.getDirection().normalize();
    CRTVector R = I - N * (2.0f * I.dot(N));
    CRTRay reflRay(P + N * shadowBias, R.normalize());

    // Отразена светлина
    CRTColor reflected = traceRayBVH(reflRay, scene, materials, lights, settings, accTree, depth + 1);

    // Дифузна светлина (подобно на DIFFUSE case)
    CRTColor diffuse(0, 0, 0);
    for (const auto& light : lights) {
        CRTVector L = (light.getPosition() - P).normalize();
        CRTRay shadowRay(P + N * shadowBias, L);
        float lightDist = (light.getPosition() - P).length();
        if (isOccluded_BVH(shadowRay, accTree, scene, lightDist))
            continue;

        float NdotL = std::max(0.0f, N.dot(L));
        float att   = light.getIntensity() / (4.0f * 3.14159265f * lightDist * lightDist + 1e-4f);
        diffuse += baseColor * (att * NdotL);
    }

    // Комбиниране на дифузно и отражение
    float k = baseColor.r / 255.0f;  // Пропорция отражение (може да се базира и на друга метрика)
    return diffuse * (1.0f - k) + reflected * k;
}
        case CRTMaterial::Type::REFRACTIVE: {
            CRTVector I = ray.getDirection().normalize();
            CRTVector n = N;
            float eta1 = 1.0f, eta2 = mat.ior;
            if (I.dot(n) > 0.0f) { n = -n; std::swap(eta1, eta2); }

            // Reflection
            CRTVector R = I - n * (2.0f * I.dot(n));
            CRTRay reflRay(P + n * shadowBias, R.normalize());
            CRTColor C_reflect = traceRayBVH(reflRay, scene, materials, lights, settings, accTree, depth+1);

            // Refraction
            CRTColor C_refract(0,0,0);
            CRTVector T;
            if (Refract(I, n, eta1, eta2, T)) {
                CRTRay refrRay(P - n * refractionBias, T.normalize());
                C_refract = traceRayBVH(refrRay, scene, materials, lights, settings, accTree, depth+1, true);
            }

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

// Паралелен рендер с BVH
inline void renderTriangleSceneWithBVH(
    const std::vector<CRTTriangle>& scene,
    const std::vector<CRTMaterial>& materials,
    const std::vector<CRTLight>& lights,
    const CRTSettings& settings,
    const CRTCamera& camera,
    CRTAccTree& accTree,
    const std::string& filename,
    unsigned desiredThreads = 0   // 0 => използвай всички налични
) {
    const int W = settings.resolutionWidth;
    const int H = settings.resolutionHeight;

    std::vector<CRTColor> buffer(W * H, settings.backgroundColor);

    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 4;                           // fallback
    unsigned numThreads = desiredThreads ? std::min(desiredThreads, hw)
                                         : hw;     // clamp до наличните
    if (numThreads == 0) numThreads = 1;

    const int rowsPerThread = (H + (int)numThreads - 1) / (int)numThreads; // ceil div
    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    auto renderRows = [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y) {
            for (int x = 0; x < W; ++x) {
                CRTRay primary = CRTRay::generatePrimaryRay(x, y, W, H, camera);
                buffer[y * W + x] =
                    traceRayBVH(primary, scene, materials, lights, settings, accTree, 0);
            }
        }
    };

    for (unsigned i = 0; i < numThreads; ++i) {
        int y0 = (int)i * rowsPerThread;
        int y1 = std::min(H, y0 + rowsPerThread);
        if (y0 >= H) break;
        threads.emplace_back(renderRows, y0, y1);
    }

    for (auto& t : threads) t.join();

    std::ofstream ppm(filename);
    ppm << "P3\n" << W << " " << H << "\n255\n";
    for (const auto& c : buffer)
        ppm << int(c.r) << ' ' << int(c.g) << ' ' << int(c.b) << ' ';
}
