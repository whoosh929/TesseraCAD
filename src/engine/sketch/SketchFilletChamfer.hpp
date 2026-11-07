#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

namespace Tessera::Sketch {

/**
 * @brief Applies a flat chamfer (corner cut) to corner vertex at cornerIdx.
 * Replaces the single sharp vertex with two beveled vertices at distance 'distance'.
 */
inline bool ApplyChamfer2D(
    std::vector<std::array<float, 2>>& points,
    size_t cornerIdx,
    float distance,
    bool isClosed = true
) {
    size_t n = points.size();
    if (n < 3 || cornerIdx >= n || distance <= 1e-4f) return false;

    size_t prevIdx = (cornerIdx + n - 1) % n;
    size_t nextIdx = (cornerIdx + 1) % n;
    if (!isClosed) {
        if (cornerIdx == 0 || cornerIdx == n - 1) return false;
    }

    float vx = points[cornerIdx][0];
    float vy = points[cornerIdx][1];

    float d1x = points[prevIdx][0] - vx;
    float d1y = points[prevIdx][1] - vy;
    float len1 = std::hypot(d1x, d1y);

    float d2x = points[nextIdx][0] - vx;
    float d2y = points[nextIdx][1] - vy;
    float len2 = std::hypot(d2x, d2y);

    if (len1 < 1e-4f || len2 < 1e-4f) return false;

    float maxDist = std::min(len1, len2) * 0.45f;
    float d = std::min(distance, maxDist);
    if (d <= 1e-4f) return false;

    float u1x = d1x / len1;
    float u1y = d1y / len1;
    float u2x = d2x / len2;
    float u2y = d2y / len2;

    std::array<float, 2> p1 = { vx + u1x * d, vy + u1y * d };
    std::array<float, 2> p2 = { vx + u2x * d, vy + u2y * d };

    points[cornerIdx] = p1;
    points.insert(points.begin() + cornerIdx + 1, p2);
    return true;
}

/**
 * @brief Applies a circular fillet (tangent arc) to corner vertex at cornerIdx.
 * Replaces the single sharp vertex with a smooth arc of radius 'radius'.
 */
inline bool ApplyFillet2D(
    std::vector<std::array<float, 2>>& points,
    size_t cornerIdx,
    float radius,
    bool isClosed = true,
    int arcSegments = 6
) {
    size_t n = points.size();
    if (n < 3 || cornerIdx >= n || radius <= 1e-4f) return false;

    size_t prevIdx = (cornerIdx + n - 1) % n;
    size_t nextIdx = (cornerIdx + 1) % n;
    if (!isClosed) {
        if (cornerIdx == 0 || cornerIdx == n - 1) return false;
    }

    float vx = points[cornerIdx][0];
    float vy = points[cornerIdx][1];

    float d1x = points[prevIdx][0] - vx;
    float d1y = points[prevIdx][1] - vy;
    float len1 = std::hypot(d1x, d1y);

    float d2x = points[nextIdx][0] - vx;
    float d2y = points[nextIdx][1] - vy;
    float len2 = std::hypot(d2x, d2y);

    if (len1 < 1e-4f || len2 < 1e-4f) return false;

    float u1x = d1x / len1;
    float u1y = d1y / len1;
    float u2x = d2x / len2;
    float u2y = d2y / len2;

    float dot = u1x * u2x + u1y * u2y;
    dot = std::clamp(dot, -0.999f, 0.999f);

    float bx = u1x + u2x;
    float by = u1y + u2y;
    float bLen = std::hypot(bx, by);
    if (bLen < 1e-4f) return false;

    bx /= bLen;
    by /= bLen;

    float cosBeta = std::clamp(u1x * bx + u1y * by, 0.01f, 0.999f);
    float sinBeta = std::sqrt(1.0f - cosBeta * cosBeta);
    float tanBeta = sinBeta / cosBeta;

    float d = radius / tanBeta;
    float maxD = std::min(len1, len2) * 0.45f;
    if (d > maxD) {
        d = maxD;
        radius = d * tanBeta;
    }
    if (radius <= 1e-4f) return false;

    float p1x = vx + u1x * d;
    float p1y = vy + u1y * d;
    float p2x = vx + u2x * d;
    float p2y = vy + u2y * d;

    float distToCenter = radius / sinBeta;
    float cx = vx + bx * distToCenter;
    float cy = vy + by * distToCenter;

    float r1x = p1x - cx;
    float r1y = p1y - cy;
    float r2x = p2x - cx;
    float r2y = p2y - cy;

    float theta1 = std::atan2(r1y, r1x);
    float theta2 = std::atan2(r2y, r2x);

    float dTheta = theta2 - theta1;
    while (dTheta > 3.14159265f) dTheta -= 2.0f * 3.14159265f;
    while (dTheta < -3.14159265f) dTheta += 2.0f * 3.14159265f;

    std::vector<std::array<float, 2>> arcPts;
    arcPts.reserve(arcSegments + 1);
    for (int i = 0; i <= arcSegments; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(arcSegments);
        float th = theta1 + t * dTheta;
        arcPts.push_back({ cx + std::cos(th) * radius, cy + std::sin(th) * radius });
    }

    points[cornerIdx] = arcPts[0];
    points.insert(points.begin() + cornerIdx + 1, arcPts.begin() + 1, arcPts.end());
    return true;
}

} // namespace Tessera::Sketch

