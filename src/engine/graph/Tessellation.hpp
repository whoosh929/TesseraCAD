#pragma once

#include "GeometryTypes.hpp"
#include <cmath>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace Tessera {

namespace Math {

inline void Cross(const float a[3], const float b[3], float out[3]) noexcept {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float Dot(const float a[3], const float b[3]) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void Normalize(float v[3]) noexcept {
    float len = std::sqrt(Dot(v, v));
    if (len > 1e-6f) {
        float inv = 1.0f / len;
        v[0] *= inv;
        v[1] *= inv;
        v[2] *= inv;
    }
}

} // namespace Math

class TessellationUtils {
public:
    /// Extracts sharp CAD feature edges (dihedral angle >= sharpAngleDegrees, or boundary edges)
    static std::vector<uint32_t> ExtractFeatureEdges(
        const std::vector<CADVertex>& vertices,
        const std::vector<uint32_t>& indices,
        float sharpAngleDegrees = 30.0f
    ) {
        if (indices.size() < 3) return {};

        const float cosThreshold = std::cos(sharpAngleDegrees * 3.1415926535f / 180.0f);
        const size_t triangleCount = indices.size() / 3;

        // Precompute face normals for all triangles
        std::vector<std::array<float, 3>> faceNormals(triangleCount);
        for (size_t t = 0; t < triangleCount; ++t) {
            uint32_t i0 = indices[t * 3 + 0];
            uint32_t i1 = indices[t * 3 + 1];
            uint32_t i2 = indices[t * 3 + 2];

            const float* p0 = vertices[i0].position;
            const float* p1 = vertices[i1].position;
            const float* p2 = vertices[i2].position;

            float v0[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
            float v1[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};

            float n[3];
            Math::Cross(v0, v1, n);
            Math::Normalize(n);
            faceNormals[t] = {n[0], n[1], n[2]};
        }

        // Map 64-bit edge key (min(i0, i1) << 32 | max(i0, i1)) to triangle indices
        struct EdgeInfo {
            uint32_t tri0{0};
            uint32_t tri1{0};
            uint8_t count{0};
        };

        std::unordered_map<uint64_t, EdgeInfo> edgeMap;
        edgeMap.reserve(indices.size());

        for (size_t t = 0; t < triangleCount; ++t) {
            uint32_t triIndices[3] = {indices[t * 3 + 0], indices[t * 3 + 1], indices[t * 3 + 2]};
            for (int e = 0; e < 3; ++e) {
                uint32_t a = triIndices[e];
                uint32_t b = triIndices[(e + 1) % 3];
                uint32_t u0 = std::min(a, b);
                uint32_t u1 = std::max(a, b);
                uint64_t key = (static_cast<uint64_t>(u0) << 32) | static_cast<uint64_t>(u1);

                auto& info = edgeMap[key];
                if (info.count == 0) {
                    info.tri0 = static_cast<uint32_t>(t);
                    info.count = 1;
                } else if (info.count == 1) {
                    info.tri1 = static_cast<uint32_t>(t);
                    info.count = 2;
                }
            }
        }

        std::vector<uint32_t> featureEdges;
        for (const auto& [key, info] : edgeMap) {
            uint32_t u0 = static_cast<uint32_t>(key >> 32);
            uint32_t u1 = static_cast<uint32_t>(key & 0xFFFFFFFF);

            if (info.count == 1) {
                // Open boundary edge -> always sharp feature
                featureEdges.push_back(u0);
                featureEdges.push_back(u1);
            } else if (info.count == 2) {
                const auto& n0 = faceNormals[info.tri0];
                const auto& n1 = faceNormals[info.tri1];
                float dot = Math::Dot(n0.data(), n1.data());
                if (dot < cosThreshold) {
                    // Dihedral angle between faces is sharp
                    featureEdges.push_back(u0);
                    featureEdges.push_back(u1);
                }
            }
        }

        return featureEdges;
    }

    /// Triangulates a simple 2D polygon (convex or concave) using Ear Clipping
    static std::vector<uint32_t> TriangulatePolygon2D(const std::vector<std::array<float, 2>>& poly) {
        const size_t n = poly.size();
        if (n < 3) return {};

        std::vector<uint32_t> indices;
        if (n == 3) {
            return {0, 1, 2};
        }

        // Determine polygon orientation (signed area)
        float area = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            area += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1];
        }

        std::vector<uint32_t> V(n);
        if (area > 0.0f) {
            for (size_t i = 0; i < n; ++i) V[i] = static_cast<uint32_t>(i);
        } else {
            // Clockwise -> reverse to CCW
            for (size_t i = 0; i < n; ++i) V[i] = static_cast<uint32_t>(n - 1 - i);
        }

        auto IsInsideTriangle = [](float ax, float ay, float bx, float by, float cx, float cy, float px, float py) -> bool {
            float cross1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
            float cross2 = (cx - bx) * (py - by) - (cy - by) * (px - bx);
            float cross3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx);
            return (cross1 >= 0.0f && cross2 >= 0.0f && cross3 >= 0.0f) ||
                   (cross1 <= 0.0f && cross2 <= 0.0f && cross3 <= 0.0f);
        };

        int count = 2 * static_cast<int>(n);
        for (size_t v = V.size() - 1; V.size() > 2;) {
            if (--count <= 0) break; // Error prevention

            size_t u = (v < V.size()) ? v : 0;
            v = (u + 1 < V.size()) ? u + 1 : 0;
            size_t w = (v + 1 < V.size()) ? v + 1 : 0;

            uint32_t iU = V[u], iV = V[v], iW = V[w];
            float ax = poly[iU][0], ay = poly[iU][1];
            float bx = poly[iV][0], by = poly[iV][1];
            float cx = poly[iW][0], cy = poly[iW][1];

            // Convex check
            if (((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) <= 1e-6f) {
                continue;
            }

            // Test if any other vertex is inside this ear
            bool ear = true;
            for (size_t p = 0; p < V.size(); ++p) {
                if (p == u || p == v || p == w) continue;
                uint32_t iP = V[p];
                if (IsInsideTriangle(ax, ay, bx, by, cx, cy, poly[iP][0], poly[iP][1])) {
                    ear = false;
                    break;
                }
            }

            if (ear) {
                indices.push_back(iU);
                indices.push_back(iV);
                indices.push_back(iW);
                V.erase(V.begin() + v);
                count = 2 * static_cast<int>(V.size());
            }
        }

        return indices;
    }
};

} // namespace Tessera
