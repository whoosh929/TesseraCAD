#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Tessera::Sketch {

inline float Cross2D(const std::array<float, 2>& a, const std::array<float, 2>& b, const std::array<float, 2>& c) noexcept {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

inline bool PointInPolygon2D(const std::array<float, 2>& pt, const std::vector<std::array<float, 2>>& poly) noexcept {
    if (poly.size() < 3) return false;
    bool inside = false;
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((poly[i][1] > pt[1]) != (poly[j][1] > pt[1])) &&
            (pt[0] < (poly[j][0] - poly[i][0]) * (pt[1] - poly[i][1]) / (poly[j][1] - poly[i][1] + 1e-12f) + poly[i][0])) {
            inside = !inside;
        }
    }
    return inside;
}

inline bool FindSegmentIntersection(
    const std::array<float, 2>& p1, const std::array<float, 2>& p2,
    const std::array<float, 2>& p3, const std::array<float, 2>& p4,
    std::array<float, 2>& outPt, float& outT, float& outU
) noexcept {
    float dx1 = p2[0] - p1[0];
    float dy1 = p2[1] - p1[1];
    float dx2 = p4[0] - p3[0];
    float dy2 = p4[1] - p3[1];

    float d = dx1 * dy2 - dy1 * dx2;
    if (std::abs(d) < 1e-7f) return false;

    float t = ((p3[0] - p1[0]) * dy2 - (p3[1] - p1[1]) * dx2) / d;
    float u = ((p3[0] - p1[0]) * dy1 - (p3[1] - p1[1]) * dx1) / d;

    if (t > 1e-5f && t < 1.0f - 1e-5f && u > 1e-5f && u < 1.0f - 1e-5f) {
        outPt = { p1[0] + t * dx1, p1[1] + t * dy1 };
        outT = t;
        outU = u;
        return true;
    }
    return false;
}

inline bool PointOnSegment(
    const std::array<float, 2>& pt,
    const std::array<float, 2>& a,
    const std::array<float, 2>& b,
    float& outT
) noexcept {
    float dx = b[0] - a[0];
    float dy = b[1] - a[1];
    float lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-8f) return false;
    float cross = (pt[0] - a[0]) * dy - (pt[1] - a[1]) * dx;
    if (std::abs(cross) > 1e-3f * std::sqrt(lenSq)) return false;
    float dot = (pt[0] - a[0]) * dx + (pt[1] - a[1]) * dy;
    float t = dot / lenSq;
    if (t > 1e-4f && t < 1.0f - 1e-4f) {
        outT = t;
        return true;
    }
    return false;
}

inline float PolygonSignedArea2D(const std::vector<std::array<float, 2>>& poly) noexcept {
    if (poly.size() < 3) return 0.0f;
    float area = 0.0f;
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1];
    }
    return 0.5f * area;
}

inline void EnsureCCW(std::vector<std::array<float, 2>>& poly) noexcept {
    if (PolygonSignedArea2D(poly) < 0.0f) {
        std::reverse(poly.begin(), poly.end());
    }
}

/// Stitches / unions two 2D polygons together if they intersect or touch
inline std::vector<std::array<float, 2>> StitchTwoPolygons(
    std::vector<std::array<float, 2>> polyA,
    std::vector<std::array<float, 2>> polyB
) {
    if (polyA.size() < 3) return polyB;
    if (polyB.size() < 3) return polyA;

    EnsureCCW(polyA);
    EnsureCCW(polyB);

    struct Segment {
        std::array<float, 2> start;
        std::array<float, 2> end;
    };

    std::vector<Segment> segsA;
    const size_t nA = polyA.size();
    for (size_t i = 0; i < nA; ++i) {
        segsA.push_back({polyA[i], polyA[(i + 1) % nA]});
    }

    std::vector<Segment> segsB;
    const size_t nB = polyB.size();
    for (size_t i = 0; i < nB; ++i) {
        segsB.push_back({polyB[i], polyB[(i + 1) % nB]});
    }

    // Subdivide segments at all intersection points and touching vertices
    bool foundAnyIntersection = false;
    std::vector<Segment> subSegsA;
    for (const auto& sa : segsA) {
        std::vector<std::pair<float, std::array<float, 2>>> cuts;
        for (const auto& sb : segsB) {
            std::array<float, 2> xPt;
            float t = 0.0f, u = 0.0f;
            if (FindSegmentIntersection(sa.start, sa.end, sb.start, sb.end, xPt, t, u)) {
                cuts.push_back({t, xPt});
                foundAnyIntersection = true;
            }
            if (PointOnSegment(sb.start, sa.start, sa.end, t)) {
                cuts.push_back({t, sb.start});
                foundAnyIntersection = true;
            }
            if (PointOnSegment(sb.end, sa.start, sa.end, t)) {
                cuts.push_back({t, sb.end});
                foundAnyIntersection = true;
            }
        }
        std::sort(cuts.begin(), cuts.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        std::array<float, 2> cur = sa.start;
        for (const auto& cut : cuts) {
            if (std::hypot(cur[0] - cut.second[0], cur[1] - cut.second[1]) > 1e-4f) {
                subSegsA.push_back({cur, cut.second});
                cur = cut.second;
            }
        }
        if (std::hypot(cur[0] - sa.end[0], cur[1] - sa.end[1]) > 1e-4f) {
            subSegsA.push_back({cur, sa.end});
        }
    }

    std::vector<Segment> subSegsB;
    for (const auto& sb : segsB) {
        std::vector<std::pair<float, std::array<float, 2>>> cuts;
        for (const auto& sa : segsA) {
            std::array<float, 2> xPt;
            float t = 0.0f, u = 0.0f;
            if (FindSegmentIntersection(sb.start, sb.end, sa.start, sa.end, xPt, t, u)) {
                cuts.push_back({t, xPt});
                foundAnyIntersection = true;
            }
            if (PointOnSegment(sa.start, sb.start, sb.end, t)) {
                cuts.push_back({t, sa.start});
                foundAnyIntersection = true;
            }
            if (PointOnSegment(sa.end, sb.start, sb.end, t)) {
                cuts.push_back({t, sa.end});
                foundAnyIntersection = true;
            }
        }
        std::sort(cuts.begin(), cuts.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        std::array<float, 2> cur = sb.start;
        for (const auto& cut : cuts) {
            if (std::hypot(cur[0] - cut.second[0], cur[1] - cut.second[1]) > 1e-4f) {
                subSegsB.push_back({cur, cut.second});
                cur = cut.second;
            }
        }
        if (std::hypot(cur[0] - sb.end[0], cur[1] - sb.end[1]) > 1e-4f) {
            subSegsB.push_back({cur, sb.end});
        }
    }

    if (!foundAnyIntersection) {
        // Check if one polygon completely contains the other
        if (PointInPolygon2D(polyA[0], polyB)) return polyB;
        if (PointInPolygon2D(polyB[0], polyA)) return polyA;

        // Disjoint or touching at edge: check for shared edge cancellation
        std::vector<Segment> kept;
        auto IsOpposite = [](const Segment& s1, const Segment& s2) {
            float tol = 1e-3f;
            return (std::hypot(s1.start[0] - s2.end[0], s1.start[1] - s2.end[1]) < tol &&
                    std::hypot(s1.end[0] - s2.start[0], s1.end[1] - s2.start[1]) < tol);
        };

        for (const auto& sa : segsA) {
            bool shared = false;
            for (const auto& sb : segsB) {
                if (IsOpposite(sa, sb)) { shared = true; break; }
            }
            if (!shared) kept.push_back(sa);
        }
        for (const auto& sb : segsB) {
            bool shared = false;
            for (const auto& sa : segsA) {
                if (IsOpposite(sb, sa)) { shared = true; break; }
            }
            if (!shared) kept.push_back(sb);
        }

        if (kept.size() < segsA.size() + segsB.size() && !kept.empty()) {
            // Chaining kept segments
            std::vector<std::array<float, 2>> result;
            result.push_back(kept[0].start);
            std::array<float, 2> cur = kept[0].end;
            result.push_back(cur);
            std::vector<bool> used(kept.size(), false);
            used[0] = true;

            for (size_t step = 1; step < kept.size(); ++step) {
                bool found = false;
                for (size_t i = 0; i < kept.size(); ++i) {
                    if (used[i]) continue;
                    if (std::hypot(kept[i].start[0] - cur[0], kept[i].start[1] - cur[1]) < 1e-3f) {
                        used[i] = true;
                        cur = kept[i].end;
                        result.push_back(cur);
                        found = true;
                        break;
                    }
                }
                if (!found) break;
            }
            if (result.size() >= 3) {
                if (std::hypot(result.front()[0] - result.back()[0], result.front()[1] - result.back()[1]) < 1e-3f) {
                    result.pop_back();
                }
                return result;
            }
        }
        return polyA; // fallback
    }

    // Keep only segments whose exterior side is outside the other polygon
    auto IsDuplicate = [](const Segment& s1, const Segment& s2) {
        float tol = 1e-3f;
        return (std::hypot(s1.start[0] - s2.start[0], s1.start[1] - s2.start[1]) < tol &&
                std::hypot(s1.end[0] - s2.end[0], s1.end[1] - s2.end[1]) < tol);
    };
    auto IsOpposite = [](const Segment& s1, const Segment& s2) {
        float tol = 1e-3f;
        return (std::hypot(s1.start[0] - s2.end[0], s1.start[1] - s2.end[1]) < tol &&
                std::hypot(s1.end[0] - s2.start[0], s1.end[1] - s2.start[1]) < tol);
    };

    auto ExteriorPoint = [](const Segment& s) -> std::array<float, 2> {
        float dx = s.end[0] - s.start[0];
        float dy = s.end[1] - s.start[1];
        float len = std::hypot(dx, dy);
        if (len < 1e-7f) return s.start;
        // In CCW polygon, inward normal is (-dy/len, dx/len).
        // Exterior point is mid - 1e-3f * inwardNormal = mid + 1e-3f * (dy/len, -dx/len).
        float nx = dy / len;
        float ny = -dx / len;
        return {
            (s.start[0] + s.end[0]) * 0.5f + 1e-3f * nx,
            (s.start[1] + s.end[1]) * 0.5f + 1e-3f * ny
        };
    };

    std::vector<Segment> outerSegs;
    for (const auto& s : subSegsA) {
        auto extPt = ExteriorPoint(s);
        if (!PointInPolygon2D(extPt, polyB)) {
            bool isOpp = false;
            for (const auto& sb : subSegsB) {
                if (IsOpposite(s, sb)) { isOpp = true; break; }
            }
            if (!isOpp) {
                outerSegs.push_back(s);
            }
        }
    }
    for (const auto& s : subSegsB) {
        auto extPt = ExteriorPoint(s);
        if (!PointInPolygon2D(extPt, polyA)) {
            bool isOpp = false;
            bool isDup = false;
            for (const auto& existing : outerSegs) {
                if (IsDuplicate(s, existing)) { isDup = true; break; }
            }
            for (const auto& sa : subSegsA) {
                if (IsOpposite(s, sa)) { isOpp = true; break; }
            }
            if (!isOpp && !isDup) {
                outerSegs.push_back(s);
            }
        }
    }

    if (outerSegs.empty()) {
        return polyA;
    }

    // Chain outer segments into a single closed boundary loop
    std::vector<std::array<float, 2>> result;
    result.push_back(outerSegs[0].start);
    std::array<float, 2> cur = outerSegs[0].end;
    result.push_back(cur);

    std::vector<bool> used(outerSegs.size(), false);
    used[0] = true;

    for (size_t step = 1; step < outerSegs.size(); ++step) {
        bool found = false;
        float bestDist = 1e-2f;
        size_t bestIdx = 0;

        for (size_t i = 0; i < outerSegs.size(); ++i) {
            if (used[i]) continue;
            float dist = std::hypot(outerSegs[i].start[0] - cur[0], outerSegs[i].start[1] - cur[1]);
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = i;
                found = true;
            }
        }

        if (found) {
            used[bestIdx] = true;
            cur = outerSegs[bestIdx].end;
            result.push_back(cur);
        } else {
            break;
        }
    }

    if (result.size() >= 3) {
        if (std::hypot(result.front()[0] - result.back()[0], result.front()[1] - result.back()[1]) < 1e-3f) {
            result.pop_back();
        }
        return result;
    }

    return polyA;
}

/// Stitches an arbitrary list of 2D shapes together
inline std::vector<std::array<float, 2>> StitchMultipleShapes(
    const std::vector<std::vector<std::array<float, 2>>>& shapes
) {
    if (shapes.empty()) return {};
    if (shapes.size() == 1) return shapes[0];

    std::vector<std::array<float, 2>> combined = shapes[0];
    for (size_t i = 1; i < shapes.size(); ++i) {
        combined = StitchTwoPolygons(std::move(combined), shapes[i]);
    }
    return combined;
}

} // namespace Tessera::Sketch
