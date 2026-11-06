#pragma once

#include "SketchProfile.hpp"
#include <vector>
#include <array>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <limits>

namespace Tessera::Sketch {

struct TriangulatedPolygon2D {
    std::vector<Geom::Point2D> vertices;
    std::vector<uint32_t> indices;
};

class ProfileTessellator {
public:
    // Triangulate a SketchProfile into an indexed triangle list
    [[nodiscard]] static TriangulatedPolygon2D Triangulate(
        const SketchProfile& profile,
        double max_sagitta = 0.05,
        int min_segs = 8
    ) {
        TriangulatedPolygon2D result;
        if (!profile.is_closed) return result;

        auto outerPts = profile.outer_wire.discretize(max_sagitta, min_segs);
        if (outerPts.size() < 3) return result;

        // Ensure outerPts is CCW
        if (SignedArea(outerPts) < 0.0) {
            std::reverse(outerPts.begin(), outerPts.end());
        }

        // If no holes, directly ear-clip outer polygon
        if (profile.inner_holes.empty()) {
            result.vertices = outerPts;
            result.indices = EarClip(outerPts);
            return result;
        }

        // With holes: bridge-cut each hole into the outer boundary to form a single polygon
        std::vector<std::vector<Geom::Point2D>> holePolys;
        for (const auto& holeWire : profile.inner_holes) {
            auto hPts = holeWire.discretize(max_sagitta, min_segs);
            if (hPts.size() >= 3) {
                // Ensure holes are CW
                if (SignedArea(hPts) > 0.0) {
                    std::reverse(hPts.begin(), hPts.end());
                }
                holePolys.push_back(std::move(hPts));
            }
        }

        // Spliced polygon vertices
        std::vector<Geom::Point2D> combined = BridgeHoles(outerPts, holePolys);
        result.vertices = combined;
        result.indices = EarClip(combined);
        return result;
    }

private:
    static double SignedArea(const std::vector<Geom::Point2D>& pts) noexcept {
        double sum = 0.0;
        size_t n = pts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t next = (i + 1) % n;
            sum += (pts[i].u * pts[next].v - pts[next].u * pts[i].v);
        }
        return 0.5 * sum;
    }

    // Check if point P is inside triangle (A, B, C)
    static bool PointInTriangle(
        const Geom::Point2D& p,
        const Geom::Point2D& a,
        const Geom::Point2D& b,
        const Geom::Point2D& c
    ) noexcept {
        if (p.approx_equal(a, Geom::CAD_TOLERANCE) ||
            p.approx_equal(b, Geom::CAD_TOLERANCE) ||
            p.approx_equal(c, Geom::CAD_TOLERANCE)) {
            return false;
        }

        double cp1 = (b.u - a.u) * (p.v - a.v) - (b.v - a.v) * (p.u - a.u);
        double cp2 = (c.u - b.u) * (p.v - b.v) - (c.v - b.v) * (p.u - b.u);
        double cp3 = (a.u - c.u) * (p.v - c.v) - (a.v - c.v) * (p.u - c.u);

        return (cp1 > Geom::CAD_EPSILON && cp2 > Geom::CAD_EPSILON && cp3 > Geom::CAD_EPSILON);
    }

    // Ear-clipping algorithm for simple polygons
    static std::vector<uint32_t> EarClip(const std::vector<Geom::Point2D>& pts) {
        std::vector<uint32_t> indices;
        size_t n = pts.size();
        if (n < 3) return indices;

        std::vector<uint32_t> V(n);
        for (size_t i = 0; i < n; ++i) V[i] = static_cast<uint32_t>(i);

        int count = 2 * static_cast<int>(n);
        for (size_t v = n - 1; n > 2;) {
            if (count-- <= 0) {
                // Safeguard against non-simple degeneracies
                break;
            }

            size_t u = (v < n) ? v : 0;
            v = (u + 1 < n) ? u + 1 : 0;
            size_t w = (v + 1 < n) ? v + 1 : 0;

            const auto& ptU = pts[V[u]];
            const auto& ptV = pts[V[v]];
            const auto& ptW = pts[V[w]];

            // Cross product (ptV - ptU) x (ptW - ptV)
            double cross = (ptV.u - ptU.u) * (ptW.v - ptV.v) - (ptV.v - ptU.v) * (ptW.u - ptV.u);
            if (cross <= Geom::CAD_EPSILON) {
                continue; // Not a convex ear
            }

            // Check if any other vertex lies inside the triangle (ptU, ptV, ptW)
            bool ear = true;
            for (size_t p = 0; p < n; ++p) {
                if (p == u || p == v || p == w) continue;
                if (PointInTriangle(pts[V[p]], ptU, ptV, ptW)) {
                    ear = false;
                    break;
                }
            }

            if (ear) {
                indices.push_back(V[u]);
                indices.push_back(V[v]);
                indices.push_back(V[w]);

                V.erase(V.begin() + v);
                n--;
                count = 2 * static_cast<int>(n);
            }
        }

        return indices;
    }

    // Bridge holes into outer polygon via ray-mating
    static std::vector<Geom::Point2D> BridgeHoles(
        const std::vector<Geom::Point2D>& outer,
        const std::vector<std::vector<Geom::Point2D>>& holes
    ) {
        std::vector<Geom::Point2D> currentPoly = outer;

        for (const auto& hole : holes) {
            if (hole.empty()) continue;

            // Find right-most point on hole (max U)
            size_t holeMaxIdx = 0;
            for (size_t i = 1; i < hole.size(); ++i) {
                if (hole[i].u > hole[holeMaxIdx].u) {
                    holeMaxIdx = i;
                }
            }
            Geom::Point2D hMax = hole[holeMaxIdx];

            // Find closest visible vertex on currentPoly
            size_t closestOuterIdx = 0;
            double minDistSq = std::numeric_limits<double>::max();

            for (size_t i = 0; i < currentPoly.size(); ++i) {
                if (currentPoly[i].u >= hMax.u - Geom::CAD_TOLERANCE) {
                    double d2 = hMax.distance_squared_to(currentPoly[i]);
                    if (d2 < minDistSq) {
                        minDistSq = d2;
                        closestOuterIdx = i;
                    }
                }
            }

            // Splice hole into currentPoly at closestOuterIdx
            std::vector<Geom::Point2D> spliced;
            spliced.reserve(currentPoly.size() + hole.size() + 2);

            // 1. Current poly up to bridge point
            for (size_t i = 0; i <= closestOuterIdx; ++i) {
                spliced.push_back(currentPoly[i]);
            }

            // 2. Traversal of hole starting from holeMaxIdx
            for (size_t i = 0; i < hole.size(); ++i) {
                spliced.push_back(hole[(holeMaxIdx + i) % hole.size()]);
            }
            // Repeat hole start to close bridge
            spliced.push_back(hole[holeMaxIdx]);

            // 3. Remainder of current poly from bridge point onwards
            for (size_t i = closestOuterIdx; i < currentPoly.size(); ++i) {
                spliced.push_back(currentPoly[i]);
            }

            currentPoly = std::move(spliced);
        }

        return currentPoly;
    }
};

} // namespace Tessera::Sketch
