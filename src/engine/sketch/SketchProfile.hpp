#pragma once

#include "engine/geom/Curves2D.hpp"
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace Tessera::Sketch {

// =============================================================================
// Wire2D: Ordered sequence of head-to-tail connected 2D curve segments
// =============================================================================

class Wire2D {
public:
    std::vector<std::shared_ptr<Geom::Curve2D>> curves;
    bool is_closed{false};

    Wire2D() = default;
    explicit Wire2D(std::vector<std::shared_ptr<Geom::Curve2D>> c, bool closed = false)
        : curves(std::move(c)), is_closed(closed) {}

    [[nodiscard]] bool empty() const noexcept { return curves.empty(); }
    [[nodiscard]] size_t size() const noexcept { return curves.size(); }

    [[nodiscard]] Geom::Point2D start_point() const noexcept {
        return curves.empty() ? Geom::Point2D() : curves.front()->start_point();
    }

    [[nodiscard]] Geom::Point2D end_point() const noexcept {
        return curves.empty() ? Geom::Point2D() : curves.back()->end_point();
    }

    [[nodiscard]] double length() const noexcept {
        double total = 0.0;
        for (const auto& c : curves) {
            total += c->length();
        }
        return total;
    }

    // Discretize all segments into a continuous sequence of points
    [[nodiscard]] std::vector<Geom::Point2D> discretize(double max_sagitta = 0.05, int min_segs = 8) const {
        std::vector<Geom::Point2D> pts;
        if (curves.empty()) return pts;

        for (size_t i = 0; i < curves.size(); ++i) {
            auto segPts = curves[i]->discretize(max_sagitta, min_segs);
            if (segPts.empty()) continue;

            if (pts.empty()) {
                pts.insert(pts.end(), segPts.begin(), segPts.end());
            } else {
                // Avoid duplicating junction point if already matching
                size_t startIdx = (pts.back().approx_equal(segPts.front(), Geom::CAD_TOLERANCE)) ? 1 : 0;
                pts.insert(pts.end(), segPts.begin() + startIdx, segPts.end());
            }
        }

        // If closed, ensure closing point doesn't duplicate the first point in polygon vertex list
        if (is_closed && pts.size() > 1 && pts.front().approx_equal(pts.back(), Geom::CAD_TOLERANCE)) {
            pts.pop_back();
        }

        return pts;
    }

    // Signed polygon area (Green's theorem). Positive = CCW, Negative = CW
    [[nodiscard]] double signed_area(double max_sagitta = 0.05, int min_segs = 8) const {
        if (!is_closed || curves.empty()) return 0.0;
        auto pts = discretize(max_sagitta, min_segs);
        if (pts.size() < 3) return 0.0;

        double sum = 0.0;
        size_t n = pts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t next = (i + 1) % n;
            sum += (pts[i].u * pts[next].v - pts[next].u * pts[i].v);
        }
        return 0.5 * sum;
    }

    [[nodiscard]] bool is_counter_clockwise() const {
        return signed_area() > 0.0;
    }

    // Reverse orientation of the wire
    void reverse() {
        std::reverse(curves.begin(), curves.end());
        for (auto& c : curves) {
            c = c->reversed();
        }
    }

    // Point in polygon test (Ray casting algorithm)
    [[nodiscard]] bool contains_point(const Geom::Point2D& pt) const {
        if (!is_closed) return false;
        auto pts = discretize();
        if (pts.size() < 3) return false;

        bool inside = false;
        size_t n = pts.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            if (((pts[i].v > pt.v) != (pts[j].v > pt.v)) &&
                (pt.u < (pts[j].u - pts[i].u) * (pt.v - pts[i].v) / (pts[j].v - pts[i].v) + pts[i].u)) {
                inside = !inside;
            }
        }
        return inside;
    }
};

// =============================================================================
// SketchProfile: Boundary representation of a single sweepable region
// =============================================================================

class SketchProfile {
public:
    Wire2D outer_wire;               // Outer boundary loop (or single open wire)
    std::vector<Wire2D> inner_holes; // Inner void loops (holes)
    bool is_closed{false};

    SketchProfile() = default;

    // Construct an open profile (sheet/surface generator)
    static SketchProfile Open(Wire2D openWire) {
        SketchProfile prof;
        prof.outer_wire = std::move(openWire);
        prof.is_closed = false;
        return prof;
    }

    // Construct a closed profile (solid or hollow sweep generator)
    static SketchProfile Closed(Wire2D outer, std::vector<Wire2D> holes = {}) {
        SketchProfile prof;
        prof.outer_wire = std::move(outer);
        prof.inner_holes = std::move(holes);
        prof.is_closed = true;

        // Ensure outer wire is CCW (positive area)
        if (!prof.outer_wire.is_counter_clockwise()) {
            prof.outer_wire.reverse();
        }

        // Ensure all inner holes are CW (negative area)
        for (auto& hole : prof.inner_holes) {
            if (hole.is_counter_clockwise()) {
                hole.reverse();
            }
        }
        return prof;
    }

    [[nodiscard]] bool has_holes() const noexcept { return !inner_holes.empty(); }

    [[nodiscard]] double net_area(double max_sagitta = 0.05, int min_segs = 8) const {
        if (!is_closed) return 0.0;
        double area = std::abs(outer_wire.signed_area(max_sagitta, min_segs));
        for (const auto& hole : inner_holes) {
            area -= std::abs(hole.signed_area(max_sagitta, min_segs));
        }
        return std::max(0.0, area);
    }
};

} // namespace Tessera::Sketch

