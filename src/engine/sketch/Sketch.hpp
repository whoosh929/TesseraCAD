#pragma once

#include "engine/geom/Plane.hpp"
#include "engine/geom/Curves2D.hpp"
#include "SketchProfile.hpp"
#include <vector>
#include <memory>
#include <map>
#include <unordered_set>
#include <iostream>

namespace Tessera::Sketch {

class Sketch {
public:
    Geom::Plane plane{Geom::Plane::XY()};
    std::vector<std::shared_ptr<Geom::Curve2D>> curves;

    Sketch() = default;
    explicit Sketch(Geom::Plane refPlane) : plane(std::move(refPlane)) {}

    void clear() noexcept {
        curves.clear();
    }

    [[nodiscard]] bool empty() const noexcept {
        return curves.empty();
    }

    [[nodiscard]] size_t curve_count() const noexcept {
        return curves.size();
    }

    // =========================================================================
    // 2D Entity Construction
    // =========================================================================

    std::shared_ptr<Geom::LineSegment2D> add_line(const Geom::Point2D& start, const Geom::Point2D& end) {
        auto line = std::make_shared<Geom::LineSegment2D>(start, end);
        curves.push_back(line);
        return line;
    }

    std::shared_ptr<Geom::Arc2D> add_arc(
        const Geom::Point2D& center,
        double radius,
        double start_angle,
        double end_angle,
        bool is_ccw = true
    ) {
        auto arc = std::make_shared<Geom::Arc2D>(center, radius, start_angle, end_angle, is_ccw);
        curves.push_back(arc);
        return arc;
    }

    // Full 360-degree circle
    std::shared_ptr<Geom::Arc2D> add_circle(const Geom::Point2D& center, double radius) {
        auto circle = Geom::Arc2D::Circle(center, radius);
        curves.push_back(circle);
        return circle;
    }

    // Centered parametric rectangle
    void add_rectangle(double width, double height, const Geom::Point2D& center = {0.0, 0.0}) {
        double hw = width * 0.5;
        double hh = height * 0.5;

        Geom::Point2D p0(center.u - hw, center.v - hh);
        Geom::Point2D p1(center.u + hw, center.v - hh);
        Geom::Point2D p2(center.u + hw, center.v + hh);
        Geom::Point2D p3(center.u - hw, center.v + hh);

        add_line(p0, p1);
        add_line(p1, p2);
        add_line(p2, p3);
        add_line(p3, p0);
    }

    std::shared_ptr<Geom::CubicBezier2D> add_bezier(
        const Geom::Point2D& p0,
        const Geom::Point2D& p1,
        const Geom::Point2D& p2,
        const Geom::Point2D& p3
    ) {
        auto bezier = std::make_shared<Geom::CubicBezier2D>(p0, p1, p2, p3);
        curves.push_back(bezier);
        return bezier;
    }

    void add_curve(std::shared_ptr<Geom::Curve2D> curve) {
        if (curve) curves.push_back(std::move(curve));
    }

    // =========================================================================
    // Profile Extraction: Wire loop detection, nesting, and validation
    // =========================================================================

    [[nodiscard]] std::vector<SketchProfile> extract_profiles(double tol = Geom::CAD_TOLERANCE) const {
        std::vector<SketchProfile> profiles;
        if (curves.empty()) return profiles;

        // Step 1: Extract individual continuous wires (closed loops or open chains)
        std::vector<Wire2D> closedLoops;
        std::vector<Wire2D> openChains;

        std::vector<bool> visited(curves.size(), false);

        // Handle single-curve self-closed entities (e.g. full 360-deg circles)
        for (size_t i = 0; i < curves.size(); ++i) {
            if (curves[i]->start_point().approx_equal(curves[i]->end_point(), tol)) {
                closedLoops.emplace_back(std::vector<std::shared_ptr<Geom::Curve2D>>{curves[i]}, true);
                visited[i] = true;
            }
        }

        // Chain remaining multi-segment curves
        for (size_t i = 0; i < curves.size(); ++i) {
            if (visited[i]) continue;

            visited[i] = true;
            std::vector<std::shared_ptr<Geom::Curve2D>> currentWire;
            currentWire.push_back(curves[i]);

            Geom::Point2D loopStart = curves[i]->start_point();
            Geom::Point2D currentEnd = curves[i]->end_point();

            bool extended = true;
            while (extended) {
                // If current end matches original loop start, loop is closed!
                if (currentEnd.approx_equal(loopStart, tol)) {
                    break;
                }

                extended = false;
                for (size_t j = 0; j < curves.size(); ++j) {
                    if (visited[j]) continue;

                    // Forward match: curves[j].start == currentEnd
                    if (curves[j]->start_point().approx_equal(currentEnd, tol)) {
                        visited[j] = true;
                        currentWire.push_back(curves[j]);
                        currentEnd = curves[j]->end_point();
                        extended = true;
                        break;
                    }
                    // Reverse match: curves[j].end == currentEnd
                    else if (curves[j]->end_point().approx_equal(currentEnd, tol)) {
                        visited[j] = true;
                        auto rev = curves[j]->reversed();
                        currentWire.push_back(rev);
                        currentEnd = rev->end_point();
                        extended = true;
                        break;
                    }
                }
            }

            bool isClosed = currentEnd.approx_equal(loopStart, tol);
            if (isClosed) {
                closedLoops.emplace_back(std::move(currentWire), true);
            } else {
                openChains.emplace_back(std::move(currentWire), false);
            }
        }

        // Add open chains directly as Open Profiles
        for (auto& openWire : openChains) {
            profiles.push_back(SketchProfile::Open(std::move(openWire)));
        }

        // Step 2: Classify closed loops by nesting hierarchy
        if (closedLoops.empty()) {
            return profiles;
        }

        // Compute containment count for each closed loop:
        // A loop with containment count % 2 == 0 is an Outer Boundary.
        // A loop with containment count % 2 == 1 is an Inner Hole.
        size_t numLoops = closedLoops.size();
        std::vector<int> containmentCount(numLoops, 0);
        std::vector<int> immediateParent(numLoops, -1);

        for (size_t i = 0; i < numLoops; ++i) {
            // Pick a representative point on loop i (e.g. midpoint of first segment)
            Geom::Point2D samplePt = closedLoops[i].curves.front()->point_at(0.5);

            for (size_t j = 0; j < numLoops; ++j) {
                if (i == j) continue;
                if (closedLoops[j].contains_point(samplePt)) {
                    containmentCount[i]++;
                    // The parent is the tightest containing loop (smallest area)
                    if (immediateParent[i] == -1 ||
                        std::abs(closedLoops[j].signed_area()) < std::abs(closedLoops[immediateParent[i]].signed_area())) {
                        immediateParent[i] = static_cast<int>(j);
                    }
                }
            }
        }

        // Group outer boundaries with their immediate child holes
        std::map<int, std::vector<Wire2D>> outerToHoles;
        std::vector<int> outerIndices;

        for (size_t i = 0; i < numLoops; ++i) {
            if (containmentCount[i] % 2 == 0) {
                // Outer boundary
                outerIndices.push_back(static_cast<int>(i));
            }
        }

        for (size_t i = 0; i < numLoops; ++i) {
            if (containmentCount[i] % 2 == 1) {
                // Hole loop
                int p = immediateParent[i];
                if (p >= 0) {
                    outerToHoles[p].push_back(closedLoops[i]);
                }
            }
        }

        for (int outerIdx : outerIndices) {
            auto outerWire = closedLoops[outerIdx];
            std::vector<Wire2D> holes = outerToHoles[outerIdx];
            profiles.push_back(SketchProfile::Closed(std::move(outerWire), std::move(holes)));
        }

        return profiles;
    }
};

} // namespace Tessera::Sketch
