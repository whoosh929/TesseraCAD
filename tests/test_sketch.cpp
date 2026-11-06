#include "engine/geom/Curves2D.hpp"
#include "engine/sketch/Sketch.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/sketch/ProfileTessellator.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace Tessera::Geom;
using namespace Tessera::Sketch;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Line " << __LINE__ << ": " << msg << "\n"; \
            return false; \
        } \
    } while(false)

bool TestCurves2D() {
    std::cout << "[Test] 2D Curve Primitives (Line, Arc, Bezier)...\n";

    // 1. LineSegment2D
    LineSegment2D line(Point2D(1.0, 2.0), Point2D(4.0, 6.0));
    TEST_ASSERT(std::abs(line.length() - 5.0) < CAD_EPSILON, "Line length != 5");
    TEST_ASSERT(line.point_at(0.0).approx_equal(Point2D(1.0, 2.0)), "Line point_at(0) incorrect");
    TEST_ASSERT(line.point_at(1.0).approx_equal(Point2D(4.0, 6.0)), "Line point_at(1) incorrect");
    TEST_ASSERT(line.point_at(0.5).approx_equal(Point2D(2.5, 4.0)), "Line point_at(0.5) incorrect");
    TEST_ASSERT(line.tangent_at(0.5).approx_equal(Vector2D(0.6, 0.8)), "Line tangent incorrect");

    auto revLine = line.reversed();
    TEST_ASSERT(revLine->start_point().approx_equal(Point2D(4.0, 6.0)), "Reversed line start != original end");

    // 2. Arc2D (90-degree quadrant arc from (3, 0) to (0, 3) around origin)
    Arc2D arc(Point2D(0.0, 0.0), 3.0, 0.0, PI * 0.5, true);
    TEST_ASSERT(std::abs(arc.length() - 3.0 * PI * 0.5) < CAD_EPSILON, "Arc length incorrect");
    TEST_ASSERT(arc.start_point().approx_equal(Point2D(3.0, 0.0)), "Arc start incorrect");
    TEST_ASSERT(arc.end_point().approx_equal(Point2D(0.0, 3.0)), "Arc end incorrect");
    TEST_ASSERT(arc.point_at(0.5).approx_equal(Point2D(3.0 * std::cos(PI * 0.25), 3.0 * std::sin(PI * 0.25))), "Arc midpoint incorrect");

    // Full circle
    auto circle = Arc2D::Circle(Point2D(5.0, 5.0), 2.5);
    TEST_ASSERT(std::abs(circle->length() - TWO_PI * 2.5) < CAD_EPSILON, "Circle circumference incorrect");
    TEST_ASSERT(circle->start_point().approx_equal(circle->end_point()), "Circle start != end");

    // 3. CubicBezier2D
    CubicBezier2D bez(Point2D(0, 0), Point2D(0, 2), Point2D(2, 2), Point2D(2, 0));
    TEST_ASSERT(bez.point_at(0.0).approx_equal(Point2D(0, 0)), "Bezier point_at(0) incorrect");
    TEST_ASSERT(bez.point_at(1.0).approx_equal(Point2D(2, 0)), "Bezier point_at(1) incorrect");
    TEST_ASSERT(bez.point_at(0.5).v > 1.0, "Bezier midpoint height incorrect");

    return true;
}

bool TestRectangleProfileExtraction() {
    std::cout << "[Test] Closed Rectangle Profile Extraction...\n";

    Sketch sketch;
    sketch.add_rectangle(6.0, 4.0, Point2D(0.0, 0.0));

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 extracted profile");

    const auto& prof = profiles[0];
    TEST_ASSERT(prof.is_closed, "Rectangle profile should be closed");
    TEST_ASSERT(!prof.has_holes(), "Rectangle profile should have no holes");

    // Area should be exactly 6.0 * 4.0 = 24.0
    double area = prof.net_area();
    TEST_ASSERT(std::abs(area - 24.0) < 1e-4, "Rectangle profile area != 24.0");

    // Outer wire must be counter-clockwise
    TEST_ASSERT(prof.outer_wire.is_counter_clockwise(), "Rectangle outer wire must be CCW");

    // Point in polygon containment
    TEST_ASSERT(prof.outer_wire.contains_point(Point2D(0.0, 0.0)), "Origin should be inside rectangle");
    TEST_ASSERT(prof.outer_wire.contains_point(Point2D(2.5, 1.5)), "Inner point should be inside");
    TEST_ASSERT(!prof.outer_wire.contains_point(Point2D(4.0, 0.0)), "Outside point should not be inside");

    return true;
}

bool TestCircleProfileExtraction() {
    std::cout << "[Test] Closed Circle Profile Extraction...\n";

    Sketch sketch;
    sketch.add_circle(Point2D(2.0, 3.0), 4.0);

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 extracted circle profile");

    const auto& prof = profiles[0];
    TEST_ASSERT(prof.is_closed, "Circle profile should be closed");

    // Approximate area with circle discretization
    double expectedArea = PI * 4.0 * 4.0; // 16 * pi ≈ 50.265
    double area = prof.net_area(0.01, 32);
    TEST_ASSERT(std::abs(area - expectedArea) / expectedArea < 0.02, "Circle discretized area divergence > 2%");

    return true;
}

bool TestAnnulusHoleExtraction() {
    std::cout << "[Test] Concentric Annulus Profile with Hole...\n";

    Sketch sketch;
    // Outer circle radius 5.0, inner circle (hole) radius 2.0
    sketch.add_circle(Point2D(0.0, 0.0), 5.0);
    sketch.add_circle(Point2D(0.0, 0.0), 2.0);

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 composite profile with hole");

    const auto& prof = profiles[0];
    TEST_ASSERT(prof.is_closed, "Annulus profile should be closed");
    TEST_ASSERT(prof.has_holes(), "Annulus profile should have 1 hole");
    TEST_ASSERT(prof.inner_holes.size() == 1, "Annulus inner_holes size != 1");

    // Outer wire is CCW, inner hole is CW
    TEST_ASSERT(prof.outer_wire.is_counter_clockwise(), "Outer wire should be CCW");
    TEST_ASSERT(!prof.inner_holes[0].is_counter_clockwise(), "Inner hole should be CW");

    // Expected net area: pi * (5^2 - 2^2) = 21 * pi ≈ 65.973
    double expectedArea = PI * (25.0 - 4.0);
    double area = prof.net_area(0.01, 32);
    TEST_ASSERT(std::abs(area - expectedArea) / expectedArea < 0.02, "Annulus net area divergence > 2%");

    return true;
}

bool TestOpenWireProfileExtraction() {
    std::cout << "[Test] Open Wire Profile Extraction (Surfacing)...\n";

    Sketch sketch;
    // Open path: (0, 0) -> (2, 0) -> (2, 3)
    sketch.add_line(Point2D(0.0, 0.0), Point2D(2.0, 0.0));
    sketch.add_line(Point2D(2.0, 0.0), Point2D(2.0, 3.0));

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 open profile");

    const auto& prof = profiles[0];
    TEST_ASSERT(!prof.is_closed, "Open profile should NOT be closed");
    TEST_ASSERT(std::abs(prof.outer_wire.length() - 5.0) < CAD_EPSILON, "Open wire length != 5.0");

    return true;
}

bool TestMultipleDisjointProfiles() {
    std::cout << "[Test] Multiple Disjoint Profiles in One Sketch...\n";

    Sketch sketch;
    // Rectangle A at (-10, 0) of size 2x2
    sketch.add_rectangle(2.0, 2.0, Point2D(-10.0, 0.0));
    // Rectangle B at (+10, 0) of size 4x4
    sketch.add_rectangle(4.0, 4.0, Point2D(10.0, 0.0));

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 2, "Expected 2 disjoint profiles");

    double area0 = profiles[0].net_area();
    double area1 = profiles[1].net_area();
    double totalArea = area0 + area1;

    // Total area: 2*2 + 4*4 = 4 + 16 = 20
    TEST_ASSERT(std::abs(totalArea - 20.0) < 1e-4, "Combined area of disjoint profiles != 20.0");

    return true;
}

bool TestProfileTessellation() {
    std::cout << "[Test] Profile Tessellation (Ear-Clipping & Bridge Holes)...\n";

    // 1. Simple rectangle tessellation
    Sketch rectSketch;
    rectSketch.add_rectangle(4.0, 3.0);
    auto rectProfiles = rectSketch.extract_profiles();
    TEST_ASSERT(rectProfiles.size() == 1, "Expected 1 rect profile");

    auto mesh = ProfileTessellator::Triangulate(rectProfiles[0]);
    TEST_ASSERT(mesh.vertices.size() == 4, "Rectangle vertex count != 4");
    TEST_ASSERT(mesh.indices.size() == 6, "Rectangle index count != 6 (2 triangles)");

    // Compute area of triangulated mesh
    double meshArea = 0.0;
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        const auto& p0 = mesh.vertices[mesh.indices[i]];
        const auto& p1 = mesh.vertices[mesh.indices[i+1]];
        const auto& p2 = mesh.vertices[mesh.indices[i+2]];
        // Triangle signed area
        meshArea += 0.5 * std::abs((p1.u - p0.u) * (p2.v - p0.v) - (p2.u - p0.u) * (p1.v - p0.v));
    }
    TEST_ASSERT(std::abs(meshArea - 12.0) < 1e-6, "Triangulated rectangle mesh area != 12.0");

    // 2. Rectangle with rectangular hole (Outer 10x10, Inner 4x4)
    Sketch hollowSketch;
    hollowSketch.add_rectangle(10.0, 10.0, Point2D(0.0, 0.0));
    hollowSketch.add_rectangle(4.0, 4.0, Point2D(0.0, 0.0));

    auto hollowProfiles = hollowSketch.extract_profiles();
    TEST_ASSERT(hollowProfiles.size() == 1, "Expected 1 hollow profile");
    TEST_ASSERT(hollowProfiles[0].has_holes(), "Profile should contain hole");

    auto hollowMesh = ProfileTessellator::Triangulate(hollowProfiles[0]);
    TEST_ASSERT(!hollowMesh.indices.empty(), "Hollow mesh indices should not be empty");

    double hollowArea = 0.0;
    for (size_t i = 0; i < hollowMesh.indices.size(); i += 3) {
        const auto& p0 = hollowMesh.vertices[hollowMesh.indices[i]];
        const auto& p1 = hollowMesh.vertices[hollowMesh.indices[i+1]];
        const auto& p2 = hollowMesh.vertices[hollowMesh.indices[i+2]];
        hollowArea += 0.5 * std::abs((p1.u - p0.u) * (p2.v - p0.v) - (p2.u - p0.u) * (p1.v - p0.v));
    }

    // Expected area: 10*10 - 4*4 = 100 - 16 = 84.0
    TEST_ASSERT(std::abs(hollowArea - 84.0) < 1e-4, "Triangulated hollow mesh area != 84.0");

    return true;
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "     TesseraCAD 2D Sketch & Profile Unit Tests          \n";
    std::cout << "=========================================================\n";

    bool passed = true;
    passed &= TestCurves2D();
    passed &= TestRectangleProfileExtraction();
    passed &= TestCircleProfileExtraction();
    passed &= TestAnnulusHoleExtraction();
    passed &= TestOpenWireProfileExtraction();
    passed &= TestMultipleDisjointProfiles();
    passed &= TestProfileTessellation();

    std::cout << "=========================================================\n";
    if (passed) {
        std::cout << "[SUCCESS] ALL 2D SKETCH & PROFILE TESTS PASSED!\n";
        return 0;
    } else {
        std::cerr << "[FAILURE] SOME TESTS FAILED!\n";
        return 1;
    }
}
