#include "engine/geom/Curves2D.hpp"
#include "engine/sketch/Sketch.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/sketch/ProfileTessellator.hpp"
#include "engine/sketch/SketchFilletChamfer.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/DatumPlaneNode.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/concurrency/ThreadPool.hpp"
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

bool TestSketch2DFilletAndChamfer() {
    std::cout << "[Test] 2D Sketch Corner Fillet and Chamfer Operations...\n";

    // 1. Rectangle 4x4: (-2, -2), (2, -2), (2, 2), (-2, 2)
    std::vector<std::array<float, 2>> rect = {
        {-2.0f, -2.0f},
        { 2.0f, -2.0f},
        { 2.0f,  2.0f},
        {-2.0f,  2.0f}
    };

    // Apply Chamfer to corner #2 (2, 2) with distance 0.5
    bool chamferOk = ApplyChamfer2D(rect, 2, 0.5f, true);
    TEST_ASSERT(chamferOk, "ApplyChamfer2D should succeed on valid corner");
    // Corner 2 replaced with 2 vertices: now 5 vertices total
    TEST_ASSERT(rect.size() == 5, "Chamfered rectangle should have 5 vertices");

    // Check beveled vertices around corner (2, 2):
    // Incoming from (2, -2): point at (2, 1.5)
    // Outgoing to (-2, 2): point at (1.5, 2)
    bool foundV1 = false, foundV2 = false;
    for (const auto& pt : rect) {
        if (std::abs(pt[0] - 2.0f) < 1e-4f && std::abs(pt[1] - 1.5f) < 1e-4f) foundV1 = true;
        if (std::abs(pt[0] - 1.5f) < 1e-4f && std::abs(pt[1] - 2.0f) < 1e-4f) foundV2 = true;
    }
    TEST_ASSERT(foundV1 && foundV2, "Chamfer points must be at distance 0.5m along each adjacent edge");

    // 2. Apply Fillet to corner #0 (-2, -2) with radius 0.4
    size_t prevCount = rect.size();
    bool filletOk = ApplyFillet2D(rect, 0, 0.4f, true, 6);
    TEST_ASSERT(filletOk, "ApplyFillet2D should succeed on valid corner");
    // Corner 0 replaced with 7 vertices (6 segments) -> net +6 vertices
    TEST_ASSERT(rect.size() == prevCount + 6, "Filleted rectangle should add arc segments");

    // Center of fillet at corner (-2, -2) with 90 deg angle is at (-1.6, -1.6)
    // Check that all generated arc vertices lie at distance ~0.4 from center
    for (size_t i = 0; i <= 6; ++i) {
        float x = rect[i][0];
        float y = rect[i][1];
        float dToCenter = std::hypot(x - (-1.6f), y - (-1.6f));
        TEST_ASSERT(std::abs(dToCenter - 0.4f) < 1e-3f, "Fillet arc points must have radial distance = 0.4m from arc center");
    }

    // 3. Apply Fillet with Angle mode to corner #1 (2, -2) with angle 60 deg and refDist 0.5m
    std::vector<std::array<float, 2>> rectAngle = {
        {-2.0f, -2.0f},
        { 2.0f, -2.0f},
        { 2.0f,  2.0f},
        {-2.0f,  2.0f}
    };
    bool filletAngleOk = ApplyFillet2D(rectAngle, 1, 60.0f, true, 6, true, 0.5f);
    TEST_ASSERT(filletAngleOk, "ApplyFillet2D with angle mode should succeed");
    TEST_ASSERT(rectAngle.size() == 4 + 6, "Filleted rectangle in angle mode should add arc segments");

    return true;
}

bool TestSketchAxisAndOriginConstraints() {
    std::cout << "[Test] 2D Sketch Axis and Origin Dimension Constraints...\n";

    // 1. Rectangle profile with 4 corners
    // Corner 0: (1.0, 1.0), Corner 1: (5.0, 1.0), Corner 2: (5.0, 4.0), Corner 3: (1.0, 4.0)
    std::vector<std::array<float, 2>> rect = {
        {1.0f, 1.0f},
        {5.0f, 1.0f},
        {5.0f, 4.0f},
        {1.0f, 4.0f}
    };

    // Constrain Corner 0 to Origin distance = 5.0m
    std::map<size_t, float> edgeConstraints; // no edge constraints initially
    std::map<size_t, Tessera::VertexAxisConstraint> vertexConstraints;
    Tessera::VertexAxisConstraint c0;
    c0.lockOriginDist = true;
    c0.originDist = 5.0f;
    vertexConstraints[0] = c0;

    Tessera::SketchNode::EnforceConstraints(rect, edgeConstraints, vertexConstraints, true);

    float dOrig = std::hypot(rect[0][0], rect[0][1]);
    TEST_ASSERT(std::abs(dOrig - 5.0f) < 1e-4f, "Corner 0 origin distance must be constrained to 5.0m");
    // Initially (1, 1), normalized is (1/sqrt(2), 1/sqrt(2)), scaled by 5 is ~3.5355m
    TEST_ASSERT(std::abs(rect[0][0] - (5.0f / std::sqrt(2.0f))) < 1e-3f, "Corner 0 U coordinate must lie on radial vector");
    TEST_ASSERT(std::abs(rect[0][1] - (5.0f / std::sqrt(2.0f))) < 1e-3f, "Corner 0 V coordinate must lie on radial vector");

    // 2. Constrain Corner 0 to U-Axis (distance |V| = 2.0) and V-Axis (distance |U| = 3.0)
    vertexConstraints.clear();
    Tessera::VertexAxisConstraint cAxes;
    cAxes.lockUAxisDist = true;
    cAxes.uAxisDist = 2.0f;
    cAxes.lockVAxisDist = true;
    cAxes.vAxisDist = 3.0f;
    vertexConstraints[0] = cAxes;

    rect[0] = {1.0f, 1.0f};
    Tessera::SketchNode::EnforceConstraints(rect, edgeConstraints, vertexConstraints, true);

    TEST_ASSERT(std::abs(rect[0][0] - 3.0f) < 1e-4f, "Corner 0 V-Axis distance (|U|) must be constrained to 3.0m");
    TEST_ASSERT(std::abs(rect[0][1] - 2.0f) < 1e-4f, "Corner 0 U-Axis distance (|V|) must be constrained to 2.0m");

    // 3. Combined edge length constraint and origin constraint
    // Edge 0 (from Corner 0 to Corner 1) has target length = 4.0m
    // Corner 0 is locked to (3.0, 2.0) via U/V axis constraints
    edgeConstraints[0] = 4.0f;
    rect[1] = {10.0f, 2.0f}; // distorted
    Tessera::SketchNode::EnforceConstraints(rect, edgeConstraints, vertexConstraints, true);

    float edge0Len = std::hypot(rect[1][0] - rect[0][0], rect[1][1] - rect[0][1]);
    TEST_ASSERT(std::abs(edge0Len - 4.0f) < 1e-4f, "Edge 0 length must satisfy 4.0m constraint");
    TEST_ASSERT(std::abs(rect[0][0] - 3.0f) < 1e-4f, "Corner 0 must remain locked to V-axis distance 3.0m");
    TEST_ASSERT(std::abs(rect[0][1] - 2.0f) < 1e-4f, "Corner 0 must remain locked to U-axis distance 2.0m");

    // 4. Projected Datum Axis Line Constraint
    // Define a datum axis line passing through (0, 0) along direction (1, 0) [horizontal line]
    Tessera::ProjectedDatumAxis dAx;
    dAx.id = 101;
    dAx.name = "RefAxis_X";
    dAx.p0 = {0.0f, 0.0f};
    dAx.dir = {1.0f, 0.0f};
    dAx.valid = true;

    vertexConstraints.clear();
    edgeConstraints.clear();
    Tessera::VertexAxisConstraint cDAx;
    cDAx.lockDatumAxisDist = true;
    cDAx.datumAxisDist = 6.5f;
    cDAx.datumAxisId = 101;
    vertexConstraints[0] = cDAx;

    rect[0] = {2.0f, 1.0f};
    Tessera::SketchNode::EnforceConstraints(rect, edgeConstraints, vertexConstraints, true, -1, {dAx});

    // Distance to horizontal line Y=0 is |V|
    TEST_ASSERT(std::abs(std::abs(rect[0][1]) - 6.5f) < 1e-4f, "Corner 0 perpendicular distance to datum axis must be 6.5m");

    // 5. SketchNode integration: set and get vertex axis constraints
    Tessera::SketchNode sketchNode("TestSketch", Tessera::SketchPlane::XY_Front);
    sketchNode.SetShapes({ rect });
    sketchNode.SetVertexOriginConstraint(0, 0, 5.0f);
    TEST_ASSERT(sketchNode.IsVertexOriginConstrained(0, 0), "SketchNode must register vertex origin constraint");
    TEST_ASSERT(std::abs(sketchNode.GetVertexOriginDistance(0, 0) - 5.0f) < 1e-4f, "SketchNode must store origin distance 5.0m");

    sketchNode.RemoveVertexOriginConstraint(0, 0);
    TEST_ASSERT(!sketchNode.IsVertexOriginConstrained(0, 0), "SketchNode must allow removing vertex origin constraint");

    return true;
}

bool TestExtrudeFromFaceSketch() {
    std::cout << "[Test] Extrude Feature From 2D Sketch on Model Face...\n";

    auto pool = std::make_shared<Tessera::ThreadPool>(2);
    Tessera::ParametricGraph graph(pool);
    Tessera::AtomicMeshExchange exchange;

    // 1. Base sketch and extrude
    std::vector<std::array<float, 2>> rect = {
        {-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, 2.0f}, {-2.0f, 2.0f}
    };
    auto sk1 = std::make_shared<Tessera::SketchNode>("Sketch_Base", Tessera::SketchPlane::XZ_Ground, rect);
    graph.AddNode(sk1);

    auto ext1 = std::make_shared<Tessera::ExtrudeNode>("Extrude_Base", sk1, 3.0f, Tessera::ExtrudeOperation::Boss, nullptr);
    graph.AddNode(ext1);

    ext1->Evaluate(nullptr, exchange);

    TEST_ASSERT(sk1->IsConsumed(), "Base sketch must be consumed by ext1");
    TEST_ASSERT(!ext1->IsConsumed(), "Base extrude must NOT be consumed");
    TEST_ASSERT(!ext1->GetCachedVertices().empty(), "Base extrude must have mesh vertices");

    // 2. Create DatumPlane on top face (normal = +Y, origin = (0, 3, 0))
    Tessera::FaceInfo topFace;
    bool foundFace = ext1->GetFace(0, topFace);
    TEST_ASSERT(foundFace, "Must find top face on ext1");

    auto planeNode = std::make_shared<Tessera::DatumPlaneNode>("Plane_Face_1", ext1, 0, topFace.plane, 0.0);
    planeNode->Evaluate(nullptr, exchange);
    graph.AddNode(planeNode);

    Tessera::Geom::Plane effPl = planeNode->GetPlane();
    auto skFace = std::make_shared<Tessera::SketchNode>("Sketch_Face_1", planeNode);
    skFace->SetParentFeature(ext1);
    skFace->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));

    // Draw circle profile on face sketch
    std::vector<std::array<float, 2>> circlePts;
    for (int i = 0; i < 16; ++i) {
        float angle = static_cast<float>(i) * (6.2831853f / 16.0f);
        circlePts.push_back({ std::cos(angle) * 1.0f, std::sin(angle) * 1.0f });
    }
    skFace->SetShapes({ circlePts });
    skFace->Evaluate(nullptr, exchange);
    graph.AddNode(skFace);

    std::cout << "skFace->IsConsumed(): " << skFace->IsConsumed() << "\n";
    std::cout << "skFace->GetCachedVertices().size(): " << skFace->GetCachedVertices().size() << "\n";
    std::cout << "skFace->HasCustomPlane(): " << skFace->HasCustomPlane() << "\n";

    // 3. Test available sketches in graph
    const auto& nodes = graph.GetNodes();
    std::vector<std::shared_ptr<Tessera::SketchNode>> availableSketches;
    for (const auto& n : nodes) {
        if (auto s = std::dynamic_pointer_cast<Tessera::SketchNode>(n)) {
            if (!s->IsConsumed()) {
                availableSketches.push_back(s);
            }
        }
    }
    std::cout << "availableSketches count: " << availableSketches.size() << "\n";
    for (const auto& s : availableSketches) {
        std::cout << "  - Available sketch: " << s->GetName() << "\n";
    }

    // 4. Create Extrude off skFace
    auto extFace = std::make_shared<Tessera::ExtrudeNode>("Extrude_Face_1", skFace, 1.5f, Tessera::ExtrudeOperation::Boss, ext1);
    graph.AddNode(extFace);
    extFace->Evaluate(nullptr, exchange);

    std::cout << "extFace->GetCachedVertices().size(): " << extFace->GetCachedVertices().size() << "\n";
    std::cout << "extFace->GetCachedIndices().size(): " << extFace->GetCachedIndices().size() << "\n";

    TEST_ASSERT(skFace->IsConsumed(), "skFace must now be consumed by extFace");
    TEST_ASSERT(!extFace->GetCachedVertices().empty(), "Extrude off face sketch must produce vertices");

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
    passed &= TestSketch2DFilletAndChamfer();
    passed &= TestSketchAxisAndOriginConstraints();
    passed &= TestExtrudeFromFaceSketch();

    std::cout << "=========================================================\n";
    if (passed) {
        std::cout << "[SUCCESS] ALL 2D SKETCH & PROFILE TESTS PASSED!\n";
        return 0;
    } else {
        std::cerr << "[FAILURE] SOME TESTS FAILED!\n";
        return 1;
    }
}

