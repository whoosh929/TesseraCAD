#include "engine/geom/Plane.hpp"
#include "engine/geom/Axis3D.hpp"
#include "engine/sketch/Sketch.hpp"
#include "engine/modeling/Topology.hpp"
#include "engine/modeling/ExtrudeOp.hpp"
#include "engine/modeling/RevolveOp.hpp"
#include "engine/modeling/Tessellator.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace Tessera;
using namespace Tessera::Geom;
using namespace Tessera::Modeling;
using Sketch2D = Tessera::Sketch::Sketch;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Line " << __LINE__ << ": " << msg << "\n"; \
            return false; \
        } \
    } while(false)

bool TestSolidExtrude() {
    std::cout << "[Test] Solid Linear Extrude of Rectangle...\n";

    Sketch2D sketch(Plane::XY());
    sketch.add_rectangle(4.0, 2.0); // 4x2 rectangle

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 profile");

    ExtrudeOp::Params params;
    params.start_depth = 0.0;
    params.end_depth = 5.0;
    params.direction = Vector3D(0.0, 0.0, 1.0);
    params.is_solid = true;

    auto body = ExtrudeOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body != nullptr, "Body is null");
    TEST_ASSERT(body->is_solid(), "Body must be solid");
    TEST_ASSERT(!body->is_sheet(), "Body must not be sheet");
    TEST_ASSERT(body->shell_count() == 1, "Expected 1 shell");

    auto shell = body->shells[0];
    TEST_ASSERT(shell->is_closed, "Shell must be closed");

    // Face count: 1 bottom cap + 1 top cap + 4 side walls = 6 faces (like a standard Box B-Rep)
    TEST_ASSERT(shell->face_count() == 6, "Expected 6 faces for extruded rectangle solid");

    // Tessellation verification
    CADMesh mesh = Tessellator::Tessellate(*body);
    TEST_ASSERT(!mesh.vertices.empty(), "Mesh vertices empty");
    TEST_ASSERT(!mesh.indices.empty(), "Mesh indices empty");
    TEST_ASSERT(!mesh.featureEdgeIndices.empty(), "Feature edges empty");

    // 6 faces * 2 triangles per face = 12 triangles = 36 indices
    TEST_ASSERT(mesh.indices.size() == 36, "Box mesh index count != 36");

    return true;
}

bool TestSurfaceExtrude() {
    std::cout << "[Test] Surface/Sheet Extrude of Open Wire...\n";

    Sketch2D sketch(Plane::XY());
    sketch.add_line(Point2D(0, 0), Point2D(5, 0));
    sketch.add_line(Point2D(5, 0), Point2D(5, 5));

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 open profile");
    TEST_ASSERT(!profiles[0].is_closed, "Profile should be open");

    ExtrudeOp::Params params;
    params.start_depth = 0.0;
    params.end_depth = 3.0;
    params.is_solid = false; // Open sheet

    auto body = ExtrudeOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body != nullptr, "Body is null");
    TEST_ASSERT(!body->is_solid(), "Surface body must NOT be solid");
    TEST_ASSERT(body->is_sheet(), "Body must be a sheet/surface");

    auto shell = body->shells[0];
    TEST_ASSERT(!shell->is_closed, "Surface shell must be open");

    // 2 segments swept -> 2 side wall faces (no caps)
    TEST_ASSERT(shell->face_count() == 2, "Expected 2 faces for 2-segment open swept sheet");

    CADMesh mesh = Tessellator::Tessellate(*body);
    // 2 quad faces = 4 triangles = 12 indices
    TEST_ASSERT(mesh.indices.size() == 12, "Sheet mesh index count != 12");

    return true;
}

bool TestHollowExtrude() {
    std::cout << "[Test] Solid Extrude of Hollow Profile (with Hole)...\n";

    Sketch2D sketch(Plane::XY());
    sketch.add_rectangle(10.0, 10.0);
    sketch.add_rectangle(4.0, 4.0);

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 composite profile");
    TEST_ASSERT(profiles[0].has_holes(), "Profile should have hole");

    ExtrudeOp::Params params;
    params.start_depth = 0.0;
    params.end_depth = 4.0;
    params.is_solid = true;

    auto body = ExtrudeOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body->is_solid(), "Hollow body should be solid");

    auto shell = body->shells[0];
    // 1 bottom cap + 1 top cap + 4 outer walls + 4 inner hole walls = 10 faces
    TEST_ASSERT(shell->face_count() == 10, "Expected 10 faces for extruded hollow box");

    CADMesh mesh = Tessellator::Tessellate(*body);
    TEST_ASSERT(!mesh.indices.empty(), "Hollow extruded mesh empty");

    return true;
}

bool TestFullRevolve() {
    std::cout << "[Test] Full 360-Degree Revolve (Torus / Ring)...\n";

    // Circle centered at (5, 0) of radius 1.5 in XY plane
    Sketch2D sketch(Plane::XY());
    sketch.add_circle(Point2D(5.0, 0.0), 1.5);

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 profile");

    RevolveOp::Params params;
    params.axis = Axis3D::Y(); // Revolve around Y-axis
    params.angle_radians = TWO_PI;
    params.angular_segments = 16;
    params.is_solid = true;

    auto body = RevolveOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body != nullptr, "Revolve body is null");
    TEST_ASSERT(body->is_solid(), "Full revolve of closed profile must be solid");

    auto shell = body->shells[0];
    TEST_ASSERT(shell->is_closed, "Full revolve shell must be closed");

    CADMesh mesh = Tessellator::Tessellate(*body);
    TEST_ASSERT(!mesh.vertices.empty(), "Revolve mesh vertices empty");
    TEST_ASSERT(!mesh.indices.empty(), "Revolve mesh indices empty");

    return true;
}

bool TestPartialRevolve() {
    std::cout << "[Test] Partial 90-Degree Revolve with Angular Caps...\n";

    Sketch2D sketch(Plane::XY());
    sketch.add_rectangle(2.0, 2.0, Point2D(4.0, 0.0));

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 profile");

    RevolveOp::Params params;
    params.axis = Axis3D::Y();
    params.angle_radians = PI * 0.5; // 90 degrees
    params.angular_segments = 8;
    params.is_solid = true;

    auto body = RevolveOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body->is_solid(), "Partial solid revolve must be solid");

    auto shell = body->shells[0];
    TEST_ASSERT(shell->is_closed, "Partial solid revolve shell must be closed with caps");

    // 2 angular caps (start at 0, end at 90 deg) + 4 profile segments * 8 slices = 2 + 32 = 34 faces
    TEST_ASSERT(shell->face_count() == 34, "Expected 34 faces for 90-deg revolve with 8 slices");

    CADMesh mesh = Tessellator::Tessellate(*body);
    TEST_ASSERT(!mesh.indices.empty(), "Partial revolve mesh indices empty");

    return true;
}

bool TestRevolveCylindricalThicknessFromSketch() {
    std::cout << "[Test] Revolve of Sketch Drawing Dictating Cylindrical Model Thickness...\n";

    // 2D sketch rectangle on XY plane:
    // Left edge at u = 1.5, right edge at u = 3.5 (wall thickness = 3.5 - 1.5 = 2.0 m)
    // Bottom edge at v = 0.0, top edge at v = 5.0 (height = 5.0 m)
    Sketch2D sketch(Plane::XY());
    sketch.add_line(Point2D(1.5, 0.0), Point2D(3.5, 0.0));
    sketch.add_line(Point2D(3.5, 0.0), Point2D(3.5, 5.0));
    sketch.add_line(Point2D(3.5, 5.0), Point2D(1.5, 5.0));
    sketch.add_line(Point2D(1.5, 5.0), Point2D(1.5, 0.0));

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 profile for rectangular sketch");

    RevolveOp::Params params;
    params.axis = Axis3D::Y(); // Revolve around in-plane vertical axis (V-axis)
    params.angle_radians = TWO_PI; // 360 deg
    params.angular_segments = 32;
    params.is_solid = true;

    auto body = RevolveOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body != nullptr, "Revolve body is null");
    TEST_ASSERT(body->is_solid(), "360-degree revolve of closed profile must be solid");

    CADMesh mesh = Tessellator::Tessellate(*body);
    TEST_ASSERT(!mesh.vertices.empty(), "Mesh vertices empty");
    TEST_ASSERT(!mesh.indices.empty(), "Mesh indices empty");

    // Verify bounding box matches sketch:
    // Radius goes from 1.5 to 3.5: x and z should be in [-3.5, 3.5]
    // Height goes from 0.0 to 5.0: y should be in [0.0, 5.0]
    float minX = 1e9f, maxX = -1e9f;
    float minY = 1e9f, maxY = -1e9f;
    float minZ = 1e9f, maxZ = -1e9f;
    float minR = 1e9f, maxR = -1e9f;

    bool foundTopCap = false;
    bool foundBottomCap = false;
    bool foundOuterWall = false;
    bool foundInnerWall = false;

    for (const auto& v : mesh.vertices) {
        float x = v.position[0];
        float y = v.position[1];
        float z = v.position[2];
        float r = std::hypot(x, z);

        minX = std::min(minX, x); maxX = std::max(maxX, x);
        minY = std::min(minY, y); maxY = std::max(maxY, y);
        minZ = std::min(minZ, z); maxZ = std::max(maxZ, z);
        minR = std::min(minR, r); maxR = std::max(maxR, r);

        // Check top cap normal (y ≈ 5.0, normal points +Y)
        if (std::abs(y - 5.0f) < 0.05f && v.normal[1] > 0.9f) {
            foundTopCap = true;
        }
        // Check bottom cap normal (y ≈ 0.0, normal points -Y)
        if (std::abs(y - 0.0f) < 0.05f && v.normal[1] < -0.9f) {
            foundBottomCap = true;
        }
        // Check outer cylinder normal (r ≈ 3.5, normal points radially outward)
        if (std::abs(r - 3.5f) < 0.05f) {
            float radDot = (x * v.normal[0] + z * v.normal[2]) / r;
            if (radDot > 0.9f) foundOuterWall = true;
        }
        // Check inner cylinder normal (r ≈ 1.5, normal points radially inward)
        if (std::abs(r - 1.5f) < 0.05f) {
            float radDot = (x * v.normal[0] + z * v.normal[2]) / r;
            if (radDot < -0.9f) foundInnerWall = true;
        }
    }

    TEST_ASSERT(std::abs(minR - 1.5f) < 0.05f, "Inner radius must be ~1.5m (dictated by sketch)");
    TEST_ASSERT(std::abs(maxR - 3.5f) < 0.05f, "Outer radius must be ~3.5m (dictated by sketch)");
    TEST_ASSERT(std::abs((maxR - minR) - 2.0f) < 0.1f, "Wall thickness must be 2.0m (dictated by sketch)");
    TEST_ASSERT(std::abs(minY - 0.0f) < 0.05f, "Height base must be 0.0m (dictated by sketch)");
    TEST_ASSERT(std::abs(maxY - 5.0f) < 0.05f, "Height top must be 5.0m (dictated by sketch)");

    TEST_ASSERT(foundTopCap, "Top cap normal must point +Y");
    TEST_ASSERT(foundBottomCap, "Bottom cap normal must point -Y");
    TEST_ASSERT(foundOuterWall, "Outer wall normal must point radially outward");
    TEST_ASSERT(foundInnerWall, "Inner wall normal must point radially inward");

    return true;
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "     TesseraCAD Modeling Operators Unit Tests            \n";
    std::cout << "=========================================================\n";

    bool passed = true;
    passed &= TestSolidExtrude();
    passed &= TestSurfaceExtrude();
    passed &= TestHollowExtrude();
    passed &= TestFullRevolve();
    passed &= TestPartialRevolve();
    passed &= TestRevolveCylindricalThicknessFromSketch();

    std::cout << "=========================================================\n";
    if (passed) {
        std::cout << "[SUCCESS] ALL MODELING OPERATOR TESTS PASSED!\n";
        return 0;
    } else {
        std::cerr << "[FAILURE] SOME TESTS FAILED!\n";
        return 1;
    }
}
