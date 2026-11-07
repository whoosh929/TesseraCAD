#include "engine/geom/Plane.hpp"
#include "engine/geom/Axis3D.hpp"
#include "engine/sketch/Sketch.hpp"
#include "engine/modeling/Topology.hpp"
#include "engine/modeling/ExtrudeOp.hpp"
#include "engine/modeling/RevolveOp.hpp"
#include "engine/modeling/Tessellator.hpp"
#include "engine/modeling/BlendGeometry.hpp"
#include "engine/modeling/CornerBlend.hpp"
#include "engine/modeling/FilletOp.hpp"
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

bool TestNegativeExtrude() {
    std::cout << "[Test] Negative Depth Linear Extrude (Reversed Direction)...\n";

    Sketch2D sketch(Plane::XY());
    sketch.add_rectangle(4.0, 2.0); // 4x2 rectangle

    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(profiles.size() == 1, "Expected 1 profile");

    ExtrudeOp::Params params;
    params.start_depth = 0.0;
    params.end_depth = -4.0;
    params.direction = Vector3D(0.0, 0.0, 1.0);
    params.is_solid = true;

    auto body = ExtrudeOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body != nullptr, "Body is null");
    TEST_ASSERT(body->is_solid(), "Body must be solid");
    TEST_ASSERT(body->shell_count() == 1, "Expected 1 shell");

    auto shell = body->shells[0];
    TEST_ASSERT(shell->is_closed, "Shell must be closed");
    TEST_ASSERT(shell->face_count() == 6, "Expected 6 faces for negative extruded solid");

    CADMesh mesh = Tessellator::Tessellate(*body);
    TEST_ASSERT(!mesh.vertices.empty(), "Mesh vertices empty");
    TEST_ASSERT(!mesh.indices.empty(), "Mesh indices empty");
    TEST_ASSERT(mesh.indices.size() == 36, "Negative extrude index count != 36");

    // Verify outward normals at Z = 0.0 (+Z outward) and Z = -4.0 (-Z outward)
    bool foundCapAtZero = false;
    bool foundCapAtMinusFour = false;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position[2] - 0.0f) < 0.05f && v.normal[2] > 0.9f) {
            foundCapAtZero = true;
        }
        if (std::abs(v.position[2] - (-4.0f)) < 0.05f && v.normal[2] < -0.9f) {
            foundCapAtMinusFour = true;
        }
    }
    TEST_ASSERT(foundCapAtZero, "Cap at Z=0.0 must have outward normal pointing +Z (+dir)");
    TEST_ASSERT(foundCapAtMinusFour, "Cap at Z=-4.0 must have outward normal pointing -Z (-dir)");

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

bool TestBlendGeometryConvexityAndDihedral() {
    std::cout << "[Test] BlendGeometry Dihedral Angle & Rolling Ball Centers...\n";

    // 1. Right-angle convex edge (e.g. top edge of cube)
    // Face 1: Top (+Y normal), Face 2: Right (+X normal), Edge tangent: +Z
    Vector3D n1(0.0, 1.0, 0.0);
    Vector3D n2(1.0, 0.0, 0.0);
    Vector3D t(0.0, 0.0, 1.0);
    Vector3D inwardHint(-1.0, 0.0, 0.0); // Points into Face 1 (-X)

    DihedralInfo infoConvex = BlendGeometry::ClassifyDihedralEdge(n1, n2, t, inwardHint);
    TEST_ASSERT(infoConvex.is_convex, "Cube outer edge must be classified as CONVEX");
    TEST_ASSERT(std::abs(infoConvex.angle_degrees - 90.0) < 1e-4, "Dihedral angle must be 90 degrees");

    Point3D pE(1.0, 1.0, 0.0);
    double radius = 0.25;
    Point3D centerConvex = BlendGeometry::ComputeRollingBallCenter(
        pE, n1, n2, radius, infoConvex.is_convex, infoConvex.dihedral_angle
    );

    // Convex center must be placed INSIDE the cube at (1.0 - 0.25, 1.0 - 0.25, 0.0) = (0.75, 0.75, 0.0)
    TEST_ASSERT(std::abs(centerConvex.x - 0.75) < 1e-4, "Convex center X must be inside solid (0.75)");
    TEST_ASSERT(std::abs(centerConvex.y - 0.75) < 1e-4, "Convex center Y must be inside solid (0.75)");

    // Contact rails must be at (0.75, 1.0, 0.0) on F1 and (1.0, 0.75, 0.0) on F2
    Point3D r1, r2;
    BlendGeometry::ComputeContactRails(centerConvex, n1, n2, radius, true, r1, r2);
    TEST_ASSERT(std::abs(r1.x - 0.75) < 1e-4 && std::abs(r1.y - 1.0) < 1e-4, "Contact rail 1 must lie on Face 1");
    TEST_ASSERT(std::abs(r2.x - 1.0) < 1e-4 && std::abs(r2.y - 0.75) < 1e-4, "Contact rail 2 must lie on Face 2");

    // 2. Right-angle concave edge (internal step)
    // Floor: y = 0 (+Y normal), Wall: x = 1 (-X normal), Edge at (1, 0, z)
    Vector3D nConcave2(-1.0, 0.0, 0.0);
    Vector3D inwardHintConcave(-1.0, 0.0, 0.0); // Points into floor (x < 1)
    DihedralInfo infoConcave = BlendGeometry::ClassifyDihedralEdge(n1, nConcave2, t, inwardHintConcave);
    TEST_ASSERT(!infoConcave.is_convex, "Internal step edge must be classified as CONCAVE");
    TEST_ASSERT(std::abs(infoConcave.angle_degrees - 270.0) < 1e-4, "Dihedral angle must be 270 degrees");

    Point3D pEConcave(1.0, 0.0, 0.0);
    Point3D centerConcave = BlendGeometry::ComputeRollingBallCenter(
        pEConcave, n1, nConcave2, radius, infoConcave.is_convex, infoConcave.dihedral_angle
    );
    // Concave center must be placed OUTSIDE the solid (in the void/cavity at x < 1, y > 0)
    TEST_ASSERT(std::abs(centerConcave.x - 0.75) < 1e-4, "Concave center X must be in cavity (0.75)");
    TEST_ASSERT(std::abs(centerConcave.y - 0.25) < 1e-4, "Concave center Y must be in cavity (0.25)");

    // 3. Oblique dihedral angle (e.g. 120 degrees obtuse / 60 degrees acute)
    Vector3D nObtuse1(0.0, 1.0, 0.0);
    Vector3D nObtuse2(std::sin(60.0 * 3.1415926535 / 180.0), std::cos(60.0 * 3.1415926535 / 180.0), 0.0);
    Vector3D inwardHintObtuse(-1.0, 0.0, 0.0);
    DihedralInfo infoObtuse = BlendGeometry::ClassifyDihedralEdge(nObtuse1, nObtuse2, t, inwardHintObtuse);
    TEST_ASSERT(infoObtuse.is_convex, "Obtuse roof ridge must be CONVEX");
    TEST_ASSERT(std::abs(infoObtuse.angle_degrees - 120.0) < 0.5, "Obtuse dihedral angle must be ~120 degrees");

    return true;
}

bool TestCornerSphereCenterIntersection() {
    std::cout << "[Test] Corner Sphere Center Intersection (3-Edge Converging Vertex)...\n";

    Point3D corner(2.0, 3.0, 4.0);
    Vector3D n1(1.0, 0.0, 0.0);
    Vector3D n2(0.0, 1.0, 0.0);
    Vector3D n3(0.0, 0.0, 1.0);
    double radius = 0.3;

    // A. Convex corner: sphere center is inside solid at corner - (R, R, R)
    auto optCenterConvex = BlendGeometry::ComputeCornerSphereCenter(corner, n1, n2, n3, radius, true);
    TEST_ASSERT(optCenterConvex.has_value(), "Must find valid corner sphere center");
    Point3D cConvex = *optCenterConvex;
    TEST_ASSERT(std::abs(cConvex.x - (2.0 - 0.3)) < 1e-4, "Corner sphere center X must be 1.7");
    TEST_ASSERT(std::abs(cConvex.y - (3.0 - 0.3)) < 1e-4, "Corner sphere center Y must be 2.7");
    TEST_ASSERT(std::abs(cConvex.z - (4.0 - 0.3)) < 1e-4, "Corner sphere center Z must be 3.7");

    // B. Concave corner: sphere center is in cavity at corner + (R, R, R)
    auto optCenterConcave = BlendGeometry::ComputeCornerSphereCenter(corner, n1, n2, n3, radius, false);
    TEST_ASSERT(optCenterConcave.has_value(), "Must find concave corner sphere center");
    Point3D cConcave = *optCenterConcave;
    TEST_ASSERT(std::abs(cConcave.x - (2.0 + 0.3)) < 1e-4, "Concave sphere center X must be 2.3");
    TEST_ASSERT(std::abs(cConcave.y - (3.0 + 0.3)) < 1e-4, "Concave sphere center Y must be 3.3");
    TEST_ASSERT(std::abs(cConcave.z - (4.0 + 0.3)) < 1e-4, "Concave sphere center Z must be 4.3");

    return true;
}

bool TestSphericalCornerPatchGeneration() {
    std::cout << "[Test] Spherical Triangle Corner Patch Generation...\n";

    Point3D cornerCenter(1.0, 1.0, 1.0);
    double radius = 0.5;
    Point3D p1 = cornerCenter + Vector3D(radius, 0.0, 0.0);
    Point3D p2 = cornerCenter + Vector3D(0.0, radius, 0.0);
    Point3D p3 = cornerCenter + Vector3D(0.0, 0.0, radius);

    auto patch = CornerBlend::CreateSphericalCornerPatch(10, cornerCenter, radius, p1, p2, p3, true, 2);
    TEST_ASSERT(patch != nullptr, "Patch must not be null");
    TEST_ASSERT(!patch->mesh_vertices.empty(), "Patch must have vertices");
    TEST_ASSERT(!patch->mesh_indices.empty(), "Patch must have indices");

    // Verify all patch vertices lie strictly on the sphere of radius R
    for (const auto& v : patch->mesh_vertices) {
        Point3D pt(v.position[0], v.position[1], v.position[2]);
        double dist = pt.distance_to(cornerCenter);
        TEST_ASSERT(std::abs(dist - radius) < 1e-3, "Patch vertex must lie exactly on spherical shell");
    }

    return true;
}

bool TestFilletOpExecution() {
    std::cout << "[Test] FilletOp B-Rep Execution and Adjacent Face Retrimming...\n";

    // 1. Create a 4x2x3 Box Body
    Sketch2D sketch(Plane::XY());
    sketch.add_rectangle(4.0, 2.0);
    auto profiles = sketch.extract_profiles();
    TEST_ASSERT(!profiles.empty(), "Profile required");

    ExtrudeOp::Params params;
    params.start_depth = 0.0;
    params.end_depth = 3.0;
    params.direction = Vector3D(0.0, 0.0, 1.0);
    params.is_solid = true;

    auto body = ExtrudeOp::Execute(profiles[0], sketch.plane, params);
    TEST_ASSERT(body && body->is_solid(), "Box body must be solid");
    TEST_ASSERT(body->shells[0]->faces.size() == 6, "Box must have 6 faces");

    // 2. Select an edge of the body to fillet
    std::vector<FilletEdgeSpec> specs;
    specs.push_back({1, 0.35}); // First edge with radius 0.35

    Body filletedBody = FilletOp::apply(*body, specs);
    TEST_ASSERT(!filletedBody.shells.empty(), "Filleted body must have shells");

    auto shell = filletedBody.shells[0];
    // Original 6 faces + 1 fillet blend face = 7 faces
    TEST_ASSERT(shell->faces.size() == 7, "Shell must have 7 faces after filleting 1 edge");

    // 3. Tessellate to verify watertightness and face retrimming (no thin fins!)
    CADMesh mesh = Tessellator::Tessellate(filletedBody);
    TEST_ASSERT(!mesh.vertices.empty(), "Tessellated mesh vertices must not be empty");
    TEST_ASSERT(!mesh.indices.empty(), "Tessellated mesh indices must not be empty");

    for (const auto& v : mesh.vertices) {
        TEST_ASSERT(std::isfinite(v.position[0]) && std::isfinite(v.position[1]) && std::isfinite(v.position[2]),
                    "All vertices must be finite");
    }

    return true;
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "     TesseraCAD Modeling Operators Unit Tests            \n";
    std::cout << "=========================================================\n";

    bool passed = true;
    passed &= TestSolidExtrude();
    passed &= TestNegativeExtrude();
    passed &= TestSurfaceExtrude();
    passed &= TestHollowExtrude();
    passed &= TestFullRevolve();
    passed &= TestPartialRevolve();
    passed &= TestRevolveCylindricalThicknessFromSketch();
    passed &= TestBlendGeometryConvexityAndDihedral();
    passed &= TestCornerSphereCenterIntersection();
    passed &= TestSphericalCornerPatchGeneration();
    passed &= TestFilletOpExecution();

    std::cout << "=========================================================\n";
    if (passed) {
        std::cout << "[SUCCESS] ALL MODELING OPERATOR TESTS PASSED!\n";
        return 0;
    } else {
        std::cerr << "[FAILURE] SOME TESTS FAILED!\n";
        return 1;
    }
}
