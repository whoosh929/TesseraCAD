#include "engine/geom/Transform3D.hpp"
#include "engine/geom/Axis3D.hpp"
#include "engine/geom/Plane.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

using namespace Tessera::Geom;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Line " << __LINE__ << ": " << msg << "\n"; \
            return false; \
        } \
    } while(false)

bool TestVectorAndPointArithmetic() {
    std::cout << "[Test] Vector3D and Point3D Math...\n";

    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(4.0, -1.0, 2.0);

    // Dot product: 1*4 + 2*(-1) + 3*2 = 4 - 2 + 6 = 8
    TEST_ASSERT(std::abs(v1.dot(v2) - 8.0) < CAD_EPSILON, "Vector3D dot product incorrect");

    // Cross product: (2*2 - 3*(-1), 3*4 - 1*2, 1*(-1) - 2*4) = (7, 10, -9)
    Vector3D c = v1.cross(v2);
    TEST_ASSERT(c.approx_equal(Vector3D(7.0, 10.0, -9.0)), "Vector3D cross product incorrect");

    // Length and normalization
    Vector3D v3(0.0, 3.0, 4.0);
    TEST_ASSERT(std::abs(v3.length() - 5.0) < CAD_EPSILON, "Vector3D length incorrect");
    Vector3D v3n = v3.normalized();
    TEST_ASSERT(std::abs(v3n.length() - 1.0) < CAD_EPSILON, "Normalized vector length != 1");
    TEST_ASSERT(v3n.approx_equal(Vector3D(0.0, 0.6, 0.8)), "Normalized vector coordinates incorrect");

    // Point and Vector operations
    Point3D p1(10.0, 20.0, 30.0);
    Point3D p2 = p1 + v1;
    TEST_ASSERT(p2.approx_equal(Point3D(11.0, 22.0, 33.0)), "Point3D + Vector3D incorrect");

    Vector3D diff = p2 - p1;
    TEST_ASSERT(diff.approx_equal(v1), "Point3D - Point3D incorrect");

    TEST_ASSERT(std::abs(p1.distance_to(p2) - v1.length()) < CAD_EPSILON, "Point3D distance_to incorrect");

    return true;
}

bool TestAxis3D() {
    std::cout << "[Test] Axis3D Operations...\n";

    // Create axis along +Z through (2, 3, 0)
    Axis3D axis(Point3D(2.0, 3.0, 0.0), Vector3D(0.0, 0.0, 5.0)); // Should normalize direction
    TEST_ASSERT(std::abs(axis.direction.length() - 1.0) < CAD_EPSILON, "Axis3D direction not normalized");
    TEST_ASSERT(axis.direction.approx_equal(Vector3D(0.0, 0.0, 1.0)), "Axis3D direction mismatch");

    // Point evaluation
    Point3D pAt = axis.point_at(10.5);
    TEST_ASSERT(pAt.approx_equal(Point3D(2.0, 3.0, 10.5)), "Axis3D point_at incorrect");

    // Projection
    Point3D query(6.0, 3.0, 8.0);
    double t = axis.project(query);
    TEST_ASSERT(std::abs(t - 8.0) < CAD_EPSILON, "Axis3D project parameter incorrect");

    Point3D projPt = axis.project_point(query);
    TEST_ASSERT(projPt.approx_equal(Point3D(2.0, 3.0, 8.0)), "Axis3D project_point incorrect");

    // Distance: query (6, 3, 8) to axis (2, 3, z) should be |6 - 2| = 4.0
    double dist = axis.distance_to(query);
    TEST_ASSERT(std::abs(dist - 4.0) < CAD_EPSILON, "Axis3D distance_to incorrect");

    // Standard axes
    Axis3D x = Axis3D::X();
    Axis3D y = Axis3D::Y();
    Axis3D z = Axis3D::Z();
    TEST_ASSERT(x.direction.approx_equal(Vector3D(1.0, 0.0, 0.0)), "Axis3D::X incorrect");
    TEST_ASSERT(y.direction.approx_equal(Vector3D(0.0, 1.0, 0.0)), "Axis3D::Y incorrect");
    TEST_ASSERT(z.direction.approx_equal(Vector3D(0.0, 0.0, 1.0)), "Axis3D::Z incorrect");

    return true;
}

bool TestStandardDatumPlanes() {
    std::cout << "[Test] Standard Datum Planes Orthonormality...\n";

    std::vector<std::pair<std::string, Plane>> planes = {
        {"XY Plane", Plane::XY(Point3D(1.0, 2.0, 3.0))},
        {"XZ Plane", Plane::XZ(Point3D(4.0, 5.0, 6.0))},
        {"YZ Plane", Plane::YZ(Point3D(7.0, 8.0, 9.0))}
    };

    for (const auto& [name, pl] : planes) {
        // Unit lengths
        TEST_ASSERT(std::abs(pl.normal.length() - 1.0) < CAD_EPSILON, name + " normal not unit length");
        TEST_ASSERT(std::abs(pl.u_axis.length() - 1.0) < CAD_EPSILON, name + " u_axis not unit length");
        TEST_ASSERT(std::abs(pl.v_axis.length() - 1.0) < CAD_EPSILON, name + " v_axis not unit length");

        // Mutual orthogonality
        TEST_ASSERT(std::abs(pl.u_axis.dot(pl.v_axis)) < CAD_EPSILON, name + " U not orthogonal to V");
        TEST_ASSERT(std::abs(pl.u_axis.dot(pl.normal)) < CAD_EPSILON, name + " U not orthogonal to Normal");
        TEST_ASSERT(std::abs(pl.v_axis.dot(pl.normal)) < CAD_EPSILON, name + " V not orthogonal to Normal");

        // Right-handed orientation: U x V == Normal
        Vector3D cross = pl.u_axis.cross(pl.v_axis);
        TEST_ASSERT(cross.approx_equal(pl.normal), name + " basis is not right-handed (U x V != Normal)");
    }

    return true;
}

bool TestArbitraryAndThreePointPlanes() {
    std::cout << "[Test] Arbitrary & Three-Point Plane Construction...\n";

    // Test automatic basis generation with diverse normals
    std::vector<Vector3D> normals = {
        {1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, -1.0},
        {1.0, 1.0, 1.0},
        {-3.5, 2.1, 7.8},
        {0.001, -0.999, 0.02}
    };

    for (const auto& rawN : normals) {
        Plane pl(Point3D(2.0, -1.0, 4.0), rawN);
        TEST_ASSERT(std::abs(pl.normal.length() - 1.0) < CAD_EPSILON, "Plane normal not normalized");
        TEST_ASSERT(std::abs(pl.u_axis.length() - 1.0) < CAD_EPSILON, "Plane u_axis not unit length");
        TEST_ASSERT(std::abs(pl.v_axis.length() - 1.0) < CAD_EPSILON, "Plane v_axis not unit length");

        TEST_ASSERT(std::abs(pl.u_axis.dot(pl.v_axis)) < CAD_EPSILON, "Plane U not orthogonal to V");
        TEST_ASSERT(std::abs(pl.u_axis.dot(pl.normal)) < CAD_EPSILON, "Plane U not orthogonal to N");
        TEST_ASSERT(std::abs(pl.v_axis.dot(pl.normal)) < CAD_EPSILON, "Plane V not orthogonal to N");

        Vector3D cross = pl.u_axis.cross(pl.v_axis);
        TEST_ASSERT(cross.approx_equal(pl.normal), "Plane basis is not right-handed");
    }

    // Three-point construction
    Point3D p0(1.0, 2.0, 3.0);
    Point3D p1(5.0, 2.0, 3.0); // +X from p0
    Point3D p2(1.0, 8.0, 3.0); // +Y from p0
    Plane pl3(p0, p1, p2);

    TEST_ASSERT(pl3.contains(p0), "3-Point plane does not contain p0");
    TEST_ASSERT(pl3.contains(p1), "3-Point plane does not contain p1");
    TEST_ASSERT(pl3.contains(p2), "3-Point plane does not contain p2");
    TEST_ASSERT(pl3.normal.approx_equal(Vector3D(0.0, 0.0, 1.0)), "3-Point plane normal incorrect");

    return true;
}

bool TestCoordinateRoundTrips() {
    std::cout << "[Test] to_world and to_local Round-Trip Invariants...\n";

    std::vector<Plane> testPlanes = {
        Plane::XY(Point3D(0.0, 0.0, 0.0)),
        Plane::XY(Point3D(12.5, -7.2, 33.1)),
        Plane::XZ(Point3D(-5.0, 10.0, 2.0)),
        Plane::YZ(Point3D(1.0, 2.0, 3.0)),
        Plane(Point3D(3.0, 4.0, 5.0), Vector3D(1.0, 2.0, 3.0)),
        Plane(Point3D(-10.0, 50.0, -25.0), Vector3D(-4.0, 7.0, -1.0))
    };

    // 1. 2D -> 3D -> 2D round trip: to_local(to_world(uv)) == uv
    for (const auto& pl : testPlanes) {
        for (double u = -25.0; u <= 25.0; u += 5.0) {
            for (double v = -25.0; v <= 25.0; v += 5.0) {
                Point2D origUV(u, v);
                Point3D worldPt = pl.to_world(origUV);

                // The point must lie on the plane
                TEST_ASSERT(pl.contains(worldPt), "to_world point does not lie on plane");
                TEST_ASSERT(std::abs(pl.distance_to(worldPt)) < CAD_EPSILON, "to_world distance_to != 0");

                Point2D backUV = pl.to_local(worldPt);
                TEST_ASSERT(origUV.approx_equal(backUV, 1e-9), "to_local(to_world(uv)) != uv");
            }
        }
    }

    // 2. Vector round trip: to_local_vector(to_world_vector(duv)) == duv
    for (const auto& pl : testPlanes) {
        for (double du = -10.0; du <= 10.0; du += 4.0) {
            for (double dv = -10.0; dv <= 10.0; dv += 4.0) {
                Vector2D origVec(du, dv);
                Vector3D worldVec = pl.to_world_vector(origVec);
                Vector2D backVec = pl.to_local_vector(worldVec);
                TEST_ASSERT(origVec.approx_equal(backVec, 1e-9), "to_local_vector round trip failed");
            }
        }
    }

    // 3. 3D point projection round trip: to_world(to_local(P)) == project(P)
    for (const auto& pl : testPlanes) {
        for (double x = -15.0; x <= 15.0; x += 7.5) {
            for (double y = -15.0; y <= 15.0; y += 7.5) {
                for (double z = -15.0; z <= 15.0; z += 7.5) {
                    Point3D query(x, y, z);
                    Point3D proj = pl.project(query);

                    TEST_ASSERT(pl.contains(proj), "projected point not on plane");

                    // Distance from projected point to query must equal signed distance
                    double signedDist = pl.distance_to(query);
                    Point3D reconstructed = proj + pl.normal * signedDist;
                    TEST_ASSERT(query.approx_equal(reconstructed, 1e-9), "P != proj + normal * distance");
                }
            }
        }
    }

    return true;
}

bool TestOffsetAndTransformOperations() {
    std::cout << "[Test] Plane Offset and Transform Operations...\n";

    Plane basePl = Plane::XY(Point3D(0.0, 0.0, 5.0));

    // Offset +10.0 along +Z
    Plane offsetPl = basePl.offset(10.0);
    TEST_ASSERT(offsetPl.origin.approx_equal(Point3D(0.0, 0.0, 15.0)), "Offset plane origin incorrect");
    TEST_ASSERT(offsetPl.normal.approx_equal(basePl.normal), "Offset plane normal changed");

    // Distance between planes
    TEST_ASSERT(std::abs(basePl.distance_to(offsetPl.origin) - 10.0) < CAD_EPSILON, "Offset plane distance incorrect");

    // Flipped plane
    Plane flippedPl = basePl.flipped();
    TEST_ASSERT(flippedPl.normal.approx_equal(Vector3D(0.0, 0.0, -1.0)), "Flipped plane normal incorrect");
    TEST_ASSERT(flippedPl.u_axis.cross(flippedPl.v_axis).approx_equal(flippedPl.normal), "Flipped plane not right-handed");

    // Affine Transformation: Rotate 90 deg around X and translate by (10, 20, 30)
    Transform3D xform = Transform3D::Translation(Vector3D(10.0, 20.0, 30.0)) * Transform3D::RotationX(1.5707963267948966);
    Plane xformedPl = basePl.transformed(xform);

    TEST_ASSERT(std::abs(xformedPl.normal.length() - 1.0) < CAD_EPSILON, "Transformed normal not unit");
    TEST_ASSERT(std::abs(xformedPl.u_axis.dot(xformedPl.v_axis)) < CAD_EPSILON, "Transformed plane not orthogonal");
    TEST_ASSERT(xformedPl.u_axis.cross(xformedPl.v_axis).approx_equal(xformedPl.normal), "Transformed plane not right-handed");

    return true;
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "       TesseraCAD Geometric Kernel Unit Tests            \n";
    std::cout << "=========================================================\n";

    bool passed = true;
    passed &= TestVectorAndPointArithmetic();
    passed &= TestAxis3D();
    passed &= TestStandardDatumPlanes();
    passed &= TestArbitraryAndThreePointPlanes();
    passed &= TestCoordinateRoundTrips();
    passed &= TestOffsetAndTransformOperations();

    std::cout << "=========================================================\n";
    if (passed) {
        std::cout << "[SUCCESS] ALL GEOMETRIC TESTS PASSED!\n";
        return 0;
    } else {
        std::cerr << "[FAILURE] SOME TESTS FAILED!\n";
        return 1;
    }
}
