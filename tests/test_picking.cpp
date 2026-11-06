#include "engine/render/Camera.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/BoxNode.hpp"
#include "engine/graph/SketchNode.hpp"
#include <iostream>
#include <memory>
#include <cmath>

using namespace Tessera;
using namespace Tessera::Geom;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Line " << __LINE__ << ": " << msg << "\n"; \
            return false; \
        } \
    } while(false)

bool TestRayIntersectsTriangle() {
    std::cout << "[Test] Möller-Trumbore Ray-Triangle Intersection...\n";

    Point3D v0(0.0, 0.0, 0.0);
    Point3D v1(2.0, 0.0, 0.0);
    Point3D v2(0.0, 2.0, 0.0);

    // 1. Direct hit on triangle front
    Point3D rayOrigin(0.5, 0.5, 5.0);
    Vector3D rayDir(0.0, 0.0, -1.0);
    double t = 0.0;
    bool hit = RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, t);
    TEST_ASSERT(hit, "Ray should intersect triangle");
    TEST_ASSERT(std::abs(t - 5.0) < 1e-6, "Hit distance t should be 5.0");

    // 2. Direct hit from back (two-sided)
    Point3D rayOriginBack(0.5, 0.5, -3.0);
    Vector3D rayDirBack(0.0, 0.0, 1.0);
    hit = RayIntersectsTriangle(rayOriginBack, rayDirBack, v0, v1, v2, t);
    TEST_ASSERT(hit, "Ray should intersect triangle from behind (two-sided)");
    TEST_ASSERT(std::abs(t - 3.0) < 1e-6, "Hit distance t should be 3.0");

    // 3. Ray misses outside hypotenuse
    Point3D rayOriginMiss(1.5, 1.5, 5.0);
    hit = RayIntersectsTriangle(rayOriginMiss, rayDir, v0, v1, v2, t);
    TEST_ASSERT(!hit, "Ray outside triangle bounds should not intersect");

    // 4. Ray is parallel to triangle plane
    Vector3D rayDirParallel(1.0, 0.0, 0.0);
    hit = RayIntersectsTriangle(Point3D(0.5, 0.5, 0.0), rayDirParallel, v0, v1, v2, t);
    TEST_ASSERT(!hit, "Parallel ray should not intersect");

    // 5. Triangle is behind ray origin
    Point3D rayOriginAway(0.5, 0.5, 5.0);
    Vector3D rayDirAway(0.0, 0.0, 1.0); // pointing away from Z=0
    hit = RayIntersectsTriangle(rayOriginAway, rayDirAway, v0, v1, v2, t);
    TEST_ASSERT(!hit, "Ray pointing away from triangle should not intersect");

    return true;
}

bool TestCameraScreenPointToRay() {
    std::cout << "[Test] Camera::ScreenPointToRay Unprojection...\n";

    Camera camera;
    float vpW = 1600.0f;
    float vpH = 900.0f;

    // Viewport center
    Ray centerRay = camera.ScreenPointToRay(800.0f, 450.0f, vpW, vpH);

    // Direction must be normalized
    TEST_ASSERT(std::abs(centerRay.direction.length() - 1.0) < 1e-5, "Ray direction must be normalized");

    // Origin should be eye position
    const auto& uniforms = camera.GetUniforms();
    Point3D expectedEye(uniforms.eyePosition[0], uniforms.eyePosition[1], uniforms.eyePosition[2]);
    TEST_ASSERT(centerRay.origin.approx_equal(expectedEye, 1e-4), "Ray origin should match camera eye");

    // Center ray should point directly toward camera target (0, 0, 0)
    Point3D target(0.0, 0.0, 0.0);
    Vector3D toTarget = (target - expectedEye).normalized();
    TEST_ASSERT(centerRay.direction.approx_equal(toTarget, 1e-3), "Center ray should point at camera target");

    return true;
}

bool TestParametricGraphPickNode() {
    std::cout << "[Test] ParametricGraph::PickNode...\n";

    ParametricGraph graph(nullptr);

    // Create a mock box node
    auto box1 = std::make_shared<BoxNode>("Box_Front", 1.0f, 1.0f, 1.0f);
    box1->SetPosition(0.0f, 0.0f, 0.0f);
    AtomicMeshExchange exchange;
    box1->Evaluate(nullptr, exchange);
    graph.AddNode(box1);

    auto box2 = std::make_shared<BoxNode>("Box_Right", 1.0f, 1.0f, 1.0f);
    box2->SetPosition(5.0f, 0.0f, 0.0f);
    box2->Evaluate(nullptr, exchange);
    graph.AddNode(box2);

    // 1. Ray targeting Box 1 at (0, 0, 0)
    Point3D rayOrig1(0.0, 0.0, 10.0);
    Vector3D rayDir1(0.0, 0.0, -1.0);
    double dist = 0.0;
    uint64_t pickedId1 = graph.PickNode(rayOrig1, rayDir1, &dist);
    TEST_ASSERT(pickedId1 == box1->GetId(), "PickNode should identify Box 1");
    TEST_ASSERT(dist > 0.0 && dist < 10.0, "Intersection distance must be valid");

    // 2. Ray targeting Box 2 at (5, 0, 0)
    Point3D rayOrig2(5.0, 0.0, 10.0);
    Vector3D rayDir2(0.0, 0.0, -1.0);
    uint64_t pickedId2 = graph.PickNode(rayOrig2, rayDir2, &dist);
    TEST_ASSERT(pickedId2 == box2->GetId(), "PickNode should identify Box 2");

    // 3. Ray through empty space (e.g. at (10, 10, 10))
    Point3D rayOrigEmpty(10.0, 10.0, 10.0);
    Vector3D rayDirEmpty(0.0, 0.0, -1.0);
    uint64_t pickedEmpty = graph.PickNode(rayOrigEmpty, rayDirEmpty, &dist);
    TEST_ASSERT(pickedEmpty == 0, "PickNode on empty space should return 0 (no hit)");

    // 4. Occlusion: Add Box 3 behind Box 1 at (0, 0, -5)
    auto box3 = std::make_shared<BoxNode>("Box_Behind", 1.0f, 1.0f, 1.0f);
    box3->SetPosition(0.0f, 0.0f, -5.0f);
    box3->Evaluate(nullptr, exchange);
    graph.AddNode(box3);

    // Ray from (0, 0, 10) pointing -Z hits both Box 1 and Box 3. Closest (Box 1) should be selected.
    uint64_t pickedClosest = graph.PickNode(rayOrig1, rayDir1, &dist);
    TEST_ASSERT(pickedClosest == box1->GetId(), "PickNode must return closest occluding node");

    return true;
}

int main() {
    std::cout << "========================================\n";
    std::cout << " Running Tessera 3D Viewport Picking Tests\n";
    std::cout << "========================================\n\n";

    bool pass = true;
    pass &= TestRayIntersectsTriangle();
    pass &= TestCameraScreenPointToRay();
    pass &= TestParametricGraphPickNode();

    if (pass) {
        std::cout << "\n[PASS] All 3D Viewport Picking test suites succeeded!\n";
        return 0;
    } else {
        std::cerr << "\n[FAIL] Viewport Picking tests failed!\n";
        return 1;
    }
}
