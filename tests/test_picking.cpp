#include "engine/render/Camera.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/BoxNode.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/RevolveNode.hpp"
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

bool TestFeatureSketchChildAndPromotion() {
    std::cout << "[Test] Feature Child Sketch Hierarchy & Promotion on Deletion...\n";

    ParametricGraph graph(nullptr);

    // 1. Create a parentless sketch
    auto sketch = std::make_shared<SketchNode>("Sketch_Rect", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(3.0f, 2.0f);
    graph.AddNode(sketch);

    TEST_ASSERT(!sketch->HasParentFeature(), "New sketch must be parentless");
    TEST_ASSERT(!sketch->IsConsumed(), "New sketch must not be consumed");
    TEST_ASSERT(sketch->IsVisible(), "Parentless sketch must be visible");

    // 2. Create an Extrude feature referencing the sketch
    auto extrude = std::make_shared<ExtrudeNode>("Extrude_Feature", sketch, 2.5f);
    graph.AddNode(extrude);

    // Verify sketch became child underneath the feature
    TEST_ASSERT(sketch->HasParentFeature(), "Sketch must now have a parent feature");
    TEST_ASSERT(sketch->GetParentFeature() == extrude, "Parent feature must be the extrude feature");
    TEST_ASSERT(extrude->GetChildSketch() == sketch, "Extrude must report sketch as its child");
    TEST_ASSERT(sketch->IsConsumed(), "Sketch must be marked consumed");
    TEST_ASSERT(!sketch->IsVisible(), "Consumed sketch should have visibility hidden under solid feature");

    // 3. Verify only unconsumed sketches are available for association
    size_t availableCount = 0;
    for (const auto& n : graph.GetNodes()) {
        if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
            if (!s->HasParentFeature() && !s->IsConsumed()) {
                availableCount++;
            }
        }
    }
    TEST_ASSERT(availableCount == 0, "Consumed sketch must not be available for association with another feature");

    // 4. Delete the Extrude feature
    uint64_t extrudeId = extrude->GetId();
    uint64_t sketchId = sketch->GetId();
    graph.RemoveNode(extrudeId);

    // Verify extrude is gone
    bool extrudeFound = false;
    bool sketchFound = false;
    for (const auto& n : graph.GetNodes()) {
        if (n->GetId() == extrudeId) extrudeFound = true;
        if (n->GetId() == sketchId) sketchFound = true;
    }
    TEST_ASSERT(!extrudeFound, "Extrude feature must be removed from graph");
    TEST_ASSERT(sketchFound, "Sketch must remain in graph when feature is deleted");

    // Verify sketch is promoted to a parentless sketch
    TEST_ASSERT(!sketch->HasParentFeature(), "Sketch must be promoted to parentless");
    TEST_ASSERT(sketch->GetParentFeature() == nullptr, "Sketch parent feature must be null");
    TEST_ASSERT(!sketch->IsConsumed(), "Promoted sketch must not be consumed");
    TEST_ASSERT(sketch->IsVisible(), "Promoted sketch visibility must be restored");
    TEST_ASSERT(sketch->IsDirty(), "Promoted sketch must be marked dirty for re-tessellation");

    // 5. Verify the promoted sketch can now be associated with a new feature (e.g. Revolve)
    auto revolve = std::make_shared<RevolveNode>("Revolve_Feature", sketch, 360.0f);
    graph.AddNode(revolve);

    TEST_ASSERT(sketch->HasParentFeature(), "Sketch should now be child of the new revolve feature");
    TEST_ASSERT(sketch->GetParentFeature() == revolve, "Parent feature must be revolve");
    TEST_ASSERT(revolve->GetChildSketch() == sketch, "Revolve must report sketch as child");

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
    pass &= TestFeatureSketchChildAndPromotion();

    if (pass) {
        std::cout << "\n[PASS] All 3D Viewport Picking test suites succeeded!\n";
        return 0;
    } else {
        std::cerr << "\n[FAIL] Viewport Picking tests failed!\n";
        return 1;
    }
}

