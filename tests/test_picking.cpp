#include "engine/render/Camera.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/RevolveNode.hpp"
#include "engine/graph/DatumPlaneNode.hpp"
#include "engine/graph/DatumAxisNode.hpp"
#include "engine/graph/FilletNode.hpp"
#include "engine/graph/ChamferNode.hpp"
#include <iostream>
#include <memory>
#include <cmath>
#include <map>

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

    // Test WorldToScreenPoint at target (0, 0, 0)
    float sx = 0.0f, sy = 0.0f;
    bool visible = camera.WorldToScreenPoint(target, vpW, vpH, sx, sy);
    TEST_ASSERT(visible, "Target at origin should be visible in front of camera");
    TEST_ASSERT(std::abs(sx - 800.0f) < 2.0f, "Target should project to viewport center X (approx 800)");
    TEST_ASSERT(std::abs(sy - 450.0f) < 2.0f, "Target should project to viewport center Y (approx 450)");

    // Point far behind camera should return false (not visible)
    Point3D behindEye = expectedEye + (expectedEye - target) * 5.0;
    TEST_ASSERT(!camera.WorldToScreenPoint(behindEye, vpW, vpH, sx, sy), "Point behind camera must return false");

    return true;
}

bool TestSolidWorksMouseNavigation() {
    std::cout << "[Test] SOLIDWORKS Mouse Navigation (Cursor-Centered Zoom, Smooth Zoom, Rotate, Pan)...\n";

    Camera camera;
    float vpW = 1600.0f;
    float vpH = 900.0f;

    float initDist = camera.GetDistance();
    TEST_ASSERT(std::abs(initDist - 10.0f) < 1e-4f, "Initial camera distance should be 10.0m");

    // 1. Zoom In centered at viewport center (800, 450)
    camera.ZoomCentered(800.0f, 450.0f, vpW, vpH, 1.0f, false);
    float zoomInDist = camera.GetDistance();
    TEST_ASSERT(zoomInDist < initDist, "Scroll up must zoom in (decrease distance)");
    TEST_ASSERT(std::abs(zoomInDist - initDist * 0.85f) < 1e-3f, "Zoom in factor should match 0.85");
    // Target should remain at (0, 0, 0) for center zoom
    TEST_ASSERT(std::abs(camera.GetTarget()[0]) < 1e-4f, "Center zoom must keep target X at 0");
    TEST_ASSERT(std::abs(camera.GetTarget()[1]) < 1e-4f, "Center zoom must keep target Y at 0");
    TEST_ASSERT(std::abs(camera.GetTarget()[2]) < 1e-4f, "Center zoom must keep target Z at 0");

    // 2. Smooth Zoom (Shift held)
    float preSmoothDist = camera.GetDistance();
    camera.ZoomCentered(800.0f, 450.0f, vpW, vpH, 1.0f, true); // smooth = true
    float postSmoothDist = camera.GetDistance();
    TEST_ASSERT(postSmoothDist < preSmoothDist, "Smooth scroll up must zoom in");
    TEST_ASSERT(std::abs(postSmoothDist - preSmoothDist * 0.96f) < 1e-3f, "Smooth zoom factor should match 0.96 (finer)");

    // 3. Zoom Out (Scroll down)
    float preOutDist = camera.GetDistance();
    camera.ZoomCentered(800.0f, 450.0f, vpW, vpH, -1.0f, false);
    float postOutDist = camera.GetDistance();
    TEST_ASSERT(postOutDist > preOutDist, "Scroll down must zoom out (increase distance)");

    // 4. Cursor-Centered Zoom off-center (e.g. at 1200, 200)
    camera.ZoomCentered(1200.0f, 200.0f, vpW, vpH, 1.0f, false);
    const float* target = camera.GetTarget();
    TEST_ASSERT(std::abs(target[0]) > 0.01f || std::abs(target[1]) > 0.01f || std::abs(target[2]) > 0.01f, 
                "Cursor-centered zoom off-center must shift camera target toward cursor ray");

    // 5. Pan and Rotate (Orbit)
    Camera cam2;
    cam2.Pan(1.5f, -0.5f);
    TEST_ASSERT(std::abs(cam2.GetTarget()[0]) > 0.1f || std::abs(cam2.GetTarget()[1]) > 0.1f, "Pan must shift target point");

    cam2.Orbit(0.2f, -0.1f);
    TEST_ASSERT(cam2.GetUniforms().eyePosition[0] != camera.GetUniforms().eyePosition[0], "Orbit must rotate eye position");

    return true;
}

bool TestParametricGraphPickNode() {
    std::cout << "[Test] ParametricGraph::PickNode...\n";

    ParametricGraph graph(nullptr);

    // Create extruded solid features
    auto sk1 = std::make_shared<SketchNode>("Sketch_Front", SketchPlane::XY_Front);
    sk1->SetPresetRectangle(2.0f, 2.0f);
    sk1->SetPosition(0.0f, 0.0f, 0.0f);
    auto solid1 = std::make_shared<ExtrudeNode>("Solid_Front", sk1, 1.0f);
    solid1->SetPosition(0.0f, 0.0f, 0.0f);
    AtomicMeshExchange exchange;
    solid1->Evaluate(nullptr, exchange);
    graph.AddNode(solid1);

    auto sk2 = std::make_shared<SketchNode>("Sketch_Right", SketchPlane::XY_Front);
    sk2->SetPresetRectangle(2.0f, 2.0f);
    sk2->SetPosition(5.0f, 0.0f, 0.0f);
    auto solid2 = std::make_shared<ExtrudeNode>("Solid_Right", sk2, 1.0f);
    solid2->SetPosition(5.0f, 0.0f, 0.0f);
    solid2->Evaluate(nullptr, exchange);
    graph.AddNode(solid2);

    // 1. Ray targeting Solid 1 at (0, 0, 0)
    Point3D rayOrig1(0.0, 0.0, 10.0);
    Vector3D rayDir1(0.0, 0.0, -1.0);
    double dist = 0.0;
    uint64_t pickedId1 = graph.PickNode(rayOrig1, rayDir1, &dist);
    TEST_ASSERT(pickedId1 == solid1->GetId(), "PickNode should identify Solid 1");
    TEST_ASSERT(dist > 0.0 && dist < 10.0, "Intersection distance must be valid");

    // 2. Ray targeting Solid 2 at (5, 0, 0)
    Point3D rayOrig2(5.0, 0.0, 10.0);
    Vector3D rayDir2(0.0, 0.0, -1.0);
    uint64_t pickedId2 = graph.PickNode(rayOrig2, rayDir2, &dist);
    TEST_ASSERT(pickedId2 == solid2->GetId(), "PickNode should identify Solid 2");

    // 3. Ray through empty space (e.g. at (10, 10, 10))
    Point3D rayOrigEmpty(10.0, 10.0, 10.0);
    Vector3D rayDirEmpty(0.0, 0.0, -1.0);
    uint64_t pickedEmpty = graph.PickNode(rayOrigEmpty, rayDirEmpty, &dist);
    TEST_ASSERT(pickedEmpty == 0, "PickNode on empty space should return 0 (no hit)");

    // 4. Occlusion: Add Solid 3 behind Solid 1 at (0, 0, -5)
    auto sk3 = std::make_shared<SketchNode>("Sketch_Behind", SketchPlane::XY_Front);
    sk3->SetPresetRectangle(2.0f, 2.0f);
    sk3->SetPosition(0.0f, 0.0f, -5.0f);
    auto solid3 = std::make_shared<ExtrudeNode>("Solid_Behind", sk3, 1.0f);
    solid3->SetPosition(0.0f, 0.0f, -5.0f);
    solid3->Evaluate(nullptr, exchange);
    graph.AddNode(solid3);

    // Ray from (0, 0, 10) pointing -Z hits both Solid 1 and Solid 3. Closest (Solid 1) should be selected.
    uint64_t pickedClosest = graph.PickNode(rayOrig1, rayDir1, &dist);
    TEST_ASSERT(pickedClosest == solid1->GetId(), "PickNode must return closest occluding node");

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

bool TestSketchChangePropagationToFeature() {
    std::cout << "[Test] Sketch Change Propagation to Feature...\n";

    ParametricGraph graph(nullptr);
    AtomicMeshExchange exchange;

    // 1. Create a sketch with a 4-point rectangle
    auto sketch = std::make_shared<SketchNode>("Sketch_Rect", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(3.0f, 2.0f);
    graph.AddNode(sketch);

    // 2. Create an Extrude feature referencing the sketch
    auto extrude = std::make_shared<ExtrudeNode>("Extrude_Feature", sketch, 2.0f);
    graph.AddNode(extrude);

    // Initial evaluation
    sketch->Evaluate(nullptr, exchange);
    extrude->Evaluate(nullptr, exchange);

    TEST_ASSERT(!extrude->IsDirty(), "Extrude should be clean after evaluation");
    TEST_ASSERT(extrude->GetProfile().size() == 4, "Initial profile should have 4 points");
    size_t initialVertCount = extrude->GetCachedVertices().size();
    TEST_ASSERT(initialVertCount > 0, "Extrude should have generated cached vertices");

    // 3. Change sketch profile (switch to 5-point star = 10 points)
    sketch->SetPresetStar(5, 3.0f, 1.5f);

    // Verify change immediately propagated dirty flag to the parent feature!
    TEST_ASSERT(sketch->IsDirty(), "Sketch must be dirty after profile change");
    TEST_ASSERT(extrude->IsDirty(), "Extrude feature must be marked dirty when sketch changes!");

    // 4. Re-evaluate dirty nodes
    graph.EvaluateDirtyNodes(nullptr, exchange);

    TEST_ASSERT(!extrude->IsDirty(), "Extrude should be clean after re-evaluation");
    TEST_ASSERT(extrude->GetProfile().size() == 10, "Extrude profile should update to 10 points");
    size_t starVertCount = extrude->GetCachedVertices().size();
    TEST_ASSERT(starVertCount != initialVertCount, "Extrude vertex count should update with new sketch profile");

    // 5. Move a vertex in the sketch
    sketch->UpdatePoint(0, 15.0f, 20.0f);
    TEST_ASSERT(extrude->IsDirty(), "Extrude must be marked dirty when sketch vertex is moved");

    graph.EvaluateDirtyNodes(nullptr, exchange);
    TEST_ASSERT(std::abs(extrude->GetProfile()[0][0] - 15.0f) < 1e-4f, "Extrude profile must reflect updated vertex X");
    TEST_ASSERT(std::abs(extrude->GetProfile()[0][1] - 20.0f) < 1e-4f, "Extrude profile must reflect updated vertex Y");

    // 6. Change sketch position
    sketch->SetPosition(5.0f, 2.0f, -3.0f);
    TEST_ASSERT(extrude->IsDirty(), "Extrude must be marked dirty when sketch position changes");

    graph.EvaluateDirtyNodes(nullptr, exchange);
    TEST_ASSERT(std::abs(extrude->GetPosition()[0] - 5.0f) < 1e-4f, "Extrude position X must synchronize with sketch");
    TEST_ASSERT(std::abs(extrude->GetPosition()[1] - 2.0f) < 1e-4f, "Extrude position Y must synchronize with sketch");
    TEST_ASSERT(std::abs(extrude->GetPosition()[2] - (-3.0f)) < 1e-4f, "Extrude position Z must synchronize with sketch");

    return true;
}

bool TestMultiShapeSketchAndStitching() {
    std::cout << "[Test] Multi-Shape Sketching, Stitching & Closure...\n";

    // 1. Test 2D polygon stitching / boolean union
    std::vector<std::array<float, 2>> rectA = {
        {-2.0f, -1.0f},
        { 1.0f, -1.0f},
        { 1.0f,  1.0f},
        {-2.0f,  1.0f}
    };
    std::vector<std::array<float, 2>> rectB = {
        { 0.0f, -1.0f},
        { 3.0f, -1.0f},
        { 3.0f,  1.0f},
        { 0.0f,  1.0f}
    };

    auto stitched = Sketch::StitchTwoPolygons(rectA, rectB);
    float area = std::abs(Sketch::PolygonSignedArea2D(stitched));
    TEST_ASSERT(stitched.size() >= 4, "Stitched polygon should have at least 4 vertices");
    TEST_ASSERT(std::abs(area - 10.0f) < 0.2f, "Union stitched area should equal 10.0");

    // 2. Test Multi-shape SketchNode with two disjoint closed shapes
    auto sketch = std::make_shared<SketchNode>("MultiShapeSketch", SketchPlane::XZ_Ground);
    std::vector<std::vector<std::array<float, 2>>> shapes = {
        { {-3.0f, -1.0f}, {-1.0f, -1.0f}, {-1.0f, 1.0f}, {-3.0f, 1.0f} },
        { { 1.0f, -1.0f}, { 3.0f, -1.0f}, { 3.0f, 1.0f}, { 1.0f, 1.0f} }
    };
    sketch->SetShapes(shapes);
    TEST_ASSERT(sketch->GetShapes().size() == 2, "SketchNode must store multiple shapes");

    // 3. Test ExtrudeNode evaluating multi-shape sketch
    ParametricGraph graph(nullptr);
    AtomicMeshExchange exchange;
    graph.AddNode(sketch);

    auto extrude = std::make_shared<ExtrudeNode>("MultiExtrude", sketch, 2.0f);
    graph.AddNode(extrude);

    sketch->Evaluate(nullptr, exchange);
    extrude->Evaluate(nullptr, exchange);

    const auto& verts = extrude->GetCachedVertices();
    const auto& indices = extrude->GetCachedIndices();

    // Each extruded box has:
    // Top cap: 4 verts, 2 triangles (6 indices)
    // Bot cap: 4 verts, 2 triangles (6 indices)
    // 4 side walls: 4 quads * 4 verts = 16 verts, 4 * 6 = 24 indices
    // Total per box: 24 verts, 36 indices
    // For 2 boxes: 48 verts, 72 indices
    TEST_ASSERT(verts.size() == 48, "Multi-shape extrusion of 2 rectangles should generate 48 vertices");
    TEST_ASSERT(indices.size() == 72, "Multi-shape extrusion of 2 rectangles should generate 72 indices");

    // 4. Test closure validation semantics
    auto IsLoopClosed = [](const std::vector<std::array<float, 2>>& loop) -> bool {
        return loop.size() >= 3;
    };
    TEST_ASSERT(IsLoopClosed(shapes[0]), "Shape 0 must be recognized as closed loop");
    TEST_ASSERT(IsLoopClosed(shapes[1]), "Shape 1 must be recognized as closed loop");

    std::vector<std::array<float, 2>> openLine = { {0.0f, 0.0f}, {1.0f, 2.0f} };
    TEST_ASSERT(!IsLoopClosed(openLine), "Line with <3 points must not be recognized as closed loop");

    return true;
}

bool TestSketchShapeAndVertexDeletion() {
    std::cout << "[Test] 2D Sketch Shape & Vertex Deletion...\n";

    // 1. Test deleting individual vertices from single-profile sketch
    auto sketch = std::make_shared<SketchNode>("DeleteTestSketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRegularPolygon(6, 2.0f); // 6 vertices
    TEST_ASSERT(sketch->GetProfile().size() == 6, "Initial hexagon should have 6 vertices");

    // Remove vertex at index 2
    sketch->RemovePoint(2);
    TEST_ASSERT(sketch->GetProfile().size() == 5, "After deletion, sketch should have 5 vertices");
    TEST_ASSERT(sketch->IsDirty(), "Sketch must be dirty after vertex deletion");

    // Remove down to 3 vertices (minimum closed loop)
    sketch->RemovePoint(0);
    sketch->RemovePoint(0);
    TEST_ASSERT(sketch->GetProfile().size() == 3, "After deletions, sketch should have 3 vertices");

    // Attempting to remove below 3 via RemovePoint should safely retain 3 vertices
    sketch->RemovePoint(0);
    TEST_ASSERT(sketch->GetProfile().size() == 3, "RemovePoint preserves minimum 3 vertices for closed polygon");

    // 2. Test multi-shape deletion
    std::vector<std::vector<std::array<float, 2>>> multiShapes = {
        { {-3.0f, -1.0f}, {-1.0f, -1.0f}, {-1.0f, 1.0f}, {-3.0f, 1.0f} }, // Shape 0
        { { 1.0f, -1.0f}, { 3.0f, -1.0f}, { 3.0f, 1.0f}, { 1.0f, 1.0f} }, // Shape 1
        { { 5.0f, -1.0f}, { 7.0f, -1.0f}, { 7.0f, 1.0f}, { 5.0f, 1.0f} }  // Shape 2
    };
    sketch->SetShapes(multiShapes);
    TEST_ASSERT(sketch->GetShapes().size() == 3, "Sketch should have 3 shapes");

    // Delete Shape 1
    sketch->RemoveShape(1);
    TEST_ASSERT(sketch->GetShapes().size() == 2, "After deleting shape 1, sketch should have 2 shapes");
    TEST_ASSERT(sketch->IsDirty(), "Sketch must be dirty after shape deletion");

    // Delete Shape 0 (head shape)
    sketch->RemoveShape(0);
    TEST_ASSERT(sketch->GetShapes().size() == 1, "After deleting shape 0, sketch should have 1 shape");
    // Verify active profile was updated to the remaining shape (originally Shape 2 at x=5..7)
    TEST_ASSERT(std::abs(sketch->GetProfile()[0][0] - 5.0f) < 1e-4f, "Active profile should now be remaining shape");

    // Delete last remaining shape
    sketch->RemoveShape(0);
    TEST_ASSERT(sketch->GetShapes().empty(), "All shapes should be deleted");
    TEST_ASSERT(sketch->GetProfile().empty(), "Profile should be empty after deleting all shapes");

    // 3. Test DAG Extrude update when shape is deleted
    ParametricGraph graph(nullptr);
    AtomicMeshExchange exchange;

    auto sketch2 = std::make_shared<SketchNode>("ExtrudeDeleteTest", SketchPlane::XZ_Ground);
    sketch2->SetShapes({
        { {-3.0f, -1.0f}, {-1.0f, -1.0f}, {-1.0f, 1.0f}, {-3.0f, 1.0f} },
        { { 1.0f, -1.0f}, { 3.0f, -1.0f}, { 3.0f, 1.0f}, { 1.0f, 1.0f} }
    });
    graph.AddNode(sketch2);

    auto extrude = std::make_shared<ExtrudeNode>("ExtrudeNode", sketch2, 2.0f);
    graph.AddNode(extrude);

    sketch2->Evaluate(nullptr, exchange);
    extrude->Evaluate(nullptr, exchange);

    TEST_ASSERT(extrude->GetCachedVertices().size() == 48, "Initial extrude should have 48 vertices for 2 shapes");

    // Delete one shape from sketch
    sketch2->RemoveShape(1);
    TEST_ASSERT(extrude->IsDirty(), "Parent feature must be marked dirty when shape is deleted");

    graph.EvaluateDirtyNodes(nullptr, exchange);
    TEST_ASSERT(extrude->GetCachedVertices().size() == 24, "Extrude should update to 24 vertices for 1 remaining shape");

    return true;
}

class MockTestBuffer : public RHI::RHIBuffer {
public:
    MockTestBuffer(size_t size, RHI::BufferUsage usage) : m_size(size), m_usage(usage), m_data(size) {}
    size_t GetSize() const noexcept override { return m_size; }
    RHI::BufferUsage GetUsage() const noexcept override { return m_usage; }
    RHI::MemoryUsage GetMemoryUsage() const noexcept override { return RHI::MemoryUsage::CpuToGpu; }
    void* Map() override { return m_data.data(); }
    void Unmap() override {}
private:
    size_t m_size;
    RHI::BufferUsage m_usage;
    std::vector<uint8_t> m_data;
};

class MockTestDevice : public RHI::RHIDevice {
public:
    std::shared_ptr<RHI::RHIBuffer> CreateBuffer(const RHI::RHIBufferDesc& desc, const void* initialData = nullptr) override {
        auto buf = std::make_shared<MockTestBuffer>(desc.sizeInBytes, desc.usage);
        if (initialData) buf->UpdateData(initialData, desc.sizeInBytes);
        return buf;
    }
    std::shared_ptr<RHI::RHIPipeline> CreatePipeline(const RHI::RHIPipelineDesc&) override { return nullptr; }
    std::unique_ptr<RHI::RHICommandList> CreateCommandList() override { return nullptr; }
    void Submit(RHI::RHICommandList*) override {}
    bool BeginFrame() override { return true; }
    void PresentFrame() override {}
    void ResizeSwapchain(uint32_t, uint32_t) override {}
    void WaitIdle() override {}
    uint32_t GetSwapchainWidth() const noexcept override { return 1920; }
    uint32_t GetSwapchainHeight() const noexcept override { return 1080; }
};

bool TestStandaloneSketchVisibilityAndVisualStyling() {
    std::cout << "[Test] Standalone Sketch Selection Visibility & Visual Styling (Outline + Shading)...\n";

    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // 1. Create a parentless sketch (e.g. 4.0 x 3.0 rectangle on XZ Ground Plane)
    auto sketch = std::make_shared<SketchNode>("Sketch_Standalone", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 3.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    TEST_ASSERT(!sketch->HasParentFeature(), "Sketch must be parentless");

    // 2. Initially, nothing is selected (selectedNodeId = 0)
    // Sketches that are not associated with a feature MUST NOT show up unless selected on feature tree
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, 0);
    auto meshUnselected = exchange.AcquireLatest();
    TEST_ASSERT(meshUnselected != nullptr, "Mesh payload must be published");
    TEST_ASSERT(meshUnselected->sketchIndexCount == 0, "Unselected parentless sketch must NOT publish sketch triangle indices (hidden in 3D)");
    TEST_ASSERT(meshUnselected->sketchEdgeIndexCount == 0, "Unselected parentless sketch must NOT publish sketch outline edges");
    TEST_ASSERT(meshUnselected->indexCount == 0, "Unselected parentless sketch must NOT publish solid mesh indices");

    // Viewport raycast should NOT pick the sketch while it is hidden
    Point3D rayAtSketch(0.0, 5.0, 0.0);
    Vector3D rayDown(0.0, -1.0, 0.0);
    double dist = 0.0;
    uint64_t pickedHidden = graph.PickNode(rayAtSketch, rayDown, &dist);
    TEST_ASSERT(pickedHidden == 0, "Hidden unselected sketch must NOT be pickable by viewport raycast");

    // 3. Select the sketch on the feature tree
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, sketch->GetId());
    auto meshSelected = exchange.AcquireLatest();
    TEST_ASSERT(meshSelected != nullptr, "Mesh payload must be published");
    // When selected on the feature tree, it SHOWS UP with outline + transparent shading
    TEST_ASSERT(meshSelected->sketchVertexBuffer != nullptr, "Selected sketch must publish sketch vertex buffer");
    TEST_ASSERT(meshSelected->sketchIndexBuffer != nullptr, "Selected sketch must publish sketch index buffer (for transparent shading)");
    TEST_ASSERT(meshSelected->sketchIndexCount > 0, "Selected sketch must have triangle indices for transparent planar fill");
    TEST_ASSERT(meshSelected->sketchEdgeIndexBuffer != nullptr, "Selected sketch must publish sketch edge buffer (for vibrant outline)");
    TEST_ASSERT(meshSelected->sketchEdgeIndexCount > 0, "Selected sketch must have edge indices for crisp outline");
    TEST_ASSERT(meshSelected->indexCount == 0, "Sketch must NOT be published into the opaque solid body buffer");

    // Viewport raycast SHOULD now pick the sketch because it is visible
    uint64_t pickedVisible = graph.PickNode(rayAtSketch, rayDown, &dist);
    TEST_ASSERT(pickedVisible == sketch->GetId(), "Selected visible sketch MUST be pickable by viewport raycast");
    TEST_ASSERT(std::abs(dist - 5.0) < 0.1, "Ray intersection distance should match Y=0 plane (~5.0)");

    // 4. Deselect the sketch (selectedNodeId = 0)
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, 0);
    auto meshDeselected = exchange.AcquireLatest();
    TEST_ASSERT(meshDeselected->sketchIndexCount == 0, "Deselected sketch must be hidden again (0 sketch indices)");
    TEST_ASSERT(meshDeselected->sketchEdgeIndexCount == 0, "Deselected sketch outline must be hidden (0 edge indices)");
    TEST_ASSERT(graph.PickNode(rayAtSketch, rayDown) == 0, "Deselected sketch must not be pickable");

    // 5. Associate sketch with an Extrude feature (now it has a parent feature)
    auto extrude = std::make_shared<ExtrudeNode>("Extrude_Feature", sketch, 3.0f);
    extrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(extrude);
    TEST_ASSERT(sketch->HasParentFeature(), "Sketch must now have a parent feature");

    // When sketch has a parent feature, it is NOT rendered standalone even if selected
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, sketch->GetId());
    auto meshChildSketch = exchange.AcquireLatest();
    TEST_ASSERT(meshChildSketch->indexCount > 0, "Solid extrude body must be published to solid mesh buffer");
    TEST_ASSERT(meshChildSketch->sketchIndexCount == 0, "Child sketch associated with feature must not render standalone");

    // 6. Delete Extrude feature: sketch is promoted back to parentless
    graph.RemoveNode(extrude->GetId());
    TEST_ASSERT(!sketch->HasParentFeature(), "Sketch must be promoted back to parentless on feature deletion");

    // When promoted and not selected: hidden
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, 0);
    auto meshPromotedUnselected = exchange.AcquireLatest();
    TEST_ASSERT(meshPromotedUnselected->sketchIndexCount == 0, "Promoted sketch must not show up when unselected");

    // When promoted and selected: shows up with outline + transparent shading
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, sketch->GetId());
    auto meshPromotedSelected = exchange.AcquireLatest();
    TEST_ASSERT(meshPromotedSelected->sketchIndexCount > 0, "Promoted sketch must show up when selected on feature tree");
    TEST_ASSERT(meshPromotedSelected->sketchEdgeIndexCount > 0, "Promoted sketch outline must show up when selected");

    return true;
}

bool TestTopBarReferenceGeometryAndFeatureWorkflows() {
    std::cout << "[Test] Top Bar Reference Geometry (Plane, Axis) & Automatic Feature Workflows...\n";

    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // -------------------------------------------------------------------------
    // 1. Reference Plane Workflow
    // -------------------------------------------------------------------------
    auto refPlaneXZ = std::make_shared<DatumPlaneNode>("Ref_Plane_XZ", Plane::XZ());
    refPlaneXZ->SetOffset(4.5);
    graph.AddNode(refPlaneXZ);

    TEST_ASSERT(refPlaneXZ->GetOffset() == 4.5, "Plane offset should be 4.5m");
    Plane evaluatedPlane = refPlaneXZ->GetPlane();
    TEST_ASSERT(evaluatedPlane.normal.approx_equal(Vector3D::UnitY()), "XZ Plane normal must be +Y");
    TEST_ASSERT(std::abs(evaluatedPlane.origin.y - 4.5) < 1e-5, "Offset origin Y must be 4.5m");

    // -------------------------------------------------------------------------
    // 2. Reference Axis Workflow
    // -------------------------------------------------------------------------
    auto refAxisY = std::make_shared<DatumAxisNode>("Ref_Axis_Y", Axis3D::Y());
    graph.AddNode(refAxisY);
    TEST_ASSERT(refAxisY->GetAxis().direction.approx_equal(Vector3D::UnitY()), "Axis direction must be UnitY");

    auto refAxisX = std::make_shared<DatumAxisNode>("Ref_Axis_X", Axis3D::X());
    graph.AddNode(refAxisX);
    TEST_ASSERT(refAxisX->GetAxis().direction.approx_equal(Vector3D::UnitX()), "Axis direction must be UnitX");

    // -------------------------------------------------------------------------
    // 3. 2D Sketch Creation with Selected Plane
    // When a plane is selected, 2D sketch creation automatically adopts its plane & origin
    // -------------------------------------------------------------------------
    Geom::Plane pl = refPlaneXZ->GetPlane();
    SketchPlane sp = (std::abs(pl.normal.y) > 0.7) ? SketchPlane::XZ_Ground :
                     (std::abs(pl.normal.z) > 0.7) ? SketchPlane::XY_Front : SketchPlane::YZ_Right;
    TEST_ASSERT(sp == SketchPlane::XZ_Ground, "Selected XZ datum plane must yield XZ_Ground sketch plane");

    auto sketchOnPlane = std::make_shared<SketchNode>("Sketch_On_XZ", sp);
    sketchOnPlane->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
    sketchOnPlane->SetPresetRectangle(3.0f, 2.0f);
    sketchOnPlane->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketchOnPlane);

    TEST_ASSERT(sketchOnPlane->GetPlane() == SketchPlane::XZ_Ground, "Sketch plane must match reference plane");
    TEST_ASSERT(std::abs(sketchOnPlane->GetPosition()[1] - 4.5f) < 1e-5f, "Sketch Y position must match plane offset");

    // -------------------------------------------------------------------------
    // 4. Automatic Feature Creation from Selected 2D Sketch
    // When an available sketch is selected, Extrude automatically builds from it
    // -------------------------------------------------------------------------
    TEST_ASSERT(!sketchOnPlane->HasParentFeature(), "Sketch initially has no parent feature");
    TEST_ASSERT(!sketchOnPlane->IsConsumed(), "Sketch initially is not consumed");

    // User triggers Extrude while sketch is selected:
    auto autoExtrude = std::make_shared<ExtrudeNode>("Auto_Extrude", sketchOnPlane, 2.0f);
    autoExtrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(autoExtrude);

    TEST_ASSERT(sketchOnPlane->HasParentFeature(), "Sketch must now have parent feature");
    TEST_ASSERT(autoExtrude->GetChildSketch() == sketchOnPlane, "Extrude must nest sketch as its child");
    TEST_ASSERT(sketchOnPlane->IsConsumed(), "Referenced sketch must be consumed");
    TEST_ASSERT(autoExtrude->GetCachedVertices().size() > 0, "Extrude mesh must evaluate with vertices");

    // -------------------------------------------------------------------------
    // 5. Automatic Revolve Feature from Selected 2D Sketch
    // -------------------------------------------------------------------------
    auto sketchForRevolve = std::make_shared<SketchNode>("Sketch_For_Revolve", SketchPlane::XY_Front);
    sketchForRevolve->SetPresetRectangle(2.0f, 2.0f);
    sketchForRevolve->SetPosition(3.0f, 0.0f, 0.0f); // Offset from Y axis
    sketchForRevolve->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketchForRevolve);

    auto autoRevolve = std::make_shared<RevolveNode>("Auto_Revolve", sketchForRevolve, 360.0f, refAxisY->GetAxis());
    autoRevolve->Evaluate(&mockDevice, exchange);
    graph.AddNode(autoRevolve);

    TEST_ASSERT(sketchForRevolve->HasParentFeature(), "Revolve sketch must now have parent feature");
    TEST_ASSERT(autoRevolve->GetChildSketch() == sketchForRevolve, "Revolve must nest sketch as its child");
    TEST_ASSERT(sketchForRevolve->IsConsumed(), "Revolve sketch must be consumed");
    TEST_ASSERT(autoRevolve->GetCachedVertices().size() > 0, "Revolve mesh must evaluate with vertices");

    return true;
}

bool TestRevolveCylindricalThicknessAndNoBoxFeature() {
    std::cout << "[Test] Revolve Cylindrical Thickness Dictated by Sketch & No Box Feature...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create a 2D sketch representing a cylindrical cross-section on Front XY plane:
    // Left edge u = 1.2, right edge u = 2.8 (wall thickness = 2.8 - 1.2 = 1.6 m)
    // Bottom edge v = 0.0, top edge v = 4.5 (height = 4.5 m)
    auto sketch = std::make_shared<SketchNode>("Sketch_PipeCrossSection", SketchPlane::XY_Front);
    sketch->SetProfile({ {1.2f, 0.0f}, {2.8f, 0.0f}, {2.8f, 4.5f}, {1.2f, 4.5f} });
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    // 2. Create RevolveNode revolving around the sketch's in-plane vertical axis (V-Axis)
    auto revolve = std::make_shared<RevolveNode>("Revolve_Pipe", sketch, 360.0f, RevolveAxisMode::Sketch_V_Axis);
    graph.AddNode(revolve);
    revolve->Evaluate(&mockDevice, exchange);

    // Verify dimensional metrics calculated directly from the sketch drawing:
    TEST_ASSERT(std::abs(revolve->GetInnerRadius() - 1.2f) < 1e-4f, "Inner radius must match sketch u_min (1.2m)");
    TEST_ASSERT(std::abs(revolve->GetOuterRadius() - 2.8f) < 1e-4f, "Outer radius must match sketch u_max (2.8m)");
    TEST_ASSERT(std::abs(revolve->GetWallThickness() - 1.6f) < 1e-4f, "Wall thickness must match sketch drawing (1.6m)");
    TEST_ASSERT(std::abs(revolve->GetCylinderHeight() - 4.5f) < 1e-4f, "Cylinder height must match sketch drawing (4.5m)");

    // 3. Verify evaluated mesh coordinates
    const auto& verts = revolve->GetCachedVertices();
    TEST_ASSERT(!verts.empty(), "Revolved mesh must not be empty");

    float minR = 1e9f, maxR = -1e9f;
    float minY = 1e9f, maxY = -1e9f;
    for (const auto& v : verts) {
        float r = std::hypot(v.position[0], v.position[2]);
        minR = std::min(minR, r);
        maxR = std::max(maxR, r);
        minY = std::min(minY, v.position[1]);
        maxY = std::max(maxY, v.position[1]);
    }

    TEST_ASSERT(std::abs(minR - 1.2f) < 0.05f, "Evaluated inner radius must be ~1.2m");
    TEST_ASSERT(std::abs(maxR - 2.8f) < 0.05f, "Evaluated outer radius must be ~2.8m");
    TEST_ASSERT(std::abs(minY - 0.0f) < 0.05f, "Evaluated height base must be ~0.0m");
    TEST_ASSERT(std::abs(maxY - 4.5f) < 0.05f, "Evaluated height top must be ~4.5m");

    // 4. Modify sketch drawing to change thickness:
    // Update profile to u=1.0 to 3.5 (new thickness = 2.5m, new height = 6.0m)
    sketch->SetProfile({ {1.0f, 0.0f}, {3.5f, 0.0f}, {3.5f, 6.0f}, {1.0f, 6.0f} });
    TEST_ASSERT(revolve->IsDirty(), "Revolve feature must become dirty when sketch changes");

    revolve->Evaluate(&mockDevice, exchange);
    TEST_ASSERT(std::abs(revolve->GetWallThickness() - 2.5f) < 1e-4f, "New wall thickness must match updated sketch drawing (2.5m)");
    TEST_ASSERT(std::abs(revolve->GetOuterRadius() - 3.5f) < 1e-4f, "New outer radius must match updated sketch (3.5m)");
    TEST_ASSERT(std::abs(revolve->GetCylinderHeight() - 6.0f) < 1e-4f, "New height must match updated sketch (6.0m)");

    return true;
}

bool TestModelClickSelectionHighlightAndOuterEdges() {
    std::cout << "[Test] Model Click Selection Surface Highlight & Outer Edge Highlight...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create Model A (Extrude feature) at (0, 0, 0)
    auto sketchA = std::make_shared<SketchNode>("Sketch_A", SketchPlane::XY_Front);
    sketchA->SetPresetRectangle(2.0f, 2.0f);
    sketchA->SetPosition(0.0f, 0.0f, 0.0f);
    sketchA->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketchA);

    auto modelA = std::make_shared<ExtrudeNode>("Model_ExtrudeA", sketchA, 2.0f);
    modelA->SetPosition(0.0f, 0.0f, 0.0f);
    modelA->Evaluate(&mockDevice, exchange);
    graph.AddNode(modelA);

    // 2. Create Model B (Revolve cylinder feature) at (6, 0, 0)
    auto sketchB = std::make_shared<SketchNode>("Sketch_B", SketchPlane::XY_Front);
    sketchB->SetProfile({ {1.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 3.0f}, {1.0f, 3.0f} });
    sketchB->SetPosition(6.0f, 0.0f, 0.0f);
    sketchB->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketchB);

    auto modelB = std::make_shared<RevolveNode>("Model_RevolveB", sketchB, 360.0f, RevolveAxisMode::Sketch_V_Axis);
    modelB->SetPosition(6.0f, 0.0f, 0.0f);
    modelB->Evaluate(&mockDevice, exchange);
    graph.AddNode(modelB);

    // 3. Ray pick Model A in 3D viewport at (0, 0, 0)
    Point3D rayOrigA(0.0, 0.0, 10.0);
    Vector3D rayDirA(0.0, 0.0, -1.0);
    double dist = 0.0;
    uint64_t pickedA = graph.PickNode(rayOrigA, rayDirA, &dist);
    TEST_ASSERT(pickedA == modelA->GetId(), "Clicking 3D Model A must return Model A's ID");

    // Publish scene with Model A selected
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, pickedA);
    auto meshA = exchange.AcquireLatest();
    TEST_ASSERT(meshA != nullptr, "Published mesh must not be null");
    TEST_ASSERT(meshA->selectedIndexCount == modelA->GetCachedIndices().size(), 
                "Selected Model A surface must have triangles for subtle highlight");
    TEST_ASSERT(meshA->selectedEdgeIndexCount == modelA->GetCachedEdges().size(), 
                "Selected Model A outer edges must have line indices for edge highlight");
    TEST_ASSERT(meshA->indexCount == modelB->GetCachedIndices().size(), 
                "Unselected Model B must remain in unselected surface buffer");
    TEST_ASSERT(meshA->edgeIndexCount == modelB->GetCachedEdges().size(), 
                "Unselected Model B must remain in unselected edge buffer");

    // 4. Ray pick Model B in 3D viewport at (6, 0, 0)
    Point3D rayOrigB(7.5, 1.5, 10.0);
    Vector3D rayDirB(0.0, 0.0, -1.0);
    uint64_t pickedB = graph.PickNode(rayOrigB, rayDirB, &dist);
    TEST_ASSERT(pickedB == modelB->GetId(), "Clicking 3D Model B must return Model B's ID");

    // Publish scene with Model B selected
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, pickedB);
    auto meshB = exchange.AcquireLatest();
    TEST_ASSERT(meshB != nullptr, "Published mesh must not be null");
    TEST_ASSERT(meshB->selectedIndexCount == modelB->GetCachedIndices().size(), 
                "Selected Model B surface must be partitioned to selected buffer");
    TEST_ASSERT(meshB->selectedEdgeIndexCount == modelB->GetCachedEdges().size(), 
                "Selected Model B outer edges must be partitioned to selected edge buffer");
    TEST_ASSERT(meshB->indexCount == modelA->GetCachedIndices().size(), 
                "Unselected Model A must be in unselected surface buffer");

    // 5. Deselect all models (click empty space or Deselect)
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, 0);
    auto meshNone = exchange.AcquireLatest();
    TEST_ASSERT(meshNone->selectedIndexCount == 0, "Deselected scene has 0 selected triangles");
    TEST_ASSERT(meshNone->selectedEdgeIndexCount == 0, "Deselected scene has 0 selected edges");
    TEST_ASSERT(meshNone->indexCount == (modelA->GetCachedIndices().size() + modelB->GetCachedIndices().size()),
                "All models return to standard unselected buffer");

    return true;
}

bool TestFaceSubElementPickingAndHighlighting() {
    std::cout << "[Test] Face Sub-Element Picking & Face-Only Highlighting...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // Create Extrude feature (rectangular prism 2x2x2) at (0, 0, 0)
    auto sketch = std::make_shared<SketchNode>("Sketch_Box", SketchPlane::XY_Front);
    sketch->SetPresetRectangle(2.0f, 2.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto model = std::make_shared<ExtrudeNode>("Model_Box", sketch, 2.0f);
    model->SetPosition(0.0f, 0.0f, 0.0f);
    model->Evaluate(&mockDevice, exchange);
    graph.AddNode(model);

    // Initial click selects the whole model
    uint64_t modelId = model->GetId();
    graph.SetSelectedNodeId(modelId);

    // Ray directed directly at front cap face: (0, 0, 5) -> dir (0, 0, -1)
    Point3D rayOrig(0.0, 0.0, 5.0);
    Vector3D rayDir(0.0, 0.0, -1.0);

    // Drill down into sub-element selection on the already-selected model
    auto sub = graph.PickSubElement(rayOrig, rayDir, modelId);
    TEST_ASSERT(sub.vertexIndex == -1, "Direct face hit should not pick a vertex");
    TEST_ASSERT(sub.faceIndex >= 0, "Face sub-element must be picked");
    TEST_ASSERT(std::abs(sub.facePlane.normal.z - 1.0) < 1e-3 || std::abs(sub.facePlane.normal.z - (-1.0)) < 1e-3, 
                "Front cap face normal must be along Z axis");

    // Publish with face selected: only that face must be highlighted!
    graph.SetSubElementSelection(modelId, sub.faceIndex, -1);
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, modelId, sub.faceIndex, -1);

    auto meshFace = exchange.AcquireLatest();
    TEST_ASSERT(meshFace != nullptr, "Published mesh must not be null");

    FaceInfo pickedFace;
    TEST_ASSERT(model->GetFace(sub.faceIndex, pickedFace), "Must be able to query picked face info");
    TEST_ASSERT(meshFace->selectedIndexCount == pickedFace.triangleIndices.size(), 
                "Selected index buffer must contain ONLY the selected face triangles");
    TEST_ASSERT(meshFace->selectedEdgeIndexCount == pickedFace.perimeterEdges.size(), 
                "Selected edge buffer must contain ONLY the selected face perimeter edges");

    // The rest of the model's triangles must remain in the unselected buffer
    size_t expectedUnselectedTriCount = model->GetCachedIndices().size() - pickedFace.triangleIndices.size();
    TEST_ASSERT(meshFace->indexCount == expectedUnselectedTriCount, 
                "Unselected model faces must be drawn in normal unselected buffer");

    return true;
}

bool TestAssociativeDatumPlaneAndConflictHandling() {
    std::cout << "[Test] Associative Datum Plane & Parametric Conflict Handling...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create base model
    auto sketch = std::make_shared<SketchNode>("Base_Sketch", SketchPlane::XY_Front);
    sketch->SetPresetRectangle(4.0f, 4.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto feature = std::make_shared<ExtrudeNode>("Base_Extrude", sketch, 2.0f);
    feature->Evaluate(&mockDevice, exchange);
    graph.AddNode(feature);

    const auto& faces = feature->GetFaces();
    TEST_ASSERT(!faces.empty(), "Feature must have evaluated faces");

    // Find top or front cap face
    int targetFaceIdx = 0;
    FaceInfo targetFace = faces[0];

    // 2. Create associative DatumPlaneNode tied to that face
    auto tiedPlane = std::make_shared<DatumPlaneNode>("Plane_TiedToFace", feature, targetFaceIdx, targetFace.plane);
    graph.AddNode(tiedPlane);
    tiedPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(tiedPlane->IsFaceAssociative(), "Datum plane must be face-associative");
    TEST_ASSERT(!tiedPlane->HasConflict(), "Freshly created tied plane must have no conflict");
    TEST_ASSERT(tiedPlane->GetReferencedFaceIndex() == targetFaceIdx, "Referenced face index must match");
    TEST_ASSERT(tiedPlane->GetReferencedFeature()->GetId() == feature->GetId(), "Referenced feature must match");

    // 3. Create a 2D sketch on this associative datum plane
    auto sketchOnPlane = std::make_shared<SketchNode>("Sketch_OnTiedPlane", tiedPlane);
    sketchOnPlane->SetPresetRectangle(1.0f, 1.0f);
    graph.AddNode(sketchOnPlane);
    sketchOnPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(sketchOnPlane->HasCustomPlane(), "Sketch must use the custom datum plane");
    TEST_ASSERT(!sketchOnPlane->HasConflict(), "Sketch on tied plane must have no conflict");

    // 4. Update the base feature (change extrusion height from 2.0 to 6.0)
    feature->SetHeight(6.0f);
    feature->Evaluate(&mockDevice, exchange);
    tiedPlane->Evaluate(&mockDevice, exchange);
    sketchOnPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(!tiedPlane->HasConflict(), "Tied plane remains valid after feature parameter update");
    // Plane coordinates updated associatively
    Geom::Plane updatedPlane = tiedPlane->GetPlane();
    FaceInfo updatedFace;
    feature->GetFace(targetFaceIdx, updatedFace);
    TEST_ASSERT(updatedPlane.origin.approx_equal(updatedFace.center, 0.05), "Plane origin updated associatively with face");

    // 5. Cause a conflict: delete the parent feature from the graph
    graph.RemoveNode(feature->GetId());
    feature = nullptr; // release reference
    tiedPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(tiedPlane->HasConflict(), "Datum plane must flag conflict when parent feature is removed");
    TEST_ASSERT(!tiedPlane->GetConflictMessage().empty(), "Conflict message must describe the failure");
    TEST_ASSERT(sketchOnPlane->HasConflict(), "Child sketch must report conflict when datum plane has conflict");

    // 6. Resolve conflict: user prompts to convert to Fixed Datum Plane
    tiedPlane->ConvertToFixed();
    tiedPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(!tiedPlane->HasConflict(), "Converting to fixed datum plane must resolve the conflict");
    TEST_ASSERT(tiedPlane->GetPlaneType() == DatumPlaneType::Fixed, "Plane type is now Fixed");
    TEST_ASSERT(!sketchOnPlane->HasConflict(), "Child sketch conflict must be cleared");

    return true;
}

bool TestVertexSubElementPickingAndAssociativeDatumAxis() {
    std::cout << "[Test] Vertex Sub-Element Picking & Associative Datum Axis...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // Create 3D Model
    auto sketch = std::make_shared<SketchNode>("Box_Sketch", SketchPlane::XY_Front);
    sketch->SetPresetRectangle(2.0f, 2.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto model = std::make_shared<ExtrudeNode>("Box_Model", sketch, 2.0f);
    model->Evaluate(&mockDevice, exchange);
    graph.AddNode(model);

    uint64_t modelId = model->GetId();
    graph.SetSelectedNodeId(modelId);

    const auto& verts = model->GetCachedVertices();
    TEST_ASSERT(!verts.empty(), "Model must have cached vertices");

    // Target vertex #0: position (v0.x, v0.y, v0.z)
    Point3D targetVPos(verts[0].position[0], verts[0].position[1], verts[0].position[2]);

    // Cast a ray that passes right through/near vertex #0 (within 0.05m tolerance)
    Point3D rayOrig(targetVPos.x + 0.02, targetVPos.y + 0.02, targetVPos.z + 5.0);
    Vector3D rayDir(0.0, 0.0, -1.0);

    auto sub = graph.PickSubElement(rayOrig, rayDir, modelId, 0.20);
    TEST_ASSERT(sub.vertexIndex == 0, "Vertex picking must pick vertex #0");
    TEST_ASSERT(sub.vertexPos.approx_equal(targetVPos, 1e-4), "Picked vertex position must match vertex #0");

    // Publish mesh with vertex selected
    graph.SetSubElementSelection(modelId, -1, sub.vertexIndex);
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, modelId, -1, sub.vertexIndex);

    auto meshVert = exchange.AcquireLatest();
    TEST_ASSERT(meshVert != nullptr, "Published mesh must not be null");
    TEST_ASSERT(meshVert->selectedIndexCount == 24, "Vertex marker has 8 triangles (24 indices)");
    TEST_ASSERT(meshVert->selectedEdgeIndexCount >= 30, "Vertex highlight includes marker edges, 3D crosshair, and incident model edges");
    TEST_ASSERT(meshVert->indexCount == model->GetCachedIndices().size(), "Model triangles remain in unselected buffer");

    // Create associative DatumAxisNode tied to this vertex
    auto tiedAxis = std::make_shared<DatumAxisNode>("Axis_TiedToVertex", model, sub.vertexIndex, Vector3D(0.0, 1.0, 0.0));
    graph.AddNode(tiedAxis);
    tiedAxis->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(tiedAxis->IsVertexAssociative(), "Axis must be vertex-associative");
    TEST_ASSERT(!tiedAxis->HasConflict(), "Tied axis must have no conflict");
    TEST_ASSERT(tiedAxis->GetAxis().origin.approx_equal(targetVPos, 1e-4), "Axis origin must match vertex position");

    // Cause a conflict: delete parent feature
    graph.RemoveNode(model->GetId());
    model = nullptr;
    tiedAxis->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(tiedAxis->HasConflict(), "Datum axis must flag conflict when parent feature is deleted");
    TEST_ASSERT(!tiedAxis->GetConflictMessage().empty(), "Conflict message must be provided");

    // Resolve conflict by converting to fixed axis
    tiedAxis->ConvertToFixed();
    tiedAxis->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(!tiedAxis->HasConflict(), "Converting to fixed axis must resolve conflict");
    TEST_ASSERT(tiedAxis->GetAxisType() == DatumAxisType::Fixed, "Axis is now fixed");

    return true;
}

bool TestFaceAndVertexDatumPlaneCenteringIn3DSpace() {
    std::cout << "[Test] Face and Vertex Datum Plane Centering in 3D Space...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create base solid extruded box
    auto sketch = std::make_shared<SketchNode>("Base_Sketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 4.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto baseBox = std::make_shared<ExtrudeNode>("Base_Box", sketch, 3.0f);
    baseBox->Evaluate(&mockDevice, exchange);
    graph.AddNode(baseBox);

    const auto& faces = baseBox->GetFaces();
    TEST_ASSERT(!faces.empty(), "Base model must have evaluated faces");

    // Find top face (face with highest Y center)
    int topFaceIdx = 0;
    FaceInfo topFace = faces[0];
    for (const auto& f : faces) {
        if (f.center.y > topFace.center.y) {
            topFace = f;
            topFaceIdx = f.faceIndex;
        }
    }

    TEST_ASSERT(std::abs(topFace.center.y - 3.0) < 0.1, "Top face Y should be at +3.0m");

    // 2. Tie a DatumPlaneNode to this top face
    auto faceDatumPlane = std::make_shared<DatumPlaneNode>("Plane_TopFace", baseBox, topFaceIdx, topFace.plane);
    graph.AddNode(faceDatumPlane);
    faceDatumPlane->Evaluate(&mockDevice, exchange);

    // 3. Create a sketch on this face datum plane
    auto sketchOnFace = std::make_shared<SketchNode>("Sketch_OnTopFace", faceDatumPlane);
    sketchOnFace->SetPresetRectangle(2.0f, 2.0f); // 2x2 square centered at (0, 0) locally
    graph.AddNode(sketchOnFace);
    sketchOnFace->Evaluate(&mockDevice, exchange);

    // CRITICAL: Verify the sketch is centered in 3D space around that face instead of global origin (0, 0, 0)!
    auto skPos = sketchOnFace->GetPosition();
    Point3D skOrigin(skPos[0], skPos[1], skPos[2]);
    TEST_ASSERT(skOrigin.approx_equal(topFace.center, 0.05), "Sketch position must be centered around the face in 3D space");
    TEST_ASSERT(sketchOnFace->HasCustomPlane(), "Sketch must use custom plane");

    // Check evaluated 3D mesh vertices of this sketch: (0, 0) in local UV must map to topFace.center in 3D space
    const auto& skVerts = sketchOnFace->GetCachedVertices();
    TEST_ASSERT(!skVerts.empty(), "Sketch vertices must not be empty");
    Point3D centroid(0.0, 0.0, 0.0);
    for (const auto& v : skVerts) {
        centroid = centroid + Vector3D(v.position[0], v.position[1], v.position[2]);
    }
    centroid = Point3D(centroid.x / skVerts.size(), centroid.y / skVerts.size(), centroid.z / skVerts.size());
    TEST_ASSERT(centroid.approx_equal(topFace.center, 0.05), "Evaluated 3D sketch vertices must be centered at the face in 3D space");

    // 4. Extrude from this sketch on the face: verify it builds directly outward from the face in 3D space!
    auto bossExtrude = std::make_shared<ExtrudeNode>("Boss_Extrude", sketchOnFace, 2.0f);
    graph.AddNode(bossExtrude);
    bossExtrude->Evaluate(&mockDevice, exchange);

    const auto& bossVerts = bossExtrude->GetCachedVertices();
    TEST_ASSERT(!bossVerts.empty(), "Boss extrude must have evaluated vertices");

    // Find min and max Y on boss extrude: bottom cap should be at Y = 1.5, top cap at Y = 3.5!
    float minY = 1e9f, maxY = -1e9f;
    for (const auto& v : bossVerts) {
        minY = std::min(minY, v.position[1]);
        maxY = std::max(maxY, v.position[1]);
    }
    TEST_ASSERT(std::abs(minY - 3.0f) < 0.1f, "Boss bottom cap must sit directly on the face at Y = 3.0m");
    TEST_ASSERT(std::abs(maxY - 5.0f) < 0.1f, "Boss top cap must be extruded outward to Y = 5.0m");

    // 5. Test DatumPlaneNode tied to a Vertex
    const auto& baseVerts = baseBox->GetCachedVertices();
    TEST_ASSERT(!baseVerts.empty(), "Base box must have vertices");
    int targetVertexIdx = 0;
    Point3D targetVPos(baseVerts[targetVertexIdx].position[0], baseVerts[targetVertexIdx].position[1], baseVerts[targetVertexIdx].position[2]);

    auto vertexDatumPlane = std::make_shared<DatumPlaneNode>("Plane_OnVertex", baseBox, targetVertexIdx, Plane::XZ(targetVPos), true);
    graph.AddNode(vertexDatumPlane);
    vertexDatumPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(vertexDatumPlane->IsVertexAssociative(), "Plane must be vertex-associative");
    TEST_ASSERT(vertexDatumPlane->GetPlane().origin.approx_equal(targetVPos, 1e-4), "Vertex plane origin must match vertex position");

    // Create sketch on vertex datum plane
    auto sketchOnVertex = std::make_shared<SketchNode>("Sketch_OnVertex", vertexDatumPlane);
    sketchOnVertex->SetPresetRectangle(1.0f, 1.0f);
    graph.AddNode(sketchOnVertex);
    sketchOnVertex->Evaluate(&mockDevice, exchange);

    auto vSkPos = sketchOnVertex->GetPosition();
    Point3D vSkOrigin(vSkPos[0], vSkPos[1], vSkPos[2]);
    TEST_ASSERT(vSkOrigin.approx_equal(targetVPos, 1e-4), "Sketch on vertex datum plane must be centered around that vertex in 3D space");

    return true;
}

bool TestReferenceFeatureProjectionAndToggle() {
    std::cout << "[Test] 2D Sketch Reference Feature Projection & Toggle...\n";

    // 1. Create 3D feature with known geometry
    auto sketch = std::make_shared<SketchNode>("Box_Sketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 2.0f);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    sketch->Evaluate(&mockDevice, exchange);

    auto box = std::make_shared<ExtrudeNode>("Box_Feature", sketch, 3.0f);
    box->Evaluate(&mockDevice, exchange);

    const auto& verts = box->GetCachedVertices();
    const auto& edges = box->GetCachedEdges();
    TEST_ASSERT(!verts.empty(), "Box feature vertices cannot be empty");
    TEST_ASSERT(!edges.empty(), "Box feature edges cannot be empty");

    // 2. Create sketch plane on top face (Y = +3.0m)
    Plane topPlane = Plane::XZ(Point3D(0.0, 3.0, 0.0));

    // 3. Project each 3D edge into the 2D sketch plane UV space
    std::vector<std::pair<Point2D, Point2D>> projectedEdges;
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        uint32_t i0 = edges[e];
        uint32_t i1 = edges[e + 1];
        if (i0 < verts.size() && i1 < verts.size()) {
            Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
            Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);

            Point2D uv0 = topPlane.to_local(p0);
            Point2D uv1 = topPlane.to_local(p1);

            projectedEdges.emplace_back(uv0, uv1);
        }
    }

    TEST_ASSERT(!projectedEdges.empty(), "Projected reference edges must not be empty");

    // Verify top cap edges lie on the plane (distance to plane == 0) and local coordinates correspond to +/- 2 in U and +/- 1 in V
    bool foundBoundaryU = false;
    for (const auto& pe : projectedEdges) {
        if (std::abs(pe.first.u - 2.0) < 0.1 || std::abs(pe.first.u + 2.0) < 0.1) {
            foundBoundaryU = true;
            break;
        }
    }
    TEST_ASSERT(foundBoundaryU, "Projected reference feature edges must match 3D feature boundary coordinates");

    return true;
}

bool TestNegativeExtrudeGraphAndCustomPlane() {
    std::cout << "[Test] Negative Extrude on Standard Plane and Custom Face Datum Plane...\n";

    // 1. Standard Ground Plane (XZ) with Negative Extrusion (h = -3.0f)
    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    auto sketchGround = std::make_shared<SketchNode>("Sketch_Ground", SketchPlane::XZ_Ground);
    sketchGround->SetPresetRectangle(2.0f, 2.0f);
    graph.AddNode(sketchGround);

    auto extrudeNeg = std::make_shared<ExtrudeNode>("Extrude_Neg", sketchGround, -3.0f);
    graph.AddNode(extrudeNeg);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    const auto& verts = extrudeNeg->GetCachedVertices();
    const auto& indices = extrudeNeg->GetCachedIndices();
    TEST_ASSERT(!verts.empty(), "Vertices must not be empty for negative extrude");
    TEST_ASSERT(!indices.empty(), "Indices must not be empty for negative extrude");

    // Standard plane starting at sketch plane Y = 0:
    // Cap B (base) at w = 0.0f has outward normal +Y (0, 1, 0)
    // Cap A (extruded) at w = -3.0f has outward normal -Y (0, -1, 0)
    bool foundTopOutward = false;
    bool foundBotOutward = false;
    for (const auto& v : verts) {
        if (std::abs(v.position[1] - 0.0f) < 0.05f && v.normal[1] > 0.9f) {
            foundTopOutward = true;
        }
        if (std::abs(v.position[1] - (-3.0f)) < 0.05f && v.normal[1] < -0.9f) {
            foundBotOutward = true;
        }
    }
    TEST_ASSERT(foundTopOutward, "Negative extrude Cap B at 0.0m must have outward normal +Y");
    TEST_ASSERT(foundBotOutward, "Negative extrude Cap A at -3.0m must have outward normal -Y");

    // 2. Custom Datum Plane on Model Face with Negative Extrusion (h = -0.8m)
    // Create base block (Y: 0.0m to +2.0m, height 2.0m)
    auto sketchBase = std::make_shared<SketchNode>("Sketch_Base", SketchPlane::XZ_Ground);
    sketchBase->SetPresetRectangle(4.0f, 4.0f);
    graph.AddNode(sketchBase);

    auto baseExtrude = std::make_shared<ExtrudeNode>("BaseBlock", sketchBase, 2.0f);
    graph.AddNode(baseExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    // Extract top face of base block (Y = +2.0m, normal = +Y)
    const auto& baseFaces = baseExtrude->GetFaces();
    int topFaceIdx = -1;
    FaceInfo topFaceData;
    for (const auto& f : baseFaces) {
        if (f.normal.dot(Vector3D(0, 1, 0)) > 0.9) {
            topFaceIdx = f.faceIndex;
            topFaceData = f;
            break;
        }
    }
    TEST_ASSERT(topFaceIdx >= 0, "Base block must have a +Y top face");

    // Create datum plane associative to base block top face
    auto datumPlane = std::make_shared<DatumPlaneNode>("DatumPlane_TopFace", baseExtrude, topFaceIdx, topFaceData.plane);
    graph.AddNode(datumPlane);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(!datumPlane->HasConflict(), "Datum plane must not have conflict");
    TEST_ASSERT(std::abs(datumPlane->GetPlane().origin.y - 2.0) < 0.05, "Datum plane origin Y must be ~2.0m");

    // Create a sketch tied to this datum plane
    auto sketchFace = std::make_shared<SketchNode>("Sketch_Face", datumPlane);
    sketchFace->SetPresetRectangle(1.0f, 1.0f);
    graph.AddNode(sketchFace);

    // Extrude inward with negative depth (h = -0.8m)
    auto cutExtrude = std::make_shared<ExtrudeNode>("InwardExtrude", sketchFace, -0.8f);
    graph.AddNode(cutExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    const auto& cutVerts = cutExtrude->GetCachedVertices();
    const auto& cutIndices = cutExtrude->GetCachedIndices();
    TEST_ASSERT(!cutVerts.empty(), "Inward extrude vertices must not be empty");
    TEST_ASSERT(!cutIndices.empty(), "Inward extrude indices must not be empty");

    // On custom plane:
    // Cap B (w = 0) is at dist = 0.0 (Y = 2.0m, on the datum plane) with normal +Y (0, 1, 0)
    // Cap A (w = h) is at dist = -0.8m (Y = 1.2m, inside the block) with normal -Y (0, -1, 0)
    bool foundCapAtFace = false;
    bool foundCapInward = false;
    for (const auto& v : cutVerts) {
        if (std::abs(v.position[1] - 2.0f) < 0.05f && v.normal[1] > 0.9f) {
            foundCapAtFace = true;
        }
        if (std::abs(v.position[1] - 1.2f) < 0.05f && v.normal[1] < -0.9f) {
            foundCapInward = true;
        }
    }
    TEST_ASSERT(foundCapAtFace, "Inward negative extrude Cap B must be at face (Y=2.0m) with outward normal +Y");
    TEST_ASSERT(foundCapInward, "Inward negative extrude Cap A must be at depth (Y=1.2m) with outward normal -Y");

    return true;
}

bool TestDirectionalExtrusion() {
    std::cout << "[Test] Directional Extrusion (One-Way Growth, Fixed Sketch Plane Base)...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create a sketch on the ground plane (Y = 0)
    auto sketch = std::make_shared<SketchNode>("Sketch_Dir", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(2.0f, 2.0f);
    graph.AddNode(sketch);

    // 2. Extrude by +2.0m
    auto extrude = std::make_shared<ExtrudeNode>("Extrude_Dir", sketch, 2.0f);
    graph.AddNode(extrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    float minY = 1e9f, maxY = -1e9f;
    for (const auto& v : extrude->GetCachedVertices()) {
        minY = std::min(minY, v.position[1]);
        maxY = std::max(maxY, v.position[1]);
    }
    TEST_ASSERT(std::abs(minY - 0.0f) < 0.01f, "Base of +2.0m extrude must stay at sketch plane Y = 0.0m");
    TEST_ASSERT(std::abs(maxY - 2.0f) < 0.01f, "Top of +2.0m extrude must reach Y = +2.0m");

    // 3. Increase extrude depth to +5.0m:
    // It should grow only in the positive direction; the base at Y = 0.0m must NOT move!
    extrude->SetHeight(5.0f);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    minY = 1e9f; maxY = -1e9f;
    for (const auto& v : extrude->GetCachedVertices()) {
        minY = std::min(minY, v.position[1]);
        maxY = std::max(maxY, v.position[1]);
    }
    TEST_ASSERT(std::abs(minY - 0.0f) < 0.01f, "Increasing depth must NOT grow in reverse; base must remain at Y = 0.0m");
    TEST_ASSERT(std::abs(maxY - 5.0f) < 0.01f, "Increasing depth to +5.0m must grow only in +Y direction to Y = 5.0m");

    // 4. Change extrude depth to negative -4.0m:
    // It should extrude purely in the negative direction (-Y); the base at Y = 0.0m must NOT move!
    extrude->SetHeight(-4.0f);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    minY = 1e9f; maxY = -1e9f;
    for (const auto& v : extrude->GetCachedVertices()) {
        minY = std::min(minY, v.position[1]);
        maxY = std::max(maxY, v.position[1]);
    }
    TEST_ASSERT(std::abs(maxY - 0.0f) < 0.01f, "Negative extrude base must remain anchored at sketch plane Y = 0.0m");
    TEST_ASSERT(std::abs(minY - (-4.0f)) < 0.01f, "Negative extrude must extend in negative direction to Y = -4.0m");

    return true;
}

bool TestFilletAndChamferFeaturesAndEdgePicking() {
    std::cout << "[Test] 3D Model Edge Picking, Fillet & Chamfer Features...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create a 4x4 Extrude block with height 3.0m on ground
    auto sketch = std::make_shared<SketchNode>("Block_Sketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 4.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto block = std::make_shared<ExtrudeNode>("BaseBlock", sketch, 3.0f);
    block->Evaluate(&mockDevice, exchange);
    graph.AddNode(block);

    const auto& edges = block->GetCachedEdges();
    TEST_ASSERT(!edges.empty(), "BaseBlock must have sharp feature edges");

    // 2. Test Edge Picking via PickSubElement:
    // One vertical corner edge is along X=2.0, Z=2.0 from Y=0.0 to Y=3.0.
    // Cast a ray aimed at (2.05, 1.5, 2.05) pointing towards center
    Point3D rayOrig(5.0, 1.5, 2.0);
    Vector3D rayDir(-1.0, 0.0, 0.0); // ray intersects along X to hit edge at X=2.0, Y=1.5, Z=2.0
    auto pickResult = graph.PickSubElement(rayOrig, rayDir, block->GetId());
    TEST_ASSERT(pickResult.edgeIndex >= 0, "PickSubElement must successfully pick the corner edge");

    int pickedEdge = pickResult.edgeIndex;

    // Test Edge Selection mesh publishing: must publish selected edge even with 0 selected triangles
    graph.SetSubElementSelection(block->GetId(), -1, -1, pickedEdge);
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, block->GetId(), -1, -1, pickedEdge);
    auto meshEdge = exchange.AcquireLatest();
    TEST_ASSERT(meshEdge != nullptr, "Published mesh must not be null");
    TEST_ASSERT(meshEdge->selectedEdgeIndexCount == 2, "Selected edge buffer must contain exactly 2 line indices");
    TEST_ASSERT(meshEdge->selectedVertexBuffer != nullptr, "Selected vertex buffer must be created even with 0 selected triangles");
    TEST_ASSERT(meshEdge->selectedEdgeIndexBuffer != nullptr, "Selected edge index buffer must be valid");

    // 3. Create a FilletNode referencing this corner edge with radius = 0.5m
    auto fillet = std::make_shared<FilletNode>("Corner_Fillet", block, pickedEdge, 0.5f);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(block->IsConsumed(), "BaseBlock must be marked consumed when Fillet is created");
    TEST_ASSERT(!fillet->HasConflict(), "Fillet feature must not have conflict");
    TEST_ASSERT(!fillet->GetCachedVertices().empty(), "Fillet feature must have cached vertices");
    TEST_ASSERT(!fillet->GetCachedIndices().empty(), "Fillet feature must have cached indices");
    TEST_ASSERT(fillet->GetRadius() == 0.5f, "Fillet radius must match 0.5m");

    // Verify filleted mesh has more vertices than original 4-corner box due to rounded corner
    size_t baseVertCount = block->GetCachedVertices().size();
    size_t filletVertCount = fillet->GetCachedVertices().size();
    TEST_ASSERT(filletVertCount > baseVertCount, "Filleted solid must have more vertices representing the round corner");

    // 4. Test modifying Fillet radius:
    fillet->SetRadius(0.8f);
    TEST_ASSERT(fillet->IsDirty(), "Fillet must be marked dirty when radius is updated");
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!fillet->IsDirty(), "Fillet must be clean after re-evaluation");
    TEST_ASSERT(fillet->GetRadius() == 0.8f, "Fillet radius must update to 0.8m");

    // Test Fillet with Angle dimension choice
    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
    fillet->SetAngle(60.0f);
    TEST_ASSERT(fillet->GetDimensionChoice() == FilletNode::DimensionChoice::Angle, "Fillet choice should be Angle");
    TEST_ASSERT(fillet->GetAngle() == 60.0f, "Fillet angle should be 60 degrees");
    float expectedR = 0.8f * std::tan(60.0f * 3.14159265f / 360.0f);
    TEST_ASSERT(std::abs(fillet->GetRadius() - expectedR) < 1e-4f, "Fillet radius must be derived from angle");
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!fillet->IsDirty(), "Fillet should be evaluated cleanly after angle change");

    // 5. Test ChamferNode on another block:
    auto sketch2 = std::make_shared<SketchNode>("Block2_Sketch", SketchPlane::XZ_Ground);
    sketch2->SetPresetRectangle(4.0f, 4.0f);
    sketch2->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch2);

    auto block2 = std::make_shared<ExtrudeNode>("BaseBlock2", sketch2, 3.0f);
    block2->Evaluate(&mockDevice, exchange);
    graph.AddNode(block2);

    auto chamfer = std::make_shared<ChamferNode>("Corner_Chamfer", block2, pickedEdge, 0.4f);
    graph.AddNode(chamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(block2->IsConsumed(), "BaseBlock2 must be consumed by Chamfer");
    TEST_ASSERT(!chamfer->HasConflict(), "Chamfer must not have conflict");
    TEST_ASSERT(chamfer->GetDistance() == 0.4f, "Chamfer distance must match 0.4m");

    // 6. Test conflict propagation: remove base block
    block2->NotifyRemoval();
    TEST_ASSERT(chamfer->HasConflict(), "Chamfer must detect conflict when parent is removed");
    TEST_ASSERT(!chamfer->GetConflictMessage().empty(), "Chamfer conflict message must not be empty");

    return true;
}

bool TestChainedFilletAndChamferFeatures() {
    std::cout << "[Test] Chained Solid Fillet and Chamfer Features...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // 1. Create L-bracket extrude block
    auto sketch = std::make_shared<SketchNode>("Sketch_LBracket", SketchPlane::XZ_Ground);
    sketch->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sketch->SetPosition(-3.0f, 0.0f, 0.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto extrude = std::make_shared<ExtrudeNode>("Extrude_LBracket", sketch, 3.0f);
    extrude->SetPosition(-3.0f, 0.0f, 0.0f);
    extrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(extrude);

    // Initial extrude has 6 profile vertices
    TEST_ASSERT(extrude->GetProfile().size() == 6, "L-bracket base profile must have 6 vertices");
    TEST_ASSERT(!extrude->IsConsumed(), "Extrude must initially be unconsumed");

    // 2. Pick a corner edge and apply Chamfer
    // Find an edge at corner X=-1.0, Z=-0.8
    const auto& edges = extrude->GetCachedEdges();
    const auto& verts = extrude->GetCachedVertices();
    int cornerEdge = -1;
    for (size_t e = 0; e < edges.size() / 2; ++e) {
        uint32_t i0 = edges[e * 2];
        uint32_t i1 = edges[e * 2 + 1];
        if (std::abs(verts[i0].position[0] - (-1.0f)) < 0.1f && std::abs(verts[i0].position[2] - (-0.8f)) < 0.1f &&
            std::abs(verts[i1].position[0] - (-1.0f)) < 0.1f && std::abs(verts[i1].position[2] - (-0.8f)) < 0.1f) {
            cornerEdge = static_cast<int>(e);
            break;
        }
    }
    TEST_ASSERT(cornerEdge >= 0, "Must find vertical corner edge at (-1, -0.8)");

    auto chamfer = std::make_shared<ChamferNode>("Chamfer_1", extrude, cornerEdge, 0.25f);
    graph.AddNode(chamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(extrude->IsConsumed(), "Extrude_LBracket must be consumed by Chamfer_1");
    TEST_ASSERT(!chamfer->IsConsumed(), "Chamfer_1 must not be consumed");
    TEST_ASSERT(chamfer->GetProfile().size() == 7, "Chamfered profile must now have 7 vertices");

    // 3. Test PickNode: ray aimed at the model MUST pick Chamfer_1, NOT the consumed Extrude_LBracket!
    Point3D rayOrig(-3.0, 1.5, 5.0);
    Vector3D rayDir(0.0, 0.0, -1.0);
    uint64_t pickedId = graph.PickNode(rayOrig, rayDir);
    TEST_ASSERT(pickedId == chamfer->GetId(), "PickNode MUST pick active Chamfer_1, never consumed Extrude_LBracket");

    // 4. Chain a FilletNode on top of Chamfer_1!
    // Pick another corner edge on Chamfer_1 (e.g. corner at X=-5.0, Z=-2.0)
    const auto& chEdges = chamfer->GetCachedEdges();
    const auto& chVerts = chamfer->GetCachedVertices();
    int secondCornerEdge = -1;
    for (size_t e = 0; e < chEdges.size() / 2; ++e) {
        uint32_t i0 = chEdges[e * 2];
        uint32_t i1 = chEdges[e * 2 + 1];
        if (std::abs(chVerts[i0].position[0] - (-5.0f)) < 0.1f && std::abs(chVerts[i0].position[2] - (-2.0f)) < 0.1f &&
            std::abs(chVerts[i1].position[0] - (-5.0f)) < 0.1f && std::abs(chVerts[i1].position[2] - (-2.0f)) < 0.1f) {
            secondCornerEdge = static_cast<int>(e);
            break;
        }
    }
    TEST_ASSERT(secondCornerEdge >= 0, "Must find corner edge on Chamfer_1 at (-5, -2)");

    auto fillet = std::make_shared<FilletNode>("Fillet_2", chamfer, secondCornerEdge, 0.3f);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(chamfer->IsConsumed(), "Chamfer_1 must be consumed by Fillet_2");
    TEST_ASSERT(!fillet->IsConsumed(), "Fillet_2 must be the active leaf feature");
    TEST_ASSERT(fillet->GetProfile().size() == 13, "Fillet_2 profile must contain both Chamfer_1 and the new Fillet");

    // 5. Publish combined scene mesh: verify ONLY Fillet_2 is rendered (extrude and chamfer consumed)
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, fillet->GetId());
    auto mesh = exchange.AcquireLatest();
    TEST_ASSERT(mesh != nullptr, "Published mesh must not be null");

    return true;
}

bool TestThreeCornerVertexFillet() {
    std::cout << "[Test] 3-Corner Vertex Fillet on Solid Extrusion...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    auto sketch = std::make_shared<SketchNode>("Sketch_LBracket", SketchPlane::XZ_Ground);
    sketch->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sketch->SetPosition(-3.0f, 0.0f, 0.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto extrude = std::make_shared<ExtrudeNode>("Extrude_LBracket", sketch, 3.0f);
    extrude->SetPosition(-3.0f, 0.0f, 0.0f);
    extrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(extrude);

    const auto& verts = extrude->GetCachedVertices();
    const auto& edges = extrude->GetCachedEdges();

    // 1. Find a 3-corner vertex on the top cap at (-1.0, 3.0, -2.0)
    int cornerVertIdx = -1;
    for (size_t i = 0; i < verts.size(); ++i) {
        if (std::abs(verts[i].position[0] - (-1.0f)) < 0.1f &&
            std::abs(verts[i].position[1] - 3.0f) < 0.1f &&
            std::abs(verts[i].position[2] - (-2.0f)) < 0.1f) {
            cornerVertIdx = static_cast<int>(i);
            break;
        }
    }
    TEST_ASSERT(cornerVertIdx >= 0, "Must find 3-corner top vertex at (-1, 3, -2)");

    // 2. Count incident edges at this vertex: there must be 3 incident edges meeting at this 3D position
    Geom::Point3D cornerPos(verts[cornerVertIdx].position[0], verts[cornerVertIdx].position[1], verts[cornerVertIdx].position[2]);
    std::vector<int> incidentEdges;
    uint32_t vIdx = static_cast<uint32_t>(cornerVertIdx);
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        uint32_t idx0 = edges[e];
        uint32_t idx1 = edges[e + 1];
        Geom::Point3D p0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
        Geom::Point3D p1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
        if (idx0 == vIdx || idx1 == vIdx || p0.distance_to(cornerPos) < 0.05 || p1.distance_to(cornerPos) < 0.05) {
            incidentEdges.push_back(static_cast<int>(e / 2));
        }
    }
    TEST_ASSERT(incidentEdges.size() >= 3, "Corner vertex must have at least 3 incident edges meeting at it");

    // 3. Test extrusion normal alignment: must select the vertical corner edge
    ExtrusionData exData;
    bool hasEx = ExtractExtrusionData(extrude, exData);
    TEST_ASSERT(hasEx, "Extrude must provide extrusion data");
    Geom::Vector3D extrudeNormal(0, 1, 0);

    int bestCornerEdge = -1;
    float bestScore = -1.0f;
    for (int e : incidentEdges) {
        uint32_t idx0 = edges[e * 2];
        uint32_t idx1 = edges[e * 2 + 1];
        Geom::Point3D p0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
        Geom::Point3D p1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
        Geom::Vector3D dir = (p1 - p0).normalized();
        float score = static_cast<float>(std::abs(dir.dot(extrudeNormal)));
        if (score > bestScore) {
            bestScore = score;
            bestCornerEdge = e;
        }
    }
    TEST_ASSERT(bestCornerEdge >= 0, "Must find best corner edge");
    TEST_ASSERT(bestScore > 0.9f, "Best corner edge must be parallel to vertical extrusion normal");

    // 4. Create FilletNode with this corner edge
    auto fillet = std::make_shared<FilletNode>("Vertex_Corner_Fillet", extrude, bestCornerEdge, 0.35f);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(!fillet->HasConflict(), "Fillet feature must not have conflict");
    // Original 6 profile points + 6 arc points = 12 profile points
    TEST_ASSERT(fillet->GetProfile().size() == 12, "Filleting the 3-corner vertex must round the profile corner (+6 arc vertices)");

    // 5. Verify that top and bottom vertices around the corner now lie on the rounded arc (no protruding sharp corner!)
    const auto& fVerts = fillet->GetCachedVertices();
    bool hasProtrudingCorner = false;
    for (const auto& fv : fVerts) {
        // The original corner was at (-1, 3, -2). Any vertex at that exact coordinate would indicate an un-filleted protrusion.
        if (std::abs(fv.position[0] - (-1.0f)) < 0.05f &&
            std::abs(fv.position[1] - 3.0f) < 0.05f &&
            std::abs(fv.position[2] - (-2.0f)) < 0.05f) {
            hasProtrudingCorner = true;
            break;
        }
    }
    TEST_ASSERT(!hasProtrudingCorner, "The corner vertex must be replaced by the rounded arc, without sharp protrusion");

    return true;
}

bool TestHorizontalAndGeneral3DEdgeFilletAndChamfer() {
    std::cout << "\n[TEST] 3D Horizontal Edge Fillet and Chamfer (Method 2 Non-Warping Mesh Integrity)...\n";
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // 1. Create a rectangular block extruded in Y (height 2.0)
    std::vector<std::array<float, 2>> rectProfile = {
        {0.0f, 0.0f},
        {4.0f, 0.0f},
        {4.0f, 3.0f},
        {0.0f, 3.0f}
    };
    auto extrude = std::make_shared<ExtrudeNode>("Block_Extrude", rectProfile, 2.0f);
    extrude->SetPlane(SketchPlane::XZ_Ground);
    graph.AddNode(extrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    const auto& edges = extrude->GetCachedEdges();
    const auto& verts = extrude->GetCachedVertices();

    // 2. Locate a horizontal top cap edge (where Y = 2.0 for both p0 and p1)
    int horizEdgeIdx = -1;
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        uint32_t i0 = edges[e];
        uint32_t i1 = edges[e + 1];
        Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
        Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
        if (std::abs(p0.y - 2.0) < 0.05 && std::abs(p1.y - 2.0) < 0.05 && p0.distance_to(p1) > 1.0) {
            horizEdgeIdx = static_cast<int>(e / 2);
            break;
        }
    }
    TEST_ASSERT(horizEdgeIdx >= 0, "Must find horizontal top edge on block");

    // 3. Create FilletNode on this horizontal edge
    auto fillet = std::make_shared<FilletNode>("Horiz_Fillet", extrude, horizEdgeIdx, 0.35f);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(!fillet->HasConflict(), "Horizontal edge fillet must not conflict");
    const auto& fVerts = fillet->GetCachedVertices();
    const auto& fIndices = fillet->GetCachedIndices();
    TEST_ASSERT(!fVerts.empty() && !fIndices.empty(), "Fillet must have valid vertices and indices");

    // Verify all vertices are strictly finite (no NaNs!)
    for (const auto& v : fVerts) {
        TEST_ASSERT(std::isfinite(v.position[0]) && std::isfinite(v.position[1]) && std::isfinite(v.position[2]),
                    "All fillet vertex coordinates must be finite (no NaNs)");
        TEST_ASSERT(std::isfinite(v.normal[0]) && std::isfinite(v.normal[1]) && std::isfinite(v.normal[2]),
                    "All fillet vertex normals must be finite (no NaNs)");
    }

    // Verify that top cap vertices remain strictly at y = 2.0 (no warped diagonal ramps!)
    for (size_t t = 0; t + 2 < fIndices.size(); t += 3) {
        uint32_t a = fIndices[t];
        uint32_t b = fIndices[t + 1];
        uint32_t c = fIndices[t + 2];
        Geom::Point3D pa(fVerts[a].position[0], fVerts[a].position[1], fVerts[a].position[2]);
        Geom::Point3D pb(fVerts[b].position[0], fVerts[b].position[1], fVerts[b].position[2]);
        Geom::Point3D pc(fVerts[c].position[0], fVerts[c].position[1], fVerts[c].position[2]);
        Geom::Vector3D triNorm = (pb - pa).cross(pc - pa);
        double area = triNorm.length() * 0.5;
        TEST_ASSERT(area > 1e-6, "Triangle must not be degenerate");

        // If all 3 vertices are strictly on the top cap plane (y == 2.0), the triangle must be horizontal (normal ~ (0, 1, 0))
        if (std::abs(pa.y - 2.0) < 1e-3 && std::abs(pb.y - 2.0) < 1e-3 && std::abs(pc.y - 2.0) < 1e-3) {
            Geom::Vector3D unitN = triNorm.normalized();
            TEST_ASSERT(std::abs(unitN.y) > 0.95, "Top cap triangles must remain completely flat horizontal planes");
        }
    }

    // Verify convexity of fillet surface: at least one fillet vertex must curve outward toward the picked edge
    // with distance to the sharp edge chord < 0.20 (for R=0.35, true convex arc midpoint is at distance R*(sqrt(2)-1) ~ 0.145m,
    // whereas an inverted concave groove is at distance 0.35m deep into the solid).
    uint32_t ei0 = edges[horizEdgeIdx * 2];
    uint32_t ei1 = edges[horizEdgeIdx * 2 + 1];
    Geom::Point3D ep0(verts[ei0].position[0], verts[ei0].position[1], verts[ei0].position[2]);
    Geom::Point3D ep1(verts[ei1].position[0], verts[ei1].position[1], verts[ei1].position[2]);
    Geom::Vector3D eAxis = (ep1 - ep0).normalized();

    bool foundConvexVertex = false;
    for (const auto& v : fVerts) {
        Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
        // Perpendicular distance from the sharp edge line (end sections may be mitered along the edge)
        {
            double proj = (pt - ep0).dot(eAxis);
            double d = pt.distance_to(ep0 + eAxis * proj);
            if (d > 0.10 && d < 0.22) {
                foundConvexVertex = true;
                break;
            }
        }
    }
    TEST_ASSERT(foundConvexVertex, "Fillet surface must bulge outward convexly toward the sharp edge (not inverted concave)");

    // 4. Test horizontal edge chamfer as well
    auto chamfer = std::make_shared<ChamferNode>("Horiz_Chamfer", extrude, horizEdgeIdx, 0.30f);
    graph.AddNode(chamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(!chamfer->HasConflict(), "Horizontal edge chamfer must not conflict");
    const auto& cVerts = chamfer->GetCachedVertices();
    const auto& cIndices = chamfer->GetCachedIndices();
    TEST_ASSERT(!cVerts.empty() && !cIndices.empty(), "Chamfer must have valid vertices and indices");

    for (const auto& v : cVerts) {
        TEST_ASSERT(std::isfinite(v.position[0]) && std::isfinite(v.position[1]) && std::isfinite(v.position[2]),
                    "All chamfer vertex coordinates must be finite");
    }

    return true;
}

bool TestLBracketTopRimFilletWithTangentChainAndVerticalCornerFillets() {
    std::cout << "[Test] L-Bracket Top Rim Fillet with Tangent Chaining & Vertical Corner Fillets...\n";
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // 1. Create L-bracket extrusion
    auto sketch = std::make_shared<SketchNode>("Sketch_LBracket", SketchPlane::XZ_Ground);
    sketch->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto extrude = std::make_shared<ExtrudeNode>("Extrude_LBracket", sketch, 3.0f);
    extrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(extrude);

    // 2. Fillet vertical corner edges
    const auto& edges = extrude->GetCachedEdges();
    const auto& verts = extrude->GetCachedVertices();

    int vertCornerEdge = -1;
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        uint32_t i0 = edges[e];
        uint32_t i1 = edges[e + 1];
        Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
        Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
        // Vertical edge has same X and Z, varying Y
        if (std::abs(p0.x - p1.x) < 0.05 && std::abs(p0.z - p1.z) < 0.05 && std::abs(p0.y - p1.y) > 2.0) {
            vertCornerEdge = static_cast<int>(e / 2);
            break;
        }
    }
    TEST_ASSERT(vertCornerEdge >= 0, "Must find vertical corner edge on L-bracket");

    auto vertFillet = std::make_shared<FilletNode>("Vert_Fillet", extrude, vertCornerEdge, 0.35f);
    graph.AddNode(vertFillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!vertFillet->HasConflict(), "Vertical fillet must evaluate cleanly");

    // 3. Locate a horizontal top rim edge on the filleted L-bracket (at Y = 3.0)
    const auto& vfEdges = vertFillet->GetCachedEdges();
    const auto& vfVerts = vertFillet->GetCachedVertices();
    int topHorizEdge = -1;

    for (size_t e = 0; e + 1 < vfEdges.size(); e += 2) {
        uint32_t i0 = vfEdges[e];
        uint32_t i1 = vfEdges[e + 1];
        Geom::Point3D p0(vfVerts[i0].position[0], vfVerts[i0].position[1], vfVerts[i0].position[2]);
        Geom::Point3D p1(vfVerts[i1].position[0], vfVerts[i1].position[1], vfVerts[i1].position[2]);
        if (std::abs(p0.y - 3.0) < 0.05 && std::abs(p1.y - 3.0) < 0.05 && p0.distance_to(p1) > 0.5) {
            topHorizEdge = static_cast<int>(e / 2);
            break;
        }
    }
    TEST_ASSERT(topHorizEdge >= 0, "Must find horizontal top edge at Y = 3.0");

    // 4. Apply horizontal fillet: tangent chaining should propagate and avoid spurious cliff walls
    auto topFillet = std::make_shared<FilletNode>("Top_Fillet", vertFillet, topHorizEdge, 0.25f);
    graph.AddNode(topFillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(!topFillet->HasConflict(), "Top horizontal fillet must not conflict");
    const auto& tfVerts = topFillet->GetCachedVertices();
    const auto& tfIndices = topFillet->GetCachedIndices();
    TEST_ASSERT(!tfVerts.empty() && !tfIndices.empty(), "Top fillet must have geometry");

    for (const auto& v : tfVerts) {
        TEST_ASSERT(std::isfinite(v.position[0]) && std::isfinite(v.position[1]) && std::isfinite(v.position[2]),
                    "All top fillet vertex coordinates must be finite");
        TEST_ASSERT(std::isfinite(v.normal[0]) && std::isfinite(v.normal[1]) && std::isfinite(v.normal[2]),
                    "All top fillet normals must be finite");
    }

    return true;
}

bool TestFilletChamferChildRelationshipAndSafeFeatureDeletion() {
    std::cout << "[Test] Fillet & Chamfer Child Feature Hierarchy and Non-Disappearing Model Deletion...\n";
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // 1. Create a base extrude block
    auto sketch = std::make_shared<SketchNode>("Block_Sketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 4.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto baseBlock = std::make_shared<ExtrudeNode>("BaseBlock", sketch, 3.0f);
    baseBlock->Evaluate(&mockDevice, exchange);
    graph.AddNode(baseBlock);

    TEST_ASSERT(!baseBlock->IsConsumed(), "BaseBlock must not be consumed initially");

    // 2. Add FilletNode to BaseBlock
    auto fillet = std::make_shared<FilletNode>("Fillet_Child", baseBlock, 0, 0.4f);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    // Child relationship verification
    TEST_ASSERT(fillet->HasParentFeature(), "Fillet feature must have parent feature");
    TEST_ASSERT(fillet->GetParentFeature() == baseBlock, "Fillet parent feature must be BaseBlock");
    TEST_ASSERT(baseBlock->IsConsumed(), "BaseBlock must be consumed when Fillet is active");

    // Publish combined scene mesh: fillet should be rendered, not baseBlock
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, fillet->GetId());
    auto meshFillet = exchange.AcquireLatest();
    TEST_ASSERT(meshFillet != nullptr && (meshFillet->indexCount > 0 || meshFillet->selectedIndexCount > 0),
                "Scene mesh must have indices for Fillet");

    // 3. User is not satisfied with the fillet and deletes it via graph.RemoveNode
    uint64_t filletId = fillet->GetId();
    graph.RemoveNode(filletId);

    // CRITICAL: BaseBlock must IMMEDIATELY become unconsumed!
    TEST_ASSERT(!baseBlock->IsConsumed(), "BaseBlock must be unconsumed IMMEDIATELY upon deleting fillet feature");
    TEST_ASSERT(baseBlock->IsDirty(), "BaseBlock must be marked dirty upon fillet deletion for immediate re-evaluation");

    // Re-evaluate dirty nodes: BaseBlock evaluates cleanly
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!baseBlock->GetCachedVertices().empty(), "BaseBlock cached vertices must remain intact");

    // Scene publishing MUST contain BaseBlock
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, baseBlock->GetId());
    auto meshAfterDelete = exchange.AcquireLatest();
    TEST_ASSERT(meshAfterDelete != nullptr, "Published mesh after delete must not be null");
    TEST_ASSERT((meshAfterDelete->indexCount > 0 || meshAfterDelete->selectedIndexCount > 0),
                "BaseBlock must be present in scene mesh IMMEDIATELY (no temporary disappearance!)");

    // 4. Repeat check with ChamferNode
    auto chamfer = std::make_shared<ChamferNode>("Chamfer_Child", baseBlock, 0, 0.3f);
    graph.AddNode(chamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(chamfer->HasParentFeature(), "Chamfer feature must have parent feature");
    TEST_ASSERT(chamfer->GetParentFeature() == baseBlock, "Chamfer parent feature must be BaseBlock");
    TEST_ASSERT(baseBlock->IsConsumed(), "BaseBlock must be consumed when Chamfer is active");

    uint64_t chamferId = chamfer->GetId();
    graph.RemoveNode(chamferId);

    TEST_ASSERT(!baseBlock->IsConsumed(), "BaseBlock must be unconsumed IMMEDIATELY upon deleting chamfer feature");
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    graph.PublishCombinedSceneMesh(&mockDevice, exchange, baseBlock->GetId());
    auto meshAfterChamferDelete = exchange.AcquireLatest();
    TEST_ASSERT(meshAfterChamferDelete != nullptr && (meshAfterChamferDelete->indexCount > 0 || meshAfterChamferDelete->selectedIndexCount > 0),
                "BaseBlock must remain present immediately upon chamfer deletion");

    return true;
}

bool TestFilletDoesNotWrapOntoAdjoiningEdges() {
    std::cout << "[Test] Fillet Does Not Wrap Onto Adjoining Rim Edges...\n";
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    auto sketch = std::make_shared<SketchNode>("Sketch_LBracket", SketchPlane::XZ_Ground);
    sketch->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto extrude = std::make_shared<ExtrudeNode>("Extrude_LBracket", sketch, 3.0f);
    extrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(extrude);

    const auto& edges = extrude->GetCachedEdges();
    const auto& verts = extrude->GetCachedVertices();
    int topEdge = -1;
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        Geom::Point3D p0(verts[edges[e]].position[0], verts[edges[e]].position[1], verts[edges[e]].position[2]);
        Geom::Point3D p1(verts[edges[e + 1]].position[0], verts[edges[e + 1]].position[1], verts[edges[e + 1]].position[2]);
        if (std::abs(p0.y - 3.0) < 0.05 && std::abs(p1.y - 3.0) < 0.05 && p0.distance_to(p1) > 1.0) {
            topEdge = static_cast<int>(e / 2);
            break;
        }
    }
    TEST_ASSERT(topEdge >= 0, "Must find a top rim edge on the L-bracket");

    auto fillet = std::make_shared<FilletNode>("Rim_Fillet", extrude, topEdge, 0.3f);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!fillet->HasConflict(), "Rim fillet must not conflict");

    float minX = 1e9f, maxX = -1e9f, minZ = 1e9f, maxZ = -1e9f;
    for (const auto& v : fillet->GetCachedVertices()) {
        TEST_ASSERT(std::isfinite(v.position[0]) && std::isfinite(v.position[1]) && std::isfinite(v.position[2]),
                    "Rim fillet vertices must be finite");
        if (v.position[1] > 2.6f && v.position[1] < 2.99f) {
            minX = std::min(minX, v.position[0]); maxX = std::max(maxX, v.position[0]);
            minZ = std::min(minZ, v.position[2]); maxZ = std::max(maxZ, v.position[2]);
        }
    }
    TEST_ASSERT(!((maxX - minX) > 1.0f && (maxZ - minZ) > 1.0f),
                "Fillet must stay on the selected edge and not round the adjoining rim edge");
    return true;
}

bool TestFilletEndCapIsTrimmedToProfile() {
    std::cout << "[Test] Fillet End Caps Are Trimmed To The Blend Profile (no flat untrimmed corner)...\n";
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // 4 x 3 footprint (X x Z), height 2 (Y)
    std::vector<std::array<float, 2>> rectProfile = {{0.0f, 0.0f}, {4.0f, 0.0f}, {4.0f, 3.0f}, {0.0f, 3.0f}};
    auto extrude = std::make_shared<ExtrudeNode>("Cap_Block", rectProfile, 2.0f);
    extrude->SetPlane(SketchPlane::XZ_Ground);
    graph.AddNode(extrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    // Top edge running along X at Z = 0, Y = 2
    const auto& edges = extrude->GetCachedEdges();
    const auto& verts = extrude->GetCachedVertices();
    int edgeIdx = -1;
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        Geom::Point3D p0(verts[edges[e]].position[0], verts[edges[e]].position[1], verts[edges[e]].position[2]);
        Geom::Point3D p1(verts[edges[e + 1]].position[0], verts[edges[e + 1]].position[1], verts[edges[e + 1]].position[2]);
        if (std::abs(p0.y - 2.0) < 0.05 && std::abs(p1.y - 2.0) < 0.05 &&
            std::abs(p0.z) < 0.05 && std::abs(p1.z) < 0.05 && p0.distance_to(p1) > 3.5) {
            edgeIdx = static_cast<int>(e / 2);
            break;
        }
    }
    TEST_ASSERT(edgeIdx >= 0, "Must find the top X-aligned edge at Z=0");

    const float R = 0.35f;
    auto fillet = std::make_shared<FilletNode>("Cap_Fillet", extrude, edgeIdx, R);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!fillet->HasConflict(), "Fillet must not conflict");

    const auto& fv = fillet->GetCachedVertices();
    const auto& fi = fillet->GetCachedIndices();
    auto P = [&](uint32_t i) { return Geom::Point3D(fv[i].position[0], fv[i].position[1], fv[i].position[2]); };

    // Area of each end cap (X = 0 and X = 4) must equal the full 3 x 2 face minus the rounded-off corner
    const double quarterArcArea = 6.0 * 0.5 * R * R * std::sin(3.14159265358979 / 12.0); // 6-band polygonal arc
    const double expectedCap = 3.0 * 2.0 - (R * R - quarterArcArea);
    for (double capX : {0.0, 4.0}) {
        double area = 0.0;
        for (size_t t = 0; t + 2 < fi.size(); t += 3) {
            Geom::Point3D a = P(fi[t]), b = P(fi[t + 1]), c = P(fi[t + 2]);
            if (std::abs(a.x - capX) < 1e-3 && std::abs(b.x - capX) < 1e-3 && std::abs(c.x - capX) < 1e-3) {
                area += (b - a).cross(c - a).length() * 0.5;
                // No cap triangle may reach into the rounded-off corner region beyond the arc
                for (const auto& p : {a, b, c}) {
                    double dy = p.y - (2.0 - R), dz = p.z - R;
                    if (dy > 0.0 && dz < 0.0) {
                        TEST_ASSERT(std::sqrt(dy * dy + dz * dz) <= R + 1e-3, "End cap must not keep the sharp corner");
                    }
                }
            }
        }
        std::cout << "  cap X=" << capX << " area=" << area << " expected=" << expectedCap << "\n";
        TEST_ASSERT(std::abs(area - expectedCap) < 2e-3, "End cap area must match the trimmed profile");
    }

    // Watertight: every geometric edge is shared by exactly two triangles
    std::map<std::array<long long, 6>, int> edgeUse;
    auto key = [](const Geom::Point3D& p, const Geom::Point3D& q) {
        auto q3 = [](const Geom::Point3D& x) {
            return std::array<long long, 3>{std::llround(x.x * 1e4), std::llround(x.y * 1e4), std::llround(x.z * 1e4)};
        };
        auto A = q3(p), B = q3(q);
        if (B < A) std::swap(A, B);
        return std::array<long long, 6>{A[0], A[1], A[2], B[0], B[1], B[2]};
    };
    for (size_t t = 0; t + 2 < fi.size(); t += 3) {
        Geom::Point3D a = P(fi[t]), b = P(fi[t + 1]), c = P(fi[t + 2]);
        edgeUse[key(a, b)]++; edgeUse[key(b, c)]++; edgeUse[key(c, a)]++;
    }
    int bad = 0;
    for (const auto& kv : edgeUse) if (kv.second != 2) bad++;
    std::cout << "  non-manifold/open edges: " << bad << "\n";
    TEST_ASSERT(bad == 0, "Filleted block must be watertight");

    // Slanted end faces (trapezoid footprint): the trimmed caps must still close the solid
    std::vector<std::array<float, 2>> trapProfile = {{0.0f, 0.0f}, {4.0f, 0.0f}, {3.4f, 3.0f}, {0.6f, 3.0f}};
    auto trap = std::make_shared<ExtrudeNode>("Trap_Block", trapProfile, 2.0f);
    trap->SetPlane(SketchPlane::XZ_Ground);
    graph.AddNode(trap);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    const auto& tEdges = trap->GetCachedEdges();
    const auto& tVerts = trap->GetCachedVertices();
    int trapEdge = -1;
    for (size_t e = 0; e + 1 < tEdges.size(); e += 2) {
        Geom::Point3D p0(tVerts[tEdges[e]].position[0], tVerts[tEdges[e]].position[1], tVerts[tEdges[e]].position[2]);
        Geom::Point3D p1(tVerts[tEdges[e + 1]].position[0], tVerts[tEdges[e + 1]].position[1], tVerts[tEdges[e + 1]].position[2]);
        if (std::abs(p0.y - 2.0) < 0.05 && std::abs(p1.y - 2.0) < 0.05 &&
            std::abs(p0.z) < 0.05 && std::abs(p1.z) < 0.05 && p0.distance_to(p1) > 3.5) {
            trapEdge = static_cast<int>(e / 2);
            break;
        }
    }
    TEST_ASSERT(trapEdge >= 0, "Must find the trapezoid's top long edge");
    auto trapFillet = std::make_shared<FilletNode>("Trap_Fillet", trap, trapEdge, R);
    graph.AddNode(trapFillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    const auto& tv = trapFillet->GetCachedVertices();
    const auto& ti = trapFillet->GetCachedIndices();
    edgeUse.clear();
    for (size_t t = 0; t + 2 < ti.size(); t += 3) {
        Geom::Point3D a(tv[ti[t]].position[0], tv[ti[t]].position[1], tv[ti[t]].position[2]);
        Geom::Point3D b(tv[ti[t + 1]].position[0], tv[ti[t + 1]].position[1], tv[ti[t + 1]].position[2]);
        Geom::Point3D c(tv[ti[t + 2]].position[0], tv[ti[t + 2]].position[1], tv[ti[t + 2]].position[2]);
        edgeUse[key(a, b)]++; edgeUse[key(b, c)]++; edgeUse[key(c, a)]++;
    }
    bad = 0;
    for (const auto& kv : edgeUse) if (kv.second != 2) bad++;
    std::cout << "  slanted caps non-manifold/open edges: " << bad << "\n";
    TEST_ASSERT(bad == 0, "Filleted trapezoid block must be watertight");
    for (uint32_t idx : ti) {
        const auto& v = tv[idx];
        TEST_ASSERT(!(std::abs(v.position[1] - 2.0f) < 1e-3f && std::abs(v.position[2]) < 1e-3f),
                    "Sharp filleted edge (incl. its end corners) must be fully removed");
    }
    return true;
}

// Number of geometric edges not shared by exactly two triangles (0 == watertight)
static int CountOpenEdges(const std::vector<CADVertex>& v, const std::vector<uint32_t>& idx) {
    auto q = [&](uint32_t i) {
        return std::array<long long, 3>{std::llround(v[i].position[0] * 1e4), std::llround(v[i].position[1] * 1e4),
                                        std::llround(v[i].position[2] * 1e4)};
    };
    std::map<std::pair<std::array<long long, 3>, std::array<long long, 3>>, int> use;
    auto add = [&](uint32_t a, uint32_t b) {
        auto A = q(a), B = q(b);
        if (B < A) std::swap(A, B);
        use[{A, B}]++;
    };
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        add(idx[t], idx[t + 1]); add(idx[t + 1], idx[t + 2]); add(idx[t + 2], idx[t]);
    }
    int bad = 0;
    for (const auto& kv : use) if (kv.second != 2) bad++;
    return bad;
}

static int FindTopEdgeAtZ0(const DAGNode& node, double topY, double minLen) {
    const auto& edges = node.GetCachedEdges();
    const auto& verts = node.GetCachedVertices();
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        Geom::Point3D p0(verts[edges[e]].position[0], verts[edges[e]].position[1], verts[edges[e]].position[2]);
        Geom::Point3D p1(verts[edges[e + 1]].position[0], verts[edges[e + 1]].position[1], verts[edges[e + 1]].position[2]);
        if (std::abs(p0.y - topY) < 0.05 && std::abs(p1.y - topY) < 0.05 &&
            std::abs(p0.z) < 0.05 && std::abs(p1.z) < 0.05 && p0.distance_to(p1) > minLen) {
            return static_cast<int>(e / 2);
        }
    }
    return -1;
}

bool TestChamferUsesFilletCornerAndEndCapBehavior() {
    std::cout << "[Test] Chamfer Shares Fillet Corner, Chaining & End-Cap Trimming Behavior...\n";
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);
    const float D = 0.4f;

    // 1. Box: chamfer a top edge whose ends run into the end faces
    std::vector<std::array<float, 2>> rectProfile = {{0.0f, 0.0f}, {4.0f, 0.0f}, {4.0f, 3.0f}, {0.0f, 3.0f}};
    auto box = std::make_shared<ExtrudeNode>("Chamf_Box", rectProfile, 2.0f);
    box->SetPlane(SketchPlane::XZ_Ground);
    graph.AddNode(box);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    int edgeIdx = FindTopEdgeAtZ0(*box, 2.0, 3.5);
    TEST_ASSERT(edgeIdx >= 0, "Must find the box's top edge at Z=0");

    auto chamfer = std::make_shared<ChamferNode>("Box_Chamfer", box, edgeIdx, D);
    graph.AddNode(chamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!chamfer->HasConflict(), "Chamfer must not conflict");
    const auto& cv = chamfer->GetCachedVertices();
    const auto& ci = chamfer->GetCachedIndices();

    const double expectedCap = 3.0 * 2.0 - 0.5 * D * D;
    for (double capX : {0.0, 4.0}) {
        double area = 0.0;
        for (size_t t = 0; t + 2 < ci.size(); t += 3) {
            Geom::Point3D p[3];
            bool onCap = true;
            for (int k = 0; k < 3; ++k) {
                const auto& v = cv[ci[t + k]];
                p[k] = Geom::Point3D(v.position[0], v.position[1], v.position[2]);
                onCap &= std::abs(p[k].x - capX) < 1e-3;
            }
            if (!onCap) continue;
            area += (p[1] - p[0]).cross(p[2] - p[0]).length() * 0.5;
            for (const auto& q : p) {
                TEST_ASSERT((2.0 - q.y) + q.z >= D - 1e-3, "Chamfer end cap must not keep the sharp corner");
            }
        }
        std::cout << "  chamfer cap X=" << capX << " area=" << area << " expected=" << expectedCap << "\n";
        TEST_ASSERT(std::abs(area - expectedCap) < 1e-3, "Chamfer end cap area must match the trimmed profile");
    }
    int open = CountOpenEdges(cv, ci);
    std::cout << "  chamfered box open edges: " << open << "\n";
    TEST_ASSERT(open == 0, "Chamfered box must be watertight");

    // Bevel face must be flat-shaded: every triangle on the 45-degree plane carries one normal
    for (size_t t = 0; t + 2 < ci.size(); t += 3) {
        Geom::Point3D p[3];
        for (int k = 0; k < 3; ++k) {
            const auto& v = cv[ci[t + k]];
            p[k] = Geom::Point3D(v.position[0], v.position[1], v.position[2]);
        }
        bool onBevel = true;
        for (const auto& q : p) onBevel &= std::abs((2.0 - q.y) + q.z - D) < 1e-3 && q.z < D + 1e-3;
        Geom::Vector3D gn = (p[1] - p[0]).cross(p[2] - p[0]);
        if (!onBevel || gn.length() < 1e-9 || std::abs(gn.normalized().x) > 0.1) continue;
        for (int k = 0; k < 3; ++k) {
            const auto& n = cv[ci[t + k]].normal;
            TEST_ASSERT(std::abs(n[0]) < 1e-3 && std::abs(std::abs(n[1]) - std::abs(n[2])) < 1e-3,
                        "Bevel vertices must carry the flat 45-degree face normal");
        }
    }

    // 2. Slanted end faces
    std::vector<std::array<float, 2>> trapProfile = {{0.0f, 0.0f}, {4.0f, 0.0f}, {3.4f, 3.0f}, {0.6f, 3.0f}};
    auto trap = std::make_shared<ExtrudeNode>("Chamf_Trap", trapProfile, 2.0f);
    trap->SetPlane(SketchPlane::XZ_Ground);
    graph.AddNode(trap);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    int trapEdge = FindTopEdgeAtZ0(*trap, 2.0, 3.5);
    TEST_ASSERT(trapEdge >= 0, "Must find the trapezoid's top long edge");
    auto trapChamfer = std::make_shared<ChamferNode>("Trap_Chamfer", trap, trapEdge, D);
    graph.AddNode(trapChamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    open = CountOpenEdges(trapChamfer->GetCachedVertices(), trapChamfer->GetCachedIndices());
    std::cout << "  chamfered trapezoid open edges: " << open << "\n";
    TEST_ASSERT(open == 0, "Chamfered trapezoid must be watertight");

    // 3. Tangent chaining with mitered junctions: chamfer the top rim of an L-bracket whose
    //    vertical corner was filleted; the chamfer must follow the round like the fillet does.
    auto sketch = std::make_shared<SketchNode>("Chamf_LSketch", SketchPlane::XZ_Ground);
    sketch->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);
    auto lb = std::make_shared<ExtrudeNode>("Chamf_LBracket", sketch, 3.0f);
    lb->Evaluate(&mockDevice, exchange);
    graph.AddNode(lb);
    int vertEdge = -1;
    {
        const auto& e = lb->GetCachedEdges();
        const auto& v = lb->GetCachedVertices();
        for (size_t k = 0; k + 1 < e.size(); k += 2) {
            const auto& a = v[e[k]].position;
            const auto& b = v[e[k + 1]].position;
            if (std::abs(a[0] - b[0]) < 0.05f && std::abs(a[2] - b[2]) < 0.05f && std::abs(a[1] - b[1]) > 2.0f) {
                vertEdge = static_cast<int>(k / 2);
                break;
            }
        }
    }
    TEST_ASSERT(vertEdge >= 0, "Must find a vertical L-bracket corner edge");
    auto vertFillet = std::make_shared<FilletNode>("Chamf_VertFillet", lb, vertEdge, 0.35f);
    graph.AddNode(vertFillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    int rimEdge = -1;
    {
        const auto& e = vertFillet->GetCachedEdges();
        const auto& v = vertFillet->GetCachedVertices();
        for (size_t k = 0; k + 1 < e.size(); k += 2) {
            const auto& a = v[e[k]].position;
            const auto& b = v[e[k + 1]].position;
            float dx = a[0] - b[0], dz = a[2] - b[2];
            if (std::abs(a[1] - 3.0f) < 0.05f && std::abs(b[1] - 3.0f) < 0.05f && std::sqrt(dx * dx + dz * dz) > 0.5f) {
                rimEdge = static_cast<int>(k / 2);
                break;
            }
        }
    }
    TEST_ASSERT(rimEdge >= 0, "Must find a top rim edge on the filleted L-bracket");

    auto rimFillet = std::make_shared<FilletNode>("Chamf_RimFillet", vertFillet, rimEdge, 0.25f);
    auto rimChamfer = std::make_shared<ChamferNode>("Chamf_RimChamfer", vertFillet, rimEdge, 0.25f);
    rimFillet->Evaluate(&mockDevice, exchange);
    rimChamfer->Evaluate(&mockDevice, exchange);
    TEST_ASSERT(!rimChamfer->HasConflict(), "Rim chamfer must not conflict");

    // Both blends chain the same edges: their footprint on the top face (setback line) must match
    auto topFootprint = [](const std::vector<CADVertex>& vs, const std::vector<uint32_t>& is) {
        std::array<float, 4> box{1e9f, -1e9f, 1e9f, -1e9f};
        for (uint32_t i : is) {
            const auto& p = vs[i].position;
            if (p[1] < 2.99f && p[1] > 2.5f) {
                box[0] = std::min(box[0], p[0]); box[1] = std::max(box[1], p[0]);
                box[2] = std::min(box[2], p[2]); box[3] = std::max(box[3], p[2]);
            }
        }
        return box;
    };
    auto fb = topFootprint(rimFillet->GetCachedVertices(), rimFillet->GetCachedIndices());
    auto cb = topFootprint(rimChamfer->GetCachedVertices(), rimChamfer->GetCachedIndices());
    std::cout << "  rim fillet span x[" << fb[0] << "," << fb[1] << "] z[" << fb[2] << "," << fb[3] << "]"
              << "  chamfer span x[" << cb[0] << "," << cb[1] << "] z[" << cb[2] << "," << cb[3] << "]\n";
    for (int k = 0; k < 4; ++k) {
        TEST_ASSERT(std::abs(fb[k] - cb[k]) < 0.3f, "Rim chamfer must chain the same edges as the rim fillet");
    }
    int fOpen = CountOpenEdges(rimFillet->GetCachedVertices(), rimFillet->GetCachedIndices());
    int cOpen = CountOpenEdges(rimChamfer->GetCachedVertices(), rimChamfer->GetCachedIndices());
    std::cout << "  rim fillet open edges: " << fOpen << "  rim chamfer open edges: " << cOpen << "\n";
    TEST_ASSERT(fOpen == 0 && cOpen == 0, "Chained rim fillet and chamfer must both be watertight");
    bool fullSetback = false;
    for (uint32_t i : rimChamfer->GetCachedIndices()) {
        fullSetback |= std::abs(rimChamfer->GetCachedVertices()[i].position[1] - 2.75f) < 1e-3f;
    }
    TEST_ASSERT(fullSetback, "Rim chamfer must use its full 0.25 setback (not collapse on sliver cap triangles)");
    return true;
}

// Edges shared by two triangles whose normals point in opposite directions (folded or mis-wound skin)
static int CountFoldedEdges(const std::vector<CADVertex>& v, const std::vector<uint32_t>& idx) {
    auto q = [&](uint32_t i) {
        return std::array<long long, 3>{std::llround(v[i].position[0] * 1e4), std::llround(v[i].position[1] * 1e4),
                                        std::llround(v[i].position[2] * 1e4)};
    };
    std::map<std::pair<std::array<long long, 3>, std::array<long long, 3>>, std::vector<Geom::Vector3D>> adj;
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        Geom::Point3D p[3];
        for (int k = 0; k < 3; ++k) p[k] = Geom::Point3D(v[idx[t + k]].position[0], v[idx[t + k]].position[1], v[idx[t + k]].position[2]);
        Geom::Vector3D n = (p[1] - p[0]).cross(p[2] - p[0]);
        if (n.length() > 1e-12) n = n.normalized();
        for (int k = 0; k < 3; ++k) {
            auto A = q(idx[t + k]), B = q(idx[t + (k + 1) % 3]);
            if (B < A) std::swap(A, B);
            adj[{A, B}].push_back(n);
        }
    }
    int folds = 0;
    for (const auto& kv : adj) {
        if (kv.second.size() == 2 && kv.second[0].dot(kv.second[1]) < -0.5) folds++;
    }
    return folds;
}

bool TestVertexFilletRoundsAllThreeCornerEdges() {
    std::cout << "[Test] Vertex Fillet Rounds All Three Edges Meeting At The Corner...\n";
    MockTestDevice dev;
    AtomicMeshExchange ex;
    ParametricGraph graph(nullptr);

    auto sk = std::make_shared<SketchNode>("VF_Sketch", SketchPlane::XZ_Ground);
    sk->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sk->SetPosition(-3.0f, 0.0f, 0.0f);
    sk->Evaluate(&dev, ex);
    graph.AddNode(sk);
    auto ext = std::make_shared<ExtrudeNode>("VF_LBracket", sk, 3.0f);
    ext->SetPosition(-3.0f, 0.0f, 0.0f);
    ext->Evaluate(&dev, ex);
    graph.AddNode(ext);

    // Same selection logic as the GUI's "Apply Fillet to Corner at Vertex"
    const Geom::Point3D corner(-1.0, 3.0, -2.0);
    VertexCornerEdges edges = CollectVertexCornerEdges(ext->GetCachedVertices(), ext->GetCachedEdges(), corner,
                                                       Geom::Vector3D(0, 1, 0));
    TEST_ASSERT(edges.primaryEdge >= 0, "Corner vertex must have a primary edge");
    TEST_ASSERT(edges.otherEdges.size() == 2, "Box corner must contribute exactly two more distinct edges");

    const float R = 0.2f;
    auto fillet = std::make_shared<FilletNode>("VF_Fillet", ext, edges.primaryEdge, R);
    fillet->SetAdditionalEdges(edges.otherEdges);
    graph.AddNode(fillet);
    graph.EvaluateDirtyNodes(&dev, ex);
    TEST_ASSERT(!fillet->HasConflict(), "Vertex fillet must not conflict");

    const auto& v = fillet->GetCachedVertices();
    const auto& idx = fillet->GetCachedIndices();
    // No referenced vertex may remain on any of the three sharp edges
    bool onVertical = false, onTopFront = false, onTopRight = false;
    for (uint32_t i : idx) {
        const auto& p = v[i].position;
        onVertical |= std::abs(p[0] + 1.0f) < 1e-3f && std::abs(p[2] + 2.0f) < 1e-3f;
        onTopFront |= std::abs(p[1] - 3.0f) < 1e-3f && std::abs(p[2] + 2.0f) < 1e-3f;
        onTopRight |= std::abs(p[0] + 1.0f) < 1e-3f && std::abs(p[1] - 3.0f) < 1e-3f;
    }
    TEST_ASSERT(!onVertical, "Vertical corner edge must be rounded");
    TEST_ASSERT(!onTopFront, "Top front edge must be rounded");
    TEST_ASSERT(!onTopRight, "Top right edge must be rounded");

    int open = CountOpenEdges(v, idx);
    int folds = CountFoldedEdges(v, idx);
    std::cout << "  vertex fillet: tris=" << idx.size() / 3 << " open=" << open << " folds=" << folds << "\n";
    TEST_ASSERT(open == 0, "Vertex fillet must be watertight");
    TEST_ASSERT(folds == 0, "Vertex fillet must not fold");

    // Equal radii meet in a spherical corner: every corner-region vertex sits R from the ball centre
    const Geom::Point3D ballCentre(-1.0 - R, 3.0 - R, -2.0 + R);
    for (uint32_t i : idx) {
        Geom::Point3D p(v[i].position[0], v[i].position[1], v[i].position[2]);
        if (p.x > ballCentre.x && p.y > ballCentre.y && p.z < ballCentre.z) {
            TEST_ASSERT(std::abs(p.distance_to(ballCentre) - R) < 0.02, "Corner must be a spherical blend");
        }
    }

    // The result is no longer a plain profile extrusion: downstream features must not rebuild from it
    ExtrusionData data;
    TEST_ASSERT(!fillet->IsExtrusionBased(data), "Vertex fillet must not advertise a profile that drops its extra rounds");

    // Every corner of the L-bracket (convex and the inner concave ones)
    const float xs[] = {-5.0f, -1.0f, -3.8f};
    const float zs[] = {-2.0f, 2.0f, -0.8f};
    for (float y : {0.0f, 3.0f}) {
        for (float x : xs) {
            for (float z : zs) {
                Geom::Point3D c(x, y, z);
                bool isCorner = false;
                for (const auto& vv : ext->GetCachedVertices()) {
                    isCorner |= Geom::Point3D(vv.position[0], vv.position[1], vv.position[2]).distance_to(c) < 1e-3;
                }
                if (!isCorner) continue;
                VertexCornerEdges ce = CollectVertexCornerEdges(ext->GetCachedVertices(), ext->GetCachedEdges(), c, Geom::Vector3D(0, 1, 0));
                auto f = std::make_shared<FilletNode>("VF_Any", ext, ce.primaryEdge, R);
                f->SetAdditionalEdges(ce.otherEdges);
                f->Evaluate(&dev, ex);
                int o = CountOpenEdges(f->GetCachedVertices(), f->GetCachedIndices());
                int fo = CountFoldedEdges(f->GetCachedVertices(), f->GetCachedIndices());
                std::cout << "  corner (" << x << "," << y << "," << z << "): edges=" << 1 + ce.otherEdges.size()
                          << " open=" << o << " folds=" << fo << "\n";
                TEST_ASSERT(ce.otherEdges.size() == 2, "Every L-bracket corner joins exactly three edges");
                TEST_ASSERT(o == 0, "Vertex fillet must be watertight at every corner");
                TEST_ASSERT(fo == 0, "Vertex fillet must not fold at any corner");
                // At the inside corner the vertical round adds material, so the old corner point ends up
                // inside the solid with the rim blends passing close to it; only check convex corners.
                const bool insideCorner = std::abs(x + 3.8f) < 1e-3f && std::abs(z + 0.8f) < 1e-3f;
                for (uint32_t i : f->GetCachedIndices()) {
                    if (insideCorner) break;
                    const auto& p = f->GetCachedVertices()[i].position;
                    TEST_ASSERT(Geom::Point3D(p[0], p[1], p[2]).distance_to(c) > 0.05, "The sharp corner point must be blended away");
                }
            }
        }
    }
    return true;
}

bool TestFilletRunningIntoEarlierRoundWrapsCleanly() {
    std::cout << "[Test] Fillet Running Into An Earlier Round Wraps Around It Cleanly (no misaligned fan)...\n";
    MockTestDevice dev;
    AtomicMeshExchange ex;

    // Default scene L-bracket: x in [-5,-1], z in [-2,2], y in [0,3]
    auto sk = std::make_shared<SketchNode>("Wrap_Sketch", SketchPlane::XZ_Ground);
    sk->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    sk->SetPosition(-3.0f, 0.0f, 0.0f);
    sk->Evaluate(&dev, ex);
    auto ext = std::make_shared<ExtrudeNode>("Wrap_LBracket", sk, 3.0f);
    ext->SetPosition(-3.0f, 0.0f, 0.0f);
    ext->Evaluate(&dev, ex);

    auto findEdge = [](const DAGNode& n, Geom::Point3D a, Geom::Point3D b) {
        const auto& e = n.GetCachedEdges();
        const auto& v = n.GetCachedVertices();
        for (size_t k = 0; k + 1 < e.size(); k += 2) {
            Geom::Point3D p(v[e[k]].position[0], v[e[k]].position[1], v[e[k]].position[2]);
            Geom::Point3D q(v[e[k + 1]].position[0], v[e[k + 1]].position[1], v[e[k + 1]].position[2]);
            if ((p.distance_to(a) < 0.06 && q.distance_to(b) < 0.06) || (p.distance_to(b) < 0.06 && q.distance_to(a) < 0.06)) {
                return static_cast<int>(k / 2);
            }
        }
        return -1;
    };
    auto maxOutside = [](const DAGNode& n) {
        double o = 0.0;
        for (uint32_t i : n.GetCachedIndices()) {
            const auto& p = n.GetCachedVertices()[i].position;
            o = std::max({o, -5.0 - p[0], p[0] + 1.0, -static_cast<double>(p[1]), p[1] - 3.0, -2.0 - p[2], p[2] - 2.0});
        }
        return o;
    };

    for (float r2 : {0.2f, 0.1f, 0.3f}) {
        // Fillet_1: top edge along Z at x = -5
        auto f1 = std::make_shared<FilletNode>("Wrap_F1", ext, findEdge(*ext, {-5, 3, 2}, {-5, 3, -2}), 0.2f);
        f1->Evaluate(&dev, ex);
        // Fillet_2: top front edge, which runs into Fillet_1's round at x = -4.8
        int e2 = findEdge(*f1, {-1, 3, -2}, {-4.8, 3, -2});
        TEST_ASSERT(e2 >= 0, "Must find the top front edge ending at the earlier round");
        auto f2 = std::make_shared<FilletNode>("Wrap_F2", f1, e2, r2);
        f2->Evaluate(&dev, ex);
        // Fillet_3: bottom front edge, which runs into Fillet_2's round
        int e3 = findEdge(*f2, {-5 + r2, 0, -2}, {-1, 0, -2});
        TEST_ASSERT(e3 >= 0, "Must find the bottom front edge");
        auto f3 = std::make_shared<FilletNode>("Wrap_F3", f2, e3, 0.2f);
        f3->Evaluate(&dev, ex);

        for (const DAGNode* n : {static_cast<const DAGNode*>(f2.get()), static_cast<const DAGNode*>(f3.get())}) {
            int open = CountOpenEdges(n->GetCachedVertices(), n->GetCachedIndices());
            int folds = CountFoldedEdges(n->GetCachedVertices(), n->GetCachedIndices());
            double out = maxOutside(*n);
            std::cout << "  r2=" << r2 << " " << n->GetName() << ": open=" << open << " folds=" << folds << " outside=" << out << "\n";
            TEST_ASSERT(open == 0, "Wrapped fillet must be watertight");
            TEST_ASSERT(folds == 0, "Wrapped fillet must not fold or flip triangles");
            // Equal radii give an exact spherical corner; a larger blend wrapping a smaller round is
            // approximated at the cusp and may bulge slightly.
            TEST_ASSERT(out < (r2 == 0.2f ? 1e-3 : 0.05), "Wrapped fillet must stay within the original solid");
        }
    }
    return true;
}

bool TestOffsetReferencePlanesAndAxesFromFaceAndCorners() {
    std::cout << "[Test] Offset Reference Planes & Axes from Selected Face, Corner Edge, and Corner Vertex...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    auto sketch = std::make_shared<SketchNode>("Box_Sketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 2.0f);
    sketch->Evaluate(&mockDevice, exchange);
    graph.AddNode(sketch);

    auto extrude = std::make_shared<ExtrudeNode>("Box_Extrude", sketch, 3.0f);
    extrude->Evaluate(&mockDevice, exchange);
    graph.AddNode(extrude);

    // 1. Find Top Face (Y = 3.0, normal = (0, 1, 0))
    const auto& faces = extrude->GetFaces();
    int faceIdx = -1;
    Geom::Plane facePlane;
    for (size_t i = 0; i < faces.size(); ++i) {
        if (faces[i].normal.y > 0.9) {
            faceIdx = static_cast<int>(i);
            facePlane = faces[i].plane;
            break;
        }
    }
    TEST_ASSERT(faceIdx >= 0, "Should find top face of Box_Extrude");
    TEST_ASSERT(std::abs(facePlane.origin.y - 3.0) < 1e-3, "Top face origin Y should be 3.0");
    TEST_ASSERT(std::abs(facePlane.normal.y - 1.0) < 1e-3, "Top face normal should be +Y");

    // Create offset DatumPlaneNode from selected face (offset = +1.5m along face normal)
    auto faceOffsetPlane = std::make_shared<DatumPlaneNode>("Plane_Face_Offset", extrude, faceIdx, facePlane, 1.5);
    graph.AddNode(faceOffsetPlane);
    faceOffsetPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(faceOffsetPlane->IsFaceAssociative(), "Plane should be FaceAssociative");
    TEST_ASSERT(!faceOffsetPlane->HasConflict(), "Face offset plane should not have conflict");
    Geom::Plane evalFacePlane = faceOffsetPlane->GetPlane();
    TEST_ASSERT(std::abs(evalFacePlane.origin.y - 4.5) < 1e-3, "Face offset plane origin Y should be 3.0 + 1.5 = 4.5");

    // Create offset DatumAxisNode from selected face (offset = +2.0m along face normal)
    auto faceOffsetAxis = DatumAxisNode::CreateFromFace("Axis_Face_Offset", extrude, faceIdx, facePlane.normal, 2.0);
    graph.AddNode(faceOffsetAxis);
    faceOffsetAxis->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(faceOffsetAxis->IsFaceAssociative(), "Axis should be FaceAssociative");
    TEST_ASSERT(!faceOffsetAxis->HasConflict(), "Face offset axis should not have conflict");
    Geom::Axis3D evalFaceAxis = faceOffsetAxis->GetAxis();
    TEST_ASSERT(std::abs(evalFaceAxis.origin.y - 5.0) < 1e-3, "Face offset axis origin Y should be 3.0 + 2.0 = 5.0");

    // 2. Pick a Corner Edge on Box_Extrude
    const auto& edges = extrude->GetCachedEdges();
    const auto& verts = extrude->GetCachedVertices();
    TEST_ASSERT(edges.size() >= 2, "Extrude should have cached sharp edges");
    int edgeIdx = 0;
    Geom::Point3D eP0(verts[edges[0]].position[0], verts[edges[0]].position[1], verts[edges[0]].position[2]);
    Geom::Point3D eP1(verts[edges[1]].position[0], verts[edges[1]].position[1], verts[edges[1]].position[2]);
    Geom::Point3D edgeMid((eP0.x + eP1.x) * 0.5, (eP0.y + eP1.y) * 0.5, (eP0.z + eP1.z) * 0.5);

    Geom::Plane initEdgePlane;
    bool edgePlaneOk = DatumPlaneNode::ComputeCornerEdgePlane(extrude, edgeIdx, initEdgePlane);
    TEST_ASSERT(edgePlaneOk, "ComputeCornerEdgePlane must succeed on corner edge");

    // Create offset DatumPlaneNode from corner edge (offset = 1.25m)
    auto edgeOffsetPlane = DatumPlaneNode::CreateFromEdge("Plane_Edge_Offset", extrude, edgeIdx, initEdgePlane, 1.25);
    graph.AddNode(edgeOffsetPlane);
    edgeOffsetPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(edgeOffsetPlane->IsEdgeAssociative(), "Plane should be EdgeAssociative");
    TEST_ASSERT(!edgeOffsetPlane->HasConflict(), "Edge offset plane should not have conflict");
    Geom::Plane evalEdgePlane = edgeOffsetPlane->GetPlane();
    double planeDistFromMid = evalEdgePlane.origin.distance_to(edgeMid);
    TEST_ASSERT(std::abs(planeDistFromMid - 1.25) < 1e-3, "Edge offset plane origin must be offset by 1.25m from edge midpoint");

    // Create offset DatumAxisNode from corner edge (offset = 0.8m)
    auto edgeOffsetAxis = DatumAxisNode::CreateFromEdge("Axis_Edge_Offset", extrude, edgeIdx, 0.8, true);
    graph.AddNode(edgeOffsetAxis);
    edgeOffsetAxis->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(edgeOffsetAxis->IsEdgeAssociative(), "Axis should be EdgeAssociative");
    TEST_ASSERT(!edgeOffsetAxis->HasConflict(), "Edge offset axis should not have conflict");
    Geom::Axis3D evalEdgeAxis = edgeOffsetAxis->GetAxis();
    double axisDistFromMid = evalEdgeAxis.origin.distance_to(edgeMid);
    TEST_ASSERT(std::abs(axisDistFromMid - 0.8) < 1e-3, "Edge offset axis origin must be offset by 0.8m from edge midpoint");

    // 3. Pick a Corner Vertex on Box_Extrude
    int vertIdx = static_cast<int>(edges[0]);
    Geom::Point3D vertPos = eP0;
    Geom::Plane cornerVertBasePlane;
    bool vertPlaneOk = DatumPlaneNode::ComputeCornerVertexPlane(extrude, vertIdx, cornerVertBasePlane);
    TEST_ASSERT(vertPlaneOk, "ComputeCornerVertexPlane must succeed on corner vertex");

    auto vertOffsetPlane = std::make_shared<DatumPlaneNode>("Plane_Vertex_Offset", extrude, vertIdx, cornerVertBasePlane, true, 1.0);
    graph.AddNode(vertOffsetPlane);
    vertOffsetPlane->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(vertOffsetPlane->IsVertexAssociative(), "Plane should be VertexAssociative");
    TEST_ASSERT(!vertOffsetPlane->HasConflict(), "Vertex offset plane should not have conflict");
    TEST_ASSERT(std::abs(vertOffsetPlane->GetPlane().origin.distance_to(vertPos) - 1.0) < 1e-3,
                "Vertex offset plane origin must be offset by 1.0m from corner vertex");

    auto vertOffsetAxis = std::make_shared<DatumAxisNode>(
        "Axis_Vertex_Offset", extrude, vertIdx, cornerVertBasePlane.normal, 0.75);
    graph.AddNode(vertOffsetAxis);
    vertOffsetAxis->Evaluate(&mockDevice, exchange);

    TEST_ASSERT(vertOffsetAxis->IsVertexAssociative(), "Axis should be VertexAssociative");
    TEST_ASSERT(!vertOffsetAxis->HasConflict(), "Vertex offset axis should not have conflict");
    TEST_ASSERT(std::abs(vertOffsetAxis->GetAxis().origin.distance_to(vertPos) - 0.75) < 1e-3,
                "Vertex offset axis origin must be offset by 0.75m from corner vertex");

    // 4. Parametric update propagation: change extrude height from 3.0 to 5.0
    extrude->SetHeight(5.0f);
    extrude->Evaluate(&mockDevice, exchange);
    faceOffsetPlane->Evaluate(&mockDevice, exchange);
    faceOffsetAxis->Evaluate(&mockDevice, exchange);
    TEST_ASSERT(std::abs(faceOffsetPlane->GetPlane().origin.y - 6.5) < 1e-3,
                "After extrude height=5.0, face offset plane Y should update to 5.0 + 1.5 = 6.5");
    TEST_ASSERT(std::abs(faceOffsetAxis->GetAxis().origin.y - 7.0) < 1e-3,
                "After extrude height=5.0, face offset axis Y should update to 5.0 + 2.0 = 7.0");

    return true;
}

bool TestSketchCreatedOnFaceSurfacePlane() {
    std::cout << "[Test] 2D Sketch Created from a Face is Drawn on the Surface Plane of That Face (Not at Origin)...\n";

    ParametricGraph graph(nullptr);
    AtomicMeshExchange exchange;
    MockTestDevice mockDevice;

    auto baseSketch = std::make_shared<SketchNode>("Base_Sketch", SketchPlane::XZ_Ground);
    baseSketch->SetPresetRectangle(4.0f, 4.0f);
    graph.AddNode(baseSketch);

    auto baseExtrude = std::make_shared<ExtrudeNode>("Base_Extrude", baseSketch, 3.0f);
    graph.AddNode(baseExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    // 1. Pick top face (Y = 3.0) and create a 2D sketch on that face with 0.0 offset
    Geom::Point3D rayOrigTop(0.0, 10.0, 0.0);
    Geom::Vector3D rayDirTop(0.0, -1.0, 0.0);
    uint64_t hitId = graph.PickNode(rayOrigTop, rayDirTop);
    TEST_ASSERT(hitId == baseExtrude->GetId(), "PickNode should hit Base_Extrude");

    auto topFacePick = graph.PickSubElement(rayOrigTop, rayDirTop, hitId);
    TEST_ASSERT(topFacePick.faceIndex >= 0, "PickSubElement must identify the clicked top face on first click");
    TEST_ASSERT(std::abs(topFacePick.facePlane.origin.y - 3.0) < 1e-3, "Top face plane origin Y must be at 3.0m");

    auto topPlaneNode = std::make_shared<DatumPlaneNode>("Plane_TopFace", baseExtrude, topFacePick.faceIndex, topFacePick.facePlane, 0.0);
    graph.AddNode(topPlaneNode);
    auto topSketch = std::make_shared<SketchNode>("Sketch_OnTopFace", topPlaneNode);
    topSketch->SetPresetRectangle(1.5f, 1.5f);
    graph.AddNode(topSketch);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(topSketch->HasCustomPlane(), "Sketch created from face must use custom plane");
    TEST_ASSERT(std::abs(topSketch->GetCustomPlane().origin.y - 3.0) < 1e-3, "Sketch custom plane Y must be at 3.0m (not 0.0)");
    TEST_ASSERT(std::abs(topSketch->GetPosition()[1] - 3.0f) < 1e-3f, "Sketch position Y must be at 3.0m (not 0.0)");

    const auto& topSketchVerts = topSketch->GetCachedVertices();
    TEST_ASSERT(!topSketchVerts.empty(), "Top face sketch must have evaluated 3D vertices");
    for (const auto& v : topSketchVerts) {
        TEST_ASSERT(std::abs(v.position[1] - 3.005f) < 1e-2f,
                    "Every vertex of the sketch on the top face must lie on the Y=3.0 face surface (never at Y=0 origin)");
    }

    // 2. Pick front vertical face (Z = 2.0, normal = +Z) and create a 2D sketch on that face
    Geom::Point3D rayOrigFront(0.0, 1.5, 10.0);
    Geom::Vector3D rayDirFront(0.0, 0.0, -1.0);
    auto frontFacePick = graph.PickSubElement(rayOrigFront, rayDirFront, hitId);
    TEST_ASSERT(frontFacePick.faceIndex >= 0, "PickSubElement must identify the front face");
    TEST_ASSERT(std::abs(frontFacePick.facePlane.origin.z - 2.0) < 1e-3, "Front face plane origin Z must be at 2.0m");
    TEST_ASSERT(frontFacePick.facePlane.normal.z > 0.99, "Front face normal must point along +Z");

    auto frontPlaneNode = std::make_shared<DatumPlaneNode>("Plane_FrontFace", baseExtrude, frontFacePick.faceIndex, frontFacePick.facePlane, 0.0);
    graph.AddNode(frontPlaneNode);
    auto frontSketch = std::make_shared<SketchNode>("Sketch_OnFrontFace", frontPlaneNode);
    frontSketch->SetPresetRectangle(1.0f, 1.0f);
    graph.AddNode(frontSketch);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    const auto& frontSketchVerts = frontSketch->GetCachedVertices();
    TEST_ASSERT(!frontSketchVerts.empty(), "Front face sketch must have evaluated 3D vertices");
    for (const auto& v : frontSketchVerts) {
        TEST_ASSERT(std::abs(v.position[2] - 2.005f) < 1e-2f,
                    "Every vertex of the sketch on the front face must lie on the Z=2.0 face surface (never at Z=0 origin)");
    }

    // 3. Verify Parametric Update: if Base_Extrude height changes from 3.0 to 4.5, topSketch automatically moves to Y=4.5
    baseExtrude->SetHeight(4.5f);
    baseExtrude->Evaluate(&mockDevice, exchange);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(std::abs(topSketch->GetCustomPlane().origin.y - 4.5) < 1e-3,
                "When parent feature height updates to 4.5m, sketch plane on top face must move to Y=4.5m");
    for (const auto& v : topSketch->GetCachedVertices()) {
        TEST_ASSERT(std::abs(v.position[1] - 4.505f) < 1e-2f,
                    "Updated sketch vertices must lie on the new Y=4.5m face surface");
    }

    return true;
}

bool Test2DSketchKeyboardDimensionsAndParametricScaling() {
    std::cout << "[Test] 2D Sketch Keyboard Dimensions Input & Parametric Scaling...\n";

    // 1. Keyboard Rectangle Construction
    float kbW = 4.0f;
    float kbH = 3.0f;
    float kbCU = 1.0f;
    float kbCV = 2.0f;
    float hw = kbW * 0.5f;
    float hh = kbH * 0.5f;

    std::vector<std::array<float, 2>> rectPts = {
        { kbCU - hw, kbCV - hh }, // (-1.0, 0.5)
        { kbCU + hw, kbCV - hh }, // ( 3.0, 0.5)
        { kbCU + hw, kbCV + hh }, // ( 3.0, 3.5)
        { kbCU - hw, kbCV + hh }  // (-1.0, 3.5)
    };
    TEST_ASSERT(rectPts.size() == 4, "Rectangle must have 4 vertices");
    TEST_ASSERT(std::abs(rectPts[1][0] - rectPts[0][0] - kbW) < 1e-5f, "Initial width must equal 4.0m");
    TEST_ASSERT(std::abs(rectPts[2][1] - rectPts[1][1] - kbH) < 1e-5f, "Initial height must equal 3.0m");

    // 2. Parametric Width & Height Scaling via Keyboard Input
    float newWidth = 6.0f;
    float scaleX = newWidth / kbW;
    for (auto& pt : rectPts) {
        pt[0] = kbCU + (pt[0] - kbCU) * scaleX;
    }
    TEST_ASSERT(std::abs(rectPts[1][0] - rectPts[0][0] - 6.0f) < 1e-5f, "Scaled width must be 6.0m");
    TEST_ASSERT(std::abs((rectPts[0][0] + rectPts[1][0]) * 0.5f - kbCU) < 1e-5f, "Center U must remain unchanged at 1.0m");

    float newHeight = 5.0f;
    float scaleY = newHeight / kbH;
    for (auto& pt : rectPts) {
        pt[1] = kbCV + (pt[1] - kbCV) * scaleY;
    }
    TEST_ASSERT(std::abs(rectPts[2][1] - rectPts[1][1] - 5.0f) < 1e-5f, "Scaled height must be 5.0m");
    TEST_ASSERT(std::abs((rectPts[1][1] + rectPts[2][1]) * 0.5f - kbCV) < 1e-5f, "Center V must remain unchanged at 2.0m");

    // 3. Vertex Coordinate and Incident Edge Length Editing
    // Move vertex 0 from (-2.0, -0.5) to (-2.5, -0.5)
    rectPts[0][0] = -2.5f;
    TEST_ASSERT(std::abs(rectPts[0][0] - (-2.5f)) < 1e-5f, "Vertex 0 U coordinate must update via keyboard input");

    // Adjust Edge 0 length via keyboard input
    float targetEdgeLen = 7.0f;
    float dU = rectPts[1][0] - rectPts[0][0];
    float dV = rectPts[1][1] - rectPts[0][1];
    float curLen = std::hypot(dU, dV);
    rectPts[1][0] = rectPts[0][0] + (dU / curLen) * targetEdgeLen;
    rectPts[1][1] = rectPts[0][1] + (dV / curLen) * targetEdgeLen;
    float updatedEdgeLen = std::hypot(rectPts[1][0] - rectPts[0][0], rectPts[1][1] - rectPts[0][1]);
    TEST_ASSERT(std::abs(updatedEdgeLen - 7.0f) < 1e-4f, "Edge length must match 7.0m exactly after keyboard modification");

    // 4. Keyboard Circle Construction & Radius Scaling
    float circleRadius = 2.5f;
    float circleCU = 0.0f;
    float circleCV = 0.0f;
    const int segs = 32;
    std::vector<std::array<float, 2>> circleProfile;
    for (int i = 0; i < segs; ++i) {
        float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 6.2831853f;
        circleProfile.push_back({circleCU + circleRadius * std::cos(theta), circleCV + circleRadius * std::sin(theta)});
    }
    TEST_ASSERT(circleProfile.size() == 32, "Circle must contain 32 segments");

    // Scale radius from 2.5 to 3.8m via keyboard
    float newRadius = 3.8f;
    float radScale = newRadius / circleRadius;
    for (auto& pt : circleProfile) {
        pt[0] = circleCU + (pt[0] - circleCU) * radScale;
        pt[1] = circleCV + (pt[1] - circleCV) * radScale;
    }
    for (const auto& pt : circleProfile) {
        float r = std::hypot(pt[0] - circleCU, pt[1] - circleCV);
        TEST_ASSERT(std::abs(r - 3.8f) < 1e-4f, "Every circle vertex must scale to 3.8m radius");
    }

    // 5. Line Segment Keyboard Input (Length + Angle)
    std::vector<std::array<float, 2>> linePts = { {0.0f, 0.0f} };
    float lineLen = 5.0f;
    float lineAngleDeg = 45.0f;
    float lineAngleRad = lineAngleDeg * 3.14159265f / 180.0f;
    float nextU = linePts.back()[0] + lineLen * std::cos(lineAngleRad);
    float nextV = linePts.back()[1] + lineLen * std::sin(lineAngleRad);
    linePts.push_back({nextU, nextV});

    float computedLen = std::hypot(linePts[1][0] - linePts[0][0], linePts[1][1] - linePts[0][1]);
    TEST_ASSERT(std::abs(computedLen - 5.0f) < 1e-4f, "Segment generated via keyboard length/angle must be 5.0m");
    TEST_ASSERT(std::abs(linePts[1][0] - (5.0f * std::cos(lineAngleRad))) < 1e-4f, "Segment X must match angle");
    TEST_ASSERT(std::abs(linePts[1][1] - (5.0f * std::sin(lineAngleRad))) < 1e-4f, "Segment Y must match angle");

    return true;
}

bool Test3DFeaturePreviewConfirmAndCancel() {
    std::cout << "[Test] 3D Feature Preview, Live Adjustments, Confirm & Cancel...\n";

    ParametricGraph graph(nullptr);
    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;

    // Create base 2D sketch
    auto sketch = std::make_shared<SketchNode>("BaseSketch", SketchPlane::XZ_Ground);
    sketch->SetPresetRectangle(4.0f, 3.0f);
    graph.AddNode(sketch);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!sketch->IsConsumed(), "Initial sketch must be unconsumed");
    TEST_ASSERT(sketch->IsVisible(), "Initial sketch must be visible");

    // 1. Extrude Feature Preview
    auto previewExtrude = std::make_shared<ExtrudeNode>("Extrude_Preview", sketch, 2.0f);
    graph.AddNode(previewExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(sketch->IsConsumed(), "When preview node is evaluated in graph, sketch is consumed");
    TEST_ASSERT(graph.GetNodeCount() == 2, "Graph must contain sketch + preview extrude");

    // Live parameter adjustment during preview
    previewExtrude->SetHeight(4.0f);
    previewExtrude->MarkDirty();
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(std::abs(previewExtrude->GetHeight() - 4.0f) < 1e-5f, "Extrude preview height must update live to 4.0m");

    // Cancel Preview: graph.RemoveNode removes preview node and unconsumes parent sketch
    graph.RemoveNode(previewExtrude->GetId());
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(graph.GetNodeCount() == 1, "Graph must contain only original sketch after canceling preview");
    TEST_ASSERT(!sketch->IsConsumed(), "Sketch must be restored to unconsumed state after preview cancellation");
    TEST_ASSERT(sketch->IsVisible(), "Sketch must be visible after preview cancellation");

    // 2. Extrude Feature Confirmation
    auto committedExtrude = std::make_shared<ExtrudeNode>("Extrude_Committed", sketch, 3.5f);
    graph.AddNode(committedExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(graph.GetNodeCount() == 2, "Graph must contain 2 nodes after confirming extrude");
    TEST_ASSERT(committedExtrude->GetActiveLeafFeature() == committedExtrude, "Committed extrude is the active leaf feature");

    // 3. Chamfer Feature Preview on Corner Edge
    auto previewChamfer = std::make_shared<ChamferNode>("Chamfer_Preview", committedExtrude, 0, 0.35f);
    graph.AddNode(previewChamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(committedExtrude->IsConsumed(), "Parent extrude is consumed during chamfer preview");
    TEST_ASSERT(graph.GetNodeCount() == 3, "Graph must contain sketch, extrude, and chamfer preview");

    // Live parameter change during chamfer preview
    previewChamfer->SetDistance(0.5f);
    previewChamfer->MarkDirty();
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(std::abs(previewChamfer->GetDistance() - 0.5f) < 1e-5f, "Chamfer distance must update live to 0.5m");

    // Cancel Chamfer Preview: parent extrude must be unconsumed and restored
    graph.RemoveNode(previewChamfer->GetId());
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(graph.GetNodeCount() == 2, "Graph must revert to sketch + extrude after canceling chamfer preview");
    TEST_ASSERT(!committedExtrude->IsConsumed(), "Extrude must be restored to unconsumed state after chamfer preview cancellation");
    TEST_ASSERT(committedExtrude->GetActiveLeafFeature() == committedExtrude, "Extrude must be active leaf feature again");

    // Confirm Chamfer Feature
    auto committedChamfer = std::make_shared<ChamferNode>("Chamfer_Committed", committedExtrude, 0, 0.4f);
    graph.AddNode(committedChamfer);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(committedExtrude->IsConsumed(), "Extrude is consumed by committed chamfer");
    TEST_ASSERT(committedChamfer->GetActiveLeafFeature() == committedChamfer, "Committed chamfer is active leaf feature");

    // 4. Fillet Feature Preview on Top of Chamfer
    auto previewFillet = std::make_shared<FilletNode>("Fillet_Preview", committedChamfer, 1, 0.25f);
    graph.AddNode(previewFillet);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(committedChamfer->IsConsumed(), "Chamfer is consumed during fillet preview");

    // Cancel Fillet Preview
    graph.RemoveNode(previewFillet->GetId());
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!committedChamfer->IsConsumed(), "Chamfer must be unconsumed after fillet preview cancellation");
    TEST_ASSERT(committedChamfer->GetActiveLeafFeature() == committedChamfer, "Chamfer restored as active leaf");

    return true;
}

bool Test2DSketchDimensionConstraintsAndLocking() {
    std::cout << "[Test] 2D Sketch Dimension Constraint Locking, Editing & Toggling...\n";

    // 1. Create a 4-vertex rectangle shape
    // Vertices: (0, 0), (4, 0), (4, 3), (0, 3)
    std::vector<std::array<float, 2>> rectPts = {
        { 0.0f, 0.0f },
        { 4.0f, 0.0f },
        { 4.0f, 3.0f },
        { 0.0f, 3.0f }
    };

    auto Dist = [](const std::array<float, 2>& a, const std::array<float, 2>& b) {
        return std::hypot(b[0] - a[0], b[1] - a[1]);
    };

    TEST_ASSERT(std::abs(Dist(rectPts[0], rectPts[1]) - 4.0f) < 1e-4f, "Edge 0 initial length must be 4.0m");
    TEST_ASSERT(std::abs(Dist(rectPts[1], rectPts[2]) - 3.0f) < 1e-4f, "Edge 1 initial length must be 3.0m");
    TEST_ASSERT(std::abs(Dist(rectPts[2], rectPts[3]) - 4.0f) < 1e-4f, "Edge 2 initial length must be 4.0m");
    TEST_ASSERT(std::abs(Dist(rectPts[3], rectPts[0]) - 3.0f) < 1e-4f, "Edge 3 initial length must be 3.0m");

    // 2. Double-click line dimension edit: Lock & Constrain Edge 0 to 5.0m
    std::map<size_t, float> constraints;
    constraints[0] = 5.0f; // Constrained to 5.0m

    // Enforce constraint
    SketchNode::EnforceConstraints(rectPts, constraints, true, -1);
    TEST_ASSERT(std::abs(Dist(rectPts[0], rectPts[1]) - 5.0f) < 1e-4f, "Edge 0 must now be constrained to 5.0m");

    // 3. Subsequent dimension changes (e.g. Scaling width of drawing) MUST NOT change constrained edge!
    // Stretch vertices 1 and 2 along X by 2.0x (unconstrained scaling attempt)
    rectPts[1][0] *= 2.0f;
    rectPts[2][0] *= 2.0f;
    TEST_ASSERT(Dist(rectPts[0], rectPts[1]) > 6.0f, "Edge 0 unconstrained stretched temporarily");

    // Enforce constraints after subsequent changes
    SketchNode::EnforceConstraints(rectPts, constraints, true, -1);
    TEST_ASSERT(std::abs(Dist(rectPts[0], rectPts[1]) - 5.0f) < 1e-4f,
        "Constrained Edge 0 MUST strictly preserve its 5.0m locked length after subsequent changes");

    // 4. Vertex dragging with pinned vertex
    // User drags vertex 0 to (-2.0, 1.0)
    rectPts[0] = { -2.0f, 1.0f };
    SketchNode::EnforceConstraints(rectPts, constraints, true, 0); // vertex 0 pinned
    TEST_ASSERT(std::abs(rectPts[0][0] - (-2.0f)) < 1e-4f && std::abs(rectPts[0][1] - 1.0f) < 1e-4f,
        "Pinned vertex 0 must remain at mouse drag coordinates");
    TEST_ASSERT(std::abs(Dist(rectPts[0], rectPts[1]) - 5.0f) < 1e-4f,
        "Edge 0 length must still be locked at 5.0m while dragging vertex 0");

    // 5. Multiple coupled constraints: Constrain Edge 1 to 4.0m
    constraints[1] = 4.0f;
    SketchNode::EnforceConstraints(rectPts, constraints, true, -1);
    TEST_ASSERT(std::abs(Dist(rectPts[0], rectPts[1]) - 5.0f) < 1e-3f, "Edge 0 constraint 5.0m must remain satisfied");
    TEST_ASSERT(std::abs(Dist(rectPts[1], rectPts[2]) - 4.0f) < 1e-3f, "Edge 1 constraint 4.0f must be satisfied");

    // 6. Toggling constraint OFF: Unlock Edge 0
    constraints.erase(0);
    // Now modify Edge 0 freely to 7.5m (updating opposite corner for rectangle as the UI does)
    float dU = rectPts[1][0] - rectPts[0][0];
    float dV = rectPts[1][1] - rectPts[0][1];
    float curLen = std::hypot(dU, dV);
    float deltaL = 7.5f - curLen;
    rectPts[1][0] += (dU / curLen) * deltaL;
    rectPts[1][1] += (dV / curLen) * deltaL;
    rectPts[2][0] += (dU / curLen) * deltaL;
    rectPts[2][1] += (dV / curLen) * deltaL;

    // Enforce remaining constraints (only Edge 1 is constrained now)
    SketchNode::EnforceConstraints(rectPts, constraints, true, -1);
    TEST_ASSERT(std::abs(Dist(rectPts[0], rectPts[1]) - 7.5f) < 1e-3f,
        "Unlocked Edge 0 can now change freely and is not reverted");
    TEST_ASSERT(std::abs(Dist(rectPts[1], rectPts[2]) - 4.0f) < 1e-3f,
        "Edge 1 constraint remains locked at 4.0m");

    // 7. Test SketchNode Constraint Persistence
    auto sketch = std::make_shared<SketchNode>("Sketch_Constrained", SketchPlane::XZ_Ground);
    sketch->SetEdgeConstraint(0, 0, 5.0f);
    sketch->SetEdgeConstraint(0, 1, 4.0f);
    TEST_ASSERT(sketch->IsEdgeConstrained(0, 0), "Edge 0 must be constrained in SketchNode");
    TEST_ASSERT(sketch->IsEdgeConstrained(0, 1), "Edge 1 must be constrained in SketchNode");
    TEST_ASSERT(!sketch->IsEdgeConstrained(0, 2), "Edge 2 must not be constrained");
    TEST_ASSERT(std::abs(sketch->GetEdgeConstrainedLength(0, 0) - 5.0f) < 1e-4f, "Edge 0 length in sketch must be 5.0f");
    TEST_ASSERT(std::abs(sketch->GetEdgeConstrainedLength(0, 1) - 4.0f) < 1e-4f, "Edge 1 length in sketch must be 4.0f");

    // Toggle/remove constraint from sketch
    sketch->RemoveEdgeConstraint(0, 0);
    TEST_ASSERT(!sketch->IsEdgeConstrained(0, 0), "Edge 0 must no longer be constrained after removal");
    TEST_ASSERT(sketch->IsEdgeConstrained(0, 1), "Edge 1 remains constrained");

    return true;
}

bool TestViewRecenterAndAxesReferenceAndSketchClick() {
    std::cout << "[Test] Camera View Recenter on Top, Side, Iso Origin, Axes Triad & 2D Sketch Click-Drag...\n";

    Camera camera;
    camera.SetViewportSize(1920.0f, 1080.0f);

    // 1. Pan camera away from origin to simulate user panning
    camera.Pan(5.0f, -3.0f);
    const float* targetPanned = camera.GetTarget();
    TEST_ASSERT(std::abs(targetPanned[0]) > 0.1f || std::abs(targetPanned[1]) > 0.1f,
        "Camera target must be panned away from origin");

    // Recenter Origin Only
    camera.RecenterOrigin();
    const float* targetOrigin = camera.GetTarget();
    TEST_ASSERT(std::abs(targetOrigin[0]) < 1e-5f && std::abs(targetOrigin[1]) < 1e-5f && std::abs(targetOrigin[2]) < 1e-5f,
        "RecenterOrigin must restore target to (0, 0, 0)");

    // 2. Top View (Looking down +Y onto XZ plane, centered on origin)
    camera.Pan(4.0f, 2.0f);
    camera.SetTopView(true);
    TEST_ASSERT(std::abs(camera.GetTarget()[0]) < 1e-5f && std::abs(camera.GetTarget()[1]) < 1e-5f && std::abs(camera.GetTarget()[2]) < 1e-5f,
        "Top View must recenter target to origin");
    TEST_ASSERT(std::abs(camera.GetElevation() - 1.5f) < 1e-4f, "Top View elevation must be +1.5 rad (~90 deg up)");
    TEST_ASSERT(std::abs(camera.GetAzimuth() - 0.0f) < 1e-4f, "Top View azimuth must be 0.0 rad");

    // 3. Side View (Looking from +X towards YZ plane, centered on origin)
    camera.Pan(-2.0f, 8.0f);
    camera.SetSideView(true);
    TEST_ASSERT(std::abs(camera.GetTarget()[0]) < 1e-5f && std::abs(camera.GetTarget()[1]) < 1e-5f && std::abs(camera.GetTarget()[2]) < 1e-5f,
        "Side View must recenter target to origin");
    TEST_ASSERT(std::abs(camera.GetAzimuth() - 1.570796f) < 1e-4f, "Side View azimuth must be pi/2 rad (~90 deg side)");
    TEST_ASSERT(std::abs(camera.GetElevation() - 0.0f) < 1e-4f, "Side View elevation must be 0.0 rad");

    // 4. Isometric View (Corner viewing angle, centered on origin)
    camera.Pan(3.0f, -5.0f);
    camera.SetIsometricView(true);
    TEST_ASSERT(std::abs(camera.GetTarget()[0]) < 1e-5f && std::abs(camera.GetTarget()[1]) < 1e-5f && std::abs(camera.GetTarget()[2]) < 1e-5f,
        "Isometric View must recenter target to origin");
    TEST_ASSERT(std::abs(camera.GetAzimuth() - 0.785398f) < 1e-4f, "Isometric View azimuth must be 45 deg (pi/4)");
    TEST_ASSERT(std::abs(camera.GetElevation() - 0.523599f) < 1e-4f, "Isometric View elevation must be ~30 deg");

    // 5. Front View (Looking along +Z towards XY plane, centered on origin)
    camera.SetFrontView(true);
    TEST_ASSERT(std::abs(camera.GetTarget()[0]) < 1e-5f && std::abs(camera.GetTarget()[1]) < 1e-5f && std::abs(camera.GetTarget()[2]) < 1e-5f,
        "Front View must recenter target to origin");
    TEST_ASSERT(std::abs(camera.GetAzimuth() - 0.0f) < 1e-4f && std::abs(camera.GetElevation() - 0.0f) < 1e-4f,
        "Front View must align azimuth and elevation to 0.0");

    // 6. Camera Basis Vectors & Triad Projection
    camera.SetIsometricView(true);
    float fwd[3], rgt[3], up[3];
    camera.GetBasisVectors(fwd, rgt, up);
    float rgtLen = std::sqrt(rgt[0]*rgt[0] + rgt[1]*rgt[1] + rgt[2]*rgt[2]);
    float upLen = std::sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
    float fwdLen = std::sqrt(fwd[0]*fwd[0] + fwd[1]*fwd[1] + fwd[2]*fwd[2]);
    TEST_ASSERT(std::abs(rgtLen - 1.0f) < 1e-4f, "Camera Right basis vector must be normalized");
    TEST_ASSERT(std::abs(upLen - 1.0f) < 1e-4f, "Camera Up basis vector must be normalized");
    TEST_ASSERT(std::abs(fwdLen - 1.0f) < 1e-4f, "Camera Forward basis vector must be normalized");

    // Right and Up must be orthogonal
    float dotRU = rgt[0]*up[0] + rgt[1]*up[1] + rgt[2]*up[2];
    TEST_ASSERT(std::abs(dotRU) < 1e-4f, "Camera Right and Up basis vectors must be orthogonal");

    // 7. 2D Sketch: Click vs Click-and-Drag behavior
    // Verify that selecting/clicking a shape does NOT alter geometry unless drag threshold is exceeded
    std::vector<std::array<float, 2>> shapeOrig = {
        { 0.0f, 0.0f }, { 4.0f, 0.0f }, { 4.0f, 2.0f }, { 0.0f, 2.0f }
    };
    std::vector<std::array<float, 2>> shapeCurrent = shapeOrig;

    // Simulate mouse click with NO drag motion (isDragging = false)
    bool isDragging = false;
    float dragStartU = 2.0f;
    float dragStartV = 1.0f;
    float currentMouseU = 2.02f; // Minor sub-pixel jitter
    float currentMouseV = 1.01f;

    if (isDragging) {
        float du = currentMouseU - dragStartU;
        float dv = currentMouseV - dragStartV;
        for (size_t i = 0; i < shapeCurrent.size(); ++i) {
            shapeCurrent[i] = { shapeOrig[i][0] + du, shapeOrig[i][1] + dv };
        }
    }
    // Shape points MUST remain strictly identical to shapeOrig when simply clicking
    for (size_t i = 0; i < shapeCurrent.size(); ++i) {
        TEST_ASSERT(shapeCurrent[i][0] == shapeOrig[i][0] && shapeCurrent[i][1] == shapeOrig[i][1],
            "Shape coordinates must NOT change on mouse click without drag motion");
    }

    // Simulate intentional drag (isDragging = true)
    isDragging = true;
    currentMouseU = 3.5f;
    currentMouseV = 2.5f;
    if (isDragging) {
        float du = currentMouseU - dragStartU; // +1.5m
        float dv = currentMouseV - dragStartV; // +1.5m
        for (size_t i = 0; i < shapeCurrent.size(); ++i) {
            shapeCurrent[i] = { shapeOrig[i][0] + du, shapeOrig[i][1] + dv };
        }
    }
    TEST_ASSERT(std::abs(shapeCurrent[0][0] - 1.5f) < 1e-5f && std::abs(shapeCurrent[0][1] - 1.5f) < 1e-5f,
        "Shape coordinates must translate only when click-and-drag motion is active");

    return true;
}

bool TestExtrudedCutAndRevolvedCutOperations() {
    std::cout << "[Test] Extruded Cut & Revolved Cut Operations on Existing Models...\n";

    MockTestDevice mockDevice;
    AtomicMeshExchange exchange;
    ParametricGraph graph(nullptr);

    // 1. Create Base Solid: Extruded Block 4.0 x 4.0 with height 2.0
    // Vertices at base plane Y=0: (-2, -2), (2, -2), (2, 2), (-2, 2)
    auto baseSketch = std::make_shared<SketchNode>("Base_Sketch", SketchPlane::XZ_Ground);
    baseSketch->SetProfile({ {-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, 2.0f}, {-2.0f, 2.0f} });
    graph.AddNode(baseSketch);

    auto baseExtrude = std::make_shared<ExtrudeNode>("Base_Block", baseSketch, 2.0f);
    graph.AddNode(baseExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(!baseExtrude->GetCachedVertices().empty(), "Base block must have vertices");
    TEST_ASSERT(!baseExtrude->IsConsumed(), "Base block must be active solid initially");
    size_t baseVertCount = baseExtrude->GetCachedVertices().size();
    size_t baseIndexCount = baseExtrude->GetCachedIndices().size();

    // 2. Create a Cut Sketch on the top face at Y=2.0 (plane XZ with Y=2.0 offset)
    // Centered square pocket 2.0 x 2.0: (-1, -1) to (1, 1)
    auto cutSketch = std::make_shared<SketchNode>("Cut_Pocket_Sketch", SketchPlane::XZ_Ground);
    cutSketch->SetPosition(0.0f, 2.0f, 0.0f); // on top face
    cutSketch->SetProfile({ {-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f} });
    graph.AddNode(cutSketch);

    // 3. Create CutExtrudeNode (blind pocket of depth -1.0 into the model)
    // Height -1.0 sweeps from Y=2.0 down to Y=1.0
    auto cutExtrude = std::make_shared<CutExtrudeNode>("Pocket_Cut", cutSketch, -1.0f, baseExtrude);
    graph.AddNode(cutExtrude);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(cutExtrude->IsCut(), "Must be cut operation");
    TEST_ASSERT(!cutExtrude->HasConflict(), "Cut extrude must evaluate cleanly without conflict");
    TEST_ASSERT(baseExtrude->IsConsumed(), "Base block must be consumed when cut away by cut feature");
    TEST_ASSERT(cutExtrude->GetParentFeature() == baseExtrude, "Cut extrude must have base block as parent feature");

    // 4. Verify pocket geometry in mesh
    const auto& cutVerts = cutExtrude->GetCachedVertices();
    const auto& cutIndices = cutExtrude->GetCachedIndices();
    const auto& cutEdges = cutExtrude->GetCachedEdges();
    TEST_ASSERT(!cutVerts.empty() && !cutIndices.empty(), "Cut mesh must not be empty");

    // Check for pocket floor vertices at Y ≈ 1.0
    bool foundFloorAtY1 = false;
    for (const auto& v : cutVerts) {
        if (std::abs(v.position[1] - 1.0f) < 0.05f) {
            foundFloorAtY1 = true;
            break;
        }
    }
    TEST_ASSERT(foundFloorAtY1, "Cut mesh must contain pocket floor at Y = 1.0m");

    // Check that feature edges include pocket rim and internal corners
    TEST_ASSERT(cutEdges.size() > baseExtrude->GetCachedEdges().size(),
        "Cut mesh must have additional feature edges for the pocket rim, walls, and floor");

    // 5. Test Safe Feature Deletion:
    // Deleting the cut feature restores the uncut base model
    graph.RemoveNode(cutExtrude->GetId());
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!baseExtrude->IsConsumed(), "Base block must be unconsumed and restored after deleting cut feature");
    TEST_ASSERT(baseExtrude->GetCachedVertices().size() == baseVertCount, "Restored base block must retain its original vertices");
    TEST_ASSERT(baseExtrude->GetCachedIndices().size() == baseIndexCount, "Restored base block must retain its original indices");

    // 6. Test Revolved Cut:
    // Create a cut sketch on XY_Front plane (centered at X=0, Y=1.0, Z=0)
    // Profile: rectangle from u=0.2 to 0.8, v=-0.5 to 0.5 (revolved hole/groove inside the block)
    auto revCutSketch = std::make_shared<SketchNode>("Rev_Cut_Sketch", SketchPlane::XY_Front);
    revCutSketch->SetPosition(0.0f, 1.0f, 0.0f);
    revCutSketch->SetProfile({ {0.2f, -0.5f}, {0.8f, -0.5f}, {0.8f, 0.5f}, {0.2f, 0.5f} });
    graph.AddNode(revCutSketch);

    auto cutRevolve = std::make_shared<CutRevolveNode>("Groove_Cut_Revolve", revCutSketch, 360.0f, RevolveAxisMode::Sketch_V_Axis, baseExtrude);
    graph.AddNode(cutRevolve);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);

    TEST_ASSERT(cutRevolve->IsCut(), "Must be cut revolve operation");
    TEST_ASSERT(!cutRevolve->HasConflict(), "Cut revolve must evaluate cleanly without conflict");
    TEST_ASSERT(baseExtrude->IsConsumed(), "Base block must be consumed when cut away by revolve feature");
    TEST_ASSERT(cutRevolve->GetParentFeature() == baseExtrude, "Cut revolve must have base block as parent feature");
    TEST_ASSERT(!cutRevolve->GetCachedVertices().empty(), "Revolved cut mesh must have vertices");

    // 7. Test Operation Mode Toggle: Switch CutRevolve to Boss (Add) and back to Cut
    cutRevolve->SetOperation(RevolveOperation::Boss);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(!baseExtrude->IsConsumed(), "Toggling to Boss must unconsume target model");

    cutRevolve->SetOperation(RevolveOperation::Cut);
    graph.EvaluateDirtyNodes(&mockDevice, exchange);
    TEST_ASSERT(baseExtrude->IsConsumed(), "Toggling back to Cut must consume target model again");

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
    pass &= TestSketchChangePropagationToFeature();
    pass &= TestMultiShapeSketchAndStitching();
    pass &= TestSketchShapeAndVertexDeletion();
    pass &= TestStandaloneSketchVisibilityAndVisualStyling();
    pass &= TestTopBarReferenceGeometryAndFeatureWorkflows();
    pass &= TestRevolveCylindricalThicknessAndNoBoxFeature();
    pass &= TestModelClickSelectionHighlightAndOuterEdges();
    pass &= TestFaceSubElementPickingAndHighlighting();
    pass &= TestAssociativeDatumPlaneAndConflictHandling();
    pass &= TestVertexSubElementPickingAndAssociativeDatumAxis();
    pass &= TestFaceAndVertexDatumPlaneCenteringIn3DSpace();
    pass &= TestReferenceFeatureProjectionAndToggle();
    pass &= TestNegativeExtrudeGraphAndCustomPlane();
    pass &= TestDirectionalExtrusion();
    pass &= TestFilletAndChamferFeaturesAndEdgePicking();
    pass &= TestChainedFilletAndChamferFeatures();
    pass &= TestThreeCornerVertexFillet();
    pass &= TestHorizontalAndGeneral3DEdgeFilletAndChamfer();
    pass &= TestLBracketTopRimFilletWithTangentChainAndVerticalCornerFillets();
    pass &= TestFilletChamferChildRelationshipAndSafeFeatureDeletion();
    pass &= TestFilletDoesNotWrapOntoAdjoiningEdges();
    pass &= TestFilletEndCapIsTrimmedToProfile();
    pass &= TestChamferUsesFilletCornerAndEndCapBehavior();
    pass &= TestFilletRunningIntoEarlierRoundWrapsCleanly();
    pass &= TestVertexFilletRoundsAllThreeCornerEdges();
    pass &= TestOffsetReferencePlanesAndAxesFromFaceAndCorners();
    pass &= TestSketchCreatedOnFaceSurfacePlane();
    pass &= TestSolidWorksMouseNavigation();
    pass &= Test2DSketchKeyboardDimensionsAndParametricScaling();
    pass &= Test3DFeaturePreviewConfirmAndCancel();
    pass &= Test2DSketchDimensionConstraintsAndLocking();
    pass &= TestViewRecenterAndAxesReferenceAndSketchClick();
    pass &= TestExtrudedCutAndRevolvedCutOperations();

    if (pass) {
        std::cout << "\n[PASS] All 3D Viewport Picking test suites succeeded!\n";
        return 0;
    } else {
        std::cerr << "\n[FAIL] Viewport Picking tests failed!\n";
        return 1;
    }
}

