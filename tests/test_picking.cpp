#include "engine/render/Camera.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/RevolveNode.hpp"
#include "engine/graph/DatumPlaneNode.hpp"
#include "engine/graph/DatumAxisNode.hpp"
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

    if (pass) {
        std::cout << "\n[PASS] All 3D Viewport Picking test suites succeeded!\n";
        return 0;
    } else {
        std::cerr << "\n[FAIL] Viewport Picking tests failed!\n";
        return 1;
    }
}

