#pragma once

#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/CylinderNode.hpp"
#include "engine/graph/SphereNode.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/RevolveNode.hpp"
#include "engine/graph/DatumPlaneNode.hpp"
#include "engine/graph/DatumAxisNode.hpp"
#include "engine/graph/FilletNode.hpp"
#include "engine/graph/ChamferNode.hpp"
#include "engine/sketch/SketchFilletChamfer.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/render/Camera.hpp"
#include "engine/render/ViewportRenderer.hpp"
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <map>

struct GLFWwindow;

namespace Tessera {

enum class SketchMouseTool : uint8_t {
    SelectMove = 0,
    Line = 1,
    Rectangle = 2,
    Circle = 3,
    Spline = 4
};

struct CanvasShape {
    std::string name;
    std::vector<std::array<float, 2>> points;
    bool isClosed{true};
    std::map<size_t, float> edgeConstraints; // edge index -> target length

    [[nodiscard]] bool IsEdgeConstrained(size_t edgeIdx) const {
        return edgeConstraints.find(edgeIdx) != edgeConstraints.end();
    }

    [[nodiscard]] float GetEdgeConstrainedLength(size_t edgeIdx) const {
        auto it = edgeConstraints.find(edgeIdx);
        return (it != edgeConstraints.end()) ? it->second : 0.0f;
    }

    void SetEdgeConstraint(size_t edgeIdx, float len) {
        if (len > 0.0001f) {
            edgeConstraints[edgeIdx] = len;
        }
    }

    void RemoveEdgeConstraint(size_t edgeIdx) {
        edgeConstraints.erase(edgeIdx);
    }

    void ToggleEdgeConstraint(size_t edgeIdx, float currentLen) {
        if (IsEdgeConstrained(edgeIdx)) {
            RemoveEdgeConstraint(edgeIdx);
        } else {
            SetEdgeConstraint(edgeIdx, currentLen);
        }
    }
};

class CADGui {
public:
    CADGui() = default;
    ~CADGui();

    void Init(GLFWwindow* window, RHI::RHIDevice* rhiDevice);
    void Shutdown();

    void BeginFrame();
    void RenderUI(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        Camera& camera,
        ViewportRenderer& renderer
    );
    void EndFrameAndDraw(RHI::RHICommandList* cmdList);

    [[nodiscard]] bool WantCaptureMouse() const noexcept;
    [[nodiscard]] bool WantCaptureKeyboard() const noexcept;

    [[nodiscard]] bool GetShowWireframe() const noexcept { return m_showWireframe; }
    [[nodiscard]] bool GetShowGrid() const noexcept { return m_showGrid; }
    [[nodiscard]] bool GetShowReferenceFeature() const noexcept { return m_showReferenceFeature; }
    void SetShowReferenceFeature(bool show) noexcept { m_showReferenceFeature = show; }
    [[nodiscard]] bool GetShowAxesReference() const noexcept { return m_showAxesReference; }
    void SetShowAxesReference(bool show) noexcept { m_showAxesReference = show; }

    [[nodiscard]] bool IsSketchModeActive() const noexcept { return m_sketchModeActive; }
    [[nodiscard]] bool IsSketchPlanePromptActive() const noexcept { return m_showSketchPlanePrompt; }

    // 3D Feature Preview & Pending Confirmation
    enum class PreviewFeatureType { None, Extrude, Revolve, Fillet, Chamfer, CutExtrude, CutRevolve };
    [[nodiscard]] bool IsFeaturePreviewActive() const noexcept { return m_featurePreviewActive; }
    [[nodiscard]] PreviewFeatureType GetPreviewFeatureType() const noexcept { return m_previewFeatureType; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetPreviewNode() const noexcept { return m_previewNode; }
    static const char* GetPreviewFeatureTypeName(PreviewFeatureType type) noexcept;

    void StartFeaturePreview(
        PreviewFeatureType type,
        std::shared_ptr<DAGNode> previewNode,
        std::shared_ptr<DAGNode> target,
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange
    );
    void ConfirmFeaturePreview();
    void CancelFeaturePreview(ParametricGraph& graph, RHI::RHIDevice* device, AtomicMeshExchange& exchange);
    void RenderFeaturePreviewDialog(ParametricGraph& graph, RHI::RHIDevice* device, AtomicMeshExchange& exchange);
    void EnterSketchMode(uint64_t sketchId) noexcept { 
        m_sketchModeActive = true; 
        m_activeSketchId = sketchId;
        m_paletteTab = 1; // Switch to 2D Sketching tab
    }
    void ExitSketchMode() noexcept { m_sketchModeActive = false; }

    [[nodiscard]] uint64_t GetSelectedNodeId() const noexcept { return m_selectedNodeId; }
    void SetSelectedNodeId(uint64_t id) noexcept {
        if (m_selectedNodeId != id) {
            m_selectedNodeId = id;
            m_selectedFaceIndex = -1;
            m_selectedVertexIndex = -1;
            m_selectedEdgeIndex = -1;
            m_selectionScrollPending = true;
        }
    }

    void SetSelectedVertex(int vIdx, Geom::Point3D pos) noexcept {
        m_selectedVertexIndex = vIdx;
        m_selectedVertexPos = pos;
        m_selectedFaceIndex = -1;
        m_selectedEdgeIndex = -1;
    }

    void SetSelectedFace(int fIdx, Geom::Plane plane) noexcept {
        m_selectedFaceIndex = fIdx;
        m_selectedFacePlane = plane;
        m_selectedVertexIndex = -1;
        m_selectedEdgeIndex = -1;
    }

    void SetSelectedEdge(int eIdx, Geom::Point3D p0, Geom::Point3D p1) noexcept {
        m_selectedEdgeIndex = eIdx;
        m_selectedEdgeP0 = p0;
        m_selectedEdgeP1 = p1;
        m_selectedFaceIndex = -1;
        m_selectedVertexIndex = -1;
    }

    void ClearSubElementSelection() noexcept {
        m_selectedFaceIndex = -1;
        m_selectedVertexIndex = -1;
        m_selectedEdgeIndex = -1;
    }

    [[nodiscard]] int GetSelectedFaceIndex() const noexcept { return m_selectedFaceIndex; }
    [[nodiscard]] int GetSelectedVertexIndex() const noexcept { return m_selectedVertexIndex; }
    [[nodiscard]] int GetSelectedEdgeIndex() const noexcept { return m_selectedEdgeIndex; }
    [[nodiscard]] Geom::Plane GetSelectedFacePlane() const noexcept { return m_selectedFacePlane; }
    [[nodiscard]] Geom::Point3D GetSelectedVertexPos() const noexcept { return m_selectedVertexPos; }
    [[nodiscard]] Geom::Point3D GetSelectedEdgeP0() const noexcept { return m_selectedEdgeP0; }
    [[nodiscard]] Geom::Point3D GetSelectedEdgeP1() const noexcept { return m_selectedEdgeP1; }

    void OpenConflictPrompt(uint64_t nodeId) noexcept {
        m_conflictedNodeId = nodeId;
        m_showConflictPrompt = true;
    }

private:
    void RenderTopBar(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes,
        std::shared_ptr<DAGNode> selectedNode
    );

    void RenderSketchPlanePromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderExtrudePromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderCutExtrudePromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderRevolvePromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderCutRevolvePromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderFilletPromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderChamferPromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderSketchCanvas(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        std::shared_ptr<SketchNode> activeSketch
    );

    void RenderConflictPromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderEmptySketchPromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange
    );

    void RenderDimensionEditModal();
    static void EnforceShapeConstraints(CanvasShape& shape, int pinnedVertexIdx = -1);

    bool m_initialized{false};
    RHI::RHIDevice* m_rhiDevice{nullptr};

    // Top Bar prompts & state
    bool m_showSketchPlanePrompt{false};
    bool m_showExtrudePrompt{false};
    bool m_showCutExtrudePrompt{false};
    bool m_showRevolvePrompt{false};
    bool m_showCutRevolvePrompt{false};
    uint64_t m_promptInitialSelectedId{0};

    // UI Navigation Mode
    int m_paletteTab{0}; // 0: 3D Features & Solids, 1: 2D Sketching
    bool m_sketchModeActive{false};
    uint64_t m_activeSketchId{0};

    // 2D Mouse Drawing Tools & Canvas State
    SketchMouseTool m_mouseTool{SketchMouseTool::Rectangle};
    bool m_gridSnapEnable{true};
    float m_gridSnapSize{0.5f};
    float m_canvasPanX{320.0f};
    float m_canvasPanY{240.0f};
    float m_canvasZoom{35.0f}; // Pixels per meter

    bool m_isDrawing{false};
    float m_drawStartU{0.0f};
    float m_drawStartV{0.0f};
    std::vector<std::array<float, 2>> m_inProgressPoints;
    std::vector<std::array<float, 2>> m_inProgressSplineControls;
    int m_draggedShapeIdx{-1};
    int m_draggedVertexIdx{-1}; // -1 while m_draggedShapeIdx >= 0 means the whole shape is being dragged
    std::vector<std::array<float, 2>> m_dragShapeOrigPoints; // shape outline when a whole-shape drag began
    float m_dragShapeStartU{0.0f};
    float m_dragShapeStartV{0.0f};
    float m_dragVertexOrigU{0.0f};
    float m_dragVertexOrigV{0.0f};
    int m_hoveredShapeIdx{-1};
    int m_hoveredVertexIdx{-1};
    int m_hoveredEdgeIdx{-1};
    int m_selectedShapeIdx{-1};
    int m_selectedVertexIdx{-1};

    std::vector<CanvasShape> m_canvasShapes;
    uint64_t m_currentEditingSketchId{0};

    // 2D Sketch Keyboard Dimensions Input State
    float m_kbRectWidth{3.0f};
    float m_kbRectHeight{2.0f};
    float m_kbRectCenterU{0.0f};
    float m_kbRectCenterV{0.0f};
    float m_kbCircleRadius{1.5f};
    float m_kbCircleCenterU{0.0f};
    float m_kbCircleCenterV{0.0f};
    float m_kbLineLength{2.0f};
    float m_kbLineAngle{0.0f};
    float m_kbLineTargetU{2.0f};
    float m_kbLineTargetV{0.0f};
    int m_selectedEdgeSegmentIdx{-1};

    // 2D Sketch Edge Dimension Constraint & In-Place Editing
    bool m_showDimensionEditPopup{false};
    int m_dimEditShapeIdx{-1};
    int m_dimEditEdgeIdx{-1};
    float m_dimEditValue{0.0f};
    bool m_dimEditConstrained{true};
    std::array<float, 2> m_dimEditPopupPos{0.0f, 0.0f};

    // 3D Feature Preview & Pending Confirmation State
    bool m_featurePreviewActive{false};
    PreviewFeatureType m_previewFeatureType{PreviewFeatureType::None};
    std::shared_ptr<DAGNode> m_previewNode{nullptr};
    std::shared_ptr<DAGNode> m_previewTarget{nullptr};
    uint64_t m_previewSavedSelectionId{0};
    int m_previewSavedFaceIndex{-1};
    int m_previewSavedVertexIndex{-1};
    int m_previewSavedEdgeIndex{-1};

    // Primitive creation state
    float m_newCylinderRadius{1.5f};
    float m_newCylinderHeight{3.0f};
    int m_newCylinderSegments{32};
    float m_newSphereRadius{2.0f};
    int m_newSphereRings{16};
    int m_newSphereSectors{32};

    // Sketch creation state
    int m_newSketchPlane{0}; // 0: XZ Ground, 1: XY Front, 2: YZ Right
    int m_newSketchPreset{0}; // 0: Rectangle, 1: Regular Polygon, 2: L-Bracket, 3: Star, 4: T-Beam, 5: Custom
    float m_sketchRectDims[2]{3.0f, 2.0f};
    int m_sketchPolySides{6};
    float m_sketchPolyRadius{2.0f};
    float m_sketchLBracket[3]{3.5f, 3.5f, 1.0f};
    int m_sketchStarPoints{5};
    float m_sketchStarRadii[2]{2.5f, 1.2f};
    float m_sketchTBeam[3]{3.5f, 3.0f, 0.7f};
    float m_newPointCoord[2]{1.0f, 1.0f};

    // Extrude & Revolve feature parameters
    float m_newExtrudeHeight{2.5f};
    int m_selectedSketchIndex{0};
    float m_newRevolveAngle{360.0f};
    int m_newRevolveSegments{32};
    int m_newRevolveAxis{1}; // 0: X-Axis, 1: Y-Axis (Up), 2: Z-Axis
    bool m_newRevolveSolid{true};

    // Selection & Sub-Elements
    uint64_t m_selectedNodeId{0};
    int m_selectedFaceIndex{-1};
    int m_selectedVertexIndex{-1};
    int m_selectedEdgeIndex{-1};
    Geom::Plane m_selectedFacePlane;
    Geom::Point3D m_selectedVertexPos{0.0, 0.0, 0.0};
    Geom::Point3D m_selectedEdgeP0{0.0, 0.0, 0.0};
    Geom::Point3D m_selectedEdgeP1{0.0, 0.0, 0.0};
    uint64_t m_lastPublishedSelectedNodeId{std::numeric_limits<uint64_t>::max()};
    int m_lastPublishedSelectedFaceIndex{-1};
    int m_lastPublishedSelectedVertexIndex{-1};
    int m_lastPublishedSelectedEdgeIndex{-1};
    bool m_selectionScrollPending{false};
    uint32_t m_shapeCounter{1};

    // Reference Plane & Axis creation parameters
    float m_newRefPlaneOffset{0.0f};
    float m_newRefAxisOffset{0.0f};

    // Fillet & Chamfer feature parameters
    enum class FilletDimensionChoice { Distance, Angle };
    FilletDimensionChoice m_filletChoice{FilletDimensionChoice::Distance};
    float m_newFilletRadius{0.2f};
    float m_newFilletAngle{45.0f};
    float m_newChamferDistance{0.2f};
    bool m_showFilletPrompt{false};
    bool m_showChamferPrompt{false};
    float m_sketchFilletRadius{0.2f};
    float m_sketchFilletAngle{45.0f};
    float m_sketchChamferDist{0.2f};

    // Conflict Resolution Modal
    bool m_showConflictPrompt{false};
    uint64_t m_conflictedNodeId{0};

    // Empty Sketch Exit Confirmation Modal
    bool m_showEmptySketchExitPrompt{false};
    uint64_t m_emptySketchToExitId{0};

    // Viewport display toggles
    bool m_showWireframe{true};
    bool m_showGrid{true};
    bool m_showReferenceFeature{true};
    bool m_showAxesReference{true};
    float m_surfaceColor[4]{0.72f, 0.76f, 0.82f, 1.0f};
};

} // namespace Tessera


