#pragma once

#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/CylinderNode.hpp"
#include "engine/graph/SphereNode.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/RevolveNode.hpp"
#include "engine/graph/SweepNode.hpp"
#include "engine/graph/DatumPlaneNode.hpp"
#include "engine/graph/DatumAxisNode.hpp"
#include "engine/graph/FilletNode.hpp"
#include "engine/graph/ChamferNode.hpp"
#include "engine/sketch/SketchFilletChamfer.hpp"
#include "engine/export/StepExporter.hpp"
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
    std::map<size_t, VertexAxisConstraint> vertexConstraints; // vertex index -> axis/origin constraints

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

    // Origin (0, 0) Constraints
    [[nodiscard]] bool IsVertexOriginConstrained(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return it != vertexConstraints.end() && it->second.lockOriginDist;
    }

    [[nodiscard]] float GetVertexOriginDistance(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return (it != vertexConstraints.end() && it->second.lockOriginDist) ? it->second.originDist : 0.0f;
    }

    void SetVertexOriginConstraint(size_t vIdx, float dist) {
        if (dist > 0.0001f) {
            vertexConstraints[vIdx].lockOriginDist = true;
            vertexConstraints[vIdx].originDist = dist;
        }
    }

    void RemoveVertexOriginConstraint(size_t vIdx) {
        auto it = vertexConstraints.find(vIdx);
        if (it != vertexConstraints.end()) {
            it->second.lockOriginDist = false;
            it->second.originDist = 0.0f;
            if (!it->second.HasAnyConstraint()) {
                vertexConstraints.erase(it);
            }
        }
    }

    void ToggleVertexOriginConstraint(size_t vIdx, float curDist) {
        if (IsVertexOriginConstrained(vIdx)) {
            RemoveVertexOriginConstraint(vIdx);
        } else {
            SetVertexOriginConstraint(vIdx, curDist);
        }
    }

    // U-Axis (V=0, |V| = dist) Constraints
    [[nodiscard]] bool IsVertexUAxisConstrained(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return it != vertexConstraints.end() && it->second.lockUAxisDist;
    }

    [[nodiscard]] float GetVertexUAxisDistance(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return (it != vertexConstraints.end() && it->second.lockUAxisDist) ? it->second.uAxisDist : 0.0f;
    }

    void SetVertexUAxisConstraint(size_t vIdx, float dist) {
        vertexConstraints[vIdx].lockUAxisDist = true;
        vertexConstraints[vIdx].uAxisDist = dist;
    }

    void RemoveVertexUAxisConstraint(size_t vIdx) {
        auto it = vertexConstraints.find(vIdx);
        if (it != vertexConstraints.end()) {
            it->second.lockUAxisDist = false;
            it->second.uAxisDist = 0.0f;
            if (!it->second.HasAnyConstraint()) {
                vertexConstraints.erase(it);
            }
        }
    }

    // V-Axis (U=0, |U| = dist) Constraints
    [[nodiscard]] bool IsVertexVAxisConstrained(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return it != vertexConstraints.end() && it->second.lockVAxisDist;
    }

    [[nodiscard]] float GetVertexVAxisDistance(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return (it != vertexConstraints.end() && it->second.lockVAxisDist) ? it->second.vAxisDist : 0.0f;
    }

    void SetVertexVAxisConstraint(size_t vIdx, float dist) {
        vertexConstraints[vIdx].lockVAxisDist = true;
        vertexConstraints[vIdx].vAxisDist = dist;
    }

    void RemoveVertexVAxisConstraint(size_t vIdx) {
        auto it = vertexConstraints.find(vIdx);
        if (it != vertexConstraints.end()) {
            it->second.lockVAxisDist = false;
            it->second.vAxisDist = 0.0f;
            if (!it->second.HasAnyConstraint()) {
                vertexConstraints.erase(it);
            }
        }
    }

    // Datum Axis Line Constraints
    [[nodiscard]] bool IsVertexDatumAxisConstrained(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return it != vertexConstraints.end() && it->second.lockDatumAxisDist;
    }

    [[nodiscard]] float GetVertexDatumAxisDistance(size_t vIdx) const {
        auto it = vertexConstraints.find(vIdx);
        return (it != vertexConstraints.end() && it->second.lockDatumAxisDist) ? it->second.datumAxisDist : 0.0f;
    }

    void SetVertexDatumAxisConstraint(size_t vIdx, float dist, uint64_t axisId, std::string axisName) {
        vertexConstraints[vIdx].lockDatumAxisDist = true;
        vertexConstraints[vIdx].datumAxisDist = dist;
        vertexConstraints[vIdx].datumAxisId = axisId;
        vertexConstraints[vIdx].datumAxisName = std::move(axisName);
    }

    void RemoveVertexDatumAxisConstraint(size_t vIdx) {
        auto it = vertexConstraints.find(vIdx);
        if (it != vertexConstraints.end()) {
            it->second.lockDatumAxisDist = false;
            it->second.datumAxisDist = 0.0f;
            it->second.datumAxisId = 0;
            it->second.datumAxisName.clear();
            if (!it->second.HasAnyConstraint()) {
                vertexConstraints.erase(it);
            }
        }
    }
};

enum class DimEditTarget : uint8_t {
    Edge = 0,
    VertexOrigin = 1,
    VertexUAxis = 2,
    VertexVAxis = 3,
    VertexDatumAxis = 4
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
    enum class PreviewFeatureType { None, Extrude, Revolve, Sweep, Fillet, Chamfer, CutExtrude, CutRevolve, CutSweep };
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
        CloseMeasurement();
        m_sketchModeActive = true; 
        m_activeSketchId = sketchId;
        m_paletteTab = 1; // Switch to 2D Sketching tab
        m_mouseTool = SketchMouseTool::SelectMove;
        m_currentEditingSketchId = 0;
    }
    void ExitSketchMode() noexcept { 
        m_sketchModeActive = false; 
        m_currentEditingSketchId = 0;
    }
    [[nodiscard]] SketchMouseTool GetSketchMouseTool() const noexcept { return m_mouseTool; }
    void SetSketchMouseTool(SketchMouseTool tool) noexcept { m_mouseTool = tool; }
    [[nodiscard]] uint64_t GetActiveSketchId() const noexcept { return m_activeSketchId; }

    [[nodiscard]] static std::shared_ptr<SketchNode> FindAssociatedSketch(
        const std::shared_ptr<DAGNode>& feature,
        const ParametricGraph& graph
    ) {
        auto res = graph.FindAssociatedSketch(feature);
        return std::dynamic_pointer_cast<SketchNode>(res);
    }

    // 3D Measurement Tool
    [[nodiscard]] bool IsMeasureToolActive() const noexcept { return m_measureToolActive; }
    [[nodiscard]] bool IsMeasureDialogOpen() const noexcept { return m_showMeasureDialog; }
    void CloseMeasurement() noexcept {
        m_measureToolActive = false;
        m_showMeasureDialog = false;
        ClearMeasurePoints();
        ClearMeasureCircle();
    }
    void SetMeasureToolActive(bool active) noexcept { 
        m_measureToolActive = active; 
        if (active) {
            m_showMeasureDialog = true;
            if (m_selectedVertexPositions.size() >= 2) {
                m_measureP1 = m_selectedVertexPositions[0];
                m_measureHasP1 = true;
                m_measureP2 = m_selectedVertexPositions[1];
                m_measureHasP2 = true;
            }
        } else {
            CloseMeasurement();
        }
    }
    void ToggleMeasureTool() noexcept { 
        if (m_measureToolActive || m_showMeasureDialog || m_measureHasP1 || m_measureHasCircle) {
            CloseMeasurement();
        } else {
            SetMeasureToolActive(true);
        }
    }
    void SetMeasurePoint1(const Geom::Point3D& p) noexcept {
        m_measureP1 = p;
        m_measureHasP1 = true;
    }
    void SetMeasurePoint2(const Geom::Point3D& p) noexcept {
        m_measureP2 = p;
        m_measureHasP2 = true;
    }
    void ClearMeasurePoints() noexcept {
        m_measureHasP1 = false;
        m_measureHasP2 = false;
    }
    void SwapMeasurePoints() noexcept {
        if (m_measureHasP1 && m_measureHasP2) {
            std::swap(m_measureP1, m_measureP2);
        }
    }
    [[nodiscard]] bool HasMeasurePoint1() const noexcept { return m_measureHasP1; }
    [[nodiscard]] bool HasMeasurePoint2() const noexcept { return m_measureHasP2; }
    [[nodiscard]] const Geom::Point3D& GetMeasurePoint1() const noexcept { return m_measureP1; }
    [[nodiscard]] const Geom::Point3D& GetMeasurePoint2() const noexcept { return m_measureP2; }
    [[nodiscard]] Geom::MeasurementResult GetActiveMeasurement() const noexcept {
        if (m_measureHasP1 && m_measureHasP2) {
            return Geom::ComputeMeasurement(m_measureP1, m_measureP2);
        }
        return {};
    }

    [[nodiscard]] bool HasMeasureCircle() const noexcept { return m_measureHasCircle; }
    [[nodiscard]] const CircularEdgeInfo& GetMeasureCircle() const noexcept { return m_measuredCircle; }
    void SetMeasureCircle(const CircularEdgeInfo& c) noexcept {
        m_measuredCircle = c;
        m_measureHasCircle = c.isCircle;
        if (c.isCircle) {
            m_showMeasureDialog = true;
            m_measureToolActive = true;
        }
    }
    void ClearMeasureCircle() noexcept {
        m_measureHasCircle = false;
        m_measuredCircle = {};
    }
    void ToggleMeasureCircleRadiusMode() noexcept {
        m_measureDisplayRadius = !m_measureDisplayRadius;
    }
    [[nodiscard]] bool IsMeasureCircleRadiusMode() const noexcept { return m_measureDisplayRadius; }

    [[nodiscard]] uint64_t GetSelectedNodeId() const noexcept { return m_selectedNodeId; }
    void SetSelectedNodeId(uint64_t id) noexcept {
        if (m_selectedNodeId != id) {
            m_selectedNodeId = id;
            m_selectedFaceIndex = -1;
            m_selectedVertexIndex = -1;
            m_selectedEdgeIndex = -1;
            m_selectedFaceIndices.clear();
            m_selectedFacePlanes.clear();
            m_selectedVertexIndices.clear();
            m_selectedVertexPositions.clear();
            m_selectionScrollPending = true;
        }
    }

    void SetSelectedVertex(int vIdx, Geom::Point3D pos) noexcept {
        m_selectedVertexIndex = vIdx;
        m_selectedVertexPos = pos;
        m_selectedVertexIndices = {vIdx};
        m_selectedVertexPositions = {pos};
        m_selectedFaceIndex = -1;
        m_selectedFaceIndices.clear();
        m_selectedFacePlanes.clear();
        m_selectedEdgeIndex = -1;
    }

    void ToggleSelectedVertex(int vIdx, Geom::Point3D pos) noexcept {
        auto it = std::find(m_selectedVertexIndices.begin(), m_selectedVertexIndices.end(), vIdx);
        if (it != m_selectedVertexIndices.end()) {
            size_t idx = static_cast<size_t>(std::distance(m_selectedVertexIndices.begin(), it));
            m_selectedVertexIndices.erase(it);
            if (idx < m_selectedVertexPositions.size()) {
                m_selectedVertexPositions.erase(m_selectedVertexPositions.begin() + idx);
            }
        } else {
            m_selectedVertexIndices.push_back(vIdx);
            m_selectedVertexPositions.push_back(pos);
        }
        m_selectedVertexIndex = m_selectedVertexIndices.empty() ? -1 : m_selectedVertexIndices.back();
        if (!m_selectedVertexPositions.empty()) {
            m_selectedVertexPos = m_selectedVertexPositions.back();
        }
        m_selectedFaceIndex = -1;
        m_selectedFaceIndices.clear();
        m_selectedFacePlanes.clear();
        m_selectedEdgeIndex = -1;
    }

    void SetSelectedFace(int fIdx, Geom::Plane plane) noexcept {
        m_selectedFaceIndex = fIdx;
        m_selectedFacePlane = plane;
        m_selectedFaceIndices = {fIdx};
        m_selectedFacePlanes = {plane};
        m_selectedVertexIndex = -1;
        m_selectedVertexIndices.clear();
        m_selectedVertexPositions.clear();
        m_selectedEdgeIndex = -1;
    }

    void ToggleSelectedFace(int fIdx, Geom::Plane plane) noexcept {
        auto it = std::find(m_selectedFaceIndices.begin(), m_selectedFaceIndices.end(), fIdx);
        if (it != m_selectedFaceIndices.end()) {
            size_t idx = static_cast<size_t>(std::distance(m_selectedFaceIndices.begin(), it));
            m_selectedFaceIndices.erase(it);
            if (idx < m_selectedFacePlanes.size()) {
                m_selectedFacePlanes.erase(m_selectedFacePlanes.begin() + idx);
            }
        } else {
            m_selectedFaceIndices.push_back(fIdx);
            m_selectedFacePlanes.push_back(plane);
        }
        m_selectedFaceIndex = m_selectedFaceIndices.empty() ? -1 : m_selectedFaceIndices.back();
        if (!m_selectedFacePlanes.empty()) {
            m_selectedFacePlane = m_selectedFacePlanes.back();
        }
        m_selectedVertexIndex = -1;
        m_selectedVertexIndices.clear();
        m_selectedVertexPositions.clear();
        m_selectedEdgeIndex = -1;
    }

    void SetSelectedEdge(int eIdx, Geom::Point3D p0, Geom::Point3D p1, const CircularEdgeInfo& circle = {}) noexcept {
        m_selectedEdgeIndex = eIdx;
        m_selectedEdgeP0 = p0;
        m_selectedEdgeP1 = p1;
        m_selectedCircle = circle;
        m_selectedEdgeIndices = circle.isCircle ? circle.segments : (eIdx >= 0 ? std::vector<int>{eIdx} : std::vector<int>{});
        m_selectedFaceIndex = -1;
        m_selectedFaceIndices.clear();
        m_selectedFacePlanes.clear();
        m_selectedVertexIndex = -1;
        m_selectedVertexIndices.clear();
        m_selectedVertexPositions.clear();
    }

    void ClearSubElementSelection() noexcept {
        m_selectedFaceIndex = -1;
        m_selectedVertexIndex = -1;
        m_selectedEdgeIndex = -1;
        m_selectedFaceIndices.clear();
        m_selectedFacePlanes.clear();
        m_selectedVertexIndices.clear();
        m_selectedVertexPositions.clear();
        m_selectedCircle = {};
        m_selectedEdgeIndices.clear();
    }

    [[nodiscard]] const CircularEdgeInfo& GetSelectedCircle() const noexcept { return m_selectedCircle; }
    [[nodiscard]] const std::vector<int>& GetSelectedEdgeIndices() const noexcept { return m_selectedEdgeIndices; }
    void SetSelectedCircle(const CircularEdgeInfo& circle) noexcept {
        m_selectedCircle = circle;
        m_selectedEdgeIndices = circle.isCircle ? circle.segments : (m_selectedEdgeIndex >= 0 ? std::vector<int>{m_selectedEdgeIndex} : std::vector<int>{});
    }

    [[nodiscard]] int GetSelectedFaceIndex() const noexcept { return m_selectedFaceIndex; }
    [[nodiscard]] int GetSelectedVertexIndex() const noexcept { return m_selectedVertexIndex; }
    [[nodiscard]] int GetSelectedEdgeIndex() const noexcept { return m_selectedEdgeIndex; }
    [[nodiscard]] Geom::Plane GetSelectedFacePlane() const noexcept { return m_selectedFacePlane; }
    [[nodiscard]] Geom::Point3D GetSelectedVertexPos() const noexcept { return m_selectedVertexPos; }
    [[nodiscard]] Geom::Point3D GetSelectedEdgeP0() const noexcept { return m_selectedEdgeP0; }
    [[nodiscard]] Geom::Point3D GetSelectedEdgeP1() const noexcept { return m_selectedEdgeP1; }

    [[nodiscard]] const std::vector<int>& GetSelectedVertexIndices() const noexcept { return m_selectedVertexIndices; }
    [[nodiscard]] const std::vector<Geom::Point3D>& GetSelectedVertexPositions() const noexcept { return m_selectedVertexPositions; }
    [[nodiscard]] const std::vector<int>& GetSelectedFaceIndices() const noexcept { return m_selectedFaceIndices; }
    [[nodiscard]] const std::vector<Geom::Plane>& GetSelectedFacePlanes() const noexcept { return m_selectedFacePlanes; }

    void SetSelectionResolutionPreset(SelectionResolutionPreset preset) noexcept {
        m_selectionResPreset = preset;
        switch (preset) {
            case SelectionResolutionPreset::UltraFine:
                m_pointPickTolerance = 0.05;
                m_edgePickTolerance = 0.05;
                break;
            case SelectionResolutionPreset::Fine:
                m_pointPickTolerance = 0.12;
                m_edgePickTolerance = 0.12;
                break;
            case SelectionResolutionPreset::Medium:
                m_pointPickTolerance = 0.25;
                m_edgePickTolerance = 0.25;
                break;
            case SelectionResolutionPreset::Coarse:
                m_pointPickTolerance = 0.45;
                m_edgePickTolerance = 0.45;
                break;
            case SelectionResolutionPreset::Custom:
                break;
        }
    }

    [[nodiscard]] SelectionResolutionPreset GetSelectionResolutionPreset() const noexcept { return m_selectionResPreset; }
    [[nodiscard]] double GetPointPickTolerance() const noexcept { return m_pointPickTolerance; }
    void SetPointPickTolerance(double tol) noexcept { m_pointPickTolerance = tol; m_selectionResPreset = SelectionResolutionPreset::Custom; }
    [[nodiscard]] double GetEdgePickTolerance() const noexcept { return m_edgePickTolerance; }
    void SetEdgePickTolerance(double tol) noexcept { m_edgePickTolerance = tol; m_selectionResPreset = SelectionResolutionPreset::Custom; }
    [[nodiscard]] SelectionFilter GetSelectionFilter() const noexcept { return m_selectionFilter; }
    void SetSelectionFilter(SelectionFilter filter) noexcept { m_selectionFilter = filter; }

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

    void RenderSweepPromptModal(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        const std::vector<std::shared_ptr<DAGNode>>& nodes
    );

    void RenderDimensionEditModal();
    static void EnforceShapeConstraints(CanvasShape& shape, int pinnedVertexIdx = -1);

    void RenderMeasureToolDialog();
    void RenderMeasureVisualOverlay(Camera& camera, ViewportRenderer& renderer);

    void RenderExportStepModal(ParametricGraph& graph, std::shared_ptr<DAGNode> selectedNode);

    // Target Solid Resolution & Cut Preview Overlay
    [[nodiscard]] std::shared_ptr<DAGNode> ResolveTargetSolid(const std::shared_ptr<DAGNode>& featureOrSketch, const ParametricGraph& graph) const;
    void RenderCutPreviewVisualOverlay(Camera& camera, ViewportRenderer& renderer);

    bool m_initialized{false};
    RHI::RHIDevice* m_rhiDevice{nullptr};

    // Top Bar prompts & state
    bool m_showSketchPlanePrompt{false};
    bool m_showExtrudePrompt{false};
    bool m_showCutExtrudePrompt{false};
    bool m_showRevolvePrompt{false};
    bool m_showCutRevolvePrompt{false};
    bool m_showSweepPrompt{false};
    uint64_t m_selectedSweepProfileSketchId{0};
    uint64_t m_selectedSweepPathSketchId{0};
    uint64_t m_promptInitialSelectedId{0};

    // UI Navigation Mode
    int m_paletteTab{0}; // 0: 3D Features & Solids, 1: 2D Sketching
    bool m_sketchModeActive{false};
    uint64_t m_activeSketchId{0};

    // 2D Mouse Drawing Tools & Canvas State
    SketchMouseTool m_mouseTool{SketchMouseTool::SelectMove};
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

    // 2D Sketch Edge / Axis Dimension Constraint & In-Place Editing
    bool m_showDimensionEditPopup{false};
    DimEditTarget m_dimEditTarget{DimEditTarget::Edge};
    int m_dimEditShapeIdx{-1};
    int m_dimEditEdgeIdx{-1};
    int m_dimEditVertexIdx{-1};
    uint64_t m_dimEditDatumAxisId{0};
    std::string m_dimEditDatumAxisName;
    float m_dimEditValue{0.0f};
    bool m_dimEditConstrained{true};
    std::array<float, 2> m_dimEditPopupPos{0.0f, 0.0f};
    std::vector<ProjectedDatumAxis> m_cachedDatumAxes;

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
    CircularEdgeInfo m_selectedCircle;
    std::vector<int> m_selectedEdgeIndices;
    std::vector<int> m_selectedFaceIndices;
    std::vector<Geom::Plane> m_selectedFacePlanes;
    std::vector<int> m_selectedVertexIndices;
    std::vector<Geom::Point3D> m_selectedVertexPositions;
    SelectionResolutionPreset m_selectionResPreset{SelectionResolutionPreset::Medium};
    double m_pointPickTolerance{0.25};
    double m_edgePickTolerance{0.25};
    SelectionFilter m_selectionFilter{SelectionFilter::All};
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

    // 3D Measurement Tool State
    bool m_measureToolActive{false};
    bool m_showMeasureDialog{false};
    bool m_measureHasP1{false};
    bool m_measureHasP2{false};
    bool m_measureHasCircle{false};
    CircularEdgeInfo m_measuredCircle;
    bool m_measureDisplayRadius{false};
    Geom::Point3D m_measureP1{0.0, 0.0, 0.0};
    Geom::Point3D m_measureP2{0.0, 0.0, 0.0};

    // STEP Export Modal & State
    bool m_showExportStepModal{false};
    char m_exportStepPath[260]{"model.stp"};
    int m_exportStepScope{0}; // 0: Entire Model (All Solids), 1: Selected Solid Only
    std::string m_exportStepStatus;
    std::string m_lastExportedStepFullPath;
    bool m_exportStepSuccess{false};

    // Viewport display toggles
    bool m_showWireframe{true};
    bool m_showGrid{true};
    bool m_showReferenceFeature{true};
    bool m_showAxesReference{true};
    float m_surfaceColor[4]{0.72f, 0.76f, 0.82f, 1.0f};
};

} // namespace Tessera


