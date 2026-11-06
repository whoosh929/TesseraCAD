#pragma once

#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/BoxNode.hpp"
#include "engine/graph/CylinderNode.hpp"
#include "engine/graph/SphereNode.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/RevolveNode.hpp"
#include "engine/graph/DatumPlaneNode.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/render/Camera.hpp"
#include "engine/render/ViewportRenderer.hpp"
#include <memory>
#include <string>
#include <vector>

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

    [[nodiscard]] bool IsSketchModeActive() const noexcept { return m_sketchModeActive; }
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
            m_selectionScrollPending = true;
        }
    }

private:
    void RenderSketchCanvas(
        ParametricGraph& graph,
        RHI::RHIDevice* device,
        AtomicMeshExchange& exchange,
        std::shared_ptr<SketchNode> activeSketch
    );

    bool m_initialized{false};
    RHI::RHIDevice* m_rhiDevice{nullptr};

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
    int m_draggedVertexIdx{-1};
    int m_hoveredShapeIdx{-1};
    int m_hoveredVertexIdx{-1};

    std::vector<CanvasShape> m_canvasShapes;
    uint64_t m_currentEditingSketchId{0};

    // Shape creation state
    int m_selectedShapeType{0}; // 0: Box, 1: Cylinder, 2: Sphere, 3: Extrude from Sketch, 4: Revolve from Sketch
    float m_newBoxDims[3]{2.0f, 1.0f, 3.0f};
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

    // Selection
    uint64_t m_selectedNodeId{0};
    bool m_selectionScrollPending{false};
    uint32_t m_shapeCounter{1};

    // Viewport display toggles
    bool m_showWireframe{true};
    bool m_showGrid{true};
    float m_surfaceColor[4]{0.72f, 0.76f, 0.82f, 1.0f};
};

} // namespace Tessera


