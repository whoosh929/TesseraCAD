#pragma once

#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/BoxNode.hpp"
#include "engine/graph/CylinderNode.hpp"
#include "engine/graph/SphereNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/render/Camera.hpp"
#include "engine/render/ViewportRenderer.hpp"
#include <memory>
#include <string>

struct GLFWwindow;

namespace Tessera {

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

private:
    bool m_initialized{false};
    RHI::RHIDevice* m_rhiDevice{nullptr};

    // Shape creation state
    int m_selectedShapeType{0}; // 0: Box, 1: Cylinder, 2: Sphere, 3: Extrude
    float m_newBoxDims[3]{2.0f, 1.0f, 3.0f};
    float m_newCylinderRadius{1.5f};
    float m_newCylinderHeight{3.0f};
    int m_newCylinderSegments{32};
    float m_newSphereRadius{2.0f};
    int m_newSphereRings{16};
    int m_newSphereSectors{32};
    int m_selectedExtrudeProfile{0}; // 0: L-Bracket, 1: Hexagon, 2: Star, 3: T-Beam
    float m_newExtrudeHeight{2.5f};

    // Selection
    uint64_t m_selectedNodeId{0};
    uint32_t m_shapeCounter{1};

    // Viewport display toggles
    bool m_showWireframe{true};
    bool m_showGrid{true};
    float m_surfaceColor[4]{0.72f, 0.76f, 0.82f, 1.0f};
};

} // namespace Tessera

