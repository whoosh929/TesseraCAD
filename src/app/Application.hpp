#pragma once

#include "engine/hal/RHI.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/concurrency/ThreadPool.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/BoxNode.hpp"
#include "engine/graph/CylinderNode.hpp"
#include "engine/graph/SphereNode.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/render/ViewportRenderer.hpp"
#include "CADGui.hpp"
#include <memory>
#include <atomic>

struct GLFWwindow;

namespace Tessera {

class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /// Runs the interactive engine demonstration
    void Run();

    /// Modifies parametric box dimensions (dirtying the DAG)
    void SetBoxDimensions(float width, float height, float depth);

    void Stop();

private:
    void InitWindow();
    void SetupCallbacks();

    std::atomic<bool> m_running{true};
    GLFWwindow* m_window{nullptr};
    uint32_t m_width{1600};
    uint32_t m_height{900};

    // Subsystems
    std::shared_ptr<ThreadPool> m_threadPool;
    std::unique_ptr<RHI::RHIDevice> m_rhiDevice;
    AtomicMeshExchange m_meshExchange;
    std::unique_ptr<ParametricGraph> m_graph;
    std::shared_ptr<BoxNode> m_testBoxNode;
    std::shared_ptr<CylinderNode> m_testCylinderNode;
    std::shared_ptr<ExtrudeNode> m_testExtrudeNode;
    std::shared_ptr<SphereNode> m_testSphereNode;
    std::unique_ptr<ViewportRenderer> m_renderer;
    std::unique_ptr<CADGui> m_gui;

    // Mouse interaction tracking
    bool m_leftMouseDown{false};
    bool m_rightMouseDown{false};
    double m_lastMouseX{0.0};
    double m_lastMouseY{0.0};
};

} // namespace Tessera
