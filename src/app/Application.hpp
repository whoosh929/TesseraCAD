#pragma once

#include "engine/hal/RHI.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/concurrency/ThreadPool.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/BoxNode.hpp"
#include "engine/render/ViewportRenderer.hpp"
#include <memory>
#include <atomic>

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
    std::atomic<bool> m_running{true};

    // Subsystems
    std::shared_ptr<ThreadPool> m_threadPool;
    std::unique_ptr<RHI::RHIDevice> m_rhiDevice;
    AtomicMeshExchange m_meshExchange;
    std::unique_ptr<ParametricGraph> m_graph;
    std::shared_ptr<BoxNode> m_testBoxNode;
    std::unique_ptr<ViewportRenderer> m_renderer;
};

} // namespace Tessera
