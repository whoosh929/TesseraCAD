#pragma once

#include "engine/hal/RHI.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <functional>

namespace Tessera {

struct BoxParams {
    float width{1.0f};
    float height{1.0f};
    float depth{1.0f};
    uint64_t generationId{0};
};

class Application {
public:
    Application();
    ~Application();

    // Non-copyable
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /// Runs the decoupled engine lifecycle
    void Run();

    /// Signals the geometry worker to compute a new parametric box
    void TriggerParametricRebuild(float width, float height, float depth);

    /// Stops all running threads cleanly
    void Stop();

private:
    // Worker loop running on dedicated std::jthread
    void GeometryWorkerLoop(std::stop_token stopToken);

    // Render loop running unblocked at 60/120 Hz
    void RenderLoop(std::stop_token stopToken);

    // Evaluates box geometry (mock 100ms CAD tessellation kernel)
    void EvaluateParametricBox(const BoxParams& params);

    std::atomic<bool> m_running{true};

    // Lock-free exchange interface between Worker and Renderer
    AtomicMeshExchange m_meshExchange;

    // Concurrency: atomic task signal for the worker
    std::atomic<bool> m_dirtyGeometry{false};
    std::atomic<uint64_t> m_currentGeneration{0};
    BoxParams m_pendingBoxParams;

    // Worker thread and Render thread (C++20 std::jthread)
    std::jthread m_geometryWorker;
    std::jthread m_renderThread;

    // Camera state updated on the render thread
    struct CameraData {
        float eye[3]{0.0f, 2.0f, 5.0f};
        float target[3]{0.0f, 0.0f, 0.0f};
        float orbitAngle{0.0f};
    } m_camera;
};

} // namespace Tessera
