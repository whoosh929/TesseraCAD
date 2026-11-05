#pragma once

#include "Camera.hpp"
#include "MeshRenderProxy.hpp"
#include "CADShaders.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

namespace Tessera {

class ViewportRenderer {
public:
    ViewportRenderer(RHI::RHIDevice* device, AtomicMeshExchange& meshExchange);
    ~ViewportRenderer();

    ViewportRenderer(const ViewportRenderer&) = delete;
    ViewportRenderer& operator=(const ViewportRenderer&) = delete;

    /// Spawns the dedicated 120 FPS presentation loop thread
    void Start();

    /// Stops the render thread
    void Stop();

    /// Thread-safe camera inputs from UI / Windowing thread
    void OrbitCamera(float deltaAzimuth, float deltaElevation);
    void PanCamera(float deltaX, float deltaY);
    void ZoomCamera(float deltaDistance);
    void OnResize(uint32_t width, uint32_t height);

    [[nodiscard]] double GetAverageFrameTimeMs() const noexcept {
        return m_avgFrameTimeMs.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint64_t GetRenderedFrames() const noexcept {
        return m_renderedFrames.load(std::memory_order_relaxed);
    }

private:
    void InitPipelines();
    void RenderLoop(std::stop_token stopToken);

    RHI::RHIDevice* m_device;
    AtomicMeshExchange& m_meshExchange;
    MeshRenderProxy m_renderProxy;

    // CAD Multi-Pass Pipelines
    std::shared_ptr<RHI::RHIPipeline> m_shadedPipeline;
    std::shared_ptr<RHI::RHIPipeline> m_wireframePipeline;
    std::shared_ptr<RHI::RHIPipeline> m_gridPipeline;

    Camera m_camera;
    std::mutex m_cameraMutex;

    std::jthread m_renderThread;
    std::atomic<bool> m_running{false};

    std::atomic<double> m_avgFrameTimeMs{0.0};
    std::atomic<uint64_t> m_renderedFrames{0};

    uint32_t m_width{1920};
    uint32_t m_height{1080};
    std::atomic<bool> m_needsResize{false};
};

} // namespace Tessera

