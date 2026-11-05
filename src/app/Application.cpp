#include "Application.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <iomanip>

namespace Tessera {

// -----------------------------------------------------------------------------
// Minimal CPU-backed RHIBuffer implementation for mock evaluation & staging
// -----------------------------------------------------------------------------
class MockBuffer final : public RHI::RHIBuffer {
public:
    MockBuffer(size_t size, RHI::BufferUsage usage, RHI::MemoryUsage mem)
        : m_size(size), m_usage(usage), m_memory(mem), m_data(size) {}

    [[nodiscard]] size_t GetSize() const noexcept override { return m_size; }
    [[nodiscard]] RHI::BufferUsage GetUsage() const noexcept override { return m_usage; }
    [[nodiscard]] RHI::MemoryUsage GetMemoryUsage() const noexcept override { return m_memory; }

    [[nodiscard]] void* Map() override { return m_data.data(); }
    void Unmap() override {}

private:
    size_t m_size;
    RHI::BufferUsage m_usage;
    RHI::MemoryUsage m_memory;
    std::vector<uint8_t> m_data;
};

struct Vertex {
    float position[3];
    float normal[3];
};

Application::Application() {
    std::cout << "[TesseraCAD] Initializing Dual-Thread Decoupled CAD Engine...\n";
}

Application::~Application() {
    Stop();
}

void Application::Stop() {
    m_running.store(false, std::memory_order_release);
}

void Application::TriggerParametricRebuild(float width, float height, float depth) {
    m_pendingBoxParams = BoxParams{
        .width = width,
        .height = height,
        .depth = depth,
        .generationId = m_currentGeneration.fetch_add(1, std::memory_order_relaxed) + 1
    };
    m_dirtyGeometry.store(true, std::memory_order_release);
}

void Application::Run() {
    // 1. Spawn Worker Thread for parametric DAG evaluation
    m_geometryWorker = std::jthread([this](std::stop_token st) {
        GeometryWorkerLoop(st);
    });

    // 2. Spawn Render Thread maintaining unblocked presentation
    m_renderThread = std::jthread([this](std::stop_token st) {
        RenderLoop(st);
    });

    // Trigger initial model build
    TriggerParametricRebuild(2.0f, 1.0f, 3.0f);

    // Simulate interactive parametric dimension adjustments from user
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    TriggerParametricRebuild(4.0f, 2.5f, 1.5f);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    TriggerParametricRebuild(1.0f, 6.0f, 2.0f);

    // Run demonstration for 1 second total
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Stop();

    // std::jthread automatically joins on scope exit
}

void Application::GeometryWorkerLoop(std::stop_token stopToken) {
    std::cout << "[Worker Thread] Geometry worker pool online.\n";

    while (!stopToken.stop_requested() && m_running.load(std::memory_order_relaxed)) {
        if (m_dirtyGeometry.exchange(false, std::memory_order_acquire)) {
            BoxParams params = m_pendingBoxParams;
            EvaluateParametricBox(params);
        } else {
            // Idle wait (can be replaced with condition variable or SPSC task queue)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    std::cout << "[Worker Thread] Geometry worker pool shutdown.\n";
}

void Application::EvaluateParametricBox(const BoxParams& params) {
    auto startTime = std::chrono::steady_clock::now();
    std::cout << "\n[Worker Thread] >>> BEGIN heavy CAD evaluation for Generation " 
              << params.generationId << " (Size: " << params.width << "x" 
              << params.height << "x" << params.depth << ")...\n";

    // Simulate heavy 100ms CAD tessellation / B-Rep Boolean operation
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Generate box mesh (24 vertices, 36 indices)
    const float hx = params.width * 0.5f;
    const float hy = params.height * 0.5f;
    const float hz = params.depth * 0.5f;

    std::vector<Vertex> vertices = {
        // Front face (+Z)
        {{-hx, -hy,  hz}, {0.0f, 0.0f, 1.0f}},
        {{ hx, -hy,  hz}, {0.0f, 0.0f, 1.0f}},
        {{ hx,  hy,  hz}, {0.0f, 0.0f, 1.0f}},
        {{-hx,  hy,  hz}, {0.0f, 0.0f, 1.0f}},
        // Back face (-Z)
        {{ hx, -hy, -hz}, {0.0f, 0.0f, -1.0f}},
        {{-hx, -hy, -hz}, {0.0f, 0.0f, -1.0f}},
        {{-hx,  hy, -hz}, {0.0f, 0.0f, -1.0f}},
        {{ hx,  hy, -hz}, {0.0f, 0.0f, -1.0f}},
    };

    std::vector<uint32_t> indices = {
        0, 1, 2,  2, 3, 0, // Front
        4, 5, 6,  6, 7, 4  // Back (minimal demonstration)
    };

    // Upload to GPU buffers (using shared/host-visible memory strategy)
    size_t vbSize = vertices.size() * sizeof(Vertex);
    size_t ibSize = indices.size() * sizeof(uint32_t);

    auto vb = std::make_shared<MockBuffer>(vbSize, RHI::BufferUsage::Vertex, RHI::MemoryUsage::CpuToGpu);
    auto ib = std::make_shared<MockBuffer>(ibSize, RHI::BufferUsage::Index, RHI::MemoryUsage::CpuToGpu);

    vb->UpdateData(vertices.data(), vbSize);
    ib->UpdateData(indices.data(), ibSize);

    // Publish to render loop via lock-free atomic pointer exchange
    m_meshExchange.Publish(vb, ib, static_cast<uint32_t>(indices.size()), params.generationId);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "[Worker Thread] <<< PUBLISHED Generation " << params.generationId 
              << " in " << elapsed << " ms (Lock-Free Swap Complete).\n\n";
}

void Application::RenderLoop(std::stop_token stopToken) {
    std::cout << "[Render Thread] Dedicated Presentation Loop online (Target: 120 FPS).\n";

    constexpr auto targetFrameDuration = std::chrono::microseconds(8333); // ~120 Hz
    uint64_t lastRenderedGeneration = 0;
    uint32_t frameCounter = 0;

    auto lastFrameTime = std::chrono::steady_clock::now();

    while (!stopToken.stop_requested() && m_running.load(std::memory_order_relaxed)) {
        auto frameStart = std::chrono::steady_clock::now();

        // 1. Lock-free acquire of latest published mesh (Zero mutex contention!)
        std::shared_ptr<RenderMeshData> currentMesh = m_meshExchange.AcquireLatest();

        if (currentMesh && currentMesh->generationId != lastRenderedGeneration) {
            lastRenderedGeneration = currentMesh->generationId;
            std::cout << "  [Render Thread] -> Swapped to new mesh generation: " 
                      << lastRenderedGeneration << " (Index count: " << currentMesh->indexCount << ")\n";
        }

        // 2. Continuous camera update (unblocked orbit)
        m_camera.orbitAngle += 0.01f;

        // 3. Record & simulate draw call
        if (currentMesh && currentMesh->vertexBuffer) {
            // In full HAL:
            // cmdList->SetVertexBuffer(0, currentMesh->vertexBuffer);
            // cmdList->SetIndexBuffer(currentMesh->indexBuffer);
            // cmdList->DrawIndexed(currentMesh->indexCount);
        }

        // 4. Measure presentation cadence
        frameCounter++;
        auto frameEnd = std::chrono::steady_clock::now();
        auto frameDelta = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - lastFrameTime).count();
        lastFrameTime = frameEnd;

        // Log sample frame latency every 20 frames
        if (frameCounter % 20 == 0) {
            std::cout << "  [Render Frame #" << std::setw(3) << frameCounter 
                      << "] Cadence: " << std::fixed << std::setprecision(2) 
                      << (frameDelta / 1000.0) << " ms | Active Gen: " 
                      << lastRenderedGeneration << " | Camera Orbit: " 
                      << m_camera.orbitAngle << " rad\n";
        }

        // Maintain fixed presentation rate
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - frameStart);
        if (elapsed < targetFrameDuration) {
            std::this_thread::sleep_for(targetFrameDuration - elapsed);
        }
    }

    std::cout << "[Render Thread] Presentation Loop shutdown.\n";
}

} // namespace Tessera
