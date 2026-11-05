#include "ViewportRenderer.hpp"
#include <iostream>
#include <iomanip>

namespace Tessera {

ViewportRenderer::ViewportRenderer(RHI::RHIDevice* device, AtomicMeshExchange& meshExchange)
    : m_device(device),
      m_meshExchange(meshExchange) {}

ViewportRenderer::~ViewportRenderer() {
    Stop();
}

void ViewportRenderer::Start() {
    if (m_running.exchange(true)) {
        return;
    }

    m_renderThread = std::jthread([this](std::stop_token st) {
        RenderLoop(st);
    });
}

void ViewportRenderer::Stop() {
    if (m_running.exchange(false)) {
        m_renderThread.request_stop();
        if (m_renderThread.joinable()) {
            m_renderThread.join();
        }
    }
}

void ViewportRenderer::OrbitCamera(float deltaAzimuth, float deltaElevation) {
    std::lock_guard<std::mutex> lock(m_cameraMutex);
    m_camera.Orbit(deltaAzimuth, deltaElevation);
}

void ViewportRenderer::PanCamera(float deltaX, float deltaY) {
    std::lock_guard<std::mutex> lock(m_cameraMutex);
    m_camera.Pan(deltaX, deltaY);
}

void ViewportRenderer::ZoomCamera(float deltaDistance) {
    std::lock_guard<std::mutex> lock(m_cameraMutex);
    m_camera.Zoom(deltaDistance);
}

void ViewportRenderer::OnResize(uint32_t width, uint32_t height) {
    m_width = width;
    m_height = height;
    m_needsResize.store(true, std::memory_order_release);
}

void ViewportRenderer::RenderLoop(std::stop_token stopToken) {
    std::cout << "[ViewportRenderer] Independent Render Loop active (Target: 120 FPS).\n";

    constexpr auto targetFrameDuration = std::chrono::microseconds(8333); // 120 FPS cadence
    uint64_t lastGenerationId = 0;
    auto lastFrameTime = std::chrono::steady_clock::now();

    std::unique_ptr<RHI::RHICommandList> cmdList;
    if (m_device) {
        cmdList = m_device->CreateCommandList();
    }

    while (!stopToken.stop_requested() && m_running.load(std::memory_order_relaxed)) {
        auto frameStart = std::chrono::steady_clock::now();

        // 1. Handle resize if requested
        if (m_needsResize.exchange(false, std::memory_order_acquire)) {
            if (m_device) {
                m_device->ResizeSwapchain(m_width, m_height);
            }
            std::lock_guard<std::mutex> lock(m_cameraMutex);
            m_camera.SetViewportSize(static_cast<float>(m_width), static_cast<float>(m_height));
        }

        // 2. Lock-free acquire of latest published mesh from Worker Pool
        std::shared_ptr<RenderMeshData> latestMesh = m_meshExchange.AcquireLatest();
        if (latestMesh && latestMesh->generationId != lastGenerationId) {
            lastGenerationId = latestMesh->generationId;
            m_renderProxy.Update(
                latestMesh->vertexBuffer,
                latestMesh->indexBuffer,
                latestMesh->indexCount,
                latestMesh->generationId
            );
            std::cout << "  [ViewportRenderer] Render proxy swapped to Gen " 
                      << lastGenerationId << " (" << latestMesh->indexCount << " indices)\n";
        }

        // 3. Acquire camera uniforms under short lock
        CameraUniforms cameraUniforms;
        {
            std::lock_guard<std::mutex> lock(m_cameraMutex);
            cameraUniforms = m_camera.GetUniforms();
        }

        // 4. Record and present frame if hardware device is active
        if (m_device && cmdList) {
            if (m_device->BeginFrame()) {
                cmdList->Begin();

                RHI::RenderPassDesc passDesc{
                    .clearColor = {0.12f, 0.12f, 0.15f, 1.0f},
                    .clearDepth = 1.0f,
                    .clearColorBuffer = true,
                    .clearDepthBuffer = true
                };

                cmdList->BeginRenderPass(passDesc);

                RHI::Viewport vp{
                    .x = 0.0f,
                    .y = 0.0f,
                    .width = static_cast<float>(m_width),
                    .height = static_cast<float>(m_height),
                    .minDepth = 0.0f,
                    .maxDepth = 1.0f
                };
                cmdList->SetViewport(vp);

                RHI::ScissorRect scissor{
                    .x = 0,
                    .y = 0,
                    .width = m_width,
                    .height = m_height
                };
                cmdList->SetScissor(scissor);

                // Push camera transform directly into command buffer stream
                cmdList->SetPushConstants(
                    RHI::ShaderStage::Vertex, 
                    &cameraUniforms, 
                    sizeof(cameraUniforms)
                );

                // Draw current render proxy mesh if valid
                if (m_renderProxy.IsValid()) {
                    cmdList->SetVertexBuffer(0, m_renderProxy.GetVertexBuffer());
                    cmdList->SetIndexBuffer(m_renderProxy.GetIndexBuffer(), 0, RHI::IndexFormat::UInt32);
                    cmdList->DrawIndexed(m_renderProxy.GetIndexCount());
                }

                cmdList->EndRenderPass();
                cmdList->End();

                m_device->Submit(cmdList.get());
                m_device->PresentFrame();
            }
        }

        // 5. Cadence timing & unblocked metrics
        uint64_t frames = m_renderedFrames.fetch_add(1, std::memory_order_relaxed) + 1;
        auto frameEnd = std::chrono::steady_clock::now();
        auto frameDuration = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - lastFrameTime);
        lastFrameTime = frameEnd;

        double frameTimeMs = frameDuration.count() / 1000.0;
        m_avgFrameTimeMs.store(frameTimeMs, std::memory_order_relaxed);

        if (frames % 40 == 0) {
            std::cout << "  [Viewport Frame #" << std::setw(4) << frames 
                      << "] Cadence: " << std::fixed << std::setprecision(2) << frameTimeMs 
                      << " ms | Active Gen: " << lastGenerationId << "\n";
        }

        auto activeDuration = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - frameStart);
        if (activeDuration < targetFrameDuration) {
            std::this_thread::sleep_for(targetFrameDuration - activeDuration);
        }
    }

    std::cout << "[ViewportRenderer] Independent Render Loop terminated.\n";
}

} // namespace Tessera
