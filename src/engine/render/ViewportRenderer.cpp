#include "ViewportRenderer.hpp"
#include "engine/graph/GeometryTypes.hpp"
#include "app/CADGui.hpp"
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

void ViewportRenderer::InitPipelines() {
    if (!m_device) return;

    RHI::VertexLayout cadVertexLayout{
        .stride = sizeof(CADVertex), // 24 bytes (3 floats pos + 3 floats normal)
        .attributes = {
            {.location = 0, .format = RHI::VertexFormat::Float3, .offset = 0},
            {.location = 1, .format = RHI::VertexFormat::Float3, .offset = 12}
        }
    };

    // 1. Shaded Surface Pipeline
    RHI::RHIPipelineDesc shadedDesc{
        .vertexShaderSource = CADShaders::Vulkan_ShadedVertex,
        .fragmentShaderSource = CADShaders::Vulkan_ShadedFragment,
        .vertexLayout = cadVertexLayout,
        .topology = RHI::PrimitiveTopology::TriangleList,
        .rasterizer = {
            .cullMode = RHI::CullMode::Back,
            .frontFace = RHI::FrontFace::CounterClockwise,
            .wireframe = false
        },
        .depthStencil = {
            .depthTestEnable = true,
            .depthWriteEnable = true,
            .depthCompareOp = RHI::CompareOp::Less
        },
        .blendState = {.blendEnable = false},
        .debugName = "CAD_Shaded_Pipeline"
    };
    m_shadedPipeline = m_device->CreatePipeline(shadedDesc);

    // 2. Crisp Wireframe / Feature Edge Pipeline
    RHI::RHIPipelineDesc wireframeDesc{
        .vertexShaderSource = CADShaders::Vulkan_WireframeVertex,
        .fragmentShaderSource = CADShaders::Vulkan_WireframeFragment,
        .vertexLayout = cadVertexLayout,
        .topology = RHI::PrimitiveTopology::LineList,
        .rasterizer = {
            .cullMode = RHI::CullMode::None,
            .frontFace = RHI::FrontFace::CounterClockwise,
            .wireframe = true,
            .depthBiasConstant = -1.0f,
            .depthBiasSlope = -1.0f
        },
        .depthStencil = {
            .depthTestEnable = true,
            .depthWriteEnable = true,
            .depthCompareOp = RHI::CompareOp::LessOrEqual
        },
        .blendState = {.blendEnable = false},
        .debugName = "CAD_Wireframe_Pipeline"
    };
    m_wireframePipeline = m_device->CreatePipeline(wireframeDesc);

    // 3. Infinite Ground Grid Pipeline
    RHI::RHIPipelineDesc gridDesc{
        .vertexShaderSource = CADShaders::Vulkan_GridVertex,
        .fragmentShaderSource = CADShaders::Vulkan_GridFragment,
        .vertexLayout = {}, // Fullscreen procedural quad, no vertex buffer input
        .topology = RHI::PrimitiveTopology::TriangleList,
        .rasterizer = {
            .cullMode = RHI::CullMode::None,
            .frontFace = RHI::FrontFace::CounterClockwise,
            .wireframe = false
        },
        .depthStencil = {
            .depthTestEnable = true,
            .depthWriteEnable = false, // Ground grid does not occlude objects behind it
            .depthCompareOp = RHI::CompareOp::Less
        },
        .blendState = {.blendEnable = true},
        .debugName = "CAD_Grid_Pipeline"
    };
    m_gridPipeline = m_device->CreatePipeline(gridDesc);

    std::cout << "[ViewportRenderer] Multi-pass CAD shaders compiled & pipelines ready.\n";
}

void ViewportRenderer::RenderLoop(std::stop_token stopToken) {
    std::cout << "[ViewportRenderer] Independent Render Loop active (Target: 120 FPS).\n";

    InitPipelines();

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
                latestMesh->generationId,
                latestMesh->edgeIndexBuffer,
                latestMesh->edgeIndexCount
            );
            std::cout << "  [ViewportRenderer] Render proxy swapped to Gen " 
                      << lastGenerationId << " (" << latestMesh->indexCount 
                      << " tri indices, " << latestMesh->edgeIndexCount << " edge indices)\n";
        }

        // 3. Acquire camera uniforms under short lock
        CameraUniforms cameraUniforms;
        {
            std::lock_guard<std::mutex> lock(m_cameraMutex);
            cameraUniforms = m_camera.GetUniforms();
        }

        // 4. Record and present multi-pass CAD frame
        if (m_device && cmdList) {
            if (m_device->BeginFrame()) {
                cmdList->Begin();

                RHI::RenderPassDesc passDesc{
                    .clearColor = {0.15f, 0.16f, 0.18f, 1.0f}, // CAD workstation dark slate background
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

                // --- PASS 1: Infinite Ground Grid ---
                if (m_gridPipeline) {
                    cmdList->SetPipeline(m_gridPipeline);
                    CADGridUniforms gridUbo{
                        .viewProjection = cameraUniforms.viewProjection,
                        .invViewProjection = cameraUniforms.invViewProjection,
                        .cameraPosition = cameraUniforms.eyePosition,
                        .nearPlane = 0.1f,
                        .farPlane = 1000.0f,
                        .gridScale = 1.0f
                    };
                    cmdList->SetPushConstants(
                        RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment,
                        &gridUbo,
                        sizeof(gridUbo)
                    );
                    cmdList->Draw(6); // 2 triangles covering ground
                }

                if (m_renderProxy.IsValid()) {
                    // --- PASS 2: Shaded CAD Model Pass ---
                    if (m_shadedPipeline) {
                        cmdList->SetPipeline(m_shadedPipeline);
                        CADModelUniforms modelUbo{
                            .modelViewProjection = cameraUniforms.viewProjection,
                            .cameraPosition = cameraUniforms.eyePosition,
                            .lightDirection = {0.577f, 0.816f, 0.577f, 0.0f},
                            .surfaceColor = {0.72f, 0.76f, 0.82f, 1.0f}, // Clean CAD aluminum
                            .lightingParams = {0.25f, 0.75f, 0.35f, 32.0f}
                        };
                        cmdList->SetPushConstants(
                            RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment,
                            &modelUbo,
                            sizeof(modelUbo)
                        );
                        cmdList->SetVertexBuffer(0, m_renderProxy.GetVertexBuffer());
                        cmdList->SetIndexBuffer(m_renderProxy.GetIndexBuffer(), 0, RHI::IndexFormat::UInt32);
                        cmdList->DrawIndexed(m_renderProxy.GetIndexCount());
                    }

                    // --- PASS 3: Crisp Feature-Edge / Wireframe Overlay ---
                    if (m_wireframePipeline && m_renderProxy.GetEdgeIndexBuffer()) {
                        cmdList->SetPipeline(m_wireframePipeline);
                        CADWireframeUniforms wireUbo{
                            .modelViewProjection = cameraUniforms.viewProjection,
                            .edgeColor = {0.10f, 0.12f, 0.14f, 1.0f} // Dark silhouette edge
                        };
                        cmdList->SetPushConstants(
                            RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment,
                            &wireUbo,
                            sizeof(wireUbo)
                        );
                        cmdList->SetVertexBuffer(0, m_renderProxy.GetVertexBuffer());
                        cmdList->SetIndexBuffer(m_renderProxy.GetEdgeIndexBuffer(), 0, RHI::IndexFormat::UInt32);
                        cmdList->DrawIndexed(m_renderProxy.GetEdgeIndexCount());
                    }
                }

                // --- PASS 4: Dear ImGui CAD Tooling UI ---
                if (m_gui) {
                    m_gui->EndFrameAndDraw(cmdList.get());
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
            std::cout << "  [CAD Viewport Frame #" << std::setw(4) << frames 
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

