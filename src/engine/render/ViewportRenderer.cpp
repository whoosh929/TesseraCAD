#include "ViewportRenderer.hpp"
#include "CADSpvBytecode.hpp"
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

void ViewportRenderer::ZoomCameraCentered(float screenX, float screenY, float scrollDelta, bool smooth) {
    std::lock_guard<std::mutex> lock(m_cameraMutex);
    m_camera.ZoomCentered(screenX, screenY, static_cast<float>(m_width), static_cast<float>(m_height), scrollDelta, smooth);
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
        .spvVertexBytes = SpvBytecode::ShadedVertex,
        .spvVertexSize = SpvBytecode::ShadedVertexSize,
        .spvFragmentBytes = SpvBytecode::ShadedFragment,
        .spvFragmentSize = SpvBytecode::ShadedFragmentSize,
        .vertexLayout = cadVertexLayout,
        .topology = RHI::PrimitiveTopology::TriangleList,
        .rasterizer = {
            .cullMode = RHI::CullMode::None,
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
        .spvVertexBytes = SpvBytecode::WireframeVertex,
        .spvVertexSize = SpvBytecode::WireframeVertexSize,
        .spvFragmentBytes = SpvBytecode::WireframeFragment,
        .spvFragmentSize = SpvBytecode::WireframeFragmentSize,
        .vertexLayout = cadVertexLayout,
        .topology = RHI::PrimitiveTopology::LineList,
        .rasterizer = {
            .cullMode = RHI::CullMode::None,
            .frontFace = RHI::FrontFace::CounterClockwise,
            .wireframe = false,
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
        .spvVertexBytes = SpvBytecode::GridVertex,
        .spvVertexSize = SpvBytecode::GridVertexSize,
        .spvFragmentBytes = SpvBytecode::GridFragment,
        .spvFragmentSize = SpvBytecode::GridFragmentSize,
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
            .depthCompareOp = RHI::CompareOp::LessOrEqual
        },
        .blendState = {.blendEnable = true},
        .debugName = "CAD_Grid_Pipeline"
    };
    m_gridPipeline = m_device->CreatePipeline(gridDesc);

    // 4. Sketch Transparent Shading Pipeline (Alpha-blended planar surface)
    RHI::RHIPipelineDesc sketchShadedDesc{
        .vertexShaderSource = CADShaders::Vulkan_ShadedVertex,
        .fragmentShaderSource = CADShaders::Vulkan_ShadedFragment,
        .spvVertexBytes = SpvBytecode::ShadedVertex,
        .spvVertexSize = SpvBytecode::ShadedVertexSize,
        .spvFragmentBytes = SpvBytecode::ShadedFragment,
        .spvFragmentSize = SpvBytecode::ShadedFragmentSize,
        .vertexLayout = cadVertexLayout,
        .topology = RHI::PrimitiveTopology::TriangleList,
        .rasterizer = {
            .cullMode = RHI::CullMode::None,
            .frontFace = RHI::FrontFace::CounterClockwise,
            .wireframe = false
        },
        .depthStencil = {
            .depthTestEnable = true,
            .depthWriteEnable = false, // Transparent sketch does not occlude objects behind it
            .depthCompareOp = RHI::CompareOp::LessOrEqual
        },
        .blendState = {.blendEnable = true},
        .debugName = "CAD_Sketch_Transparent_Pipeline"
    };
    m_sketchShadedPipeline = m_device->CreatePipeline(sketchShadedDesc);

    std::cout << "[ViewportRenderer] Multi-pass CAD shaders compiled & pipelines ready.\n";
}

void ViewportRenderer::RenderFrame() {
    if (!m_pipelinesInitialized) {
        InitPipelines();
        m_pipelinesInitialized = true;
    }

    if (!m_cmdList && m_device) {
        m_cmdList = m_device->CreateCommandList();
    }

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
    if (latestMesh && latestMesh->generationId != m_lastGenerationId) {
        m_lastGenerationId = latestMesh->generationId;
        m_renderProxy.Update(
            latestMesh->vertexBuffer,
            latestMesh->indexBuffer,
            latestMesh->indexCount,
            latestMesh->generationId,
            latestMesh->edgeIndexBuffer,
            latestMesh->edgeIndexCount,
            latestMesh->sketchVertexBuffer,
            latestMesh->sketchIndexBuffer,
            latestMesh->sketchIndexCount,
            latestMesh->sketchEdgeIndexBuffer,
            latestMesh->sketchEdgeIndexCount,
            latestMesh->selectedVertexBuffer,
            latestMesh->selectedIndexBuffer,
            latestMesh->selectedIndexCount,
            latestMesh->selectedEdgeIndexBuffer,
            latestMesh->selectedEdgeIndexCount
        );
        std::cout << "  [ViewportRenderer] Render proxy swapped to Gen " 
                  << m_lastGenerationId << " (" << latestMesh->indexCount 
                  << " tri indices, " << latestMesh->edgeIndexCount << " edge indices | Selected: "
                  << latestMesh->selectedIndexCount << " tri, " << latestMesh->selectedEdgeIndexCount
                  << " edge | Sketch: " << latestMesh->sketchIndexCount << " tri, " 
                  << latestMesh->sketchEdgeIndexCount << " edge)\n";
    }

    // 3. Acquire camera uniforms under short lock
    CameraUniforms cameraUniforms;
    {
        std::lock_guard<std::mutex> lock(m_cameraMutex);
        cameraUniforms = m_camera.GetUniforms();
    }

    // 4. Record and present frame
    if (m_device && m_cmdList) {
        if (m_device->BeginFrame()) {
            m_cmdList->Begin();

            RHI::RenderPassDesc passDesc{
                .clearColor = {0.14f, 0.15f, 0.18f, 1.0f}, // Dark slate CAD workstation background
                .clearDepth = 1.0f,
                .clearColorBuffer = true,
                .clearDepthBuffer = true
            };

            m_cmdList->BeginRenderPass(passDesc);

            RHI::Viewport vp{
                .x = 0.0f,
                .y = 0.0f,
                .width = static_cast<float>(m_width),
                .height = static_cast<float>(m_height),
                .minDepth = 0.0f,
                .maxDepth = 1.0f
            };
            m_cmdList->SetViewport(vp);

            RHI::ScissorRect scissor{
                .x = 0,
                .y = 0,
                .width = m_width,
                .height = m_height
            };
            m_cmdList->SetScissor(scissor);

            // 1. Draw Infinite Ground Datum Grid (if enabled in UI)
            if ((!m_gui || m_gui->GetShowGrid()) && m_gridPipeline) {
                CADGridUniforms gridUniforms{};
                gridUniforms.viewProjection = cameraUniforms.viewProjection;
                gridUniforms.invViewProjection = cameraUniforms.invViewProjection;
                gridUniforms.cameraPosition = cameraUniforms.eyePosition;
                gridUniforms.nearPlane = 0.1f;
                gridUniforms.farPlane = 1000.0f;
                gridUniforms.gridScale = 1.0f;

                m_cmdList->SetPipeline(m_gridPipeline);
                m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &gridUniforms, sizeof(gridUniforms));
                m_cmdList->Draw(6, 0);
            }

            // 2. Draw unselected 3D CAD meshes (neutral machine aluminum / steel)
            if (m_renderProxy.GetVertexBuffer() && m_renderProxy.GetIndexBuffer() && m_renderProxy.GetIndexCount() > 0 && m_shadedPipeline) {
                CADModelUniforms modelUniforms{};
                modelUniforms.modelViewProjection = cameraUniforms.viewProjection;
                modelUniforms.cameraPosition = cameraUniforms.eyePosition;
                modelUniforms.lightDirection = {0.577f, 0.707f, 0.408f, 0.0f}; // Directional light
                modelUniforms.surfaceColor = {0.68f, 0.74f, 0.82f, 1.0f};      // CAD machine aluminum / steel
                modelUniforms.lightingParams = {0.30f, 0.70f, 0.40f, 32.0f};    // ambient, diff, spec, shininess

                m_cmdList->SetPipeline(m_shadedPipeline);
                m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &modelUniforms, sizeof(modelUniforms));
                m_cmdList->SetVertexBuffer(0, m_renderProxy.GetVertexBuffer());
                m_cmdList->SetIndexBuffer(m_renderProxy.GetIndexBuffer());
                m_cmdList->DrawIndexed(m_renderProxy.GetIndexCount());

                // Unselected sharp feature edges / wireframe overlay (if enabled in UI)
                if ((!m_gui || m_gui->GetShowWireframe()) && m_wireframePipeline && 
                    m_renderProxy.GetEdgeIndexBuffer() && m_renderProxy.GetEdgeIndexCount() > 0) {
                    CADWireframeUniforms wireUniforms{};
                    wireUniforms.modelViewProjection = cameraUniforms.viewProjection;
                    wireUniforms.edgeColor = {0.10f, 0.12f, 0.16f, 1.0f}; // Sharp dark charcoal CAD edge

                    m_cmdList->SetPipeline(m_wireframePipeline);
                    m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &wireUniforms, sizeof(wireUniforms));
                    m_cmdList->SetVertexBuffer(0, m_renderProxy.GetVertexBuffer());
                    m_cmdList->SetIndexBuffer(m_renderProxy.GetEdgeIndexBuffer());
                    m_cmdList->DrawIndexed(m_renderProxy.GetEdgeIndexCount());
                }
            }

            // 3a. Draw SELECTED surface (if any selected triangles exist)
            if (m_renderProxy.GetSelectedVertexBuffer() && m_renderProxy.GetSelectedIndexBuffer() && 
                m_renderProxy.GetSelectedIndexCount() > 0 && m_shadedPipeline) {
                CADModelUniforms selUniforms{};
                selUniforms.modelViewProjection = cameraUniforms.viewProjection;
                selUniforms.cameraPosition = cameraUniforms.eyePosition;
                selUniforms.lightDirection = {0.577f, 0.707f, 0.408f, 0.0f};
                // Subtle selection tint: soft luminous cyan-steel sheen
                selUniforms.surfaceColor = {0.45f, 0.72f, 0.92f, 1.0f};
                selUniforms.lightingParams = {0.42f, 0.68f, 0.55f, 48.0f};

                m_cmdList->SetPipeline(m_shadedPipeline);
                m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &selUniforms, sizeof(selUniforms));
                m_cmdList->SetVertexBuffer(0, m_renderProxy.GetSelectedVertexBuffer());
                m_cmdList->SetIndexBuffer(m_renderProxy.GetSelectedIndexBuffer());
                m_cmdList->DrawIndexed(m_renderProxy.GetSelectedIndexCount());
            }

            // 3b. Highlighted outer feature edges & vertex markers (vibrant electric cyan highlight)
            if (m_wireframePipeline && m_renderProxy.GetSelectedVertexBuffer() && 
                m_renderProxy.GetSelectedEdgeIndexBuffer() && 
                m_renderProxy.GetSelectedEdgeIndexCount() > 0) {
                CADWireframeUniforms selWireUniforms{};
                selWireUniforms.modelViewProjection = cameraUniforms.viewProjection;
                selWireUniforms.edgeColor = {0.0f, 0.88f, 1.0f, 1.0f}; // Vibrant electric cyan selection edge highlight!

                m_cmdList->SetPipeline(m_wireframePipeline);
                m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &selWireUniforms, sizeof(selWireUniforms));
                m_cmdList->SetVertexBuffer(0, m_renderProxy.GetSelectedVertexBuffer());
                m_cmdList->SetIndexBuffer(m_renderProxy.GetSelectedEdgeIndexBuffer());
                m_cmdList->DrawIndexed(m_renderProxy.GetSelectedEdgeIndexCount());
            }

            // 4. Draw standalone selected 2D sketch (visually distinct: outline with transparent shading)
            if (m_renderProxy.GetSketchVertexBuffer() && m_renderProxy.GetSketchIndexBuffer() && 
                m_renderProxy.GetSketchIndexCount() > 0 && m_sketchShadedPipeline) {
                CADModelUniforms sketchUniforms{};
                sketchUniforms.modelViewProjection = cameraUniforms.viewProjection;
                sketchUniforms.cameraPosition = cameraUniforms.eyePosition;
                sketchUniforms.lightDirection = {0.577f, 0.707f, 0.408f, 0.0f};
                // Transparent shading: translucent vibrant sky blue with alpha blending
                sketchUniforms.surfaceColor = {0.12f, 0.65f, 1.0f, 0.28f};
                sketchUniforms.lightingParams = {0.75f, 0.25f, 0.10f, 16.0f};

                m_cmdList->SetPipeline(m_sketchShadedPipeline);
                m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &sketchUniforms, sizeof(sketchUniforms));
                m_cmdList->SetVertexBuffer(0, m_renderProxy.GetSketchVertexBuffer());
                m_cmdList->SetIndexBuffer(m_renderProxy.GetSketchIndexBuffer());
                m_cmdList->DrawIndexed(m_renderProxy.GetSketchIndexCount());
            }

            // Sketch Outline (crisp vibrant cyan feature boundary)
            if (m_renderProxy.GetSketchVertexBuffer() && m_renderProxy.GetSketchEdgeIndexBuffer() && 
                m_renderProxy.GetSketchEdgeIndexCount() > 0 && m_wireframePipeline) {
                CADWireframeUniforms sketchWireUniforms{};
                sketchWireUniforms.modelViewProjection = cameraUniforms.viewProjection;
                sketchWireUniforms.edgeColor = {0.0f, 0.85f, 1.0f, 1.0f}; // Vibrant electric cyan outline

                m_cmdList->SetPipeline(m_wireframePipeline);
                m_cmdList->SetPushConstants(RHI::ShaderStage::Vertex | RHI::ShaderStage::Fragment, &sketchWireUniforms, sizeof(sketchWireUniforms));
                m_cmdList->SetVertexBuffer(0, m_renderProxy.GetSketchVertexBuffer());
                m_cmdList->SetIndexBuffer(m_renderProxy.GetSketchEdgeIndexBuffer());
                m_cmdList->DrawIndexed(m_renderProxy.GetSketchEdgeIndexCount());
            }

            // 5. Draw Dear ImGui CAD Tooling UI into the swapchain renderpass
            if (m_gui) {
                m_gui->EndFrameAndDraw(m_cmdList.get());
            }

            m_cmdList->EndRenderPass();
            m_cmdList->End();

            m_device->Submit(m_cmdList.get());
            m_device->PresentFrame();
        }
    }

    // 5. Cadence timing & unblocked metrics
    uint64_t frames = m_renderedFrames.fetch_add(1, std::memory_order_relaxed) + 1;
    auto frameEnd = std::chrono::steady_clock::now();
    auto frameDuration = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - m_lastFrameTime);
    m_lastFrameTime = frameEnd;

    double frameTimeMs = frameDuration.count() / 1000.0;
    m_avgFrameTimeMs.store(frameTimeMs, std::memory_order_relaxed);

    if (frames % 120 == 0) {
        std::cout << "  [CAD Viewport Frame #" << std::setw(5) << frames 
                  << "] Cadence: " << std::fixed << std::setprecision(2) << frameTimeMs 
                  << " ms | Active Gen: " << m_lastGenerationId << "\n";
    }
}

void ViewportRenderer::RenderLoop(std::stop_token stopToken) {
    std::cout << "[ViewportRenderer] Independent Render Loop active (Target: 120 FPS).\n";

    constexpr auto targetFrameDuration = std::chrono::microseconds(8333); // 120 FPS cadence

    while (!stopToken.stop_requested() && m_running.load(std::memory_order_relaxed)) {
        auto start = std::chrono::steady_clock::now();
        RenderFrame();
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        if (elapsed < targetFrameDuration) {
            std::this_thread::sleep_for(targetFrameDuration - elapsed);
        }
    }

    std::cout << "[ViewportRenderer] Independent Render Loop terminated.\n";
}

} // namespace Tessera

