#ifdef __APPLE__

// Exact translation unit containing the private metal-cpp implementations
#define NS_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include "MetalDevice.hpp"
#include <iostream>
#include <stdexcept>

namespace Tessera::RHI {

// -----------------------------------------------------------------------------
// Metal Pipeline implementation
// -----------------------------------------------------------------------------
class MetalPipeline final : public RHIPipeline {
public:
    explicit MetalPipeline(RHIPipelineDesc desc) : m_desc(std::move(desc)) {}
    [[nodiscard]] const RHIPipelineDesc& GetDesc() const noexcept override { return m_desc; }
private:
    RHIPipelineDesc m_desc;
};

// -----------------------------------------------------------------------------
// Metal Command List implementation
// -----------------------------------------------------------------------------
class MetalCommandList final : public RHICommandList {
public:
    MetalCommandList(MTL::CommandQueue* queue) : m_queue(queue) {}

    void Begin() override {
        m_cmdBuffer = m_queue->commandBuffer();
    }

    void End() override {
        // Ready for commit
    }

    void BeginRenderPass(const RenderPassDesc& passDesc) override {
        (void)passDesc;
    }

    void EndRenderPass() override {
        if (m_encoder) {
            m_encoder->endEncoding();
            m_encoder = nullptr;
        }
    }

    void SetViewport(const Viewport& viewport) override {
        if (m_encoder) {
            MTL::Viewport vp{
                .originX = viewport.x,
                .originY = viewport.y,
                .width = viewport.width,
                .height = viewport.height,
                .znear = viewport.minDepth,
                .zfar = viewport.maxDepth
            };
            m_encoder->setViewport(vp);
        }
    }

    void SetScissor(const ScissorRect& scissor) override {
        if (m_encoder) {
            MTL::ScissorRect sc{
                .x = static_cast<NS::UInteger>(scissor.x),
                .y = static_cast<NS::UInteger>(scissor.y),
                .width = scissor.width,
                .height = scissor.height
            };
            m_encoder->setScissorRect(sc);
        }
    }

    void SetPipeline(std::shared_ptr<RHIPipeline> pipeline) override {
        m_activePipeline = std::move(pipeline);
    }

    void SetVertexBuffer(uint32_t slot, std::shared_ptr<RHIBuffer> buffer, size_t offset) override {
        if (m_encoder) {
            if (auto metalBuf = std::dynamic_pointer_cast<MetalBuffer>(buffer)) {
                m_encoder->setVertexBuffer(metalBuf->GetMetalBuffer(), offset, slot);
            }
        }
    }

    void SetIndexBuffer(std::shared_ptr<RHIBuffer> buffer, size_t offset, IndexFormat format) override {
        m_indexBuffer = std::dynamic_pointer_cast<MetalBuffer>(buffer);
        m_indexOffset = offset;
        m_indexType = (format == IndexFormat::UInt32) ? MTL::IndexTypeUInt32 : MTL::IndexTypeUInt16;
    }

    void SetUniformBuffer(uint32_t slot, std::shared_ptr<RHIBuffer> buffer, size_t offset) override {
        if (m_encoder) {
            if (auto metalBuf = std::dynamic_pointer_cast<MetalBuffer>(buffer)) {
                m_encoder->setVertexBuffer(metalBuf->GetMetalBuffer(), offset, slot);
                m_encoder->setFragmentBuffer(metalBuf->GetMetalBuffer(), offset, slot);
            }
        }
    }

    void SetPushConstants(ShaderStage stage, const void* data, size_t size, size_t offset) override {
        (void)offset;
        if (m_encoder) {
            if (stage == ShaderStage::Vertex) {
                m_encoder->setVertexBytes(data, size, 1);
            } else if (stage == ShaderStage::Fragment) {
                m_encoder->setFragmentBytes(data, size, 1);
            }
        }
    }

    void Draw(uint32_t vertexCount, uint32_t firstVertex) override {
        if (m_encoder) {
            m_encoder->drawPrimitives(MTL::PrimitiveTypeTriangle, firstVertex, vertexCount);
        }
    }

    void DrawIndexed(uint32_t indexCount, uint32_t firstIndex, int32_t vertexOffset) override {
        (void)firstIndex; (void)vertexOffset;
        if (m_encoder && m_indexBuffer) {
            m_encoder->drawIndexedPrimitives(
                MTL::PrimitiveTypeTriangle,
                indexCount,
                m_indexType,
                m_indexBuffer->GetMetalBuffer(),
                m_indexOffset
            );
        }
    }

    [[nodiscard]] MTL::CommandBuffer* GetMetalCommandBuffer() const noexcept { return m_cmdBuffer; }

private:
    MTL::CommandQueue* m_queue{nullptr};
    MTL::CommandBuffer* m_cmdBuffer{nullptr};
    MTL::RenderCommandEncoder* m_encoder{nullptr};
    std::shared_ptr<RHIPipeline> m_activePipeline;
    std::shared_ptr<MetalBuffer> m_indexBuffer;
    size_t m_indexOffset{0};
    MTL::IndexType m_indexType{MTL::IndexTypeUInt32};
};

// -----------------------------------------------------------------------------
// MetalDevice implementation
// -----------------------------------------------------------------------------
MetalDevice::MetalDevice(const NativeWindowHandle& windowHandle, uint32_t width, uint32_t height)
    : m_width(width), m_height(height), m_windowHandle(windowHandle) {

    m_device = MTL::CreateSystemDefaultDevice();
    if (!m_device) {
        throw std::runtime_error("Apple Metal is not supported on this device!");
    }

    m_commandQueue = m_device->newCommandQueue();

    if (windowHandle.windowHandle) {
        m_metalLayer = static_cast<CA::MetalLayer*>(windowHandle.windowHandle);
        m_metalLayer->setDevice(m_device);
        m_metalLayer->setPixelFormat(MTL::PixelFormatBGRA8Unorm);
        m_metalLayer->setDrawableSize(CGSizeMake(width, height));
    }

    std::cout << "[MetalDevice] Apple Metal 3 Device initialized: " 
              << m_device->name()->utf8String() << " (" << m_width << "x" << m_height << ").\n";
}

MetalDevice::~MetalDevice() {
    WaitIdle();
    if (m_commandQueue) {
        m_commandQueue->release();
        m_commandQueue = nullptr;
    }
    if (m_device) {
        m_device->release();
        m_device = nullptr;
    }
}

std::shared_ptr<RHIBuffer> MetalDevice::CreateBuffer(const RHIBufferDesc& desc, const void* initialData) {
    auto buf = std::make_shared<MetalBuffer>(m_device, desc);
    if (initialData && desc.sizeInBytes > 0) {
        buf->UpdateData(initialData, desc.sizeInBytes, 0);
    }
    return buf;
}

std::shared_ptr<RHIPipeline> MetalDevice::CreatePipeline(const RHIPipelineDesc& desc) {
    return std::make_shared<MetalPipeline>(desc);
}

std::unique_ptr<RHICommandList> MetalDevice::CreateCommandList() {
    return std::make_unique<MetalCommandList>(m_commandQueue);
}

void MetalDevice::ResizeSwapchain(uint32_t width, uint32_t height) {
    m_width = width;
    m_height = height;
    if (m_metalLayer) {
        m_metalLayer->setDrawableSize(CGSizeMake(width, height));
    }
}

bool MetalDevice::BeginFrame() {
    if (!m_metalLayer) return false;
    m_currentDrawable = m_metalLayer->nextDrawable();
    return m_currentDrawable != nullptr;
}

void MetalDevice::Submit(RHICommandList* commandList) {
    auto* metalCmd = dynamic_cast<MetalCommandList*>(commandList);
    if (!metalCmd) return;

    m_activeCommandBuffer = metalCmd->GetMetalCommandBuffer();
    if (m_activeCommandBuffer && m_currentDrawable) {
        m_activeCommandBuffer->presentDrawable(m_currentDrawable);
        m_activeCommandBuffer->commit();
    }
}

void MetalDevice::PresentFrame() {
    // Presented via presentDrawable during buffer commit
    m_currentDrawable = nullptr;
    m_activeCommandBuffer = nullptr;
}

void MetalDevice::WaitIdle() {
    // Command buffers track their completion handlers
}

} // namespace Tessera::RHI

namespace Tessera::RHI {
std::unique_ptr<RHIDevice> RHIDevice::Create(
    const NativeWindowHandle& windowHandle,
    uint32_t width,
    uint32_t height
) {
    return std::make_unique<MetalDevice>(windowHandle, width, height);
}
}

#endif
