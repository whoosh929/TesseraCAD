#pragma once

#include "RHICommon.hpp"
#include <memory>
#include <span>
#include <cassert>

namespace Tessera::RHI {

// -----------------------------------------------------------------------------
// [RHIBuffer]
// Hardware-backed GPU memory buffer (Vertex, Index, Uniform, Staging)
// -----------------------------------------------------------------------------
class RHIBuffer {
public:
    virtual ~RHIBuffer() = default;

    [[nodiscard]] virtual size_t GetSize() const noexcept = 0;
    [[nodiscard]] virtual BufferUsage GetUsage() const noexcept = 0;
    [[nodiscard]] virtual MemoryUsage GetMemoryUsage() const noexcept = 0;

    /// Direct CPU pointer access for HostVisible / Shared memory buffers
    [[nodiscard]] virtual void* Map() = 0;
    virtual void Unmap() = 0;

    /// Convenience upload helper
    virtual void UpdateData(const void* data, size_t size, size_t offset = 0) {
        assert(offset + size <= GetSize());
        void* mapped = Map();
        if (mapped) {
            std::memcpy(static_cast<uint8_t*>(mapped) + offset, data, size);
            Unmap();
        }
    }
};

// -----------------------------------------------------------------------------
// [RHIPipeline]
// Immutable state object encapsulating shaders, rasterization, depth/stencil
// -----------------------------------------------------------------------------
class RHIPipeline {
public:
    virtual ~RHIPipeline() = default;
    [[nodiscard]] virtual const RHIPipelineDesc& GetDesc() const noexcept = 0;
};

// -----------------------------------------------------------------------------
// [RHICommandList]
// Records GPU command sequences (draws, bindings, state changes)
// -----------------------------------------------------------------------------
class RHICommandList {
public:
    virtual ~RHICommandList() = default;

    virtual void Begin() = 0;
    virtual void End() = 0;

    virtual void BeginRenderPass(const RenderPassDesc& passDesc) = 0;
    virtual void EndRenderPass() = 0;

    virtual void SetViewport(const Viewport& viewport) = 0;
    virtual void SetScissor(const ScissorRect& scissor) = 0;

    virtual void SetPipeline(std::shared_ptr<RHIPipeline> pipeline) = 0;
    virtual void SetVertexBuffer(uint32_t slot, std::shared_ptr<RHIBuffer> buffer, size_t offset = 0) = 0;
    virtual void SetIndexBuffer(std::shared_ptr<RHIBuffer> buffer, size_t offset = 0, IndexFormat format = IndexFormat::UInt32) = 0;
    virtual void SetUniformBuffer(uint32_t slot, std::shared_ptr<RHIBuffer> buffer, size_t offset = 0) = 0;

    /// Push constant / dynamic small payload directly into the command stream
    virtual void SetPushConstants(ShaderStage stage, const void* data, size_t size, size_t offset = 0) = 0;

    virtual void Draw(uint32_t vertexCount, uint32_t firstVertex = 0) = 0;
    virtual void DrawIndexed(uint32_t indexCount, uint32_t firstIndex = 0, int32_t vertexOffset = 0) = 0;
};

// -----------------------------------------------------------------------------
// [RHIDevice]
// Primary interface: resource factory, swapchain lifecycle, frame orchestration
// -----------------------------------------------------------------------------
class RHIDevice {
public:
    virtual ~RHIDevice() = default;

    [[nodiscard]] virtual std::shared_ptr<RHIBuffer> CreateBuffer(
        const RHIBufferDesc& desc, 
        const void* initialData = nullptr
    ) = 0;

    [[nodiscard]] virtual std::shared_ptr<RHIPipeline> CreatePipeline(
        const RHIPipelineDesc& desc
    ) = 0;

    [[nodiscard]] virtual std::unique_ptr<RHICommandList> CreateCommandList() = 0;

    /// Resizes swapchain buffers upon window resolution change
    virtual void ResizeSwapchain(uint32_t width, uint32_t height) = 0;

    /// Acquires next backbuffer image; returns false if swapchain is out-of-date
    virtual bool BeginFrame() = 0;

    /// Dispatches recorded commands to the hardware queue
    virtual void Submit(RHICommandList* commandList) = 0;

    /// Presents current backbuffer to display
    virtual void PresentFrame() = 0;

    /// Blocks CPU until GPU has finished all in-flight work (used on shutdown/resize)
    virtual void WaitIdle() = 0;

    [[nodiscard]] virtual uint32_t GetSwapchainWidth() const noexcept = 0;
    [[nodiscard]] virtual uint32_t GetSwapchainHeight() const noexcept = 0;

    /// Static factory to instantiate the platform backend (Metal on macOS, Vulkan on Windows)
    [[nodiscard]] static std::unique_ptr<RHIDevice> Create(
        const NativeWindowHandle& windowHandle,
        uint32_t width,
        uint32_t height
    );
};

} // namespace Tessera::RHI

