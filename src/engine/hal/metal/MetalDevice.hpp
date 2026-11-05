#pragma once

#include "engine/hal/RHI.hpp"
#include "MetalBuffer.hpp"

#ifdef __APPLE__
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

namespace Tessera::RHI {

class MetalDevice final : public RHIDevice {
public:
    MetalDevice(const NativeWindowHandle& windowHandle, uint32_t width, uint32_t height);
    ~MetalDevice() override;

    [[nodiscard]] std::shared_ptr<RHIBuffer> CreateBuffer(
        const RHIBufferDesc& desc, 
        const void* initialData = nullptr
    ) override;

    [[nodiscard]] std::shared_ptr<RHIPipeline> CreatePipeline(
        const RHIPipelineDesc& desc
    ) override;

    [[nodiscard]] std::unique_ptr<RHICommandList> CreateCommandList() override;

    void ResizeSwapchain(uint32_t width, uint32_t height) override;
    bool BeginFrame() override;
    void Submit(RHICommandList* commandList) override;
    void PresentFrame() override;
    void WaitIdle() override;

    [[nodiscard]] uint32_t GetSwapchainWidth() const noexcept override { return m_width; }
    [[nodiscard]] uint32_t GetSwapchainHeight() const noexcept override { return m_height; }

    [[nodiscard]] MTL::Device* GetMetalDevice() const noexcept { return m_device; }

private:
    uint32_t m_width{1920};
    uint32_t m_height{1080};
    NativeWindowHandle m_windowHandle{};

    MTL::Device* m_device{nullptr};
    MTL::CommandQueue* m_commandQueue{nullptr};
    CA::MetalLayer* m_metalLayer{nullptr};
    CA::MetalDrawable* m_currentDrawable{nullptr};
    MTL::CommandBuffer* m_activeCommandBuffer{nullptr};
};

} // namespace Tessera::RHI
#endif

