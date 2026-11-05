#pragma once

#include "engine/hal/RHI.hpp"
#include "VulkanBuffer.hpp"
#include <vector>

namespace Tessera::RHI {

class VulkanDevice final : public RHIDevice {
public:
    VulkanDevice(const NativeWindowHandle& windowHandle, uint32_t width, uint32_t height);
    ~VulkanDevice() override;

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

    [[nodiscard]] VkDevice GetVkDevice() const noexcept { return m_device; }
    [[nodiscard]] VkQueue GetGraphicsQueue() const noexcept { return m_graphicsQueue; }

private:
    void InitInstance();
    void InitSurface(const NativeWindowHandle& windowHandle);
    void PickPhysicalDevice();
    void CreateLogicalDevice();
    void CreateSwapchain();
    void CleanupSwapchain();
    void CreateSyncObjects();

    uint32_t m_width{1920};
    uint32_t m_height{1080};
    NativeWindowHandle m_windowHandle{};

    VkInstance m_instance{VK_NULL_HANDLE};
    VkSurfaceKHR m_surface{VK_NULL_HANDLE};
    VkPhysicalDevice m_physicalDevice{VK_NULL_HANDLE};
    VkDevice m_device{VK_NULL_HANDLE};

    VkQueue m_graphicsQueue{VK_NULL_HANDLE};
    VkQueue m_presentQueue{VK_NULL_HANDLE};
    uint32_t m_graphicsQueueFamily{0};
    uint32_t m_presentQueueFamily{0};

    VkSwapchainKHR m_swapchain{VK_NULL_HANDLE};
    std::vector<VkImage> m_swapchainImages;
    std::vector<VkImageView> m_swapchainImageViews;
    VkFormat m_swapchainFormat{VK_FORMAT_B8G8R8A8_UNORM};

    VkCommandPool m_commandPool{VK_NULL_HANDLE};

    static constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;
    uint32_t m_currentFrame{0};
    uint32_t m_imageIndex{0};

    std::vector<VkSemaphore> m_imageAvailableSemaphores;
    std::vector<VkSemaphore> m_renderFinishedSemaphores;
    std::vector<VkFence> m_inFlightFences;
};

} // namespace Tessera::RHI
