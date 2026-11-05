#pragma once

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

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
    [[nodiscard]] VkPhysicalDevice GetVkPhysicalDevice() const noexcept { return m_physicalDevice; }
    [[nodiscard]] VkInstance GetVkInstance() const noexcept { return m_instance; }
    [[nodiscard]] VkQueue GetGraphicsQueue() const noexcept { return m_graphicsQueue; }
    [[nodiscard]] uint32_t GetGraphicsQueueFamily() const noexcept { return m_graphicsQueueFamily; }
    [[nodiscard]] VkRenderPass GetRenderPass() const noexcept { return m_renderPass; }
    [[nodiscard]] VkDescriptorPool GetDescriptorPool() const noexcept { return m_descriptorPool; }
    [[nodiscard]] uint32_t GetImageCount() const noexcept { return static_cast<uint32_t>(m_swapchainImages.size()); }
    [[nodiscard]] VkFramebuffer GetCurrentFramebuffer() const noexcept {
        return (m_imageIndex < m_framebuffers.size()) ? m_framebuffers[m_imageIndex] : VK_NULL_HANDLE;
    }

private:
    void InitInstance();
    void InitSurface(const NativeWindowHandle& windowHandle);
    void PickPhysicalDevice();
    void CreateLogicalDevice();
    void CreateSwapchain();
    void CleanupSwapchain();
    void CreateDepthResources();
    void CleanupDepthResources();
    void CreateRenderPass();
    void CreateFramebuffers();
    void CreateDescriptorPool();
    void CreateSyncObjects();
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

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

    VkImage m_depthImage{VK_NULL_HANDLE};
    VkDeviceMemory m_depthImageMemory{VK_NULL_HANDLE};
    VkImageView m_depthImageView{VK_NULL_HANDLE};

    VkCommandPool m_commandPool{VK_NULL_HANDLE};
    VkRenderPass m_renderPass{VK_NULL_HANDLE};
    std::vector<VkFramebuffer> m_framebuffers;
    VkDescriptorPool m_descriptorPool{VK_NULL_HANDLE};

    static constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;
    uint32_t m_currentFrame{0};
    uint32_t m_imageIndex{0};

    std::vector<VkSemaphore> m_imageAvailableSemaphores;
    std::vector<VkSemaphore> m_renderFinishedSemaphores;
    std::vector<VkFence> m_inFlightFences;
};

} // namespace Tessera::RHI

