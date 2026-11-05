#include "VulkanDevice.hpp"
#include <iostream>
#include <stdexcept>
#include <array>
#include <algorithm>

namespace Tessera::RHI {

// -----------------------------------------------------------------------------
// Minimal Vulkan Pipeline implementation
// -----------------------------------------------------------------------------
class VulkanPipeline final : public RHIPipeline {
public:
    explicit VulkanPipeline(RHIPipelineDesc desc) : m_desc(std::move(desc)) {}
    [[nodiscard]] const RHIPipelineDesc& GetDesc() const noexcept override { return m_desc; }
private:
    RHIPipelineDesc m_desc;
};

// -----------------------------------------------------------------------------
// Vulkan Command List implementation
// -----------------------------------------------------------------------------
class VulkanCommandList final : public RHICommandList {
public:
    VulkanCommandList(VkDevice device, VkCommandPool pool)
        : m_device(device), m_pool(pool) {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = pool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;
        vkAllocateCommandBuffers(m_device, &allocInfo, &m_cmd);
    }

    ~VulkanCommandList() override {
        if (m_cmd != VK_NULL_HANDLE) {
            vkFreeCommandBuffers(m_device, m_pool, 1, &m_cmd);
        }
    }

    void Begin() override {
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(m_cmd, &beginInfo);
    }

    void End() override {
        vkEndCommandBuffer(m_cmd);
    }

    void BeginRenderPass(const RenderPassDesc& passDesc) override {
        (void)passDesc;
        // In full pipeline, begins VkRenderPass
    }

    void EndRenderPass() override {
        // vkCmdEndRenderPass(m_cmd);
    }

    void SetViewport(const Viewport& viewport) override {
        VkViewport vp{
            .x = viewport.x,
            .y = viewport.y,
            .width = viewport.width,
            .height = viewport.height,
            .minDepth = viewport.minDepth,
            .maxDepth = viewport.maxDepth
        };
        vkCmdSetViewport(m_cmd, 0, 1, &vp);
    }

    void SetScissor(const ScissorRect& scissor) override {
        VkRect2D sc{
            .offset = {scissor.x, scissor.y},
            .extent = {scissor.width, scissor.height}
        };
        vkCmdSetScissor(m_cmd, 0, 1, &sc);
    }

    void SetPipeline(std::shared_ptr<RHIPipeline> pipeline) override {
        m_activePipeline = std::move(pipeline);
    }

    void SetVertexBuffer(uint32_t slot, std::shared_ptr<RHIBuffer> buffer, size_t offset) override {
        if (auto vkBuf = std::dynamic_pointer_cast<VulkanBuffer>(buffer)) {
            VkBuffer b = vkBuf->GetVkBuffer();
            VkDeviceSize off = offset;
            vkCmdBindVertexBuffers(m_cmd, slot, 1, &b, &off);
        }
    }

    void SetIndexBuffer(std::shared_ptr<RHIBuffer> buffer, size_t offset, IndexFormat format) override {
        if (auto vkBuf = std::dynamic_pointer_cast<VulkanBuffer>(buffer)) {
            VkIndexType it = (format == IndexFormat::UInt32) ? VK_INDEX_TYPE_UINT32 : VK_INDEX_TYPE_UINT16;
            vkCmdBindIndexBuffer(m_cmd, vkBuf->GetVkBuffer(), offset, it);
        }
    }

    void SetUniformBuffer(uint32_t slot, std::shared_ptr<RHIBuffer> buffer, size_t offset) override {
        (void)slot; (void)buffer; (void)offset;
    }

    void SetPushConstants(ShaderStage stage, const void* data, size_t size, size_t offset) override {
        (void)stage; (void)data; (void)size; (void)offset;
        // vkCmdPushConstants(m_cmd, layout, stageFlags, offset, size, data);
    }

    void Draw(uint32_t vertexCount, uint32_t firstVertex) override {
        vkCmdDraw(m_cmd, vertexCount, 1, firstVertex, 0);
    }

    void DrawIndexed(uint32_t indexCount, uint32_t firstIndex, int32_t vertexOffset) override {
        vkCmdDrawIndexed(m_cmd, indexCount, 1, firstIndex, vertexOffset, 0);
    }

    [[nodiscard]] VkCommandBuffer GetVkCommandBuffer() const noexcept { return m_cmd; }

private:
    VkDevice m_device{VK_NULL_HANDLE};
    VkCommandPool m_pool{VK_NULL_HANDLE};
    VkCommandBuffer m_cmd{VK_NULL_HANDLE};
    std::shared_ptr<RHIPipeline> m_activePipeline;
};

// -----------------------------------------------------------------------------
// VulkanDevice implementation
// -----------------------------------------------------------------------------
VulkanDevice::VulkanDevice(const NativeWindowHandle& windowHandle, uint32_t width, uint32_t height)
    : m_width(width), m_height(height), m_windowHandle(windowHandle) {

#if __has_include(<volk.h>)
    if (volkInitialize() != VK_SUCCESS) {
        throw std::runtime_error("Failed to initialize volk loader!");
    }
#endif

    InitInstance();
    InitSurface(windowHandle);
    PickPhysicalDevice();
    CreateLogicalDevice();
    CreateSwapchain();
    CreateSyncObjects();

    std::cout << "[VulkanDevice] Vulkan 1.3 Device & Swapchain initialized (" 
              << m_width << "x" << m_height << ").\n";
}

VulkanDevice::~VulkanDevice() {
    WaitIdle();
    CleanupSwapchain();

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        vkDestroySemaphore(m_device, m_imageAvailableSemaphores[i], nullptr);
        vkDestroySemaphore(m_device, m_renderFinishedSemaphores[i], nullptr);
        vkDestroyFence(m_device, m_inFlightFences[i], nullptr);
    }

    if (m_commandPool != VK_NULL_HANDLE) {
        vkDestroyCommandPool(m_device, m_commandPool, nullptr);
    }
    if (m_device != VK_NULL_HANDLE) {
        vkDestroyDevice(m_device, nullptr);
    }
    if (m_surface != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
    }
    if (m_instance != VK_NULL_HANDLE) {
        vkDestroyInstance(m_instance, nullptr);
    }
}

void VulkanDevice::InitInstance() {
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "TesseraCAD";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "TesseraEngine";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_3;

    std::vector<const char*> extensions = {
        VK_KHR_SURFACE_EXTENSION_NAME,
#ifdef VK_USE_PLATFORM_WIN32_KHR
        VK_KHR_WIN32_SURFACE_EXTENSION_NAME
#endif
    };

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.data();

    if (vkCreateInstance(&createInfo, nullptr, &m_instance) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan instance!");
    }

#if __has_include(<volk.h>)
    volkLoadInstance(m_instance);
#endif
}

void VulkanDevice::InitSurface(const NativeWindowHandle& windowHandle) {
#ifdef VK_USE_PLATFORM_WIN32_KHR
    if (windowHandle.windowHandle) {
        VkWin32SurfaceCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
        createInfo.hwnd = static_cast<HWND>(windowHandle.windowHandle);
        createInfo.hinstance = static_cast<HINSTANCE>(windowHandle.displayHandle ? windowHandle.displayHandle : GetModuleHandle(nullptr));

        if (vkCreateWin32SurfaceKHR(m_instance, &createInfo, nullptr, &m_surface) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create Win32 Vulkan surface!");
        }
    }
#else
    (void)windowHandle;
#endif
}

void VulkanDevice::PickPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);
    if (deviceCount == 0) {
        throw std::runtime_error("Failed to find GPUs with Vulkan support!");
    }

    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

    // Prefer discrete GPU
    for (const auto& dev : devices) {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(dev, &props);
        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            m_physicalDevice = dev;
            std::cout << "[VulkanDevice] Selected Discrete GPU: " << props.deviceName << "\n";
            return;
        }
    }
    m_physicalDevice = devices[0];
}

void VulkanDevice::CreateLogicalDevice() {
    // Find graphics queue family
    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, queueFamilies.data());

    for (uint32_t i = 0; i < queueFamilyCount; i++) {
        if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            m_graphicsQueueFamily = i;
            m_presentQueueFamily = i;
            break;
        }
    }

    float queuePriority = 1.0f;
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = m_graphicsQueueFamily;
    queueCreateInfo.queueCount = 1;
    queueCreateInfo.pQueuePriorities = &queuePriority;

    std::vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueCreateInfo;
    createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    if (vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan logical device!");
    }

#if __has_include(<volk.h>)
    volkLoadDevice(m_device);
#endif

    vkGetDeviceQueue(m_device, m_graphicsQueueFamily, 0, &m_graphicsQueue);
    vkGetDeviceQueue(m_device, m_presentQueueFamily, 0, &m_presentQueue);

    // Command pool
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = m_graphicsQueueFamily;

    if (vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan command pool!");
    }
}

void VulkanDevice::CreateSwapchain() {
    if (m_surface == VK_NULL_HANDLE) return;

    VkSurfaceCapabilitiesKHR capabilities;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_physicalDevice, m_surface, &capabilities);

    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = m_surface;
    createInfo.minImageCount = std::max(2u, capabilities.minImageCount);
    createInfo.imageFormat = m_swapchainFormat;
    createInfo.imageColorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    createInfo.imageExtent = {m_width, m_height};
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.preTransform = capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR; // VSync / fixed cadence
    createInfo.clipped = VK_TRUE;

    if (vkCreateSwapchainKHR(m_device, &createInfo, nullptr, &m_swapchain) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan swapchain!");
    }

    uint32_t imageCount = 0;
    vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, nullptr);
    m_swapchainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, m_swapchainImages.data());
}

void VulkanDevice::CleanupSwapchain() {
    for (auto imageView : m_swapchainImageViews) {
        vkDestroyImageView(m_device, imageView, nullptr);
    }
    m_swapchainImageViews.clear();

    if (m_swapchain != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(m_device, m_swapchain, nullptr);
        m_swapchain = VK_NULL_HANDLE;
    }
}

void VulkanDevice::CreateSyncObjects() {
    m_imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semInfo{};
    semInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        vkCreateSemaphore(m_device, &semInfo, nullptr, &m_imageAvailableSemaphores[i]);
        vkCreateSemaphore(m_device, &semInfo, nullptr, &m_renderFinishedSemaphores[i]);
        vkCreateFence(m_device, &fenceInfo, nullptr, &m_inFlightFences[i]);
    }
}

std::shared_ptr<RHIBuffer> VulkanDevice::CreateBuffer(const RHIBufferDesc& desc, const void* initialData) {
    auto buf = std::make_shared<VulkanBuffer>(m_device, m_physicalDevice, desc);
    if (initialData && desc.sizeInBytes > 0) {
        buf->UpdateData(initialData, desc.sizeInBytes, 0);
    }
    return buf;
}

std::shared_ptr<RHIPipeline> VulkanDevice::CreatePipeline(const RHIPipelineDesc& desc) {
    return std::make_shared<VulkanPipeline>(desc);
}

std::unique_ptr<RHICommandList> VulkanDevice::CreateCommandList() {
    return std::make_unique<VulkanCommandList>(m_device, m_commandPool);
}

void VulkanDevice::ResizeSwapchain(uint32_t width, uint32_t height) {
    m_width = width;
    m_height = height;
    WaitIdle();
    CleanupSwapchain();
    CreateSwapchain();
}

bool VulkanDevice::BeginFrame() {
    if (m_swapchain == VK_NULL_HANDLE) return false;

    vkWaitForFences(m_device, 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);

    VkResult result = vkAcquireNextImageKHR(
        m_device, 
        m_swapchain, 
        UINT64_MAX, 
        m_imageAvailableSemaphores[m_currentFrame], 
        VK_NULL_HANDLE, 
        &m_imageIndex
    );

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        ResizeSwapchain(m_width, m_height);
        return false;
    }

    vkResetFences(m_device, 1, &m_inFlightFences[m_currentFrame]);
    return true;
}

void VulkanDevice::Submit(RHICommandList* commandList) {
    auto* vkCmd = dynamic_cast<VulkanCommandList*>(commandList);
    if (!vkCmd) return;

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = {m_imageAvailableSemaphores[m_currentFrame]};
    VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;

    VkCommandBuffer cmd = vkCmd->GetVkCommandBuffer();
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;

    VkSemaphore signalSemaphores[] = {m_renderFinishedSemaphores[m_currentFrame]};
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_inFlightFences[m_currentFrame]);
}

void VulkanDevice::PresentFrame() {
    if (m_swapchain == VK_NULL_HANDLE) return;

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

    VkSemaphore signalSemaphores[] = {m_renderFinishedSemaphores[m_currentFrame]};
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;

    VkSwapchainKHR swapchains[] = {m_swapchain};
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapchains;
    presentInfo.pImageIndices = &m_imageIndex;

    vkQueuePresentKHR(m_presentQueue, &presentInfo);
    m_currentFrame = (m_currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void VulkanDevice::WaitIdle() {
    if (m_device != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(m_device);
    }
}

} // namespace Tessera::RHI

#ifdef _WIN32
namespace Tessera::RHI {
std::unique_ptr<RHIDevice> RHIDevice::Create(
    const NativeWindowHandle& windowHandle,
    uint32_t width,
    uint32_t height
) {
    return std::make_unique<VulkanDevice>(windowHandle, width, height);
}
}
#endif

