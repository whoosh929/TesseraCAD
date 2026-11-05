#include "VulkanDevice.hpp"
#include <iostream>
#include <stdexcept>
#include <array>
#include <algorithm>

namespace Tessera::RHI {

// -----------------------------------------------------------------------------
// Real Vulkan Graphics Pipeline implementation
// -----------------------------------------------------------------------------
class VulkanPipeline final : public RHIPipeline {
public:
    VulkanPipeline(VkDevice device, VkRenderPass renderPass, const RHIPipelineDesc& desc)
        : m_device(device), m_desc(desc) {

        // 1. Create Shader Modules from SPIR-V
        VkShaderModule vertModule = VK_NULL_HANDLE;
        VkShaderModule fragModule = VK_NULL_HANDLE;

        if (desc.spvVertexBytes && desc.spvVertexSize > 0) {
            VkShaderModuleCreateInfo vertInfo{};
            vertInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            vertInfo.codeSize = desc.spvVertexSize;
            vertInfo.pCode = reinterpret_cast<const uint32_t*>(desc.spvVertexBytes);
            if (vkCreateShaderModule(m_device, &vertInfo, nullptr, &vertModule) != VK_SUCCESS) {
                throw std::runtime_error("Failed to create vertex shader module: " + desc.debugName);
            }
        }

        if (desc.spvFragmentBytes && desc.spvFragmentSize > 0) {
            VkShaderModuleCreateInfo fragInfo{};
            fragInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            fragInfo.codeSize = desc.spvFragmentSize;
            fragInfo.pCode = reinterpret_cast<const uint32_t*>(desc.spvFragmentBytes);
            if (vkCreateShaderModule(m_device, &fragInfo, nullptr, &fragModule) != VK_SUCCESS) {
                if (vertModule != VK_NULL_HANDLE) vkDestroyShaderModule(m_device, vertModule, nullptr);
                throw std::runtime_error("Failed to create fragment shader module: " + desc.debugName);
            }
        }

        VkPipelineShaderStageCreateInfo shaderStages[2]{};
        shaderStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        shaderStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        shaderStages[0].module = vertModule;
        shaderStages[0].pName = desc.vertexEntryPoint.c_str();

        shaderStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        shaderStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        shaderStages[1].module = fragModule;
        shaderStages[1].pName = desc.fragmentEntryPoint.c_str();

        // 2. Vertex input layout
        std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
        VkVertexInputBindingDescription bindingDescription{};
        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

        if (!desc.vertexLayout.attributes.empty()) {
            bindingDescription.binding = 0;
            bindingDescription.stride = desc.vertexLayout.stride;
            bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

            for (const auto& attr : desc.vertexLayout.attributes) {
                VkVertexInputAttributeDescription attrDesc{};
                attrDesc.binding = 0;
                attrDesc.location = attr.location;
                attrDesc.offset = attr.offset;
                switch (attr.format) {
                    case VertexFormat::Float:        attrDesc.format = VK_FORMAT_R32_SFLOAT; break;
                    case VertexFormat::Float2:       attrDesc.format = VK_FORMAT_R32G32_SFLOAT; break;
                    case VertexFormat::Float3:       attrDesc.format = VK_FORMAT_R32G32B32_SFLOAT; break;
                    case VertexFormat::Float4:       attrDesc.format = VK_FORMAT_R32G32B32A32_SFLOAT; break;
                    case VertexFormat::UByte4_Norm:  attrDesc.format = VK_FORMAT_R8G8B8A8_UNORM; break;
                    default:                         attrDesc.format = VK_FORMAT_R32G32B32_SFLOAT; break;
                }
                attributeDescriptions.push_back(attrDesc);
            }

            vertexInputInfo.vertexBindingDescriptionCount = 1;
            vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
            vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
            vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();
        }

        // 3. Input Assembly
        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = (desc.topology == PrimitiveTopology::TriangleList) 
            ? VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST 
            : VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
        inputAssembly.primitiveRestartEnable = VK_FALSE;

        // 4. Dynamic Viewport / Scissor
        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        std::array<VkDynamicState, 2> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR
        };
        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        // 5. Rasterizer
        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = (desc.rasterizer.cullMode == CullMode::Back) ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
        rasterizer.frontFace = (desc.rasterizer.frontFace == FrontFace::CounterClockwise) ? VK_FRONT_FACE_COUNTER_CLOCKWISE : VK_FRONT_FACE_CLOCKWISE;
        rasterizer.depthBiasEnable = (desc.rasterizer.depthBiasConstant != 0.0f);
        rasterizer.depthBiasConstantFactor = desc.rasterizer.depthBiasConstant;
        rasterizer.depthBiasSlopeFactor = desc.rasterizer.depthBiasSlope;

        // 6. Multisampling
        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.sampleShadingEnable = VK_FALSE;
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        // 7. Depth Stencil
        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = desc.depthStencil.depthTestEnable ? VK_TRUE : VK_FALSE;
        depthStencil.depthWriteEnable = desc.depthStencil.depthWriteEnable ? VK_TRUE : VK_FALSE;
        depthStencil.depthCompareOp = (desc.depthStencil.depthCompareOp == CompareOp::Less) ? VK_COMPARE_OP_LESS : VK_COMPARE_OP_LESS_OR_EQUAL;
        depthStencil.depthBoundsTestEnable = VK_FALSE;
        depthStencil.stencilTestEnable = VK_FALSE;

        // 8. Color Blending
        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        if (desc.blendState.blendEnable) {
            colorBlendAttachment.blendEnable = VK_TRUE;
            colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
            colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
            colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
            colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        } else {
            colorBlendAttachment.blendEnable = VK_FALSE;
        }

        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.logicOpEnable = VK_FALSE;
        colorBlending.attachmentCount = 1;
        colorBlending.pAttachments = &colorBlendAttachment;

        // 9. Pipeline Layout with 256-byte Push Constants
        VkPushConstantRange pushConstantRange{};
        pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pushConstantRange.offset = 0;
        pushConstantRange.size = 256;

        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.pushConstantRangeCount = 1;
        pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

        if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create Vulkan pipeline layout: " + desc.debugName);
        }

        // 10. Create Graphics Pipeline
        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = m_pipelineLayout;
        pipelineInfo.renderPass = renderPass;
        pipelineInfo.subpass = 0;

        if (vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipeline) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create Vulkan graphics pipeline: " + desc.debugName);
        }

        if (vertModule != VK_NULL_HANDLE) vkDestroyShaderModule(m_device, vertModule, nullptr);
        if (fragModule != VK_NULL_HANDLE) vkDestroyShaderModule(m_device, fragModule, nullptr);
    }

    ~VulkanPipeline() override {
        if (m_pipeline != VK_NULL_HANDLE) {
            vkDestroyPipeline(m_device, m_pipeline, nullptr);
        }
        if (m_pipelineLayout != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
        }
    }

    [[nodiscard]] const RHIPipelineDesc& GetDesc() const noexcept override { return m_desc; }
    [[nodiscard]] VkPipeline GetVkPipeline() const noexcept { return m_pipeline; }
    [[nodiscard]] VkPipelineLayout GetVkPipelineLayout() const noexcept { return m_pipelineLayout; }

private:
    VkDevice m_device{VK_NULL_HANDLE};
    RHIPipelineDesc m_desc;
    VkPipeline m_pipeline{VK_NULL_HANDLE};
    VkPipelineLayout m_pipelineLayout{VK_NULL_HANDLE};
};

// -----------------------------------------------------------------------------
// Vulkan Command List implementation
// -----------------------------------------------------------------------------
class VulkanCommandList final : public RHICommandList {
public:
    VulkanCommandList(VulkanDevice* device, VkCommandPool pool)
        : m_vulkanDevice(device), m_device(device->GetVkDevice()), m_pool(pool) {
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
        VkRenderPass renderPass = m_vulkanDevice->GetRenderPass();
        VkFramebuffer framebuffer = m_vulkanDevice->GetCurrentFramebuffer();
        if (renderPass != VK_NULL_HANDLE && framebuffer != VK_NULL_HANDLE) {
            VkRenderPassBeginInfo beginInfo{};
            beginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            beginInfo.renderPass = renderPass;
            beginInfo.framebuffer = framebuffer;
            beginInfo.renderArea.offset = {0, 0};
            beginInfo.renderArea.extent = {m_vulkanDevice->GetSwapchainWidth(), m_vulkanDevice->GetSwapchainHeight()};

            std::array<VkClearValue, 2> clearValues{};
            clearValues[0].color = {{passDesc.clearColor.r, passDesc.clearColor.g, passDesc.clearColor.b, passDesc.clearColor.a}};
            clearValues[1].depthStencil = {passDesc.clearDepth, 0};
            beginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
            beginInfo.pClearValues = clearValues.data();

            vkCmdBeginRenderPass(m_cmd, &beginInfo, VK_SUBPASS_CONTENTS_INLINE);
        }
    }

    void EndRenderPass() override {
        if (m_vulkanDevice->GetRenderPass() != VK_NULL_HANDLE) {
            vkCmdEndRenderPass(m_cmd);
        }
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
        auto vkPipeline = std::dynamic_pointer_cast<VulkanPipeline>(m_activePipeline);
        m_activeVkPipeline = vkPipeline ? vkPipeline->GetVkPipeline() : VK_NULL_HANDLE;
        m_activeVkPipelineLayout = vkPipeline ? vkPipeline->GetVkPipelineLayout() : VK_NULL_HANDLE;

        if (m_activeVkPipeline != VK_NULL_HANDLE) {
            vkCmdBindPipeline(m_cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_activeVkPipeline);
        }
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
        (void)stage;
        if (m_activeVkPipelineLayout != VK_NULL_HANDLE && data && size > 0) {
            vkCmdPushConstants(
                m_cmd, 
                m_activeVkPipelineLayout, 
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 
                static_cast<uint32_t>(offset), 
                static_cast<uint32_t>(size), 
                data
            );
        }
    }

    void Draw(uint32_t vertexCount, uint32_t firstVertex) override {
        if (m_activeVkPipeline != VK_NULL_HANDLE) {
            vkCmdDraw(m_cmd, vertexCount, 1, firstVertex, 0);
        }
    }

    void DrawIndexed(uint32_t indexCount, uint32_t firstIndex, int32_t vertexOffset) override {
        if (m_activeVkPipeline != VK_NULL_HANDLE) {
            vkCmdDrawIndexed(m_cmd, indexCount, 1, firstIndex, vertexOffset, 0);
        }
    }

    [[nodiscard]] VkCommandBuffer GetVkCommandBuffer() const noexcept { return m_cmd; }
    [[nodiscard]] void* GetNativeCommandBuffer() noexcept override { return static_cast<void*>(m_cmd); }

private:
    VulkanDevice* m_vulkanDevice{nullptr};
    VkDevice m_device{VK_NULL_HANDLE};
    VkCommandPool m_pool{VK_NULL_HANDLE};
    VkCommandBuffer m_cmd{VK_NULL_HANDLE};
    std::shared_ptr<RHIPipeline> m_activePipeline;
    VkPipeline m_activeVkPipeline{VK_NULL_HANDLE};
    VkPipelineLayout m_activeVkPipelineLayout{VK_NULL_HANDLE};
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
    CreateDepthResources();
    CreateRenderPass();
    CreateFramebuffers();
    CreateDescriptorPool();
    CreateSyncObjects();

    std::cout << "[VulkanDevice] Vulkan 1.3 Device & Swapchain initialized (" 
              << m_width << "x" << m_height << ").\n";
}

VulkanDevice::~VulkanDevice() {
    WaitIdle();
    CleanupSwapchain();

    if (m_descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
        m_descriptorPool = VK_NULL_HANDLE;
    }

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

        auto pfnCreateWin32SurfaceKHR = reinterpret_cast<PFN_vkCreateWin32SurfaceKHR>(
            vkGetInstanceProcAddr(m_instance, "vkCreateWin32SurfaceKHR")
        );
        if (pfnCreateWin32SurfaceKHR) {
            if (pfnCreateWin32SurfaceKHR(m_instance, &createInfo, nullptr, &m_surface) != VK_SUCCESS) {
                throw std::runtime_error("Failed to create Win32 Vulkan surface!");
            }
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

    m_swapchainImageViews.resize(imageCount);
    for (uint32_t i = 0; i < imageCount; ++i) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = m_swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_swapchainFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_device, &viewInfo, nullptr, &m_swapchainImageViews[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create swapchain image view!");
        }
    }
}

uint32_t VulkanDevice::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);
    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) && 
            (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    throw std::runtime_error("Failed to find suitable Vulkan memory type!");
}

void VulkanDevice::CreateDepthResources() {
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = m_width;
    imageInfo.extent.height = m_height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = VK_FORMAT_D32_SFLOAT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateImage(m_device, &imageInfo, nullptr, &m_depthImage) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan depth image!");
    }

    VkMemoryRequirements memReq;
    vkGetImageMemoryRequirements(m_device, m_depthImage, &memReq);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memReq.size;
    allocInfo.memoryTypeIndex = FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    if (vkAllocateMemory(m_device, &allocInfo, nullptr, &m_depthImageMemory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate Vulkan depth image memory!");
    }

    vkBindImageMemory(m_device, m_depthImage, m_depthImageMemory, 0);

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = m_depthImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_D32_SFLOAT;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(m_device, &viewInfo, nullptr, &m_depthImageView) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan depth image view!");
    }
}

void VulkanDevice::CleanupDepthResources() {
    if (m_depthImageView != VK_NULL_HANDLE) {
        vkDestroyImageView(m_device, m_depthImageView, nullptr);
        m_depthImageView = VK_NULL_HANDLE;
    }
    if (m_depthImage != VK_NULL_HANDLE) {
        vkDestroyImage(m_device, m_depthImage, nullptr);
        m_depthImage = VK_NULL_HANDLE;
    }
    if (m_depthImageMemory != VK_NULL_HANDLE) {
        vkFreeMemory(m_device, m_depthImageMemory, nullptr);
        m_depthImageMemory = VK_NULL_HANDLE;
    }
}

void VulkanDevice::CreateRenderPass() {
    VkAttachmentDescription attachments[2]{};

    // 0: Color attachment
    attachments[0].format = m_swapchainFormat;
    attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    // 1: Depth attachment
    attachments[1].format = VK_FORMAT_D32_SFLOAT;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 1;
    depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;
    subpass.pDepthStencilAttachment = &depthAttachmentRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = 2;
    renderPassInfo.pAttachments = attachments;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(m_device, &renderPassInfo, nullptr, &m_renderPass) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan render pass!");
    }
}

void VulkanDevice::CreateFramebuffers() {
    m_framebuffers.resize(m_swapchainImageViews.size());
    for (size_t i = 0; i < m_swapchainImageViews.size(); i++) {
        std::array<VkImageView, 2> attachments = { 
            m_swapchainImageViews[i],
            m_depthImageView
        };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = m_renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = m_width;
        framebufferInfo.height = m_height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(m_device, &framebufferInfo, nullptr, &m_framebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create Vulkan framebuffer!");
        }
    }
}

void VulkanDevice::CreateDescriptorPool() {
    VkDescriptorPoolSize pool_sizes[] = {
        { VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
        { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
        { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }
    };
    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000 * 11;
    pool_info.poolSizeCount = static_cast<uint32_t>(sizeof(pool_sizes)/sizeof(pool_sizes[0]));
    pool_info.pPoolSizes = pool_sizes;

    if (vkCreateDescriptorPool(m_device, &pool_info, nullptr, &m_descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan descriptor pool!");
    }
}

void VulkanDevice::CleanupSwapchain() {
    CleanupDepthResources();

    for (auto fb : m_framebuffers) {
        vkDestroyFramebuffer(m_device, fb, nullptr);
    }
    m_framebuffers.clear();

    if (m_renderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(m_device, m_renderPass, nullptr);
        m_renderPass = VK_NULL_HANDLE;
    }

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
    return std::make_shared<VulkanPipeline>(m_device, m_renderPass, desc);
}

std::unique_ptr<RHICommandList> VulkanDevice::CreateCommandList() {
    return std::make_unique<VulkanCommandList>(this, m_commandPool);
}

void VulkanDevice::ResizeSwapchain(uint32_t width, uint32_t height) {
    m_width = width;
    m_height = height;
    WaitIdle();
    CleanupSwapchain();
    CreateSwapchain();
    CreateDepthResources();
    CreateRenderPass();
    CreateFramebuffers();
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

