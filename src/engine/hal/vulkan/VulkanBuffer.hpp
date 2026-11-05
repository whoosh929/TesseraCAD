#pragma once

#include "engine/hal/RHI.hpp"

#ifdef _WIN32
#ifndef VK_USE_PLATFORM_WIN32_KHR
#define VK_USE_PLATFORM_WIN32_KHR
#endif
#endif

#if __has_include(<volk.h>)
#include <volk.h>
#else
#include <vulkan/vulkan.h>
#endif

namespace Tessera::RHI {

class VulkanBuffer final : public RHIBuffer {
public:
    VulkanBuffer(
        VkDevice device,
        VkPhysicalDevice physicalDevice,
        const RHIBufferDesc& desc
    );
    ~VulkanBuffer() override;

    [[nodiscard]] size_t GetSize() const noexcept override { return m_desc.sizeInBytes; }
    [[nodiscard]] BufferUsage GetUsage() const noexcept override { return m_desc.usage; }
    [[nodiscard]] MemoryUsage GetMemoryUsage() const noexcept override { return m_desc.memory; }

    [[nodiscard]] void* Map() override;
    void Unmap() override;

    [[nodiscard]] VkBuffer GetVkBuffer() const noexcept { return m_buffer; }
    [[nodiscard]] VkDeviceMemory GetVkMemory() const noexcept { return m_memory; }

private:
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

    VkDevice m_device{VK_NULL_HANDLE};
    VkPhysicalDevice m_physicalDevice{VK_NULL_HANDLE};
    RHIBufferDesc m_desc;
    VkBuffer m_buffer{VK_NULL_HANDLE};
    VkDeviceMemory m_memory{VK_NULL_HANDLE};
    void* m_mappedData{nullptr};
};

} // namespace Tessera::RHI

