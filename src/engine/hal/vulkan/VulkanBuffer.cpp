#include "VulkanBuffer.hpp"
#include <stdexcept>
#include <cstring>

namespace Tessera::RHI {

static VkBufferUsageFlags MapBufferUsage(BufferUsage usage) {
    VkBufferUsageFlags flags = 0;
    if (HasFlag(usage, BufferUsage::Vertex))      flags |= VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    if (HasFlag(usage, BufferUsage::Index))       flags |= VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    if (HasFlag(usage, BufferUsage::Uniform))     flags |= VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    if (HasFlag(usage, BufferUsage::Storage))     flags |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    if (HasFlag(usage, BufferUsage::TransferSrc)) flags |= VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    if (HasFlag(usage, BufferUsage::TransferDst)) flags |= VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    return flags;
}

VulkanBuffer::VulkanBuffer(
    VkDevice device,
    VkPhysicalDevice physicalDevice,
    const RHIBufferDesc& desc
) : m_device(device),
    m_physicalDevice(physicalDevice),
    m_desc(desc) {

    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = desc.sizeInBytes;
    bufferInfo.usage = MapBufferUsage(desc.usage);
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &m_buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan buffer: " + desc.debugName);
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(m_device, m_buffer, &memRequirements);

    VkMemoryPropertyFlags properties = 0;
    if (desc.memory == MemoryUsage::CpuToGpu) {
        // Zero-overhead host-visible coherent buffer mapping
        properties = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    } else {
        properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
    }

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = FindMemoryType(memRequirements.memoryTypeBits, properties);

    if (vkAllocateMemory(m_device, &allocInfo, nullptr, &m_memory) != VK_SUCCESS) {
        vkDestroyBuffer(m_device, m_buffer, nullptr);
        throw std::runtime_error("Failed to allocate Vulkan buffer memory: " + desc.debugName);
    }

    vkBindBufferMemory(m_device, m_buffer, m_memory, 0);

    // If host visible, keep mapped for zero-overhead pointer writes
    if (desc.memory == MemoryUsage::CpuToGpu) {
        vkMapMemory(m_device, m_memory, 0, desc.sizeInBytes, 0, &m_mappedData);
    }
}

VulkanBuffer::~VulkanBuffer() {
    if (m_mappedData) {
        vkUnmapMemory(m_device, m_memory);
        m_mappedData = nullptr;
    }
    if (m_buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(m_device, m_buffer, nullptr);
        m_buffer = VK_NULL_HANDLE;
    }
    if (m_memory != VK_NULL_HANDLE) {
        vkFreeMemory(m_device, m_memory, nullptr);
        m_memory = VK_NULL_HANDLE;
    }
}

void* VulkanBuffer::Map() {
    if (m_mappedData) {
        return m_mappedData;
    }
    if (m_desc.memory == MemoryUsage::CpuToGpu) {
        vkMapMemory(m_device, m_memory, 0, m_desc.sizeInBytes, 0, &m_mappedData);
        return m_mappedData;
    }
    return nullptr;
}

void VulkanBuffer::Unmap() {
    // Persistent mapped buffers do not need unmapping on every write
}

uint32_t VulkanBuffer::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
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

} // namespace Tessera::RHI
