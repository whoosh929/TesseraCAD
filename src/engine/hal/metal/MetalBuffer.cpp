#ifdef __APPLE__
#include "MetalBuffer.hpp"
#include <stdexcept>

namespace Tessera::RHI {

MetalBuffer::MetalBuffer(MTL::Device* device, const RHIBufferDesc& desc)
    : m_desc(desc) {

    MTL::ResourceOptions options = MTL::ResourceStorageModeShared;
    if (desc.memory == MemoryUsage::GpuOnly) {
        options = MTL::ResourceStorageModePrivate;
    }

    m_buffer = device->newBuffer(desc.sizeInBytes, options);
    if (!m_buffer) {
        throw std::runtime_error("Failed to allocate Metal buffer: " + desc.debugName);
    }
}

MetalBuffer::~MetalBuffer() {
    if (m_buffer) {
        m_buffer->release();
        m_buffer = nullptr;
    }
}

void* MetalBuffer::Map() {
    if (m_buffer && m_desc.memory == MemoryUsage::CpuToGpu) {
        return m_buffer->contents();
    }
    return nullptr;
}

void MetalBuffer::Unmap() {
    // Shared Apple Silicon memory is unified; no unmap required
}

} // namespace Tessera::RHI
#endif
