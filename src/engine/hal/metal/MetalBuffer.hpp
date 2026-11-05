#pragma once

#include "engine/hal/RHI.hpp"

#ifdef __APPLE__
#include <Metal/Metal.hpp>

namespace Tessera::RHI {

class MetalBuffer final : public RHIBuffer {
public:
    MetalBuffer(MTL::Device* device, const RHIBufferDesc& desc);
    ~MetalBuffer() override;

    [[nodiscard]] size_t GetSize() const noexcept override { return m_desc.sizeInBytes; }
    [[nodiscard]] BufferUsage GetUsage() const noexcept override { return m_desc.usage; }
    [[nodiscard]] MemoryUsage GetMemoryUsage() const noexcept override { return m_desc.memory; }

    [[nodiscard]] void* Map() override;
    void Unmap() override;

    [[nodiscard]] MTL::Buffer* GetMetalBuffer() const noexcept { return m_buffer; }

private:
    RHIBufferDesc m_desc;
    MTL::Buffer* m_buffer{nullptr};
};

} // namespace Tessera::RHI
#endif
