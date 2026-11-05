#pragma once

#include "engine/hal/RHI.hpp"
#include <memory>
#include <array>

namespace Tessera {

class MeshRenderProxy {
public:
    MeshRenderProxy() = default;

    void Update(
        std::shared_ptr<RHI::RHIBuffer> vb,
        std::shared_ptr<RHI::RHIBuffer> ib,
        uint32_t indexCount,
        uint64_t generationId
    ) {
        m_vertexBuffer = std::move(vb);
        m_indexBuffer = std::move(ib);
        m_indexCount = indexCount;
        m_generationId = generationId;
    }

    [[nodiscard]] bool IsValid() const noexcept {
        return m_vertexBuffer != nullptr && m_indexCount > 0;
    }

    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetVertexBuffer() const noexcept { return m_vertexBuffer; }
    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetIndexBuffer() const noexcept { return m_indexBuffer; }
    [[nodiscard]] uint32_t GetIndexCount() const noexcept { return m_indexCount; }
    [[nodiscard]] uint64_t GetGenerationId() const noexcept { return m_generationId; }

private:
    std::shared_ptr<RHI::RHIBuffer> m_vertexBuffer;
    std::shared_ptr<RHI::RHIBuffer> m_indexBuffer;
    uint32_t m_indexCount{0};
    uint64_t m_generationId{0};
};

} // namespace Tessera
