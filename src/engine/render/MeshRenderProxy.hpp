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
        uint64_t generationId,
        std::shared_ptr<RHI::RHIBuffer> edgeIb = nullptr,
        uint32_t edgeIndexCount = 0,
        std::shared_ptr<RHI::RHIBuffer> sketchVb = nullptr,
        std::shared_ptr<RHI::RHIBuffer> sketchIb = nullptr,
        uint32_t sketchIndexCount = 0,
        std::shared_ptr<RHI::RHIBuffer> sketchEdgeIb = nullptr,
        uint32_t sketchEdgeIndexCount = 0
    ) {
        m_vertexBuffer = std::move(vb);
        m_indexBuffer = std::move(ib);
        m_indexCount = indexCount;
        m_edgeIndexBuffer = std::move(edgeIb);
        m_edgeIndexCount = edgeIndexCount;
        m_generationId = generationId;
        m_sketchVertexBuffer = std::move(sketchVb);
        m_sketchIndexBuffer = std::move(sketchIb);
        m_sketchIndexCount = sketchIndexCount;
        m_sketchEdgeIndexBuffer = std::move(sketchEdgeIb);
        m_sketchEdgeIndexCount = sketchEdgeIndexCount;
    }

    [[nodiscard]] bool IsValid() const noexcept {
        return (m_vertexBuffer != nullptr && m_indexCount > 0) ||
               (m_sketchVertexBuffer != nullptr && m_sketchIndexCount > 0);
    }

    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetVertexBuffer() const noexcept { return m_vertexBuffer; }
    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetIndexBuffer() const noexcept { return m_indexBuffer; }
    [[nodiscard]] uint32_t GetIndexCount() const noexcept { return m_indexCount; }
    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetEdgeIndexBuffer() const noexcept { return m_edgeIndexBuffer; }
    [[nodiscard]] uint32_t GetEdgeIndexCount() const noexcept { return m_edgeIndexCount; }
    [[nodiscard]] uint64_t GetGenerationId() const noexcept { return m_generationId; }

    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetSketchVertexBuffer() const noexcept { return m_sketchVertexBuffer; }
    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetSketchIndexBuffer() const noexcept { return m_sketchIndexBuffer; }
    [[nodiscard]] uint32_t GetSketchIndexCount() const noexcept { return m_sketchIndexCount; }
    [[nodiscard]] std::shared_ptr<RHI::RHIBuffer> GetSketchEdgeIndexBuffer() const noexcept { return m_sketchEdgeIndexBuffer; }
    [[nodiscard]] uint32_t GetSketchEdgeIndexCount() const noexcept { return m_sketchEdgeIndexCount; }

private:
    std::shared_ptr<RHI::RHIBuffer> m_vertexBuffer;
    std::shared_ptr<RHI::RHIBuffer> m_indexBuffer;
    uint32_t m_indexCount{0};
    std::shared_ptr<RHI::RHIBuffer> m_edgeIndexBuffer{nullptr};
    uint32_t m_edgeIndexCount{0};
    uint64_t m_generationId{0};

    std::shared_ptr<RHI::RHIBuffer> m_sketchVertexBuffer{nullptr};
    std::shared_ptr<RHI::RHIBuffer> m_sketchIndexBuffer{nullptr};
    uint32_t m_sketchIndexCount{0};
    std::shared_ptr<RHI::RHIBuffer> m_sketchEdgeIndexBuffer{nullptr};
    uint32_t m_sketchEdgeIndexCount{0};
};

} // namespace Tessera

