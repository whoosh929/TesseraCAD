#pragma once

#include "engine/hal/RHI.hpp"
#include <memory>
#include <atomic>
#include <cstdint>
#include <array>
#include <optional>

namespace Tessera {

// -----------------------------------------------------------------------------
// [RenderMeshData]
// GPU-resident geometry payload consumed by the Render Thread
// -----------------------------------------------------------------------------
struct RenderMeshData {
    std::shared_ptr<RHI::RHIBuffer> vertexBuffer;
    std::shared_ptr<RHI::RHIBuffer> indexBuffer;
    uint32_t indexCount{0};
    uint64_t generationId{0};
};

// -----------------------------------------------------------------------------
// [AtomicMeshExchange]
// Zero-contention atomic exchange using C++20 atomic std::shared_ptr semantics.
// The geometry worker publishes with std::memory_order_release.
// The render thread acquires with std::memory_order_acquire.
// -----------------------------------------------------------------------------
class AtomicMeshExchange {
public:
    AtomicMeshExchange() = default;
    ~AtomicMeshExchange() = default;

    // Non-copyable, non-movable
    AtomicMeshExchange(const AtomicMeshExchange&) = delete;
    AtomicMeshExchange& operator=(const AtomicMeshExchange&) = delete;

    /// Called exclusively by Geometry/Parametric Worker Pool
    void Publish(std::shared_ptr<RenderMeshData> newMesh) noexcept {
        m_activeMesh.store(std::move(newMesh), std::memory_order_release);
        m_publishedGeneration.fetch_add(1, std::memory_order_release);
    }

    /// Convenience overload to publish inline data
    void Publish(
        std::shared_ptr<RHI::RHIBuffer> vb,
        std::shared_ptr<RHI::RHIBuffer> ib,
        uint32_t indexCount,
        uint64_t generationId
    ) {
        auto payload = std::make_shared<RenderMeshData>(RenderMeshData{
            .vertexBuffer = std::move(vb),
            .indexBuffer = std::move(ib),
            .indexCount = indexCount,
            .generationId = generationId
        });
        Publish(std::move(payload));
    }

    /// Called on the hot path by the Render Thread.
    /// Guaranteed lock-free read of the latest published mesh snapshot.
    [[nodiscard]] std::shared_ptr<RenderMeshData> AcquireLatest() const noexcept {
        return m_activeMesh.load(std::memory_order_acquire);
    }

    /// Fast generation check to skip redundant state changes
    [[nodiscard]] uint64_t GetPublishedGeneration() const noexcept {
        return m_publishedGeneration.load(std::memory_order_acquire);
    }

    /// Clear current payload
    void Reset() noexcept {
        m_activeMesh.store(nullptr, std::memory_order_release);
    }

private:
    std::atomic<std::shared_ptr<RenderMeshData>> m_activeMesh{nullptr};
    std::atomic<uint64_t> m_publishedGeneration{0};
};

// -----------------------------------------------------------------------------
// [TripleBufferedMeshExchange]
// Pure lock-free triple buffer with zero dynamic memory allocation on the swap path.
// Encodes: [WriteSlot (Worker)] <--> [SharedNewestSlot] <--> [ReadSlot (Renderer)]
// -----------------------------------------------------------------------------
class TripleBufferedMeshExchange {
public:
    TripleBufferedMeshExchange() {
        m_readIndex = 0;
        m_writeIndex = 1;
        // Pack initial state: newest = 2, hasNew = 0
        m_sharedState.store(PackState(2, false), std::memory_order_relaxed);
    }

    /// Worker acquires staging slot to populate
    [[nodiscard]] RenderMeshData& GetWriteSlot() noexcept {
        return m_slots[m_writeIndex];
    }

    /// Worker publishes the populated write slot
    void PublishWriteSlot() noexcept {
        // Exchange our write index with the newest slot, marking hasNew = true
        uint32_t currentShared = m_sharedState.load(std::memory_order_relaxed);
        uint32_t newShared;
        do {
            uint8_t currentNewest = UnpackIndex(currentShared);
            newShared = PackState(m_writeIndex, true);
            if (m_sharedState.compare_exchange_weak(
                    currentShared,
                    newShared,
                    std::memory_order_release,
                    std::memory_order_relaxed)) {
                // The previous newest slot becomes our next write target
                m_writeIndex = currentNewest;
                break;
            }
        } while (true);
    }

    /// Render thread updates its local read slot if a newer one was published
    [[nodiscard]] const RenderMeshData* AcquireLatestReadSlot() noexcept {
        uint32_t currentShared = m_sharedState.load(std::memory_order_relaxed);
        if (!UnpackHasNew(currentShared)) {
            // No new data; continue using current read slot if valid
            return m_hasReadEver ? &m_slots[m_readIndex] : nullptr;
        }

        // Try to atomically swap newest slot into readIndex and clear hasNew
        do {
            if (!UnpackHasNew(currentShared)) {
                break;
            }
            uint8_t newestIndex = UnpackIndex(currentShared);
            uint32_t newShared = PackState(m_readIndex, false);
            if (m_sharedState.compare_exchange_weak(
                    currentShared,
                    newShared,
                    std::memory_order_acquire,
                    std::memory_order_relaxed)) {
                m_readIndex = newestIndex;
                m_hasReadEver = true;
                break;
            }
        } while (true);

        return m_hasReadEver ? &m_slots[m_readIndex] : nullptr;
    }

private:
    static constexpr uint32_t PackState(uint8_t index, bool hasNew) noexcept {
        return static_cast<uint32_t>(index) | (hasNew ? 0x100u : 0u);
    }
    static constexpr uint8_t UnpackIndex(uint32_t state) noexcept {
        return static_cast<uint8_t>(state & 0xFFu);
    }
    static constexpr bool UnpackHasNew(uint32_t state) noexcept {
        return (state & 0x100u) != 0;
    }

    std::array<RenderMeshData, 3> m_slots;
    uint8_t m_readIndex{0};
    uint8_t m_writeIndex{1};
    bool m_hasReadEver{false};
    std::atomic<uint32_t> m_sharedState{0};
};

} // namespace Tessera

