#pragma once

#include "GeometryTypes.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <cstdint>

namespace Tessera {

class DAGNode : public std::enable_shared_from_this<DAGNode> {
public:
    explicit DAGNode(std::string name)
        : m_name(std::move(name)),
          m_id(s_nextId.fetch_add(1, std::memory_order_relaxed)),
          m_isDirty(true),
          m_generation(1) {}

    virtual ~DAGNode() = default;

    [[nodiscard]] uint64_t GetId() const noexcept { return m_id; }
    [[nodiscard]] const std::string& GetName() const noexcept { return m_name; }
    [[nodiscard]] bool IsDirty() const noexcept { return m_isDirty.load(std::memory_order_acquire); }
    [[nodiscard]] uint64_t GetGeneration() const noexcept { return m_generation.load(std::memory_order_acquire); }

    [[nodiscard]] const std::vector<CADVertex>& GetCachedVertices() const noexcept { return m_cachedVertices; }
    [[nodiscard]] const std::vector<uint32_t>& GetCachedIndices() const noexcept { return m_cachedIndices; }
    [[nodiscard]] const std::vector<uint32_t>& GetCachedEdges() const noexcept { return m_cachedEdges; }

    void SetPosition(float x, float y, float z) {
        m_position[0] = x;
        m_position[1] = y;
        m_position[2] = z;
        MarkDirty();
    }
    [[nodiscard]] const float* GetPosition() const noexcept { return m_position; }

    /// Child sketch association for feature nodes (Extrude, Revolve)
    [[nodiscard]] virtual std::shared_ptr<DAGNode> GetParentFeature() const noexcept { return nullptr; }
    [[nodiscard]] virtual std::shared_ptr<DAGNode> GetChildSketch() const noexcept { return nullptr; }
    [[nodiscard]] virtual bool HasParentFeature() const noexcept { return false; }
    virtual void DetachChildSketch() noexcept {}

    /// Connects this node to depend on an upstream parent
    void AddInput(std::shared_ptr<DAGNode> parent) {
        m_inputs.push_back(parent);
        parent->m_dependents.push_back(weak_from_this());
    }

    /// Flags this node as dirty and propagates downstream through the DAG
    void MarkDirty() {
        m_isDirty.store(true, std::memory_order_release);
        m_generation.fetch_add(1, std::memory_order_release);

        for (auto it = m_dependents.begin(); it != m_dependents.end();) {
            if (auto dependent = it->lock()) {
                dependent->MarkDirty();
                ++it;
            } else {
                it = m_dependents.erase(it);
            }
        }
    }

    /// Clear dirty flag after evaluation completes
    void ClearDirty() noexcept {
        m_isDirty.store(false, std::memory_order_release);
    }

    /// Pure virtual evaluation invoked asynchronously on the Worker Thread Pool
    virtual void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) = 0;

protected:
    std::string m_name;
    uint64_t m_id;
    std::atomic<bool> m_isDirty;
    std::atomic<uint64_t> m_generation;

    float m_position[3]{0.0f, 0.0f, 0.0f};
    std::vector<CADVertex> m_cachedVertices;
    std::vector<uint32_t> m_cachedIndices;
    std::vector<uint32_t> m_cachedEdges;

    std::vector<std::shared_ptr<DAGNode>> m_inputs;
    std::vector<std::weak_ptr<DAGNode>> m_dependents;

private:
    static inline std::atomic<uint64_t> s_nextId{1};
};

} // namespace Tessera

