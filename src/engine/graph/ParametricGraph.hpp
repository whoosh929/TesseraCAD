#pragma once

#include "Node.hpp"
#include "engine/concurrency/ThreadPool.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/hal/RHI.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <iostream>

namespace Tessera {

class ParametricGraph {
public:
    explicit ParametricGraph(std::shared_ptr<ThreadPool> threadPool)
        : m_threadPool(std::move(threadPool)) {}

    void AddNode(std::shared_ptr<DAGNode> node) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_nodes.push_back(node);
    }

    /// Evaluates any dirty nodes asynchronously via the ThreadPool
    void EvaluateDirtyNodes(RHI::RHIDevice* device, AtomicMeshExchange& exchange) {
        std::vector<std::shared_ptr<DAGNode>> dirtyList;
        {
            std::lock_guard<std::mutex> lock(m_graphMutex);
            for (const auto& node : m_nodes) {
                if (node->IsDirty()) {
                    dirtyList.push_back(node);
                }
            }
        }

        for (auto& node : dirtyList) {
            if (m_threadPool) {
                // Submit to background thread pool
                m_threadPool->Enqueue([this, node, device, &exchange]() {
                    node->Evaluate(device, exchange);
                    PublishCombinedSceneMesh(device, exchange);
                });
            } else {
                node->Evaluate(device, exchange);
                PublishCombinedSceneMesh(device, exchange);
            }
        }
    }

    void PublishCombinedSceneMesh(RHI::RHIDevice* device, AtomicMeshExchange& exchange) {
        if (!device) return;
        std::lock_guard<std::mutex> lock(m_graphMutex);
        std::vector<CADVertex> combinedVertices;
        std::vector<uint32_t> combinedIndices;
        std::vector<uint32_t> combinedEdges;

        for (const auto& node : m_nodes) {
            const auto& verts = node->GetCachedVertices();
            const auto& indices = node->GetCachedIndices();
            const auto& edges = node->GetCachedEdges();

            uint32_t vOffset = static_cast<uint32_t>(combinedVertices.size());
            combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());

            for (uint32_t idx : indices) {
                combinedIndices.push_back(idx + vOffset);
            }
            for (uint32_t edgeIdx : edges) {
                combinedEdges.push_back(edgeIdx + vOffset);
            }
        }

        if (!combinedVertices.empty() && !combinedIndices.empty()) {
            RHI::RHIBufferDesc vbDesc{
                .sizeInBytes = combinedVertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Scene_Combined_VB"
            };
            RHI::RHIBufferDesc ibDesc{
                .sizeInBytes = combinedIndices.size() * sizeof(uint32_t),
                .usage = RHI::BufferUsage::Index,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Scene_Combined_IB"
            };
            auto vb = device->CreateBuffer(vbDesc, combinedVertices.data());
            auto ib = device->CreateBuffer(ibDesc, combinedIndices.data());

            std::shared_ptr<RHI::RHIBuffer> edgeIb = nullptr;
            if (!combinedEdges.empty()) {
                RHI::RHIBufferDesc edgeDesc{
                    .sizeInBytes = combinedEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Scene_Combined_EdgeIB"
                };
                edgeIb = device->CreateBuffer(edgeDesc, combinedEdges.data());
            }

            exchange.Publish(
                vb, 
                ib, 
                static_cast<uint32_t>(combinedIndices.size()), 
                s_sceneGen.fetch_add(1, std::memory_order_relaxed),
                edgeIb,
                static_cast<uint32_t>(combinedEdges.size())
            );
        } else {
            exchange.Publish(nullptr, nullptr, 0, s_sceneGen.fetch_add(1, std::memory_order_relaxed), nullptr, 0);
        }
    }

    [[nodiscard]] size_t GetNodeCount() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_nodes.size();
    }

    [[nodiscard]] std::vector<std::shared_ptr<DAGNode>> GetNodes() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_nodes;
    }

    void RemoveNode(uint64_t id) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_nodes.erase(std::remove_if(m_nodes.begin(), m_nodes.end(), [id](const auto& n) {
            return n->GetId() == id;
        }), m_nodes.end());
    }

    void ClearNodes() {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_nodes.clear();
    }

private:
    std::shared_ptr<ThreadPool> m_threadPool;
    std::vector<std::shared_ptr<DAGNode>> m_nodes;
    mutable std::mutex m_graphMutex;
    static inline std::atomic<uint64_t> s_sceneGen{1};
};

} // namespace Tessera

