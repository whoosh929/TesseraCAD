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
                m_threadPool->Enqueue([node, device, &exchange]() {
                    node->Evaluate(device, exchange);
                });
            } else {
                node->Evaluate(device, exchange);
            }
        }
    }

    [[nodiscard]] size_t GetNodeCount() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_nodes.size();
    }

private:
    std::shared_ptr<ThreadPool> m_threadPool;
    std::vector<std::shared_ptr<DAGNode>> m_nodes;
    mutable std::mutex m_graphMutex;
};

} // namespace Tessera

