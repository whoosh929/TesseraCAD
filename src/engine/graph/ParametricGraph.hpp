#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "engine/concurrency/ThreadPool.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/geom/Transform3D.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <iostream>
#include <limits>
#include <cmath>

namespace Tessera {

/// Möller-Trumbore ray-triangle intersection test (two-sided)
inline bool RayIntersectsTriangle(
    const Geom::Point3D& rayOrigin,
    const Geom::Vector3D& rayDir,
    const Geom::Point3D& v0,
    const Geom::Point3D& v1,
    const Geom::Point3D& v2,
    double& outT
) noexcept {
    constexpr double EPSILON = 1e-8;

    Geom::Vector3D edge1 = v1 - v0;
    Geom::Vector3D edge2 = v2 - v0;
    Geom::Vector3D h = rayDir.cross(edge2);
    double a = edge1.dot(h);

    if (std::abs(a) < EPSILON) {
        return false; // Ray is parallel to triangle
    }

    double f = 1.0 / a;
    Geom::Vector3D s = rayOrigin - v0;
    double u = f * s.dot(h);

    if (u < -EPSILON || u > 1.0 + EPSILON) {
        return false;
    }

    Geom::Vector3D q = s.cross(edge1);
    double v = f * rayDir.dot(q);

    if (v < -EPSILON || (u + v) > 1.0 + EPSILON) {
        return false;
    }

    double t = f * edge2.dot(q);
    if (t > EPSILON) {
        outT = t;
        return true;
    }

    return false;
}

class ParametricGraph {
public:
    explicit ParametricGraph(std::shared_ptr<ThreadPool> threadPool)
        : m_threadPool(std::move(threadPool)) {}

    void AddNode(std::shared_ptr<DAGNode> node) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        if (node) {
            if (auto child = node->GetChildSketch()) {
                if (auto sketch = std::dynamic_pointer_cast<SketchNode>(child)) {
                    sketch->SetParentFeature(node);
                    sketch->SetVisible(false);
                    node->AddInput(sketch);
                }
            }
        }
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
                    PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                });
            } else {
                node->Evaluate(device, exchange);
                PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
            }
        }
    }

    void SetSelectedNodeId(uint64_t id) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedNodeId = id;
    }

    [[nodiscard]] uint64_t GetSelectedNodeId() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedNodeId;
    }

    void PublishCombinedSceneMesh(
        RHI::RHIDevice* device, 
        AtomicMeshExchange& exchange,
        std::optional<uint64_t> selectedNodeIdOverride = std::nullopt
    ) {
        if (!device) return;
        std::lock_guard<std::mutex> lock(m_graphMutex);
        if (selectedNodeIdOverride.has_value()) {
            m_selectedNodeId = *selectedNodeIdOverride;
        }

        std::vector<CADVertex> combinedVertices;
        std::vector<uint32_t> combinedIndices;
        std::vector<uint32_t> combinedEdges;

        std::vector<CADVertex> sketchVertices;
        std::vector<uint32_t> sketchIndices;
        std::vector<uint32_t> sketchEdges;

        for (const auto& node : m_nodes) {
            if (!node) continue;

            if (auto sketch = std::dynamic_pointer_cast<SketchNode>(node)) {
                // Child sketches do not render standalone (parent solid feature renders)
                if (sketch->HasParentFeature()) {
                    continue;
                }
                // Sketches not associated with a feature must NOT show up unless selected on the feature tree
                if (sketch->GetId() != m_selectedNodeId) {
                    continue;
                }

                const auto& verts = sketch->GetCachedVertices();
                const auto& indices = sketch->GetCachedIndices();
                const auto& edges = sketch->GetCachedEdges();

                uint32_t vOffset = static_cast<uint32_t>(sketchVertices.size());
                sketchVertices.insert(sketchVertices.end(), verts.begin(), verts.end());

                for (uint32_t idx : indices) {
                    sketchIndices.push_back(idx + vOffset);
                }
                for (uint32_t edgeIdx : edges) {
                    sketchEdges.push_back(edgeIdx + vOffset);
                }
            } else {
                // Solid 3D features (Extrude, Revolve, etc.)
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
        }

        std::shared_ptr<RHI::RHIBuffer> vb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> ib = nullptr;
        std::shared_ptr<RHI::RHIBuffer> edgeIb = nullptr;
        uint32_t triCount = 0;
        uint32_t edgeCount = 0;

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
            vb = device->CreateBuffer(vbDesc, combinedVertices.data());
            ib = device->CreateBuffer(ibDesc, combinedIndices.data());
            triCount = static_cast<uint32_t>(combinedIndices.size());

            if (!combinedEdges.empty()) {
                RHI::RHIBufferDesc edgeDesc{
                    .sizeInBytes = combinedEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Scene_Combined_EdgeIB"
                };
                edgeIb = device->CreateBuffer(edgeDesc, combinedEdges.data());
                edgeCount = static_cast<uint32_t>(combinedEdges.size());
            }
        }

        std::shared_ptr<RHI::RHIBuffer> sketchVb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> sketchIb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> sketchEdgeIb = nullptr;
        uint32_t sketchTriCount = 0;
        uint32_t sketchEdgeCount = 0;

        if (!sketchVertices.empty() && !sketchIndices.empty()) {
            RHI::RHIBufferDesc svbDesc{
                .sizeInBytes = sketchVertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Sketch_Transparent_VB"
            };
            RHI::RHIBufferDesc sibDesc{
                .sizeInBytes = sketchIndices.size() * sizeof(uint32_t),
                .usage = RHI::BufferUsage::Index,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Sketch_Transparent_IB"
            };
            sketchVb = device->CreateBuffer(svbDesc, sketchVertices.data());
            sketchIb = device->CreateBuffer(sibDesc, sketchIndices.data());
            sketchTriCount = static_cast<uint32_t>(sketchIndices.size());

            if (!sketchEdges.empty()) {
                RHI::RHIBufferDesc sedgeDesc{
                    .sizeInBytes = sketchEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Sketch_Outline_EdgeIB"
                };
                sketchEdgeIb = device->CreateBuffer(sedgeDesc, sketchEdges.data());
                sketchEdgeCount = static_cast<uint32_t>(sketchEdges.size());
            }
        }

        exchange.Publish(
            vb, 
            ib, 
            triCount, 
            s_sceneGen.fetch_add(1, std::memory_order_relaxed),
            edgeIb,
            edgeCount,
            sketchVb,
            sketchIb,
            sketchTriCount,
            sketchEdgeIb,
            sketchEdgeCount
        );
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
        for (auto& node : m_nodes) {
            if (node && node->GetId() == id) {
                // If this feature has a child sketch, promote the sketch to parentless
                node->DetachChildSketch();
                break;
            }
        }
        m_nodes.erase(std::remove_if(m_nodes.begin(), m_nodes.end(), [id](const auto& n) {
            return n->GetId() == id;
        }), m_nodes.end());
    }

    void ClearNodes() {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_nodes.clear();
    }

    /// Casts a ray through all DAG nodes and returns the ID of the closest intersected node (0 if none hit)
    [[nodiscard]] uint64_t PickNode(
        const Geom::Point3D& rayOrigin,
        const Geom::Vector3D& rayDir,
        double* outDist = nullptr
    ) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        uint64_t closestNodeId = 0;
        double closestDist = std::numeric_limits<double>::infinity();

        for (const auto& node : m_nodes) {
            if (!node) continue;

            // If it's a sketch node:
            // 1) Child sketches under a feature are picked via the feature's 3D solid
            // 2) Parentless sketches do not show up unless selected on the feature tree
            if (auto sketch = std::dynamic_pointer_cast<SketchNode>(node)) {
                if (sketch->HasParentFeature()) continue;
                if (sketch->GetId() != m_selectedNodeId) continue;
            }

            const auto& verts = node->GetCachedVertices();
            const auto& indices = node->GetCachedIndices();

            if (verts.empty() || indices.size() < 3) continue;

            const size_t triCount = indices.size() / 3;
            for (size_t i = 0; i < triCount; ++i) {
                uint32_t idx0 = indices[i * 3 + 0];
                uint32_t idx1 = indices[i * 3 + 1];
                uint32_t idx2 = indices[i * 3 + 2];

                if (idx0 >= verts.size() || idx1 >= verts.size() || idx2 >= verts.size()) {
                    continue;
                }

                Geom::Point3D v0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
                Geom::Point3D v1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
                Geom::Point3D v2(verts[idx2].position[0], verts[idx2].position[1], verts[idx2].position[2]);

                double t = 0.0;
                if (RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, t)) {
                    if (t < closestDist) {
                        closestDist = t;
                        closestNodeId = node->GetId();
                    }
                }
            }
        }

        if (outDist && closestNodeId != 0) {
            *outDist = closestDist;
        }

        return closestNodeId;
    }

private:
    std::shared_ptr<ThreadPool> m_threadPool;
    std::vector<std::shared_ptr<DAGNode>> m_nodes;
    uint64_t m_selectedNodeId{0};
    mutable std::mutex m_graphMutex;
    static inline std::atomic<uint64_t> s_sceneGen{1};
};

} // namespace Tessera

