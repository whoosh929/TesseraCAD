#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "DatumPlaneNode.hpp"
#include "DatumAxisNode.hpp"
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
#include <unordered_set>

namespace Tessera {

/// Pick result for sub-elements (Face, Vertex, or Edge) on a selected 3D model
struct SubElementPickResult {
    uint64_t nodeId{0};
    int faceIndex{-1};
    int vertexIndex{-1};
    int edgeIndex{-1};
    Geom::Point3D edgeP0{0.0, 0.0, 0.0};
    Geom::Point3D edgeP1{0.0, 0.0, 0.0};
    Geom::Plane facePlane;
    Geom::Point3D vertexPos{0.0, 0.0, 0.0};
    double distance{std::numeric_limits<double>::infinity()};
};

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
            if (auto plane = std::dynamic_pointer_cast<DatumPlaneNode>(node)) {
                if (auto feat = plane->GetReferencedFeature()) {
                    plane->AddInput(feat);
                }
            } else if (auto axis = std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                if (auto feat = axis->GetReferencedFeature()) {
                    axis->AddInput(feat);
                }
            } else if (auto sketch = std::dynamic_pointer_cast<SketchNode>(node)) {
                if (auto datumPlane = sketch->GetDatumPlaneReference()) {
                    sketch->AddInput(datumPlane);
                }
            }
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
        std::vector<std::shared_ptr<DAGNode>> dirtyMetadataNodes;
        std::vector<std::shared_ptr<DAGNode>> dirtyList;
        {
            std::lock_guard<std::mutex> lock(m_graphMutex);
            for (const auto& node : m_nodes) {
                if (node->IsDirty()) {
                    if (std::dynamic_pointer_cast<DatumPlaneNode>(node) ||
                        std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                        dirtyMetadataNodes.push_back(node);
                    } else {
                        dirtyList.push_back(node);
                    }
                }
            }
        }

        // Evaluate lightweight datum planes/axes synchronously first so dependent sketches and features read updated planes
        for (auto& metaNode : dirtyMetadataNodes) {
            metaNode->Evaluate(device, exchange);
        }

        for (auto& node : dirtyList) {
            if (std::dynamic_pointer_cast<SketchNode>(node)) {
                // Evaluate 2D sketches synchronously first so downstream 3D features and the viewport immediately have the updated 3D sketch plane/mesh
                node->Evaluate(device, exchange);
                PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
            } else if (m_threadPool) {
                // Submit heavy 3D tessellations to background thread pool
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
        m_selectedFaceIndex = -1;
        m_selectedVertexIndex = -1;
        m_selectedEdgeIndex = -1;
    }

    [[nodiscard]] uint64_t GetSelectedNodeId() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedNodeId;
    }

    void SetSubElementSelection(uint64_t nodeId, int faceIndex, int vertexIndex, int edgeIndex = -1) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedNodeId = nodeId;
        m_selectedFaceIndex = faceIndex;
        m_selectedVertexIndex = vertexIndex;
        m_selectedEdgeIndex = edgeIndex;
    }

    void GetSubElementSelection(uint64_t& outNodeId, int& outFaceIndex, int& outVertexIndex) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        outNodeId = m_selectedNodeId;
        outFaceIndex = m_selectedFaceIndex;
        outVertexIndex = m_selectedVertexIndex;
    }

    [[nodiscard]] int GetSelectedFaceIndex() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedFaceIndex;
    }

    [[nodiscard]] int GetSelectedVertexIndex() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedVertexIndex;
    }

    [[nodiscard]] int GetSelectedEdgeIndex() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedEdgeIndex;
    }

    void PublishCombinedSceneMesh(
        RHI::RHIDevice* device, 
        AtomicMeshExchange& exchange,
        std::optional<uint64_t> selectedNodeIdOverride = std::nullopt,
        std::optional<int> selectedFaceIndexOverride = std::nullopt,
        std::optional<int> selectedVertexIndexOverride = std::nullopt,
        std::optional<int> selectedEdgeIndexOverride = std::nullopt
    ) {
        if (!device) return;
        std::lock_guard<std::mutex> lock(m_graphMutex);
        if (selectedNodeIdOverride.has_value()) {
            if (*selectedNodeIdOverride != m_selectedNodeId) {
                m_selectedNodeId = *selectedNodeIdOverride;
                m_selectedFaceIndex = -1;
                m_selectedVertexIndex = -1;
                m_selectedEdgeIndex = -1;
            }
        }
        if (selectedFaceIndexOverride.has_value()) {
            m_selectedFaceIndex = *selectedFaceIndexOverride;
        }
        if (selectedVertexIndexOverride.has_value()) {
            m_selectedVertexIndex = *selectedVertexIndexOverride;
        }
        if (selectedEdgeIndexOverride.has_value()) {
            m_selectedEdgeIndex = *selectedEdgeIndexOverride;
        }

        std::vector<CADVertex> combinedVertices;
        std::vector<uint32_t> combinedIndices;
        std::vector<uint32_t> combinedEdges;

        std::vector<CADVertex> selectedVertices;
        std::vector<uint32_t> selectedIndices;
        std::vector<uint32_t> selectedEdges;

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
                // Solid 3D features (Extrude, Revolve, Cylinder, Sphere, Fillet, Chamfer, etc.)
                if (node->IsConsumed()) {
                    continue; // Consumed features are superseded by their downstream operations
                }

                // If this node is an earlier sibling targeting the same base feature as an active downstream/selected sibling, skip it
                bool superseded = false;
                for (const auto& other : m_nodes) {
                    if (other && other != node && !other->IsConsumed()) {
                        if (!node->GetInputs().empty() && !other->GetInputs().empty()) {
                            if (node->GetInputs()[0] == other->GetInputs()[0]) {
                                if (other->GetId() == m_selectedNodeId || other->GetId() > node->GetId()) {
                                    superseded = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (superseded) continue;

                const auto& verts = node->GetCachedVertices();
                const auto& indices = node->GetCachedIndices();
                const auto& edges = node->GetCachedEdges();
                if (verts.empty() || indices.empty()) continue;

                bool isSelected = (node->GetId() == m_selectedNodeId);

                if (isSelected && m_selectedEdgeIndex >= 0) {
                    // Edge drill-down selection: render whole model + ONLY selected corner edge in highlighted selectedEdges buffer!
                    uint32_t unselBase = static_cast<uint32_t>(combinedVertices.size());
                    combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());
                    for (uint32_t idx : indices) combinedIndices.push_back(idx + unselBase);
                    for (uint32_t edgeIdx : edges) combinedEdges.push_back(edgeIdx + unselBase);

                    size_t e = static_cast<size_t>(m_selectedEdgeIndex * 2);
                    if (e + 1 < edges.size()) {
                        uint32_t selBase = static_cast<uint32_t>(selectedVertices.size());
                        selectedVertices.insert(selectedVertices.end(), verts.begin(), verts.end());
                        selectedEdges.push_back(edges[e] + selBase);
                        selectedEdges.push_back(edges[e + 1] + selBase);
                    }
                } else if (isSelected && m_selectedFaceIndex >= 0) {
                    // Face drill-down selection: ONLY highlight that face!
                    FaceInfo selFace;
                    if (node->GetFace(m_selectedFaceIndex, selFace) && !selFace.triangleIndices.empty()) {
                        // Put selected face triangles + its perimeter outline into selected buffers
                        uint32_t selBase = static_cast<uint32_t>(selectedVertices.size());
                        selectedVertices.insert(selectedVertices.end(), verts.begin(), verts.end());
                        for (uint32_t idx : selFace.triangleIndices) {
                            selectedIndices.push_back(idx + selBase);
                        }
                        for (uint32_t edgeIdx : selFace.perimeterEdges) {
                            selectedEdges.push_back(edgeIdx + selBase);
                        }

                        // Put all non-selected triangles + non-perimeter edges into unselected combined buffers
                        std::unordered_set<size_t> faceTriIndices;
                        for (size_t k = 0; k < selFace.triangleIndices.size(); k += 3) {
                            uint32_t fa = selFace.triangleIndices[k + 0];
                            uint32_t fb = selFace.triangleIndices[k + 1];
                            uint32_t fc = selFace.triangleIndices[k + 2];
                            for (size_t t = 0; t < indices.size() / 3; ++t) {
                                if (indices[t * 3 + 0] == fa && indices[t * 3 + 1] == fb && indices[t * 3 + 2] == fc) {
                                    faceTriIndices.insert(t);
                                }
                            }
                        }

                        uint32_t unselBase = static_cast<uint32_t>(combinedVertices.size());
                        combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());
                        for (size_t t = 0; t < indices.size() / 3; ++t) {
                            if (faceTriIndices.find(t) == faceTriIndices.end()) {
                                combinedIndices.push_back(indices[t * 3 + 0] + unselBase);
                                combinedIndices.push_back(indices[t * 3 + 1] + unselBase);
                                combinedIndices.push_back(indices[t * 3 + 2] + unselBase);
                            }
                        }
                        for (uint32_t edgeIdx : edges) {
                            combinedEdges.push_back(edgeIdx + unselBase);
                        }
                    } else {
                        // Fallback to whole model selection
                        uint32_t vOffset = static_cast<uint32_t>(selectedVertices.size());
                        selectedVertices.insert(selectedVertices.end(), verts.begin(), verts.end());
                        for (uint32_t idx : indices) selectedIndices.push_back(idx + vOffset);
                        for (uint32_t edgeIdx : edges) selectedEdges.push_back(edgeIdx + vOffset);
                    }
                } else if (isSelected && m_selectedVertexIndex >= 0 && static_cast<size_t>(m_selectedVertexIndex) < verts.size()) {
                    // Vertex drill-down selection: render model normally + 3D marker for vertex + highlight all incident edges in cyan!
                    uint32_t unselBase = static_cast<uint32_t>(combinedVertices.size());
                    combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());
                    for (uint32_t idx : indices) combinedIndices.push_back(idx + unselBase);
                    for (uint32_t edgeIdx : edges) combinedEdges.push_back(edgeIdx + unselBase);

                    // 1. Highlight all incident model edges connecting to this selected vertex in vibrant electric cyan!
                    uint32_t vIdx = static_cast<uint32_t>(m_selectedVertexIndex);
                    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
                        if (edges[e] == vIdx || edges[e + 1] == vIdx) {
                            uint32_t selBase = static_cast<uint32_t>(selectedVertices.size());
                            selectedVertices.push_back(verts[edges[e]]);
                            selectedVertices.push_back(verts[edges[e + 1]]);
                            selectedEdges.push_back(selBase);
                            selectedEdges.push_back(selBase + 1);
                        }
                    }

                    // 2. Prominent 3D diamond octahedron marker for selected vertex
                    const auto& v = verts[m_selectedVertexIndex];
                    float vx = v.position[0];
                    float vy = v.position[1];
                    float vz = v.position[2];
                    float s = 0.22f; // Increased from 0.08f so it is clearly visible in 3D

                    uint32_t markBase = static_cast<uint32_t>(selectedVertices.size());
                    selectedVertices.push_back({.position = {vx, vy + s, vz}, .normal = {0, 1, 0}});
                    selectedVertices.push_back({.position = {vx - s, vy, vz}, .normal = {-1, 0, 0}});
                    selectedVertices.push_back({.position = {vx + s, vy, vz}, .normal = {1, 0, 0}});
                    selectedVertices.push_back({.position = {vx, vy, vz - s}, .normal = {0, 0, -1}});
                    selectedVertices.push_back({.position = {vx, vy, vz + s}, .normal = {0, 0, 1}});
                    selectedVertices.push_back({.position = {vx, vy - s, vz}, .normal = {0, -1, 0}});

                    std::vector<uint32_t> mTris = {
                        0, 1, 3,  0, 3, 2,  0, 2, 4,  0, 4, 1,
                        5, 3, 1,  5, 2, 3,  5, 4, 2,  5, 1, 4
                    };
                    for (uint32_t mi : mTris) selectedIndices.push_back(markBase + mi);
                    std::vector<uint32_t> mEdges = {
                        0, 1,  0, 2,  0, 3,  0, 4,
                        5, 1,  5, 2,  5, 3,  5, 4,
                        1, 3,  3, 2,  2, 4,  4, 1
                    };
                    for (uint32_t me : mEdges) selectedEdges.push_back(markBase + me);

                    // 3. Multi-axis 3D reticle crosshair along X, Y, Z at vertex
                    float axisLen = 0.35f;
                    uint32_t axisBase = static_cast<uint32_t>(selectedVertices.size());
                    selectedVertices.push_back({.position = {vx - axisLen, vy, vz}, .normal = {1, 0, 0}});
                    selectedVertices.push_back({.position = {vx + axisLen, vy, vz}, .normal = {1, 0, 0}});
                    selectedVertices.push_back({.position = {vx, vy - axisLen, vz}, .normal = {0, 1, 0}});
                    selectedVertices.push_back({.position = {vx, vy + axisLen, vz}, .normal = {0, 1, 0}});
                    selectedVertices.push_back({.position = {vx, vy, vz - axisLen}, .normal = {0, 0, 1}});
                    selectedVertices.push_back({.position = {vx, vy, vz + axisLen}, .normal = {0, 0, 1}});
                    selectedEdges.push_back(axisBase + 0);
                    selectedEdges.push_back(axisBase + 1);
                    selectedEdges.push_back(axisBase + 2);
                    selectedEdges.push_back(axisBase + 3);
                    selectedEdges.push_back(axisBase + 4);
                    selectedEdges.push_back(axisBase + 5);
                } else {
                    auto& targetVerts = isSelected ? selectedVertices : combinedVertices;
                    auto& targetIndices = isSelected ? selectedIndices : combinedIndices;
                    auto& targetEdges = isSelected ? selectedEdges : combinedEdges;

                    uint32_t vOffset = static_cast<uint32_t>(targetVerts.size());
                    targetVerts.insert(targetVerts.end(), verts.begin(), verts.end());

                    for (uint32_t idx : indices) {
                        targetIndices.push_back(idx + vOffset);
                    }
                    for (uint32_t edgeIdx : edges) {
                        targetEdges.push_back(edgeIdx + vOffset);
                    }
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

        // Selected 3D Model Buffers (for subtle surface highlight + outer edge highlight)
        std::shared_ptr<RHI::RHIBuffer> selVb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> selIb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> selEdgeIb = nullptr;
        uint32_t selTriCount = 0;
        uint32_t selEdgeCount = 0;

        if (!selectedVertices.empty()) {
            RHI::RHIBufferDesc selVbDesc{
                .sizeInBytes = selectedVertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Selected_Model_VB"
            };
            selVb = device->CreateBuffer(selVbDesc, selectedVertices.data());

            if (!selectedIndices.empty()) {
                RHI::RHIBufferDesc selIbDesc{
                    .sizeInBytes = selectedIndices.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Selected_Model_IB"
                };
                selIb = device->CreateBuffer(selIbDesc, selectedIndices.data());
                selTriCount = static_cast<uint32_t>(selectedIndices.size());
            }

            if (!selectedEdges.empty()) {
                RHI::RHIBufferDesc selEdgeDesc{
                    .sizeInBytes = selectedEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Selected_Model_EdgeIB"
                };
                selEdgeIb = device->CreateBuffer(selEdgeDesc, selectedEdges.data());
                selEdgeCount = static_cast<uint32_t>(selectedEdges.size());
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
            sketchEdgeCount,
            selVb,
            selIb,
            selTriCount,
            selEdgeIb,
            selEdgeCount
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

    [[nodiscard]] std::shared_ptr<DAGNode> GetNode(uint64_t id) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        for (const auto& node : m_nodes) {
            if (node && node->GetId() == id) {
                return node;
            }
        }
        return nullptr;
    }

    void RemoveNode(uint64_t id) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        std::shared_ptr<DAGNode> targetNode = nullptr;
        for (auto& node : m_nodes) {
            if (node && node->GetId() == id) {
                targetNode = node;
                // If this feature has a child sketch, promote the sketch to parentless
                node->DetachChildSketch();
                break;
            }
        }
        if (targetNode) {
            // Unconsume upstream parent features if this node was consuming them
            for (const auto& parent : targetNode->GetInputs()) {
                if (!parent) continue;
                bool stillConsumed = false;
                for (const auto& dep : parent->GetDependents()) {
                    if (dep && dep != targetNode && dep->GetId() != id) {
                        for (const auto& n : m_nodes) {
                            if (n && n == dep && !n->HasConflict()) {
                                stillConsumed = true;
                                break;
                            }
                        }
                    }
                    if (stillConsumed) break;
                }
                if (!stillConsumed) {
                    parent->SetConsumed(false);
                    parent->MarkDirty();
                }
            }

            // If the deleted node is currently selected, update selection to its parent or child
            if (m_selectedNodeId == id) {
                if (targetNode->GetParentFeature()) {
                    m_selectedNodeId = targetNode->GetParentFeature()->GetId();
                } else if (targetNode->GetChildSketch()) {
                    m_selectedNodeId = targetNode->GetChildSketch()->GetId();
                } else {
                    m_selectedNodeId = 0;
                }
            }

            targetNode->NotifyRemoval();
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

            // Never pick consumed solid features (they are hidden/superseded by downstream operations)
            if (node->IsConsumed()) continue;

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

    /// Casts a ray specifically against a selected node to pick its sub-element (Vertex or Face)
    [[nodiscard]] SubElementPickResult PickSubElement(
        const Geom::Point3D& rayOrigin,
        const Geom::Vector3D& rayDir,
        uint64_t targetNodeId,
        double vertexTolerance = 0.25,
        double edgeTolerance = 0.25
    ) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        SubElementPickResult result;
        result.nodeId = targetNodeId;

        std::shared_ptr<DAGNode> targetNode = nullptr;
        for (const auto& node : m_nodes) {
            if (node && node->GetId() == targetNodeId) {
                targetNode = node;
                break;
            }
        }
        if (!targetNode) return result;

        if (targetNode->IsConsumed()) {
            auto leaf = targetNode->GetActiveLeafFeature();
            if (leaf) {
                targetNode = leaf;
                result.nodeId = leaf->GetId();
            }
        }

        const auto& verts = targetNode->GetCachedVertices();
        const auto& indices = targetNode->GetCachedIndices();
        if (verts.empty()) return result;

        // 1. Proximity check to vertices
        int bestVertexIdx = -1;
        double bestVertexDist = vertexTolerance;
        double bestVertexRayT = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < verts.size(); ++i) {
            Geom::Point3D p(verts[i].position[0], verts[i].position[1], verts[i].position[2]);
            Geom::Vector3D toP = p - rayOrigin;
            double t = toP.dot(rayDir);
            if (t > 1e-4) {
                Geom::Point3D rayPt = rayOrigin + rayDir * t;
                double d = p.distance_to(rayPt);
                if (d <= vertexTolerance && (d < bestVertexDist || (std::abs(d - bestVertexDist) < 0.02 && t < bestVertexRayT))) {
                    bestVertexDist = d;
                    bestVertexRayT = t;
                    bestVertexIdx = static_cast<int>(i);
                }
            }
        }

        if (bestVertexIdx >= 0) {
            result.vertexIndex = bestVertexIdx;
            result.vertexPos = Geom::Point3D(verts[bestVertexIdx].position[0], verts[bestVertexIdx].position[1], verts[bestVertexIdx].position[2]);
            result.distance = bestVertexRayT;
            return result;
        }

        // 2. Proximity check to feature edges (corner edges)
        const auto& edges = targetNode->GetCachedEdges();
        double bestEdgeDist = edgeTolerance; // Configurable picking tolerance for corner edges
        double bestEdgeRayT = std::numeric_limits<double>::infinity();
        int bestEdgeIdx = -1;
        Geom::Point3D bestEdgeP0, bestEdgeP1;

        for (size_t e = 0; e + 1 < edges.size(); e += 2) {
            uint32_t i0 = edges[e];
            uint32_t i1 = edges[e + 1];
            if (i0 >= verts.size() || i1 >= verts.size()) continue;

            Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
            Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);

            Geom::Vector3D u = rayDir;
            Geom::Vector3D v = p1 - p0;
            Geom::Vector3D w = rayOrigin - p0;

            double b = u.dot(v);
            double c = v.dot(v);
            double d = u.dot(w);
            double e_dot = v.dot(w);

            double denom = c - b * b;
            double s = (denom > 1e-6) ? (e_dot - b * d) / denom : 0.5;
            s = std::clamp(s, 0.0, 1.0);

            double t = b * s - d;
            if (t > 1e-4) {
                Geom::Point3D rayPt = rayOrigin + u * t;
                Geom::Point3D segPt = p0 + v * s;
                double dist = rayPt.distance_to(segPt);
                if (dist <= bestEdgeDist && (dist < bestEdgeDist || (std::abs(dist - bestEdgeDist) < 0.02 && t < bestEdgeRayT))) {
                    bestEdgeDist = dist;
                    bestEdgeRayT = t;
                    bestEdgeIdx = static_cast<int>(e / 2);
                    bestEdgeP0 = p0;
                    bestEdgeP1 = p1;
                }
            }
        }

        if (bestEdgeIdx >= 0) {
            result.edgeIndex = bestEdgeIdx;
            result.edgeP0 = bestEdgeP0;
            result.edgeP1 = bestEdgeP1;
            result.distance = bestEdgeRayT;
            return result;
        }

        // 3. Intersected face check
        double closestTriT = std::numeric_limits<double>::infinity();
        int hitTriIdx = -1;

        const size_t triCount = indices.size() / 3;
        for (size_t i = 0; i < triCount; ++i) {
            uint32_t idx0 = indices[i * 3 + 0];
            uint32_t idx1 = indices[i * 3 + 1];
            uint32_t idx2 = indices[i * 3 + 2];
            if (idx0 >= verts.size() || idx1 >= verts.size() || idx2 >= verts.size()) continue;

            Geom::Point3D v0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
            Geom::Point3D v1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
            Geom::Point3D v2(verts[idx2].position[0], verts[idx2].position[1], verts[idx2].position[2]);

            double t = 0.0;
            if (RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, t)) {
                if (t < closestTriT) {
                    closestTriT = t;
                    hitTriIdx = static_cast<int>(i);
                }
            }
        }

        if (hitTriIdx >= 0) {
            int faceIdx = targetNode->GetFaceIndexForTriangle(static_cast<size_t>(hitTriIdx));
            FaceInfo face;
            if (faceIdx >= 0 && targetNode->GetFace(faceIdx, face)) {
                result.faceIndex = faceIdx;
                result.facePlane = face.plane;
            } else {
                result.faceIndex = 0;
                uint32_t i0 = indices[hitTriIdx * 3 + 0];
                uint32_t i1 = indices[hitTriIdx * 3 + 1];
                uint32_t i2 = indices[hitTriIdx * 3 + 2];
                Geom::Point3D v0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
                Geom::Point3D v1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
                Geom::Point3D v2(verts[i2].position[0], verts[i2].position[1], verts[i2].position[2]);
                result.facePlane = Geom::Plane(v0, v1, v2);
            }
            result.distance = closestTriT;
            return result;
        }

        return result;
    }

private:
    std::shared_ptr<ThreadPool> m_threadPool;
    std::vector<std::shared_ptr<DAGNode>> m_nodes;
    uint64_t m_selectedNodeId{0};
    int m_selectedFaceIndex{-1};
    int m_selectedVertexIndex{-1};
    int m_selectedEdgeIndex{-1};
    mutable std::mutex m_graphMutex;
    static inline std::atomic<uint64_t> s_sceneGen{1};
};

} // namespace Tessera

