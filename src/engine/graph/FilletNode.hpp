#pragma once

#include "Node.hpp"
#include "ExtrudeNode.hpp"
#include "Tessellation.hpp"
#include "engine/sketch/SketchFilletChamfer.hpp"
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Tessera {

class FilletNode : public DAGNode {
public:
    FilletNode(
        std::string name,
        std::shared_ptr<DAGNode> targetNode,
        int edgeIndex,
        float radius = 0.2f
    ) : DAGNode(std::move(name)),
        m_targetNode(targetNode),
        m_targetNodeId(targetNode ? targetNode->GetId() : 0),
        m_edgeIndex(edgeIndex),
        m_radius(std::max(0.01f, radius)) {
        if (m_targetNode) {
            m_targetNode->SetConsumed(true);
        }
    }

    ~FilletNode() override {
        if (m_targetNode) {
            m_targetNode->SetConsumed(false);
        }
    }

    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_targetNode; }
    [[nodiscard]] int GetReferencedEdgeIndex() const noexcept { return m_edgeIndex; }
    [[nodiscard]] float GetRadius() const noexcept { return m_radius; }
    [[nodiscard]] Geom::Point3D GetEdgeP0() const noexcept { return m_edgeP0; }
    [[nodiscard]] Geom::Point3D GetEdgeP1() const noexcept { return m_edgeP1; }

    void SetRadius(float radius) {
        float r = std::max(0.01f, radius);
        if (std::abs(m_radius - r) > 1e-4f) {
            m_radius = r;
            MarkDirty();
        }
    }

    void SetEdgeReference(std::shared_ptr<DAGNode> targetNode, int edgeIndex) {
        if (m_targetNode && m_targetNode != targetNode) {
            m_targetNode->SetConsumed(false);
        }
        m_targetNode = targetNode;
        m_targetNodeId = targetNode ? targetNode->GetId() : 0;
        m_edgeIndex = edgeIndex;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (m_targetNode) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
        MarkDirty();
    }

    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }

    void OnParentRemoved(uint64_t parentId) override {
        if (m_targetNodeId == parentId) {
            m_targetNode = nullptr;
            m_hasConflict = true;
            m_conflictMessage = "Referenced parent model feature was deleted.";
            MarkDirty();
        }
        DAGNode::OnParentRemoved(parentId);
    }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;

        if (m_targetNode) {
            bool alreadyInput = false;
            for (const auto& in : m_inputs) {
                if (in && in->GetId() == m_targetNode->GetId()) {
                    alreadyInput = true;
                    break;
                }
            }
            if (!alreadyInput) {
                AddInput(m_targetNode);
            }
        }

        if (!m_targetNode) {
            m_hasConflict = true;
            m_conflictMessage = "No parent model feature referenced.";
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            ClearDirty();
            return;
        }

        if (m_targetNode->HasConflict()) {
            m_hasConflict = true;
            m_conflictMessage = "Referenced model feature has a conflict: " + m_targetNode->GetConflictMessage();
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            ClearDirty();
            return;
        }

        const auto& baseEdges = m_targetNode->GetCachedEdges();
        const auto& baseVerts = m_targetNode->GetCachedVertices();
        const auto& baseIndices = m_targetNode->GetCachedIndices();

        if (m_edgeIndex < 0 || static_cast<size_t>(m_edgeIndex * 2 + 1) >= baseEdges.size() || baseVerts.empty()) {
            m_hasConflict = true;
            m_conflictMessage = "Referenced edge index #" + std::to_string(m_edgeIndex) + " does not exist on target feature.";
            m_cachedVertices = baseVerts;
            m_cachedIndices = baseIndices;
            m_cachedEdges = baseEdges;
            ComputeFaces();
            ClearDirty();
            return;
        }

        m_hasConflict = false;
        m_conflictMessage.clear();
        m_targetNode->SetConsumed(true);

        uint32_t i0 = baseEdges[m_edgeIndex * 2];
        uint32_t i1 = baseEdges[m_edgeIndex * 2 + 1];
        m_edgeP0 = Geom::Point3D(baseVerts[i0].position[0], baseVerts[i0].position[1], baseVerts[i0].position[2]);
        m_edgeP1 = Geom::Point3D(baseVerts[i1].position[0], baseVerts[i1].position[1], baseVerts[i1].position[2]);

        // Method 1: If target is an ExtrudeNode and edge is a vertical corner edge, apply 2D fillet to profile
        if (auto extrude = std::dynamic_pointer_cast<ExtrudeNode>(m_targetNode)) {
            const auto& profile = extrude->GetProfile();
            float h = extrude->GetHeight();
            bool foundCorner = false;
            size_t cornerIdx = 0;

            for (size_t p = 0; p < profile.size(); ++p) {
                float u = profile[p][0] + extrude->GetPosition()[0];
                float v = profile[p][1] + extrude->GetPosition()[2];
                float dx0 = std::abs(static_cast<float>(m_edgeP0.x) - u);
                float dz0 = std::abs(static_cast<float>(m_edgeP0.z) - v);
                float dx1 = std::abs(static_cast<float>(m_edgeP1.x) - u);
                float dz1 = std::abs(static_cast<float>(m_edgeP1.z) - v);

                if ((dx0 < 0.1f && dz0 < 0.1f) || (dx1 < 0.1f && dz1 < 0.1f)) {
                    cornerIdx = p;
                    foundCorner = true;
                    break;
                }
            }

            if (foundCorner && profile.size() >= 3) {
                auto filletedProfile = profile;
                Sketch::ApplyFillet2D(filletedProfile, cornerIdx, m_radius, true, 6);

                ExtrudeNode tempNode(m_name + "_Eval", filletedProfile, h);
                tempNode.SetPosition(extrude->GetPosition()[0], extrude->GetPosition()[1], extrude->GetPosition()[2]);
                tempNode.Evaluate(device, exchange);

                m_cachedVertices = tempNode.GetCachedVertices();
                m_cachedIndices = tempNode.GetCachedIndices();
                m_cachedEdges = tempNode.GetCachedEdges();
                ComputeFaces();
                ClearDirty();
                return;
            }
        }

        // Method 2: General 3D Edge Bevel/Round Generator
        std::vector<CADVertex> vertices = baseVerts;
        std::vector<uint32_t> indices = baseIndices;

        Geom::Vector3D edgeVec = (m_edgeP1 - m_edgeP0).normalized();
        Geom::Vector3D n0(baseVerts[i0].normal[0], baseVerts[i0].normal[1], baseVerts[i0].normal[2]);
        Geom::Vector3D n1(baseVerts[i1].normal[0], baseVerts[i1].normal[1], baseVerts[i1].normal[2]);

        Geom::Vector3D u1 = edgeVec.cross(n0).normalized();
        Geom::Vector3D u2 = n1.cross(edgeVec).normalized();

        const int numBands = 5;
        uint32_t baseV = static_cast<uint32_t>(vertices.size());

        for (int b = 0; b <= numBands; ++b) {
            float t = static_cast<float>(b) / static_cast<float>(numBands);
            Geom::Vector3D blendedDir = (u1 * (1.0f - t) + u2 * t).normalized();
            Geom::Vector3D filletNorm = (n0 * (1.0f - t) + n1 * t).normalized();

            Geom::Point3D pt0 = m_edgeP0 + blendedDir * m_radius;
            Geom::Point3D pt1 = m_edgeP1 + blendedDir * m_radius;

            vertices.push_back({
                {static_cast<float>(pt0.x), static_cast<float>(pt0.y), static_cast<float>(pt0.z)},
                {static_cast<float>(filletNorm.x), static_cast<float>(filletNorm.y), static_cast<float>(filletNorm.z)}
            });
            vertices.push_back({
                {static_cast<float>(pt1.x), static_cast<float>(pt1.y), static_cast<float>(pt1.z)},
                {static_cast<float>(filletNorm.x), static_cast<float>(filletNorm.y), static_cast<float>(filletNorm.z)}
            });
        }

        for (int b = 0; b < numBands; ++b) {
            uint32_t v0 = baseV + b * 2 + 0;
            uint32_t v1 = baseV + b * 2 + 1;
            uint32_t v2 = baseV + (b + 1) * 2 + 1;
            uint32_t v3 = baseV + (b + 1) * 2 + 0;

            indices.push_back(v0);
            indices.push_back(v1);
            indices.push_back(v2);

            indices.push_back(v0);
            indices.push_back(v2);
            indices.push_back(v3);
        }

        m_cachedVertices = std::move(vertices);
        m_cachedIndices = std::move(indices);
        m_cachedEdges = TessellationUtils::ExtractFeatureEdges(m_cachedVertices, m_cachedIndices, 30.0f);
        ComputeFaces();
        ClearDirty();
    }

private:
    std::shared_ptr<DAGNode> m_targetNode{nullptr};
    uint64_t m_targetNodeId{0};
    int m_edgeIndex{0};
    float m_radius{0.2f};
    Geom::Point3D m_edgeP0{0, 0, 0};
    Geom::Point3D m_edgeP1{0, 0, 0};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

} // namespace Tessera
