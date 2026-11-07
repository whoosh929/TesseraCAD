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

class ChamferNode : public DAGNode {
public:
    ChamferNode(
        std::string name,
        std::shared_ptr<DAGNode> targetNode,
        int edgeIndex,
        float distance = 0.2f
    ) : DAGNode(std::move(name)),
        m_targetNode(targetNode),
        m_targetNodeId(targetNode ? targetNode->GetId() : 0),
        m_edgeIndex(edgeIndex),
        m_distance(std::max(0.01f, distance)) {
        if (m_targetNode) {
            m_targetNode->SetConsumed(true);
        }
    }

    ~ChamferNode() override {
        if (m_targetNode) {
            m_targetNode->SetConsumed(false);
        }
    }

    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_targetNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override { return m_targetNode; }
    [[nodiscard]] bool HasParentFeature() const noexcept override { return m_targetNode != nullptr; }
    [[nodiscard]] int GetReferencedEdgeIndex() const noexcept { return m_edgeIndex; }
    [[nodiscard]] float GetDistance() const noexcept { return m_distance; }
    [[nodiscard]] Geom::Point3D GetEdgeP0() const noexcept { return m_edgeP0; }
    [[nodiscard]] Geom::Point3D GetEdgeP1() const noexcept { return m_edgeP1; }

    void SetDistance(float distance) {
        float d = std::max(0.01f, distance);
        if (std::abs(m_distance - d) > 1e-4f) {
            m_distance = d;
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

    void NotifyRemoval() override {
        if (m_targetNode) {
            m_targetNode->SetConsumed(false);
            m_targetNode->MarkDirty();
        }
        DAGNode::NotifyRemoval();
    }

    void OnParentRemoved(uint64_t parentId) override {
        if (m_targetNodeId == parentId) {
            m_targetNode = nullptr;
            m_hasConflict = true;
            m_conflictMessage = "Referenced parent model feature was deleted.";
            MarkDirty();
        }
        DAGNode::OnParentRemoved(parentId);
    }

    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept { return m_profile; }
    [[nodiscard]] float GetHeight() const noexcept { return m_height; }
    [[nodiscard]] SketchPlane GetPlane() const noexcept { return m_plane; }
    [[nodiscard]] bool UsesCustomPlane() const noexcept { return m_useCustomPlane; }
    [[nodiscard]] Geom::Plane GetCustomPlane() const noexcept { return m_customPlane; }

    bool IsExtrusionBased(ExtrusionData& out) const override {
        out.profile = m_profile;
        out.height = m_height;
        out.position = {m_position[0], m_position[1], m_position[2]};
        out.plane = m_plane;
        out.useCustomPlane = m_useCustomPlane;
        out.customPlane = m_customPlane;
        return !out.profile.empty();
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

        // Method 1: If target or active leaf is an Extrusion, apply 2D chamfer to profile and extrude
        ExtrusionData data;
        if (ExtractExtrusionData(m_targetNode, data) && data.profile.size() >= 3) {
            float h = data.height;
            bool foundCorner = false;
            size_t cornerIdx = 0;

            for (size_t p = 0; p < data.profile.size(); ++p) {
                float u = data.profile[p][0];
                float v = data.profile[p][1];

                Geom::Point3D pBot = MapExtrusionPointToWorld(data, u, v, 0.0f);
                Geom::Point3D pTop = MapExtrusionPointToWorld(data, u, v, h);

                double dBot0 = m_edgeP0.distance_to(pBot);
                double dTop1 = m_edgeP1.distance_to(pTop);
                double dTop0 = m_edgeP0.distance_to(pTop);
                double dBot1 = m_edgeP1.distance_to(pBot);

                double tol = std::max(0.35, static_cast<double>(m_distance) + 0.15);
                if ((dBot0 < tol && dTop1 < tol) || (dTop0 < tol && dBot1 < tol)) {
                    cornerIdx = p;
                    foundCorner = true;
                    break;
                }

                // Check distance of edge midpoint to corner line segment
                Geom::Point3D mid(
                    (m_edgeP0.x + m_edgeP1.x) * 0.5,
                    (m_edgeP0.y + m_edgeP1.y) * 0.5,
                    (m_edgeP0.z + m_edgeP1.z) * 0.5
                );
                Geom::Vector3D axis = pTop - pBot;
                double lenSq = axis.length_squared();
                if (lenSq > 1e-4) {
                    double t = (mid - pBot).dot(axis) / lenSq;
                    if (t >= -0.15 && t <= 1.15) {
                        Geom::Point3D proj = pBot + axis * t;
                        if (mid.distance_to(proj) < tol) {
                            cornerIdx = p;
                            foundCorner = true;
                            break;
                        }
                    }
                }
            }

            if (foundCorner) {
                auto chamferedProfile = data.profile;
                bool ok = Sketch::ApplyChamfer2D(chamferedProfile, cornerIdx, m_distance, true);
                if (ok) {
                    ExtrudeNode tempNode(m_name + "_Eval", chamferedProfile, h);
                    tempNode.SetPosition(data.position[0], data.position[1], data.position[2]);
                    tempNode.SetPlane(data.plane);
                    if (data.useCustomPlane) {
                        tempNode.SetCustomPlane(data.customPlane);
                    }
                    tempNode.Evaluate(device, exchange);

                    m_cachedVertices = tempNode.GetCachedVertices();
                    m_cachedIndices = tempNode.GetCachedIndices();
                    m_cachedEdges = tempNode.GetCachedEdges();
                    m_profile = std::move(chamferedProfile);
                    m_height = h;
                    m_position[0] = data.position[0];
                    m_position[1] = data.position[1];
                    m_position[2] = data.position[2];
                    m_plane = data.plane;
                    m_useCustomPlane = data.useCustomPlane;
                    m_customPlane = data.customPlane;

                    // Ensure target node and its leaf are marked consumed
                    m_targetNode->SetConsumed(true);
                    auto leaf = m_targetNode->GetActiveLeafFeature();
                    if (leaf && leaf.get() != this) {
                        leaf->SetConsumed(true);
                    }

                    ComputeFaces();
                    ClearDirty();
                    return;
                }
            }
        }

        // Method 2: General 3D Edge Bevel Generator (with clean manifold boundary updates)
        std::vector<CADVertex> vertices = baseVerts;
        std::vector<uint32_t> indices = baseIndices;

        double edgeLen = m_edgeP0.distance_to(m_edgeP1);
        if (edgeLen < 1e-4) {
            m_cachedVertices = baseVerts;
            m_cachedIndices = baseIndices;
            m_cachedEdges = baseEdges;
            ComputeFaces();
            ClearDirty();
            return;
        }

        Geom::Vector3D edgeVec = (m_edgeP1 - m_edgeP0) / edgeLen;

        // 1. Find all triangles incident to the edge (by geometric proximity <= 0.05)
        // and identify the two distinct face normals normalA and normalB meeting at the edge.
        Geom::Vector3D normalA(0, 0, 0);
        Geom::Vector3D normalB(0, 0, 0);
        Geom::Point3D thirdPtA, thirdPtB;
        bool hasNormalA = false;
        bool hasNormalB = false;

        const size_t numTriangles = baseIndices.size() / 3;
        for (size_t t = 0; t < numTriangles; ++t) {
            uint32_t a = baseIndices[t * 3 + 0];
            uint32_t b = baseIndices[t * 3 + 1];
            uint32_t c = baseIndices[t * 3 + 2];

            Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
            Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
            Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);

            bool hasP0 = (pa.distance_to(m_edgeP0) < 0.05 || pb.distance_to(m_edgeP0) < 0.05 || pc.distance_to(m_edgeP0) < 0.05);
            bool hasP1 = (pa.distance_to(m_edgeP1) < 0.05 || pb.distance_to(m_edgeP1) < 0.05 || pc.distance_to(m_edgeP1) < 0.05);

            if (hasP0 && hasP1) {
                Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();
                if (triNorm.length() < 0.5) continue;

                // Find the third vertex not on the edge
                Geom::Point3D pThird = pc;
                if (pa.distance_to(m_edgeP0) >= 0.05 && pa.distance_to(m_edgeP1) >= 0.05) pThird = pa;
                else if (pb.distance_to(m_edgeP0) >= 0.05 && pb.distance_to(m_edgeP1) >= 0.05) pThird = pb;

                if (!hasNormalA) {
                    normalA = triNorm;
                    thirdPtA = pThird;
                    hasNormalA = true;
                } else if (!hasNormalB && normalA.dot(triNorm) < 0.95) {
                    normalB = triNorm;
                    thirdPtB = pThird;
                    hasNormalB = true;
                }
            }
        }

        if (!hasNormalA || !hasNormalB) {
            // Cannot form two-face bevel (e.g. boundary or coplanar edge); retain base geometry safely
            m_cachedVertices = baseVerts;
            m_cachedIndices = baseIndices;
            m_cachedEdges = baseEdges;
            ComputeFaces();
            ClearDirty();
            return;
        }

        // 2. Compute in-plane directions uA and uB pointing into Face A and Face B away from the edge
        Geom::Vector3D vIntoA = (thirdPtA - m_edgeP0) - edgeVec * ((thirdPtA - m_edgeP0).dot(edgeVec));
        Geom::Vector3D uA = (vIntoA.length_squared() > 1e-6) ? vIntoA.normalized() : edgeVec.cross(normalA).normalized();

        Geom::Vector3D vIntoB = (thirdPtB - m_edgeP0) - edgeVec * ((thirdPtB - m_edgeP0).dot(edgeVec));
        Geom::Vector3D uB = (vIntoB.length_squared() > 1e-6) ? vIntoB.normalized() : normalB.cross(edgeVec).normalized();

        uA = (uA - normalA * uA.dot(normalA) - edgeVec * uA.dot(edgeVec)).normalized();
        uB = (uB - normalB * uB.dot(normalB) - edgeVec * uB.dot(edgeVec)).normalized();

        // Prevent self-intersection by limiting setback to at most 45% of edge length or face extent
        float maxSetback = static_cast<float>(edgeLen) * 0.45f;
        if (vIntoA.length() > 1e-4) maxSetback = std::min(maxSetback, static_cast<float>(vIntoA.length()) * 0.45f);
        if (vIntoB.length() > 1e-4) maxSetback = std::min(maxSetback, static_cast<float>(vIntoB.length()) * 0.45f);
        float dEff = std::clamp(m_distance, 0.005f, std::max(0.005f, maxSetback));

        // 3. Setback points on Face A and Face B
        Geom::Point3D s0A = m_edgeP0 + uA * dEff;
        Geom::Point3D s0B = m_edgeP0 + uB * dEff;
        Geom::Point3D s1A = m_edgeP1 + uA * dEff;
        Geom::Point3D s1B = m_edgeP1 + uB * dEff;

        // Create setback anchor vertices on Face A and Face B
        uint32_t v_s0A = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(s0A.x), static_cast<float>(s0A.y), static_cast<float>(s0A.z)},
            {static_cast<float>(normalA.x), static_cast<float>(normalA.y), static_cast<float>(normalA.z)}
        });

        uint32_t v_s1A = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(s1A.x), static_cast<float>(s1A.y), static_cast<float>(s1A.z)},
            {static_cast<float>(normalA.x), static_cast<float>(normalA.y), static_cast<float>(normalA.z)}
        });

        uint32_t v_s0B = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(s0B.x), static_cast<float>(s0B.y), static_cast<float>(s0B.z)},
            {static_cast<float>(normalB.x), static_cast<float>(normalB.y), static_cast<float>(normalB.z)}
        });

        uint32_t v_s1B = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(s1B.x), static_cast<float>(s1B.y), static_cast<float>(s1B.z)},
            {static_cast<float>(normalB.x), static_cast<float>(normalB.y), static_cast<float>(normalB.z)}
        });

        // 4. Update incident triangles belonging to Face A or Face B by cleanly sliding edge vertices to setback anchors
        for (size_t t = 0; t < numTriangles; ++t) {
            uint32_t a = baseIndices[t * 3 + 0];
            uint32_t b = baseIndices[t * 3 + 1];
            uint32_t c = baseIndices[t * 3 + 2];

            Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
            Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
            Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);

            Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();
            if (triNorm.length() < 0.5) continue;

            double dotA = triNorm.dot(normalA);
            double dotB = triNorm.dot(normalB);

            if (dotA > 0.85) {
                for (int k = 0; k < 3; ++k) {
                    uint32_t origIdx = baseIndices[t * 3 + k];
                    Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                    if (p.distance_to(m_edgeP0) < 0.05) {
                        indices[t * 3 + k] = v_s0A;
                    } else if (p.distance_to(m_edgeP1) < 0.05) {
                        indices[t * 3 + k] = v_s1A;
                    }
                }
            } else if (dotB > 0.85) {
                for (int k = 0; k < 3; ++k) {
                    uint32_t origIdx = baseIndices[t * 3 + k];
                    Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                    if (p.distance_to(m_edgeP0) < 0.05) {
                        indices[t * 3 + k] = v_s0B;
                    } else if (p.distance_to(m_edgeP1) < 0.05) {
                        indices[t * 3 + k] = v_s1B;
                    }
                }
            }
        }

        // 5. Connect chamfer bevel quad between Face A and Face B
        Geom::Point3D pa(vertices[v_s0A].position[0], vertices[v_s0A].position[1], vertices[v_s0A].position[2]);
        Geom::Point3D pb(vertices[v_s1A].position[0], vertices[v_s1A].position[1], vertices[v_s1A].position[2]);
        Geom::Point3D pc(vertices[v_s0B].position[0], vertices[v_s0B].position[1], vertices[v_s0B].position[2]);
        Geom::Vector3D n = (pb - pa).cross(pc - pa);
        Geom::Vector3D expectedNorm = (normalA + normalB).normalized();

        if (n.dot(expectedNorm) >= 0.0) {
            indices.push_back(v_s0A); indices.push_back(v_s1A); indices.push_back(v_s1B);
            indices.push_back(v_s0A); indices.push_back(v_s1B); indices.push_back(v_s0B);
        } else {
            indices.push_back(v_s0A); indices.push_back(v_s1B); indices.push_back(v_s1A);
            indices.push_back(v_s0A); indices.push_back(v_s0B); indices.push_back(v_s1B);
        }

        // 6. Add end caps at p0 and p1 only if actual capping faces meet at the ends of the edge
        bool hasEnd0Face = false;
        bool hasEnd1Face = false;
        for (size_t t = 0; t < numTriangles; ++t) {
            uint32_t a = baseIndices[t * 3 + 0];
            uint32_t b = baseIndices[t * 3 + 1];
            uint32_t c = baseIndices[t * 3 + 2];
            Geom::Point3D tpa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
            Geom::Point3D tpb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
            Geom::Point3D tpc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
            Geom::Vector3D triNorm = (tpb - tpa).cross(tpc - tpa).normalized();
            if (triNorm.dot(normalA) <= 0.85 && triNorm.dot(normalB) <= 0.85) {
                if (triNorm.dot(-edgeVec) > 0.5) {
                    if (tpa.distance_to(m_edgeP0) < 0.05 || tpb.distance_to(m_edgeP0) < 0.05 || tpc.distance_to(m_edgeP0) < 0.05) {
                        hasEnd0Face = true;
                    }
                }
                if (triNorm.dot(edgeVec) > 0.5) {
                    if (tpa.distance_to(m_edgeP1) < 0.05 || tpb.distance_to(m_edgeP1) < 0.05 || tpc.distance_to(m_edgeP1) < 0.05) {
                        hasEnd1Face = true;
                    }
                }
            }
        }

        if (hasEnd0Face) {
            for (size_t t = 0; t < numTriangles; ++t) {
                uint32_t a = baseIndices[t * 3 + 0];
                uint32_t b = baseIndices[t * 3 + 1];
                uint32_t c = baseIndices[t * 3 + 2];
                Geom::Point3D tpa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                Geom::Point3D tpb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                Geom::Point3D tpc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                Geom::Vector3D triNorm = (tpb - tpa).cross(tpc - tpa).normalized();
                if (triNorm.dot(-edgeVec) > 0.4) {
                    for (int k = 0; k < 3; ++k) {
                        uint32_t origIdx = baseIndices[t * 3 + k];
                        Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                        if (p.distance_to(m_edgeP0) < 0.05) {
                            indices[t * 3 + k] = v_s0A;
                        }
                    }
                }
            }
        }

        if (hasEnd1Face) {
            for (size_t t = 0; t < numTriangles; ++t) {
                uint32_t a = baseIndices[t * 3 + 0];
                uint32_t b = baseIndices[t * 3 + 1];
                uint32_t c = baseIndices[t * 3 + 2];
                Geom::Point3D tpa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                Geom::Point3D tpb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                Geom::Point3D tpc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                Geom::Vector3D triNorm = (tpb - tpa).cross(tpc - tpa).normalized();
                if (triNorm.dot(edgeVec) > 0.4) {
                    for (int k = 0; k < 3; ++k) {
                        uint32_t origIdx = baseIndices[t * 3 + k];
                        Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                        if (p.distance_to(m_edgeP1) < 0.05) {
                            indices[t * 3 + k] = v_s1A;
                        }
                    }
                }
            }
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
    float m_distance{0.2f};
    Geom::Point3D m_edgeP0{0, 0, 0};
    Geom::Point3D m_edgeP1{0, 0, 0};
    bool m_hasConflict{false};
    std::string m_conflictMessage;

    // Extrusion profile propagation metadata for chained 3D solid operations
    std::vector<std::array<float, 2>> m_profile;
    float m_height{2.0f};
    SketchPlane m_plane{SketchPlane::XZ_Ground};
    bool m_useCustomPlane{false};
    Geom::Plane m_customPlane{Geom::Point3D(0, 0, 0), Geom::Vector3D(0, 1, 0)};
};

} // namespace Tessera
