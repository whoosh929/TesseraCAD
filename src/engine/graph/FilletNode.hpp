#pragma once

#include "Node.hpp"
#include "ExtrudeNode.hpp"
#include "Tessellation.hpp"
#include "EdgeBlend.hpp"
#include "engine/sketch/SketchFilletChamfer.hpp"
#include <memory>
#include <string>
#include <vector>
#include <utility>
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
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    ~FilletNode() override {
        if (m_targetNode) {
            m_targetNode->SetConsumed(false);
        }
    }

    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_targetNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetSolid() const noexcept override { return m_targetNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override { return m_targetNode; }
    [[nodiscard]] bool HasParentFeature() const noexcept override { return m_targetNode != nullptr; }
    [[nodiscard]] int GetReferencedEdgeIndex() const noexcept { return m_edgeIndex; }
    [[nodiscard]] float GetRadius() const noexcept { return m_radius; }
    [[nodiscard]] Geom::Point3D GetEdgeP0() const noexcept { return m_edgeP0; }
    [[nodiscard]] Geom::Point3D GetEdgeP1() const noexcept { return m_edgeP1; }

    enum class DimensionChoice { Distance, Angle };
    [[nodiscard]] DimensionChoice GetDimensionChoice() const noexcept { return m_choice; }
    [[nodiscard]] float GetAngle() const noexcept { return m_angle; }

    void SetDimensionChoice(DimensionChoice choice) {
        if (m_choice != choice) {
            m_choice = choice;
            UpdateRadiusFromChoice();
            MarkDirty();
        }
    }

    void SetAngle(float angleDeg) {
        float a = std::clamp(angleDeg, 5.0f, 175.0f);
        if (std::abs(m_angle - a) > 1e-3f) {
            m_angle = a;
            if (m_choice == DimensionChoice::Angle) {
                UpdateRadiusFromChoice();
            }
            MarkDirty();
        }
    }

    void UpdateRadiusFromChoice() {
        if (m_choice == DimensionChoice::Angle) {
            float alpha = m_angle * 3.14159265f / 180.0f;
            m_radius = std::max(0.01f, m_nominalSetback * std::tan(alpha * 0.5f));
        }
    }

    void SetRadius(float radius) {
        float r = std::max(0.01f, radius);
        if (std::abs(m_radius - r) > 1e-4f) {
            m_radius = r;
            m_nominalSetback = r;
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
        m_extraEdges.clear(); // re-linked to a single edge
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

        // Method 1: If target or active leaf is an Extrusion, apply 2D fillet to profile and extrude
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

                double tol = std::max(0.35, static_cast<double>(m_radius) + 0.15);
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
                auto filletedProfile = data.profile;
                bool ok = Sketch::ApplyFillet2D(filletedProfile, cornerIdx, m_radius, true, 6);
                if (ok) {
                    ExtrudeNode tempNode(m_name + "_Eval", filletedProfile, h);
                    tempNode.SetPosition(data.position[0], data.position[1], data.position[2]);
                    tempNode.SetPlane(data.plane);
                    if (data.useCustomPlane) {
                        tempNode.SetCustomPlane(data.customPlane);
                    }
                    tempNode.Evaluate(device, exchange);

                    m_cachedVertices = tempNode.GetCachedVertices();
                    m_cachedIndices = tempNode.GetCachedIndices();
                    m_cachedEdges = tempNode.GetCachedEdges();
                    m_profile = std::move(filletedProfile);
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

                    ApplyAdditionalEdges();
                    ComputeFaces();
                    ClearDirty();
                    return;
                }
            }
        }

        // Method 2: General 3D edge round (tangent chaining, mitered junctions, trimmed end caps)
        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;
        if (!BuildEdgeBlend(baseVerts, baseIndices, baseEdges, m_edgeIndex, m_edgeP0, m_edgeP1,
                            m_radius, EdgeBlendKind::Round, vertices, indices)) {
            m_cachedVertices = baseVerts;
            m_cachedIndices = baseIndices;
            m_cachedEdges = baseEdges;
            ApplyAdditionalEdges();
            ComputeFaces();
            ClearDirty();
            return;
        }

        m_cachedVertices = std::move(vertices);
        m_cachedIndices = std::move(indices);
        m_cachedEdges = TessellationUtils::ExtractFeatureEdges(m_cachedVertices, m_cachedIndices, 30.0f);
        ApplyAdditionalEdges();
        ComputeFaces();
        ClearDirty();
    }

    /**
     * @brief Extra edges rounded by this same feature, e.g. the other edges meeting at a filleted
     * vertex. Stored by their endpoints on the target model, since edge indices change once the
     * primary edge is rounded.
     */
    void SetAdditionalEdges(std::vector<std::pair<Geom::Point3D, Geom::Point3D>> edges) {
        m_extraEdges = std::move(edges);
        MarkDirty();
    }
    [[nodiscard]] const std::vector<std::pair<Geom::Point3D, Geom::Point3D>>& GetAdditionalEdges() const noexcept {
        return m_extraEdges;
    }

private:
    /// Index of the feature edge lying on the original edge a->b (it may have been shortened by an
    /// earlier blend), or -1 when that edge no longer exists as a sharp edge.
    static int FindEdgeOnSegment(const std::vector<CADVertex>& verts, const std::vector<uint32_t>& edges,
                                 const Geom::Point3D& a, const Geom::Point3D& b) {
        Geom::Vector3D axis = b - a;
        double len = axis.length();
        if (len < 1e-6) return -1;
        axis = axis / len;
        int best = -1;
        double bestLen = 1e-4;
        for (size_t e = 0; e + 1 < edges.size(); e += 2) {
            Geom::Point3D p(verts[edges[e]].position[0], verts[edges[e]].position[1], verts[edges[e]].position[2]);
            Geom::Point3D q(verts[edges[e + 1]].position[0], verts[edges[e + 1]].position[1], verts[edges[e + 1]].position[2]);
            bool onLine = true;
            for (const auto& x : {p, q}) {
                Geom::Vector3D d = x - a;
                double t = d.dot(axis);
                if (t < -1e-3 || t > len + 1e-3 || (d - axis * t).length() > 1e-3) onLine = false;
            }
            double el = p.distance_to(q);
            if (onLine && el > bestLen) {
                bestLen = el;
                best = static_cast<int>(e / 2);
            }
        }
        return best;
    }

    /// Rounds the additional edges on top of the primary result. Edges already rounded by tangent
    /// chaining (e.g. both top edges of a corner wrap the vertical round together) are skipped.
    void ApplyAdditionalEdges() {
        if (m_extraEdges.empty()) return;
        bool applied = false;
        for (const auto& [a, b] : m_extraEdges) {
            auto edges = TessellationUtils::ExtractFeatureEdges(m_cachedVertices, m_cachedIndices, 30.0f);
            int idx = FindEdgeOnSegment(m_cachedVertices, edges, a, b);
            if (idx < 0) continue;
            const auto& v0 = m_cachedVertices[edges[idx * 2]].position;
            const auto& v1 = m_cachedVertices[edges[idx * 2 + 1]].position;
            Geom::Point3D p0(v0[0], v0[1], v0[2]);
            Geom::Point3D p1(v1[0], v1[1], v1[2]);
            std::vector<CADVertex> vertices;
            std::vector<uint32_t> indices;
            if (BuildEdgeBlend(m_cachedVertices, m_cachedIndices, edges, idx, p0, p1,
                               m_radius, EdgeBlendKind::Round, vertices, indices)) {
                m_cachedVertices = std::move(vertices);
                m_cachedIndices = std::move(indices);
                applied = true;
            }
        }
        m_cachedEdges = TessellationUtils::ExtractFeatureEdges(m_cachedVertices, m_cachedIndices, 30.0f);
        // The mesh is no longer a plain extrusion of m_profile; downstream features must not rebuild
        // from the profile (that would silently drop these extra rounds).
        if (applied) m_profile.clear();
    }

    std::vector<std::pair<Geom::Point3D, Geom::Point3D>> m_extraEdges;
    std::shared_ptr<DAGNode> m_targetNode{nullptr};
    uint64_t m_targetNodeId{0};
    int m_edgeIndex{0};
    float m_radius{0.2f};
    DimensionChoice m_choice{DimensionChoice::Distance};
    float m_angle{45.0f};
    float m_nominalSetback{0.25f};
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

/// The distinct feature edges meeting at a model vertex, for corner (vertex) fillets.
struct VertexCornerEdges {
    int primaryEdge{-1}; ///< Edge index on the model, the one most parallel to preferredDir
    std::vector<std::pair<Geom::Point3D, Geom::Point3D>> otherEdges; ///< Remaining edges, by endpoints
};

/**
 * @brief Collects every distinct feature edge touching vertexPos. Meshes list the same geometric edge
 * once per face-vertex copy, so duplicates are merged. The primary edge is the one most parallel to
 * preferredDir (the extrusion direction, so it can use the exact profile-based fillet).
 */
inline VertexCornerEdges CollectVertexCornerEdges(const std::vector<CADVertex>& verts,
                                                  const std::vector<uint32_t>& edges,
                                                  const Geom::Point3D& vertexPos,
                                                  const Geom::Vector3D& preferredDir) {
    struct Cand { int index; Geom::Point3D p0, p1; double score; };
    std::vector<Cand> cands;
    for (size_t e = 0; e + 1 < edges.size(); e += 2) {
        Geom::Point3D p0(verts[edges[e]].position[0], verts[edges[e]].position[1], verts[edges[e]].position[2]);
        Geom::Point3D p1(verts[edges[e + 1]].position[0], verts[edges[e + 1]].position[1], verts[edges[e + 1]].position[2]);
        if (p0.distance_to(vertexPos) >= 0.05 && p1.distance_to(vertexPos) >= 0.05) continue;
        double len = p0.distance_to(p1);
        if (len < 1e-4) continue;
        bool dup = false;
        for (const auto& c : cands) {
            if ((c.p0.distance_to(p0) < 1e-4 && c.p1.distance_to(p1) < 1e-4) ||
                (c.p0.distance_to(p1) < 1e-4 && c.p1.distance_to(p0) < 1e-4)) {
                dup = true;
                break;
            }
        }
        if (dup) continue;
        cands.push_back({static_cast<int>(e / 2), p0, p1, std::abs(((p1 - p0) / len).dot(preferredDir))});
    }
    VertexCornerEdges out;
    if (cands.empty()) return out;
    size_t best = 0;
    for (size_t i = 1; i < cands.size(); ++i) {
        if (cands[i].score > cands[best].score) best = i;
    }
    out.primaryEdge = cands[best].index;
    for (size_t i = 0; i < cands.size(); ++i) {
        if (i != best) out.otherEdges.emplace_back(cands[i].p0, cands[i].p1);
    }
    return out;
}

} // namespace Tessera
