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

                    ComputeFaces();
                    ClearDirty();
                    return;
                }
            }
        }

        // Method 2: General 3D Edge Bevel/Round Generator (with Tangent Chaining & clean manifold boundary updates)
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

        // 1. Find all triangles incident to the seed edge and identify the two distinct face normals normalA and normalB
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
            m_cachedVertices = baseVerts;
            m_cachedIndices = baseIndices;
            m_cachedEdges = baseEdges;
            ComputeFaces();
            ClearDirty();
            return;
        }

        // Check if normalB has more incident triangles than normalA; if so, swap so Face A is the dominant shared face
        int countA = 0, countB = 0;
        for (size_t t = 0; t < numTriangles; ++t) {
            uint32_t a = baseIndices[t * 3 + 0];
            uint32_t b = baseIndices[t * 3 + 1];
            uint32_t c = baseIndices[t * 3 + 2];
            Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
            Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
            Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
            Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
            if (tn.dot(normalA) > 0.85) countA++;
            if (tn.dot(normalB) > 0.85) countB++;
        }
        if (countB > countA) {
            std::swap(normalA, normalB);
            std::swap(thirdPtA, thirdPtB);
        }

        struct ChainEdge {
            Geom::Point3D p0;
            Geom::Point3D p1;
            Geom::Vector3D edgeVec;
            Geom::Vector3D normalB;
            Geom::Vector3D uA;
            Geom::Vector3D uB;
            float rEff{0.0f};
        };

        Geom::Vector3D vIntoA = (thirdPtA - m_edgeP0) - edgeVec * ((thirdPtA - m_edgeP0).dot(edgeVec));
        Geom::Vector3D uA = (vIntoA.length_squared() > 1e-6) ? vIntoA.normalized() : edgeVec.cross(normalA).normalized();
        Geom::Vector3D vIntoB = (thirdPtB - m_edgeP0) - edgeVec * ((thirdPtB - m_edgeP0).dot(edgeVec));
        Geom::Vector3D uB = (vIntoB.length_squared() > 1e-6) ? vIntoB.normalized() : normalB.cross(edgeVec).normalized();
        uA = (uA - normalA * uA.dot(normalA) - edgeVec * uA.dot(edgeVec)).normalized();
        uB = (uB - normalB * uB.dot(normalB) - edgeVec * uB.dot(edgeVec)).normalized();

        float maxSetback = static_cast<float>(edgeLen) * 0.45f;
        if (vIntoA.length() > 1e-4) maxSetback = std::min(maxSetback, static_cast<float>(vIntoA.length()) * 0.45f);
        if (vIntoB.length() > 1e-4) maxSetback = std::min(maxSetback, static_cast<float>(vIntoB.length()) * 0.45f);
        float seedRadius = std::clamp(m_radius, 0.005f, std::max(0.005f, maxSetback));

        std::vector<ChainEdge> chain;
        chain.push_back({m_edgeP0, m_edgeP1, edgeVec, normalB, uA, uB, seedRadius});

        std::unordered_set<size_t> visitedEdges;
        visitedEdges.insert(static_cast<size_t>(m_edgeIndex));

        // Forward propagation along tangent feature edges sharing Face A
        bool grewForward = true;
        while (grewForward) {
            grewForward = false;
            Geom::Point3D currTip = chain.back().p1;
            Geom::Vector3D currDir = chain.back().edgeVec;

            if (chain.size() >= 3 && currTip.distance_to(chain.front().p0) < 0.05) {
                break;
            }

            int bestEdgeIdx = -1;
            Geom::Point3D bestP0, bestP1;
            Geom::Vector3D bestDir, bestNormB, bestUA, bestUB;
            double bestScore = 0.35; // tangent-continuous chaining only (no wrapping around sharp corners)

            for (size_t e = 0; e < baseEdges.size() / 2; ++e) {
                if (visitedEdges.count(e)) continue;

                uint32_t edgeI0 = baseEdges[e * 2];
                uint32_t edgeI1 = baseEdges[e * 2 + 1];
                Geom::Point3D ep0(baseVerts[edgeI0].position[0], baseVerts[edgeI0].position[1], baseVerts[edgeI0].position[2]);
                Geom::Point3D ep1(baseVerts[edgeI1].position[0], baseVerts[edgeI1].position[1], baseVerts[edgeI1].position[2]);

                Geom::Point3D candP0, candP1;
                if (ep0.distance_to(currTip) < 0.05) {
                    candP0 = ep0; candP1 = ep1;
                } else if (ep1.distance_to(currTip) < 0.05) {
                    candP0 = ep1; candP1 = ep0;
                } else {
                    continue;
                }

                double cLen = candP0.distance_to(candP1);
                if (cLen < 1e-4) continue;
                Geom::Vector3D candDir = (candP1 - candP0) / cLen;

                double dotDir = currDir.dot(candDir);
                if (dotDir <= bestScore) continue;

                bool candHasFaceA = false;
                Geom::Vector3D candNormB(0, 0, 0);
                Geom::Point3D candThirdA, candThirdB;
                bool hasCandB = false;

                for (size_t t = 0; t < numTriangles; ++t) {
                    uint32_t a = baseIndices[t * 3 + 0];
                    uint32_t b = baseIndices[t * 3 + 1];
                    uint32_t c = baseIndices[t * 3 + 2];
                    Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                    Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                    Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                    bool hasP0 = (pa.distance_to(candP0) < 0.05 || pb.distance_to(candP0) < 0.05 || pc.distance_to(candP0) < 0.05);
                    bool hasP1 = (pa.distance_to(candP1) < 0.05 || pb.distance_to(candP1) < 0.05 || pc.distance_to(candP1) < 0.05);
                    if (hasP0 && hasP1) {
                        Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
                        if (tn.length() < 0.5) continue;
                        Geom::Point3D p3 = pc;
                        if (pa.distance_to(candP0) >= 0.05 && pa.distance_to(candP1) >= 0.05) p3 = pa;
                        else if (pb.distance_to(candP0) >= 0.05 && pb.distance_to(candP1) >= 0.05) p3 = pb;

                        if (tn.dot(normalA) > 0.85) {
                            candHasFaceA = true;
                            candThirdA = p3;
                        } else if (!hasCandB && tn.dot(normalA) < 0.7) {
                            candNormB = tn;
                            candThirdB = p3;
                            hasCandB = true;
                        }
                    }
                }

                if (candHasFaceA && hasCandB) {
                    bestScore = dotDir;
                    bestEdgeIdx = static_cast<int>(e);
                    bestP0 = candP0; bestP1 = candP1; bestDir = candDir;
                    bestNormB = candNormB;

                    Geom::Vector3D vA = (candThirdA - candP0) - candDir * ((candThirdA - candP0).dot(candDir));
                    bestUA = (vA.length_squared() > 1e-6) ? vA.normalized() : candDir.cross(normalA).normalized();
                    Geom::Vector3D vB = (candThirdB - candP0) - candDir * ((candThirdB - candP0).dot(candDir));
                    bestUB = (vB.length_squared() > 1e-6) ? vB.normalized() : candNormB.cross(candDir).normalized();
                    bestUA = (bestUA - normalA * bestUA.dot(normalA) - candDir * bestUA.dot(candDir)).normalized();
                    bestUB = (bestUB - candNormB * bestUB.dot(candNormB) - candDir * bestUB.dot(candDir)).normalized();
                }
            }

            if (bestEdgeIdx >= 0) {
                visitedEdges.insert(static_cast<size_t>(bestEdgeIdx));
                chain.push_back({bestP0, bestP1, bestDir, bestNormB, bestUA, bestUB, seedRadius});
                grewForward = true;
            }
        }

        // Backward propagation along tangent feature edges sharing Face A
        bool grewBackward = true;
        while (grewBackward) {
            grewBackward = false;
            Geom::Point3D currRoot = chain.front().p0;
            Geom::Vector3D currDir = chain.front().edgeVec;

            if (chain.size() >= 3 && currRoot.distance_to(chain.back().p1) < 0.05) {
                break;
            }

            int bestEdgeIdx = -1;
            Geom::Point3D bestP0, bestP1;
            Geom::Vector3D bestDir, bestNormB, bestUA, bestUB;
            double bestScore = 0.35; // tangent-continuous chaining only (no wrapping around sharp corners)

            for (size_t e = 0; e < baseEdges.size() / 2; ++e) {
                if (visitedEdges.count(e)) continue;

                uint32_t edgeI0 = baseEdges[e * 2];
                uint32_t edgeI1 = baseEdges[e * 2 + 1];
                Geom::Point3D ep0(baseVerts[edgeI0].position[0], baseVerts[edgeI0].position[1], baseVerts[edgeI0].position[2]);
                Geom::Point3D ep1(baseVerts[edgeI1].position[0], baseVerts[edgeI1].position[1], baseVerts[edgeI1].position[2]);

                Geom::Point3D candP0, candP1;
                if (ep1.distance_to(currRoot) < 0.05) {
                    candP0 = ep0; candP1 = ep1;
                } else if (ep0.distance_to(currRoot) < 0.05) {
                    candP0 = ep1; candP1 = ep0;
                } else {
                    continue;
                }

                double cLen = candP0.distance_to(candP1);
                if (cLen < 1e-4) continue;
                Geom::Vector3D candDir = (candP1 - candP0) / cLen;

                double dotDir = candDir.dot(currDir);
                if (dotDir <= bestScore) continue;

                bool candHasFaceA = false;
                Geom::Vector3D candNormB(0, 0, 0);
                Geom::Point3D candThirdA, candThirdB;
                bool hasCandB = false;

                for (size_t t = 0; t < numTriangles; ++t) {
                    uint32_t a = baseIndices[t * 3 + 0];
                    uint32_t b = baseIndices[t * 3 + 1];
                    uint32_t c = baseIndices[t * 3 + 2];
                    Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                    Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                    Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                    bool hasP0 = (pa.distance_to(candP0) < 0.05 || pb.distance_to(candP0) < 0.05 || pc.distance_to(candP0) < 0.05);
                    bool hasP1 = (pa.distance_to(candP1) < 0.05 || pb.distance_to(candP1) < 0.05 || pc.distance_to(candP1) < 0.05);
                    if (hasP0 && hasP1) {
                        Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
                        if (tn.length() < 0.5) continue;
                        Geom::Point3D p3 = pc;
                        if (pa.distance_to(candP0) >= 0.05 && pa.distance_to(candP1) >= 0.05) p3 = pa;
                        else if (pb.distance_to(candP0) >= 0.05 && pb.distance_to(candP1) >= 0.05) p3 = pb;

                        if (tn.dot(normalA) > 0.85) {
                            candHasFaceA = true;
                            candThirdA = p3;
                        } else if (!hasCandB && tn.dot(normalA) < 0.7) {
                            candNormB = tn;
                            candThirdB = p3;
                            hasCandB = true;
                        }
                    }
                }

                if (candHasFaceA && hasCandB) {
                    bestScore = dotDir;
                    bestEdgeIdx = static_cast<int>(e);
                    bestP0 = candP0; bestP1 = candP1; bestDir = candDir;
                    bestNormB = candNormB;

                    Geom::Vector3D vA = (candThirdA - candP0) - candDir * ((candThirdA - candP0).dot(candDir));
                    bestUA = (vA.length_squared() > 1e-6) ? vA.normalized() : candDir.cross(normalA).normalized();
                    Geom::Vector3D vB = (candThirdB - candP0) - candDir * ((candThirdB - candP0).dot(candDir));
                    bestUB = (vB.length_squared() > 1e-6) ? vB.normalized() : candNormB.cross(candDir).normalized();
                    bestUA = (bestUA - normalA * bestUA.dot(normalA) - candDir * bestUA.dot(candDir)).normalized();
                    bestUB = (bestUB - candNormB * bestUB.dot(candNormB) - candDir * bestUB.dot(candDir)).normalized();
                }
            }

            if (bestEdgeIdx >= 0) {
                visitedEdges.insert(static_cast<size_t>(bestEdgeIdx));
                chain.insert(chain.begin(), {bestP0, bestP1, bestDir, bestNormB, bestUA, bestUB, seedRadius});
                grewBackward = true;
            }
        }

        const size_t chainCount = chain.size();
        bool isLoop = (chainCount >= 3 && chain.front().p0.distance_to(chain.back().p1) < 0.05);

        // Compute setback distance d = R * tan(alpha/2) for each chain segment
        std::vector<double> setbackD(chainCount);
        for (size_t i = 0; i < chainCount; ++i) {
            double cosAlpha = std::clamp(normalA.dot(chain[i].normalB), -0.999, 0.999);
            double sinAlpha = (normalA.cross(chain[i].normalB)).length();
            double tanHalf = (1.0 + cosAlpha > 1e-5) ? (sinAlpha / (1.0 + cosAlpha)) : 1.0;
            setbackD[i] = seedRadius * tanHalf;
        }

        // Compute continuous mitered setback points on Face A for every junction in the chain
        std::vector<Geom::Point3D> sA_pts(chainCount + 1);
        std::vector<uint32_t> v_sA(chainCount + 1);

        // Exact mitered offset point at a junction: the point p on face A with p.u1 = d1 and p.u2 = d2,
        // i.e. the intersection of the two setback lines. Handles sharp corner turns (e.g. 90 degrees).
        auto miterPoint = [](const Geom::Point3D& J, const Geom::Vector3D& u1, double d1,
                             const Geom::Vector3D& u2, double d2) -> Geom::Point3D {
            double c = u1.dot(u2);
            double den = 1.0 - c * c;
            if (den < 1e-3) {
                return J + (u1 + u2).normalized() * ((d1 + d2) * 0.5);
            }
            double a = (d1 - d2 * c) / den;
            double b = (d2 - d1 * c) / den;
            return J + u1 * a + u2 * b;
        };

        for (size_t i = 0; i <= chainCount; ++i) {
            Geom::Point3D junctionPt;
            if (i == 0) {
                junctionPt = chain.front().p0;
                sA_pts[i] = isLoop
                    ? miterPoint(junctionPt, chain.back().uA, setbackD.back(), chain.front().uA, setbackD.front())
                    : junctionPt + chain.front().uA * setbackD.front();
            } else if (i == chainCount) {
                junctionPt = chain.back().p1;
                sA_pts[i] = isLoop
                    ? miterPoint(junctionPt, chain.back().uA, setbackD.back(), chain.front().uA, setbackD.front())
                    : junctionPt + chain.back().uA * setbackD.back();
            } else {
                junctionPt = chain[i].p0;
                sA_pts[i] = miterPoint(junctionPt, chain[i - 1].uA, setbackD[i - 1], chain[i].uA, setbackD[i]);
            }

            v_sA[i] = static_cast<uint32_t>(vertices.size());
            vertices.push_back({
                {static_cast<float>(sA_pts[i].x), static_cast<float>(sA_pts[i].y), static_cast<float>(sA_pts[i].z)},
                {static_cast<float>(normalA.x), static_cast<float>(normalA.y), static_cast<float>(normalA.z)}
            });
        }

        // Slide a cross-section point of segment i along its edge direction so it lies on the miter
        // (bisector) plane of the junction at the segment's start or end. Makes adjacent filleted edges
        // meet cleanly at a corner instead of overlapping or leaving gaps.
        auto toMiterPlane = [&](const Geom::Point3D& pt, size_t i, bool atEnd) -> Geom::Point3D {
            const Geom::Vector3D* prevDir = nullptr;
            const Geom::Vector3D* nextDir = nullptr;
            Geom::Point3D J;
            if (atEnd) {
                J = chain[i].p1;
                if (i + 1 < chainCount) nextDir = &chain[i + 1].edgeVec;
                else if (isLoop) nextDir = &chain.front().edgeVec;
                if (!nextDir) return pt;
                prevDir = &chain[i].edgeVec;
            } else {
                J = chain[i].p0;
                if (i > 0) prevDir = &chain[i - 1].edgeVec;
                else if (isLoop) prevDir = &chain.back().edgeVec;
                if (!prevDir) return pt;
                nextDir = &chain[i].edgeVec;
            }
            Geom::Vector3D sum = *prevDir + *nextDir;
            if (sum.length() < 1e-6) return pt;
            Geom::Vector3D nm = sum.normalized();
            const Geom::Vector3D& e = chain[i].edgeVec;
            double denom = e.dot(nm);
            if (std::abs(denom) < 0.2) return pt;
            double s = -((pt - J).dot(nm)) / denom;
            return pt + e * s;
        };

        // Compute setback points on side Face B for each segment
        std::vector<uint32_t> v_sB0(chainCount);
        std::vector<uint32_t> v_sB1(chainCount);
        std::vector<Geom::Point3D> sB0_pts(chainCount);
        std::vector<Geom::Point3D> sB1_pts(chainCount);

        for (size_t i = 0; i < chainCount; ++i) {
            sB0_pts[i] = toMiterPlane(chain[i].p0 + chain[i].uB * setbackD[i], i, false);
            sB1_pts[i] = toMiterPlane(chain[i].p1 + chain[i].uB * setbackD[i], i, true);

            v_sB0[i] = static_cast<uint32_t>(vertices.size());
            vertices.push_back({
                {static_cast<float>(sB0_pts[i].x), static_cast<float>(sB0_pts[i].y), static_cast<float>(sB0_pts[i].z)},
                {static_cast<float>(chain[i].normalB.x), static_cast<float>(chain[i].normalB.y), static_cast<float>(chain[i].normalB.z)}
            });

            v_sB1[i] = static_cast<uint32_t>(vertices.size());
            vertices.push_back({
                {static_cast<float>(sB1_pts[i].x), static_cast<float>(sB1_pts[i].y), static_cast<float>(sB1_pts[i].z)},
                {static_cast<float>(chain[i].normalB.x), static_cast<float>(chain[i].normalB.y), static_cast<float>(chain[i].normalB.z)}
            });
        }

        // Update incident triangles belonging to Face A or Face B by cleanly sliding vertices to setback anchors
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

            if (dotA > 0.85) {
                for (int k = 0; k < 3; ++k) {
                    uint32_t origIdx = baseIndices[t * 3 + k];
                    Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                    for (size_t j = 0; j <= chainCount; ++j) {
                        Geom::Point3D jPt = (j < chainCount) ? chain[j].p0 : chain.back().p1;
                        if (p.distance_to(jPt) < 0.05) {
                            indices[t * 3 + k] = v_sA[j];
                            break;
                        }
                    }
                }
            } else {
                for (size_t i = 0; i < chainCount; ++i) {
                    if (triNorm.dot(chain[i].normalB) > 0.85) {
                        for (int k = 0; k < 3; ++k) {
                            uint32_t origIdx = baseIndices[t * 3 + k];
                            Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                            if (p.distance_to(chain[i].p0) < 0.05) {
                                indices[t * 3 + k] = v_sB0[i];
                            } else if (p.distance_to(chain[i].p1) < 0.05) {
                                indices[t * 3 + k] = v_sB1[i];
                            }
                        }
                        break;
                    }
                }
            }
        }

        // Generate smooth fillet surface bands connecting Face A to Face B for each segment in the chain
        const int numBands = 6;
        std::vector<std::vector<uint32_t>> allBand0(chainCount);
        std::vector<std::vector<uint32_t>> allBand1(chainCount);

        for (size_t i = 0; i < chainCount; ++i) {
            std::vector<uint32_t> band0(numBands + 1);
            std::vector<uint32_t> band1(numBands + 1);

            band0[0] = v_sA[i];
            band1[0] = v_sA[i + 1];
            band0[numBands] = v_sB0[i];
            band1[numBands] = v_sB1[i];

            // Convexity check: uA points into Face A, normalB points outward from Face B
            bool isConvex = (chain[i].uA.dot(chain[i].normalB) < 0.0);

            // Rolling-ball center inside the solid for convex edge, outside for concave edge
            Geom::Point3D center0 = isConvex
                ? (chain[i].p0 + chain[i].uA * setbackD[i] - normalA * seedRadius)
                : (chain[i].p0 + chain[i].uA * setbackD[i] + normalA * seedRadius);
            Geom::Point3D center1 = isConvex
                ? (chain[i].p1 + chain[i].uA * setbackD[i] - normalA * seedRadius)
                : (chain[i].p1 + chain[i].uA * setbackD[i] + normalA * seedRadius);

            for (int b = 1; b < numBands; ++b) {
                float t = static_cast<float>(b) / static_cast<float>(numBands);
                Geom::Vector3D filletNorm = (normalA * (1.0f - t) + chain[i].normalB * t).normalized();

                Geom::Point3D pt0 = toMiterPlane(isConvex
                    ? (center0 + filletNorm * seedRadius)
                    : (center0 - filletNorm * seedRadius), i, false);
                Geom::Point3D pt1 = toMiterPlane(isConvex
                    ? (center1 + filletNorm * seedRadius)
                    : (center1 - filletNorm * seedRadius), i, true);

                band0[b] = static_cast<uint32_t>(vertices.size());
                vertices.push_back({
                    {static_cast<float>(pt0.x), static_cast<float>(pt0.y), static_cast<float>(pt0.z)},
                    {static_cast<float>(filletNorm.x), static_cast<float>(filletNorm.y), static_cast<float>(filletNorm.z)}
                });

                band1[b] = static_cast<uint32_t>(vertices.size());
                vertices.push_back({
                    {static_cast<float>(pt1.x), static_cast<float>(pt1.y), static_cast<float>(pt1.z)},
                    {static_cast<float>(filletNorm.x), static_cast<float>(filletNorm.y), static_cast<float>(filletNorm.z)}
                });
            }

            allBand0[i] = band0;
            allBand1[i] = band1;

            // Stitch fillet skin quads
            for (int b = 0; b < numBands; ++b) {
                uint32_t p0_a = band0[b];
                uint32_t p1_a = band1[b];
                uint32_t p0_b = band0[b + 1];
                uint32_t p1_b = band1[b + 1];

                Geom::Point3D pa(vertices[p0_a].position[0], vertices[p0_a].position[1], vertices[p0_a].position[2]);
                Geom::Point3D pb(vertices[p1_a].position[0], vertices[p1_a].position[1], vertices[p1_a].position[2]);
                Geom::Point3D pc(vertices[p0_b].position[0], vertices[p0_b].position[1], vertices[p0_b].position[2]);
                Geom::Vector3D n = (pb - pa).cross(pc - pa);
                Geom::Vector3D expectedNorm = (normalA + chain[i].normalB).normalized();

                if (n.dot(expectedNorm) >= 0.0) {
                    indices.push_back(p0_a); indices.push_back(p1_a); indices.push_back(p1_b);
                    indices.push_back(p0_a); indices.push_back(p1_b); indices.push_back(p0_b);
                } else {
                    indices.push_back(p0_a); indices.push_back(p1_b); indices.push_back(p1_a);
                    indices.push_back(p0_a); indices.push_back(p0_b); indices.push_back(p1_b);
                }
            }

            // Connect miter seam between adjacent segments on Face B
            if (i + 1 < chainCount) {
                uint32_t v_currB1 = v_sB1[i];
                uint32_t v_nextB0 = v_sB0[i + 1];
                if (v_currB1 != v_nextB0 && sB1_pts[i].distance_to(sB0_pts[i + 1]) > 1e-4) {
                    uint32_t v_midA = v_sA[i + 1];
                    Geom::Point3D pA(vertices[v_midA].position[0], vertices[v_midA].position[1], vertices[v_midA].position[2]);
                    Geom::Point3D pB1(vertices[v_currB1].position[0], vertices[v_currB1].position[1], vertices[v_currB1].position[2]);
                    Geom::Point3D pB0(vertices[v_nextB0].position[0], vertices[v_nextB0].position[1], vertices[v_nextB0].position[2]);
                    Geom::Vector3D tn = (pB1 - pA).cross(pB0 - pA);
                    Geom::Vector3D expN = (chain[i].normalB + chain[i + 1].normalB).normalized();
                    if (tn.dot(expN) >= 0.0) {
                        indices.push_back(v_midA); indices.push_back(v_currB1); indices.push_back(v_nextB0);
                    } else {
                        indices.push_back(v_midA); indices.push_back(v_nextB0); indices.push_back(v_currB1);
                    }
                }
            } else if (isLoop) {
                uint32_t v_currB1 = v_sB1.back();
                uint32_t v_nextB0 = v_sB0.front();
                if (v_currB1 != v_nextB0 && sB1_pts.back().distance_to(sB0_pts.front()) > 1e-4) {
                    uint32_t v_midA = v_sA.front();
                    Geom::Point3D pA(vertices[v_midA].position[0], vertices[v_midA].position[1], vertices[v_midA].position[2]);
                    Geom::Point3D pB1(vertices[v_currB1].position[0], vertices[v_currB1].position[1], vertices[v_currB1].position[2]);
                    Geom::Point3D pB0(vertices[v_nextB0].position[0], vertices[v_nextB0].position[1], vertices[v_nextB0].position[2]);
                    Geom::Vector3D tn = (pB1 - pA).cross(pB0 - pA);
                    Geom::Vector3D expN = (chain.back().normalB + chain.front().normalB).normalized();
                    if (tn.dot(expN) >= 0.0) {
                        indices.push_back(v_midA); indices.push_back(v_currB1); indices.push_back(v_nextB0);
                    } else {
                        indices.push_back(v_midA); indices.push_back(v_nextB0); indices.push_back(v_currB1);
                    }
                }
            }
        }

        // Add end caps ONLY if actual capping faces meet at the open ends of the chain
        if (!isLoop) {
            bool hasEnd0Face = false;
            bool hasEnd1Face = false;
            Geom::Point3D chainP0 = chain.front().p0;
            Geom::Point3D chainP1 = chain.back().p1;
            Geom::Vector3D dir0 = chain.front().edgeVec;
            Geom::Vector3D dir1 = chain.back().edgeVec;

            for (size_t t = 0; t < numTriangles; ++t) {
                uint32_t a = baseIndices[t * 3 + 0];
                uint32_t b = baseIndices[t * 3 + 1];
                uint32_t c = baseIndices[t * 3 + 2];
                Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();

                if (triNorm.dot(normalA) <= 0.85 && triNorm.dot(chain.front().normalB) <= 0.85) {
                    if (triNorm.dot(-dir0) > 0.4) {
                        if (pa.distance_to(chainP0) < 0.05 || pb.distance_to(chainP0) < 0.05 || pc.distance_to(chainP0) < 0.05) {
                            hasEnd0Face = true;
                        }
                    }
                }
                if (triNorm.dot(normalA) <= 0.85 && triNorm.dot(chain.back().normalB) <= 0.85) {
                    if (triNorm.dot(dir1) > 0.4) {
                        if (pa.distance_to(chainP1) < 0.05 || pb.distance_to(chainP1) < 0.05 || pc.distance_to(chainP1) < 0.05) {
                            hasEnd1Face = true;
                        }
                    }
                }
            }

            if (hasEnd0Face) {
                Geom::Vector3D endNorm0 = (-dir0).normalized();
                uint32_t v_startA = v_sA.front();

                // 1. In existing base mesh triangles on the end face, update vertices near chainP0 to v_startA
                for (size_t t = 0; t < numTriangles; ++t) {
                    uint32_t a = baseIndices[t * 3 + 0];
                    uint32_t b = baseIndices[t * 3 + 1];
                    uint32_t c = baseIndices[t * 3 + 2];
                    Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                    Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                    Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                    Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();

                    if (triNorm.dot(endNorm0) > 0.4) {
                        for (int k = 0; k < 3; ++k) {
                            uint32_t origIdx = baseIndices[t * 3 + k];
                            Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                            if (p.distance_to(chainP0) < 0.05) {
                                indices[t * 3 + k] = v_startA;
                            }
                        }
                    }
                }

                // 2. Add planar fan covering the circular segment between chord (band0[0], band0[numBands]) and the arc
                for (int b = 1; b < numBands; ++b) {
                    uint32_t va = allBand0[0][b];
                    uint32_t vb = allBand0[0][b + 1];
                    Geom::Point3D pa(vertices[v_startA].position[0], vertices[v_startA].position[1], vertices[v_startA].position[2]);
                    Geom::Point3D pb(vertices[va].position[0], vertices[va].position[1], vertices[va].position[2]);
                    Geom::Point3D pc(vertices[vb].position[0], vertices[vb].position[1], vertices[vb].position[2]);
                    Geom::Vector3D tn = (pb - pa).cross(pc - pa);
                    if (tn.dot(endNorm0) >= 0.0) {
                        indices.push_back(v_startA); indices.push_back(va); indices.push_back(vb);
                    } else {
                        indices.push_back(v_startA); indices.push_back(vb); indices.push_back(va);
                    }
                }
            }

            if (hasEnd1Face) {
                Geom::Vector3D endNorm1 = dir1.normalized();
                uint32_t v_endA = v_sA.back();

                // 1. In existing base mesh triangles on the end face, update vertices near chainP1 to v_endA
                for (size_t t = 0; t < numTriangles; ++t) {
                    uint32_t a = baseIndices[t * 3 + 0];
                    uint32_t b = baseIndices[t * 3 + 1];
                    uint32_t c = baseIndices[t * 3 + 2];
                    Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                    Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                    Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                    Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();

                    if (triNorm.dot(endNorm1) > 0.4) {
                        for (int k = 0; k < 3; ++k) {
                            uint32_t origIdx = baseIndices[t * 3 + k];
                            Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                            if (p.distance_to(chainP1) < 0.05) {
                                indices[t * 3 + k] = v_endA;
                            }
                        }
                    }
                }

                // 2. Add planar fan covering the circular segment between chord (band1[0], band1[numBands]) and the arc
                for (int b = 1; b < numBands; ++b) {
                    uint32_t va = allBand1[chainCount - 1][b];
                    uint32_t vb = allBand1[chainCount - 1][b + 1];
                    Geom::Point3D pa(vertices[v_endA].position[0], vertices[v_endA].position[1], vertices[v_endA].position[2]);
                    Geom::Point3D pb(vertices[va].position[0], vertices[va].position[1], vertices[va].position[2]);
                    Geom::Point3D pc(vertices[vb].position[0], vertices[vb].position[1], vertices[vb].position[2]);
                    Geom::Vector3D tn = (pb - pa).cross(pc - pa);
                    if (tn.dot(endNorm1) >= 0.0) {
                        indices.push_back(v_endA); indices.push_back(va); indices.push_back(vb);
                    } else {
                        indices.push_back(v_endA); indices.push_back(vb); indices.push_back(va);
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

} // namespace Tessera
