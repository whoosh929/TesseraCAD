#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/modeling/MeshBoolean3D.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <memory>

namespace Tessera {

enum class ExtrudeOperation : uint8_t {
    Boss = 0, // Add material (default)
    Cut  = 1  // Cut away material from target model
};

class ExtrudeNode : public DAGNode {
public:
    // Standalone profile constructor
    ExtrudeNode(
        std::string name,
        std::vector<std::array<float, 2>> profile2D,
        float height,
        ExtrudeOperation op = ExtrudeOperation::Boss,
        std::shared_ptr<DAGNode> target = nullptr
    ) : DAGNode(std::move(name)),
        m_profile(std::move(profile2D)),
        m_height(height),
        m_operation(op),
        m_targetNode(target),
        m_targetNodeId(target ? target->GetId() : 0) {
        if (m_targetNode && m_operation == ExtrudeOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    // Reference Sketch constructor (Parametric DAG dependency)
    ExtrudeNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float height,
        ExtrudeOperation op = ExtrudeOperation::Boss,
        std::shared_ptr<DAGNode> target = nullptr
    ) : DAGNode(std::move(name)),
        m_sketchNode(sketch),
        m_height(height),
        m_operation(op),
        m_targetNode(target),
        m_targetNodeId(target ? target->GetId() : 0) {
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_profile = m_sketchNode->GetProfile();
            m_plane = m_sketchNode->GetPlane();
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }
        if (m_targetNode && m_operation == ExtrudeOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    ~ExtrudeNode() override {
        if (m_targetNode && m_operation == ExtrudeOperation::Cut) {
            m_targetNode->SetConsumed(false);
        }
    }

    [[nodiscard]] ExtrudeOperation GetOperation() const noexcept { return m_operation; }
    void SetOperation(ExtrudeOperation op) {
        if (m_operation != op) {
            m_operation = op;
            if (m_operation == ExtrudeOperation::Boss && m_targetNode) {
                m_targetNode->SetConsumed(false);
            } else if (m_operation == ExtrudeOperation::Cut && m_targetNode) {
                m_targetNode->SetConsumed(true);
            }
            MarkDirty();
        }
    }
    [[nodiscard]] bool IsCut() const noexcept { return m_operation == ExtrudeOperation::Cut; }

    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_targetNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetSolid() const noexcept override {
        return (m_operation == ExtrudeOperation::Cut) ? m_targetNode : nullptr;
    }
    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override {
        return (m_operation == ExtrudeOperation::Cut) ? m_targetNode : nullptr;
    }
    [[nodiscard]] bool HasParentFeature() const noexcept override {
        return (m_operation == ExtrudeOperation::Cut) && (m_targetNode != nullptr);
    }

    void SetTargetFeature(std::shared_ptr<DAGNode> target) {
        if (m_targetNode && m_targetNode != target && m_operation == ExtrudeOperation::Cut) {
            m_targetNode->SetConsumed(false);
            m_targetNode->MarkDirty();
        }
        m_targetNode = target;
        m_targetNodeId = target ? target->GetId() : 0;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (m_targetNode && m_operation == ExtrudeOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
        MarkDirty();
    }

    void NotifyRemoval() override {
        if (m_targetNode && m_operation == ExtrudeOperation::Cut) {
            m_targetNode->SetConsumed(false);
            m_targetNode->MarkDirty();
        }
        DAGNode::NotifyRemoval();
    }

    void OnParentRemoved(uint64_t parentId) override {
        if (m_sketchNode && m_sketchNode->GetId() == parentId) {
            m_sketchNode = nullptr;
            m_hasConflict = true;
            m_conflictMessage = "Referenced 2D profile sketch was deleted.";
            MarkDirty();
        }
        if (m_targetNodeId == parentId) {
            m_targetNode = nullptr;
            m_hasConflict = true;
            m_conflictMessage = "Referenced parent model feature was deleted.";
            MarkDirty();
        }
        DAGNode::OnParentRemoved(parentId);
    }

    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }

    void SetHeight(float height) {
        if (m_height != height) {
            m_height = height;
            MarkDirty();
        }
    }

    void SetProfile(std::vector<std::array<float, 2>> profile2D) {
        m_profile = std::move(profile2D);
        m_sketchNode = nullptr; // Detached
        MarkDirty();
    }

    void SetSketchReference(std::shared_ptr<SketchNode> sketch) {
        m_sketchNode = sketch;
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_profile = m_sketchNode->GetProfile();
            m_plane = m_sketchNode->GetPlane();
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
            MarkDirty();
        }
    }

    [[nodiscard]] std::shared_ptr<SketchNode> GetSketchReference() const noexcept { return m_sketchNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetChildSketch() const noexcept override { return m_sketchNode; }

    void DetachChildSketch() noexcept override {
        if (m_sketchNode) {
            m_sketchNode->SetParentFeature(nullptr);
            m_sketchNode->SetConsumed(false);
            m_sketchNode->SetVisible(true);
            m_sketchNode->MarkDirty();
            m_sketchNode = nullptr;
        }
    }

    [[nodiscard]] float GetHeight() const noexcept { return m_height; }
    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept { return m_profile; }
    [[nodiscard]] SketchPlane GetPlane() const noexcept { return m_plane; }
    void SetPlane(SketchPlane plane) { m_plane = plane; MarkDirty(); }
    void SetCustomPlane(const Geom::Plane& plane) { m_customPlane = plane; m_useCustomPlane = true; MarkDirty(); }
    [[nodiscard]] bool UsesCustomPlane() const noexcept { return m_useCustomPlane; }

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

        // If referencing a sketch node, synchronize profile, plane, and position
        if (m_sketchNode) {
            m_profile = m_sketchNode->GetProfile();
            m_plane = m_sketchNode->GetPlane();
            m_useCustomPlane = m_sketchNode->HasCustomPlane();
            if (m_useCustomPlane) {
                m_customPlane = m_sketchNode->GetCustomPlane();
            }
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }

        std::vector<std::vector<std::array<float, 2>>> loopsToExtrude;
        if (m_sketchNode && !m_sketchNode->GetShapes().empty()) {
            loopsToExtrude = m_sketchNode->GetShapes();
        } else if (!m_profile.empty()) {
            loopsToExtrude.push_back(m_profile);
        }

        if (!IsDirty()) return;

        if (m_hasConflict || loopsToExtrude.empty()) {
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            ClearDirty();
            return;
        }

        uint64_t currentGen = GetGeneration();
        float h = m_height;

        std::cout << "[ExtrudeNode::Evaluate] Starting CAD 2D-to-3D Extrusion for '" 
                  << m_name << "' (Gen " << currentGen << " | Loops: " 
                  << loopsToExtrude.size() << ", Height: " << h 
                  << (m_sketchNode ? " | Ref: " + m_sketchNode->GetName() : " | Standalone") 
                  << ")...\n";

        // Simulate CAD tessellation latency
        std::this_thread::sleep_for(std::chrono::milliseconds(90));

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;

        // Helper lambdas to map 2D point (u, v) and extrusion coordinate w (distance along extrusion direction from sketch plane) to 3D
        auto MakePos = [this](float u, float v, float w) -> std::array<float, 3> {
            if (m_useCustomPlane) {
                Geom::Point3D ptOnPlane = m_customPlane.to_world(Geom::Point2D(u, v));
                Geom::Point3D pt3D = ptOnPlane + m_customPlane.normal * static_cast<double>(w);
                return { static_cast<float>(pt3D.x), static_cast<float>(pt3D.y), static_cast<float>(pt3D.z) };
            }
            switch (m_plane) {
                case SketchPlane::XZ_Ground:
                    return {u + m_position[0], w + m_position[1], v + m_position[2]};
                case SketchPlane::XY_Front:
                    return {u + m_position[0], v + m_position[1], w + m_position[2]};
                case SketchPlane::YZ_Right:
                    return {w + m_position[0], v + m_position[1], u + m_position[2]};
            }
            return {u, w, v};
        };

        auto MakeNormal = [this](float nu, float nv, float nw) -> std::array<float, 3> {
            if (m_useCustomPlane) {
                Geom::Vector3D n = m_customPlane.to_world_vector(Geom::Vector2D(nu, nv)) + m_customPlane.normal * nw;
                double len = n.length();
                if (len > 1e-6) n = n / len;
                return { static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z) };
            }
            switch (m_plane) {
                case SketchPlane::XZ_Ground:
                    return {nu, nw, nv};
                case SketchPlane::XY_Front:
                    return {nu, nv, nw};
                case SketchPlane::YZ_Right:
                    return {nw, nv, nu};
            }
            return {nu, nw, nv};
        };

        // -------------------------------------------------------------
        // Branch A: Cut Material from Target Model (Extruded Cut)
        // -------------------------------------------------------------
        if (m_operation == ExtrudeOperation::Cut) {
            // Ensure target is a solid feature, not a sketch or datum plane
            if (m_targetNode && (std::dynamic_pointer_cast<SketchNode>(m_targetNode) ||
                                 std::dynamic_pointer_cast<DatumPlaneNode>(m_targetNode) ||
                                 std::dynamic_pointer_cast<DatumAxisNode>(m_targetNode) ||
                                 m_targetNode.get() == this)) {
                if (auto sk = std::dynamic_pointer_cast<SketchNode>(m_targetNode)) {
                    auto p = sk->GetParentFeature();
                    if (p && p.get() != this && !std::dynamic_pointer_cast<SketchNode>(p) &&
                        !std::dynamic_pointer_cast<DatumPlaneNode>(p) && !std::dynamic_pointer_cast<DatumAxisNode>(p)) {
                        m_targetNode = p;
                        m_targetNodeId = p->GetId();
                    } else if (auto dp = sk->GetDatumPlaneReference()) {
                        auto t = dp->GetTargetFeature();
                        if (t && t.get() != this) {
                            m_targetNode = t;
                            m_targetNodeId = t->GetId();
                        } else {
                            m_targetNode = nullptr;
                        }
                    } else {
                        m_targetNode = nullptr;
                    }
                } else if (auto dp = std::dynamic_pointer_cast<DatumPlaneNode>(m_targetNode)) {
                    auto t = dp->GetTargetFeature();
                    if (t && t.get() != this) {
                        m_targetNode = t;
                        m_targetNodeId = t->GetId();
                    } else {
                        m_targetNode = nullptr;
                    }
                } else {
                    m_targetNode = nullptr;
                }
            }

            if (!m_targetNode || m_targetNode.get() == this) {
                m_targetNode = nullptr;
                if (m_sketchNode) {
                    if (auto dp = m_sketchNode->GetDatumPlaneReference()) {
                        auto t = dp->GetTargetFeature();
                        if (t && t.get() != this) {
                            m_targetNode = t;
                            m_targetNodeId = t->GetId();
                        }
                    }
                    if (!m_targetNode) {
                        auto p = m_sketchNode->GetParentFeature();
                        if (p && p.get() != this && !std::dynamic_pointer_cast<SketchNode>(p) &&
                            !std::dynamic_pointer_cast<DatumPlaneNode>(p) && !std::dynamic_pointer_cast<DatumAxisNode>(p)) {
                            m_targetNode = p;
                            m_targetNodeId = p->GetId();
                        }
                    }
                }
            }

            if (!m_targetNode || m_targetNode.get() == this) {
                m_hasConflict = true;
                m_conflictMessage = "No target model feature referenced for Cut Extrude.";
                m_cachedVertices.clear();
                m_cachedIndices.clear();
                m_cachedEdges.clear();
                ClearDirty();
                return;
            }

            bool hasTargetInInputs = false;
            for (const auto& in : m_inputs) {
                if (in && in->GetId() == m_targetNode->GetId()) {
                    hasTargetInInputs = true;
                    break;
                }
            }
            if (!hasTargetInInputs) {
                AddInput(m_targetNode);
            }

            if (m_targetNode->HasConflict()) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced target model has a conflict: " + m_targetNode->GetConflictMessage();
                m_cachedVertices.clear();
                m_cachedIndices.clear();
                m_cachedEdges.clear();
                ClearDirty();
                return;
            }

            if (m_targetNode != nullptr && m_targetNode.get() != this && m_targetNode->IsDirty()) {
                m_targetNode->Evaluate(device, exchange);
            }

            const auto& targetVerts = m_targetNode->GetCachedVertices();
            const auto& targetIndices = m_targetNode->GetCachedIndices();

            if (targetVerts.empty() || targetIndices.empty()) {
                m_hasConflict = true;
                m_conflictMessage = "Target model feature has empty mesh.";
                m_cachedVertices.clear();
                m_cachedIndices.clear();
                m_cachedEdges.clear();
                ClearDirty();
                return;
            }

            m_hasConflict = false;
            m_conflictMessage.clear();
            m_targetNode->SetConsumed(true);
            auto leaf = m_targetNode->GetActiveLeafFeature();
            if (leaf && leaf.get() != this) {
                leaf->SetConsumed(true);
            }

            // Generate cutter solid mesh (extending slightly backward to avoid coplanar boundary issues)
            std::vector<CADVertex> cutterVerts;
            std::vector<uint32_t> cutterIndices;

            float cutterStart = (h >= 0.0f) ? -0.015f : 0.015f;
            float cutterEnd = h;
            bool cutterReversed = (cutterEnd < cutterStart);

            auto cTopNormal = cutterReversed ? MakeNormal(0.0f, 0.0f, -1.0f) : MakeNormal(0.0f, 0.0f, 1.0f);
            auto cBotNormal = cutterReversed ? MakeNormal(0.0f, 0.0f, 1.0f) : MakeNormal(0.0f, 0.0f, -1.0f);

            for (const auto& loop : loopsToExtrude) {
                if (loop.size() < 3) continue;
                const size_t ptCount = loop.size();
                std::vector<uint32_t> capIndices = TessellationUtils::TriangulatePolygon2D(loop);

                // Top Cap at w = cutterEnd
                uint32_t topBase = static_cast<uint32_t>(cutterVerts.size());
                for (const auto& pt : loop) {
                    cutterVerts.push_back({
                        .position = {MakePos(pt[0], pt[1], cutterEnd)[0], MakePos(pt[0], pt[1], cutterEnd)[1], MakePos(pt[0], pt[1], cutterEnd)[2]},
                        .normal = {cTopNormal[0], cTopNormal[1], cTopNormal[2]}
                    });
                }
                if (!cutterReversed) {
                    for (uint32_t idx : capIndices) cutterIndices.push_back(topBase + idx);
                } else {
                    for (size_t i = 0; i < capIndices.size(); i += 3) {
                        cutterIndices.push_back(topBase + capIndices[i]);
                        cutterIndices.push_back(topBase + capIndices[i + 2]);
                        cutterIndices.push_back(topBase + capIndices[i + 1]);
                    }
                }

                // Base Cap at w = cutterStart
                uint32_t botBase = static_cast<uint32_t>(cutterVerts.size());
                for (const auto& pt : loop) {
                    cutterVerts.push_back({
                        .position = {MakePos(pt[0], pt[1], cutterStart)[0], MakePos(pt[0], pt[1], cutterStart)[1], MakePos(pt[0], pt[1], cutterStart)[2]},
                        .normal = {cBotNormal[0], cBotNormal[1], cBotNormal[2]}
                    });
                }
                if (!cutterReversed) {
                    for (size_t i = 0; i < capIndices.size(); i += 3) {
                        cutterIndices.push_back(botBase + capIndices[i]);
                        cutterIndices.push_back(botBase + capIndices[i + 2]);
                        cutterIndices.push_back(botBase + capIndices[i + 1]);
                    }
                } else {
                    for (uint32_t idx : capIndices) cutterIndices.push_back(botBase + idx);
                }

                // Side Walls between cutterStart and cutterEnd
                for (size_t i = 0; i < ptCount; ++i) {
                    size_t next = (i + 1) % ptCount;
                    float x0 = loop[i][0];
                    float z0 = loop[i][1];
                    float x1 = loop[next][0];
                    float z1 = loop[next][1];

                    float dx = x1 - x0;
                    float dz = z1 - z0;
                    float len = std::sqrt(dx * dx + dz * dz);
                    float nx = (len > 1e-6f) ? (dz / len) : 0.0f;
                    float nz = (len > 1e-6f) ? (-dx / len) : 1.0f;

                    auto wallNormal = MakeNormal(nx, nz, 0.0f);
                    uint32_t quadBase = static_cast<uint32_t>(cutterVerts.size());

                    auto p0 = MakePos(x0, z0, cutterEnd);
                    auto p1 = MakePos(x1, z1, cutterEnd);
                    auto p2 = MakePos(x1, z1, cutterStart);
                    auto p3 = MakePos(x0, z0, cutterStart);

                    cutterVerts.push_back({.position = {p0[0], p0[1], p0[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}});
                    cutterVerts.push_back({.position = {p1[0], p1[1], p1[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}});
                    cutterVerts.push_back({.position = {p2[0], p2[1], p2[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}});
                    cutterVerts.push_back({.position = {p3[0], p3[1], p3[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}});

                    if (!cutterReversed) {
                        cutterIndices.push_back(quadBase + 0);
                        cutterIndices.push_back(quadBase + 3);
                        cutterIndices.push_back(quadBase + 1);
                        cutterIndices.push_back(quadBase + 1);
                        cutterIndices.push_back(quadBase + 3);
                        cutterIndices.push_back(quadBase + 2);
                    } else {
                        cutterIndices.push_back(quadBase + 0);
                        cutterIndices.push_back(quadBase + 1);
                        cutterIndices.push_back(quadBase + 3);
                        cutterIndices.push_back(quadBase + 1);
                        cutterIndices.push_back(quadBase + 2);
                        cutterIndices.push_back(quadBase + 3);
                    }
                }
            }

            // Perform 3D Boolean Difference: target \ cutter
            bool diffOk = Modeling::MeshBoolean3D::Difference(
                targetVerts, targetIndices,
                cutterVerts, cutterIndices,
                m_cachedVertices, m_cachedIndices
            );

            if (!diffOk || m_cachedVertices.empty() || m_cachedIndices.empty()) {
                m_cachedVertices = targetVerts;
                m_cachedIndices = targetIndices;
            }

            m_cachedEdges = TessellationUtils::ExtractFeatureEdges(m_cachedVertices, m_cachedIndices, 30.0f);
            ComputeFaces();
            ClearDirty();
            return;
        }

        // -------------------------------------------------------------
        // Branch B: Add Material / Boss Extrude
        // -------------------------------------------------------------
        const bool isNegative = (h < 0.0f);
        auto topNormal = isNegative ? MakeNormal(0.0f, 0.0f, -1.0f) : MakeNormal(0.0f, 0.0f, 1.0f);
        auto botNormal = isNegative ? MakeNormal(0.0f, 0.0f, 1.0f) : MakeNormal(0.0f, 0.0f, -1.0f);

        for (const auto& loop : loopsToExtrude) {
            if (loop.size() < 3) continue;
            const size_t ptCount = loop.size();

            // 1. Extruded Cap at w = h (+Extrude direction when h > 0, -Extrude direction when h < 0)
            std::vector<uint32_t> capIndices = TessellationUtils::TriangulatePolygon2D(loop);

            uint32_t topBase = static_cast<uint32_t>(vertices.size());
            for (const auto& pt : loop) {
                vertices.push_back({
                    .position = {MakePos(pt[0], pt[1], h)[0], MakePos(pt[0], pt[1], h)[1], MakePos(pt[0], pt[1], h)[2]},
                    .normal = {topNormal[0], topNormal[1], topNormal[2]}
                });
            }
            if (!isNegative) {
                for (uint32_t idx : capIndices) {
                    indices.push_back(topBase + idx);
                }
            } else {
                for (size_t i = 0; i < capIndices.size(); i += 3) {
                    indices.push_back(topBase + capIndices[i]);
                    indices.push_back(topBase + capIndices[i + 2]);
                    indices.push_back(topBase + capIndices[i + 1]);
                }
            }

            // 2. Base Cap at w = 0 (Fixed on the sketch plane; -Extrude direction when h > 0, +Extrude direction when h < 0)
            uint32_t botBase = static_cast<uint32_t>(vertices.size());
            for (const auto& pt : loop) {
                vertices.push_back({
                    .position = {MakePos(pt[0], pt[1], 0.0f)[0], MakePos(pt[0], pt[1], 0.0f)[1], MakePos(pt[0], pt[1], 0.0f)[2]},
                    .normal = {botNormal[0], botNormal[1], botNormal[2]}
                });
            }
            if (!isNegative) {
                for (size_t i = 0; i < capIndices.size(); i += 3) {
                    indices.push_back(botBase + capIndices[i]);
                    indices.push_back(botBase + capIndices[i + 2]);
                    indices.push_back(botBase + capIndices[i + 1]);
                }
            } else {
                for (uint32_t idx : capIndices) {
                    indices.push_back(botBase + idx);
                }
            }

            // 3. Side Walls (Extruded Quads between w = h and w = 0)
            for (size_t i = 0; i < ptCount; ++i) {
                size_t next = (i + 1) % ptCount;

                float x0 = loop[i][0];
                float z0 = loop[i][1];
                float x1 = loop[next][0];
                float z1 = loop[next][1];

                // 2D edge vector (dx, dz) -> outward 2D normal (dz, -dx)
                float dx = x1 - x0;
                float dz = z1 - z0;
                float len = std::sqrt(dx * dx + dz * dz);
                float nx = (len > 1e-6f) ? (dz / len) : 0.0f;
                float nz = (len > 1e-6f) ? (-dx / len) : 1.0f;

                auto wallNormal = MakeNormal(nx, nz, 0.0f);

                uint32_t quadBase = static_cast<uint32_t>(vertices.size());

                auto p0 = MakePos(x0, z0, h);
                auto p1 = MakePos(x1, z1, h);
                auto p2 = MakePos(x1, z1, 0.0f);
                auto p3 = MakePos(x0, z0, 0.0f);

                vertices.push_back({.position = {p0[0], p0[1], p0[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 0: Top start
                vertices.push_back({.position = {p1[0], p1[1], p1[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 1: Top end
                vertices.push_back({.position = {p2[0], p2[1], p2[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 2: Bot end
                vertices.push_back({.position = {p3[0], p3[1], p3[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 3: Bot start

                if (!isNegative) {
                    indices.push_back(quadBase + 0);
                    indices.push_back(quadBase + 3);
                    indices.push_back(quadBase + 1);

                    indices.push_back(quadBase + 1);
                    indices.push_back(quadBase + 3);
                    indices.push_back(quadBase + 2);
                } else {
                    indices.push_back(quadBase + 0);
                    indices.push_back(quadBase + 1);
                    indices.push_back(quadBase + 3);

                    indices.push_back(quadBase + 1);
                    indices.push_back(quadBase + 2);
                    indices.push_back(quadBase + 3);
                }
            }
        }

        // Extract sharp feature silhouette edges
        std::vector<uint32_t> featureEdges = TessellationUtils::ExtractFeatureEdges(vertices, indices, 30.0f);

        m_cachedVertices = vertices;
        m_cachedIndices = indices;
        m_cachedEdges = featureEdges;

        ComputeFaces();
        ClearDirty();
    }

    [[nodiscard]] bool HasCustomPlane() const noexcept { return m_useCustomPlane; }
    [[nodiscard]] Geom::Plane GetCustomPlane() const noexcept { return m_customPlane; }

private:
    std::shared_ptr<SketchNode> m_sketchNode{nullptr};
    std::vector<std::array<float, 2>> m_profile;
    float m_height{2.0f};
    ExtrudeOperation m_operation{ExtrudeOperation::Boss};
    std::shared_ptr<DAGNode> m_targetNode{nullptr};
    uint64_t m_targetNodeId{0};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
    SketchPlane m_plane{SketchPlane::XZ_Ground};
    bool m_useCustomPlane{false};
    Geom::Plane m_customPlane{Geom::Plane::XY()};
};

class CutExtrudeNode : public ExtrudeNode {
public:
    CutExtrudeNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float depth,
        std::shared_ptr<DAGNode> target = nullptr
    ) : ExtrudeNode(std::move(name), sketch, depth, ExtrudeOperation::Cut, target) {}
};

} // namespace Tessera
