#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "Tessellation.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/geom/Axis3D.hpp"
#include "engine/sketch/Sketch.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/modeling/Topology.hpp"
#include "engine/modeling/RevolveOp.hpp"
#include "engine/modeling/Tessellator.hpp"
#include "engine/modeling/MeshBoolean3D.hpp"
#include "engine/hal/RHI.hpp"
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace Tessera {

enum class RevolveOperation : uint8_t {
    Boss = 0, // Add material (default)
    Cut  = 1  // Cut away material from target model
};

enum class RevolveAxisMode : uint8_t {
    Sketch_V_Axis = 0,    // In-plane Vertical (V) axis of sketch
    Sketch_U_Axis = 1,    // In-plane Horizontal (U) axis of sketch
    World_Y_Axis  = 2,    // Global Y-axis (0, 1, 0)
    World_X_Axis  = 3,    // Global X-axis (1, 0, 0)
    World_Z_Axis  = 4,    // Global Z-axis (0, 0, 1)
    Custom_Axis   = 5     // User-specified 3D datum axis
};

inline const char* GetRevolveAxisModeName(RevolveAxisMode mode) noexcept {
    switch (mode) {
        case RevolveAxisMode::Sketch_V_Axis: return "Sketch V-Axis (Vertical)";
        case RevolveAxisMode::Sketch_U_Axis: return "Sketch U-Axis (Horizontal)";
        case RevolveAxisMode::World_Y_Axis:  return "World Y-Axis (0, 1, 0)";
        case RevolveAxisMode::World_X_Axis:  return "World X-Axis (1, 0, 0)";
        case RevolveAxisMode::World_Z_Axis:  return "World Z-Axis (0, 0, 1)";
        case RevolveAxisMode::Custom_Axis:   return "Custom Datum Axis";
    }
    return "Unknown Axis";
}

class RevolveNode : public DAGNode {
public:
    RevolveNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float angleDegrees = 360.0f,
        RevolveAxisMode axisMode = RevolveAxisMode::Sketch_V_Axis,
        RevolveOperation op = RevolveOperation::Boss,
        std::shared_ptr<DAGNode> target = nullptr
    ) : DAGNode(std::move(name)),
        m_sketchNode(sketch),
        m_angleDegrees(angleDegrees),
        m_axisMode(axisMode),
        m_operation(op),
        m_targetNode(target),
        m_targetNodeId(target ? target->GetId() : 0) {
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }
        if (m_targetNode && m_operation == RevolveOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    RevolveNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float angleDegrees,
        Geom::Axis3D customAxis,
        RevolveOperation op = RevolveOperation::Boss,
        std::shared_ptr<DAGNode> target = nullptr
    ) : DAGNode(std::move(name)),
        m_sketchNode(sketch),
        m_angleDegrees(angleDegrees),
        m_axis(customAxis),
        m_axisMode(RevolveAxisMode::Custom_Axis),
        m_operation(op),
        m_targetNode(target),
        m_targetNodeId(target ? target->GetId() : 0) {
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }
        if (m_targetNode && m_operation == RevolveOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    ~RevolveNode() override {
        if (m_targetNode && m_operation == RevolveOperation::Cut) {
            m_targetNode->SetConsumed(false);
        }
    }

    [[nodiscard]] RevolveOperation GetOperation() const noexcept { return m_operation; }
    void SetOperation(RevolveOperation op) {
        if (m_operation != op) {
            m_operation = op;
            if (m_operation == RevolveOperation::Boss && m_targetNode) {
                m_targetNode->SetConsumed(false);
            } else if (m_operation == RevolveOperation::Cut && m_targetNode) {
                m_targetNode->SetConsumed(true);
            }
            MarkDirty();
        }
    }
    [[nodiscard]] bool IsCut() const noexcept { return m_operation == RevolveOperation::Cut; }

    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_targetNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override {
        return (m_operation == RevolveOperation::Cut) ? m_targetNode : nullptr;
    }
    [[nodiscard]] bool HasParentFeature() const noexcept override {
        return (m_operation == RevolveOperation::Cut) && (m_targetNode != nullptr);
    }

    void SetTargetFeature(std::shared_ptr<DAGNode> target) {
        if (m_targetNode && m_targetNode != target && m_operation == RevolveOperation::Cut) {
            m_targetNode->SetConsumed(false);
            m_targetNode->MarkDirty();
        }
        m_targetNode = target;
        m_targetNodeId = target ? target->GetId() : 0;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (m_targetNode && m_operation == RevolveOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
        MarkDirty();
    }

    void NotifyRemoval() override {
        if (m_targetNode && m_operation == RevolveOperation::Cut) {
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

    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }

    void SetAngleDegrees(float deg) {
        if (std::abs(m_angleDegrees - deg) > 0.01f) {
            m_angleDegrees = deg;
            MarkDirty();
        }
    }

    [[nodiscard]] float GetAngleDegrees() const noexcept { return m_angleDegrees; }

    void SetAngularSegments(int segs) {
        if (m_angularSegments != segs) {
            m_angularSegments = std::clamp(segs, 8, 96);
            MarkDirty();
        }
    }

    [[nodiscard]] int GetAngularSegments() const noexcept { return m_angularSegments; }

    void SetAxisMode(RevolveAxisMode mode) {
        if (m_axisMode != mode) {
            m_axisMode = mode;
            MarkDirty();
        }
    }

    [[nodiscard]] RevolveAxisMode GetAxisMode() const noexcept { return m_axisMode; }

    void SetAxis(Geom::Axis3D axis) {
        m_axis = axis;
        m_axisMode = RevolveAxisMode::Custom_Axis;
        MarkDirty();
    }

    [[nodiscard]] Geom::Axis3D GetAxis() const noexcept { return m_axis; }

    void SetSolid(bool solid) {
        if (m_isSolid != solid) {
            m_isSolid = solid;
            MarkDirty();
        }
    }

    [[nodiscard]] bool IsSolid() const noexcept { return m_isSolid; }

    void SetSketchReference(std::shared_ptr<SketchNode> sketch) {
        m_sketchNode = sketch;
        if (m_sketchNode) {
            AddInput(m_sketchNode);
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

    // Live dimensional metrics calculated directly from the 2D sketch profile
    [[nodiscard]] float GetInnerRadius() const noexcept {
        float rMin = 0.0f, rMax = 0.0f, vMin = 0.0f, vMax = 0.0f;
        CalculateMetrics(rMin, rMax, vMin, vMax);
        return rMin;
    }

    [[nodiscard]] float GetOuterRadius() const noexcept {
        float rMin = 0.0f, rMax = 0.0f, vMin = 0.0f, vMax = 0.0f;
        CalculateMetrics(rMin, rMax, vMin, vMax);
        return rMax;
    }

    [[nodiscard]] float GetWallThickness() const noexcept {
        float rMin = 0.0f, rMax = 0.0f, vMin = 0.0f, vMax = 0.0f;
        CalculateMetrics(rMin, rMax, vMin, vMax);
        return std::max(0.0f, rMax - rMin);
    }

    [[nodiscard]] float GetCylinderHeight() const noexcept {
        float rMin = 0.0f, rMax = 0.0f, vMin = 0.0f, vMax = 0.0f;
        CalculateMetrics(rMin, rMax, vMin, vMax);
        return std::max(0.0f, vMax - vMin);
    }

    [[nodiscard]] Geom::Axis3D ComputeEffectiveAxis(const Geom::Plane& sketchPlane) const noexcept {
        switch (m_axisMode) {
            case RevolveAxisMode::Sketch_V_Axis:
                return Geom::Axis3D(sketchPlane.origin, sketchPlane.v_axis);
            case RevolveAxisMode::Sketch_U_Axis:
                return Geom::Axis3D(sketchPlane.origin, sketchPlane.u_axis);
            case RevolveAxisMode::World_Y_Axis:
                return Geom::Axis3D::Y();
            case RevolveAxisMode::World_X_Axis:
                return Geom::Axis3D::X();
            case RevolveAxisMode::World_Z_Axis:
                return Geom::Axis3D::Z();
            case RevolveAxisMode::Custom_Axis:
                return m_axis;
        }
        return Geom::Axis3D(sketchPlane.origin, sketchPlane.v_axis);
    }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;

        if (!m_sketchNode) return;

        std::vector<std::vector<std::array<float, 2>>> loopsToRevolve;
        if (!m_sketchNode->GetShapes().empty()) {
            loopsToRevolve = m_sketchNode->GetShapes();
        } else if (!m_sketchNode->GetProfile().empty()) {
            loopsToRevolve.push_back(m_sketchNode->GetProfile());
        }

        if (loopsToRevolve.empty()) return;

        m_position[0] = m_sketchNode->GetPosition()[0];
        m_position[1] = m_sketchNode->GetPosition()[1];
        m_position[2] = m_sketchNode->GetPosition()[2];

        uint64_t currentGen = GetGeneration();

        Geom::Point3D sketchOrigin(m_position[0], m_position[1], m_position[2]);
        Geom::Plane sketchPlane;
        if (m_sketchNode->HasCustomPlane()) {
            sketchPlane = m_sketchNode->GetCustomPlane();
        } else {
            switch (m_sketchNode->GetPlane()) {
                case SketchPlane::XZ_Ground: sketchPlane = Geom::Plane::XZ(sketchOrigin); break;
                case SketchPlane::XY_Front:  sketchPlane = Geom::Plane::XY(sketchOrigin); break;
                case SketchPlane::YZ_Right:  sketchPlane = Geom::Plane::YZ(sketchOrigin); break;
            }
        }

        Geom::Axis3D effectiveAxis = ComputeEffectiveAxis(sketchPlane);

        std::cout << "[RevolveNode::Evaluate] Starting Revolve sweep for '" 
                  << m_name << "' (Gen " << currentGen 
                  << " | Angle: " << m_angleDegrees << " deg, Slices: " << m_angularSegments 
                  << " | Mode: " << GetRevolveAxisModeName(m_axisMode)
                  << " | Wall Thickness: " << GetWallThickness() << " m"
                  << " | Height: " << GetCylinderHeight() << " m"
                  << " | Ref: " << m_sketchNode->GetName() << ")...\n";

        std::vector<CADVertex> allVerts;
        std::vector<uint32_t> allIndices;
        std::vector<uint32_t> allEdges;

        for (const auto& loop : loopsToRevolve) {
            if (loop.size() < 3) continue;

            Sketch::Sketch sketchEnv(sketchPlane);
            for (size_t i = 0; i < loop.size(); ++i) {
                size_t next = (i + 1) % loop.size();
                sketchEnv.add_line(Geom::Point2D(loop[i][0], loop[i][1]),
                                   Geom::Point2D(loop[next][0], loop[next][1]));
            }

            auto profiles = sketchEnv.extract_profiles();
            if (profiles.empty()) continue;

            Modeling::RevolveOp::Params revParams;
            revParams.axis = effectiveAxis;
            revParams.angle_radians = (static_cast<double>(m_angleDegrees) / 180.0) * Geom::PI;
            revParams.angular_segments = m_angularSegments;
            revParams.is_solid = m_isSolid;

            auto body = Modeling::RevolveOp::Execute(profiles[0], sketchPlane, revParams);
            CADMesh mesh = Modeling::Tessellator::Tessellate(*body);

            uint32_t baseIdx = static_cast<uint32_t>(allVerts.size());
            for (const auto& v : mesh.vertices) {
                allVerts.push_back(v);
            }
            for (auto idx : mesh.indices) {
                allIndices.push_back(baseIdx + idx);
            }
            for (auto edge : mesh.featureEdgeIndices) {
                allEdges.push_back(baseIdx + edge);
            }
        }

        std::cout << "[RevolveNode::Evaluate] Node '" << m_name 
                  << "' generated cylindrical B-Rep mesh (Gen " << currentGen 
                  << ", Verts: " << allVerts.size() 
                  << ", Indices: " << allIndices.size() 
                  << ", FeatureEdges: " << allEdges.size() << ").\n";

        if (m_operation == RevolveOperation::Cut) {
            if (!m_targetNode) {
                if (m_sketchNode) {
                    if (auto p = m_sketchNode->GetParentFeature()) {
                        m_targetNode = p;
                        m_targetNodeId = p->GetId();
                    } else if (auto dp = m_sketchNode->GetDatumPlaneReference()) {
                        if (auto t = dp->GetTargetFeature()) {
                            m_targetNode = t;
                            m_targetNodeId = t->GetId();
                        }
                    }
                }
            }

            if (!m_targetNode) {
                m_hasConflict = true;
                m_conflictMessage = "No target model feature referenced for Cut Revolve.";
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

            if (m_targetNode->IsDirty()) {
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

            // Perform 3D Boolean Difference: target \ cutter (allVerts, allIndices)
            bool diffOk = Modeling::MeshBoolean3D::Difference(
                targetVerts, targetIndices,
                allVerts, allIndices,
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

        m_cachedVertices = std::move(allVerts);
        m_cachedIndices = std::move(allIndices);
        m_cachedEdges = std::move(allEdges);
        ComputeFaces();
        ClearDirty();
    }

private:
    void CalculateMetrics(float& rMin, float& rMax, float& vMin, float& vMax) const noexcept {
        rMin = 0.0f;
        rMax = 0.0f;
        vMin = 0.0f;
        vMax = 0.0f;
        if (!m_sketchNode) return;

        const auto& shapes = m_sketchNode->GetShapes();
        const auto& profile = m_sketchNode->GetProfile();

        bool first = true;
        auto processPoint = [&](float u, float v) {
            float r = (m_axisMode == RevolveAxisMode::Sketch_U_Axis) ? std::abs(v) : std::abs(u);
            float h = (m_axisMode == RevolveAxisMode::Sketch_U_Axis) ? u : v;

            if (first) {
                rMin = rMax = r;
                vMin = vMax = h;
                first = false;
            } else {
                rMin = std::min(rMin, r);
                rMax = std::max(rMax, r);
                vMin = std::min(vMin, h);
                vMax = std::max(vMax, h);
            }
        };

        if (!shapes.empty()) {
            for (const auto& s : shapes) {
                for (const auto& pt : s) {
                    processPoint(pt[0], pt[1]);
                }
            }
        } else {
            for (const auto& pt : profile) {
                processPoint(pt[0], pt[1]);
            }
        }
    }

    std::shared_ptr<SketchNode> m_sketchNode{nullptr};
    float m_angleDegrees{360.0f};
    int m_angularSegments{32};
    Geom::Axis3D m_axis{Geom::Axis3D::Y()};
    RevolveAxisMode m_axisMode{RevolveAxisMode::Sketch_V_Axis};
    bool m_isSolid{true};
    RevolveOperation m_operation{RevolveOperation::Boss};
    std::shared_ptr<DAGNode> m_targetNode{nullptr};
    uint64_t m_targetNodeId{0};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

class CutRevolveNode : public RevolveNode {
public:
    CutRevolveNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float angleDegrees = 360.0f,
        RevolveAxisMode axisMode = RevolveAxisMode::Sketch_V_Axis,
        std::shared_ptr<DAGNode> target = nullptr
    ) : RevolveNode(std::move(name), sketch, angleDegrees, axisMode, RevolveOperation::Cut, target) {}
};

} // namespace Tessera
