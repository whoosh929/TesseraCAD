#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/geom/Axis3D.hpp"
#include "engine/sketch/Sketch.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/modeling/Topology.hpp"
#include "engine/modeling/RevolveOp.hpp"
#include "engine/modeling/Tessellator.hpp"
#include "engine/hal/RHI.hpp"
#include <memory>
#include <string>
#include <vector>
#include <iostream>

namespace Tessera {

class RevolveNode : public DAGNode {
public:
    RevolveNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float angleDegrees = 360.0f,
        Geom::Axis3D axis = Geom::Axis3D::Y()
    ) : DAGNode(std::move(name)),
        m_sketchNode(sketch),
        m_angleDegrees(angleDegrees),
        m_axis(axis) {
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }
    }

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

    void SetAxis(Geom::Axis3D axis) {
        m_axis = axis;
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

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;

        if (!m_sketchNode || m_sketchNode->GetProfile().size() < 3) return;

        const auto& profile2D = m_sketchNode->GetProfile();
        uint64_t currentGen = GetGeneration();

        std::cout << "[RevolveNode::Evaluate] Starting Revolve sweep for '" 
                  << m_name << "' (Gen " << currentGen 
                  << " | Angle: " << m_angleDegrees << " deg, Slices: " << m_angularSegments 
                  << " | Ref: " << m_sketchNode->GetName() << ")...\n";

        Geom::Plane sketchPlane;
        switch (m_sketchNode->GetPlane()) {
            case SketchPlane::XZ_Ground: sketchPlane = Geom::Plane::XZ(); break;
            case SketchPlane::XY_Front:  sketchPlane = Geom::Plane::XY(); break;
            case SketchPlane::YZ_Right:  sketchPlane = Geom::Plane::YZ(); break;
        }

        Sketch::Sketch sketchEnv(sketchPlane);
        for (size_t i = 0; i < profile2D.size(); ++i) {
            size_t next = (i + 1) % profile2D.size();
            sketchEnv.add_line(Geom::Point2D(profile2D[i][0], profile2D[i][1]),
                               Geom::Point2D(profile2D[next][0], profile2D[next][1]));
        }

        auto profiles = sketchEnv.extract_profiles();
        if (profiles.empty()) return;

        Modeling::RevolveOp::Params revParams;
        revParams.axis = m_axis;
        revParams.angle_radians = (static_cast<double>(m_angleDegrees) / 180.0) * Geom::PI;
        revParams.angular_segments = m_angularSegments;
        revParams.is_solid = m_isSolid;

        auto body = Modeling::RevolveOp::Execute(profiles[0], sketchPlane, revParams);
        CADMesh mesh = Modeling::Tessellator::Tessellate(*body);

        // Apply node position
        for (auto& v : mesh.vertices) {
            v.position[0] += m_position[0];
            v.position[1] += m_position[1];
            v.position[2] += m_position[2];
        }

        std::cout << "[RevolveNode::Evaluate] Node '" << m_name 
                  << "' generated B-Rep mesh (Gen " << currentGen 
                  << ", Verts: " << mesh.vertices.size() 
                  << ", Indices: " << mesh.indices.size() 
                  << ", FeatureEdges: " << mesh.featureEdgeIndices.size() 
                  << ", Solid: " << (body->is_solid() ? "Yes" : "No") << ").\n";

        m_cachedVertices = std::move(mesh.vertices);
        m_cachedIndices = std::move(mesh.indices);
        m_cachedEdges = std::move(mesh.featureEdgeIndices);
        ClearDirty();
    }

private:
    std::shared_ptr<SketchNode> m_sketchNode{nullptr};
    float m_angleDegrees{360.0f};
    int m_angularSegments{32};
    Geom::Axis3D m_axis{Geom::Axis3D::Y()};
    bool m_isSolid{true};
};

} // namespace Tessera
