#pragma once

#include "Node.hpp"
#include "engine/geom/Axis3D.hpp"
#include <string>
#include <memory>
#include <iostream>

namespace Tessera {

enum class DatumAxisPreset : uint8_t {
    X_Axis = 0,
    Y_Axis = 1,
    Z_Axis = 2,
    Custom = 3
};

enum class DatumAxisType : uint8_t {
    Fixed = 0,
    VertexAssociative = 1
};

class DatumAxisNode : public DAGNode {
public:
    DatumAxisNode(std::string name, Geom::Axis3D axis = Geom::Axis3D::Y())
        : DAGNode(std::move(name)), m_axis(axis), m_lastValidOrigin(axis.origin), m_axisDir(axis.direction) {}

    DatumAxisNode(std::string name, std::shared_ptr<DAGNode> feature, int vertexIndex, Geom::Vector3D direction = Geom::Vector3D(0.0, 1.0, 0.0))
        : DAGNode(std::move(name)),
          m_type(DatumAxisType::VertexAssociative),
          m_parentFeature(feature),
          m_parentFeatureId(feature ? feature->GetId() : 0),
          m_vertexIndex(vertexIndex),
          m_axisDir(direction.normalized()) {
        if (feature) {
            Geom::Point3D vPos;
            if (feature->GetVertexPosition(vertexIndex, vPos)) {
                m_axis = Geom::Axis3D(vPos, m_axisDir);
                m_lastValidOrigin = vPos;
            }
        }
    }

    void SetVertexReference(std::shared_ptr<DAGNode> feature, int vertexIndex, Geom::Vector3D direction = Geom::Vector3D(0.0, 1.0, 0.0)) {
        m_type = DatumAxisType::VertexAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_vertexIndex = vertexIndex;
        m_axisDir = direction.normalized();
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (feature) {
            AddInput(feature);
            Geom::Point3D vPos;
            if (feature->GetVertexPosition(vertexIndex, vPos)) {
                m_axis = Geom::Axis3D(vPos, m_axisDir);
                m_lastValidOrigin = vPos;
            }
        }
        MarkDirty();
    }

    void ConvertToFixed() {
        m_type = DatumAxisType::Fixed;
        m_axis.origin = m_lastValidOrigin;
        m_axis.direction = m_axisDir;
        m_parentFeature.reset();
        m_parentFeatureId = 0;
        m_vertexIndex = -1;
        m_hasConflict = false;
        m_conflictMessage.clear();
        MarkDirty();
    }

    void SetAxis(Geom::Axis3D axis) {
        m_axis = axis;
        m_lastValidOrigin = axis.origin;
        m_axisDir = axis.direction;
        m_type = DatumAxisType::Fixed;
        m_parentFeature.reset();
        m_parentFeatureId = 0;
        m_hasConflict = false;
        m_conflictMessage.clear();
        MarkDirty();
    }

    [[nodiscard]] Geom::Axis3D GetAxis() const noexcept {
        return m_axis;
    }

    [[nodiscard]] DatumAxisType GetAxisType() const noexcept { return m_type; }
    [[nodiscard]] bool IsVertexAssociative() const noexcept { return m_type == DatumAxisType::VertexAssociative; }

    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }
    void ClearConflict() noexcept { m_hasConflict = false; m_conflictMessage.clear(); }

    [[nodiscard]] int GetReferencedVertexIndex() const noexcept { return m_vertexIndex; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetReferencedFeature() const noexcept { return m_parentFeature.lock(); }
    [[nodiscard]] uint64_t GetReferencedFeatureId() const noexcept { return m_parentFeatureId; }

    void OnParentRemoved(uint64_t parentId) override {
        DAGNode::OnParentRemoved(parentId);
        if (m_parentFeatureId == parentId || (m_parentFeature.expired() && m_type == DatumAxisType::VertexAssociative)) {
            m_parentFeature.reset();
            m_hasConflict = true;
            m_conflictMessage = "Referenced parent feature was deleted from the model tree.";
            MarkDirty();
        }
    }

    void Evaluate(RHI::RHIDevice*, AtomicMeshExchange&) override {
        if (m_type == DatumAxisType::VertexAssociative) {
            if (m_hasConflict) {
                return;
            }
            auto feature = m_parentFeature.lock();
            if (!feature) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced parent feature was deleted from the model tree.";
                return;
            }

            Geom::Point3D vPos;
            if (!feature->GetVertexPosition(m_vertexIndex, vPos)) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced vertex #" + std::to_string(m_vertexIndex) + 
                                    " on feature '" + feature->GetName() + "' is missing or invalid.";
                return;
            }

            // Valid vertex position update: update axis origin
            m_hasConflict = false;
            m_conflictMessage.clear();
            m_axis = Geom::Axis3D(vPos, m_axisDir);
            m_lastValidOrigin = vPos;
        }

        ClearDirty();
    }

private:
    Geom::Axis3D m_axis{Geom::Axis3D::Y()};
    Geom::Point3D m_lastValidOrigin{0.0, 0.0, 0.0};
    Geom::Vector3D m_axisDir{0.0, 1.0, 0.0};

    DatumAxisType m_type{DatumAxisType::Fixed};
    std::weak_ptr<DAGNode> m_parentFeature;
    uint64_t m_parentFeatureId{0};
    int m_vertexIndex{-1};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

} // namespace Tessera


