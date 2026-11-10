#pragma once

#include "Node.hpp"
#include "DatumPlaneNode.hpp"
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
    VertexAssociative = 1,
    FaceAssociative = 2,
    EdgeAssociative = 3
};

class DatumAxisNode : public DAGNode {
public:
    DatumAxisNode(std::string name, Geom::Axis3D axis = Geom::Axis3D::Y(), double offset = 0.0)
        : DAGNode(std::move(name)),
          m_axis(axis),
          m_lastValidOrigin(axis.origin),
          m_axisDir(axis.direction),
          m_offset(offset) {
        Geom::Plane basis(axis.origin, axis.direction);
        m_offsetDir = basis.u_axis;
    }

    DatumAxisNode(
        std::string name,
        std::shared_ptr<DAGNode> feature,
        int vertexIndex,
        Geom::Vector3D direction = Geom::Vector3D(0.0, 1.0, 0.0),
        double offset = 0.0
    ) : DAGNode(std::move(name)),
        m_axisDir(direction.normalized()),
        m_offset(offset),
        m_type(DatumAxisType::VertexAssociative),
        m_parentFeature(feature),
        m_parentFeatureId(feature ? feature->GetId() : 0),
        m_vertexIndex(vertexIndex) {
        m_offsetDir = m_axisDir;
        if (feature) {
            Geom::Point3D vPos;
            if (feature->GetVertexPosition(vertexIndex, vPos)) {
                m_axis = Geom::Axis3D(vPos, m_axisDir);
                m_lastValidOrigin = vPos;
            }
            Geom::Plane cornerPl;
            if (DatumPlaneNode::ComputeCornerVertexPlane(feature, vertexIndex, cornerPl)) {
                m_offsetDir = cornerPl.normal;
            }
        }
    }

    static std::shared_ptr<DatumAxisNode> CreateFromFace(
        std::string name,
        std::shared_ptr<DAGNode> feature,
        int faceIndex,
        Geom::Vector3D direction = Geom::Vector3D(0.0, 0.0, 0.0),
        double offset = 0.0
    ) {
        auto node = std::make_shared<DatumAxisNode>(std::move(name));
        node->SetFaceReference(std::move(feature), faceIndex, direction, offset);
        return node;
    }

    static std::shared_ptr<DatumAxisNode> CreateFromEdge(
        std::string name,
        std::shared_ptr<DAGNode> feature,
        int edgeIndex,
        double offset = 0.0,
        bool alongEdge = true
    ) {
        auto node = std::make_shared<DatumAxisNode>(std::move(name));
        node->SetEdgeReference(std::move(feature), edgeIndex, offset, alongEdge);
        return node;
    }

    void SetVertexReference(std::shared_ptr<DAGNode> feature, int vertexIndex, Geom::Vector3D direction = Geom::Vector3D(0.0, 1.0, 0.0)) {
        m_type = DatumAxisType::VertexAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_vertexIndex = vertexIndex;
        m_faceIndex = -1;
        m_edgeIndex = -1;
        m_axisDir = direction.normalized();
        m_offsetDir = m_axisDir;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (feature) {
            AddInput(feature);
            Geom::Point3D vPos;
            if (feature->GetVertexPosition(vertexIndex, vPos)) {
                m_axis = Geom::Axis3D(vPos, m_axisDir);
                m_lastValidOrigin = vPos;
            }
            Geom::Plane cornerPl;
            if (DatumPlaneNode::ComputeCornerVertexPlane(feature, vertexIndex, cornerPl)) {
                m_offsetDir = cornerPl.normal;
            }
        }
        MarkDirty();
    }

    void SetFaceReference(
        std::shared_ptr<DAGNode> feature,
        int faceIndex,
        Geom::Vector3D direction = Geom::Vector3D(0.0, 0.0, 0.0),
        double offset = 0.0
    ) {
        m_type = DatumAxisType::FaceAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_faceIndex = faceIndex;
        m_vertexIndex = -1;
        m_edgeIndex = -1;
        m_offset = offset;
        m_hasConflict = false;
        m_conflictMessage.clear();

        if (feature) {
            AddInput(feature);
            FaceInfo face;
            if (feature->GetFace(faceIndex, face)) {
                m_offsetDir = face.normal.normalized();
                if (direction.length_squared() > 1e-6) {
                    m_axisDir = direction.normalized();
                    m_faceUseNormalDir = (std::abs(m_axisDir.dot(m_offsetDir)) > 0.99);
                } else {
                    m_axisDir = m_offsetDir;
                    m_faceUseNormalDir = true;
                }
                m_axis = Geom::Axis3D(face.center, m_axisDir);
                m_lastValidOrigin = face.center;
            }
        }
        MarkDirty();
    }

    void SetEdgeReference(
        std::shared_ptr<DAGNode> feature,
        int edgeIndex,
        double offset = 0.0,
        bool alongEdge = true
    ) {
        m_type = DatumAxisType::EdgeAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_edgeIndex = edgeIndex;
        m_faceIndex = -1;
        m_vertexIndex = -1;
        m_offset = offset;
        m_edgeAlongDir = alongEdge;
        m_hasConflict = false;
        m_conflictMessage.clear();

        if (feature) {
            AddInput(feature);
            Geom::Plane edgePlane;
            if (DatumPlaneNode::ComputeCornerEdgePlane(feature, edgeIndex, edgePlane)) {
                m_offsetDir = edgePlane.normal;
                m_axisDir = alongEdge ? edgePlane.u_axis : edgePlane.normal;
                m_axis = Geom::Axis3D(edgePlane.origin, m_axisDir);
                m_lastValidOrigin = edgePlane.origin;
            }
        }
        MarkDirty();
    }

    void ConvertToFixed() {
        Geom::Axis3D eff = GetAxis();
        m_type = DatumAxisType::Fixed;
        m_axis = eff;
        m_lastValidOrigin = eff.origin;
        m_axisDir = eff.direction;
        m_offset = 0.0;
        m_parentFeature.reset();
        m_parentFeatureId = 0;
        m_vertexIndex = -1;
        m_faceIndex = -1;
        m_edgeIndex = -1;
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
        m_vertexIndex = -1;
        m_faceIndex = -1;
        m_edgeIndex = -1;
        m_hasConflict = false;
        m_conflictMessage.clear();
        MarkDirty();
    }

    void SetOffset(double dist) {
        m_offset = dist;
        MarkDirty();
    }

    [[nodiscard]] double GetOffset() const noexcept { return m_offset; }

    void SetOffsetDirection(Geom::Vector3D dir) {
        if (dir.length_squared() > 1e-6) {
            m_offsetDir = dir.normalized();
            MarkDirty();
        }
    }

    [[nodiscard]] Geom::Vector3D GetOffsetDirection() const noexcept { return m_offsetDir; }

    [[nodiscard]] Geom::Axis3D GetAxis() const noexcept {
        if (std::abs(m_offset) > 1e-9) {
            return Geom::Axis3D(m_axis.origin + m_offsetDir * m_offset, m_axis.direction);
        }
        return m_axis;
    }

    [[nodiscard]] DatumAxisType GetAxisType() const noexcept { return m_type; }
    [[nodiscard]] bool IsVertexAssociative() const noexcept { return m_type == DatumAxisType::VertexAssociative; }
    [[nodiscard]] bool IsFaceAssociative() const noexcept { return m_type == DatumAxisType::FaceAssociative; }
    [[nodiscard]] bool IsEdgeAssociative() const noexcept { return m_type == DatumAxisType::EdgeAssociative; }

    [[nodiscard]] bool IsSolidFeature() const noexcept override { return false; }
    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }
    void ClearConflict() noexcept { m_hasConflict = false; m_conflictMessage.clear(); }

    [[nodiscard]] int GetReferencedVertexIndex() const noexcept { return m_vertexIndex; }
    [[nodiscard]] int GetReferencedFaceIndex() const noexcept { return m_faceIndex; }
    [[nodiscard]] int GetReferencedEdgeIndex() const noexcept { return m_edgeIndex; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetReferencedFeature() const noexcept { return m_parentFeature.lock(); }
    [[nodiscard]] uint64_t GetReferencedFeatureId() const noexcept { return m_parentFeatureId; }

    void OnParentRemoved(uint64_t parentId) override {
        DAGNode::OnParentRemoved(parentId);
        if (m_parentFeatureId == parentId || (m_parentFeature.expired() && m_type != DatumAxisType::Fixed)) {
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

            Geom::Plane cornerPl;
            if (DatumPlaneNode::ComputeCornerVertexPlane(feature, m_vertexIndex, cornerPl)) {
                m_offsetDir = cornerPl.normal;
            }

            // Valid vertex position update: update axis origin
            m_hasConflict = false;
            m_conflictMessage.clear();
            m_axis = Geom::Axis3D(vPos, m_axisDir);
            m_lastValidOrigin = vPos;
        } else if (m_type == DatumAxisType::FaceAssociative) {
            if (m_hasConflict) {
                return;
            }
            auto feature = m_parentFeature.lock();
            if (!feature) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced parent feature was deleted from the model tree.";
                return;
            }

            FaceInfo face;
            if (!feature->GetFace(m_faceIndex, face) || face.normal.length_squared() < 1e-4) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced face #" + std::to_string(m_faceIndex) +
                                    " on feature '" + feature->GetName() + "' is missing or degenerate.";
                return;
            }

            m_hasConflict = false;
            m_conflictMessage.clear();
            m_offsetDir = face.normal.normalized();
            if (m_faceUseNormalDir) {
                m_axisDir = m_offsetDir;
            }
            m_axis = Geom::Axis3D(face.center, m_axisDir);
            m_lastValidOrigin = face.center;
        } else if (m_type == DatumAxisType::EdgeAssociative) {
            if (m_hasConflict) {
                return;
            }
            auto feature = m_parentFeature.lock();
            if (!feature) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced parent feature was deleted from the model tree.";
                return;
            }

            Geom::Plane edgePlane;
            if (!DatumPlaneNode::ComputeCornerEdgePlane(feature, m_edgeIndex, edgePlane)) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced corner edge #" + std::to_string(m_edgeIndex) +
                                    " on feature '" + feature->GetName() + "' is missing or degenerate.";
                return;
            }

            m_hasConflict = false;
            m_conflictMessage.clear();
            m_offsetDir = edgePlane.normal;
            m_axisDir = m_edgeAlongDir ? edgePlane.u_axis : edgePlane.normal;
            m_axis = Geom::Axis3D(edgePlane.origin, m_axisDir);
            m_lastValidOrigin = edgePlane.origin;
        }

        ClearDirty();
    }

private:
    Geom::Axis3D m_axis{Geom::Axis3D::Y()};
    Geom::Point3D m_lastValidOrigin{0.0, 0.0, 0.0};
    Geom::Vector3D m_axisDir{0.0, 1.0, 0.0};
    Geom::Vector3D m_offsetDir{0.0, 1.0, 0.0};
    double m_offset{0.0};
    bool m_faceUseNormalDir{true};
    bool m_edgeAlongDir{true};

    DatumAxisType m_type{DatumAxisType::Fixed};
    std::weak_ptr<DAGNode> m_parentFeature;
    uint64_t m_parentFeatureId{0};
    int m_vertexIndex{-1};
    int m_faceIndex{-1};
    int m_edgeIndex{-1};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

} // namespace Tessera


