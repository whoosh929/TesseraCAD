#pragma once

#include "Node.hpp"
#include "engine/geom/Plane.hpp"
#include <string>
#include <memory>
#include <iostream>

namespace Tessera {

enum class DatumPlanePreset : uint8_t {
    XY_Front = 0,
    XZ_Top   = 1,
    YZ_Right = 2,
    Custom   = 3
};

enum class DatumPlaneType : uint8_t {
    Fixed = 0,
    FaceAssociative = 1,
    VertexAssociative = 2
};

class DatumPlaneNode : public DAGNode {
public:
    DatumPlaneNode(std::string name, Geom::Plane plane = Geom::Plane::XY())
        : DAGNode(std::move(name)), m_plane(plane), m_lastValidPlane(plane) {}

    DatumPlaneNode(std::string name, std::shared_ptr<DAGNode> feature, int faceIndex, Geom::Plane initialPlane)
        : DAGNode(std::move(name)), 
          m_type(DatumPlaneType::FaceAssociative),
          m_parentFeature(feature),
          m_parentFeatureId(feature ? feature->GetId() : 0),
          m_faceIndex(faceIndex),
          m_plane(initialPlane),
          m_lastValidPlane(initialPlane) {}

    DatumPlaneNode(std::string name, std::shared_ptr<DAGNode> feature, int vertexIndex, Geom::Plane initialPlane, bool isVertex)
        : DAGNode(std::move(name)),
          m_type(DatumPlaneType::VertexAssociative),
          m_parentFeature(feature),
          m_parentFeatureId(feature ? feature->GetId() : 0),
          m_vertexIndex(vertexIndex),
          m_plane(initialPlane),
          m_lastValidPlane(initialPlane) {
        (void)isVertex;
    }

    void SetFaceReference(std::shared_ptr<DAGNode> feature, int faceIndex, Geom::Plane initialPlane) {
        m_type = DatumPlaneType::FaceAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_faceIndex = faceIndex;
        m_vertexIndex = -1;
        m_plane = initialPlane;
        m_lastValidPlane = initialPlane;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (feature) {
            AddInput(feature);
        }
        MarkDirty();
    }

    void SetVertexReference(std::shared_ptr<DAGNode> feature, int vertexIndex, Geom::Plane initialPlane) {
        m_type = DatumPlaneType::VertexAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_vertexIndex = vertexIndex;
        m_faceIndex = -1;
        m_plane = initialPlane;
        m_lastValidPlane = initialPlane;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (feature) {
            AddInput(feature);
        }
        MarkDirty();
    }

    void ConvertToFixed() {
        m_type = DatumPlaneType::Fixed;
        m_plane = m_lastValidPlane;
        m_parentFeature.reset();
        m_parentFeatureId = 0;
        m_faceIndex = -1;
        m_vertexIndex = -1;
        m_hasConflict = false;
        m_conflictMessage.clear();
        MarkDirty();
    }

    void SetPlane(Geom::Plane pl) {
        m_plane = pl;
        m_lastValidPlane = pl;
        m_type = DatumPlaneType::Fixed;
        m_parentFeature.reset();
        m_parentFeatureId = 0;
        m_faceIndex = -1;
        m_vertexIndex = -1;
        m_hasConflict = false;
        m_conflictMessage.clear();
        MarkDirty();
    }

    void SetOffset(double dist) {
        m_offset = dist;
        MarkDirty();
    }

    [[nodiscard]] Geom::Plane GetPlane() const noexcept {
        return m_plane.offset(m_offset);
    }

    [[nodiscard]] double GetOffset() const noexcept { return m_offset; }
    [[nodiscard]] DatumPlaneType GetPlaneType() const noexcept { return m_type; }
    [[nodiscard]] bool IsFaceAssociative() const noexcept { return m_type == DatumPlaneType::FaceAssociative; }
    [[nodiscard]] bool IsVertexAssociative() const noexcept { return m_type == DatumPlaneType::VertexAssociative; }

    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }
    void ClearConflict() noexcept { m_hasConflict = false; m_conflictMessage.clear(); }

    [[nodiscard]] int GetReferencedFaceIndex() const noexcept { return m_faceIndex; }
    [[nodiscard]] int GetReferencedVertexIndex() const noexcept { return m_vertexIndex; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetReferencedFeature() const noexcept { return m_parentFeature.lock(); }
    [[nodiscard]] uint64_t GetReferencedFeatureId() const noexcept { return m_parentFeatureId; }

    void OnParentRemoved(uint64_t parentId) override {
        DAGNode::OnParentRemoved(parentId);
        if (m_parentFeatureId == parentId || (m_parentFeature.expired() && m_type != DatumPlaneType::Fixed)) {
            m_parentFeature.reset();
            m_hasConflict = true;
            m_conflictMessage = "Referenced parent feature was deleted from the model tree.";
            MarkDirty();
        }
    }

    void Evaluate(RHI::RHIDevice*, AtomicMeshExchange&) override {
        if (m_type == DatumPlaneType::FaceAssociative) {
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
            if (!feature->GetFace(m_faceIndex, face) || !face.isPlanar) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced face #" + std::to_string(m_faceIndex) + 
                                    " on feature '" + feature->GetName() + "' is missing or degenerate.";
                return;
            }

            // Normal check: verify plane has not collapsed
            if (face.normal.length_squared() < 1e-4) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced face #" + std::to_string(m_faceIndex) + " normal is collapsed.";
                return;
            }

            // Valid associative update: synchronize plane coordinates with latest feature geometry
            m_hasConflict = false;
            m_conflictMessage.clear();
            m_plane = face.plane;
            m_lastValidPlane = face.plane;
        } else if (m_type == DatumPlaneType::VertexAssociative) {
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
                                    " on feature '" + feature->GetName() + "' is missing.";
                return;
            }

            m_hasConflict = false;
            m_conflictMessage.clear();
            m_plane.origin = vPos;
            m_lastValidPlane.origin = vPos;
        }

        ClearDirty();
    }

private:
    Geom::Plane m_plane{Geom::Plane::XY()};
    Geom::Plane m_lastValidPlane{Geom::Plane::XY()};
    double m_offset{0.0};

    DatumPlaneType m_type{DatumPlaneType::Fixed};
    std::weak_ptr<DAGNode> m_parentFeature;
    uint64_t m_parentFeatureId{0};
    int m_faceIndex{-1};
    int m_vertexIndex{-1};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

} // namespace Tessera


