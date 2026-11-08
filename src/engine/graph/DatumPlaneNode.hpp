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
    VertexAssociative = 2,
    EdgeAssociative = 3
};

class DatumPlaneNode : public DAGNode {
public:
    DatumPlaneNode(std::string name, Geom::Plane plane = Geom::Plane::XY(), double offset = 0.0)
        : DAGNode(std::move(name)), m_plane(plane), m_lastValidPlane(plane), m_offset(offset) {}

    DatumPlaneNode(std::string name, std::shared_ptr<DAGNode> feature, int faceIndex, Geom::Plane initialPlane, double offset = 0.0)
        : DAGNode(std::move(name)), 
          m_offset(offset),
          m_type(DatumPlaneType::FaceAssociative),
          m_parentFeature(feature),
          m_parentFeatureId(feature ? feature->GetId() : 0),
          m_faceIndex(faceIndex),
          m_plane(initialPlane),
          m_lastValidPlane(initialPlane) {}

    DatumPlaneNode(std::string name, std::shared_ptr<DAGNode> feature, int vertexIndex, Geom::Plane initialPlane, bool isVertex, double offset = 0.0)
        : DAGNode(std::move(name)),
          m_offset(offset),
          m_type(DatumPlaneType::VertexAssociative),
          m_parentFeature(feature),
          m_parentFeatureId(feature ? feature->GetId() : 0),
          m_vertexIndex(vertexIndex),
          m_plane(initialPlane),
          m_lastValidPlane(initialPlane) {
        (void)isVertex;
    }

    static std::shared_ptr<DatumPlaneNode> CreateFromEdge(
        std::string name,
        std::shared_ptr<DAGNode> feature,
        int edgeIndex,
        Geom::Plane initialPlane,
        double offset = 0.0
    ) {
        auto node = std::make_shared<DatumPlaneNode>(std::move(name), initialPlane, offset);
        node->SetEdgeReference(std::move(feature), edgeIndex, initialPlane);
        node->SetOffset(offset);
        return node;
    }

    static bool ComputeCornerEdgePlane(
        const std::shared_ptr<DAGNode>& feature,
        int edgeIndex,
        Geom::Plane& outPlane
    ) {
        if (!feature || edgeIndex < 0) return false;
        const auto& edges = feature->GetCachedEdges();
        const auto& verts = feature->GetCachedVertices();
        size_t e = static_cast<size_t>(edgeIndex) * 2;
        if (e + 1 >= edges.size()) return false;

        uint32_t i0 = edges[e];
        uint32_t i1 = edges[e + 1];
        if (i0 >= verts.size() || i1 >= verts.size()) return false;

        Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
        Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
        Geom::Point3D mid((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5, (p0.z + p1.z) * 0.5);

        Geom::Vector3D edgeVec = p1 - p0;
        double edgeLen = edgeVec.length();
        if (edgeLen < 1e-5) return false;
        Geom::Vector3D edgeDir = edgeVec / edgeLen;

        // Compute outward corner bisector normal from triangles bordering this edge segment
        const auto& indices = feature->GetCachedIndices();
        std::vector<Geom::Vector3D> adjNormals;
        for (size_t t = 0; t + 2 < indices.size(); t += 3) {
            uint32_t ta = indices[t + 0];
            uint32_t tb = indices[t + 1];
            uint32_t tc = indices[t + 2];
            if (ta >= verts.size() || tb >= verts.size() || tc >= verts.size()) continue;

            Geom::Point3D pts[3] = {
                Geom::Point3D(verts[ta].position[0], verts[ta].position[1], verts[ta].position[2]),
                Geom::Point3D(verts[tb].position[0], verts[tb].position[1], verts[tb].position[2]),
                Geom::Point3D(verts[tc].position[0], verts[tc].position[1], verts[tc].position[2])
            };

            int onEdgeCount = 0;
            for (int k = 0; k < 3; ++k) {
                double proj = (pts[k] - p0).dot(edgeDir);
                if (proj >= -0.02 && proj <= edgeLen + 0.02) {
                    Geom::Point3D closest = p0 + edgeDir * std::clamp(proj, 0.0, edgeLen);
                    if (pts[k].distance_to(closest) < 0.02) {
                        onEdgeCount++;
                    }
                }
            }

            if (onEdgeCount >= 2) {
                Geom::Vector3D n = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
                double nLen = n.length();
                if (nLen > 1e-6) {
                    n = n / nLen;
                    Geom::Vector3D vn(verts[ta].normal[0], verts[ta].normal[1], verts[ta].normal[2]);
                    if (vn.length_squared() > 1e-4 && n.dot(vn) < 0.0) {
                        n = -n;
                    }
                    bool duplicate = false;
                    for (const auto& existing : adjNormals) {
                        if (existing.dot(n) > 0.98) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate) {
                        adjNormals.push_back(n);
                    }
                }
            }
        }

        Geom::Vector3D sumN(0.0, 0.0, 0.0);
        for (const auto& an : adjNormals) {
            sumN = sumN + an;
        }
        // Orthogonalize against edgeDir so the corner plane contains the edge direction
        Geom::Vector3D perpN = sumN - edgeDir * sumN.dot(edgeDir);
        if (perpN.length_squared() < 1e-6) {
            Geom::Vector3D vn0(verts[i0].normal[0], verts[i0].normal[1], verts[i0].normal[2]);
            perpN = vn0 - edgeDir * vn0.dot(edgeDir);
        }
        if (perpN.length_squared() < 1e-6) {
            Geom::Plane fallback(mid, edgeDir);
            perpN = fallback.u_axis;
        } else {
            perpN = perpN.normalized();
        }

        outPlane = Geom::Plane(mid, perpN, edgeDir);
        return true;
    }

    static bool ComputeCornerVertexPlane(
        const std::shared_ptr<DAGNode>& feature,
        int vertexIndex,
        Geom::Plane& outPlane
    ) {
        if (!feature || vertexIndex < 0) return false;
        Geom::Point3D vPos;
        if (!feature->GetVertexPosition(vertexIndex, vPos)) return false;

        const auto& verts = feature->GetCachedVertices();
        const auto& indices = feature->GetCachedIndices();
        std::vector<Geom::Vector3D> adjNormals;
        for (size_t t = 0; t + 2 < indices.size(); t += 3) {
            uint32_t ta = indices[t + 0];
            uint32_t tb = indices[t + 1];
            uint32_t tc = indices[t + 2];
            if (ta >= verts.size() || tb >= verts.size() || tc >= verts.size()) continue;

            Geom::Point3D pa(verts[ta].position[0], verts[ta].position[1], verts[ta].position[2]);
            Geom::Point3D pb(verts[tb].position[0], verts[tb].position[1], verts[tb].position[2]);
            Geom::Point3D pc(verts[tc].position[0], verts[tc].position[1], verts[tc].position[2]);

            if (pa.distance_to(vPos) < 0.02 || pb.distance_to(vPos) < 0.02 || pc.distance_to(vPos) < 0.02) {
                Geom::Vector3D n = (pb - pa).cross(pc - pa);
                double nLen = n.length();
                if (nLen > 1e-6) {
                    n = n / nLen;
                    Geom::Vector3D vn(verts[ta].normal[0], verts[ta].normal[1], verts[ta].normal[2]);
                    if (vn.length_squared() > 1e-4 && n.dot(vn) < 0.0) {
                        n = -n;
                    }
                    bool duplicate = false;
                    for (const auto& existing : adjNormals) {
                        if (existing.dot(n) > 0.98) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate) {
                        adjNormals.push_back(n);
                    }
                }
            }
        }

        Geom::Vector3D sumN(0.0, 0.0, 0.0);
        for (const auto& an : adjNormals) {
            sumN = sumN + an;
        }
        if (sumN.length_squared() > 1e-6) {
            outPlane = Geom::Plane(vPos, sumN.normalized());
        } else {
            outPlane = Geom::Plane::XZ(vPos);
        }
        return true;
    }

    void SetFaceReference(std::shared_ptr<DAGNode> feature, int faceIndex, Geom::Plane initialPlane) {
        m_type = DatumPlaneType::FaceAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_faceIndex = faceIndex;
        m_vertexIndex = -1;
        m_edgeIndex = -1;
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
        m_edgeIndex = -1;
        m_plane = initialPlane;
        m_lastValidPlane = initialPlane;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (feature) {
            AddInput(feature);
        }
        MarkDirty();
    }

    void SetEdgeReference(std::shared_ptr<DAGNode> feature, int edgeIndex, Geom::Plane initialPlane) {
        m_type = DatumPlaneType::EdgeAssociative;
        m_parentFeature = feature;
        m_parentFeatureId = feature ? feature->GetId() : 0;
        m_edgeIndex = edgeIndex;
        m_faceIndex = -1;
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

    void ConvertToFixed() {
        m_type = DatumPlaneType::Fixed;
        m_plane = m_lastValidPlane;
        m_parentFeature.reset();
        m_parentFeatureId = 0;
        m_faceIndex = -1;
        m_vertexIndex = -1;
        m_edgeIndex = -1;
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
        m_edgeIndex = -1;
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
    [[nodiscard]] bool IsEdgeAssociative() const noexcept { return m_type == DatumPlaneType::EdgeAssociative; }

    [[nodiscard]] bool HasConflict() const noexcept override { return m_hasConflict; }
    [[nodiscard]] std::string GetConflictMessage() const noexcept override { return m_conflictMessage; }
    void ClearConflict() noexcept { m_hasConflict = false; m_conflictMessage.clear(); }

    [[nodiscard]] int GetReferencedFaceIndex() const noexcept { return m_faceIndex; }
    [[nodiscard]] int GetReferencedVertexIndex() const noexcept { return m_vertexIndex; }
    [[nodiscard]] int GetReferencedEdgeIndex() const noexcept { return m_edgeIndex; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetReferencedFeature() const noexcept { return m_parentFeature.lock(); }
    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_parentFeature.lock(); }
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
        } else if (m_type == DatumPlaneType::EdgeAssociative) {
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
            if (!ComputeCornerEdgePlane(feature, m_edgeIndex, edgePlane)) {
                m_hasConflict = true;
                m_conflictMessage = "Referenced corner edge #" + std::to_string(m_edgeIndex) +
                                    " on feature '" + feature->GetName() + "' is missing or degenerate.";
                return;
            }

            m_hasConflict = false;
            m_conflictMessage.clear();
            m_plane = edgePlane;
            m_lastValidPlane = edgePlane;
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
    int m_edgeIndex{-1};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

} // namespace Tessera


