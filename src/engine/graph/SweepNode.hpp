#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "engine/modeling/SweepOp.hpp"
#include "engine/modeling/Tessellator.hpp"
#include "engine/modeling/MeshBoolean3D.hpp"
#include <memory>
#include <vector>
#include <string>
#include <iostream>

namespace Tessera {

enum class SweepOperation : uint8_t {
    Boss = 0,
    Cut  = 1
};

class SweepNode : public DAGNode {
public:
    SweepNode(
        std::string name,
        std::shared_ptr<SketchNode> profileSketch,
        std::shared_ptr<SketchNode> pathSketch,
        SweepOperation op = SweepOperation::Boss,
        std::shared_ptr<DAGNode> target = nullptr
    ) : DAGNode(std::move(name)),
        m_profileSketch(profileSketch),
        m_pathSketch(pathSketch),
        m_operation(op),
        m_targetNode(target),
        m_targetNodeId(target ? target->GetId() : 0) {
        if (m_profileSketch && m_pathSketch) {
            if (m_profileSketch->IsPathSketch() && m_pathSketch->IsEnclosed()) {
                std::swap(m_profileSketch, m_pathSketch);
            }
        }
        if (m_profileSketch) {
            AddInput(m_profileSketch);
            m_profileSketch->SetConsumed(true);
            m_position[0] = m_profileSketch->GetPosition()[0];
            m_position[1] = m_profileSketch->GetPosition()[1];
            m_position[2] = m_profileSketch->GetPosition()[2];
        }
        if (m_pathSketch && m_pathSketch != m_profileSketch) {
            AddInput(m_pathSketch);
            m_pathSketch->SetConsumed(true);
        }
        if (m_targetNode && m_operation == SweepOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    SweepNode(
        std::string name,
        std::shared_ptr<SketchNode> profileSketch,
        std::vector<Geom::Point3D> customPath,
        SweepOperation op = SweepOperation::Boss,
        std::shared_ptr<DAGNode> target = nullptr
    ) : DAGNode(std::move(name)),
        m_profileSketch(profileSketch),
        m_pathPoints(std::move(customPath)),
        m_operation(op),
        m_targetNode(target),
        m_targetNodeId(target ? target->GetId() : 0) {
        if (m_profileSketch && m_pathSketch) {
            if (m_profileSketch->IsPathSketch() && m_pathSketch->IsEnclosed()) {
                std::swap(m_profileSketch, m_pathSketch);
            }
        }
        if (m_profileSketch) {
            AddInput(m_profileSketch);
            m_profileSketch->SetConsumed(true);
            m_position[0] = m_profileSketch->GetPosition()[0];
            m_position[1] = m_profileSketch->GetPosition()[1];
            m_position[2] = m_profileSketch->GetPosition()[2];
        }
        if (m_targetNode && m_operation == SweepOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    ~SweepNode() override {
        if (m_targetNode && m_operation == SweepOperation::Cut) {
            m_targetNode->SetConsumed(false);
        }
    }

    [[nodiscard]] SweepOperation GetOperation() const noexcept { return m_operation; }
    void SetOperation(SweepOperation op) {
        if (m_operation != op) {
            m_operation = op;
            if (m_operation == SweepOperation::Boss && m_targetNode) {
                m_targetNode->SetConsumed(false);
            } else if (m_operation == SweepOperation::Cut && m_targetNode) {
                m_targetNode->SetConsumed(true);
            }
            MarkDirty();
        }
    }
    [[nodiscard]] bool IsCut() const noexcept { return m_operation == SweepOperation::Cut; }

    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetFeature() const noexcept { return m_targetNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetTargetSolid() const noexcept override {
        return (m_operation == SweepOperation::Cut) ? m_targetNode : nullptr;
    }
    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override {
        return (m_operation == SweepOperation::Cut) ? m_targetNode : nullptr;
    }
    [[nodiscard]] bool HasParentFeature() const noexcept override {
        return (m_operation == SweepOperation::Cut) && (m_targetNode != nullptr);
    }

    void SetTargetFeature(std::shared_ptr<DAGNode> target) {
        if (m_targetNode && m_targetNode != target && m_operation == SweepOperation::Cut) {
            m_targetNode->SetConsumed(false);
            m_targetNode->MarkDirty();
        }
        m_targetNode = target;
        m_targetNodeId = target ? target->GetId() : 0;
        m_hasConflict = false;
        m_conflictMessage.clear();
        if (m_targetNode && m_operation == SweepOperation::Cut) {
            AddInput(m_targetNode);
            m_targetNode->SetConsumed(true);
        }
    }

    [[nodiscard]] std::shared_ptr<SketchNode> GetProfileSketch() const noexcept { return m_profileSketch; }
    [[nodiscard]] std::shared_ptr<SketchNode> GetPathSketch() const noexcept { return m_pathSketch; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetChildSketch() const noexcept override { return m_profileSketch; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetSecondaryChildSketch() const noexcept override { return m_pathSketch; }

    void SetProfileSketch(std::shared_ptr<SketchNode> sketch) {
        m_profileSketch = sketch;
        if (m_profileSketch) {
            AddInput(m_profileSketch);
            m_profileSketch->SetConsumed(true);
        }
        MarkDirty();
    }

    void SetPathSketch(std::shared_ptr<SketchNode> sketch) {
        m_pathSketch = sketch;
        if (m_pathSketch) {
            AddInput(m_pathSketch);
            m_pathSketch->SetConsumed(true);
        }
        MarkDirty();
    }

    void SetCustomPath(std::vector<Geom::Point3D> path) {
        m_pathPoints = std::move(path);
        MarkDirty();
    }
    [[nodiscard]] const std::vector<Geom::Point3D>& GetCustomPath() const noexcept { return m_pathPoints; }

    void DetachChildSketch() noexcept override {
        if (m_profileSketch) {
            m_profileSketch->SetParentFeature(nullptr);
            m_profileSketch->SetConsumed(false);
            m_profileSketch->SetVisible(true);
            m_profileSketch->MarkDirty();
            m_profileSketch = nullptr;
        }
        if (m_pathSketch) {
            m_pathSketch->SetConsumed(false);
            m_pathSketch->SetVisible(true);
            m_pathSketch->MarkDirty();
            m_pathSketch = nullptr;
        }
    }

    void Evaluate(RHI::RHIDevice*, AtomicMeshExchange&) override {
        std::cout << "[SweepNode::Evaluate] Starting Sweep operation for '" << m_name << "'\n";

        // Auto-detect and swap if profile is a single line and path is enclosed
        if (m_profileSketch && m_pathSketch) {
            if (m_profileSketch->IsPathSketch() && m_pathSketch->IsEnclosed()) {
                std::swap(m_profileSketch, m_pathSketch);
            }
        }

        if (!m_profileSketch) {
            m_hasConflict = true;
            m_conflictMessage = "Sweep requires an enclosed 2D profile sketch";
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            m_facesDirty = true;
            m_cachedFaces.clear();
            ClearDirty();
            return;
        }

        if (!m_pathSketch && m_pathPoints.size() < 2) {
            m_hasConflict = true;
            m_conflictMessage = "Sweep requires a single line 2D sketch as guide path";
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            m_facesDirty = true;
            m_cachedFaces.clear();
            ClearDirty();
            return;
        }

        // Build 3D path points
        std::vector<Geom::Point3D> pathPts;
        if (m_pathSketch) {
            const auto& pathShapes = m_pathSketch->GetShapes();
            Geom::Plane pathPlane = m_pathSketch->GetEffectivePlane();
            if (!pathShapes.empty() && !pathShapes[0].empty()) {
                pathPts.reserve(pathShapes[0].size());
                for (const auto& pt2 : pathShapes[0]) {
                    pathPts.push_back(pathPlane.to_world(Geom::Point2D(pt2[0], pt2[1])));
                }
            } else if (!m_pathSketch->GetProfile().empty()) {
                for (const auto& pt2 : m_pathSketch->GetProfile()) {
                    pathPts.push_back(pathPlane.to_world(Geom::Point2D(pt2[0], pt2[1])));
                }
            }
        }
        if (pathPts.size() < 2) {
            pathPts = m_pathPoints;
        }

        if (pathPts.size() < 2) {
            m_hasConflict = true;
            m_conflictMessage = "Guide path sketch must contain at least 2 points (a line segment)";
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            m_facesDirty = true;
            m_cachedFaces.clear();
            ClearDirty();
            return;
        }

        Geom::Plane sketchPlane = m_profileSketch->GetEffectivePlane();
        const auto& shapes = m_profileSketch->GetShapes();

        // Build SketchProfile
        Sketch::SketchProfile sketchProf;
        sketchProf.is_closed = true;

        if (!shapes.empty() && !shapes[0].empty()) {
            std::vector<std::shared_ptr<Geom::Curve2D>> curves;
            const auto& pts = shapes[0];
            size_t n = pts.size();
            for (size_t i = 0; i < n; ++i) {
                size_t nextI = (i + 1) % n;
                curves.push_back(std::make_shared<Geom::LineSegment2D>(
                    Geom::Point2D(pts[i][0], pts[i][1]),
                    Geom::Point2D(pts[nextI][0], pts[nextI][1])
                ));
            }
            sketchProf.outer_wire = Sketch::Wire2D(std::move(curves));

            // Inner hole loops
            for (size_t s = 1; s < shapes.size(); ++s) {
                const auto& hPts = shapes[s];
                if (hPts.size() < 3) continue;
                std::vector<std::shared_ptr<Geom::Curve2D>> holeCurves;
                size_t hn = hPts.size();
                for (size_t i = 0; i < hn; ++i) {
                    size_t nextI = (i + 1) % hn;
                    holeCurves.push_back(std::make_shared<Geom::LineSegment2D>(
                        Geom::Point2D(hPts[i][0], hPts[i][1]),
                        Geom::Point2D(hPts[nextI][0], hPts[nextI][1])
                    ));
                }
                sketchProf.inner_holes.push_back(Sketch::Wire2D(std::move(holeCurves)));
            }
        } else if (!m_profileSketch->GetProfile().empty()) {
            const auto& pts = m_profileSketch->GetProfile();
            std::vector<std::shared_ptr<Geom::Curve2D>> curves;
            size_t n = pts.size();
            for (size_t i = 0; i < n; ++i) {
                size_t nextI = (i + 1) % n;
                curves.push_back(std::make_shared<Geom::LineSegment2D>(
                    Geom::Point2D(pts[i][0], pts[i][1]),
                    Geom::Point2D(pts[nextI][0], pts[nextI][1])
                ));
            }
            sketchProf.outer_wire = Sketch::Wire2D(std::move(curves));
        }

        Modeling::SweepOp::Params params;
        params.path_points = pathPts;
        params.is_solid = true;

        auto body = Modeling::SweepOp::Execute(sketchProf, sketchPlane, params);
        auto tess = Modeling::Tessellator::Tessellate(*body);

        if (m_operation == SweepOperation::Cut && m_targetNode) {
            std::vector<CADVertex> sweptVerts;
            std::vector<uint32_t> sweptIndices = tess.indices;
            sweptVerts.reserve(tess.vertices.size());
            for (const auto& v : tess.vertices) {
                sweptVerts.push_back({
                    {v.position[0], v.position[1], v.position[2]},
                    {v.normal[0], v.normal[1], v.normal[2]}
                });
            }

            const auto& targetVerts = m_targetNode->GetCachedVertices();
            const auto& targetIndices = m_targetNode->GetCachedIndices();

            bool diffOk = Modeling::MeshBoolean3D::Difference(
                targetVerts, targetIndices,
                sweptVerts, sweptIndices,
                m_cachedVertices, m_cachedIndices
            );
            if (!diffOk || m_cachedVertices.empty() || m_cachedIndices.empty()) {
                m_cachedVertices = targetVerts;
                m_cachedIndices = targetIndices;
            }
            m_cachedEdges = TessellationUtils::ExtractFeatureEdges(m_cachedVertices, m_cachedIndices, 30.0f);
        } else {
            m_cachedVertices = std::move(tess.vertices);
            m_cachedIndices = std::move(tess.indices);
            m_cachedEdges = std::move(tess.featureEdgeIndices);
        }

        m_hasConflict = false;
        m_conflictMessage.clear();
        m_facesDirty = true;
        ComputeFaces();
        ClearDirty();
        std::cout << "[SweepNode::Evaluate] Finished: " << m_cachedVertices.size() 
                  << " vertices, " << (m_cachedIndices.size() / 3) << " triangles\n";
    }

private:
    std::shared_ptr<SketchNode> m_profileSketch;
    std::shared_ptr<SketchNode> m_pathSketch;
    std::vector<Geom::Point3D> m_pathPoints;
    SweepOperation m_operation{SweepOperation::Boss};
    std::shared_ptr<DAGNode> m_targetNode;
    uint64_t m_targetNodeId{0};
    bool m_hasConflict{false};
    std::string m_conflictMessage;
};

class CutSweepNode : public SweepNode {
public:
    CutSweepNode(
        std::string name,
        std::shared_ptr<SketchNode> profileSketch,
        std::shared_ptr<SketchNode> pathSketch,
        std::shared_ptr<DAGNode> target = nullptr
    ) : SweepNode(std::move(name), profileSketch, pathSketch, SweepOperation::Cut, target) {}
};

} // namespace Tessera
