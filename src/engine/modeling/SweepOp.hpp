#pragma once

#include "Topology.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/sketch/ProfileTessellator.hpp"
#include <cmath>
#include <memory>
#include <vector>

namespace Tessera::Modeling {

class SweepOp {
public:
    struct Params {
        std::vector<Geom::Point3D> path_points; // 3D path curve points
        bool is_solid{true};
    };

    /// Sweeps a 2D profile along a 3D polyline / spline path
    [[nodiscard]] static std::shared_ptr<Body> Execute(
        const Sketch::SketchProfile& profile,
        const Geom::Plane& sketchPlane,
        const Params& params
    ) {
        auto body = std::make_shared<Body>("SweepBody");
        auto shell = std::make_shared<Shell>();

        const auto& path = params.path_points;
        if (path.size() < 2) {
            body->add_shell(shell);
            return body;
        }

        bool generateSolid = profile.is_closed && params.is_solid;
        shell->is_closed = generateSolid;

        uint32_t faceIdCounter = 1;
        uint32_t vertexIdCounter = 1;
        uint32_t edgeIdCounter = 1;

        auto outerPts2D = profile.outer_wire.discretize();
        if (outerPts2D.size() < 2) {
            body->add_shell(shell);
            return body;
        }

        // =====================================================================
        // 1. Compute Frenet-Serret / Rotation-Minimizing Frames (RMF) along path
        // =====================================================================
        size_t numPathPts = path.size();
        std::vector<Geom::Vector3D> tangents(numPathPts);
        for (size_t k = 0; k < numPathPts; ++k) {
            if (k == 0) {
                tangents[k] = (path[1] - path[0]).normalized();
            } else if (k == numPathPts - 1) {
                tangents[k] = (path[numPathPts - 1] - path[numPathPts - 2]).normalized();
            } else {
                tangents[k] = ((path[k + 1] - path[k]).normalized() + (path[k] - path[k - 1]).normalized()).normalized();
            }
            if (tangents[k].length_squared() < Geom::CAD_EPSILON) {
                tangents[k] = Geom::Vector3D(0, 1, 0);
            }
        }

        // Base frame vectors at path[0] aligned with sketchPlane
        Geom::Vector3D u0 = sketchPlane.u_axis;
        Geom::Vector3D v0 = sketchPlane.v_axis;

        // Orthogonalize u0 and v0 with tangents[0]
        u0 = (u0 - tangents[0] * u0.dot(tangents[0]));
        if (u0.length_squared() < 0.01) {
            u0 = (v0 - tangents[0] * v0.dot(tangents[0]));
            if (u0.length_squared() < 0.01) {
                Geom::Vector3D ref = (std::abs(tangents[0].y) > 0.9) ? Geom::Vector3D(1, 0, 0) : Geom::Vector3D(0, 1, 0);
                u0 = tangents[0].cross(ref);
            }
        }
        u0 = u0.normalized();

        // If the path tangent opposes the sketchPlane normal, flip u0 so rotation preserves profile orientation (no mirroring)
        if (sketchPlane.normal.dot(tangents[0]) < -0.1) {
            u0 = -u0;
        }

        v0 = tangents[0].cross(u0).normalized();

        std::vector<Geom::Vector3D> uFrames(numPathPts);
        std::vector<Geom::Vector3D> vFrames(numPathPts);
        uFrames[0] = u0;
        vFrames[0] = v0;

        // Determine 2D reference attachment point for profile:
        // If path[0] connects directly to the profile sketch plane, use projected point.
        // Otherwise use the profile polygon's centroid so the cross-section is centered on the line.
        Geom::Point2D projPath = sketchPlane.to_local(path[0]);
        Geom::Point3D worldProj = sketchPlane.to_world(projPath);
        Geom::Point2D attachPt(0.0, 0.0);

        double sumU = 0.0, sumV = 0.0;
        for (const auto& pt : outerPts2D) {
            sumU += pt.u;
            sumV += pt.v;
        }
        Geom::Point2D centroid(sumU / outerPts2D.size(), sumV / outerPts2D.size());

        if ((path[0] - worldProj).length() < 0.05) {
            attachPt = projPath;
        } else {
            attachPt = centroid;
        }

        // Parallel transport frames along the path (Double Reflection RMF)
        for (size_t k = 0; k + 1 < numPathPts; ++k) {
            Geom::Vector3D v1 = path[k + 1] - path[k];
            double c1 = v1.dot(v1);
            if (c1 < Geom::CAD_EPSILON) {
                uFrames[k + 1] = uFrames[k];
                vFrames[k + 1] = vFrames[k];
                continue;
            }
            Geom::Vector3D rL = uFrames[k] - v1 * (2.0 / c1 * v1.dot(uFrames[k]));
            Geom::Vector3D tL = tangents[k] - v1 * (2.0 / c1 * v1.dot(tangents[k]));
            Geom::Vector3D v2 = tangents[k + 1] - tL;
            double c2 = v2.dot(v2);
            if (c2 < Geom::CAD_EPSILON) {
                uFrames[k + 1] = rL;
            } else {
                uFrames[k + 1] = rL - v2 * (2.0 / c2 * v2.dot(rL));
            }
            uFrames[k + 1] = (uFrames[k + 1] - tangents[k + 1] * tangents[k + 1].dot(uFrames[k + 1])).normalized();
            vFrames[k + 1] = tangents[k + 1].cross(uFrames[k + 1]).normalized();
        }

        // =====================================================================
        // 2. Start & End End-Cap Faces (for solid sweep)
        // =====================================================================
        if (generateSolid) {
            auto poly2D = Sketch::ProfileTessellator::Triangulate(profile);

            // A. Start Cap Face at path[0] (Normal points against path: -tangents[0])
            auto startFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Plane);
            startFace->mesh_vertices.reserve(poly2D.vertices.size());
            Geom::Vector3D startNormal = -tangents[0];
            for (const auto& uv : poly2D.vertices) {
                Geom::Point3D pt3d = path[0] + uFrames[0] * (uv.u - attachPt.u) + vFrames[0] * (uv.v - attachPt.v);
                startFace->mesh_vertices.push_back({
                    {static_cast<float>(pt3d.x), static_cast<float>(pt3d.y), static_cast<float>(pt3d.z)},
                    {static_cast<float>(startNormal.x), static_cast<float>(startNormal.y), static_cast<float>(startNormal.z)}
                });
            }
            // Reverse winding for start cap so normal faces outwards
            startFace->mesh_indices.reserve(poly2D.indices.size());
            for (size_t i = 0; i < poly2D.indices.size(); i += 3) {
                startFace->mesh_indices.push_back(poly2D.indices[i]);
                startFace->mesh_indices.push_back(poly2D.indices[i + 2]);
                startFace->mesh_indices.push_back(poly2D.indices[i + 1]);
            }
            shell->add_face(startFace);

            // B. End Cap Face at path[numPathPts - 1] (Normal points along path: +tangents[numPathPts - 1])
            size_t lastIdx = numPathPts - 1;
            auto endFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Plane);
            endFace->mesh_vertices.reserve(poly2D.vertices.size());
            Geom::Vector3D endNormal = tangents[lastIdx];
            for (const auto& uv : poly2D.vertices) {
                Geom::Point3D pt3d = path[lastIdx] + uFrames[lastIdx] * (uv.u - attachPt.u) + vFrames[lastIdx] * (uv.v - attachPt.v);
                endFace->mesh_vertices.push_back({
                    {static_cast<float>(pt3d.x), static_cast<float>(pt3d.y), static_cast<float>(pt3d.z)},
                    {static_cast<float>(endNormal.x), static_cast<float>(endNormal.y), static_cast<float>(endNormal.z)}
                });
            }
            endFace->mesh_indices = poly2D.indices;
            shell->add_face(endFace);
        }

        // =====================================================================
        // 3. Side Swept Faces
        // =====================================================================
        auto GenerateSweptWalls = [&](const std::vector<Geom::Point2D>& wirePts, bool isHole) {
            size_t n = wirePts.size();
            if (n < 2) return;
            size_t segCount = profile.is_closed ? n : (n - 1);

            // Compute 3D ring points along path: ringPoints[k][i]
            std::vector<std::vector<Geom::Point3D>> ringPoints(numPathPts, std::vector<Geom::Point3D>(n));
            for (size_t k = 0; k < numPathPts; ++k) {
                for (size_t i = 0; i < n; ++i) {
                    ringPoints[k][i] = path[k] + uFrames[k] * (wirePts[i].u - attachPt.u) + vFrames[k] * (wirePts[i].v - attachPt.v);
                }
            }

            for (size_t i = 0; i < segCount; ++i) {
                size_t nextI = (i + 1) % n;
                auto sideFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Ruled);

                for (size_t k = 0; k + 1 < numPathPts; ++k) {
                    Geom::Point3D p0 = ringPoints[k][i];
                    Geom::Point3D p1 = ringPoints[k][nextI];
                    Geom::Point3D p2 = ringPoints[k + 1][nextI];
                    Geom::Point3D p3 = ringPoints[k + 1][i];

                    Geom::Vector3D segDir = (p1 - p0).normalized();
                    Geom::Vector3D sweepDir = (p3 - p0).normalized();
                    Geom::Vector3D faceNorm = segDir.cross(sweepDir).normalized();
                    if (isHole) {
                        faceNorm = -faceNorm;
                    }

                    uint32_t baseIdx = static_cast<uint32_t>(sideFace->mesh_vertices.size());
                    sideFace->mesh_vertices.push_back({
                        {static_cast<float>(p0.x), static_cast<float>(p0.y), static_cast<float>(p0.z)},
                        {static_cast<float>(faceNorm.x), static_cast<float>(faceNorm.y), static_cast<float>(faceNorm.z)}
                    });
                    sideFace->mesh_vertices.push_back({
                        {static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z)},
                        {static_cast<float>(faceNorm.x), static_cast<float>(faceNorm.y), static_cast<float>(faceNorm.z)}
                    });
                    sideFace->mesh_vertices.push_back({
                        {static_cast<float>(p2.x), static_cast<float>(p2.y), static_cast<float>(p2.z)},
                        {static_cast<float>(faceNorm.x), static_cast<float>(faceNorm.y), static_cast<float>(faceNorm.z)}
                    });
                    sideFace->mesh_vertices.push_back({
                        {static_cast<float>(p3.x), static_cast<float>(p3.y), static_cast<float>(p3.z)},
                        {static_cast<float>(faceNorm.x), static_cast<float>(faceNorm.y), static_cast<float>(faceNorm.z)}
                    });

                    sideFace->mesh_indices.push_back(baseIdx + 0);
                    sideFace->mesh_indices.push_back(baseIdx + 1);
                    sideFace->mesh_indices.push_back(baseIdx + 2);
                    sideFace->mesh_indices.push_back(baseIdx + 0);
                    sideFace->mesh_indices.push_back(baseIdx + 2);
                    sideFace->mesh_indices.push_back(baseIdx + 3);

                    // Feature edges on boundaries
                    if (k == 0) {
                        sideFace->feature_edge_indices.push_back(baseIdx + 0);
                        sideFace->feature_edge_indices.push_back(baseIdx + 1);
                    }
                    if (k + 2 == numPathPts) {
                        sideFace->feature_edge_indices.push_back(baseIdx + 3);
                        sideFace->feature_edge_indices.push_back(baseIdx + 2);
                    }
                    sideFace->feature_edge_indices.push_back(baseIdx + 0);
                    sideFace->feature_edge_indices.push_back(baseIdx + 3);
                    sideFace->feature_edge_indices.push_back(baseIdx + 1);
                    sideFace->feature_edge_indices.push_back(baseIdx + 2);
                }

                shell->add_face(sideFace);
            }
        };

        GenerateSweptWalls(outerPts2D, false);

        for (const auto& holeWire : profile.inner_holes) {
            auto holePts2D = holeWire.discretize();
            if (holePts2D.size() >= 2) {
                GenerateSweptWalls(holePts2D, true);
            }
        }

        body->add_shell(shell);
        return body;
    }
};

} // namespace Tessera::Modeling
