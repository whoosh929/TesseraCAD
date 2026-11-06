#pragma once

#include "Topology.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/sketch/ProfileTessellator.hpp"
#include <cmath>
#include <memory>

namespace Tessera::Modeling {

class RevolveOp {
public:
    struct Params {
        Geom::Axis3D axis{Geom::Point3D(0, 0, 0), Geom::Vector3D(0, 1, 0)};
        double angle_radians{Geom::TWO_PI};
        int angular_segments{32};
        bool is_solid{true};
    };

    // Perform rotational sweep around an axis
    [[nodiscard]] static std::shared_ptr<Body> Execute(
        const Sketch::SketchProfile& profile,
        const Geom::Plane& sketchPlane,
        const Params& params
    ) {
        auto body = std::make_shared<Body>("RevolveBody");
        auto shell = std::make_shared<Shell>();

        double sweepAngle = std::clamp(params.angle_radians, 0.01, Geom::TWO_PI);
        bool isFullRevolve = (std::abs(sweepAngle - Geom::TWO_PI) < 1e-4);
        int numSlices = std::max(4, params.angular_segments);

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
        // 1. Angular Caps for Partial Revolution (if solid)
        // =====================================================================
        if (generateSolid && !isFullRevolve) {
            auto poly2D = Sketch::ProfileTessellator::Triangulate(profile);

            // A. Start Cap Face (at theta = 0)
            // Normal points against revolution direction: -tangent_axis
            auto startFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Plane);
            startFace->mesh_vertices.reserve(poly2D.vertices.size());

            for (const auto& uv : poly2D.vertices) {
                Geom::Point3D pt3d = sketchPlane.to_world(uv);
                Geom::Vector3D radVec = pt3d - params.axis.project_point(pt3d);
                Geom::Vector3D rotTan = params.axis.direction.cross(radVec).normalized();

                startFace->mesh_vertices.push_back({
                    {static_cast<float>(pt3d.x), static_cast<float>(pt3d.y), static_cast<float>(pt3d.z)},
                    {static_cast<float>(-rotTan.x), static_cast<float>(-rotTan.y), static_cast<float>(-rotTan.z)}
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

            // B. End Cap Face (at theta = sweepAngle)
            auto endFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Plane);
            endFace->mesh_vertices.reserve(poly2D.vertices.size());

            auto rotEnd = Geom::Transform3D::RotationAroundAxis(params.axis.origin, params.axis.direction, sweepAngle);
            for (const auto& uv : poly2D.vertices) {
                Geom::Point3D pt3d = rotEnd.transform_point(sketchPlane.to_world(uv));
                Geom::Vector3D radVec = pt3d - params.axis.project_point(pt3d);
                Geom::Vector3D rotTan = params.axis.direction.cross(radVec).normalized();

                endFace->mesh_vertices.push_back({
                    {static_cast<float>(pt3d.x), static_cast<float>(pt3d.y), static_cast<float>(pt3d.z)},
                    {static_cast<float>(rotTan.x), static_cast<float>(rotTan.y), static_cast<float>(rotTan.z)}
                });
            }
            endFace->mesh_indices = poly2D.indices;
            shell->add_face(endFace);
        }

        // =====================================================================
        // 2. Revolved Surface Walls
        // =====================================================================
        auto GenerateRevolvedWalls = [&](const std::vector<Geom::Point2D>& wirePts, bool isHole) {
            size_t n = wirePts.size();
            size_t segCount = profile.is_closed ? n : (n - 1);

            // Pre-calculate 3D points at each angular slice
            // ringPoints[k][i] = 3D point of vertex i at angle slice k
            std::vector<std::vector<Geom::Point3D>> ringPoints(numSlices + 1, std::vector<Geom::Point3D>(n));
            std::vector<std::vector<Geom::Vector3D>> ringNormals(numSlices + 1, std::vector<Geom::Vector3D>(n));

            for (int k = 0; k <= numSlices; ++k) {
                double theta = (static_cast<double>(k) / static_cast<double>(numSlices)) * sweepAngle;
                auto rot = Geom::Transform3D::RotationAroundAxis(params.axis.origin, params.axis.direction, theta);

                for (size_t i = 0; i < n; ++i) {
                    Geom::Point3D basePt = sketchPlane.to_world(wirePts[i]);
                    Geom::Point3D rotPt = rot.transform_point(basePt);
                    ringPoints[k][i] = rotPt;

                    // Approximate vertex normal
                    Geom::Point3D projAxis = params.axis.project_point(rotPt);
                    Geom::Vector3D radDir = (rotPt - projAxis).normalized();
                    ringNormals[k][i] = isHole ? -radDir : radDir;
                }
            }

            // Create ruled quad faces between slices
            for (size_t i = 0; i < segCount; ++i) {
                size_t nextI = (i + 1) % n;

                for (int k = 0; k < numSlices; ++k) {
                    int nextK = k + 1;

                    Geom::Point3D p00 = ringPoints[k][i];
                    Geom::Point3D p10 = ringPoints[k][nextI];
                    Geom::Point3D p11 = ringPoints[nextK][nextI];
                    Geom::Point3D p01 = ringPoints[nextK][i];

                    Geom::Vector3D n00 = ringNormals[k][i];
                    Geom::Vector3D n10 = ringNormals[k][nextI];
                    Geom::Vector3D n11 = ringNormals[nextK][nextI];
                    Geom::Vector3D n01 = ringNormals[nextK][i];

                    auto face = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Ruled);

                    face->mesh_vertices = {
                        {{static_cast<float>(p00.x), static_cast<float>(p00.y), static_cast<float>(p00.z)},
                         {static_cast<float>(n00.x), static_cast<float>(n00.y), static_cast<float>(n00.z)}},
                        {{static_cast<float>(p10.x), static_cast<float>(p10.y), static_cast<float>(p10.z)},
                         {static_cast<float>(n10.x), static_cast<float>(n10.y), static_cast<float>(n10.z)}},
                        {{static_cast<float>(p11.x), static_cast<float>(p11.y), static_cast<float>(p11.z)},
                         {static_cast<float>(n11.x), static_cast<float>(n11.y), static_cast<float>(n11.z)}},
                        {{static_cast<float>(p01.x), static_cast<float>(p01.y), static_cast<float>(p01.z)},
                         {static_cast<float>(n01.x), static_cast<float>(n01.y), static_cast<float>(n01.z)}}
                    };

                    face->mesh_indices = {0, 1, 2, 0, 2, 3};
                    face->feature_edge_indices = {0, 1, 3, 2, 0, 3, 1, 2};

                    // Topological Vertices & Edges
                    auto v0 = std::make_shared<Vertex>(vertexIdCounter++, p00);
                    auto v1 = std::make_shared<Vertex>(vertexIdCounter++, p10);
                    auto v2 = std::make_shared<Vertex>(vertexIdCounter++, p11);
                    auto v3 = std::make_shared<Vertex>(vertexIdCounter++, p01);

                    auto e0 = std::make_shared<Edge>(edgeIdCounter++, v0, v1);
                    auto e1 = std::make_shared<Edge>(edgeIdCounter++, v1, v2);
                    auto e2 = std::make_shared<Edge>(edgeIdCounter++, v2, v3);
                    auto e3 = std::make_shared<Edge>(edgeIdCounter++, v3, v0);

                    face->outer_wire = Wire({e0, e1, e2, e3}, true);
                    shell->add_face(face);
                }
            }
        };

        GenerateRevolvedWalls(outerPts2D, false);

        for (const auto& holeWire : profile.inner_holes) {
            auto holePts = holeWire.discretize();
            if (holePts.size() >= 3) {
                GenerateRevolvedWalls(holePts, true);
            }
        }

        body->add_shell(shell);
        return body;
    }
};

} // namespace Tessera::Modeling
