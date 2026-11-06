#pragma once

#include "Topology.hpp"
#include "engine/sketch/SketchProfile.hpp"
#include "engine/sketch/ProfileTessellator.hpp"
#include <cmath>
#include <memory>

namespace Tessera::Modeling {

class ExtrudeOp {
public:
    struct Params {
        double start_depth{0.0};
        double end_depth{1.0};
        Geom::Vector3D direction{0.0, 1.0, 0.0}; // Defaults to sketch plane normal
        bool is_solid{true};
    };

    // Perform linear extrusion sweep
    [[nodiscard]] static std::shared_ptr<Body> Execute(
        const Sketch::SketchProfile& profile,
        const Geom::Plane& sketchPlane,
        const Params& params
    ) {
        auto body = std::make_shared<Body>("ExtrudeBody");
        auto shell = std::make_shared<Shell>();

        Geom::Vector3D dir = params.direction.normalized();
        if (dir.length_squared() < Geom::CAD_EPSILON) {
            dir = sketchPlane.normal;
        }

        double hStart = params.start_depth;
        double hEnd = params.end_depth;
        if (std::abs(hEnd - hStart) < Geom::CAD_EPSILON) {
            hEnd = hStart + 1.0;
        }

        Geom::Vector3D startOffset = dir * hStart;
        Geom::Vector3D endOffset = dir * hEnd;
        Geom::Vector3D sweepVec = endOffset - startOffset;

        bool generateSolid = profile.is_closed && params.is_solid;
        shell->is_closed = generateSolid;

        uint32_t faceIdCounter = 1;
        uint32_t vertexIdCounter = 1;
        uint32_t edgeIdCounter = 1;

        // Discretize outer wire and inner hole wires
        auto outerPts2D = profile.outer_wire.discretize();
        if (outerPts2D.size() < 2) {
            body->add_shell(shell);
            return body;
        }

        // =====================================================================
        // 1. Bottom & Top Cap Faces (Only for closed solid bodies)
        // =====================================================================
        if (generateSolid) {
            auto poly2D = Sketch::ProfileTessellator::Triangulate(profile);

            // A. Bottom Cap Face (Normal = -dir)
            auto bottomFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Plane);
            bottomFace->plane_geom = Geom::Plane(sketchPlane.origin + startOffset, -dir, sketchPlane.u_axis, -sketchPlane.v_axis);

            // Populate mesh
            bottomFace->mesh_vertices.reserve(poly2D.vertices.size());
            for (const auto& uv : poly2D.vertices) {
                Geom::Point3D pt3d = sketchPlane.to_world(uv) + startOffset;
                bottomFace->mesh_vertices.push_back({
                    {static_cast<float>(pt3d.x), static_cast<float>(pt3d.y), static_cast<float>(pt3d.z)},
                    {static_cast<float>(-dir.x), static_cast<float>(-dir.y), static_cast<float>(-dir.z)}
                });
            }
            // Reverse winding for bottom face so normals face outwards
            bottomFace->mesh_indices.reserve(poly2D.indices.size());
            for (size_t i = 0; i < poly2D.indices.size(); i += 3) {
                bottomFace->mesh_indices.push_back(poly2D.indices[i]);
                bottomFace->mesh_indices.push_back(poly2D.indices[i + 2]);
                bottomFace->mesh_indices.push_back(poly2D.indices[i + 1]);
            }
            shell->add_face(bottomFace);

            // B. Top Cap Face (Normal = +dir)
            auto topFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Plane);
            topFace->plane_geom = Geom::Plane(sketchPlane.origin + endOffset, dir, sketchPlane.u_axis, sketchPlane.v_axis);

            topFace->mesh_vertices.reserve(poly2D.vertices.size());
            for (const auto& uv : poly2D.vertices) {
                Geom::Point3D pt3d = sketchPlane.to_world(uv) + endOffset;
                topFace->mesh_vertices.push_back({
                    {static_cast<float>(pt3d.x), static_cast<float>(pt3d.y), static_cast<float>(pt3d.z)},
                    {static_cast<float>(dir.x), static_cast<float>(dir.y), static_cast<float>(dir.z)}
                });
            }
            topFace->mesh_indices = poly2D.indices;
            shell->add_face(topFace);
        }

        // =====================================================================
        // 2. Side Wall Faces (Generated for outer wire and all hole wires)
        // =====================================================================
        auto GenerateSideWalls = [&](const std::vector<Geom::Point2D>& wirePts, bool isHole) {
            size_t n = wirePts.size();
            size_t segCount = profile.is_closed ? n : (n - 1);

            for (size_t i = 0; i < segCount; ++i) {
                size_t next = (i + 1) % n;
                Geom::Point2D uv0 = wirePts[i];
                Geom::Point2D uv1 = wirePts[next];

                Geom::Point3D p0_bot = sketchPlane.to_world(uv0) + startOffset;
                Geom::Point3D p1_bot = sketchPlane.to_world(uv1) + startOffset;
                Geom::Point3D p1_top = p1_bot + sweepVec;
                Geom::Point3D p0_top = p0_bot + sweepVec;

                // Outward normal: segment direction cross sweep direction
                Geom::Vector3D segDir = (p1_bot - p0_bot).normalized();
                Geom::Vector3D faceNormal = segDir.cross(dir).normalized();
                if (isHole) {
                    faceNormal = -faceNormal;
                }

                auto sideFace = std::make_shared<Face>(faceIdCounter++, SurfaceGeometryType::Ruled);

                // Build 4 quad vertices
                sideFace->mesh_vertices = {
                    {{static_cast<float>(p0_bot.x), static_cast<float>(p0_bot.y), static_cast<float>(p0_bot.z)},
                     {static_cast<float>(faceNormal.x), static_cast<float>(faceNormal.y), static_cast<float>(faceNormal.z)}},
                    {{static_cast<float>(p1_bot.x), static_cast<float>(p1_bot.y), static_cast<float>(p1_bot.z)},
                     {static_cast<float>(faceNormal.x), static_cast<float>(faceNormal.y), static_cast<float>(faceNormal.z)}},
                    {{static_cast<float>(p1_top.x), static_cast<float>(p1_top.y), static_cast<float>(p1_top.z)},
                     {static_cast<float>(faceNormal.x), static_cast<float>(faceNormal.y), static_cast<float>(faceNormal.z)}},
                    {{static_cast<float>(p0_top.x), static_cast<float>(p0_top.y), static_cast<float>(p0_top.z)},
                     {static_cast<float>(faceNormal.x), static_cast<float>(faceNormal.y), static_cast<float>(faceNormal.z)}}
                };

                // Two triangles: (0, 1, 2) and (0, 2, 3)
                sideFace->mesh_indices = {0, 1, 2, 0, 2, 3};

                // Feature edges: bottom segment (0-1), top segment (3-2), vertical seams (0-3, 1-2)
                sideFace->feature_edge_indices = {0, 1, 3, 2, 0, 3, 1, 2};

                // Topological B-Rep Edges & Vertices
                auto v0 = std::make_shared<Vertex>(vertexIdCounter++, p0_bot);
                auto v1 = std::make_shared<Vertex>(vertexIdCounter++, p1_bot);
                auto v2 = std::make_shared<Vertex>(vertexIdCounter++, p1_top);
                auto v3 = std::make_shared<Vertex>(vertexIdCounter++, p0_top);

                auto eBot = std::make_shared<Edge>(edgeIdCounter++, v0, v1);
                auto eRight = std::make_shared<Edge>(edgeIdCounter++, v1, v2);
                auto eTop = std::make_shared<Edge>(edgeIdCounter++, v2, v3);
                auto eLeft = std::make_shared<Edge>(edgeIdCounter++, v3, v0);

                sideFace->outer_wire = Wire({eBot, eRight, eTop, eLeft}, true);

                shell->add_face(sideFace);
            }
        };

        // Outer boundary walls
        GenerateSideWalls(outerPts2D, false);

        // Hole boundary walls (if any)
        for (const auto& holeWire : profile.inner_holes) {
            auto holePts = holeWire.discretize();
            if (holePts.size() >= 3) {
                GenerateSideWalls(holePts, true);
            }
        }

        body->add_shell(shell);
        return body;
    }
};

} // namespace Tessera::Modeling

