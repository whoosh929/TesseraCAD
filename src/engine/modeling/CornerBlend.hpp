#pragma once

#include "engine/modeling/Topology.hpp"
#include "engine/modeling/BlendGeometry.hpp"
#include <vector>
#include <memory>
#include <cmath>

namespace Tessera::Modeling {

class CornerBlend {
public:
    /**
     * @brief Constructs a spherical triangle patch at a 3-edge corner vertex
     *        tangent to all 3 incoming rolling-ball cylinders.
     */
    static std::shared_ptr<Face> CreateSphericalCornerPatch(
        uint32_t faceId,
        const Geom::Point3D& cornerCenter,
        double radius,
        const Geom::Point3D& p1,
        const Geom::Point3D& p2,
        const Geom::Point3D& p3,
        bool is_convex,
        int subdivisions = 2
    ) {
        auto face = std::make_shared<Face>(faceId, SurfaceGeometryType::Ruled);
        face->radius = radius;

        // Subdivided spherical triangle mesh
        std::vector<CADVertex> verts;
        std::vector<uint32_t> indices;

        auto ProjectToSphere = [&](const Geom::Point3D& pt) -> CADVertex {
            Geom::Vector3D dir = (pt - cornerCenter).normalized();
            Geom::Point3D spherePt = cornerCenter + dir * radius;
            Geom::Vector3D normal = is_convex ? dir : -dir;
            return CADVertex{
                {static_cast<float>(spherePt.x), static_cast<float>(spherePt.y), static_cast<float>(spherePt.z)},
                {static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z)}
            };
        };

        // Recursive spherical triangle subdivision
        struct Tri {
            Geom::Point3D a, b, c;
        };

        std::vector<Tri> currentTris = {{p1, p2, p3}};

        for (int iter = 0; iter < subdivisions; ++iter) {
            std::vector<Tri> nextTris;
            nextTris.reserve(currentTris.size() * 4);
            for (const auto& tri : currentTris) {
                Geom::Point3D ab = cornerCenter + ((tri.a - cornerCenter).normalized() + (tri.b - cornerCenter).normalized()).normalized() * radius;
                Geom::Point3D bc = cornerCenter + ((tri.b - cornerCenter).normalized() + (tri.c - cornerCenter).normalized()).normalized() * radius;
                Geom::Point3D ca = cornerCenter + ((tri.c - cornerCenter).normalized() + (tri.a - cornerCenter).normalized()).normalized() * radius;

                nextTris.push_back({tri.a, ab, ca});
                nextTris.push_back({ab, tri.b, bc});
                nextTris.push_back({ca, bc, tri.c});
                nextTris.push_back({ab, bc, ca});
            }
            currentTris = std::move(nextTris);
        }

        // Check winding orientation relative to sphere outward direction
        Geom::Vector3D refDir = ((p1 - cornerCenter) + (p2 - cornerCenter) + (p3 - cornerCenter)).normalized();

        for (const auto& tri : currentTris) {
            CADVertex v0 = ProjectToSphere(tri.a);
            CADVertex v1 = ProjectToSphere(tri.b);
            CADVertex v2 = ProjectToSphere(tri.c);

            Geom::Vector3D triNormal = (Geom::Point3D(v1.position[0], v1.position[1], v1.position[2]) - Geom::Point3D(v0.position[0], v0.position[1], v0.position[2]))
                .cross(Geom::Point3D(v2.position[0], v2.position[1], v2.position[2]) - Geom::Point3D(v0.position[0], v0.position[1], v0.position[2]));

            bool needsFlip = is_convex ? (triNormal.dot(refDir) < 0.0) : (triNormal.dot(refDir) > 0.0);

            uint32_t baseIdx = static_cast<uint32_t>(verts.size());
            verts.push_back(v0);
            verts.push_back(v1);
            verts.push_back(v2);

            if (!needsFlip) {
                indices.push_back(baseIdx + 0);
                indices.push_back(baseIdx + 1);
                indices.push_back(baseIdx + 2);
            } else {
                indices.push_back(baseIdx + 0);
                indices.push_back(baseIdx + 2);
                indices.push_back(baseIdx + 1);
            }
        }

        face->mesh_vertices = std::move(verts);
        face->mesh_indices = std::move(indices);

        return face;
    }
};

} // namespace Tessera::Modeling

