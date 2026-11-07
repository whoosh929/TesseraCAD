#include "engine/modeling/blend/VertexBlend3Corner.hpp"
#include <cmath>
#include <algorithm>

namespace Tessera::Modeling::Blend {

CornerBlendResult VertexBlend3Corner::Solve(
    const Geom::Point3D& cornerVertex,
    const std::array<CornerBranch, 3>& branches,
    double radius,
    bool is_convex,
    int subdivisions
) {
    CornerBlendResult result;

    // 1. Extract the 3 distinct face normals meeting at the corner
    std::vector<Geom::Vector3D> candidateNorms;
    for (const auto& br : branches) {
        candidateNorms.push_back(br.face_normal_a.normalized());
        candidateNorms.push_back(br.face_normal_b.normalized());
    }

    std::vector<Geom::Vector3D> uniqueNorms;
    for (const auto& cn : candidateNorms) {
        bool alreadyExists = false;
        for (const auto& un : uniqueNorms) {
            if (std::abs(un.dot(cn)) > 0.95) {
                alreadyExists = true;
                break;
            }
        }
        if (!alreadyExists) {
            uniqueNorms.push_back(cn);
        }
    }

    if (uniqueNorms.size() < 3) {
        result.success = false;
        return result;
    }

    Geom::Vector3D n1 = uniqueNorms[0];
    Geom::Vector3D n2 = uniqueNorms[1];
    Geom::Vector3D n3 = uniqueNorms[2];

    // 2. Compute the exact 3-plane offset corner sphere center
    auto optCenter = BlendGeometry::ComputeCornerSphereCenter(
        cornerVertex, n1, n2, n3, radius, is_convex
    );

    if (!optCenter) {
        result.success = false;
        return result;
    }

    Geom::Point3D center = *optCenter;
    result.sphere_center = center;
    result.sphere_radius = radius;

    // 3. Contact points on the three incident faces
    Geom::Point3D p1 = center + (is_convex ? n1 : -n1) * radius;
    Geom::Point3D p2 = center + (is_convex ? n2 : -n2) * radius;
    Geom::Point3D p3 = center + (is_convex ? n3 : -n3) * radius;
    result.contact_points = {p1, p2, p3};

    // 4. Trimming planes for the 3 incoming rolling-ball cylinders
    for (int i = 0; i < 3; ++i) {
        Geom::Vector3D branchDir = branches[i].edge_direction.normalized();
        // The clipping plane passes through the sphere center perpendicular to the cylinder axis
        result.branch_trim_planes.emplace_back(center, branchDir);
    }

    // 5. Construct the triangular spherical patch
    auto ProjectToSphere = [&](const Geom::Point3D& pt) -> CADVertex {
        Geom::Vector3D dir = (pt - center).normalized();
        Geom::Point3D spherePt = center + dir * radius;
        Geom::Vector3D normal = is_convex ? dir : -dir;
        return CADVertex{
            {static_cast<float>(spherePt.x), static_cast<float>(spherePt.y), static_cast<float>(spherePt.z)},
            {static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z)}
        };
    };

    struct Tri {
        Geom::Point3D a, b, c;
    };
    std::vector<Tri> currentTris = {{p1, p2, p3}};

    int iters = std::clamp(subdivisions, 1, 5);
    for (int iter = 0; iter < iters; ++iter) {
        std::vector<Tri> nextTris;
        nextTris.reserve(currentTris.size() * 4);
        for (const auto& tri : currentTris) {
            Geom::Point3D ab = center + ((tri.a - center).normalized() + (tri.b - center).normalized()).normalized() * radius;
            Geom::Point3D bc = center + ((tri.b - center).normalized() + (tri.c - center).normalized()).normalized() * radius;
            Geom::Point3D ca = center + ((tri.c - center).normalized() + (tri.a - center).normalized()).normalized() * radius;

            nextTris.push_back({tri.a, ab, ca});
            nextTris.push_back({ab, tri.b, bc});
            nextTris.push_back({ca, bc, tri.c});
            nextTris.push_back({ab, bc, ca});
        }
        currentTris = std::move(nextTris);
    }

    Geom::Vector3D refDir = ((p1 - center) + (p2 - center) + (p3 - center)).normalized();

    for (const auto& tri : currentTris) {
        CADVertex v0 = ProjectToSphere(tri.a);
        CADVertex v1 = ProjectToSphere(tri.b);
        CADVertex v2 = ProjectToSphere(tri.c);

        Geom::Vector3D triNormal = (Geom::Point3D(v1.position[0], v1.position[1], v1.position[2]) - Geom::Point3D(v0.position[0], v0.position[1], v0.position[2]))
            .cross(Geom::Point3D(v2.position[0], v2.position[1], v2.position[2]) - Geom::Point3D(v0.position[0], v0.position[1], v0.position[2]));

        bool needsFlip = is_convex ? (triNormal.dot(refDir) < 0.0) : (triNormal.dot(refDir) > 0.0);

        uint32_t baseIdx = static_cast<uint32_t>(result.patch_vertices.size());
        result.patch_vertices.push_back(v0);
        result.patch_vertices.push_back(v1);
        result.patch_vertices.push_back(v2);

        if (!needsFlip) {
            result.patch_indices.push_back(baseIdx + 0);
            result.patch_indices.push_back(baseIdx + 1);
            result.patch_indices.push_back(baseIdx + 2);
        } else {
            result.patch_indices.push_back(baseIdx + 0);
            result.patch_indices.push_back(baseIdx + 2);
            result.patch_indices.push_back(baseIdx + 1);
        }
    }

    result.success = true;
    return result;
}

} // namespace Tessera::Modeling::Blend

