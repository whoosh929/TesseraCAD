#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/graph/GeometryTypes.hpp"
#include "engine/modeling/BlendGeometry.hpp"
#include "engine/modeling/blend/FilletSpine.hpp"
#include <vector>
#include <memory>
#include <optional>

namespace Tessera::Modeling::Blend {

struct CornerBranch {
    Geom::Point3D edge_endpoint{0.0, 0.0, 0.0};
    Geom::Vector3D edge_direction{0.0, 0.0, 1.0}; // Pointing away from corner vertex
    Geom::Vector3D face_normal_a{0.0, 1.0, 0.0};
    Geom::Vector3D face_normal_b{1.0, 0.0, 0.0};
    double radius{0.2};
};

struct CornerBlendResult {
    bool success{false};
    Geom::Point3D sphere_center{0.0, 0.0, 0.0};
    double sphere_radius{0.0};
    std::vector<Geom::Point3D> contact_points; // p1, p2, p3
    std::vector<CADVertex> patch_vertices;
    std::vector<uint32_t> patch_indices;
    std::vector<SSI::AnalyticalPlane> branch_trim_planes; // 3 clipping planes for the incoming cylinder blends
};

class VertexBlend3Corner {
public:
    /**
     * @brief Solves a 3-edge converging vertex blend.
     *        Computes the shared corner sphere, generates the triangular G1 spherical patch,
     *        and determines the 3 exact trimming planes for the converging cylinder blends.
     * @param cornerVertex The 3D position of the sharp corner vertex.
     * @param branches The 3 incident edges meeting at the corner.
     * @param radius Fillet radius.
     * @param is_convex Whether the corner is convex (subtracting) or concave (adding).
     * @param subdivisions Tessellation subdivision level for the spherical patch.
     */
    static CornerBlendResult Solve(
        const Geom::Point3D& cornerVertex,
        const std::array<CornerBranch, 3>& branches,
        double radius,
        bool is_convex = true,
        int subdivisions = 3
    );
};

} // namespace Tessera::Modeling::Blend
