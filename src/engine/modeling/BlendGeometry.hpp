#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/geom/Plane.hpp"
#include <vector>
#include <array>
#include <optional>

namespace Tessera::Modeling {

struct DihedralInfo {
    bool is_convex{true};
    double dihedral_angle{1.5707963267948966}; // radians
    double angle_degrees{90.0};
    Geom::Vector3D normal1;
    Geom::Vector3D normal2;
    Geom::Vector3D edge_tangent;
    Geom::Vector3D bisector_direction;
};

class BlendGeometry {
public:
    /**
     * @brief Evaluates the signed dihedral angle and convexity between two adjacent faces
     *        sharing an edge with tangent vector t.
     * @param n1 Outward unit normal of Face 1.
     * @param n2 Outward unit normal of Face 2.
     * @param edgeTangent Unit tangent oriented along the shared edge.
     * @param face1InwardHint Vector in Face 1's plane pointing into the interior of Face 1.
     */
    static DihedralInfo ClassifyDihedralEdge(
        const Geom::Vector3D& n1,
        const Geom::Vector3D& n2,
        const Geom::Vector3D& edgeTangent,
        const Geom::Vector3D& face1InwardHint = Geom::Vector3D(0, 0, 0)
    );

    /**
     * @brief Computes the rolling ball center C(s) for a fillet sphere of radius R
     *        at point P_E(s) on the shared edge.
     *        Formula: C(s) = P_E(s) ± (R / sin(theta / 2)) * ((n1 + n2) / ||n1 + n2||)
     *        Convex edge: subtracts bisector offset (places center inside the solid).
     *        Concave edge: adds bisector offset (places center outside the solid / in cavity).
     */
    static Geom::Point3D ComputeRollingBallCenter(
        const Geom::Point3D& edgePt,
        const Geom::Vector3D& n1,
        const Geom::Vector3D& n2,
        double radius,
        bool is_convex,
        double dihedral_angle
    );

    /**
     * @brief Computes the two contact rails r1(s) and r2(s) where the rolling ball touches F1 and F2.
     */
    static void ComputeContactRails(
        const Geom::Point3D& center,
        const Geom::Vector3D& n1,
        const Geom::Vector3D& n2,
        double radius,
        bool is_convex,
        Geom::Point3D& out_r1,
        Geom::Point3D& out_r2
    );

    /**
     * @brief Computes the center of a shared corner sphere where 3 filleted edges converge.
     *        Solves the 3-plane offset intersection: (C - V) . n_i = ± R.
     */
    static std::optional<Geom::Point3D> ComputeCornerSphereCenter(
        const Geom::Point3D& cornerVertex,
        const Geom::Vector3D& n1,
        const Geom::Vector3D& n2,
        const Geom::Vector3D& n3,
        double radius,
        bool is_convex
    );
};

} // namespace Tessera::Modeling

