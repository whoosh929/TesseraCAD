#include "engine/modeling/BlendGeometry.hpp"
#include <cmath>
#include <algorithm>

namespace Tessera::Modeling {

DihedralInfo BlendGeometry::ClassifyDihedralEdge(
    const Geom::Vector3D& n1,
    const Geom::Vector3D& n2,
    const Geom::Vector3D& edgeTangent,
    const Geom::Vector3D& face1InwardHint
) {
    DihedralInfo info;
    info.normal1 = n1.normalized();
    info.normal2 = n2.normalized();

    Geom::Vector3D t = edgeTangent.normalized();
    if (t.length_squared() < 1e-6) {
        // Fallback: cross normal1 with normal2
        t = info.normal1.cross(info.normal2).normalized();
    }

    // If an inward hint for Face 1 was provided, orient t along Face 1's boundary wire
    // so that u1 = n1 x t points into Face 1
    if (face1InwardHint.length_squared() > 1e-4) {
        Geom::Vector3D u1 = info.normal1.cross(t);
        if (u1.dot(face1InwardHint) < 0.0) {
            t = -t;
        }
    }
    info.edge_tangent = t;

    // Signed dihedral test: m = n1 x n2
    Geom::Vector3D m = info.normal1.cross(info.normal2);
    double mDotT = m.dot(t);

    // If m is nearly zero (coplanar or anti-parallel faces)
    if (m.length_squared() < 1e-6) {
        double dotNormals = info.normal1.dot(info.normal2);
        if (dotNormals > 0.0) {
            // Flat coplanar surface
            info.is_convex = true;
            info.dihedral_angle = 3.141592653589793;
            info.angle_degrees = 180.0;
            info.bisector_direction = info.normal1;
            return info;
        } else {
            // Knife-edge 0 deg / 360 deg
            info.is_convex = true;
            info.dihedral_angle = 0.0;
            info.angle_degrees = 0.0;
            info.bisector_direction = (info.normal1.cross(t)).normalized();
            return info;
        }
    }

    // Convex if m . t > 0; Concave if m . t < 0
    info.is_convex = (mDotT >= 0.0);

    // Angle between outward normals
    double cosAlpha = std::clamp(info.normal1.dot(info.normal2), -1.0, 1.0);
    double sinAlpha = m.length();
    double alpha = std::atan2(sinAlpha, cosAlpha); // alpha in [0, pi]

    if (info.is_convex) {
        info.dihedral_angle = 3.141592653589793 - alpha; // interior angle < 180 deg
    } else {
        info.dihedral_angle = 3.141592653589793 + alpha; // interior angle > 180 deg
    }
    info.angle_degrees = info.dihedral_angle * (180.0 / 3.141592653589793);

    // Outward bisector direction
    Geom::Vector3D sumN = info.normal1 + info.normal2;
    if (sumN.length_squared() > 1e-6) {
        info.bisector_direction = sumN.normalized();
    } else {
        info.bisector_direction = info.normal1.cross(t).normalized();
    }

    return info;
}

Geom::Point3D BlendGeometry::ComputeRollingBallCenter(
    const Geom::Point3D& edgePt,
    const Geom::Vector3D& n1,
    const Geom::Vector3D& n2,
    double radius,
    bool is_convex,
    double dihedral_angle
) {
    double halfTheta = dihedral_angle * 0.5;
    double sinHalf = std::sin(halfTheta);
    if (std::abs(sinHalf) < 0.02) {
        sinHalf = (sinHalf >= 0.0) ? 0.02 : -0.02;
    }

    double distCenter = radius / std::abs(sinHalf);

    Geom::Vector3D bisector = (n1.normalized() + n2.normalized());
    if (bisector.length_squared() > 1e-6) {
        bisector = bisector.normalized();
    } else {
        bisector = n1.normalized();
    }

    // Convex edge: subtract outward bisector offset to place center inside solid
    // Concave edge: add outward bisector offset to place center outside solid (in cavity)
    if (is_convex) {
        return edgePt - bisector * distCenter;
    } else {
        return edgePt + bisector * distCenter;
    }
}

void BlendGeometry::ComputeContactRails(
    const Geom::Point3D& center,
    const Geom::Vector3D& n1,
    const Geom::Vector3D& n2,
    double radius,
    bool is_convex,
    Geom::Point3D& out_r1,
    Geom::Point3D& out_r2
) {
    Geom::Vector3D unitN1 = n1.normalized();
    Geom::Vector3D unitN2 = n2.normalized();

    // Convex: contact point is center + R * outward_normal
    // Concave: contact point is center - R * outward_normal
    if (is_convex) {
        out_r1 = center + unitN1 * radius;
        out_r2 = center + unitN2 * radius;
    } else {
        out_r1 = center - unitN1 * radius;
        out_r2 = center - unitN2 * radius;
    }
}

std::optional<Geom::Point3D> BlendGeometry::ComputeCornerSphereCenter(
    const Geom::Point3D& cornerVertex,
    const Geom::Vector3D& n1,
    const Geom::Vector3D& n2,
    const Geom::Vector3D& n3,
    double radius,
    bool is_convex
) {
    Geom::Vector3D u1 = n1.normalized();
    Geom::Vector3D u2 = n2.normalized();
    Geom::Vector3D u3 = n3.normalized();

    double det = (u1.cross(u2)).dot(u3);
    if (std::abs(det) < 1e-4) {
        return std::nullopt; // Collinear or coplanar face normals at vertex
    }

    double sign = is_convex ? -radius : +radius;
    double b1 = sign;
    double b2 = sign;
    double b3 = sign;

    // x = (b1*(u2 x u3) + b2*(u3 x u1) + b3*(u1 x u2)) / det
    Geom::Vector3D x = (u2.cross(u3) * b1 + u3.cross(u1) * b2 + u1.cross(u2) * b3) / det;

    return cornerVertex + x;
}

} // namespace Tessera::Modeling
