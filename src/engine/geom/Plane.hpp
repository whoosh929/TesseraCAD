#pragma once

#include "Transform3D.hpp"
#include "Axis3D.hpp"
#include <cmath>

namespace Tessera::Geom {

class Plane {
public:
    Point3D origin{0.0, 0.0, 0.0};
    Vector3D normal{0.0, 0.0, 1.0};  // Right-handed: normal = u_axis.cross(v_axis)
    Vector3D u_axis{1.0, 0.0, 0.0};  // In-plane local X (tangent)
    Vector3D v_axis{0.0, 1.0, 0.0};  // In-plane local Y (bitangent)

    constexpr Plane() noexcept = default;

    // Explicit constructor with origin and orthonormal vectors
    constexpr Plane(const Point3D& orig, const Vector3D& n, const Vector3D& u, const Vector3D& v) noexcept
        : origin(orig), normal(n), u_axis(u), v_axis(v) {}

    // Orthonormal basis constructed from origin, normal, and a tangent hint direction
    Plane(const Point3D& orig, const Vector3D& n, const Vector3D& u_hint) noexcept
        : origin(orig) {
        normal = n.normalized();
        // Gram-Schmidt orthogonalization against normal
        Vector3D proj = u_hint - normal * u_hint.dot(normal);
        if (proj.length_squared() <= CAD_EPSILON) {
            // u_hint was parallel to normal; generate fallback basis
            GenerateBasisFromNormal();
        } else {
            u_axis = proj.normalized();
            v_axis = normal.cross(u_axis).normalized();
        }
    }

    // Automatic orthonormal basis from origin and normal vector
    Plane(const Point3D& orig, const Vector3D& n) noexcept
        : origin(orig), normal(n.normalized()) {
        GenerateBasisFromNormal();
    }

    // Three-point construction: p0 = origin, p1 defines +U, p2 defines +V half-plane
    Plane(const Point3D& p0, const Point3D& p1, const Point3D& p2) noexcept
        : origin(p0) {
        Vector3D v01 = p1 - p0;
        Vector3D v02 = p2 - p0;

        Vector3D cross = v01.cross(v02);
        if (cross.length_squared() <= CAD_EPSILON) {
            // Collinear points fallback
            normal = Vector3D::UnitZ();
            GenerateBasisFromNormal();
        } else {
            normal = cross.normalized();
            u_axis = v01.normalized();
            v_axis = normal.cross(u_axis).normalized();
        }
    }

    // =========================================================================
    // Standard Datum Planes
    // =========================================================================

    // XY Datum Plane (Normal = +Z, U = +X, V = +Y)
    static Plane XY(const Point3D& orig = {0.0, 0.0, 0.0}) noexcept {
        return Plane(
            orig,
            Vector3D(0.0, 0.0, 1.0),
            Vector3D(1.0, 0.0, 0.0),
            Vector3D(0.0, 1.0, 0.0)
        );
    }

    // XZ Datum Plane (Top view: Normal = +Y, U = +X, V = -Z)
    // Satisfies: U x V = (1, 0, 0) x (0, 0, -1) = (0, 1, 0) = Normal
    static Plane XZ(const Point3D& orig = {0.0, 0.0, 0.0}) noexcept {
        return Plane(
            orig,
            Vector3D(0.0, 1.0, 0.0),
            Vector3D(1.0, 0.0, 0.0),
            Vector3D(0.0, 0.0, -1.0)
        );
    }

    // YZ Datum Plane (Right view: Normal = +X, U = -Z, V = +Y)
    // Satisfies: U x V = (0, 0, -1) x (0, 1, 0) = (1, 0, 0) = Normal
    static Plane YZ(const Point3D& orig = {0.0, 0.0, 0.0}) noexcept {
        return Plane(
            orig,
            Vector3D(1.0, 0.0, 0.0),
            Vector3D(0.0, 0.0, -1.0),
            Vector3D(0.0, 1.0, 0.0)
        );
    }

    // =========================================================================
    // Coordinate Transformations
    // =========================================================================

    // Local 2D UV coordinate -> 3D World space Point
    [[nodiscard]] Point3D to_world(const Point2D& uv) const noexcept {
        return origin + u_axis * uv.u + v_axis * uv.v;
    }

    // Local 2D UV direction vector -> 3D World space Vector
    [[nodiscard]] Vector3D to_world_vector(const Vector2D& uv) const noexcept {
        return u_axis * uv.u + v_axis * uv.v;
    }

    // 3D World space Point -> Local 2D UV coordinate
    [[nodiscard]] Point2D to_local(const Point3D& xyz) const noexcept {
        Vector3D d = xyz - origin;
        return Point2D(d.dot(u_axis), d.dot(v_axis));
    }

    // 3D World space Vector -> Local 2D UV direction vector
    [[nodiscard]] Vector2D to_local_vector(const Vector3D& xyz) const noexcept {
        return Vector2D(xyz.dot(u_axis), xyz.dot(v_axis));
    }

    // =========================================================================
    // Geometric Queries
    // =========================================================================

    // Signed distance from plane to 3D point (positive if on side of normal)
    [[nodiscard]] double distance_to(const Point3D& xyz) const noexcept {
        return (xyz - origin).dot(normal);
    }

    // Project 3D point orthogonally onto this plane
    [[nodiscard]] Point3D project(const Point3D& xyz) const noexcept {
        return to_world(to_local(xyz));
    }

    // True if 3D point lies on plane within tolerance
    [[nodiscard]] bool contains(const Point3D& xyz, double tol = CAD_TOLERANCE) const noexcept {
        return std::abs(distance_to(xyz)) <= tol;
    }

    [[nodiscard]] Axis3D normal_axis() const noexcept {
        return Axis3D(origin, normal);
    }

    [[nodiscard]] Axis3D u_axis3d() const noexcept {
        return Axis3D(origin, u_axis);
    }

    [[nodiscard]] Axis3D v_axis3d() const noexcept {
        return Axis3D(origin, v_axis);
    }

    // =========================================================================
    // Derived Datum Operations
    // =========================================================================

    // Create an offset plane parallel to this plane shifted along normal
    [[nodiscard]] Plane offset(double distance) const noexcept {
        return Plane(origin + normal * distance, normal, u_axis, v_axis);
    }

    // Reverse orientation of plane (retains right-handed basis)
    [[nodiscard]] Plane flipped() const noexcept {
        return Plane(origin, -normal, u_axis, -v_axis);
    }

    // Transform plane by affine 4x4 matrix
    [[nodiscard]] Plane transformed(const Transform3D& xform) const noexcept {
        Point3D newOrigin = xform.transform_point(origin);
        Vector3D newNormal = xform.transform_vector(normal).normalized();
        Vector3D newU = xform.transform_vector(u_axis).normalized();
        Vector3D newV = newNormal.cross(newU).normalized();
        return Plane(newOrigin, newNormal, newU, newV);
    }

    [[nodiscard]] bool approx_equal(const Plane& o, double tol = CAD_TOLERANCE) const noexcept {
        return origin.approx_equal(o.origin, tol) &&
               normal.approx_equal(o.normal, tol) &&
               u_axis.approx_equal(o.u_axis, tol) &&
               v_axis.approx_equal(o.v_axis, tol);
    }

private:
    // Generate right-handed orthonormal basis (u, v) given unit normal
    void GenerateBasisFromNormal() noexcept {
        if (std::abs(normal.x) > std::abs(normal.z)) {
            u_axis = Vector3D(-normal.y, normal.x, 0.0).normalized();
        } else {
            u_axis = Vector3D(0.0, -normal.z, normal.y).normalized();
        }
        v_axis = normal.cross(u_axis).normalized();
    }
};

} // namespace Tessera::Geom
