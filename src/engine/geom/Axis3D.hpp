#pragma once

#include "Transform3D.hpp"

namespace Tessera::Geom {

class Axis3D {
public:
    Point3D origin{0.0, 0.0, 0.0};
    Vector3D direction{0.0, 0.0, 1.0}; // Guaranteed unit vector

    constexpr Axis3D() noexcept = default;

    Axis3D(const Point3D& orig, const Vector3D& dir) noexcept
        : origin(orig), direction(dir.normalized()) {}

    [[nodiscard]] Point3D point_at(double t) const noexcept {
        return origin + direction * t;
    }

    [[nodiscard]] double project(const Point3D& p) const noexcept {
        return (p - origin).dot(direction);
    }

    [[nodiscard]] Point3D project_point(const Point3D& p) const noexcept {
        return point_at(project(p));
    }

    [[nodiscard]] double distance_squared_to(const Point3D& p) const noexcept {
        Point3D proj = project_point(p);
        return p.distance_squared_to(proj);
    }

    [[nodiscard]] double distance_to(const Point3D& p) const noexcept {
        return std::sqrt(distance_squared_to(p));
    }

    [[nodiscard]] Axis3D reversed() const noexcept {
        return Axis3D(origin, -direction);
    }

    [[nodiscard]] Axis3D transformed(const Transform3D& transform) const noexcept {
        return Axis3D(
            transform.transform_point(origin),
            transform.transform_vector(direction).normalized()
        );
    }

    // Standard coordinate axes
    static Axis3D X(const Point3D& orig = {0.0, 0.0, 0.0}) noexcept {
        return Axis3D(orig, Vector3D::UnitX());
    }

    static Axis3D Y(const Point3D& orig = {0.0, 0.0, 0.0}) noexcept {
        return Axis3D(orig, Vector3D::UnitY());
    }

    static Axis3D Z(const Point3D& orig = {0.0, 0.0, 0.0}) noexcept {
        return Axis3D(orig, Vector3D::UnitZ());
    }

    [[nodiscard]] bool approx_equal(const Axis3D& other, double tol = CAD_TOLERANCE) const noexcept {
        return origin.approx_equal(other.origin, tol) && direction.approx_equal(other.direction, tol);
    }
};

} // namespace Tessera::Geom
