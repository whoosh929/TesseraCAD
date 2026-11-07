#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/geom/Axis3D.hpp"
#include <vector>
#include <optional>
#include <cmath>

namespace Tessera::Modeling::SSI {

struct AnalyticalPlane {
    Geom::Point3D origin{0.0, 0.0, 0.0};
    Geom::Vector3D normal{0.0, 1.0, 0.0};

    AnalyticalPlane() = default;
    AnalyticalPlane(const Geom::Point3D& orig, const Geom::Vector3D& n)
        : origin(orig), normal(n.normalized()) {}

    [[nodiscard]] double signed_distance_to(const Geom::Point3D& pt) const noexcept {
        return (pt - origin).dot(normal);
    }
};

struct AnalyticalCylinder {
    Geom::Point3D axis_origin{0.0, 0.0, 0.0};
    Geom::Vector3D axis_direction{0.0, 0.0, 1.0};
    double radius{1.0};

    AnalyticalCylinder() = default;
    AnalyticalCylinder(const Geom::Point3D& orig, const Geom::Vector3D& dir, double r)
        : axis_origin(orig), axis_direction(dir.normalized()), radius(std::abs(r)) {}

    [[nodiscard]] Geom::Point3D project_to_axis(const Geom::Point3D& pt) const noexcept {
        double t = (pt - axis_origin).dot(axis_direction);
        return axis_origin + axis_direction * t;
    }

    [[nodiscard]] double distance_to_axis(const Geom::Point3D& pt) const noexcept {
        return pt.distance_to(project_to_axis(pt));
    }

    [[nodiscard]] Geom::Point3D point_at(double axis_t, double theta_rad, const Geom::Vector3D& u_ref) const noexcept {
        Geom::Vector3D u = (u_ref - axis_direction * u_ref.dot(axis_direction)).normalized();
        Geom::Vector3D v = axis_direction.cross(u).normalized();
        Geom::Point3D centerOnAxis = axis_origin + axis_direction * axis_t;
        return centerOnAxis + (u * std::cos(theta_rad) + v * std::sin(theta_rad)) * radius;
    }
};

struct AnalyticalSphere {
    Geom::Point3D center{0.0, 0.0, 0.0};
    double radius{1.0};

    AnalyticalSphere() = default;
    AnalyticalSphere(const Geom::Point3D& c, double r)
        : center(c), radius(std::abs(r)) {}
};

enum class IntersectionType {
    Empty,
    Point,
    Circle,
    Ellipse,
    ParallelLines,
    SpaceCurve
};

struct QuadricIntersectionResult {
    IntersectionType type{IntersectionType::Empty};
    std::vector<Geom::Point3D> discrete_points;
    Geom::Point3D center{0.0, 0.0, 0.0};
    Geom::Vector3D plane_normal{0.0, 1.0, 0.0};
    double major_radius{0.0};
    double minor_radius{0.0};
};

class QuadricIntersection {
public:
    /**
     * @brief Computes analytical intersection between a Plane and a Cylinder.
     *        If plane is perpendicular to cylinder axis, returns a circle.
     *        If plane is oblique, returns an ellipse.
     *        If plane is parallel to cylinder axis and within distance <= R, returns parallel lines.
     */
    static QuadricIntersectionResult IntersectPlaneCylinder(
        const AnalyticalPlane& plane,
        const AnalyticalCylinder& cylinder,
        int sampleCount = 32
    );

    /**
     * @brief Computes the contact circle seam where a cylinder is tangent to a sphere of the same radius.
     *        When a rolling ball fillet cylinder of radius R terminates at a corner sphere of radius R,
     *        their intersection is a planar circle of radius R perpendicular to the cylinder axis.
     */
    static QuadricIntersectionResult IntersectCylinderSphereTangent(
        const AnalyticalCylinder& cylinder,
        const AnalyticalSphere& sphere,
        int sampleCount = 32
    );

    /**
     * @brief Intersects two intersecting rolling-ball cylinders of radius R1, R2.
     *        When two orthogonal fillet cylinders with R1 = R2 meet, the intersection
     *        is an elliptical miter plane seam (Steinmetz intersection).
     */
    static QuadricIntersectionResult IntersectCylinderCylinderMiter(
        const AnalyticalCylinder& cyl1,
        const AnalyticalCylinder& cyl2,
        int sampleCount = 32
    );

    /**
     * @brief Intersects a line segment [p0, p1] with an analytical plane.
     * @param out_t Parameter along segment in [0, 1].
     * @param out_pt Exact 3D intersection point.
     * @return true if intersection exists in [0, 1].
     */
    static bool IntersectSegmentPlane(
        const Geom::Point3D& p0,
        const Geom::Point3D& p1,
        const AnalyticalPlane& plane,
        double& out_t,
        Geom::Point3D& out_pt
    );
};

} // namespace Tessera::Modeling::SSI
