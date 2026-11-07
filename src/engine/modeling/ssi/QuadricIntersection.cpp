#include "engine/modeling/ssi/QuadricIntersection.hpp"
#include <cmath>
#include <algorithm>

namespace Tessera::Modeling::SSI {

QuadricIntersectionResult QuadricIntersection::IntersectPlaneCylinder(
    const AnalyticalPlane& plane,
    const AnalyticalCylinder& cylinder,
    int sampleCount
) {
    QuadricIntersectionResult result;
    result.plane_normal = plane.normal;

    double cosAlpha = cylinder.axis_direction.dot(plane.normal);
    double absCos = std::abs(cosAlpha);

    // Case 1: Plane is parallel to cylinder axis (absCos ~ 0)
    if (absCos < 1e-4) {
        double distToPlane = std::abs(plane.signed_distance_to(cylinder.axis_origin));
        if (distToPlane > cylinder.radius + 1e-6) {
            result.type = IntersectionType::Empty;
            return result;
        }
        result.type = IntersectionType::ParallelLines;
        double offset = std::sqrt(std::max(0.0, cylinder.radius * cylinder.radius - distToPlane * distToPlane));
        Geom::Vector3D inPlanePerp = cylinder.axis_direction.cross(plane.normal).normalized();
        Geom::Point3D projOnPlane = cylinder.axis_origin - plane.normal * plane.signed_distance_to(cylinder.axis_origin);

        result.center = projOnPlane;
        result.discrete_points.push_back(projOnPlane + inPlanePerp * offset);
        result.discrete_points.push_back(projOnPlane - inPlanePerp * offset);
        return result;
    }

    // Piercing point of cylinder axis through plane
    double tAxis = (plane.origin - cylinder.axis_origin).dot(plane.normal) / cosAlpha;
    Geom::Point3D center = cylinder.axis_origin + cylinder.axis_direction * tAxis;
    result.center = center;

    // Case 2: Plane is perpendicular to cylinder axis (absCos ~ 1) -> Circle
    if (absCos > 0.9999) {
        result.type = IntersectionType::Circle;
        result.major_radius = cylinder.radius;
        result.minor_radius = cylinder.radius;

        // Choose arbitrary orthonormal basis in plane
        Geom::Vector3D u = (std::abs(plane.normal.x) < 0.9)
            ? plane.normal.cross(Geom::Vector3D(1, 0, 0)).normalized()
            : plane.normal.cross(Geom::Vector3D(0, 1, 0)).normalized();
        Geom::Vector3D v = plane.normal.cross(u).normalized();

        int n = std::max(8, sampleCount);
        for (int i = 0; i < n; ++i) {
            double theta = (2.0 * 3.141592653589793 * static_cast<double>(i)) / static_cast<double>(n);
            Geom::Point3D pt = center + (u * std::cos(theta) + v * std::sin(theta)) * cylinder.radius;
            result.discrete_points.push_back(pt);
        }
        return result;
    }

    // Case 3: Oblique intersection -> Ellipse
    result.type = IntersectionType::Ellipse;
    double sinAlpha = std::sqrt(std::max(1e-6, 1.0 - cosAlpha * cosAlpha));

    // Minor axis is in the plane and perpendicular to the projection of cylinder axis
    Geom::Vector3D uMinor = cylinder.axis_direction.cross(plane.normal).normalized();
    // Major axis is in the plane, perpendicular to minor axis
    Geom::Vector3D uMajor = plane.normal.cross(uMinor).normalized();

    double b = cylinder.radius;            // Minor radius
    double a = cylinder.radius / sinAlpha; // Major radius

    result.major_radius = a;
    result.minor_radius = b;

    int n = std::max(8, sampleCount);
    for (int i = 0; i < n; ++i) {
        double theta = (2.0 * 3.141592653589793 * static_cast<double>(i)) / static_cast<double>(n);
        Geom::Point3D pt = center + uMinor * (b * std::cos(theta)) + uMajor * (a * std::sin(theta));
        result.discrete_points.push_back(pt);
    }

    return result;
}

QuadricIntersectionResult QuadricIntersection::IntersectCylinderSphereTangent(
    const AnalyticalCylinder& cylinder,
    const AnalyticalSphere& sphere,
    int sampleCount
) {
    QuadricIntersectionResult result;
    result.type = IntersectionType::Circle;
    result.major_radius = cylinder.radius;
    result.minor_radius = cylinder.radius;
    result.plane_normal = cylinder.axis_direction;

    // Contact circle is centered at the projection of sphere center onto cylinder axis
    Geom::Point3D center = cylinder.project_to_axis(sphere.center);
    result.center = center;

    Geom::Vector3D u = (std::abs(cylinder.axis_direction.x) < 0.9)
        ? cylinder.axis_direction.cross(Geom::Vector3D(1, 0, 0)).normalized()
        : cylinder.axis_direction.cross(Geom::Vector3D(0, 1, 0)).normalized();
    Geom::Vector3D v = cylinder.axis_direction.cross(u).normalized();

    int n = std::max(8, sampleCount);
    for (int i = 0; i < n; ++i) {
        double theta = (2.0 * 3.141592653589793 * static_cast<double>(i)) / static_cast<double>(n);
        Geom::Point3D pt = center + (u * std::cos(theta) + v * std::sin(theta)) * cylinder.radius;
        result.discrete_points.push_back(pt);
    }

    return result;
}

QuadricIntersectionResult QuadricIntersection::IntersectCylinderCylinderMiter(
    const AnalyticalCylinder& cyl1,
    const AnalyticalCylinder& cyl2,
    int sampleCount
) {
    QuadricIntersectionResult result;
    result.type = IntersectionType::Ellipse;

    // Miter plane bisects the two cylinder axes
    Geom::Vector3D d1 = cyl1.axis_direction;
    Geom::Vector3D d2 = cyl2.axis_direction;
    Geom::Vector3D miterNormal = (d1 - d2).normalized();
    if (miterNormal.length_squared() < 1e-4) {
        miterNormal = (d1 + d2).normalized();
    }
    result.plane_normal = miterNormal;

    // Intersection center of axes
    Geom::Point3D center = cyl1.axis_origin;
    result.center = center;

    // Use Plane-Cylinder intersection against the miter plane
    AnalyticalPlane miterPlane(center, miterNormal);
    return IntersectPlaneCylinder(miterPlane, cyl1, sampleCount);
}

bool QuadricIntersection::IntersectSegmentPlane(
    const Geom::Point3D& p0,
    const Geom::Point3D& p1,
    const AnalyticalPlane& plane,
    double& out_t,
    Geom::Point3D& out_pt
) {
    Geom::Vector3D seg = p1 - p0;
    double denom = seg.dot(plane.normal);
    if (std::abs(denom) < 1e-8) {
        return false; // Segment parallel to plane
    }

    double t = (plane.origin - p0).dot(plane.normal) / denom;
    if (t < -1e-6 || t > 1.0 + 1e-6) {
        return false; // Intersection outside segment
    }

    out_t = std::clamp(t, 0.0, 1.0);
    out_pt = p0 + seg * out_t;
    return true;
}

} // namespace Tessera::Modeling::SSI

