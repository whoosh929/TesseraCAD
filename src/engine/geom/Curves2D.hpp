#pragma once

#include "Transform3D.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <numbers>
#include <algorithm>

namespace Tessera::Geom {

inline constexpr double TWO_PI = 6.28318530717958647692;
inline constexpr double PI     = 3.14159265358979323846;

enum class CurveType : uint8_t {
    Line = 0,
    Arc = 1,
    CubicBezier = 2
};

// =============================================================================
// Abstract 2D Parametric Curve Interface
// =============================================================================

class Curve2D {
public:
    virtual ~Curve2D() = default;

    [[nodiscard]] virtual CurveType type() const noexcept = 0;
    [[nodiscard]] virtual Point2D start_point() const noexcept = 0;
    [[nodiscard]] virtual Point2D end_point() const noexcept = 0;
    [[nodiscard]] virtual Point2D point_at(double t) const noexcept = 0;
    [[nodiscard]] virtual Vector2D tangent_at(double t) const noexcept = 0;
    [[nodiscard]] virtual double length() const noexcept = 0;
    [[nodiscard]] virtual std::shared_ptr<Curve2D> reversed() const = 0;
    [[nodiscard]] virtual std::vector<Point2D> discretize(double max_sagitta = 0.05, int min_segments = 8) const = 0;
    [[nodiscard]] virtual std::shared_ptr<Curve2D> clone() const = 0;
};

// =============================================================================
// 2D Line Segment
// =============================================================================

class LineSegment2D : public Curve2D {
public:
    Point2D start{0.0, 0.0};
    Point2D end{0.0, 0.0};

    constexpr LineSegment2D() noexcept = default;
    constexpr LineSegment2D(const Point2D& s, const Point2D& e) noexcept : start(s), end(e) {}

    [[nodiscard]] CurveType type() const noexcept override { return CurveType::Line; }
    [[nodiscard]] Point2D start_point() const noexcept override { return start; }
    [[nodiscard]] Point2D end_point() const noexcept override { return end; }

    [[nodiscard]] Point2D point_at(double t) const noexcept override {
        return Point2D(
            start.u + (end.u - start.u) * t,
            start.v + (end.v - start.v) * t
        );
    }

    [[nodiscard]] Vector2D tangent_at(double) const noexcept override {
        return (end - start).normalized();
    }

    [[nodiscard]] double length() const noexcept override {
        return start.distance_to(end);
    }

    [[nodiscard]] std::shared_ptr<Curve2D> reversed() const override {
        return std::make_shared<LineSegment2D>(end, start);
    }

    [[nodiscard]] std::vector<Point2D> discretize(double, int) const override {
        return {start, end};
    }

    [[nodiscard]] std::shared_ptr<Curve2D> clone() const override {
        return std::make_shared<LineSegment2D>(start, end);
    }
};

// =============================================================================
// 2D Circular Arc
// =============================================================================

class Arc2D : public Curve2D {
public:
    Point2D center{0.0, 0.0};
    double radius{1.0};
    double start_angle{0.0}; // in radians
    double end_angle{0.0};   // in radians
    bool is_ccw{true};

    Arc2D() = default;
    Arc2D(const Point2D& c, double r, double s_ang, double e_ang, bool ccw = true) noexcept
        : center(c), radius(std::max(CAD_EPSILON, r)), start_angle(s_ang), end_angle(e_ang), is_ccw(ccw) {}

    static std::shared_ptr<Arc2D> Circle(const Point2D& c, double r) noexcept {
        return std::make_shared<Arc2D>(c, r, 0.0, TWO_PI, true);
    }

    [[nodiscard]] CurveType type() const noexcept override { return CurveType::Arc; }

    [[nodiscard]] double sweep_angle() const noexcept {
        double diff = end_angle - start_angle;
        if (is_ccw) {
            while (diff <= 0.0) diff += TWO_PI;
            while (diff > TWO_PI + CAD_EPSILON) diff -= TWO_PI;
        } else {
            diff = start_angle - end_angle;
            while (diff <= 0.0) diff += TWO_PI;
            while (diff > TWO_PI + CAD_EPSILON) diff -= TWO_PI;
        }
        return diff;
    }

    [[nodiscard]] Point2D start_point() const noexcept override {
        return Point2D(center.u + radius * std::cos(start_angle),
                       center.v + radius * std::sin(start_angle));
    }

    [[nodiscard]] Point2D end_point() const noexcept override {
        return Point2D(center.u + radius * std::cos(end_angle),
                       center.v + radius * std::sin(end_angle));
    }

    [[nodiscard]] Point2D point_at(double t) const noexcept override {
        double sw = sweep_angle();
        double ang = is_ccw ? (start_angle + t * sw) : (start_angle - t * sw);
        return Point2D(center.u + radius * std::cos(ang),
                       center.v + radius * std::sin(ang));
    }

    [[nodiscard]] Vector2D tangent_at(double t) const noexcept override {
        double sw = sweep_angle();
        double ang = is_ccw ? (start_angle + t * sw) : (start_angle - t * sw);
        Vector2D tan(-std::sin(ang), std::cos(ang));
        return is_ccw ? tan : -tan;
    }

    [[nodiscard]] double length() const noexcept override {
        return radius * sweep_angle();
    }

    [[nodiscard]] std::shared_ptr<Curve2D> reversed() const override {
        return std::make_shared<Arc2D>(center, radius, end_angle, start_angle, !is_ccw);
    }

    [[nodiscard]] std::vector<Point2D> discretize(double max_sagitta = 0.05, int min_segments = 8) const override {
        double sw = sweep_angle();
        int segs = min_segments;

        // Number of segments based on sagitta: sagitta = r * (1 - cos(theta / 2))
        if (radius > CAD_EPSILON && max_sagitta > 0.0) {
            double cosHalfTheta = 1.0 - std::min(0.99999, max_sagitta / radius);
            double theta = 2.0 * std::acos(cosHalfTheta);
            if (theta > 1e-4) {
                int sagittaSegs = static_cast<int>(std::ceil(sw / theta));
                segs = std::clamp(std::max(segs, sagittaSegs), 4, 128);
            }
        }

        std::vector<Point2D> pts;
        pts.reserve(segs + 1);
        for (int i = 0; i <= segs; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(segs);
            pts.push_back(point_at(t));
        }
        return pts;
    }

    [[nodiscard]] std::shared_ptr<Curve2D> clone() const override {
        return std::make_shared<Arc2D>(center, radius, start_angle, end_angle, is_ccw);
    }
};

// =============================================================================
// 2D Cubic Bezier Spline
// =============================================================================

class CubicBezier2D : public Curve2D {
public:
    Point2D p0{0.0, 0.0};
    Point2D p1{0.0, 0.0};
    Point2D p2{0.0, 0.0};
    Point2D p3{0.0, 0.0};

    CubicBezier2D() = default;
    CubicBezier2D(const Point2D& pt0, const Point2D& pt1, const Point2D& pt2, const Point2D& pt3) noexcept
        : p0(pt0), p1(pt1), p2(pt2), p3(pt3) {}

    [[nodiscard]] CurveType type() const noexcept override { return CurveType::CubicBezier; }
    [[nodiscard]] Point2D start_point() const noexcept override { return p0; }
    [[nodiscard]] Point2D end_point() const noexcept override { return p3; }

    [[nodiscard]] Point2D point_at(double t) const noexcept override {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        return Point2D(
            uuu * p0.u + 3.0 * uu * t * p1.u + 3.0 * u * tt * p2.u + ttt * p3.u,
            uuu * p0.v + 3.0 * uu * t * p1.v + 3.0 * u * tt * p2.v + ttt * p3.v
        );
    }

    [[nodiscard]] Vector2D tangent_at(double t) const noexcept override {
        double u = 1.0 - t;
        double du = 3.0 * u * u * (p1.u - p0.u) + 6.0 * u * t * (p2.u - p1.u) + 3.0 * t * t * (p3.u - p2.u);
        double dv = 3.0 * u * u * (p1.v - p0.v) + 6.0 * u * t * (p2.v - p1.v) + 3.0 * t * t * (p3.v - p2.v);
        return Vector2D(du, dv).normalized();
    }

    [[nodiscard]] double length() const noexcept override {
        // Approximate length via 16-segment numerical integration
        double len = 0.0;
        Point2D prev = p0;
        constexpr int N = 16;
        for (int i = 1; i <= N; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(N);
            Point2D curr = point_at(t);
            len += prev.distance_to(curr);
            prev = curr;
        }
        return len;
    }

    [[nodiscard]] std::shared_ptr<Curve2D> reversed() const override {
        return std::make_shared<CubicBezier2D>(p3, p2, p1, p0);
    }

    [[nodiscard]] std::vector<Point2D> discretize(double, int min_segments = 16) const override {
        int segs = std::max(min_segments, 16);
        std::vector<Point2D> pts;
        pts.reserve(segs + 1);
        for (int i = 0; i <= segs; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(segs);
            pts.push_back(point_at(t));
        }
        return pts;
    }

    [[nodiscard]] std::shared_ptr<Curve2D> clone() const override {
        return std::make_shared<CubicBezier2D>(p0, p1, p2, p3);
    }
};

} // namespace Tessera::Geom
