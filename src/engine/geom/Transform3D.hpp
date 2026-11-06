#pragma once

#include <cmath>
#include <array>
#include <limits>
#include <algorithm>
#include <cassert>

namespace Tessera::Geom {

inline constexpr double CAD_TOLERANCE = 1e-7;
inline constexpr double CAD_EPSILON = 1e-12;

// =============================================================================
// 2D Geometric Primitives
// =============================================================================

struct Vector2D {
    double u{0.0};
    double v{0.0};

    constexpr Vector2D() noexcept = default;
    constexpr Vector2D(double u_, double v_) noexcept : u(u_), v(v_) {}

    [[nodiscard]] constexpr double dot(const Vector2D& other) const noexcept {
        return u * other.u + v * other.v;
    }

    [[nodiscard]] constexpr double cross(const Vector2D& other) const noexcept {
        return u * other.v - v * other.u; // 2D wedge product / determinant
    }

    [[nodiscard]] double length_squared() const noexcept {
        return u * u + v * v;
    }

    [[nodiscard]] double length() const noexcept {
        return std::hypot(u, v);
    }

    [[nodiscard]] Vector2D normalized(double tol = CAD_EPSILON) const noexcept {
        double len = length();
        if (len <= tol) return Vector2D(0.0, 0.0);
        double inv = 1.0 / len;
        return Vector2D(u * inv, v * inv);
    }

    constexpr Vector2D operator+(const Vector2D& rhs) const noexcept { return {u + rhs.u, v + rhs.v}; }
    constexpr Vector2D operator-(const Vector2D& rhs) const noexcept { return {u - rhs.u, v - rhs.v}; }
    constexpr Vector2D operator-() const noexcept { return {-u, -v}; }
    constexpr Vector2D operator*(double scalar) const noexcept { return {u * scalar, v * scalar}; }
    constexpr Vector2D operator/(double scalar) const noexcept { return {u / scalar, v / scalar}; }

    Vector2D& operator+=(const Vector2D& rhs) noexcept { u += rhs.u; v += rhs.v; return *this; }
    Vector2D& operator-=(const Vector2D& rhs) noexcept { u -= rhs.u; v -= rhs.v; return *this; }
    Vector2D& operator*=(double s) noexcept { u *= s; v *= s; return *this; }
    Vector2D& operator/=(double s) noexcept { u /= s; v /= s; return *this; }

    [[nodiscard]] bool approx_equal(const Vector2D& o, double tol = CAD_TOLERANCE) const noexcept {
        return std::abs(u - o.u) <= tol && std::abs(v - o.v) <= tol;
    }
};

inline constexpr Vector2D operator*(double scalar, const Vector2D& v) noexcept {
    return v * scalar;
}

struct Point2D {
    double u{0.0};
    double v{0.0};

    constexpr Point2D() noexcept = default;
    constexpr Point2D(double u_, double v_) noexcept : u(u_), v(v_) {}

    [[nodiscard]] double distance_squared_to(const Point2D& other) const noexcept {
        double du = u - other.u;
        double dv = v - other.v;
        return du * du + dv * dv;
    }

    [[nodiscard]] double distance_to(const Point2D& other) const noexcept {
        return std::hypot(u - other.u, v - other.v);
    }

    constexpr Point2D operator+(const Vector2D& vec) const noexcept { return {u + vec.u, v + vec.v}; }
    constexpr Point2D operator-(const Vector2D& vec) const noexcept { return {u - vec.u, v - vec.v}; }
    constexpr Vector2D operator-(const Point2D& rhs) const noexcept { return {u - rhs.u, v - rhs.v}; }

    Point2D& operator+=(const Vector2D& vec) noexcept { u += vec.u; v += vec.v; return *this; }
    Point2D& operator-=(const Vector2D& vec) noexcept { u -= vec.u; v -= vec.v; return *this; }

    [[nodiscard]] bool approx_equal(const Point2D& o, double tol = CAD_TOLERANCE) const noexcept {
        return std::abs(u - o.u) <= tol && std::abs(v - o.v) <= tol;
    }

    [[nodiscard]] bool operator==(const Point2D& o) const noexcept {
        return approx_equal(o);
    }
};

// =============================================================================
// 3D Geometric Primitives
// =============================================================================

struct Vector3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vector3D() noexcept = default;
    constexpr Vector3D(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr double dot(const Vector3D& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    [[nodiscard]] constexpr Vector3D cross(const Vector3D& other) const noexcept {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    [[nodiscard]] double length_squared() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] double length() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }

    [[nodiscard]] Vector3D normalized(double tol = CAD_EPSILON) const noexcept {
        double len = length();
        if (len <= tol) return Vector3D(0.0, 0.0, 0.0);
        double inv = 1.0 / len;
        return Vector3D(x * inv, y * inv, z * inv);
    }

    constexpr Vector3D operator+(const Vector3D& rhs) const noexcept { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vector3D operator-(const Vector3D& rhs) const noexcept { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vector3D operator-() const noexcept { return {-x, -y, -z}; }
    constexpr Vector3D operator*(double scalar) const noexcept { return {x * scalar, y * scalar, z * scalar}; }
    constexpr Vector3D operator/(double scalar) const noexcept { return {x / scalar, y / scalar, z / scalar}; }

    Vector3D& operator+=(const Vector3D& rhs) noexcept { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vector3D& operator-=(const Vector3D& rhs) noexcept { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vector3D& operator*=(double s) noexcept { x *= s; y *= s; z *= s; return *this; }
    Vector3D& operator/=(double s) noexcept { x /= s; y /= s; z /= s; return *this; }

    [[nodiscard]] bool approx_equal(const Vector3D& o, double tol = CAD_TOLERANCE) const noexcept {
        return std::abs(x - o.x) <= tol && std::abs(y - o.y) <= tol && std::abs(z - o.z) <= tol;
    }

    [[nodiscard]] bool operator==(const Vector3D& o) const noexcept {
        return approx_equal(o);
    }

    static constexpr Vector3D UnitX() noexcept { return {1.0, 0.0, 0.0}; }
    static constexpr Vector3D UnitY() noexcept { return {0.0, 1.0, 0.0}; }
    static constexpr Vector3D UnitZ() noexcept { return {0.0, 0.0, 1.0}; }
};

inline constexpr Vector3D operator*(double scalar, const Vector3D& v) noexcept {
    return v * scalar;
}

struct Point3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Point3D() noexcept = default;
    constexpr Point3D(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    [[nodiscard]] double distance_squared_to(const Point3D& other) const noexcept {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return dx * dx + dy * dy + dz * dz;
    }

    [[nodiscard]] double distance_to(const Point3D& other) const noexcept {
        return std::sqrt(distance_squared_to(other));
    }

    constexpr Point3D operator+(const Vector3D& vec) const noexcept { return {x + vec.x, y + vec.y, z + vec.z}; }
    constexpr Point3D operator-(const Vector3D& vec) const noexcept { return {x - vec.x, y - vec.y, z - vec.z}; }
    constexpr Vector3D operator-(const Point3D& rhs) const noexcept { return {x - rhs.x, y - rhs.y, z - rhs.z}; }

    Point3D& operator+=(const Vector3D& vec) noexcept { x += vec.x; y += vec.y; z += vec.z; return *this; }
    Point3D& operator-=(const Vector3D& vec) noexcept { x -= vec.x; y -= vec.y; z -= vec.z; return *this; }

    [[nodiscard]] bool approx_equal(const Point3D& o, double tol = CAD_TOLERANCE) const noexcept {
        return std::abs(x - o.x) <= tol && std::abs(y - o.y) <= tol && std::abs(z - o.z) <= tol;
    }

    [[nodiscard]] bool operator==(const Point3D& o) const noexcept {
        return approx_equal(o);
    }
};

// =============================================================================
// 4x4 Affine Transform (Row-Major: m[row][col])
// =============================================================================

class Transform3D {
public:
    // Stored as row-major 4x4 matrix
    std::array<std::array<double, 4>, 4> m{{{1, 0, 0, 0},
                                            {0, 1, 0, 0},
                                            {0, 0, 1, 0},
                                            {0, 0, 0, 1}}};

    constexpr Transform3D() noexcept = default;

    constexpr explicit Transform3D(const std::array<std::array<double, 4>, 4>& matrix) noexcept
        : m(matrix) {}

    static constexpr Transform3D Identity() noexcept {
        return Transform3D();
    }

    static Transform3D Translation(const Vector3D& translation) noexcept {
        Transform3D t;
        t.m[0][3] = translation.x;
        t.m[1][3] = translation.y;
        t.m[2][3] = translation.z;
        return t;
    }

    static Transform3D RotationX(double angleRad) noexcept {
        double c = std::cos(angleRad);
        double s = std::sin(angleRad);
        Transform3D r;
        r.m[1][1] = c;  r.m[1][2] = -s;
        r.m[2][1] = s;  r.m[2][2] = c;
        return r;
    }

    static Transform3D RotationY(double angleRad) noexcept {
        double c = std::cos(angleRad);
        double s = std::sin(angleRad);
        Transform3D r;
        r.m[0][0] = c;   r.m[0][2] = s;
        r.m[2][0] = -s;  r.m[2][2] = c;
        return r;
    }

    static Transform3D RotationZ(double angleRad) noexcept {
        double c = std::cos(angleRad);
        double s = std::sin(angleRad);
        Transform3D r;
        r.m[0][0] = c;  r.m[0][1] = -s;
        r.m[1][0] = s;  r.m[1][1] = c;
        return r;
    }

    // Rodrigues rotation around an arbitrary unit axis through origin
    static Transform3D RotationAxis(const Vector3D& unitAxis, double angleRad) noexcept {
        Vector3D a = unitAxis.normalized();
        double c = std::cos(angleRad);
        double s = std::sin(angleRad);
        double t = 1.0 - c;

        Transform3D r;
        r.m[0][0] = t * a.x * a.x + c;
        r.m[0][1] = t * a.x * a.y - s * a.z;
        r.m[0][2] = t * a.x * a.z + s * a.y;

        r.m[1][0] = t * a.x * a.y + s * a.z;
        r.m[1][1] = t * a.y * a.y + c;
        r.m[1][2] = t * a.y * a.z - s * a.x;

        r.m[2][0] = t * a.x * a.z - s * a.y;
        r.m[2][1] = t * a.y * a.z + s * a.x;
        r.m[2][2] = t * a.z * a.z + c;

        return r;
    }

    // Rotation around an arbitrary axis passing through pivot point
    static Transform3D RotationAroundAxis(const Point3D& pivot, const Vector3D& unitAxis, double angleRad) noexcept {
        Vector3D pVec(pivot.x, pivot.y, pivot.z);
        return Translation(pVec) * RotationAxis(unitAxis, angleRad) * Translation(-pVec);
    }

    static Transform3D Scaling(double sx, double sy, double sz) noexcept {
        Transform3D s;
        s.m[0][0] = sx;
        s.m[1][1] = sy;
        s.m[2][2] = sz;
        return s;
    }

    [[nodiscard]] Point3D transform_point(const Point3D& p) const noexcept {
        double x = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
        double y = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
        double z = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
        double w = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];

        if (std::abs(w - 1.0) > CAD_EPSILON && std::abs(w) > CAD_EPSILON) {
            double invW = 1.0 / w;
            return {x * invW, y * invW, z * invW};
        }
        return {x, y, z};
    }

    [[nodiscard]] Vector3D transform_vector(const Vector3D& v) const noexcept {
        // Linear 3x3 portion only (translations do not affect directions)
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    [[nodiscard]] Transform3D operator*(const Transform3D& rhs) const noexcept {
        Transform3D res;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                res.m[i][j] = m[i][0] * rhs.m[0][j] +
                              m[i][1] * rhs.m[1][j] +
                              m[i][2] * rhs.m[2][j] +
                              m[i][3] * rhs.m[3][j];
            }
        }
        return res;
    }

    // Rigid-body inverse (assumes orthogonal upper-left 3x3 rotation + translation)
    [[nodiscard]] Transform3D rigid_inverse() const noexcept {
        Transform3D inv;
        // Transpose 3x3 rotation
        inv.m[0][0] = m[0][0]; inv.m[0][1] = m[1][0]; inv.m[0][2] = m[2][0];
        inv.m[1][0] = m[0][1]; inv.m[1][1] = m[1][1]; inv.m[1][2] = m[2][1];
        inv.m[2][0] = m[0][2]; inv.m[2][1] = m[1][2]; inv.m[2][2] = m[2][2];

        // -R^T * T
        inv.m[0][3] = -(inv.m[0][0] * m[0][3] + inv.m[0][1] * m[1][3] + inv.m[0][2] * m[2][3]);
        inv.m[1][3] = -(inv.m[1][0] * m[0][3] + inv.m[1][1] * m[1][3] + inv.m[1][2] * m[2][3]);
        inv.m[2][3] = -(inv.m[2][0] * m[0][3] + inv.m[2][1] * m[1][3] + inv.m[2][2] * m[2][3]);

        inv.m[3][0] = 0.0; inv.m[3][1] = 0.0; inv.m[3][2] = 0.0; inv.m[3][3] = 1.0;
        return inv;
    }
};

} // namespace Tessera::Geom

