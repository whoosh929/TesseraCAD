#pragma once

#include "engine/geom/Transform3D.hpp"
#include <cmath>
#include <array>
#include <algorithm>

namespace Tessera {

struct Ray {
    Geom::Point3D origin;
    Geom::Vector3D direction;
};

struct alignas(16) CameraUniforms {
    std::array<float, 16> viewProjection;
    std::array<float, 16> invViewProjection;
    std::array<float, 4> eyePosition;
};

class Camera {
public:
    Camera() {
        UpdateMatrices();
    }

    void Orbit(float deltaAzimuth, float deltaElevation) noexcept {
        m_azimuth += deltaAzimuth;
        m_elevation = std::clamp(m_elevation + deltaElevation, -1.5f, 1.5f);
        UpdateMatrices();
    }

    void Pan(float deltaX, float deltaY) noexcept {
        float forward[3], right[3], up[3];
        GetBasisVectors(forward, right, up);

        m_target[0] += right[0] * deltaX + up[0] * deltaY;
        m_target[1] += right[1] * deltaX + up[1] * deltaY;
        m_target[2] += right[2] * deltaX + up[2] * deltaY;
        UpdateMatrices();
    }

    void Zoom(float deltaDistance) noexcept {
        m_distance = std::max(0.1f, m_distance - deltaDistance);
        UpdateMatrices();
    }

    /// SOLIDWORKS-style cursor-centered zoom:
    /// Rolls scroll wheel up (scrollDelta > 0) to zoom IN, down (scrollDelta < 0) to zoom OUT centered on the cursor.
    /// When smooth=true (Shift key held), uses a finer zoom factor.
    void ZoomCentered(
        float screenX, 
        float screenY, 
        float viewportWidth, 
        float viewportHeight, 
        float scrollDelta, 
        bool smooth = false
    ) noexcept {
        if (viewportWidth <= 0.0f || viewportHeight <= 0.0f || std::abs(scrollDelta) < 1e-4f) return;

        // SOLIDWORKS: scroll up (delta > 0) zooms IN (factor < 1); scroll down (delta < 0) zooms OUT (factor > 1).
        float baseFactor = smooth ? 0.96f : 0.85f;
        float factor = (scrollDelta > 0.0f) ? std::pow(baseFactor, std::abs(scrollDelta)) 
                                            : std::pow(1.0f / baseFactor, std::abs(scrollDelta));

        float oldDist = m_distance;
        float newDist = std::clamp(oldDist * factor, 0.05f, 2000.0f);
        if (std::abs(newDist - oldDist) < 1e-5f) return;

        // Compute 3D ray through cursor
        Ray ray = ScreenPointToRay(screenX, screenY, viewportWidth, viewportHeight);

        float forward[3], right[3], up[3];
        GetBasisVectors(forward, right, up);
        Geom::Vector3D camFwd(forward[0], forward[1], forward[2]);

        double denom = ray.direction.dot(camFwd);
        if (std::abs(denom) > 1e-4) {
            Geom::Point3D targetPt(m_target[0], m_target[1], m_target[2]);
            double t = (targetPt - ray.origin).dot(camFwd) / denom;
            if (t > 0.0) {
                Geom::Point3D cursorWorldPt = ray.origin + ray.direction * t;
                double ratio = static_cast<double>(newDist / oldDist);
                Geom::Vector3D shift = (cursorWorldPt - targetPt) * (1.0 - ratio);

                m_target[0] += static_cast<float>(shift.x);
                m_target[1] += static_cast<float>(shift.y);
                m_target[2] += static_cast<float>(shift.z);
            }
        }

        m_distance = newDist;
        UpdateMatrices();
    }

    [[nodiscard]] float GetDistance() const noexcept { return m_distance; }
    [[nodiscard]] const float* GetTarget() const noexcept { return m_target; }

    void SetViewportSize(float width, float height) noexcept {
        if (width > 0.0f && height > 0.0f) {
            m_aspect = width / height;
            UpdateMatrices();
        }
    }

    [[nodiscard]] const CameraUniforms& GetUniforms() const noexcept {
        return m_uniforms;
    }

    [[nodiscard]] Ray ScreenPointToRay(float screenX, float screenY, float viewportWidth, float viewportHeight) const noexcept {
        if (viewportWidth <= 0.0f || viewportHeight <= 0.0f) {
            return {Geom::Point3D(m_uniforms.eyePosition[0], m_uniforms.eyePosition[1], m_uniforms.eyePosition[2]),
                    Geom::Vector3D(0.0, 0.0, -1.0)};
        }

        // Convert window pixel coords (0..W, 0..H) to Vulkan NDC (-1..1, -1..1)
        float ndcX = (2.0f * screenX) / viewportWidth - 1.0f;
        float ndcY = (2.0f * screenY) / viewportHeight - 1.0f;

        const float* inv = m_uniforms.invViewProjection.data();

        // Unproject near point (NDC z = 0.0f)
        float nearX = inv[0] * ndcX + inv[4] * ndcY + inv[8] * 0.0f + inv[12] * 1.0f;
        float nearY = inv[1] * ndcX + inv[5] * ndcY + inv[9] * 0.0f + inv[13] * 1.0f;
        float nearZ = inv[2] * ndcX + inv[6] * ndcY + inv[10] * 0.0f + inv[14] * 1.0f;
        float nearW = inv[3] * ndcX + inv[7] * ndcY + inv[11] * 0.0f + inv[15] * 1.0f;

        // Unproject far point (NDC z = 1.0f)
        float farX = inv[0] * ndcX + inv[4] * ndcY + inv[8] * 1.0f + inv[12] * 1.0f;
        float farY = inv[1] * ndcX + inv[5] * ndcY + inv[9] * 1.0f + inv[13] * 1.0f;
        float farZ = inv[2] * ndcX + inv[6] * ndcY + inv[10] * 1.0f + inv[14] * 1.0f;
        float farW = inv[3] * ndcX + inv[7] * ndcY + inv[11] * 1.0f + inv[15] * 1.0f;

        if (std::abs(nearW) > 1e-7f) {
            nearX /= nearW;
            nearY /= nearW;
            nearZ /= nearW;
        }

        if (std::abs(farW) > 1e-7f) {
            farX /= farW;
            farY /= farW;
            farZ /= farW;
        }

        Geom::Point3D eye(m_uniforms.eyePosition[0], m_uniforms.eyePosition[1], m_uniforms.eyePosition[2]);
        Geom::Vector3D dir = Geom::Vector3D(farX - eye.x, farY - eye.y, farZ - eye.z).normalized();

        return {eye, dir};
    }

    [[nodiscard]] bool WorldToScreenPoint(
        const Geom::Point3D& worldPt, 
        float viewportWidth, 
        float viewportHeight, 
        float& outScreenX, 
        float& outScreenY
    ) const noexcept {
        if (viewportWidth <= 0.0f || viewportHeight <= 0.0f) return false;
        const float* m = m_uniforms.viewProjection.data();
        float x = static_cast<float>(worldPt.x);
        float y = static_cast<float>(worldPt.y);
        float z = static_cast<float>(worldPt.z);

        // Column-major P * V: clip = M * [x, y, z, 1]^T
        float clipX = m[0] * x + m[4] * y + m[8]  * z + m[12];
        float clipY = m[1] * x + m[5] * y + m[9]  * z + m[13];
        float clipZ = m[2] * x + m[6] * y + m[10] * z + m[14];
        float clipW = m[3] * x + m[7] * y + m[11] * z + m[15];

        if (clipW <= 1e-4f) return false; // Behind camera eye plane

        float ndcX = clipX / clipW;
        float ndcY = clipY / clipW;

        outScreenX = (ndcX + 1.0f) * 0.5f * viewportWidth;
        outScreenY = (ndcY + 1.0f) * 0.5f * viewportHeight;

        (void)clipZ;
        return (ndcX >= -1.5f && ndcX <= 1.5f && ndcY >= -1.5f && ndcY <= 1.5f);
    }

    void RecenterOrigin() noexcept {
        m_target[0] = 0.0f;
        m_target[1] = 0.0f;
        m_target[2] = 0.0f;
        UpdateMatrices();
    }

    void SetViewPreset(float azimuth, float elevation, bool recenterOrigin = true) noexcept {
        m_azimuth = azimuth;
        m_elevation = std::clamp(elevation, -1.5f, 1.5f);
        if (recenterOrigin) {
            m_target[0] = 0.0f;
            m_target[1] = 0.0f;
            m_target[2] = 0.0f;
        }
        UpdateMatrices();
    }

    void SetTopView(bool recenter = true) noexcept {
        SetViewPreset(0.0f, 1.5f, recenter);
    }

    void SetSideView(bool recenter = true) noexcept {
        SetViewPreset(1.570796f, 0.0f, recenter);
    }

    void SetFrontView(bool recenter = true) noexcept {
        SetViewPreset(0.0f, 0.0f, recenter);
    }

    void SetIsometricView(bool recenter = true) noexcept {
        SetViewPreset(0.785398f, 0.523599f, recenter);
    }

    [[nodiscard]] float GetAzimuth() const noexcept { return m_azimuth; }
    [[nodiscard]] float GetElevation() const noexcept { return m_elevation; }

    void GetBasisVectors(float forward[3], float right[3], float up[3]) const noexcept {
        float cosElev = std::cos(m_elevation);
        float sinElev = std::sin(m_elevation);
        float cosAzim = std::cos(m_azimuth);
        float sinAzim = std::sin(m_azimuth);

        forward[0] = -cosElev * sinAzim;
        forward[1] = -sinElev;
        forward[2] = -cosElev * cosAzim;

        right[0] = cosAzim;
        right[1] = 0.0f;
        right[2] = -sinAzim;

        up[0] = -sinElev * sinAzim;
        up[1] = cosElev;
        up[2] = -sinElev * cosAzim;
    }

private:

    void UpdateMatrices() noexcept {
        float cosElev = std::cos(m_elevation);
        float sinElev = std::sin(m_elevation);
        float cosAzim = std::cos(m_azimuth);
        float sinAzim = std::sin(m_azimuth);

        float eyeX = m_target[0] + m_distance * cosElev * sinAzim;
        float eyeY = m_target[1] + m_distance * sinElev;
        float eyeZ = m_target[2] + m_distance * cosElev * cosAzim;

        m_uniforms.eyePosition = {eyeX, eyeY, eyeZ, 1.0f};

        // Compute LookAt View Matrix (column-major)
        float f[3] = {m_target[0] - eyeX, m_target[1] - eyeY, m_target[2] - eyeZ};
        float fLen = std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
        f[0] /= fLen; f[1] /= fLen; f[2] /= fLen;

        float up[3] = {0.0f, 1.0f, 0.0f};
        float s[3] = {f[1]*up[2] - f[2]*up[1], f[2]*up[0] - f[0]*up[2], f[0]*up[1] - f[1]*up[0]};
        float sLen = std::sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
        s[0] /= sLen; s[1] /= sLen; s[2] /= sLen;

        float u[3] = {s[1]*f[2] - s[2]*f[1], s[2]*f[0] - s[0]*f[2], s[0]*f[1] - s[1]*f[0]};

        std::array<float, 16> V = {
             s[0],  u[0], -f[0], 0.0f,
             s[1],  u[1], -f[1], 0.0f,
             s[2],  u[2], -f[2], 0.0f,
            -(s[0]*eyeX + s[1]*eyeY + s[2]*eyeZ),
            -(u[0]*eyeX + u[1]*eyeY + u[2]*eyeZ),
             (f[0]*eyeX + f[1]*eyeY + f[2]*eyeZ),
             1.0f
        };

        // Perspective projection matrix (Vulkan & Metal depth range [0, 1])
        float tanHalfFov = std::tan(m_fovY * 0.5f);
        float nearP = m_nearPlane;
        float farP = m_farPlane;

        std::array<float, 16> P = {0.0f};
        P[0]  = 1.0f / (m_aspect * tanHalfFov);
        P[5]  = -1.0f / tanHalfFov; // Vulkan NDC Y points downwards; negate to keep world +Y pointing up
        P[10] = farP / (nearP - farP);
        P[11] = -1.0f;
        P[14] = -(farP * nearP) / (farP - nearP);

        // Multiply P * V (column-major: result = P * V)
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += P[row + k * 4] * V[k + col * 4];
                }
                m_uniforms.viewProjection[row + col * 4] = sum;
            }
        }

        // Invert viewProjection for infinite ground plane ray unprojection
        const float* m = m_uniforms.viewProjection.data();
        float inv[16];

        inv[0] = m[5]  * m[10] * m[15] - m[5]  * m[11] * m[14] - m[9]  * m[6]  * m[15] + m[9]  * m[7]  * m[14] + m[13] * m[6]  * m[11] - m[13] * m[7]  * m[10];
        inv[4] = -m[4]  * m[10] * m[15] + m[4]  * m[11] * m[14] + m[8]  * m[6]  * m[15] - m[8]  * m[7]  * m[14] - m[12] * m[6]  * m[11] + m[12] * m[7]  * m[10];
        inv[8] = m[4]  * m[9]  * m[15] - m[4]  * m[11] * m[13] - m[8]  * m[5]  * m[15] + m[8]  * m[7]  * m[13] + m[12] * m[5]  * m[11] - m[12] * m[7]  * m[9];
        inv[12] = -m[4]  * m[9]  * m[14] + m[4]  * m[10] * m[13] + m[8]  * m[5]  * m[14] - m[8]  * m[6]  * m[13] - m[12] * m[5]  * m[10] + m[12] * m[6]  * m[9];
        inv[1] = -m[1]  * m[10] * m[15] + m[1]  * m[11] * m[14] + m[9]  * m[2]  * m[15] - m[9]  * m[3]  * m[14] - m[13] * m[2]  * m[11] + m[13] * m[3]  * m[10];
        inv[5] = m[0]  * m[10] * m[15] - m[0]  * m[11] * m[14] - m[8]  * m[2]  * m[15] + m[8]  * m[3]  * m[14] + m[12] * m[2]  * m[11] - m[12] * m[3]  * m[10];
        inv[9] = -m[0]  * m[9]  * m[15] + m[0]  * m[11] * m[13] + m[8]  * m[1]  * m[15] - m[8]  * m[3]  * m[13] - m[12] * m[1]  * m[11] + m[12] * m[3]  * m[9];
        inv[13] = m[0]  * m[9]  * m[14] - m[0]  * m[10] * m[13] - m[8]  * m[1]  * m[14] + m[8]  * m[2]  * m[13] + m[12] * m[1]  * m[10] - m[12] * m[2]  * m[9];
        inv[2] = m[1]  * m[6]  * m[15] - m[1]  * m[7]  * m[14] - m[5]  * m[2]  * m[15] + m[5]  * m[3]  * m[14] + m[13] * m[2]  * m[7]  - m[13] * m[3]  * m[6];
        inv[6] = -m[0]  * m[6]  * m[15] + m[0]  * m[7]  * m[14] + m[4]  * m[2]  * m[15] - m[4]  * m[3]  * m[14] - m[12] * m[2]  * m[7]  + m[12] * m[3]  * m[6];
        inv[10] = m[0]  * m[5]  * m[15] - m[0]  * m[7]  * m[13] - m[4]  * m[1]  * m[15] + m[4]  * m[3]  * m[13] + m[12] * m[1]  * m[7]  - m[12] * m[3]  * m[5];
        inv[14] = -m[0]  * m[5]  * m[14] + m[0]  * m[6]  * m[13] + m[4]  * m[1]  * m[14] - m[4]  * m[2]  * m[13] - m[12] * m[1]  * m[6]  + m[12] * m[2]  * m[5];
        inv[3] = -m[1]  * m[6]  * m[11] + m[1]  * m[7]  * m[10] + m[5]  * m[2]  * m[11] - m[5]  * m[3]  * m[10] - m[9]  * m[2]  * m[7]  + m[9]  * m[3]  * m[6];
        inv[7] = m[0]  * m[6]  * m[11] - m[0]  * m[7]  * m[10] - m[4]  * m[2]  * m[11] + m[4]  * m[3]  * m[10] + m[8]  * m[2]  * m[7]  - m[8]  * m[3]  * m[6];
        inv[11] = -m[0]  * m[5]  * m[11] + m[0]  * m[7]  * m[9]  + m[4]  * m[1]  * m[11] - m[4]  * m[3]  * m[9]  - m[8]  * m[1]  * m[7]  + m[8]  * m[3]  * m[5];
        inv[15] = m[0]  * m[5]  * m[10] - m[0]  * m[6]  * m[9]  - m[4]  * m[1]  * m[10] + m[4]  * m[2]  * m[9]  + m[8]  * m[1]  * m[6]  - m[8]  * m[2]  * m[5];

        float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        if (std::abs(det) > 1e-8f) {
            float invDet = 1.0f / det;
            for (int i = 0; i < 16; ++i) {
                m_uniforms.invViewProjection[i] = inv[i] * invDet;
            }
        }
    }

    float m_target[3]{0.0f, 0.0f, 0.0f};
    float m_azimuth{0.785f};    // 45 degrees
    float m_elevation{0.523f};  // 30 degrees
    float m_distance{10.0f};

    float m_fovY{0.785f};       // 45 degrees
    float m_aspect{16.0f / 9.0f};
    float m_nearPlane{0.1f};
    float m_farPlane{1000.0f};

    CameraUniforms m_uniforms{};
};

} // namespace Tessera

