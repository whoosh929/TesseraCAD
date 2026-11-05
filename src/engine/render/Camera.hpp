#pragma once

#include <cmath>
#include <array>

namespace Tessera {

struct alignas(16) CameraUniforms {
    std::array<float, 16> viewProjection;
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

    void SetViewportSize(float width, float height) noexcept {
        if (width > 0.0f && height > 0.0f) {
            m_aspect = width / height;
            UpdateMatrices();
        }
    }

    [[nodiscard]] const CameraUniforms& GetUniforms() const noexcept {
        return m_uniforms;
    }

private:
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
        P[5]  = 1.0f / tanHalfFov;
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
    }

    float m_target[3]{0.0f, 0.0f, 0.0f};
    float m_azimuth{0.785f};    // 45 degrees
    float m_elevation{0.523f};  // 30 degrees
    float m_distance{6.0f};

    float m_fovY{0.785f};       // 45 degrees
    float m_aspect{16.0f / 9.0f};
    float m_nearPlane{0.1f};
    float m_farPlane{1000.0f};

    CameraUniforms m_uniforms{};
};

} // namespace Tessera
