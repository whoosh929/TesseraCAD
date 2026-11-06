#pragma once

#include "Node.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Tessera {

enum class SketchPlane : uint8_t {
    XZ_Ground = 0, // Top/Horizontal datum plane (Normal +Y)
    XY_Front  = 1, // Front datum plane (Normal +Z)
    YZ_Right  = 2  // Right datum plane (Normal +X)
};

inline const char* GetSketchPlaneName(SketchPlane plane) noexcept {
    switch (plane) {
        case SketchPlane::XZ_Ground: return "XZ Ground Plane (Top)";
        case SketchPlane::XY_Front:  return "XY Front Plane";
        case SketchPlane::YZ_Right:  return "YZ Right Plane";
    }
    return "Unknown Plane";
}

enum class SketchPreset : uint8_t {
    Rectangle = 0,
    RegularPolygon = 1,
    LBracket = 2,
    Star = 3,
    TBeam = 4,
    Custom = 5
};

class SketchNode : public DAGNode {
public:
    SketchNode(
        std::string name,
        SketchPlane plane = SketchPlane::XZ_Ground,
        std::vector<std::array<float, 2>> initialProfile = {}
    ) : DAGNode(std::move(name)),
        m_plane(plane),
        m_profile(std::move(initialProfile)) {
        if (m_profile.empty()) {
            SetPresetRectangle(3.0f, 2.0f);
        }
    }

    [[nodiscard]] SketchPlane GetPlane() const noexcept { return m_plane; }
    void SetPlane(SketchPlane plane) {
        if (m_plane != plane) {
            m_plane = plane;
            MarkDirty();
        }
    }

    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept { return m_profile; }
    void SetProfile(std::vector<std::array<float, 2>> profile) {
        m_profile = std::move(profile);
        m_preset = SketchPreset::Custom;
        MarkDirty();
    }

    [[nodiscard]] SketchPreset GetPreset() const noexcept { return m_preset; }

    void SetPresetRectangle(float width, float height) {
        m_preset = SketchPreset::Rectangle;
        float hw = width * 0.5f;
        float hh = height * 0.5f;
        m_profile = {
            {-hw, -hh},
            { hw, -hh},
            { hw,  hh},
            {-hw,  hh}
        };
        MarkDirty();
    }

    void SetPresetRegularPolygon(uint32_t sides, float radius) {
        m_preset = SketchPreset::RegularPolygon;
        sides = std::clamp(sides, 3u, 32u);
        m_profile.clear();
        for (uint32_t i = 0; i < sides; ++i) {
            float angle = (static_cast<float>(i) / static_cast<float>(sides)) * 6.2831853f;
            m_profile.push_back({radius * std::cos(angle), radius * std::sin(angle)});
        }
        MarkDirty();
    }

    void SetPresetLBracket(float sizeX, float sizeY, float thickness) {
        m_preset = SketchPreset::LBracket;
        float hx = sizeX * 0.5f;
        float hy = sizeY * 0.5f;
        float t = std::min(thickness, std::min(sizeX, sizeY) * 0.8f);
        m_profile = {
            {-hx, -hy},
            { hx, -hy},
            { hx, -hy + t},
            {-hx + t, -hy + t},
            {-hx + t,  hy},
            {-hx,  hy}
        };
        MarkDirty();
    }

    void SetPresetStar(uint32_t points, float outerRadius, float innerRadius) {
        m_preset = SketchPreset::Star;
        points = std::clamp(points, 3u, 16u);
        m_profile.clear();
        for (uint32_t i = 0; i < points * 2; ++i) {
            float r = (i % 2 == 0) ? outerRadius : innerRadius;
            float angle = (static_cast<float>(i) / static_cast<float>(points * 2)) * 6.2831853f;
            m_profile.push_back({r * std::cos(angle), r * std::sin(angle)});
        }
        MarkDirty();
    }

    void SetPresetTBeam(float flangeWidth, float webHeight, float thickness) {
        m_preset = SketchPreset::TBeam;
        float hx = flangeWidth * 0.5f;
        float ht = thickness * 0.5f;
        float hy = webHeight * 0.5f;
        m_profile = {
            {-hx,  hy},
            { hx,  hy},
            { hx,  hy - thickness},
            { ht,  hy - thickness},
            { ht, -hy},
            {-ht, -hy},
            {-ht,  hy - thickness},
            {-hx,  hy - thickness}
        };
        MarkDirty();
    }

    void AddPoint(float x, float y) {
        m_profile.push_back({x, y});
        m_preset = SketchPreset::Custom;
        MarkDirty();
    }

    void RemovePoint(size_t index) {
        if (index < m_profile.size() && m_profile.size() > 3) {
            m_profile.erase(m_profile.begin() + index);
            m_preset = SketchPreset::Custom;
            MarkDirty();
        }
    }

    void UpdatePoint(size_t index, float x, float y) {
        if (index < m_profile.size()) {
            m_profile[index] = {x, y};
            m_preset = SketchPreset::Custom;
            MarkDirty();
        }
    }

    [[nodiscard]] bool IsConsumed() const noexcept { return m_isConsumed; }
    void SetConsumed(bool consumed) noexcept { m_isConsumed = consumed; }

    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override {
        return m_parentFeature.lock();
    }
    void SetParentFeature(std::shared_ptr<DAGNode> parent) noexcept {
        m_parentFeature = parent;
        m_isConsumed = (parent != nullptr);
    }
    [[nodiscard]] bool HasParentFeature() const noexcept override {
        return !m_parentFeature.expired();
    }

    [[nodiscard]] bool IsVisible() const noexcept { return m_visible; }
    void SetVisible(bool visible) noexcept { 
        m_visible = visible;
        MarkDirty();
    }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;
        if (!IsDirty() || m_profile.size() < 3) return;

        uint64_t currentGen = GetGeneration();
        std::cout << "[SketchNode::Evaluate] Evaluating 2D Sketch '" << m_name 
                  << "' (Gen " << currentGen << " | Points: " << m_profile.size() 
                  << ", Plane: " << GetSketchPlaneName(m_plane) << ")...\n";

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;
        std::vector<uint32_t> featureEdges;

        if (m_visible) {
            const size_t ptCount = m_profile.size();
            vertices.reserve(ptCount);

            // 1. Generate 3D vertices mapped to sketch plane
            for (size_t i = 0; i < ptCount; ++i) {
                float u = m_profile[i][0];
                float v = m_profile[i][1];
                CADVertex vert{};

                switch (m_plane) {
                    case SketchPlane::XZ_Ground:
                        vert.position[0] = u + m_position[0];
                        vert.position[1] = m_position[1] + 0.005f; // Slight elevation
                        vert.position[2] = v + m_position[2];
                        vert.normal[0] = 0.0f;
                        vert.normal[1] = 1.0f;
                        vert.normal[2] = 0.0f;
                        break;
                    case SketchPlane::XY_Front:
                        vert.position[0] = u + m_position[0];
                        vert.position[1] = v + m_position[1];
                        vert.position[2] = m_position[2] + 0.005f;
                        vert.normal[0] = 0.0f;
                        vert.normal[1] = 0.0f;
                        vert.normal[2] = 1.0f;
                        break;
                    case SketchPlane::YZ_Right:
                        vert.position[0] = m_position[0] + 0.005f;
                        vert.position[1] = v + m_position[1];
                        vert.position[2] = u + m_position[2];
                        vert.normal[0] = 1.0f;
                        vert.normal[1] = 0.0f;
                        vert.normal[2] = 0.0f;
                        break;
                }
                vertices.push_back(vert);

                // Closed boundary wireframe edge loop
                featureEdges.push_back(static_cast<uint32_t>(i));
                featureEdges.push_back(static_cast<uint32_t>((i + 1) % ptCount));
            }

            // 2. Triangulate planar face for semi-translucent surface fill
            indices = TessellationUtils::TriangulatePolygon2D(m_profile);
        }

        m_cachedVertices = vertices;
        m_cachedIndices = indices;
        m_cachedEdges = featureEdges;

        ClearDirty();
    }

private:
    SketchPlane m_plane{SketchPlane::XZ_Ground};
    SketchPreset m_preset{SketchPreset::Rectangle};
    std::vector<std::array<float, 2>> m_profile;
    bool m_isConsumed{false};
    bool m_visible{true};
    std::weak_ptr<DAGNode> m_parentFeature;
};

} // namespace Tessera
