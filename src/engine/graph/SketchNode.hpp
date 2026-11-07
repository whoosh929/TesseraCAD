#pragma once

#include "Node.hpp"
#include "DatumPlaneNode.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/sketch/PolygonBool2D.hpp"
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Tessera {

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

    SketchNode(
        std::string name,
        std::shared_ptr<DatumPlaneNode> datumPlane,
        std::vector<std::array<float, 2>> initialProfile = {}
    ) : DAGNode(std::move(name)),
        m_profile(std::move(initialProfile)) {
        SetDatumPlaneReference(datumPlane);
        if (m_profile.empty()) {
            SetPresetRectangle(3.0f, 2.0f);
        }
    }

    void SetDatumPlaneReference(std::shared_ptr<DatumPlaneNode> planeNode) {
        m_datumPlaneRef = planeNode;
        if (planeNode) {
            AddInput(planeNode);
            m_customPlane = planeNode->GetPlane();
            m_useCustomPlane = true;
            m_position[0] = static_cast<float>(m_customPlane.origin.x);
            m_position[1] = static_cast<float>(m_customPlane.origin.y);
            m_position[2] = static_cast<float>(m_customPlane.origin.z);
        } else {
            m_useCustomPlane = false;
        }
        MarkDirty();
    }

    [[nodiscard]] std::shared_ptr<DatumPlaneNode> GetDatumPlaneReference() const noexcept {
        return m_datumPlaneRef;
    }

    [[nodiscard]] bool HasCustomPlane() const noexcept { return m_useCustomPlane; }
    [[nodiscard]] Geom::Plane GetCustomPlane() const noexcept { return m_customPlane; }

    [[nodiscard]] bool HasConflict() const noexcept override {
        if (m_datumPlaneRef && m_datumPlaneRef->HasConflict()) {
            return true;
        }
        return false;
    }

    [[nodiscard]] std::string GetConflictMessage() const noexcept override {
        if (m_datumPlaneRef && m_datumPlaneRef->HasConflict()) {
            return "Sketch plane conflict: " + m_datumPlaneRef->GetConflictMessage();
        }
        return "";
    }

    [[nodiscard]] SketchPlane GetPlane() const noexcept { return m_plane; }
    void SetPlane(SketchPlane plane) {
        if (m_plane != plane || m_useCustomPlane) {
            m_plane = plane;
            m_useCustomPlane = false;
            m_datumPlaneRef = nullptr;
            MarkDirty();
        }
    }

    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept { return m_profile; }
    void SetProfile(std::vector<std::array<float, 2>> profile) {
        m_profile = std::move(profile);
        m_shapes = { m_profile };
        m_preset = SketchPreset::Custom;
        MarkDirty();
    }

    [[nodiscard]] const std::vector<std::vector<std::array<float, 2>>>& GetShapes() const noexcept {
        if (m_shapes.empty() && !m_profile.empty()) {
            const_cast<SketchNode*>(this)->m_shapes = { m_profile };
        }
        return m_shapes;
    }

    void SetShapes(std::vector<std::vector<std::array<float, 2>>> shapes) {
        m_shapes = std::move(shapes);
        if (!m_shapes.empty()) {
            if (m_shapes.size() == 1) {
                m_profile = m_shapes[0];
            } else {
                auto stitched = Sketch::StitchMultipleShapes(m_shapes);
                m_profile = stitched.empty() ? m_shapes[0] : stitched;
            }
        }
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
        m_shapes = { m_profile };
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
        m_shapes = { m_profile };
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
        m_shapes = { m_profile };
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
        m_shapes = { m_profile };
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
        m_shapes = { m_profile };
        MarkDirty();
    }

    void AddPoint(float x, float y) {
        m_profile.push_back({x, y});
        m_shapes = { m_profile };
        m_preset = SketchPreset::Custom;
        MarkDirty();
    }

    void RemovePoint(size_t index) {
        if (index < m_profile.size() && m_profile.size() > 3) {
            m_profile.erase(m_profile.begin() + index);
            m_shapes = { m_profile };
            m_preset = SketchPreset::Custom;
            MarkDirty();
        }
    }

    void RemoveShape(size_t shapeIndex) {
        if (shapeIndex < m_shapes.size()) {
            m_shapes.erase(m_shapes.begin() + shapeIndex);
            if (!m_shapes.empty()) {
                m_profile = m_shapes[0];
            } else {
                m_profile.clear();
            }
            m_preset = SketchPreset::Custom;
            MarkDirty();
        }
    }

    void RemoveVertexFromShape(size_t shapeIndex, size_t vertexIndex) {
        if (shapeIndex < m_shapes.size()) {
            auto& shape = m_shapes[shapeIndex];
            if (vertexIndex < shape.size() && shape.size() > 3) {
                shape.erase(shape.begin() + vertexIndex);
                if (shapeIndex == 0) {
                    m_profile = shape;
                }
                m_preset = SketchPreset::Custom;
                MarkDirty();
            }
        }
    }

    void UpdatePoint(size_t index, float x, float y) {
        if (index < m_profile.size()) {
            m_profile[index] = {x, y};
            m_shapes = { m_profile };
            m_preset = SketchPreset::Custom;
            MarkDirty();
        }
    }

    [[nodiscard]] bool IsConsumed() const noexcept override { return m_isConsumed; }
    void SetConsumed(bool consumed) noexcept override { m_isConsumed = consumed; }

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

    void MarkDirty() override {
        DAGNode::MarkDirty();
        if (auto parent = m_parentFeature.lock()) {
            parent->MarkDirty();
        }
    }

    [[nodiscard]] bool IsVisible() const noexcept { return m_visible; }
    void SetVisible(bool visible) noexcept { 
        m_visible = visible;
    }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;
        if (!IsDirty() || m_profile.size() < 3) return;

        if (m_datumPlaneRef) {
            m_customPlane = m_datumPlaneRef->GetPlane();
            m_useCustomPlane = true;
            m_position[0] = static_cast<float>(m_customPlane.origin.x);
            m_position[1] = static_cast<float>(m_customPlane.origin.y);
            m_position[2] = static_cast<float>(m_customPlane.origin.z);
        }

        uint64_t currentGen = GetGeneration();
        std::cout << "[SketchNode::Evaluate] Evaluating 2D Sketch '" << m_name 
                  << "' (Gen " << currentGen << " | Points: " << m_profile.size() 
                  << (m_useCustomPlane ? " | Plane: Custom Associative Datum Plane" : (std::string(" | Plane: ") + GetSketchPlaneName(m_plane)))
                  << ")...\n";

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;
        std::vector<uint32_t> featureEdges;

        const auto& shapesToRender = GetShapes();
        for (const auto& poly : shapesToRender) {
            if (poly.size() < 3) continue;

            uint32_t baseIdx = static_cast<uint32_t>(vertices.size());
            const size_t ptCount = poly.size();

            for (size_t i = 0; i < ptCount; ++i) {
                float u = poly[i][0];
                float v = poly[i][1];
                CADVertex vert{};

                if (m_useCustomPlane) {
                    Geom::Point3D pt3D = m_customPlane.to_world(Geom::Point2D(u, v));
                    vert.position[0] = static_cast<float>(pt3D.x);
                    vert.position[1] = static_cast<float>(pt3D.y);
                    vert.position[2] = static_cast<float>(pt3D.z);
                    vert.normal[0] = static_cast<float>(m_customPlane.normal.x);
                    vert.normal[1] = static_cast<float>(m_customPlane.normal.y);
                    vert.normal[2] = static_cast<float>(m_customPlane.normal.z);
                } else {
                    switch (m_plane) {
                        case SketchPlane::XZ_Ground:
                            vert.position[0] = u + m_position[0];
                            vert.position[1] = m_position[1] + 0.005f;
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
                            vert.normal[0] = 0.0f;
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
                }
                vertices.push_back(vert);

                featureEdges.push_back(baseIdx + static_cast<uint32_t>(i));
                featureEdges.push_back(baseIdx + static_cast<uint32_t>((i + 1) % ptCount));
            }

            auto polyIndices = TessellationUtils::TriangulatePolygon2D(poly);
            for (uint32_t idx : polyIndices) {
                indices.push_back(baseIdx + idx);
            }
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
    std::vector<std::vector<std::array<float, 2>>> m_shapes;
    bool m_visible{true};
    std::weak_ptr<DAGNode> m_parentFeature;

    std::shared_ptr<DatumPlaneNode> m_datumPlaneRef{nullptr};
    Geom::Plane m_customPlane;
    bool m_useCustomPlane{false};
};

} // namespace Tessera
