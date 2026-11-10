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
#include <map>

namespace Tessera {

enum class SketchPreset : uint8_t {
    Rectangle = 0,
    RegularPolygon = 1,
    LBracket = 2,
    Star = 3,
    TBeam = 4,
    Custom = 5
};

struct VertexAxisConstraint {
    bool lockOriginDist{false};
    float originDist{0.0f};

    bool lockUAxisDist{false}; // distance to U-Axis (|V|)
    float uAxisDist{0.0f};

    bool lockVAxisDist{false}; // distance to V-Axis (|U|)
    float vAxisDist{0.0f};

    bool lockDatumAxisDist{false}; // distance to reference DatumAxis line
    float datumAxisDist{0.0f};
    uint64_t datumAxisId{0};
    std::string datumAxisName;

    [[nodiscard]] bool HasAnyConstraint() const noexcept {
        return lockOriginDist || lockUAxisDist || lockVAxisDist || lockDatumAxisDist;
    }
};

struct ProjectedDatumAxis {
    uint64_t id{0};
    std::string name;
    std::array<float, 2> p0{0.0f, 0.0f};
    std::array<float, 2> dir{1.0f, 0.0f};
    bool valid{false};
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
        if (!m_profile.empty()) {
            m_shapes = { m_profile };
            m_isClosed = (m_profile.size() >= 3);
            m_preset = SketchPreset::Custom;
        } else {
            m_preset = SketchPreset::Custom;
        }
    }

    SketchNode(
        std::string name,
        std::shared_ptr<DatumPlaneNode> datumPlane,
        std::vector<std::array<float, 2>> initialProfile = {}
    ) : DAGNode(std::move(name)),
        m_profile(std::move(initialProfile)) {
        SetDatumPlaneReference(datumPlane);
        if (!m_profile.empty()) {
            m_shapes = { m_profile };
            m_isClosed = (m_profile.size() >= 3);
            m_preset = SketchPreset::Custom;
        } else {
            m_preset = SketchPreset::Custom;
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

    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept {
        if (m_profile.empty() && !m_shapes.empty()) {
            const_cast<SketchNode*>(this)->m_profile = m_shapes[0];
        }
        return m_profile;
    }
    void SetProfile(std::vector<std::array<float, 2>> profile, bool isClosed = true) {
        m_profile = std::move(profile);
        m_shapes = { m_profile };
        m_isClosed = isClosed && (m_profile.size() >= 3);
        m_shapeClosed = { m_isClosed };
        m_preset = SketchPreset::Custom;
        MarkDirty();
    }

    [[nodiscard]] const std::vector<std::vector<std::array<float, 2>>>& GetShapes() const noexcept {
        if (m_shapes.empty() && !m_profile.empty()) {
            const_cast<SketchNode*>(this)->m_shapes = { m_profile };
        }
        return m_shapes;
    }

    void SetShapes(std::vector<std::vector<std::array<float, 2>>> shapes, std::vector<bool> shapeClosed = {}) {
        m_shapes = std::move(shapes);
        m_shapeClosed = std::move(shapeClosed);
        if (m_shapeClosed.size() < m_shapes.size()) {
            m_shapeClosed.resize(m_shapes.size(), m_isClosed);
        }
        if (!m_shapes.empty()) {
            if (m_shapes.size() == 1) {
                m_profile = m_shapes[0];
            } else {
                auto stitched = Sketch::StitchMultipleShapes(m_shapes);
                m_profile = stitched.empty() ? m_shapes[0] : stitched;
            }
            m_isClosed = true;
            for (bool c : m_shapeClosed) {
                if (!c) {
                    m_isClosed = false;
                    break;
                }
            }
        }
        m_preset = SketchPreset::Custom;
        MarkDirty();
    }

    [[nodiscard]] const std::vector<bool>& GetShapeClosed() const noexcept {
        return m_shapeClosed;
    }
    [[nodiscard]] bool IsShapeClosed(size_t index) const noexcept {
        if (index < m_shapeClosed.size()) {
            return m_shapeClosed[index];
        }
        return m_isClosed;
    }

    [[nodiscard]] bool IsClosed() const noexcept { return m_isClosed; }
    void SetClosed(bool closed) noexcept {
        m_isClosed = closed;
        for (size_t i = 0; i < m_shapeClosed.size(); ++i) {
            m_shapeClosed[i] = closed;
        }
        if (m_shapeClosed.empty()) {
            m_shapeClosed = { closed };
        }
        MarkDirty();
    }

    [[nodiscard]] bool IsEnclosed() const noexcept {
        if (!m_isClosed) return false;
        for (bool c : m_shapeClosed) {
            if (!c) return false;
        }
        const auto& shs = GetShapes();
        if (!shs.empty()) {
            return shs[0].size() >= 3;
        }
        return m_profile.size() >= 3;
    }

    [[nodiscard]] bool IsSingleLine() const noexcept {
        const auto& shs = GetShapes();
        if (!shs.empty()) {
            return shs.size() == 1 && shs[0].size() == 2;
        }
        return m_profile.size() == 2;
    }

    [[nodiscard]] bool IsPathSketch() const noexcept {
        return IsSingleLine() || !IsEnclosed();
    }

    [[nodiscard]] const std::map<size_t, std::map<size_t, float>>& GetEdgeConstraints() const noexcept {
        return m_shapeEdgeConstraints;
    }
    void SetEdgeConstraints(std::map<size_t, std::map<size_t, float>> constraints) {
        m_shapeEdgeConstraints = std::move(constraints);
        MarkDirty();
    }
    void SetEdgeConstraint(size_t shapeIdx, size_t edgeIdx, float length) {
        if (length > 0.0001f) {
            m_shapeEdgeConstraints[shapeIdx][edgeIdx] = length;
            MarkDirty();
        }
    }
    void RemoveEdgeConstraint(size_t shapeIdx, size_t edgeIdx) {
        auto it = m_shapeEdgeConstraints.find(shapeIdx);
        if (it != m_shapeEdgeConstraints.end()) {
            it->second.erase(edgeIdx);
            if (it->second.empty()) {
                m_shapeEdgeConstraints.erase(it);
            }
            MarkDirty();
        }
    }
    [[nodiscard]] bool IsEdgeConstrained(size_t shapeIdx, size_t edgeIdx) const {
        auto it = m_shapeEdgeConstraints.find(shapeIdx);
        if (it == m_shapeEdgeConstraints.end()) return false;
        return it->second.find(edgeIdx) != it->second.end();
    }
    [[nodiscard]] float GetEdgeConstrainedLength(size_t shapeIdx, size_t edgeIdx) const {
        auto it = m_shapeEdgeConstraints.find(shapeIdx);
        if (it == m_shapeEdgeConstraints.end()) return 0.0f;
        auto edgeIt = it->second.find(edgeIdx);
        return (edgeIt != it->second.end()) ? edgeIt->second : 0.0f;
    }
    void ClearEdgeConstraints() {
        m_shapeEdgeConstraints.clear();
        MarkDirty();
    }

    [[nodiscard]] const std::map<size_t, std::map<size_t, VertexAxisConstraint>>& GetVertexAxisConstraints() const noexcept {
        return m_shapeVertexAxisConstraints;
    }
    void SetVertexAxisConstraints(std::map<size_t, std::map<size_t, VertexAxisConstraint>> constraints) {
        m_shapeVertexAxisConstraints = std::move(constraints);
        MarkDirty();
    }
    void SetVertexOriginConstraint(size_t shapeIdx, size_t vIdx, float dist) {
        if (dist > 0.0001f) {
            m_shapeVertexAxisConstraints[shapeIdx][vIdx].lockOriginDist = true;
            m_shapeVertexAxisConstraints[shapeIdx][vIdx].originDist = dist;
            MarkDirty();
        }
    }
    void RemoveVertexOriginConstraint(size_t shapeIdx, size_t vIdx) {
        auto it = m_shapeVertexAxisConstraints.find(shapeIdx);
        if (it != m_shapeVertexAxisConstraints.end()) {
            it->second[vIdx].lockOriginDist = false;
            it->second[vIdx].originDist = 0.0f;
            if (!it->second[vIdx].HasAnyConstraint()) {
                it->second.erase(vIdx);
            }
            if (it->second.empty()) {
                m_shapeVertexAxisConstraints.erase(it);
            }
            MarkDirty();
        }
    }
    [[nodiscard]] bool IsVertexOriginConstrained(size_t shapeIdx, size_t vIdx) const {
        auto it = m_shapeVertexAxisConstraints.find(shapeIdx);
        if (it == m_shapeVertexAxisConstraints.end()) return false;
        auto vIt = it->second.find(vIdx);
        return vIt != it->second.end() && vIt->second.lockOriginDist;
    }
    [[nodiscard]] float GetVertexOriginDistance(size_t shapeIdx, size_t vIdx) const {
        auto it = m_shapeVertexAxisConstraints.find(shapeIdx);
        if (it == m_shapeVertexAxisConstraints.end()) return 0.0f;
        auto vIt = it->second.find(vIdx);
        return (vIt != it->second.end() && vIt->second.lockOriginDist) ? vIt->second.originDist : 0.0f;
    }
    void ClearVertexAxisConstraints() {
        m_shapeVertexAxisConstraints.clear();
        MarkDirty();
    }

    [[nodiscard]] Geom::Plane GetEffectivePlane() const noexcept {
        if (m_useCustomPlane) {
            return m_customPlane;
        }
        switch (m_plane) {
            case SketchPlane::XZ_Ground:
                return Geom::Plane(
                    Geom::Point3D(m_position[0], m_position[1], m_position[2]),
                    Geom::Vector3D(0.0, 1.0, 0.0), // Normal: +Y
                    Geom::Vector3D(1.0, 0.0, 0.0), // U: +X
                    Geom::Vector3D(0.0, 0.0, 1.0)  // V: +Z
                );
            case SketchPlane::XY_Front:
                return Geom::Plane(
                    Geom::Point3D(m_position[0], m_position[1], m_position[2]),
                    Geom::Vector3D(0.0, 0.0, 1.0), // Normal: +Z
                    Geom::Vector3D(1.0, 0.0, 0.0), // U: +X
                    Geom::Vector3D(0.0, 1.0, 0.0)  // V: +Y
                );
            case SketchPlane::YZ_Right:
                return Geom::Plane(
                    Geom::Point3D(m_position[0], m_position[1], m_position[2]),
                    Geom::Vector3D(1.0, 0.0, 0.0), // Normal: +X
                    Geom::Vector3D(0.0, 0.0, 1.0), // U: +Z
                    Geom::Vector3D(0.0, 1.0, 0.0)  // V: +Y
                );
        }
        return Geom::Plane::XY();
    }

    static void EnforceConstraints(
        std::vector<std::array<float, 2>>& points,
        const std::map<size_t, float>& edgeConstraints,
        const std::map<size_t, VertexAxisConstraint>& vertexConstraints,
        bool isClosed = true,
        int pinnedVertexIdx = -1,
        const std::vector<ProjectedDatumAxis>& datumAxes = {})
    {
        if (points.empty()) return;
        if (edgeConstraints.empty() && vertexConstraints.empty()) return;

        size_t n = points.size();

        // Iterate up to 10 passes for relaxation across coupled constraints
        for (int iter = 0; iter < 10; ++iter) {
            bool allSatisfied = true;

            // 1. Enforce vertex origin and axis constraints
            for (const auto& [vIdx, vc] : vertexConstraints) {
                if (vIdx >= n) continue;
                auto& pt = points[vIdx];

                // Constrain distance from Sketch Plane Origin (0, 0)
                if (vc.lockOriginDist && vc.originDist > 1e-4f) {
                    float curDist = std::hypot(pt[0], pt[1]);
                    float diff = std::abs(curDist - vc.originDist);
                    if (diff > 1e-4f) {
                        allSatisfied = false;
                        if (curDist > 1e-6f) {
                            float scale = vc.originDist / curDist;
                            pt[0] *= scale;
                            pt[1] *= scale;
                        } else {
                            pt[0] = vc.originDist;
                            pt[1] = 0.0f;
                        }
                    }
                }

                // Constrain distance from Horizontal U-Axis (V=0, |V| = target)
                if (vc.lockUAxisDist) {
                    float targetV = (pt[1] >= 0.0f) ? vc.uAxisDist : -vc.uAxisDist;
                    if (std::abs(pt[1] - targetV) > 1e-4f) {
                        allSatisfied = false;
                        pt[1] = targetV;
                    }
                }

                // Constrain distance from Vertical V-Axis (U=0, |U| = target)
                if (vc.lockVAxisDist) {
                    float targetU = (pt[0] >= 0.0f) ? vc.vAxisDist : -vc.vAxisDist;
                    if (std::abs(pt[0] - targetU) > 1e-4f) {
                        allSatisfied = false;
                        pt[0] = targetU;
                    }
                }

                // Constrain distance from Reference Datum Axis line
                if (vc.lockDatumAxisDist) {
                    for (const auto& dAx : datumAxes) {
                        if (dAx.valid && (vc.datumAxisId == 0 || dAx.id == vc.datumAxisId)) {
                            float du = pt[0] - dAx.p0[0];
                            float dv = pt[1] - dAx.p0[1];
                            float t = du * dAx.dir[0] + dv * dAx.dir[1];
                            float projU = dAx.p0[0] + t * dAx.dir[0];
                            float projV = dAx.p0[1] + t * dAx.dir[1];
                            float normU = -dAx.dir[1];
                            float normV = dAx.dir[0];
                            float signedDist = du * normU + dv * normV;
                            float sign = (signedDist >= 0.0f) ? 1.0f : -1.0f;
                            float curD = std::abs(signedDist);
                            if (std::abs(curD - vc.datumAxisDist) > 1e-4f) {
                                allSatisfied = false;
                                pt[0] = projU + normU * sign * vc.datumAxisDist;
                                pt[1] = projV + normV * sign * vc.datumAxisDist;
                            }
                            break;
                        }
                    }
                }
            }

            // 2. Enforce edge length constraints
            for (const auto& [edgeIdx, targetLen] : edgeConstraints) {
                if (targetLen <= 0.0001f || edgeIdx >= n) continue;
                size_t i0 = edgeIdx;
                size_t i1 = (i0 + 1) % n;
                if (!isClosed && i0 + 1 >= n) continue;

                auto& p0 = points[i0];
                auto& p1 = points[i1];

                float dx = p1[0] - p0[0];
                float dy = p1[1] - p0[1];
                float curLen = std::hypot(dx, dy);
                float diff = curLen - targetLen;
                if (std::abs(diff) < 1e-4f) continue;

                allSatisfied = false;
                float ux = (curLen > 1e-6f) ? (dx / curLen) : 1.0f;
                float uy = (curLen > 1e-6f) ? (dy / curLen) : 0.0f;

                bool p0Fixed = (pinnedVertexIdx == static_cast<int>(i0));
                bool p1Fixed = (pinnedVertexIdx == static_cast<int>(i1));
                auto it0 = vertexConstraints.find(i0);
                auto it1 = vertexConstraints.find(i1);
                if (it0 != vertexConstraints.end() && it0->second.HasAnyConstraint()) p0Fixed = true;
                if (it1 != vertexConstraints.end() && it1->second.HasAnyConstraint()) p1Fixed = true;

                if (p0Fixed && !p1Fixed) {
                    p1[0] = p0[0] + ux * targetLen;
                    p1[1] = p0[1] + uy * targetLen;
                } else if (!p0Fixed && p1Fixed) {
                    p0[0] = p1[0] - ux * targetLen;
                    p0[1] = p1[1] - uy * targetLen;
                } else {
                    float midX = (p0[0] + p1[0]) * 0.5f;
                    float midY = (p0[1] + p1[1]) * 0.5f;
                    float halfL = targetLen * 0.5f;
                    p0[0] = midX - ux * halfL;
                    p0[1] = midY - uy * halfL;
                    p1[0] = midX + ux * halfL;
                    p1[1] = midY + uy * halfL;
                }
            }
            if (allSatisfied) break;
        }
    }

    static void EnforceConstraints(
        std::vector<std::array<float, 2>>& points,
        const std::map<size_t, float>& edgeConstraints,
        bool isClosed = true,
        int pinnedVertexIdx = -1)
    {
        std::map<size_t, VertexAxisConstraint> emptyVC;
        EnforceConstraints(points, edgeConstraints, emptyVC, isClosed, pinnedVertexIdx);
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
        m_isClosed = true;
        m_shapeClosed = { true };
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
        m_isClosed = true;
        m_shapeClosed = { true };
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
        m_isClosed = true;
        m_shapeClosed = { true };
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
        m_isClosed = true;
        m_shapeClosed = { true };
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
        m_isClosed = true;
        m_shapeClosed = { true };
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
            if (shapeIndex < m_shapeClosed.size()) {
                m_shapeClosed.erase(m_shapeClosed.begin() + shapeIndex);
            }
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

    [[nodiscard]] bool IsSolidFeature() const noexcept override { return false; }
    [[nodiscard]] bool IsConsumed() const noexcept override { return m_isConsumed; }
    void SetConsumed(bool consumed) noexcept override { m_isConsumed = consumed; }

    [[nodiscard]] std::shared_ptr<DAGNode> GetParentFeature() const noexcept override {
        return m_parentFeature.lock();
    }
    void SetParentFeature(std::shared_ptr<DAGNode> parent) noexcept {
        m_parentFeature = parent;
    }
    [[nodiscard]] bool HasParentFeature() const noexcept override {
        return !m_parentFeature.expired();
    }

    void MarkDirty() override {
        if (m_isDirty.load(std::memory_order_acquire)) return;
        DAGNode::MarkDirty();
        if (auto parent = m_parentFeature.lock()) {
            if (!parent->IsDirty()) {
                parent->MarkDirty();
            }
        }
    }

    [[nodiscard]] bool IsVisible() const noexcept { return m_visible; }
    void SetVisible(bool visible) noexcept { 
        m_visible = visible;
    }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;
        if (!IsDirty()) return;

        if (m_datumPlaneRef) {
            m_customPlane = m_datumPlaneRef->GetPlane();
            m_useCustomPlane = true;
            m_position[0] = static_cast<float>(m_customPlane.origin.x);
            m_position[1] = static_cast<float>(m_customPlane.origin.y);
            m_position[2] = static_cast<float>(m_customPlane.origin.z);
        }

        const auto& shapesToRender = GetShapes();
        if (m_profile.size() < 2 && shapesToRender.empty()) {
            m_cachedVertices.clear();
            m_cachedIndices.clear();
            m_cachedEdges.clear();
            ClearDirty();
            return;
        }

        uint64_t currentGen = GetGeneration();
        std::cout << "[SketchNode::Evaluate] Evaluating 2D Sketch '" << m_name 
                  << "' (Gen " << currentGen << " | Points: " << m_profile.size() 
                  << (m_useCustomPlane ? " | Plane: Custom Associative Datum Plane" : (std::string(" | Plane: ") + GetSketchPlaneName(m_plane)))
                  << ")...\n";

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;
        std::vector<uint32_t> featureEdges;

        for (size_t sIdx = 0; sIdx < shapesToRender.size(); ++sIdx) {
            const auto& poly = shapesToRender[sIdx];
            if (poly.size() < 2) continue;

            bool isPolyClosed = (sIdx < m_shapeClosed.size()) ? m_shapeClosed[sIdx] : (m_isClosed && poly.size() >= 3);

            uint32_t baseIdx = static_cast<uint32_t>(vertices.size());
            const size_t ptCount = poly.size();

            for (size_t i = 0; i < ptCount; ++i) {
                float u = poly[i][0];
                float v = poly[i][1];
                CADVertex vert{};

                if (m_useCustomPlane) {
                    Geom::Point3D pt3D = m_customPlane.to_world(Geom::Point2D(u, v)) + m_customPlane.normal * 0.005;
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
            }

            size_t edgeSegments = isPolyClosed ? ptCount : (ptCount - 1);
            for (size_t i = 0; i < edgeSegments; ++i) {
                featureEdges.push_back(baseIdx + static_cast<uint32_t>(i));
                featureEdges.push_back(baseIdx + static_cast<uint32_t>((i + 1) % ptCount));
            }

            if (isPolyClosed && ptCount >= 3) {
                auto polyIndices = TessellationUtils::TriangulatePolygon2D(poly);
                for (uint32_t idx : polyIndices) {
                    indices.push_back(baseIdx + idx);
                }
            }
        }

        m_cachedVertices = vertices;
        m_cachedIndices = indices;
        m_cachedEdges = featureEdges;

        ClearDirty();
    }

private:
    SketchPlane m_plane{SketchPlane::XZ_Ground};
    SketchPreset m_preset{SketchPreset::Custom};
    std::vector<std::array<float, 2>> m_profile;
    std::vector<std::vector<std::array<float, 2>>> m_shapes;
    std::vector<bool> m_shapeClosed;
    std::map<size_t, std::map<size_t, float>> m_shapeEdgeConstraints;
    std::map<size_t, std::map<size_t, VertexAxisConstraint>> m_shapeVertexAxisConstraints;
    bool m_isClosed{true};
    bool m_visible{true};
    std::weak_ptr<DAGNode> m_parentFeature;

    std::shared_ptr<DatumPlaneNode> m_datumPlaneRef{nullptr};
    Geom::Plane m_customPlane;
    bool m_useCustomPlane{false};
};

} // namespace Tessera
