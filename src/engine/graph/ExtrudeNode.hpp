#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <memory>

namespace Tessera {

class ExtrudeNode : public DAGNode {
public:
    // Standalone profile constructor
    ExtrudeNode(
        std::string name,
        std::vector<std::array<float, 2>> profile2D,
        float height
    ) : DAGNode(std::move(name)),
        m_profile(std::move(profile2D)),
        m_height(height) {}

    // Reference Sketch constructor (Parametric DAG dependency)
    ExtrudeNode(
        std::string name,
        std::shared_ptr<SketchNode> sketch,
        float height
    ) : DAGNode(std::move(name)),
        m_sketchNode(sketch),
        m_height(height) {
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_profile = m_sketchNode->GetProfile();
            m_plane = m_sketchNode->GetPlane();
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }
    }

    void SetHeight(float height) {
        if (m_height != height) {
            m_height = height;
            MarkDirty();
        }
    }

    void SetProfile(std::vector<std::array<float, 2>> profile2D) {
        m_profile = std::move(profile2D);
        m_sketchNode = nullptr; // Detached
        MarkDirty();
    }

    void SetSketchReference(std::shared_ptr<SketchNode> sketch) {
        m_sketchNode = sketch;
        if (m_sketchNode) {
            AddInput(m_sketchNode);
            m_profile = m_sketchNode->GetProfile();
            m_plane = m_sketchNode->GetPlane();
            m_sketchNode->SetConsumed(true);
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
            MarkDirty();
        }
    }

    [[nodiscard]] std::shared_ptr<SketchNode> GetSketchReference() const noexcept { return m_sketchNode; }
    [[nodiscard]] std::shared_ptr<DAGNode> GetChildSketch() const noexcept override { return m_sketchNode; }

    void DetachChildSketch() noexcept override {
        if (m_sketchNode) {
            m_sketchNode->SetParentFeature(nullptr);
            m_sketchNode->SetConsumed(false);
            m_sketchNode->SetVisible(true);
            m_sketchNode->MarkDirty();
            m_sketchNode = nullptr;
        }
    }

    [[nodiscard]] float GetHeight() const noexcept { return m_height; }
    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept { return m_profile; }
    [[nodiscard]] SketchPlane GetPlane() const noexcept { return m_plane; }
    void SetPlane(SketchPlane plane) { m_plane = plane; MarkDirty(); }
    void SetCustomPlane(const Geom::Plane& plane) { m_customPlane = plane; m_useCustomPlane = true; MarkDirty(); }
    [[nodiscard]] bool UsesCustomPlane() const noexcept { return m_useCustomPlane; }

    bool IsExtrusionBased(ExtrusionData& out) const override {
        out.profile = m_profile;
        out.height = m_height;
        out.position = {m_position[0], m_position[1], m_position[2]};
        out.plane = m_plane;
        out.useCustomPlane = m_useCustomPlane;
        out.customPlane = m_customPlane;
        return !out.profile.empty();
    }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        (void)device;
        (void)exchange;

        // If referencing a sketch node, synchronize profile, plane, and position
        if (m_sketchNode) {
            m_profile = m_sketchNode->GetProfile();
            m_plane = m_sketchNode->GetPlane();
            m_useCustomPlane = m_sketchNode->HasCustomPlane();
            if (m_useCustomPlane) {
                m_customPlane = m_sketchNode->GetCustomPlane();
            }
            m_position[0] = m_sketchNode->GetPosition()[0];
            m_position[1] = m_sketchNode->GetPosition()[1];
            m_position[2] = m_sketchNode->GetPosition()[2];
        }

        std::vector<std::vector<std::array<float, 2>>> loopsToExtrude;
        if (m_sketchNode && !m_sketchNode->GetShapes().empty()) {
            loopsToExtrude = m_sketchNode->GetShapes();
        } else if (!m_profile.empty()) {
            loopsToExtrude.push_back(m_profile);
        }

        if (!IsDirty() || loopsToExtrude.empty()) return;

        uint64_t currentGen = GetGeneration();
        float h = m_height;

        std::cout << "[ExtrudeNode::Evaluate] Starting CAD 2D-to-3D Extrusion for '" 
                  << m_name << "' (Gen " << currentGen << " | Loops: " 
                  << loopsToExtrude.size() << ", Height: " << h 
                  << (m_sketchNode ? " | Ref: " + m_sketchNode->GetName() : " | Standalone") 
                  << ")...\n";

        // Simulate CAD tessellation latency
        std::this_thread::sleep_for(std::chrono::milliseconds(90));

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;

        // Helper lambdas to map 2D point (u, v) and extrusion coordinate w (distance along extrusion direction from sketch plane) to 3D
        auto MakePos = [this](float u, float v, float w) -> std::array<float, 3> {
            if (m_useCustomPlane) {
                Geom::Point3D ptOnPlane = m_customPlane.to_world(Geom::Point2D(u, v));
                Geom::Point3D pt3D = ptOnPlane + m_customPlane.normal * static_cast<double>(w);
                return { static_cast<float>(pt3D.x), static_cast<float>(pt3D.y), static_cast<float>(pt3D.z) };
            }
            switch (m_plane) {
                case SketchPlane::XZ_Ground:
                    return {u + m_position[0], w + m_position[1], v + m_position[2]};
                case SketchPlane::XY_Front:
                    return {u + m_position[0], v + m_position[1], w + m_position[2]};
                case SketchPlane::YZ_Right:
                    return {w + m_position[0], v + m_position[1], u + m_position[2]};
            }
            return {u, w, v};
        };

        auto MakeNormal = [this](float nu, float nv, float nw) -> std::array<float, 3> {
            if (m_useCustomPlane) {
                Geom::Vector3D n = m_customPlane.to_world_vector(Geom::Vector2D(nu, nv)) + m_customPlane.normal * nw;
                double len = n.length();
                if (len > 1e-6) n = n / len;
                return { static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z) };
            }
            switch (m_plane) {
                case SketchPlane::XZ_Ground:
                    return {nu, nw, nv};
                case SketchPlane::XY_Front:
                    return {nu, nv, nw};
                case SketchPlane::YZ_Right:
                    return {nw, nv, nu};
            }
            return {nu, nw, nv};
        };

        const bool isNegative = (h < 0.0f);
        auto topNormal = isNegative ? MakeNormal(0.0f, 0.0f, -1.0f) : MakeNormal(0.0f, 0.0f, 1.0f);
        auto botNormal = isNegative ? MakeNormal(0.0f, 0.0f, 1.0f) : MakeNormal(0.0f, 0.0f, -1.0f);

        for (const auto& loop : loopsToExtrude) {
            if (loop.size() < 3) continue;
            const size_t ptCount = loop.size();

            // 1. Extruded Cap at w = h (+Extrude direction when h > 0, -Extrude direction when h < 0)
            std::vector<uint32_t> capIndices = TessellationUtils::TriangulatePolygon2D(loop);

            uint32_t topBase = static_cast<uint32_t>(vertices.size());
            for (const auto& pt : loop) {
                vertices.push_back({
                    .position = {MakePos(pt[0], pt[1], h)[0], MakePos(pt[0], pt[1], h)[1], MakePos(pt[0], pt[1], h)[2]},
                    .normal = {topNormal[0], topNormal[1], topNormal[2]}
                });
            }
            if (!isNegative) {
                for (uint32_t idx : capIndices) {
                    indices.push_back(topBase + idx);
                }
            } else {
                for (size_t i = 0; i < capIndices.size(); i += 3) {
                    indices.push_back(topBase + capIndices[i]);
                    indices.push_back(topBase + capIndices[i + 2]);
                    indices.push_back(topBase + capIndices[i + 1]);
                }
            }

            // 2. Base Cap at w = 0 (Fixed on the sketch plane; -Extrude direction when h > 0, +Extrude direction when h < 0)
            uint32_t botBase = static_cast<uint32_t>(vertices.size());
            for (const auto& pt : loop) {
                vertices.push_back({
                    .position = {MakePos(pt[0], pt[1], 0.0f)[0], MakePos(pt[0], pt[1], 0.0f)[1], MakePos(pt[0], pt[1], 0.0f)[2]},
                    .normal = {botNormal[0], botNormal[1], botNormal[2]}
                });
            }
            if (!isNegative) {
                for (size_t i = 0; i < capIndices.size(); i += 3) {
                    indices.push_back(botBase + capIndices[i]);
                    indices.push_back(botBase + capIndices[i + 2]);
                    indices.push_back(botBase + capIndices[i + 1]);
                }
            } else {
                for (uint32_t idx : capIndices) {
                    indices.push_back(botBase + idx);
                }
            }

            // 3. Side Walls (Extruded Quads between w = h and w = 0)
            for (size_t i = 0; i < ptCount; ++i) {
                size_t next = (i + 1) % ptCount;

                float x0 = loop[i][0];
                float z0 = loop[i][1];
                float x1 = loop[next][0];
                float z1 = loop[next][1];

                // 2D edge vector (dx, dz) -> outward 2D normal (dz, -dx)
                float dx = x1 - x0;
                float dz = z1 - z0;
                float len = std::sqrt(dx * dx + dz * dz);
                float nx = (len > 1e-6f) ? (dz / len) : 0.0f;
                float nz = (len > 1e-6f) ? (-dx / len) : 1.0f;

                auto wallNormal = MakeNormal(nx, nz, 0.0f);

                uint32_t quadBase = static_cast<uint32_t>(vertices.size());

                auto p0 = MakePos(x0, z0, h);
                auto p1 = MakePos(x1, z1, h);
                auto p2 = MakePos(x1, z1, 0.0f);
                auto p3 = MakePos(x0, z0, 0.0f);

                vertices.push_back({.position = {p0[0], p0[1], p0[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 0: Top start
                vertices.push_back({.position = {p1[0], p1[1], p1[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 1: Top end
                vertices.push_back({.position = {p2[0], p2[1], p2[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 2: Bot end
                vertices.push_back({.position = {p3[0], p3[1], p3[2]}, .normal = {wallNormal[0], wallNormal[1], wallNormal[2]}}); // 3: Bot start

                if (!isNegative) {
                    indices.push_back(quadBase + 0);
                    indices.push_back(quadBase + 3);
                    indices.push_back(quadBase + 1);

                    indices.push_back(quadBase + 1);
                    indices.push_back(quadBase + 3);
                    indices.push_back(quadBase + 2);
                } else {
                    indices.push_back(quadBase + 0);
                    indices.push_back(quadBase + 1);
                    indices.push_back(quadBase + 3);

                    indices.push_back(quadBase + 1);
                    indices.push_back(quadBase + 2);
                    indices.push_back(quadBase + 3);
                }
            }
        }

        // Extract sharp feature silhouette edges
        std::vector<uint32_t> featureEdges = TessellationUtils::ExtractFeatureEdges(vertices, indices, 30.0f);

        m_cachedVertices = vertices;
        m_cachedIndices = indices;
        m_cachedEdges = featureEdges;

        ClearDirty();
    }

    [[nodiscard]] bool HasCustomPlane() const noexcept { return m_useCustomPlane; }
    [[nodiscard]] Geom::Plane GetCustomPlane() const noexcept { return m_customPlane; }

private:
    std::shared_ptr<SketchNode> m_sketchNode{nullptr};
    std::vector<std::array<float, 2>> m_profile;
    float m_height{2.0f};
    SketchPlane m_plane{SketchPlane::XZ_Ground};
    bool m_useCustomPlane{false};
    Geom::Plane m_customPlane{Geom::Plane::XY()};
};

} // namespace Tessera
