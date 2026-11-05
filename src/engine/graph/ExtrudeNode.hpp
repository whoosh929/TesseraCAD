#pragma once

#include "Node.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

namespace Tessera {

class ExtrudeNode : public DAGNode {
public:
    ExtrudeNode(
        std::string name,
        std::vector<std::array<float, 2>> profile2D,
        float height
    ) : DAGNode(std::move(name)),
        m_profile(std::move(profile2D)),
        m_height(height) {}

    void SetHeight(float height) {
        if (m_height != height) {
            m_height = height;
            MarkDirty();
        }
    }

    void SetProfile(std::vector<std::array<float, 2>> profile2D) {
        m_profile = std::move(profile2D);
        MarkDirty();
    }

    [[nodiscard]] float GetHeight() const noexcept { return m_height; }
    [[nodiscard]] const std::vector<std::array<float, 2>>& GetProfile() const noexcept { return m_profile; }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        if (!IsDirty() || m_profile.size() < 3) return;

        uint64_t currentGen = GetGeneration();
        float h = m_height;
        const size_t ptCount = m_profile.size();

        std::cout << "[ExtrudeNode::Evaluate] Starting CAD 2D-to-3D Extrusion for '" 
                  << m_name << "' (Gen " << currentGen << " | Profile Points: " 
                  << ptCount << ", Height: " << h << ")...\n";

        // Simulate CAD tessellation latency
        std::this_thread::sleep_for(std::chrono::milliseconds(110));

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;

        const float halfH = h * 0.5f;

        // 1. Top Cap (+Y)
        std::vector<uint32_t> capIndices = TessellationUtils::TriangulatePolygon2D(m_profile);

        uint32_t topBase = static_cast<uint32_t>(vertices.size());
        for (const auto& pt : m_profile) {
            vertices.push_back({
                .position = {pt[0], halfH, pt[1]},
                .normal = {0.0f, 1.0f, 0.0f}
            });
        }
        for (uint32_t idx : capIndices) {
            indices.push_back(topBase + idx);
        }

        // 2. Bottom Cap (-Y)
        uint32_t botBase = static_cast<uint32_t>(vertices.size());
        for (const auto& pt : m_profile) {
            vertices.push_back({
                .position = {pt[0], -halfH, pt[1]},
                .normal = {0.0f, -1.0f, 0.0f}
            });
        }
        // Invert winding for bottom cap facing downwards
        for (size_t i = 0; i < capIndices.size(); i += 3) {
            indices.push_back(botBase + capIndices[i]);
            indices.push_back(botBase + capIndices[i + 2]);
            indices.push_back(botBase + capIndices[i + 1]);
        }

        // 3. Side Walls (Extruded Quads with distinct per-face outward normals)
        for (size_t i = 0; i < ptCount; ++i) {
            size_t next = (i + 1) % ptCount;

            float x0 = m_profile[i][0];
            float z0 = m_profile[i][1];
            float x1 = m_profile[next][0];
            float z1 = m_profile[next][1];

            // 2D edge vector (dx, dz) -> outward normal (dz, -dx)
            float dx = x1 - x0;
            float dz = z1 - z0;
            float len = std::sqrt(dx * dx + dz * dz);
            float nx = dz / len;
            float nz = -dx / len;

            uint32_t quadBase = static_cast<uint32_t>(vertices.size());

            vertices.push_back({.position = {x0, halfH,  z0}, .normal = {nx, 0.0f, nz}}); // 0: Top start
            vertices.push_back({.position = {x1, halfH,  z1}, .normal = {nx, 0.0f, nz}}); // 1: Top end
            vertices.push_back({.position = {x1, -halfH, z1}, .normal = {nx, 0.0f, nz}}); // 2: Bot end
            vertices.push_back({.position = {x0, -halfH, z0}, .normal = {nx, 0.0f, nz}}); // 3: Bot start

            indices.push_back(quadBase + 0);
            indices.push_back(quadBase + 3);
            indices.push_back(quadBase + 1);

            indices.push_back(quadBase + 1);
            indices.push_back(quadBase + 3);
            indices.push_back(quadBase + 2);
        }

        // Extract sharp feature silhouette edges
        std::vector<uint32_t> featureEdges = TessellationUtils::ExtractFeatureEdges(vertices, indices, 30.0f);

        for (auto& v : vertices) {
            v.position[0] += m_position[0];
            v.position[1] += m_position[1];
            v.position[2] += m_position[2];
        }

        m_cachedVertices = vertices;
        m_cachedIndices = indices;
        m_cachedEdges = featureEdges;

        std::shared_ptr<RHI::RHIBuffer> vb;
        std::shared_ptr<RHI::RHIBuffer> ib;
        std::shared_ptr<RHI::RHIBuffer> edgeIb;

        if (device) {
            RHI::RHIBufferDesc vbDesc{
                .sizeInBytes = vertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = m_name + "_VB"
            };
            RHI::RHIBufferDesc ibDesc{
                .sizeInBytes = indices.size() * sizeof(uint32_t),
                .usage = RHI::BufferUsage::Index,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = m_name + "_IB"
            };
            vb = device->CreateBuffer(vbDesc, vertices.data());
            ib = device->CreateBuffer(ibDesc, indices.data());

            if (!featureEdges.empty()) {
                RHI::RHIBufferDesc edgeDesc{
                    .sizeInBytes = featureEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = m_name + "_EdgeIB"
                };
                edgeIb = device->CreateBuffer(edgeDesc, featureEdges.data());
            }
        }

        exchange.Publish(
            vb, 
            ib, 
            static_cast<uint32_t>(indices.size()), 
            currentGen,
            edgeIb,
            static_cast<uint32_t>(featureEdges.size())
        );

        ClearDirty();

        std::cout << "[ExtrudeNode::Evaluate] Node '" << m_name 
                  << "' published mesh (Gen " << currentGen 
                  << ", Verts: " << vertices.size() 
                  << ", TriIndices: " << indices.size() 
                  << ", EdgeIndices: " << featureEdges.size() << ").\n";
    }

private:
    std::vector<std::array<float, 2>> m_profile;
    float m_height{2.0f};
};

} // namespace Tessera

