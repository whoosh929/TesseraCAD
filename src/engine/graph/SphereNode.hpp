#pragma once

#include "Node.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>

namespace Tessera {

class SphereNode : public DAGNode {
public:
    SphereNode(std::string name, float radius, uint32_t rings = 16, uint32_t sectors = 32)
        : DAGNode(std::move(name)),
          m_radius(radius),
          m_rings(std::max(3u, rings)),
          m_sectors(std::max(3u, sectors)) {}

    void SetParameters(float radius, uint32_t rings = 16, uint32_t sectors = 32) {
        if (m_radius != radius || m_rings != rings || m_sectors != sectors) {
            m_radius = radius;
            m_rings = std::max(3u, rings);
            m_sectors = std::max(3u, sectors);
            MarkDirty();
        }
    }

    [[nodiscard]] float GetRadius() const noexcept { return m_radius; }
    [[nodiscard]] uint32_t GetRings() const noexcept { return m_rings; }
    [[nodiscard]] uint32_t GetSectors() const noexcept { return m_sectors; }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        if (!IsDirty()) return;

        uint64_t currentGen = GetGeneration();
        float r = m_radius;
        uint32_t rings = m_rings;
        uint32_t sectors = m_sectors;

        std::cout << "[SphereNode::Evaluate] Starting parametric evaluation for '" 
                  << m_name << "' (Gen " << currentGen << " | R: " << r 
                  << ", Rings: " << rings << ", Sectors: " << sectors << ")...\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(90));

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;

        const float pi = 3.14159265359f;
        const float twoPi = 2.0f * pi;

        const float R = 1.0f / static_cast<float>(rings - 1);
        const float S = 1.0f / static_cast<float>(sectors - 1);

        vertices.reserve(rings * sectors);
        for (uint32_t rIdx = 0; rIdx < rings; ++rIdx) {
            for (uint32_t sIdx = 0; sIdx < sectors; ++sIdx) {
                float y = std::sin(-pi * 0.5f + pi * rIdx * R);
                float x = std::cos(twoPi * sIdx * S) * std::sin(pi * rIdx * R);
                float z = std::sin(twoPi * sIdx * S) * std::sin(pi * rIdx * R);

                vertices.push_back({
                    .position = {x * r, y * r, z * r},
                    .normal = {x, y, z}
                });
            }
        }

        indices.reserve(rings * sectors * 6);
        for (uint32_t rIdx = 0; rIdx < rings - 1; ++rIdx) {
            for (uint32_t sIdx = 0; sIdx < sectors - 1; ++sIdx) {
                uint32_t cur = rIdx * sectors + sIdx;
                uint32_t next = cur + sectors;

                indices.push_back(cur);
                indices.push_back(next);
                indices.push_back(cur + 1);

                indices.push_back(next);
                indices.push_back(next + 1);
                indices.push_back(cur + 1);
            }
        }

        // Feature edge detection (smooth sphere yields 0 sharp edges unless boundary)
        std::vector<uint32_t> featureEdges = TessellationUtils::ExtractFeatureEdges(vertices, indices, 30.0f);

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

        std::cout << "[SphereNode::Evaluate] Node '" << m_name 
                  << "' published mesh (Gen " << currentGen 
                  << ", Verts: " << vertices.size() 
                  << ", TriIndices: " << indices.size() 
                  << ", EdgeIndices: " << featureEdges.size() << ").\n";
    }

private:
    float m_radius{1.0f};
    uint32_t m_rings{16};
    uint32_t m_sectors{32};
};

} // namespace Tessera

