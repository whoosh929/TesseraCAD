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

class CylinderNode : public DAGNode {
public:
    CylinderNode(std::string name, float radius, float height, uint32_t radialSegments = 32)
        : DAGNode(std::move(name)),
          m_radius(radius),
          m_height(height),
          m_segments(std::max(3u, radialSegments)) {}

    void SetParameters(float radius, float height, uint32_t radialSegments = 32) {
        if (m_radius != radius || m_height != height || m_segments != radialSegments) {
            m_radius = radius;
            m_height = height;
            m_segments = std::max(3u, radialSegments);
            MarkDirty();
        }
    }

    [[nodiscard]] float GetRadius() const noexcept { return m_radius; }
    [[nodiscard]] float GetHeight() const noexcept { return m_height; }
    [[nodiscard]] uint32_t GetSegments() const noexcept { return m_segments; }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        if (!IsDirty()) return;

        uint64_t currentGen = GetGeneration();
        float r = m_radius;
        float h = m_height;
        uint32_t segs = m_segments;

        std::cout << "[CylinderNode::Evaluate] Starting parametric evaluation for '" 
                  << m_name << "' (Gen " << currentGen << " | R: " << r << ", H: " << h 
                  << ", Segments: " << segs << ")...\n";

        // Simulate CAD tessellation latency
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        std::vector<CADVertex> vertices;
        std::vector<uint32_t> indices;

        const float halfH = h * 0.5f;
        const float twoPi = 6.28318530718f;

        // 1. Side wall quads (discrete smooth outward normals)
        uint32_t sideBaseIndex = 0;
        for (uint32_t i = 0; i <= segs; ++i) {
            float theta = (static_cast<float>(i) / segs) * twoPi;
            float cosT = std::cos(theta);
            float sinT = std::sin(theta);

            // Top ring
            vertices.push_back({
                .position = {r * cosT, halfH, r * sinT},
                .normal = {cosT, 0.0f, sinT}
            });

            // Bottom ring
            vertices.push_back({
                .position = {r * cosT, -halfH, r * sinT},
                .normal = {cosT, 0.0f, sinT}
            });
        }

        for (uint32_t i = 0; i < segs; ++i) {
            uint32_t top0 = sideBaseIndex + i * 2;
            uint32_t bot0 = sideBaseIndex + i * 2 + 1;
            uint32_t top1 = sideBaseIndex + (i + 1) * 2;
            uint32_t bot1 = sideBaseIndex + (i + 1) * 2 + 1;

            indices.push_back(top0);
            indices.push_back(bot0);
            indices.push_back(top1);

            indices.push_back(top1);
            indices.push_back(bot0);
            indices.push_back(bot1);
        }

        // 2. Top cap (+Y, normal {0, 1, 0})
        uint32_t topCenterIndex = static_cast<uint32_t>(vertices.size());
        vertices.push_back({.position = {0.0f, halfH, 0.0f}, .normal = {0.0f, 1.0f, 0.0f}});

        uint32_t topRingBase = static_cast<uint32_t>(vertices.size());
        for (uint32_t i = 0; i < segs; ++i) {
            float theta = (static_cast<float>(i) / segs) * twoPi;
            vertices.push_back({
                .position = {r * std::cos(theta), halfH, r * std::sin(theta)},
                .normal = {0.0f, 1.0f, 0.0f}
            });
        }
        for (uint32_t i = 0; i < segs; ++i) {
            uint32_t next = (i + 1) % segs;
            indices.push_back(topCenterIndex);
            indices.push_back(topRingBase + next);
            indices.push_back(topRingBase + i);
        }

        // 3. Bottom cap (-Y, normal {0, -1, 0})
        uint32_t botCenterIndex = static_cast<uint32_t>(vertices.size());
        vertices.push_back({.position = {0.0f, -halfH, 0.0f}, .normal = {0.0f, -1.0f, 0.0f}});

        uint32_t botRingBase = static_cast<uint32_t>(vertices.size());
        for (uint32_t i = 0; i < segs; ++i) {
            float theta = (static_cast<float>(i) / segs) * twoPi;
            vertices.push_back({
                .position = {r * std::cos(theta), -halfH, r * std::sin(theta)},
                .normal = {0.0f, -1.0f, 0.0f}
            });
        }
        for (uint32_t i = 0; i < segs; ++i) {
            uint32_t next = (i + 1) % segs;
            indices.push_back(botCenterIndex);
            indices.push_back(botRingBase + i);
            indices.push_back(botRingBase + next);
        }

        // Extract feature edges (captures the circular top and bottom rims)
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

        std::cout << "[CylinderNode::Evaluate] Node '" << m_name 
                  << "' published mesh (Gen " << currentGen 
                  << ", Verts: " << vertices.size() 
                  << ", TriIndices: " << indices.size() 
                  << ", EdgeIndices: " << featureEdges.size() << ").\n";
    }

private:
    float m_radius{1.0f};
    float m_height{2.0f};
    uint32_t m_segments{32};
};

} // namespace Tessera

