#pragma once

#include "Node.hpp"
#include "Tessellation.hpp"
#include "engine/hal/RHI.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>

namespace Tessera {

class BoxNode : public DAGNode {
public:
    BoxNode(std::string name, float width, float height, float depth)
        : DAGNode(std::move(name)),
          m_width(width),
          m_height(height),
          m_depth(depth) {}

    void SetDimensions(float width, float height, float depth) {
        if (m_width != width || m_height != height || m_depth != depth) {
            m_width = width;
            m_height = height;
            m_depth = depth;
            MarkDirty();
        }
    }

    [[nodiscard]] float GetWidth() const noexcept { return m_width; }
    [[nodiscard]] float GetHeight() const noexcept { return m_height; }
    [[nodiscard]] float GetDepth() const noexcept { return m_depth; }

    void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) override {
        if (!IsDirty()) {
            return;
        }

        uint64_t currentGen = GetGeneration();
        float w = m_width;
        float h = m_height;
        float d = m_depth;

        std::cout << "[BoxNode::Evaluate] Starting 100ms CAD tessellation for Node '" 
                  << m_name << "' (Gen " << currentGen << " | Dimensions: " 
                  << w << "x" << h << "x" << d << ")...\n";

        // Simulate 100ms heavy parametric evaluation
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Generate full 6-face 24-vertex parametric box with discrete normals
        const float hx = w * 0.5f;
        const float hy = h * 0.5f;
        const float hz = d * 0.5f;

        std::vector<CADVertex> vertices = {
            // Front (+Z)
            {{-hx, -hy,  hz}, { 0.0f,  0.0f,  1.0f}},
            {{ hx, -hy,  hz}, { 0.0f,  0.0f,  1.0f}},
            {{ hx,  hy,  hz}, { 0.0f,  0.0f,  1.0f}},
            {{-hx,  hy,  hz}, { 0.0f,  0.0f,  1.0f}},
            // Back (-Z)
            {{ hx, -hy, -hz}, { 0.0f,  0.0f, -1.0f}},
            {{-hx, -hy, -hz}, { 0.0f,  0.0f, -1.0f}},
            {{-hx,  hy, -hz}, { 0.0f,  0.0f, -1.0f}},
            {{ hx,  hy, -hz}, { 0.0f,  0.0f, -1.0f}},
            // Top (+Y)
            {{-hx,  hy,  hz}, { 0.0f,  1.0f,  0.0f}},
            {{ hx,  hy,  hz}, { 0.0f,  1.0f,  0.0f}},
            {{ hx,  hy, -hz}, { 0.0f,  1.0f,  0.0f}},
            {{-hx,  hy, -hz}, { 0.0f,  1.0f,  0.0f}},
            // Bottom (-Y)
            {{-hx, -hy, -hz}, { 0.0f, -1.0f,  0.0f}},
            {{ hx, -hy, -hz}, { 0.0f, -1.0f,  0.0f}},
            {{ hx, -hy,  hz}, { 0.0f, -1.0f,  0.0f}},
            {{-hx, -hy,  hz}, { 0.0f, -1.0f,  0.0f}},
            // Right (+X)
            {{ hx, -hy,  hz}, { 1.0f,  0.0f,  0.0f}},
            {{ hx, -hy, -hz}, { 1.0f,  0.0f,  0.0f}},
            {{ hx,  hy, -hz}, { 1.0f,  0.0f,  0.0f}},
            {{ hx,  hy,  hz}, { 1.0f,  0.0f,  0.0f}},
            // Left (-X)
            {{-hx, -hy, -hz}, {-1.0f,  0.0f,  0.0f}},
            {{-hx, -hy,  hz}, {-1.0f,  0.0f,  0.0f}},
            {{-hx,  hy,  hz}, {-1.0f,  0.0f,  0.0f}},
            {{-hx,  hy, -hz}, {-1.0f,  0.0f,  0.0f}}
        };

        std::vector<uint32_t> indices;
        indices.reserve(36);
        for (uint32_t f = 0; f < 6; ++f) {
            uint32_t base = f * 4;
            indices.push_back(base + 0);
            indices.push_back(base + 1);
            indices.push_back(base + 2);
            indices.push_back(base + 2);
            indices.push_back(base + 3);
            indices.push_back(base + 0);
        }

        // Extract sharp feature edges (dihedral angle = 90 deg > 30 deg threshold)
        std::vector<uint32_t> featureEdges = TessellationUtils::ExtractFeatureEdges(vertices, indices, 30.0f);

        std::shared_ptr<RHI::RHIBuffer> vb;
        std::shared_ptr<RHI::RHIBuffer> ib;
        std::shared_ptr<RHI::RHIBuffer> edgeIb;

        size_t vbBytes = vertices.size() * sizeof(CADVertex);
        size_t ibBytes = indices.size() * sizeof(uint32_t);
        size_t edgeBytes = featureEdges.size() * sizeof(uint32_t);

        if (device) {
            RHI::RHIBufferDesc vbDesc{
                .sizeInBytes = vbBytes,
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = m_name + "_VB"
            };
            RHI::RHIBufferDesc ibDesc{
                .sizeInBytes = ibBytes,
                .usage = RHI::BufferUsage::Index,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = m_name + "_IB"
            };
            vb = device->CreateBuffer(vbDesc, vertices.data());
            ib = device->CreateBuffer(ibDesc, indices.data());

            if (!featureEdges.empty()) {
                RHI::RHIBufferDesc edgeDesc{
                    .sizeInBytes = edgeBytes,
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

        std::cout << "[BoxNode::Evaluate] Node '" << m_name 
                  << "' published mesh (Gen " << currentGen 
                  << ", Verts: 24, TriIndices: 36, FeatureEdgeIndices: " 
                  << featureEdges.size() << ").\n";
    }

private:
    float m_width{1.0f};
    float m_height{1.0f};
    float m_depth{1.0f};
};

} // namespace Tessera
