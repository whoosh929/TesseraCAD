#pragma once

#include "Topology.hpp"
#include "engine/graph/GeometryTypes.hpp"
#include <memory>
#include <vector>

namespace Tessera::Modeling {

class Tessellator {
public:
    // Evaluate a B-Rep Body into a unified indexed triangle mesh buffer
    [[nodiscard]] static CADMesh Tessellate(const Body& body) {
        CADMesh mesh;

        uint32_t baseVertex = 0;

        for (const auto& shell : body.shells) {
            if (!shell) continue;

            for (const auto& face : shell->faces) {
                if (!face) continue;

                // Append vertices
                mesh.vertices.insert(mesh.vertices.end(), face->mesh_vertices.begin(), face->mesh_vertices.end());

                // Append triangles with base offset
                for (uint32_t idx : face->mesh_indices) {
                    mesh.indices.push_back(baseVertex + idx);
                }

                // Append feature edges with base offset
                for (uint32_t edgeIdx : face->feature_edge_indices) {
                    mesh.featureEdgeIndices.push_back(baseVertex + edgeIdx);
                }

                baseVertex += static_cast<uint32_t>(face->mesh_vertices.size());
            }
        }

        return mesh;
    }

    // Evaluate an individual Shell
    [[nodiscard]] static CADMesh Tessellate(const Shell& shell) {
        Body tempBody;
        tempBody.add_shell(std::make_shared<Shell>(shell));
        return Tessellate(tempBody);
    }
};

} // namespace Tessera::Modeling

