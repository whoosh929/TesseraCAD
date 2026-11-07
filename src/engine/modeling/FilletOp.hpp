#pragma once

#include "engine/modeling/Topology.hpp"
#include <vector>
#include <cstdint>

namespace Tessera::Modeling {

struct FilletEdgeSpec {
    uint64_t edge_id{0};
    double radius{0.2};
};

class FilletOp {
public:
    // Executes fillet on selected edges of a Body
    static Body apply(const Body& input_body, const std::vector<FilletEdgeSpec>& edges);

private:
    // 1. Validates edge connectivity and determines signed convexity
    static bool evaluate_edge_convexity(
        const Face& f1, 
        const Face& f2, 
        const Edge& edge, 
        bool& out_is_convex, 
        double& out_dihedral_angle
    );

    // 2. Generates the swept blend surface between contact rails
    static Face generate_fillet_surface(
        const Face& f1, 
        const Face& f2, 
        const Edge& edge, 
        double radius, 
        bool is_convex
    );

    // 3. Trims the original faces back to the contact rails
    static void retrim_adjacent_faces(
        Face& f1, 
        Face& f2, 
        const Edge& edge, 
        const Face& fillet_surface
    );

    // 4. Handles 3-corner vertex joins where multiple filleted edges meet
    static Face generate_3corner_patch(
        const Vertex& corner_vertex, 
        const std::vector<Face>& adjacent_fillet_surfaces,
        double radius
    );
};

} // namespace Tessera::Modeling
