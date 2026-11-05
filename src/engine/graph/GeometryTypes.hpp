#pragma once

#include <cstdint>
#include <array>
#include <vector>

namespace Tessera {

struct CADVertex {
    float position[3];
    float normal[3];
};

struct CADMesh {
    std::vector<CADVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> featureEdgeIndices;
};

} // namespace Tessera

