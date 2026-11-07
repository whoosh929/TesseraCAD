#pragma once

#include <cstdint>
#include <array>
#include <vector>

namespace Tessera {

enum class SketchPlane : uint8_t {
    XZ_Ground = 0, // Top/Horizontal datum plane (Normal +Y)
    XY_Front  = 1, // Front datum plane (Normal +Z)
    YZ_Right  = 2  // Right datum plane (Normal +X)
};

inline const char* GetSketchPlaneName(SketchPlane plane) noexcept {
    switch (plane) {
        case SketchPlane::XZ_Ground: return "XZ Ground Plane (Top)";
        case SketchPlane::XY_Front:  return "XY Front Plane";
        case SketchPlane::YZ_Right:  return "YZ Right Plane";
    }
    return "Unknown Plane";
}

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

