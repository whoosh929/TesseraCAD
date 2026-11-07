#pragma once

#include "engine/graph/GeometryTypes.hpp"
#include "engine/geom/Plane.hpp"
#include <vector>
#include <array>
#include <memory>

namespace Tessera {

struct ExtrusionData {
    std::vector<std::array<float, 2>> profile;
    float height{2.0f};
    std::array<float, 3> position{0.0f, 0.0f, 0.0f};
    SketchPlane plane{SketchPlane::XZ_Ground};
    bool useCustomPlane{false};
    Geom::Plane customPlane{Geom::Point3D(0, 0, 0), Geom::Vector3D(0, 1, 0)};
};

inline Geom::Point3D MapExtrusionPointToWorld(const ExtrusionData& data, float u, float v, float w) {
    if (data.useCustomPlane) {
        Geom::Point3D ptOnPlane = data.customPlane.to_world(Geom::Point2D(u, v));
        return ptOnPlane + data.customPlane.normal * static_cast<double>(w);
    }
    switch (data.plane) {
        case SketchPlane::XZ_Ground:
            return Geom::Point3D(u + data.position[0], w + data.position[1], v + data.position[2]);
        case SketchPlane::XY_Front:
            return Geom::Point3D(u + data.position[0], v + data.position[1], w + data.position[2]);
        case SketchPlane::YZ_Right:
            return Geom::Point3D(w + data.position[0], v + data.position[1], u + data.position[2]);
    }
    return Geom::Point3D(u + data.position[0], w + data.position[1], v + data.position[2]);
}

} // namespace Tessera
