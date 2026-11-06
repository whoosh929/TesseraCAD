#pragma once

#include "Node.hpp"
#include "engine/geom/Plane.hpp"
#include <string>

namespace Tessera {

enum class DatumPlanePreset : uint8_t {
    XY_Front = 0,
    XZ_Top   = 1,
    YZ_Right = 2,
    Custom   = 3
};

class DatumPlaneNode : public DAGNode {
public:
    DatumPlaneNode(std::string name, Geom::Plane plane = Geom::Plane::XY())
        : DAGNode(std::move(name)), m_plane(plane) {}

    void SetPlane(Geom::Plane pl) {
        m_plane = pl;
        MarkDirty();
    }

    void SetOffset(double dist) {
        m_offset = dist;
        MarkDirty();
    }

    [[nodiscard]] Geom::Plane GetPlane() const noexcept {
        return m_plane.offset(m_offset);
    }

    [[nodiscard]] double GetOffset() const noexcept { return m_offset; }

    void Evaluate(RHI::RHIDevice*, AtomicMeshExchange&) override {
        // Reference geometry nodes propagate coordinate frames down the DAG.
    }

private:
    Geom::Plane m_plane{Geom::Plane::XY()};
    double m_offset{0.0};
};

} // namespace Tessera
