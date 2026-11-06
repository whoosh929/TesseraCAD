#pragma once

#include "Node.hpp"
#include "engine/geom/Axis3D.hpp"
#include <string>

namespace Tessera {

enum class DatumAxisPreset : uint8_t {
    X_Axis = 0,
    Y_Axis = 1,
    Z_Axis = 2,
    Custom = 3
};

class DatumAxisNode : public DAGNode {
public:
    DatumAxisNode(std::string name, Geom::Axis3D axis = Geom::Axis3D::Y())
        : DAGNode(std::move(name)), m_axis(axis) {}

    void SetAxis(Geom::Axis3D axis) {
        m_axis = axis;
        MarkDirty();
    }

    [[nodiscard]] Geom::Axis3D GetAxis() const noexcept {
        return m_axis;
    }

    void Evaluate(RHI::RHIDevice*, AtomicMeshExchange&) override {
        // Reference geometry nodes propagate coordinate frames down the DAG.
    }

private:
    Geom::Axis3D m_axis{Geom::Axis3D::Y()};
};

} // namespace Tessera
