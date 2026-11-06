#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/geom/Axis3D.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/graph/GeometryTypes.hpp"
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <limits>

namespace Tessera::Modeling {

enum class SurfaceGeometryType : uint8_t {
    Plane = 0,
    Cylinder = 1,
    Cone = 2,
    Torus = 3,
    Ruled = 4,
    BSpline = 5
};

enum class EdgeGeometryType : uint8_t {
    Line = 0,
    CircleArc = 1,
    Spline = 2
};

// =============================================================================
// Topological Vertex: 0D Boundary Entity
// =============================================================================

class Vertex {
public:
    uint32_t id{0};
    Geom::Point3D point{0.0, 0.0, 0.0};

    Vertex() = default;
    Vertex(uint32_t vertexId, Geom::Point3D pt) noexcept : id(vertexId), point(pt) {}
};

// =============================================================================
// Topological Edge: 1D Boundary Entity bounded by two vertices
// =============================================================================

class Edge {
public:
    uint32_t id{0};
    std::shared_ptr<Vertex> start_vertex;
    std::shared_ptr<Vertex> end_vertex;
    EdgeGeometryType geom_type{EdgeGeometryType::Line};

    // Intermediate samples along non-linear curves
    std::vector<Geom::Point3D> intermediate_points;

    Edge() = default;
    Edge(uint32_t edgeId, std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> end, EdgeGeometryType type = EdgeGeometryType::Line)
        : id(edgeId), start_vertex(std::move(start)), end_vertex(std::move(end)), geom_type(type) {}

    [[nodiscard]] double length() const noexcept {
        if (!start_vertex || !end_vertex) return 0.0;
        if (intermediate_points.empty()) {
            return start_vertex->point.distance_to(end_vertex->point);
        }
        double total = 0.0;
        Geom::Point3D prev = start_vertex->point;
        for (const auto& pt : intermediate_points) {
            total += prev.distance_to(pt);
            prev = pt;
        }
        total += prev.distance_to(end_vertex->point);
        return total;
    }

    [[nodiscard]] std::vector<Geom::Point3D> discretize() const {
        std::vector<Geom::Point3D> pts;
        if (!start_vertex || !end_vertex) return pts;

        pts.push_back(start_vertex->point);
        pts.insert(pts.end(), intermediate_points.begin(), intermediate_points.end());
        pts.push_back(end_vertex->point);
        return pts;
    }
};

// =============================================================================
// Topological Wire: Ordered closed or open chain of edges
// =============================================================================

class Wire {
public:
    std::vector<std::shared_ptr<Edge>> edges;
    bool is_closed{true};

    Wire() = default;
    explicit Wire(std::vector<std::shared_ptr<Edge>> e, bool closed = true)
        : edges(std::move(e)), is_closed(closed) {}

    [[nodiscard]] bool empty() const noexcept { return edges.empty(); }
    [[nodiscard]] size_t size() const noexcept { return edges.size(); }
};

// =============================================================================
// Topological Face: 2D Boundary Entity on a parametric surface
// =============================================================================

class Face {
public:
    uint32_t id{0};
    SurfaceGeometryType surface_type{SurfaceGeometryType::Plane};
    Wire outer_wire;
    std::vector<Wire> inner_wires; // Holes / voids

    // Surface analytical descriptors
    Geom::Plane plane_geom;
    Geom::Axis3D axis_geom;
    double radius{0.0};
    double minor_radius{0.0};

    // Pre-computed discrete facets for rendering
    std::vector<CADVertex> mesh_vertices;
    std::vector<uint32_t> mesh_indices;
    std::vector<uint32_t> feature_edge_indices;

    Face() = default;
    Face(uint32_t faceId, SurfaceGeometryType type) : id(faceId), surface_type(type) {}

    [[nodiscard]] bool has_holes() const noexcept { return !inner_wires.empty(); }
};

// =============================================================================
// Topological Shell: Connected 2-Manifold of Faces (Open Sheet or Closed Solid)
// =============================================================================

class Shell {
public:
    std::vector<std::shared_ptr<Face>> faces;
    bool is_closed{false}; // true: watertight solid shell; false: open surface shell

    Shell() = default;
    explicit Shell(bool closed) : is_closed(closed) {}

    [[nodiscard]] size_t face_count() const noexcept { return faces.size(); }
    void add_face(std::shared_ptr<Face> face) {
        if (face) faces.push_back(std::move(face));
    }
};

// =============================================================================
// Topological Body: Top-level Modeling Object (Solid Body or Surface Body)
// =============================================================================

class Body {
public:
    std::string name{"Body"};
    std::vector<std::shared_ptr<Shell>> shells;

    Body() = default;
    explicit Body(std::string bodyName) : name(std::move(bodyName)) {}

    // A body is considered a Solid if all shells are closed watertight manifolds
    [[nodiscard]] bool is_solid() const noexcept {
        if (shells.empty()) return false;
        return std::all_of(shells.begin(), shells.end(), [](const auto& s) {
            return s && s->is_closed;
        });
    }

    // A body is an open Surface / Sheet if any shell is open
    [[nodiscard]] bool is_sheet() const noexcept {
        return !is_solid();
    }

    [[nodiscard]] size_t shell_count() const noexcept { return shells.size(); }

    [[nodiscard]] size_t face_count() const noexcept {
        size_t count = 0;
        for (const auto& s : shells) {
            if (s) count += s->face_count();
        }
        return count;
    }

    void add_shell(std::shared_ptr<Shell> shell) {
        if (shell) shells.push_back(std::move(shell));
    }
};

} // namespace Tessera::Modeling

