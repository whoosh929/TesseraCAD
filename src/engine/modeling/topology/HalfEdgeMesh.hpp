#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/graph/GeometryTypes.hpp"
#include <vector>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include <cmath>

namespace Tessera::Modeling::Topology {

struct HVertex;
struct HHalfEdge;
struct HEdge;
struct HFace;

struct HVertex {
    uint32_t id{0};
    Geom::Point3D point{0.0, 0.0, 0.0};
    Geom::Vector3D normal{0.0, 1.0, 0.0};
    HHalfEdge* half_edge{nullptr}; // Outgoing half-edge
};

struct HHalfEdge {
    uint32_t id{0};
    HVertex* origin{nullptr};      // Start vertex
    HHalfEdge* twin{nullptr};      // Opposite half-edge
    HHalfEdge* next{nullptr};      // Next in loop (CCW)
    HHalfEdge* prev{nullptr};      // Previous in loop
    HEdge* edge{nullptr};          // Associated edge
    HFace* face{nullptr};          // Incident face
};

struct HEdge {
    uint32_t id{0};
    HHalfEdge* half_edge{nullptr}; // One of the half-edges
};

struct HFace {
    uint32_t id{0};
    HHalfEdge* half_edge{nullptr}; // One half-edge in outer cycle
    Geom::Vector3D normal{0.0, 1.0, 0.0};
};

class HalfEdgeMesh {
public:
    std::vector<std::unique_ptr<HVertex>> vertices;
    std::vector<std::unique_ptr<HHalfEdge>> half_edges;
    std::vector<std::unique_ptr<HEdge>> edges;
    std::vector<std::unique_ptr<HFace>> faces;

    HVertex* AddVertex(const Geom::Point3D& pt, const Geom::Vector3D& norm = {0, 1, 0}) {
        auto v = std::make_unique<HVertex>();
        v->id = static_cast<uint32_t>(vertices.size());
        v->point = pt;
        v->normal = norm;
        HVertex* ptr = v.get();
        vertices.push_back(std::move(v));
        return ptr;
    }

    HEdge* AddEdge(HVertex* v1, HVertex* v2) {
        auto e = std::make_unique<HEdge>();
        e->id = static_cast<uint32_t>(edges.size());

        auto h1 = std::make_unique<HHalfEdge>();
        auto h2 = std::make_unique<HHalfEdge>();

        h1->id = static_cast<uint32_t>(half_edges.size());
        h2->id = static_cast<uint32_t>(half_edges.size() + 1);

        h1->origin = v1;
        h2->origin = v2;

        h1->twin = h2.get();
        h2->twin = h1.get();

        h1->edge = e.get();
        h2->edge = e.get();

        e->half_edge = h1.get();

        if (!v1->half_edge) v1->half_edge = h1.get();
        if (!v2->half_edge) v2->half_edge = h2.get();

        HEdge* ptr = e.get();
        edges.push_back(std::move(e));
        half_edges.push_back(std::move(h1));
        half_edges.push_back(std::move(h2));
        return ptr;
    }

    HFace* AddFace(const std::vector<HVertex*>& loopVertices) {
        if (loopVertices.size() < 3) return nullptr;

        auto f = std::make_unique<HFace>();
        f->id = static_cast<uint32_t>(faces.size());

        // Compute face geometric normal
        Geom::Vector3D v01 = loopVertices[1]->point - loopVertices[0]->point;
        Geom::Vector3D v02 = loopVertices[2]->point - loopVertices[0]->point;
        f->normal = v01.cross(v02).normalized();

        std::vector<HHalfEdge*> loopHalfEdges;
        loopHalfEdges.reserve(loopVertices.size());

        for (size_t i = 0; i < loopVertices.size(); ++i) {
            HVertex* va = loopVertices[i];
            HVertex* vb = loopVertices[(i + 1) % loopVertices.size()];

            // Check if half-edge already exists
            HHalfEdge* h = nullptr;
            for (const auto& existingH : half_edges) {
                if (existingH->origin == va && existingH->twin && existingH->twin->origin == vb) {
                    if (!existingH->face) {
                        h = existingH.get();
                        break;
                    }
                }
            }

            if (!h) {
                HEdge* newE = AddEdge(va, vb);
                h = newE->half_edge; // h goes va -> vb
            }

            h->face = f.get();
            loopHalfEdges.push_back(h);
        }

        // Link next and prev pointers
        size_t n = loopHalfEdges.size();
        for (size_t i = 0; i < n; ++i) {
            loopHalfEdges[i]->next = loopHalfEdges[(i + 1) % n];
            loopHalfEdges[(i + 1) % n]->prev = loopHalfEdges[i];
        }

        f->half_edge = loopHalfEdges[0];
        HFace* ptr = f.get();
        faces.push_back(std::move(f));
        return ptr;
    }

    static HalfEdgeMesh FromIndexedMesh(const std::vector<CADVertex>& verts, const std::vector<uint32_t>& indices) {
        HalfEdgeMesh mesh;
        if (indices.size() < 3) return mesh;

        // Add vertices
        std::vector<HVertex*> vPtrs;
        vPtrs.reserve(verts.size());
        for (const auto& v : verts) {
            Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
            Geom::Vector3D norm(v.normal[0], v.normal[1], v.normal[2]);
            vPtrs.push_back(mesh.AddVertex(pt, norm));
        }

        // Add triangles
        for (size_t t = 0; t + 2 < indices.size(); t += 3) {
            uint32_t i0 = indices[t];
            uint32_t i1 = indices[t + 1];
            uint32_t i2 = indices[t + 2];
            if (i0 < vPtrs.size() && i1 < vPtrs.size() && i2 < vPtrs.size()) {
                mesh.AddFace({vPtrs[i0], vPtrs[i1], vPtrs[i2]});
            }
        }

        return mesh;
    }

    [[nodiscard]] CADMesh ToCADMesh() const {
        CADMesh result;
        std::unordered_map<const HVertex*, uint32_t> vMap;

        for (const auto& v : vertices) {
            vMap[v.get()] = static_cast<uint32_t>(result.vertices.size());
            result.vertices.push_back({
                {static_cast<float>(v->point.x), static_cast<float>(v->point.y), static_cast<float>(v->point.z)},
                {static_cast<float>(v->normal.x), static_cast<float>(v->normal.y), static_cast<float>(v->normal.z)}
            });
        }

        for (const auto& f : faces) {
            if (!f || !f->half_edge) continue;
            std::vector<HVertex*> fVerts = GetFaceVertices(f.get());
            if (fVerts.size() < 3) continue;

            // Simple fan triangulation for n-gons
            for (size_t i = 1; i + 1 < fVerts.size(); ++i) {
                result.indices.push_back(vMap[fVerts[0]]);
                result.indices.push_back(vMap[fVerts[i]]);
                result.indices.push_back(vMap[fVerts[i + 1]]);
            }
        }

        // Extract sharp feature edges (boundary or sharp dihedral)
        for (const auto& e : edges) {
            if (!e || !e->half_edge) continue;
            HHalfEdge* h = e->half_edge;
            HHalfEdge* twin = h->twin;

            bool isSharp = false;
            if (!h->face || !twin || !twin->face) {
                isSharp = true; // Boundary edge
            } else {
                double dotN = h->face->normal.dot(twin->face->normal);
                if (dotN < 0.866) { // > 30 degrees
                    isSharp = true;
                }
            }

            if (isSharp && h->origin && twin && twin->origin) {
                result.featureEdgeIndices.push_back(vMap[h->origin]);
                result.featureEdgeIndices.push_back(vMap[twin->origin]);
            }
        }

        return result;
    }

    [[nodiscard]] std::vector<HVertex*> GetFaceVertices(const HFace* f) const {
        std::vector<HVertex*> verts;
        if (!f || !f->half_edge) return verts;

        HHalfEdge* curr = f->half_edge;
        do {
            if (curr->origin) verts.push_back(curr->origin);
            curr = curr->next;
        } while (curr && curr != f->half_edge);

        return verts;
    }

    [[nodiscard]] std::vector<HHalfEdge*> GetFaceHalfEdges(const HFace* f) const {
        std::vector<HHalfEdge*> hes;
        if (!f || !f->half_edge) return hes;

        HHalfEdge* curr = f->half_edge;
        do {
            hes.push_back(curr);
            curr = curr->next;
        } while (curr && curr != f->half_edge);

        return hes;
    }

    [[nodiscard]] size_t Valence(const HVertex* v) const {
        if (!v || !v->half_edge) return 0;
        size_t count = 0;
        HHalfEdge* curr = v->half_edge;
        do {
            ++count;
            if (!curr->twin) break;
            curr = curr->twin->next;
        } while (curr && curr != v->half_edge);
        return count;
    }
};

} // namespace Tessera::Modeling::Topology

