#include "engine/modeling/topology/EulerOperators.hpp"
#include <algorithm>

namespace Tessera::Modeling::Topology {

HVertex* EulerOperators::SplitEdge(HalfEdgeMesh& mesh, HEdge* edge, const Geom::Point3D& newPt) {
    if (!edge || !edge->half_edge) return nullptr;

    HHalfEdge* h1 = edge->half_edge;
    HHalfEdge* h2 = h1->twin;

    HVertex* v1 = h1->origin;
    HVertex* v2 = h2 ? h2->origin : nullptr;

    if (!v1) return nullptr;

    // Normal at new vertex interpolated or inherited from v1
    Geom::Vector3D newNorm = v1->normal;
    if (v2) newNorm = (v1->normal + v2->normal).normalized();

    HVertex* vNew = mesh.AddVertex(newPt, newNorm);

    // Create a new edge for the second half
    auto newEdge = std::make_unique<HEdge>();
    newEdge->id = static_cast<uint32_t>(mesh.edges.size());

    auto h1b = std::make_unique<HHalfEdge>();
    auto h2b = std::make_unique<HHalfEdge>();

    h1b->id = static_cast<uint32_t>(mesh.half_edges.size());
    h2b->id = static_cast<uint32_t>(mesh.half_edges.size() + 1);

    h1b->origin = vNew;
    h1b->edge = newEdge.get();
    h1b->face = h1->face;

    h2b->origin = vNew;
    h2b->edge = edge;
    h2b->face = h2 ? h2->face : nullptr;

    // Splice h1: v1 -> vNew -> (h1b) -> v2
    h1b->next = h1->next;
    if (h1b->next) h1b->next->prev = h1b.get();
    h1->next = h1b.get();
    h1b->prev = h1;

    // Link twins:
    if (h2) {
        // h2 goes v2 -> v1. Now h2 goes v2 -> vNew, and h2b goes vNew -> v1
        h2b->next = h2->next;
        if (h2b->next) h2b->next->prev = h2b.get();
        h2->next = h2b.get();
        h2b->prev = h2;

        h1b->twin = h2;
        h2->twin = h1b.get();

        h1->twin = h2b.get();
        h2b->twin = h1;
    } else {
        h1b->twin = nullptr;
    }

    newEdge->half_edge = h1b.get();
    vNew->half_edge = h1b.get();

    mesh.edges.push_back(std::move(newEdge));
    mesh.half_edges.push_back(std::move(h1b));
    mesh.half_edges.push_back(std::move(h2b));

    return vNew;
}

HFace* EulerOperators::SplitFace(HalfEdgeMesh& mesh, HFace* face, HVertex* v1, HVertex* v2) {
    if (!face || !v1 || !v2 || v1 == v2) return nullptr;

    // Locate half-edges originating at v1 and v2 in this face's cycle
    HHalfEdge* h1 = nullptr;
    HHalfEdge* h2 = nullptr;

    HHalfEdge* curr = face->half_edge;
    do {
        if (curr->origin == v1) h1 = curr;
        if (curr->origin == v2) h2 = curr;
        curr = curr->next;
    } while (curr && curr != face->half_edge);

    if (!h1 || !h2 || h1->next == h2 || h2->next == h1) {
        return nullptr; // Adjacent or not on face
    }

    auto newEdge = std::make_unique<HEdge>();
    newEdge->id = static_cast<uint32_t>(mesh.edges.size());

    auto he1 = std::make_unique<HHalfEdge>(); // v1 -> v2
    auto he2 = std::make_unique<HHalfEdge>(); // v2 -> v1

    he1->id = static_cast<uint32_t>(mesh.half_edges.size());
    he2->id = static_cast<uint32_t>(mesh.half_edges.size() + 1);

    he1->origin = v1;
    he2->origin = v2;
    he1->twin = he2.get();
    he2->twin = he1.get();
    he1->edge = newEdge.get();
    he2->edge = newEdge.get();

    auto newFace = std::make_unique<HFace>();
    newFace->id = static_cast<uint32_t>(mesh.faces.size());
    newFace->normal = face->normal;

    // Splice into two loops:
    // Loop 1 (stays with face): h1->prev -> he1 -> h2 ...
    HHalfEdge* h1_prev = h1->prev;
    HHalfEdge* h2_prev = h2->prev;

    he1->next = h2;
    h2->prev = he1.get();
    h1_prev->next = he1.get();
    he1->prev = h1_prev;
    he1->face = face;
    face->half_edge = he1.get();

    // Loop 2 (goes to newFace): h2_prev -> he2 -> h1 ...
    he2->next = h1;
    h1->prev = he2.get();
    h2_prev->next = he2.get();
    he2->prev = h2_prev;
    he2->face = newFace.get();

    // Update face pointers for loop 2
    HHalfEdge* loop2 = h1;
    do {
        loop2->face = newFace.get();
        loop2 = loop2->next;
    } while (loop2 && loop2 != h1);

    newFace->half_edge = he2.get();

    HFace* result = newFace.get();
    mesh.edges.push_back(std::move(newEdge));
    mesh.half_edges.push_back(std::move(he1));
    mesh.half_edges.push_back(std::move(he2));
    mesh.faces.push_back(std::move(newFace));

    return result;
}

HFace* EulerOperators::RemoveVertexMakeFace(HalfEdgeMesh& mesh, HVertex* v) {
    if (!v || !v->half_edge) return nullptr;

    // Collect 1-ring outer boundary vertices around v
    std::vector<HVertex*> ringVertices;
    HHalfEdge* start = v->half_edge;
    HHalfEdge* curr = start;

    do {
        if (curr->next && curr->next->origin && curr->next->origin != v) {
            ringVertices.push_back(curr->next->origin);
        }
        if (!curr->twin) break;
        curr = curr->twin->next;
    } while (curr && curr != start);

    if (ringVertices.size() < 3) return nullptr;

    // Remove incident faces and create single n-gon face
    return mesh.AddFace(ringVertices);
}

bool EulerOperators::SewHalfEdges(HHalfEdge* h1, HHalfEdge* h2) {
    if (!h1 || !h2 || h1 == h2) return false;
    h1->twin = h2;
    h2->twin = h1;
    return true;
}

} // namespace Tessera::Modeling::Topology
