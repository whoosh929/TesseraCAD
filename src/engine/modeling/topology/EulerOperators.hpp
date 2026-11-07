#pragma once

#include "engine/modeling/topology/HalfEdgeMesh.hpp"

namespace Tessera::Modeling::Topology {

class EulerOperators {
public:
    /**
     * @brief Splits an edge (v1, v2) by inserting a new vertex at newPt.
     *        Maintains loop integrity for incident faces.
     *        Euler invariant: V' = V + 1, E' = E + 1.
     * @return Pointer to the newly created HVertex.
     */
    static HVertex* SplitEdge(HalfEdgeMesh& mesh, HEdge* edge, const Geom::Point3D& newPt);

    /**
     * @brief Splits a face by connecting two non-adjacent vertices v1 and v2 on its boundary.
     *        Euler invariant: E' = E + 1, F' = F + 1.
     * @return Pointer to the newly created HFace.
     */
    static HFace* SplitFace(HalfEdgeMesh& mesh, HFace* face, HVertex* v1, HVertex* v2);

    /**
     * @brief Removes a vertex v and all its incident edges, merging the surrounding faces
     *        into a single hole/face bounded by the 1-ring neighbors.
     *        Used for corner vertex surgery to make room for spherical / G1 blend patches.
     * @return Pointer to the newly created merged HFace.
     */
    static HFace* RemoveVertexMakeFace(HalfEdgeMesh& mesh, HVertex* v);

    /**
     * @brief Glues two boundary half-edges as twins along a shared boundary seam.
     */
    static bool SewHalfEdges(HHalfEdge* h1, HHalfEdge* h2);
};

} // namespace Tessera::Modeling::Topology

