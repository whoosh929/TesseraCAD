#include "engine/modeling/FilletOp.hpp"
#include "engine/modeling/BlendGeometry.hpp"
#include "engine/modeling/CornerBlend.hpp"
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <iostream>

namespace Tessera::Modeling {

namespace {

Geom::Vector3D GetFaceOutwardNormal(const Face& face) {
    if (!face.mesh_vertices.empty() && face.mesh_indices.size() >= 3) {
        // Average normal from mesh vertices
        Geom::Vector3D avgNorm(0, 0, 0);
        for (const auto& v : face.mesh_vertices) {
            avgNorm = avgNorm + Geom::Vector3D(v.normal[0], v.normal[1], v.normal[2]);
        }
        if (avgNorm.length_squared() > 1e-4) {
            return avgNorm.normalized();
        }
        // Fallback: cross product of first triangle
        uint32_t i0 = face.mesh_indices[0];
        uint32_t i1 = face.mesh_indices[1];
        uint32_t i2 = face.mesh_indices[2];
        Geom::Point3D p0(face.mesh_vertices[i0].position[0], face.mesh_vertices[i0].position[1], face.mesh_vertices[i0].position[2]);
        Geom::Point3D p1(face.mesh_vertices[i1].position[0], face.mesh_vertices[i1].position[1], face.mesh_vertices[i1].position[2]);
        Geom::Point3D p2(face.mesh_vertices[i2].position[0], face.mesh_vertices[i2].position[1], face.mesh_vertices[i2].position[2]);
        Geom::Vector3D cross = (p1 - p0).cross(p2 - p0);
        if (cross.length_squared() > 1e-6) {
            return cross.normalized();
        }
    }
    if (face.plane_geom.normal.length_squared() > 0.5) {
        return face.plane_geom.normal.normalized();
    }
    return Geom::Vector3D(0, 1, 0);
}

Geom::Vector3D GetFaceInwardHint(const Face& face, const Geom::Point3D& p0, const Geom::Point3D& p1) {
    Geom::Vector3D edgeVec = (p1 - p0).normalized();
    for (const auto& v : face.mesh_vertices) {
        Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
        if (pt.distance_to(p0) > 0.05 && pt.distance_to(p1) > 0.05) {
            Geom::Vector3D toPt = pt - p0;
            Geom::Vector3D inPlane = toPt - edgeVec * (toPt.dot(edgeVec));
            if (inPlane.length_squared() > 1e-4) {
                return inPlane.normalized();
            }
        }
    }
    return Geom::Vector3D(0, 0, 0);
}

} // anonymous namespace

bool FilletOp::evaluate_edge_convexity(
    const Face& f1, 
    const Face& f2, 
    const Edge& edge, 
    bool& out_is_convex, 
    double& out_dihedral_angle
) {
    if (!edge.start_vertex || !edge.end_vertex) {
        return false;
    }

    Geom::Point3D p0 = edge.start_vertex->point;
    Geom::Point3D p1 = edge.end_vertex->point;
    Geom::Vector3D tangent = (p1 - p0).normalized();
    if (tangent.length_squared() < 1e-6) {
        return false;
    }

    Geom::Vector3D n1 = GetFaceOutwardNormal(f1);
    Geom::Vector3D n2 = GetFaceOutwardNormal(f2);
    Geom::Vector3D inwardHint = GetFaceInwardHint(f1, p0, p1);

    DihedralInfo info = BlendGeometry::ClassifyDihedralEdge(n1, n2, tangent, inwardHint);
    out_is_convex = info.is_convex;
    out_dihedral_angle = info.dihedral_angle;

    return true;
}

Face FilletOp::generate_fillet_surface(
    const Face& f1, 
    const Face& f2, 
    const Edge& edge, 
    double radius, 
    bool is_convex
) {
    Face fillet(0, SurfaceGeometryType::Cylinder);
    fillet.radius = radius;

    if (!edge.start_vertex || !edge.end_vertex) {
        return fillet;
    }

    Geom::Point3D p0 = edge.start_vertex->point;
    Geom::Point3D p1 = edge.end_vertex->point;
    double edgeLen = p0.distance_to(p1);
    if (edgeLen < 1e-4) {
        return fillet;
    }

    Geom::Vector3D tangent = (p1 - p0) / edgeLen;
    Geom::Vector3D n1 = GetFaceOutwardNormal(f1);
    Geom::Vector3D n2 = GetFaceOutwardNormal(f2);
    Geom::Vector3D inwardHint = GetFaceInwardHint(f1, p0, p1);

    DihedralInfo info = BlendGeometry::ClassifyDihedralEdge(n1, n2, tangent, inwardHint);

    // Limit radius to prevent self-intersection
    double maxRadius = edgeLen * 0.45;
    double effRadius = std::clamp(radius, 0.005, std::max(0.005, maxRadius));

    // Number of lengthwise segments and radial bands
    const int numLengthSegments = std::max(2, static_cast<int>(edgeLen / 0.5));
    const int numBands = 6;

    std::vector<CADVertex> vertices;
    std::vector<uint32_t> indices;

    for (int i = 0; i <= numLengthSegments; ++i) {
        double s = static_cast<double>(i) / static_cast<double>(numLengthSegments);
        Geom::Point3D pE = p0 + (p1 - p0) * s;

        Geom::Point3D center = BlendGeometry::ComputeRollingBallCenter(
            pE, n1, n2, effRadius, is_convex, info.dihedral_angle
        );

        for (int b = 0; b <= numBands; ++b) {
            double u = static_cast<double>(b) / static_cast<double>(numBands);
            Geom::Vector3D norm = (n1 * (1.0 - u) + n2 * u).normalized();
            Geom::Vector3D dirFromCenter = is_convex ? norm : -norm;
            Geom::Point3D surfPt = center + dirFromCenter * effRadius;

            vertices.push_back({
                {static_cast<float>(surfPt.x), static_cast<float>(surfPt.y), static_cast<float>(surfPt.z)},
                {static_cast<float>(norm.x), static_cast<float>(norm.y), static_cast<float>(norm.z)}
            });
        }
    }

    // Connect grid of vertices into quads (2 triangles per quad)
    int stride = numBands + 1;
    for (int i = 0; i < numLengthSegments; ++i) {
        for (int b = 0; b < numBands; ++b) {
            uint32_t v0 = static_cast<uint32_t>(i * stride + b);
            uint32_t v1 = static_cast<uint32_t>((i + 1) * stride + b);
            uint32_t v2 = static_cast<uint32_t>((i + 1) * stride + (b + 1));
            uint32_t v3 = static_cast<uint32_t>(i * stride + (b + 1));

            Geom::Point3D pa(vertices[v0].position[0], vertices[v0].position[1], vertices[v0].position[2]);
            Geom::Point3D pb(vertices[v1].position[0], vertices[v1].position[1], vertices[v1].position[2]);
            Geom::Point3D pc(vertices[v2].position[0], vertices[v2].position[1], vertices[v2].position[2]);
            Geom::Vector3D triNorm = (pb - pa).cross(pc - pa);
            Geom::Vector3D expectedNorm = (n1 + n2).normalized();

            if (triNorm.dot(expectedNorm) >= 0.0) {
                indices.push_back(v0); indices.push_back(v1); indices.push_back(v2);
                indices.push_back(v0); indices.push_back(v2); indices.push_back(v3);
            } else {
                indices.push_back(v0); indices.push_back(v2); indices.push_back(v1);
                indices.push_back(v0); indices.push_back(v3); indices.push_back(v2);
            }
        }
    }

    fillet.mesh_vertices = std::move(vertices);
    fillet.mesh_indices = std::move(indices);

    return fillet;
}

void FilletOp::retrim_adjacent_faces(
    Face& f1, 
    Face& f2, 
    const Edge& edge, 
    const Face& fillet_surface
) {
    if (!edge.start_vertex || !edge.end_vertex || fillet_surface.mesh_vertices.empty()) {
        return;
    }

    Geom::Point3D p0 = edge.start_vertex->point;
    Geom::Point3D p1 = edge.end_vertex->point;

    // Contact rail 1 endpoints are the first vertices of band 0 at s=0 and s=1
    // In fillet_surface, stride is (numBands + 1). Band 0 is at offset 0; Band numBands is at offset numBands.
    size_t vertCount = fillet_surface.mesh_vertices.size();
    if (vertCount < 4) return;

    // Retrieve contact rail 1 endpoints (on F1)
    const auto& v_r1_0 = fillet_surface.mesh_vertices[0];
    const auto& v_r1_1 = fillet_surface.mesh_vertices[vertCount - 7]; // last row, band 0
    Geom::Point3D r1_0(v_r1_0.position[0], v_r1_0.position[1], v_r1_0.position[2]);
    Geom::Point3D r1_1(v_r1_1.position[0], v_r1_1.position[1], v_r1_1.position[2]);

    // Retrieve contact rail 2 endpoints (on F2)
    const auto& v_r2_0 = fillet_surface.mesh_vertices[6]; // first row, band numBands
    const auto& v_r2_1 = fillet_surface.mesh_vertices[vertCount - 1]; // last row, band numBands
    Geom::Point3D r2_0(v_r2_0.position[0], v_r2_0.position[1], v_r2_0.position[2]);
    Geom::Point3D r2_1(v_r2_1.position[0], v_r2_1.position[1], v_r2_1.position[2]);

    // Retrim F1: Slide boundary vertices along edge E back to contact rail r1
    for (auto& v : f1.mesh_vertices) {
        Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
        if (pt.distance_to(p0) < 0.05) {
            v.position[0] = static_cast<float>(r1_0.x);
            v.position[1] = static_cast<float>(r1_0.y);
            v.position[2] = static_cast<float>(r1_0.z);
        } else if (pt.distance_to(p1) < 0.05) {
            v.position[0] = static_cast<float>(r1_1.x);
            v.position[1] = static_cast<float>(r1_1.y);
            v.position[2] = static_cast<float>(r1_1.z);
        }
    }

    // Retrim F2: Slide boundary vertices along edge E back to contact rail r2
    for (auto& v : f2.mesh_vertices) {
        Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
        if (pt.distance_to(p0) < 0.05) {
            v.position[0] = static_cast<float>(r2_0.x);
            v.position[1] = static_cast<float>(r2_0.y);
            v.position[2] = static_cast<float>(r2_0.z);
        } else if (pt.distance_to(p1) < 0.05) {
            v.position[0] = static_cast<float>(r2_1.x);
            v.position[1] = static_cast<float>(r2_1.y);
            v.position[2] = static_cast<float>(r2_1.z);
        }
    }
}

Face FilletOp::generate_3corner_patch(
    const Vertex& corner_vertex, 
    const std::vector<Face>& adjacent_fillet_surfaces,
    double radius
) {
    if (adjacent_fillet_surfaces.size() < 3) {
        return Face(0, SurfaceGeometryType::Ruled);
    }

    // Gather normals from the 3 incoming fillets
    Geom::Vector3D n1 = GetFaceOutwardNormal(adjacent_fillet_surfaces[0]);
    Geom::Vector3D n2 = GetFaceOutwardNormal(adjacent_fillet_surfaces[1]);
    Geom::Vector3D n3 = GetFaceOutwardNormal(adjacent_fillet_surfaces[2]);

    auto maybeCenter = BlendGeometry::ComputeCornerSphereCenter(
        corner_vertex.point, n1, n2, n3, radius, true
    );

    if (!maybeCenter) {
        return Face(0, SurfaceGeometryType::Ruled);
    }

    Geom::Point3D center = *maybeCenter;
    Geom::Point3D p1 = center + n1 * radius;
    Geom::Point3D p2 = center + n2 * radius;
    Geom::Point3D p3 = center + n3 * radius;

    auto patch = CornerBlend::CreateSphericalCornerPatch(
        0, center, radius, p1, p2, p3, true, 2
    );

    return *patch;
}

Body FilletOp::apply(const Body& input_body, const std::vector<FilletEdgeSpec>& edges) {
    Body resultBody = input_body;
    if (edges.empty() || resultBody.shells.empty()) {
        return resultBody;
    }

    uint32_t nextFaceId = 1000;

    for (const auto& spec : edges) {
        if (spec.radius <= 1e-4) continue;

        for (auto& shell : resultBody.shells) {
            if (!shell) continue;

            // Find edge and adjacent faces in this shell
            std::shared_ptr<Edge> targetEdge = nullptr;
            std::shared_ptr<Face> face1 = nullptr;
            std::shared_ptr<Face> face2 = nullptr;

            for (const auto& f : shell->faces) {
                if (!f) continue;
                for (const auto& e : f->outer_wire.edges) {
                    if (e && e->id == spec.edge_id) {
                        targetEdge = e;
                        if (!face1) {
                            face1 = f;
                        } else if (!face2 && f != face1) {
                            face2 = f;
                        }
                    }
                }
            }

            // Fallback 1: match by geometric endpoints if edge ID is not explicitly indexed
            if (!targetEdge || !face1 || !face2) {
                for (size_t fi = 0; fi < shell->faces.size(); ++fi) {
                    const auto& f = shell->faces[fi];
                    if (!f) continue;
                    for (const auto& e : f->outer_wire.edges) {
                        if (!e || !e->start_vertex || !e->end_vertex) continue;
                        if (spec.edge_id == 0 || e->id == spec.edge_id) {
                            targetEdge = e;
                            face1 = f;
                            // Find neighboring face sharing this edge
                            for (size_t fj = 0; fj < shell->faces.size(); ++fj) {
                                if (fi == fj) continue;
                                const auto& otherF = shell->faces[fj];
                                for (const auto& oe : otherF->outer_wire.edges) {
                                    if (oe && oe->start_vertex && oe->end_vertex) {
                                        bool match0 = oe->start_vertex->point.distance_to(e->start_vertex->point) < 0.05 &&
                                                      oe->end_vertex->point.distance_to(e->end_vertex->point) < 0.05;
                                        bool match1 = oe->start_vertex->point.distance_to(e->end_vertex->point) < 0.05 &&
                                                      oe->end_vertex->point.distance_to(e->start_vertex->point) < 0.05;
                                        if (match0 || match1) {
                                            face2 = otherF;
                                            break;
                                        }
                                    }
                                }
                                if (face2) break;
                            }
                            if (face1 && face2) break;
                        }
                    }
                    if (face1 && face2) break;
                }
            }

            // Fallback 2: If face2 was not found via outer_wire (e.g. cap face), match via mesh_vertices
            if (targetEdge && face1 && !face2) {
                Geom::Point3D p0 = targetEdge->start_vertex->point;
                Geom::Point3D p1 = targetEdge->end_vertex->point;
                Geom::Vector3D n1 = GetFaceOutwardNormal(*face1);

                for (size_t fi = 0; fi < shell->faces.size(); ++fi) {
                    const auto& otherF = shell->faces[fi];
                    if (otherF == face1) continue;
                    bool hasP0 = false, hasP1 = false;
                    for (const auto& v : otherF->mesh_vertices) {
                        Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
                        if (pt.distance_to(p0) < 0.05) hasP0 = true;
                        if (pt.distance_to(p1) < 0.05) hasP1 = true;
                    }
                    if (hasP0 && hasP1) {
                        Geom::Vector3D n2 = GetFaceOutwardNormal(*otherF);
                        double dotN = std::abs(n1.dot(n2));
                        if (dotN < 0.95) {
                            face2 = otherF;
                            break;
                        }
                    }
                }
            }

            if (!targetEdge || !face1 || !face2) {
                continue;
            }

            // 1. Evaluate convexity & dihedral angle
            bool isConvex = true;
            double dihedralAngle = 1.5707963267948966;
            evaluate_edge_convexity(*face1, *face2, *targetEdge, isConvex, dihedralAngle);

            // 2. Generate fillet blend surface
            Face filletFace = generate_fillet_surface(*face1, *face2, *targetEdge, spec.radius, isConvex);
            filletFace.id = nextFaceId++;

            // 3. Retrim adjacent faces
            retrim_adjacent_faces(*face1, *face2, *targetEdge, filletFace);

            // 4. Add fillet blend face to shell
            shell->add_face(std::make_shared<Face>(std::move(filletFace)));
        }
    }

    return resultBody;
}

} // namespace Tessera::Modeling
