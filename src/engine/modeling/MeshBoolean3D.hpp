#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/graph/GeometryTypes.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <iostream>

namespace Tessera::Modeling {

namespace CSG {

struct CSGVertex {
    Geom::Point3D pos;
    Geom::Vector3D normal;

    CSGVertex() = default;
    CSGVertex(const Geom::Point3D& p, const Geom::Vector3D& n) : pos(p), normal(n) {}

    [[nodiscard]] CSGVertex interpolate(const CSGVertex& other, double t) const noexcept {
        return CSGVertex(
            pos + (other.pos - pos) * t,
            (normal + (other.normal - normal) * t).normalized()
        );
    }
};

struct CSGPolygon;

struct CSGPlane {
    Geom::Vector3D normal{0.0, 0.0, 0.0};
    double w{0.0}; // plane equation: normal . pt = w
    bool valid{false};

    static constexpr double EPSILON = 1e-4;

    CSGPlane() = default;
    CSGPlane(const Geom::Vector3D& n, double wVal) {
        double lenSq = n.length_squared();
        if (lenSq > 1e-10) {
            normal = n.normalized();
            w = wVal;
            valid = true;
        } else {
            normal = Geom::Vector3D(0.0, 0.0, 0.0);
            w = 0.0;
            valid = false;
        }
    }

    CSGPlane(const Geom::Point3D& a, const Geom::Point3D& b, const Geom::Point3D& c) {
        Geom::Vector3D cross = (b - a).cross(c - a);
        double lenSq = cross.length_squared();
        if (lenSq > 1e-10) {
            normal = cross.normalized();
            w = normal.dot(Geom::Vector3D(a.x, a.y, a.z));
            valid = true;
        } else {
            normal = Geom::Vector3D(0.0, 0.0, 0.0);
            w = 0.0;
            valid = false;
        }
    }

    void flip() noexcept {
        normal = -normal;
        w = -w;
    }

    void splitPolygon(
        const CSGPolygon& polygon,
        std::vector<CSGPolygon>& coplanarFront,
        std::vector<CSGPolygon>& coplanarBack,
        std::vector<CSGPolygon>& front,
        std::vector<CSGPolygon>& back
    ) const;
};

struct CSGPolygon {
    std::vector<CSGVertex> vertices;
    CSGPlane plane;

    CSGPolygon() = default;
    CSGPolygon(std::vector<CSGVertex> v, const CSGPlane& p) : vertices(std::move(v)), plane(p) {}
    explicit CSGPolygon(std::vector<CSGVertex> v) : vertices(std::move(v)) {
        if (vertices.size() >= 3) {
            plane = CSGPlane(vertices[0].pos, vertices[1].pos, vertices[2].pos);
        }
    }

    void flip() noexcept {
        std::reverse(vertices.begin(), vertices.end());
        for (auto& v : vertices) {
            v.normal = -v.normal;
        }
        plane.flip();
    }
};

inline void CSGPlane::splitPolygon(
    const CSGPolygon& polygon,
    std::vector<CSGPolygon>& coplanarFront,
    std::vector<CSGPolygon>& coplanarBack,
    std::vector<CSGPolygon>& front,
    std::vector<CSGPolygon>& back
) const {
    if (!valid || normal.length_squared() < 1e-6) {
        front.push_back(polygon);
        return;
    }

    enum { COPLANAR = 0, FRONT = 1, BACK = 2, SPANNING = 3 };

    int polygonType = 0;
    std::vector<int> types;
    types.reserve(polygon.vertices.size());

    for (const auto& vertex : polygon.vertices) {
        double t = normal.dot(Geom::Vector3D(vertex.pos.x, vertex.pos.y, vertex.pos.z)) - w;
        int type = (t < -EPSILON) ? BACK : ((t > EPSILON) ? FRONT : COPLANAR);
        polygonType |= type;
        types.push_back(type);
    }

    switch (polygonType) {
        case COPLANAR: {
            double d = normal.dot(polygon.plane.normal);
            if (d > 0.0) {
                coplanarFront.push_back(polygon);
            } else {
                coplanarBack.push_back(polygon);
            }
            break;
        }
        case FRONT:
            front.push_back(polygon);
            break;
        case BACK:
            back.push_back(polygon);
            break;
        case SPANNING: {
            std::vector<CSGVertex> f, b;
            const size_t numVerts = polygon.vertices.size();
            for (size_t i = 0; i < numVerts; ++i) {
                size_t j = (i + 1) % numVerts;
                int ti = types[i], tj = types[j];
                const auto& vi = polygon.vertices[i];
                const auto& vj = polygon.vertices[j];

                if (ti != BACK) f.push_back(vi);
                if (ti != FRONT) b.push_back(vi);

                if ((ti | tj) == SPANNING) {
                    Geom::Vector3D edgeVec(vj.pos.x - vi.pos.x, vj.pos.y - vi.pos.y, vj.pos.z - vi.pos.z);
                    double denom = normal.dot(edgeVec);
                    double t = 0.5;
                    if (std::abs(denom) > 1e-9) {
                        t = (w - normal.dot(Geom::Vector3D(vi.pos.x, vi.pos.y, vi.pos.z))) / denom;
                        t = std::clamp(t, 0.0, 1.0);
                    }
                    CSGVertex v = vi.interpolate(vj, t);
                    f.push_back(v);
                    b.push_back(v);
                }
            }

            auto cleanPoly = [](std::vector<CSGVertex>& pts) {
                if (pts.size() < 3) return;
                std::vector<CSGVertex> cleaned;
                cleaned.reserve(pts.size());
                for (size_t k = 0; k < pts.size(); ++k) {
                    size_t nextK = (k + 1) % pts.size();
                    if (pts[k].pos.distance_to(pts[nextK].pos) > 1e-5) {
                        cleaned.push_back(pts[k]);
                    }
                }
                pts = std::move(cleaned);
            };

            cleanPoly(f);
            cleanPoly(b);

            if (f.size() >= 3) front.emplace_back(std::move(f), polygon.plane);
            if (b.size() >= 3) back.emplace_back(std::move(b), polygon.plane);
            break;
        }
        default:
            break;
    }
}

class CSGNode {
public:
    CSGPlane plane;
    std::vector<CSGPolygon> polygons;
    std::unique_ptr<CSGNode> front;
    std::unique_ptr<CSGNode> back;

    CSGNode() = default;
    explicit CSGNode(const std::vector<CSGPolygon>& list) {
        build(list);
    }

    void invert() noexcept {
        plane.flip();
        for (auto& poly : polygons) {
            poly.flip();
        }
        if (front) front->invert();
        if (back) back->invert();
        std::swap(front, back);
    }

    [[nodiscard]] std::vector<CSGPolygon> clipPolygons(const std::vector<CSGPolygon>& list) const {
        if (!plane.valid || plane.normal.length_squared() < 1e-6) return list;

        std::vector<CSGPolygon> fList, bList;
        for (const auto& poly : list) {
            plane.splitPolygon(poly, fList, bList, fList, bList);
        }

        if (front) fList = front->clipPolygons(fList);
        if (back) {
            bList = back->clipPolygons(bList);
        } else {
            bList.clear();
        }

        fList.insert(fList.end(), bList.begin(), bList.end());
        return fList;
    }

    void clipTo(const CSGNode* other) {
        if (!other) return;
        polygons = other->clipPolygons(polygons);
        if (front) front->clipTo(other);
        if (back) back->clipTo(other);
    }

    [[nodiscard]] std::vector<CSGPolygon> allPolygons() const {
        std::vector<CSGPolygon> list = polygons;
        if (front) {
            auto fList = front->allPolygons();
            list.insert(list.end(), fList.begin(), fList.end());
        }
        if (back) {
            auto bList = back->allPolygons();
            list.insert(list.end(), bList.begin(), bList.end());
        }
        return list;
    }

    void build(const std::vector<CSGPolygon>& list, int depth = 0) {
        if (list.empty() || depth > 80) return;

        if (!plane.valid || plane.normal.length_squared() < 1e-6) {
            for (const auto& poly : list) {
                if (poly.plane.valid && poly.plane.normal.length_squared() > 1e-6) {
                    plane = poly.plane;
                    plane.valid = true;
                    break;
                }
            }
        }

        if (!plane.valid || plane.normal.length_squared() < 1e-6) {
            polygons.insert(polygons.end(), list.begin(), list.end());
            return;
        }

        std::vector<CSGPolygon> fList, bList;
        for (const auto& poly : list) {
            plane.splitPolygon(poly, polygons, polygons, fList, bList);
        }

        if (!fList.empty()) {
            if (!front) front = std::make_unique<CSGNode>();
            front->build(fList, depth + 1);
        }
        if (!bList.empty()) {
            if (!back) back = std::make_unique<CSGNode>();
            back->build(bList, depth + 1);
        }
    }
};

inline std::vector<CSGPolygon> Difference(
    const std::vector<CSGPolygon>& aPolys,
    const std::vector<CSGPolygon>& bPolys
) {
    if (aPolys.empty()) return {};
    if (bPolys.empty()) return aPolys;

    auto a = std::make_unique<CSGNode>(aPolys);
    auto b = std::make_unique<CSGNode>(bPolys);

    a->invert();
    a->clipTo(b.get());
    b->clipTo(a.get());
    b->invert();
    b->clipTo(a.get());
    b->invert();
    a->build(b->allPolygons());
    a->invert();

    return a->allPolygons();
}

inline std::vector<CSGPolygon> MeshToPolygons(
    const std::vector<CADVertex>& vertices,
    const std::vector<uint32_t>& indices
) {
    std::vector<CSGPolygon> polys;
    const size_t triCount = indices.size() / 3;
    polys.reserve(triCount);

    for (size_t t = 0; t < triCount; ++t) {
        uint32_t i0 = indices[t * 3 + 0];
        uint32_t i1 = indices[t * 3 + 1];
        uint32_t i2 = indices[t * 3 + 2];
        if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) continue;

        const auto& cv0 = vertices[i0];
        const auto& cv1 = vertices[i1];
        const auto& cv2 = vertices[i2];

        Geom::Point3D p0(cv0.position[0], cv0.position[1], cv0.position[2]);
        Geom::Point3D p1(cv1.position[0], cv1.position[1], cv1.position[2]);
        Geom::Point3D p2(cv2.position[0], cv2.position[1], cv2.position[2]);

        Geom::Vector3D n = (p1 - p0).cross(p2 - p0);
        if (n.length_squared() < 1e-10) continue;
        n = n.normalized();

        Geom::Vector3D n0(cv0.normal[0], cv0.normal[1], cv0.normal[2]);
        Geom::Vector3D n1(cv1.normal[0], cv1.normal[1], cv1.normal[2]);
        Geom::Vector3D n2(cv2.normal[0], cv2.normal[1], cv2.normal[2]);

        Geom::Vector3D avgNorm = n0 + n1 + n2;
        if (avgNorm.length_squared() > 1e-4) {
            avgNorm = avgNorm.normalized();
            if (n.dot(avgNorm) < 0.0) {
                n = -n;
                std::swap(p1, p2);
                std::swap(n1, n2);
            }
        }

        if (n0.length_squared() < 1e-4) n0 = n;
        if (n1.length_squared() < 1e-4) n1 = n;
        if (n2.length_squared() < 1e-4) n2 = n;

        std::vector<CSGVertex> triVerts = {
            CSGVertex(p0, n0),
            CSGVertex(p1, n1),
            CSGVertex(p2, n2)
        };
        polys.emplace_back(std::move(triVerts), CSGPlane(n, n.dot(Geom::Vector3D(p0.x, p0.y, p0.z))));
    }
    return polys;
}

inline void PolygonsToMesh(
    const std::vector<CSGPolygon>& polys,
    std::vector<CADVertex>& outVertices,
    std::vector<uint32_t>& outIndices
) {
    outVertices.clear();
    outIndices.clear();

    struct VertexKey {
        int64_t px, py, pz;
        int32_t nx, ny, nz;
        bool operator==(const VertexKey& o) const noexcept {
            return px == o.px && py == o.py && pz == o.pz &&
                   nx == o.nx && ny == o.ny && nz == o.nz;
        }
    };
    struct VertexKeyHash {
        size_t operator()(const VertexKey& k) const noexcept {
            size_t h1 = std::hash<int64_t>()(k.px) ^ (std::hash<int64_t>()(k.py) << 1) ^ (std::hash<int64_t>()(k.pz) << 2);
            size_t h2 = std::hash<int32_t>()(k.nx) ^ (std::hash<int32_t>()(k.ny) << 1) ^ (std::hash<int32_t>()(k.nz) << 2);
            return h1 ^ (h2 << 3);
        }
    };

    std::unordered_map<VertexKey, uint32_t, VertexKeyHash> vertexMap;

    auto GetOrCreateVertex = [&](const CSGVertex& v, const Geom::Vector3D& planeNorm) -> uint32_t {
        constexpr double pScale = 10000.0; // 0.1 mm precision
        constexpr double nScale = 100.0;
        Geom::Vector3D norm = (v.normal.length_squared() > 1e-4) ? v.normal : planeNorm;

        VertexKey key{
            static_cast<int64_t>(std::round(v.pos.x * pScale)),
            static_cast<int64_t>(std::round(v.pos.y * pScale)),
            static_cast<int64_t>(std::round(v.pos.z * pScale)),
            static_cast<int32_t>(std::round(norm.x * nScale)),
            static_cast<int32_t>(std::round(norm.y * nScale)),
            static_cast<int32_t>(std::round(norm.z * nScale))
        };

        auto it = vertexMap.find(key);
        if (it != vertexMap.end()) {
            return it->second;
        }

        uint32_t newIdx = static_cast<uint32_t>(outVertices.size());
        outVertices.push_back({
            .position = { static_cast<float>(v.pos.x), static_cast<float>(v.pos.y), static_cast<float>(v.pos.z) },
            .normal = { static_cast<float>(norm.x), static_cast<float>(norm.y), static_cast<float>(norm.z) }
        });
        vertexMap[key] = newIdx;
        return newIdx;
    };

    for (const auto& poly : polys) {
        if (poly.vertices.size() < 3) continue;

        // Fan-triangulate planar convex polygon
        const size_t n = poly.vertices.size();
        for (size_t i = 1; i + 1 < n; ++i) {
            const auto& v0 = poly.vertices[0];
            const auto& v1 = poly.vertices[i];
            const auto& v2 = poly.vertices[i + 1];

            Geom::Vector3D d1(v1.pos.x - v0.pos.x, v1.pos.y - v0.pos.y, v1.pos.z - v0.pos.z);
            Geom::Vector3D d2(v2.pos.x - v0.pos.x, v2.pos.y - v0.pos.y, v2.pos.z - v0.pos.z);
            Geom::Vector3D triNorm = d1.cross(d2);
            if (triNorm.length_squared() < 1e-10) continue; // skip degenerate triangle

            // Ensure triangle winding agrees with polygon plane normal
            bool matchesPlane = (triNorm.dot(poly.plane.normal) >= 0.0);

            uint32_t idx0 = GetOrCreateVertex(v0, poly.plane.normal);
            uint32_t idx1 = GetOrCreateVertex(v1, poly.plane.normal);
            uint32_t idx2 = GetOrCreateVertex(v2, poly.plane.normal);

            if (idx0 == idx1 || idx1 == idx2 || idx0 == idx2) continue;

            if (matchesPlane) {
                outIndices.push_back(idx0);
                outIndices.push_back(idx1);
                outIndices.push_back(idx2);
            } else {
                outIndices.push_back(idx0);
                outIndices.push_back(idx2);
                outIndices.push_back(idx1);
            }
        }
    }
}

} // namespace CSG

class MeshBoolean3D {
public:
    /// Perform 3D Boolean Difference: targetMesh \ cutterMesh
    static bool Difference(
        const std::vector<CADVertex>& targetVerts,
        const std::vector<uint32_t>& targetIndices,
        const std::vector<CADVertex>& cutterVerts,
        const std::vector<uint32_t>& cutterIndices,
        std::vector<CADVertex>& outVerts,
        std::vector<uint32_t>& outIndices
    ) {
        if (targetVerts.empty() || targetIndices.empty()) {
            outVerts.clear();
            outIndices.clear();
            return false;
        }
        if (cutterVerts.empty() || cutterIndices.empty()) {
            outVerts = targetVerts;
            outIndices = targetIndices;
            return true;
        }

        auto aPolys = CSG::MeshToPolygons(targetVerts, targetIndices);
        auto bPolys = CSG::MeshToPolygons(cutterVerts, cutterIndices);

        if (aPolys.empty()) {
            outVerts = targetVerts;
            outIndices = targetIndices;
            return false;
        }
        if (bPolys.empty()) {
            outVerts = targetVerts;
            outIndices = targetIndices;
            return true;
        }

        auto resultPolys = CSG::Difference(aPolys, bPolys);
        if (resultPolys.empty()) {
            outVerts.clear();
            outIndices.clear();
            return false;
        }

        CSG::PolygonsToMesh(resultPolys, outVerts, outIndices);
        return !outVerts.empty() && !outIndices.empty();
    }
};

} // namespace Tessera::Modeling
