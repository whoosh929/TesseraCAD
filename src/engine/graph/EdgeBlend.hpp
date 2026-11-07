#pragma once

#include "ExtrudeNode.hpp"
#include "CapTrim.hpp"
#include <algorithm>
#include <cmath>
#include <array>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Tessera {

/// Cross-section of an edge blend: a circular round (fillet) or a straight equal-distance bevel (chamfer).
enum class EdgeBlendKind { Round, Bevel };

inline Geom::Point3D basePos3(const std::vector<CADVertex>& v, uint32_t i) {
    return Geom::Point3D(v[i].position[0], v[i].position[1], v[i].position[2]);
}

/**
 * @brief Boundary of a set of triangles (indices into 'indices', by triangle number) as one closed
 * loop of vertex ids, following the triangles' winding. Fails for holes or several pieces.
 */
inline bool PlanarBoundaryLoop(const std::vector<CADVertex>& vertices, const std::vector<uint32_t>& indices,
                               const std::vector<size_t>& tris, std::vector<uint32_t>& loop) {
    using Key = std::array<long long, 3>;
    auto key = [&](uint32_t i) {
        const auto& p = vertices[i].position;
        return Key{std::llround(p[0] * 1e5), std::llround(p[1] * 1e5), std::llround(p[2] * 1e5)};
    };
    std::map<std::pair<Key, Key>, int> directed;
    std::map<std::pair<Key, Key>, uint32_t> startId;
    for (size_t t : tris) {
        for (int k = 0; k < 3; ++k) {
            uint32_t a = indices[t * 3 + k], b = indices[t * 3 + (k + 1) % 3];
            Key ka = key(a), kb = key(b);
            if (ka == kb) continue;
            directed[{ka, kb}]++;
            startId[{ka, kb}] = a;
        }
    }
    std::map<Key, std::pair<Key, uint32_t>> next; // start -> (end, start vertex id)
    for (const auto& [e, cnt] : directed) {
        if (directed.count({e.second, e.first})) continue; // interior edge
        if (next.count(e.first)) return false;            // pinched / non-manifold boundary
        next[e.first] = {e.second, startId[e]};
    }
    if (next.size() < 3) return false;
    Key start = next.begin()->first, cur = start;
    loop.clear();
    do {
        auto it = next.find(cur);
        if (it == next.end() || loop.size() > next.size()) return false;
        loop.push_back(it->second.second);
        cur = it->second.first;
    } while (cur != start);
    return loop.size() == next.size(); // a single loop covering the whole boundary
}

/**
 * @brief Ear-clips a planar loop (counter-clockwise about n) into triangles with the same winding.
 * Collinear vertices that cannot form an ear are dropped (RepairTJunctions re-stitches them).
 */
inline bool EarClipPlanar(const std::vector<CADVertex>& vertices, std::vector<uint32_t> work,
                          const Geom::Vector3D& n, std::vector<uint32_t>& out) {
    Geom::Vector3D e1 = std::abs(n.x) < 0.9 ? Geom::Vector3D(1, 0, 0) : Geom::Vector3D(0, 1, 0);
    e1 = (e1 - n * e1.dot(n)).normalized();
    Geom::Vector3D e2 = n.cross(e1);
    auto P = [&](uint32_t i, double& x, double& y) {
        const auto& p = vertices[i].position;
        x = p[0] * e1.x + p[1] * e1.y + p[2] * e1.z;
        y = p[0] * e2.x + p[1] * e2.y + p[2] * e2.z;
    };
    auto cross2 = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };
    out.clear();
    int guard = 0;
    while (work.size() > 3 && guard++ < 100000) {
        const size_t m = work.size();
        bool clipped = false;
        for (size_t i = 0; i < m && !clipped; ++i) {
            uint32_t ia = work[(i + m - 1) % m], ib = work[i], ic = work[(i + 1) % m];
            double ax, ay, bx, by, cx, cy;
            P(ia, ax, ay); P(ib, bx, by); P(ic, cx, cy);
            if (cross2(ax, ay, bx, by, cx, cy) <= 1e-12) continue;
            bool inside = false;
            for (size_t j = 0; j < m && !inside; ++j) {
                uint32_t ip = work[j];
                if (ip == ia || ip == ib || ip == ic) continue;
                double px, py;
                P(ip, px, py);
                inside = cross2(ax, ay, bx, by, px, py) >= -1e-12 && cross2(bx, by, cx, cy, px, py) >= -1e-12 &&
                         cross2(cx, cy, ax, ay, px, py) >= -1e-12;
            }
            if (inside) continue;
            out.insert(out.end(), {ia, ib, ic});
            work.erase(work.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
        }
        if (clipped) continue;
        // No ear: drop a collinear vertex if there is one, otherwise give up
        bool dropped = false;
        for (size_t i = 0; i < m && !dropped; ++i) {
            double ax, ay, bx, by, cx, cy;
            P(work[(i + m - 1) % m], ax, ay); P(work[i], bx, by); P(work[(i + 1) % m], cx, cy);
            if (std::abs(cross2(ax, ay, bx, by, cx, cy)) <= 1e-12) {
                work.erase(work.begin() + static_cast<std::ptrdiff_t>(i));
                dropped = true;
            }
        }
        if (!dropped) return false;
    }
    if (work.size() == 3) {
        double ax, ay, bx, by, cx, cy;
        P(work[0], ax, ay); P(work[1], bx, by); P(work[2], cx, cy);
        if (cross2(ax, ay, bx, by, cx, cy) > 1e-12) out.insert(out.end(), {work[0], work[1], work[2]});
    }
    return !out.empty();
}

/**
 * @brief Splits triangles along open edges that have another open-boundary vertex lying inside them
 * (T-junctions). Arises where a blend ends at a concave corner: the neighbouring wall keeps the sharp
 * corner vertex while the blended faces' boundary now passes straight through it.
 */
inline void RepairTJunctions(const std::vector<CADVertex>& vertices, std::vector<uint32_t>& indices) {
    auto pos = [&](uint32_t i) {
        return Geom::Point3D(vertices[i].position[0], vertices[i].position[1], vertices[i].position[2]);
    };
    auto key = [&](uint32_t i) {
        const auto& p = vertices[i].position;
        return std::array<long long, 3>{std::llround(p[0] * 1e5), std::llround(p[1] * 1e5), std::llround(p[2] * 1e5)};
    };
    for (int iter = 0; iter < 16; ++iter) {
        std::map<std::pair<std::array<long long, 3>, std::array<long long, 3>>, std::vector<size_t>> use;
        for (size_t t = 0; t + 2 < indices.size(); t += 3) {
            for (int k = 0; k < 3; ++k) {
                auto A = key(indices[t + k]), B = key(indices[t + (k + 1) % 3]);
                if (B < A) std::swap(A, B);
                use[{A, B}].push_back(t * 3 + static_cast<size_t>(k)); // triangle offset * 3 + local edge
            }
        }
        std::vector<size_t> openEdges;
        std::map<std::array<long long, 3>, uint32_t> openVerts;
        for (const auto& kv : use) {
            if (kv.second.size() != 1) continue;
            size_t code = kv.second.front();
            size_t k = code - (code / 3) * 3;
            openEdges.push_back(code);
            size_t base = (code / 3);
            uint32_t a = indices[base + k], b = indices[base + (k + 1) % 3];
            openVerts.emplace(key(a), a);
            openVerts.emplace(key(b), b);
        }
        if (openEdges.empty()) return;

        std::vector<char> touched(indices.size() / 3, 0);
        std::vector<uint32_t> added;
        bool changed = false;
        for (size_t code : openEdges) {
            size_t base = code / 3;        // offset of the triangle's first index
            size_t k = code - base * 3;
            if (touched[base / 3]) continue;
            uint32_t a = indices[base + k], b = indices[base + (k + 1) % 3], c = indices[base + (k + 2) % 3];
            Geom::Point3D pa = pos(a), pb = pos(b);
            Geom::Vector3D ab = pb - pa;
            double len2 = ab.length_squared();
            if (len2 < 1e-12) continue;
            uint32_t split = 0;
            double bestT = 2.0;
            for (const auto& [kk, v] : openVerts) {
                if (kk == key(a) || kk == key(b)) continue;
                Geom::Vector3D av = pos(v) - pa;
                double tt = av.dot(ab) / len2;
                if (tt <= 1e-4 || tt >= 1.0 - 1e-4) continue;
                if ((av - ab * tt).length() > 1e-4) continue;
                if (tt < bestT) { bestT = tt; split = v; }
            }
            if (bestT > 1.0) continue;
            // (a, b, c) -> (a, v, c) + (v, b, c), keeping the winding
            indices[base + 0] = a; indices[base + 1] = split; indices[base + 2] = c;
            added.insert(added.end(), {split, b, c});
            touched[base / 3] = 1;
            changed = true;
        }
        indices.insert(indices.end(), added.begin(), added.end());
        if (!changed) return;
    }
}

/**
 * @brief General mesh-based edge blend shared by FilletNode and ChamferNode.
 *
 * Starting from the seed edge it chains tangent-continuous feature edges that share face A, builds
 * exact mitered setback points at chain junctions, sweeps the blend cross-section (arc or straight
 * line) along every chain segment, and trims the planar end caps at open chain ends to the blend
 * profile so no flat piece of the original sharp corner remains.
 *
 * @param size  Fillet radius (Round) or chamfer setback distance on both faces (Bevel).
 * @return false when no two-face blend can be formed; outputs are then left as the base mesh copy.
 */
inline bool BuildEdgeBlend(
    const std::vector<CADVertex>& baseVerts,
    const std::vector<uint32_t>& baseIndices,
    const std::vector<uint32_t>& baseEdges,
    int edgeIndex,
    const Geom::Point3D& edgeP0,
    const Geom::Point3D& edgeP1,
    float size,
    EdgeBlendKind kind,
    std::vector<CADVertex>& vertices,
    std::vector<uint32_t>& indices)
{
    vertices = baseVerts;
    indices = baseIndices;

    // Point-coincidence tolerance. Scaled to the shortest feature edge so the short arc segments of an
    // earlier round (~0.04 long for r = 0.2) are not mistaken for one point, which made large faces
    // "contain" whole arc segments and snapped vertices onto the wrong junction.
    double kTol = 0.05;
    for (size_t e = 0; e + 1 < baseEdges.size(); e += 2) {
        const auto& a = baseVerts[baseEdges[e]].position;
        const auto& b = baseVerts[baseEdges[e + 1]].position;
        double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        double len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 1e-6) kTol = std::min(kTol, 0.25 * len);
    }
    kTol = std::max(kTol, 1e-4);

    double edgeLen = edgeP0.distance_to(edgeP1);
    if (edgeLen < 1e-4) {
        return false;
    }

    Geom::Vector3D edgeVec = (edgeP1 - edgeP0) / edgeLen;

    // 1. Find all triangles incident to the seed edge and identify the two distinct face normals normalA and normalB
    Geom::Vector3D normalA(0, 0, 0);
    Geom::Vector3D normalB(0, 0, 0);
    Geom::Point3D thirdPtA, thirdPtB;
    bool hasNormalA = false;
    bool hasNormalB = false;

    const size_t numTriangles = baseIndices.size() / 3;
    for (size_t t = 0; t < numTriangles; ++t) {
        uint32_t a = baseIndices[t * 3 + 0];
        uint32_t b = baseIndices[t * 3 + 1];
        uint32_t c = baseIndices[t * 3 + 2];

        Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
        Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
        Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);

        bool hasP0 = (pa.distance_to(edgeP0) < kTol || pb.distance_to(edgeP0) < kTol || pc.distance_to(edgeP0) < kTol);
        bool hasP1 = (pa.distance_to(edgeP1) < kTol || pb.distance_to(edgeP1) < kTol || pc.distance_to(edgeP1) < kTol);

        if (hasP0 && hasP1) {
            Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();
            if (triNorm.length() < 0.5) continue;

            Geom::Point3D pThird = pc;
            if (pa.distance_to(edgeP0) >= kTol && pa.distance_to(edgeP1) >= kTol) pThird = pa;
            else if (pb.distance_to(edgeP0) >= kTol && pb.distance_to(edgeP1) >= kTol) pThird = pb;

            if (!hasNormalA) {
                normalA = triNorm;
                thirdPtA = pThird;
                hasNormalA = true;
            } else if (!hasNormalB && normalA.dot(triNorm) < 0.95) {
                normalB = triNorm;
                thirdPtB = pThird;
                hasNormalB = true;
            }
        }
    }

    if (!hasNormalA || !hasNormalB) {
        return false;
    }

    struct ChainEdge {
        Geom::Point3D p0;
        Geom::Point3D p1;
        Geom::Vector3D edgeVec;
        Geom::Vector3D normalB;
        Geom::Vector3D uA;
        Geom::Vector3D uB;
        float rEff{0.0f};
    };

    // Builds the tangent chain treating 'normalA' as the face shared by every chained edge.
    auto buildChain = [&](const Geom::Vector3D normalA, const Geom::Vector3D normalB,
                          const Geom::Point3D thirdPtA, const Geom::Point3D thirdPtB,
                          float& seedRadiusOut) -> std::vector<ChainEdge> {
    Geom::Vector3D vIntoA = (thirdPtA - edgeP0) - edgeVec * ((thirdPtA - edgeP0).dot(edgeVec));
    Geom::Vector3D uA = (vIntoA.length_squared() > 1e-6) ? vIntoA.normalized() : edgeVec.cross(normalA).normalized();
    Geom::Vector3D vIntoB = (thirdPtB - edgeP0) - edgeVec * ((thirdPtB - edgeP0).dot(edgeVec));
    Geom::Vector3D uB = (vIntoB.length_squared() > 1e-6) ? vIntoB.normalized() : normalB.cross(edgeVec).normalized();
    uA = (uA - normalA * uA.dot(normalA) - edgeVec * uA.dot(edgeVec)).normalized();
    uB = (uB - normalB * uB.dot(normalB) - edgeVec * uB.dot(edgeVec)).normalized();

    // Limit the setback by how far each face actually extends away from the edge. Measured over the
    // whole planar face, not just the edge's own triangle: ear-clipped faces often put a sliver
    // triangle on the edge whose third point lies almost on it, which would collapse the blend.
    auto faceExtent = [&](const Geom::Vector3D& n, const Geom::Vector3D& u) {
        double ext = 0.0;
        for (size_t t = 0; t < numTriangles; ++t) {
            Geom::Point3D p[3];
            for (int k = 0; k < 3; ++k) {
                const auto& v = baseVerts[baseIndices[t * 3 + k]];
                p[k] = Geom::Point3D(v.position[0], v.position[1], v.position[2]);
            }
            Geom::Vector3D tn = (p[1] - p[0]).cross(p[2] - p[0]);
            if (tn.length() < 1e-9 || tn.normalized().dot(n) < 0.95) continue;
            for (const auto& q : p) {
                if (std::abs((q - edgeP0).dot(n)) > 1e-3) break; // different parallel face
                ext = std::max(ext, (q - edgeP0).dot(u));
            }
        }
        return ext;
    };
    const double extentA = std::max(vIntoA.length(), faceExtent(normalA, uA));
    const double extentB = std::max(vIntoB.length(), faceExtent(normalB, uB));
    float maxSetback = static_cast<float>(edgeLen) * 0.45f;
    if (extentA > 1e-4) maxSetback = std::min(maxSetback, static_cast<float>(extentA) * 0.45f);
    if (extentB > 1e-4) maxSetback = std::min(maxSetback, static_cast<float>(extentB) * 0.45f);
    float seedRadius = std::clamp(size, 0.005f, std::max(0.005f, maxSetback));

    std::vector<ChainEdge> chain;
    chain.push_back({edgeP0, edgeP1, edgeVec, normalB, uA, uB, seedRadius});

    std::unordered_set<size_t> visitedEdges;
    visitedEdges.insert(static_cast<size_t>(edgeIndex));
    auto alreadyChained = [&](const Geom::Point3D& a, const Geom::Point3D& b) {
        for (const auto& c : chain) {
            if ((c.p0.distance_to(a) < 0.01 && c.p1.distance_to(b) < 0.01) || (c.p0.distance_to(b) < 0.01 && c.p1.distance_to(a) < 0.01)) return true;
        }
        return false;
    };

    // Forward propagation along tangent feature edges sharing Face A
    bool grewForward = true;
    while (grewForward) {
        grewForward = false;
        Geom::Point3D currTip = chain.back().p1;
        Geom::Vector3D currDir = chain.back().edgeVec;

        if (chain.size() >= 3 && currTip.distance_to(chain.front().p0) < kTol) {
            break;
        }

        int bestEdgeIdx = -1;
        Geom::Point3D bestP0, bestP1;
        Geom::Vector3D bestDir, bestNormB, bestUA, bestUB;
        double bestScore = 0.35; // tangent-continuous chaining only (no wrapping around sharp corners)

        for (size_t e = 0; e < baseEdges.size() / 2; ++e) {
            if (visitedEdges.count(e)) continue;

            uint32_t edgeI0 = baseEdges[e * 2];
            uint32_t edgeI1 = baseEdges[e * 2 + 1];
            Geom::Point3D ep0(baseVerts[edgeI0].position[0], baseVerts[edgeI0].position[1], baseVerts[edgeI0].position[2]);
            Geom::Point3D ep1(baseVerts[edgeI1].position[0], baseVerts[edgeI1].position[1], baseVerts[edgeI1].position[2]);
            if (alreadyChained(ep0, ep1)) continue; // same geometric edge listed twice (per-face vertex copies)

            Geom::Point3D candP0, candP1;
            if (ep0.distance_to(currTip) < kTol) {
                candP0 = ep0; candP1 = ep1;
            } else if (ep1.distance_to(currTip) < kTol) {
                candP0 = ep1; candP1 = ep0;
            } else {
                continue;
            }

            double cLen = candP0.distance_to(candP1);
            if (cLen < 1e-4) continue;
            Geom::Vector3D candDir = (candP1 - candP0) / cLen;

            double dotDir = currDir.dot(candDir);
            if (dotDir <= bestScore) continue;

            bool candHasFaceA = false;
            Geom::Vector3D candNormB(0, 0, 0);
            Geom::Point3D candThirdA, candThirdB;
            bool hasCandB = false;

            for (size_t t = 0; t < numTriangles; ++t) {
                uint32_t a = baseIndices[t * 3 + 0];
                uint32_t b = baseIndices[t * 3 + 1];
                uint32_t c = baseIndices[t * 3 + 2];
                Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                bool hasP0 = (pa.distance_to(candP0) < kTol || pb.distance_to(candP0) < kTol || pc.distance_to(candP0) < kTol);
                bool hasP1 = (pa.distance_to(candP1) < kTol || pb.distance_to(candP1) < kTol || pc.distance_to(candP1) < kTol);
                if (hasP0 && hasP1) {
                    Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
                    if (tn.length() < 0.5) continue;
                    Geom::Point3D p3 = pc;
                    if (pa.distance_to(candP0) >= kTol && pa.distance_to(candP1) >= kTol) p3 = pa;
                    else if (pb.distance_to(candP0) >= kTol && pb.distance_to(candP1) >= kTol) p3 = pb;

                    if (tn.dot(normalA) > 0.85) {
                        candHasFaceA = true;
                        candThirdA = p3;
                    } else if (!hasCandB && tn.dot(normalA) < 0.7) {
                        candNormB = tn;
                        candThirdB = p3;
                        hasCandB = true;
                    }
                }
            }

            if (candHasFaceA && hasCandB) {
                bestScore = dotDir;
                bestEdgeIdx = static_cast<int>(e);
                bestP0 = candP0; bestP1 = candP1; bestDir = candDir;
                bestNormB = candNormB;

                Geom::Vector3D vA = (candThirdA - candP0) - candDir * ((candThirdA - candP0).dot(candDir));
                bestUA = (vA.length_squared() > 1e-6) ? vA.normalized() : candDir.cross(normalA).normalized();
                Geom::Vector3D vB = (candThirdB - candP0) - candDir * ((candThirdB - candP0).dot(candDir));
                bestUB = (vB.length_squared() > 1e-6) ? vB.normalized() : candNormB.cross(candDir).normalized();
                bestUA = (bestUA - normalA * bestUA.dot(normalA) - candDir * bestUA.dot(candDir)).normalized();
                bestUB = (bestUB - candNormB * bestUB.dot(candNormB) - candDir * bestUB.dot(candDir)).normalized();
            }
        }

        if (bestEdgeIdx >= 0) {
            visitedEdges.insert(static_cast<size_t>(bestEdgeIdx));
            chain.push_back({bestP0, bestP1, bestDir, bestNormB, bestUA, bestUB, seedRadius});
            grewForward = true;
        }
    }

    // Backward propagation along tangent feature edges sharing Face A
    bool grewBackward = true;
    while (grewBackward) {
        grewBackward = false;
        Geom::Point3D currRoot = chain.front().p0;
        Geom::Vector3D currDir = chain.front().edgeVec;

        if (chain.size() >= 3 && currRoot.distance_to(chain.back().p1) < kTol) {
            break;
        }

        int bestEdgeIdx = -1;
        Geom::Point3D bestP0, bestP1;
        Geom::Vector3D bestDir, bestNormB, bestUA, bestUB;
        double bestScore = 0.35; // tangent-continuous chaining only (no wrapping around sharp corners)

        for (size_t e = 0; e < baseEdges.size() / 2; ++e) {
            if (visitedEdges.count(e)) continue;

            uint32_t edgeI0 = baseEdges[e * 2];
            uint32_t edgeI1 = baseEdges[e * 2 + 1];
            Geom::Point3D ep0(baseVerts[edgeI0].position[0], baseVerts[edgeI0].position[1], baseVerts[edgeI0].position[2]);
            Geom::Point3D ep1(baseVerts[edgeI1].position[0], baseVerts[edgeI1].position[1], baseVerts[edgeI1].position[2]);
            if (alreadyChained(ep0, ep1)) continue; // same geometric edge listed twice (per-face vertex copies)

            Geom::Point3D candP0, candP1;
            if (ep1.distance_to(currRoot) < kTol) {
                candP0 = ep0; candP1 = ep1;
            } else if (ep0.distance_to(currRoot) < kTol) {
                candP0 = ep1; candP1 = ep0;
            } else {
                continue;
            }

            double cLen = candP0.distance_to(candP1);
            if (cLen < 1e-4) continue;
            Geom::Vector3D candDir = (candP1 - candP0) / cLen;

            double dotDir = candDir.dot(currDir);
            if (dotDir <= bestScore) continue;

            bool candHasFaceA = false;
            Geom::Vector3D candNormB(0, 0, 0);
            Geom::Point3D candThirdA, candThirdB;
            bool hasCandB = false;

            for (size_t t = 0; t < numTriangles; ++t) {
                uint32_t a = baseIndices[t * 3 + 0];
                uint32_t b = baseIndices[t * 3 + 1];
                uint32_t c = baseIndices[t * 3 + 2];
                Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
                Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
                Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
                bool hasP0 = (pa.distance_to(candP0) < kTol || pb.distance_to(candP0) < kTol || pc.distance_to(candP0) < kTol);
                bool hasP1 = (pa.distance_to(candP1) < kTol || pb.distance_to(candP1) < kTol || pc.distance_to(candP1) < kTol);
                if (hasP0 && hasP1) {
                    Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
                    if (tn.length() < 0.5) continue;
                    Geom::Point3D p3 = pc;
                    if (pa.distance_to(candP0) >= kTol && pa.distance_to(candP1) >= kTol) p3 = pa;
                    else if (pb.distance_to(candP0) >= kTol && pb.distance_to(candP1) >= kTol) p3 = pb;

                    if (tn.dot(normalA) > 0.85) {
                        candHasFaceA = true;
                        candThirdA = p3;
                    } else if (!hasCandB && tn.dot(normalA) < 0.7) {
                        candNormB = tn;
                        candThirdB = p3;
                        hasCandB = true;
                    }
                }
            }

            if (candHasFaceA && hasCandB) {
                bestScore = dotDir;
                bestEdgeIdx = static_cast<int>(e);
                bestP0 = candP0; bestP1 = candP1; bestDir = candDir;
                bestNormB = candNormB;

                Geom::Vector3D vA = (candThirdA - candP0) - candDir * ((candThirdA - candP0).dot(candDir));
                bestUA = (vA.length_squared() > 1e-6) ? vA.normalized() : candDir.cross(normalA).normalized();
                Geom::Vector3D vB = (candThirdB - candP0) - candDir * ((candThirdB - candP0).dot(candDir));
                bestUB = (vB.length_squared() > 1e-6) ? vB.normalized() : candNormB.cross(candDir).normalized();
                bestUA = (bestUA - normalA * bestUA.dot(normalA) - candDir * bestUA.dot(candDir)).normalized();
                bestUB = (bestUB - candNormB * bestUB.dot(candNormB) - candDir * bestUB.dot(candDir)).normalized();
            }
        }

        if (bestEdgeIdx >= 0) {
            visitedEdges.insert(static_cast<size_t>(bestEdgeIdx));
            chain.insert(chain.begin(), {bestP0, bestP1, bestDir, bestNormB, bestUA, bestUB, seedRadius});
            grewBackward = true;
        }
    }
    seedRadiusOut = seedRadius;
    return chain;
    };

    // Either face of the seed edge can be the one the chain shares. Prefer the choice that propagates
    // further along tangent edges: e.g. a top edge running into an earlier round continues around that
    // round's arc on the front face (which borders the whole run) instead of dead-ending on a curved
    // facet. Ties keep the dominant face (more triangles), as before.
    int countA = 0, countB = 0;
    for (size_t t = 0; t < numTriangles; ++t) {
        uint32_t a = baseIndices[t * 3 + 0];
        uint32_t b = baseIndices[t * 3 + 1];
        uint32_t c = baseIndices[t * 3 + 2];
        Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
        Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
        Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);
        Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
        if (tn.dot(normalA) > 0.85) countA++;
        if (tn.dot(normalB) > 0.85) countB++;
    }
    float seedRadiusAB = 0.0f, seedRadiusBA = 0.0f;
    std::vector<ChainEdge> chainAB = buildChain(normalA, normalB, thirdPtA, thirdPtB, seedRadiusAB);
    std::vector<ChainEdge> chainBA = buildChain(normalB, normalA, thirdPtB, thirdPtA, seedRadiusBA);
    const bool useBA = chainBA.size() > chainAB.size() || (chainBA.size() == chainAB.size() && countB > countA);
    if (useBA) {
        std::swap(normalA, normalB);
        std::swap(thirdPtA, thirdPtB);
    }
    std::vector<ChainEdge> chain = useBA ? std::move(chainBA) : std::move(chainAB);
    const float seedRadius = useBA ? seedRadiusBA : seedRadiusAB;


    const size_t chainCount = chain.size();
    bool isLoop = (chainCount >= 3 && chain.front().p0.distance_to(chain.back().p1) < kTol);

    // Compute setback distance d = R * tan(alpha/2) for each chain segment
    std::vector<double> setbackD(chainCount);
    for (size_t i = 0; i < chainCount; ++i) {
        double cosAlpha = std::clamp(normalA.dot(chain[i].normalB), -0.999, 0.999);
        double sinAlpha = (normalA.cross(chain[i].normalB)).length();
        double tanHalf = (1.0 + cosAlpha > 1e-5) ? (sinAlpha / (1.0 + cosAlpha)) : 1.0;
        setbackD[i] = (kind == EdgeBlendKind::Round) ? seedRadius * tanHalf : seedRadius;
    }

    // Compute continuous mitered setback points on Face A for every junction in the chain
    std::vector<Geom::Point3D> sA_pts(chainCount + 1);
    std::vector<uint32_t> v_sA(chainCount + 1);

    // Exact mitered offset point at a junction: the point p on face A with p.u1 = d1 and p.u2 = d2,
    // i.e. the intersection of the two setback lines. Handles sharp corner turns (e.g. 90 degrees).
    auto miterPoint = [](const Geom::Point3D& J, const Geom::Vector3D& u1, double d1,
                         const Geom::Vector3D& u2, double d2) -> Geom::Point3D {
        double c = u1.dot(u2);
        double den = 1.0 - c * c;
        if (den < 1e-3) {
            return J + (u1 + u2).normalized() * ((d1 + d2) * 0.5);
        }
        double a = (d1 - d2 * c) / den;
        double b = (d2 - d1 * c) / den;
        return J + u1 * a + u2 * b;
    };

    for (size_t i = 0; i <= chainCount; ++i) {
        Geom::Point3D junctionPt;
        if (i == 0) {
            junctionPt = chain.front().p0;
            sA_pts[i] = isLoop
                ? miterPoint(junctionPt, chain.back().uA, setbackD.back(), chain.front().uA, setbackD.front())
                : junctionPt + chain.front().uA * setbackD.front();
        } else if (i == chainCount) {
            junctionPt = chain.back().p1;
            sA_pts[i] = isLoop
                ? miterPoint(junctionPt, chain.back().uA, setbackD.back(), chain.front().uA, setbackD.front())
                : junctionPt + chain.back().uA * setbackD.back();
        } else {
            junctionPt = chain[i].p0;
            sA_pts[i] = miterPoint(junctionPt, chain[i - 1].uA, setbackD[i - 1], chain[i].uA, setbackD[i]);
        }
    }

    // A blend larger than the radius of curvature of the edge it follows (e.g. r = 0.3 wrapping an
    // earlier r = 0.2 round) offsets that arc past its centre: the setback points cross over and the
    // surface folds back on itself. Collapse each crossed-over run into one cusp point instead.
    for (size_t i = 0; i < chainCount; ++i) {
        if ((sA_pts[i + 1] - sA_pts[i]).dot(chain[i].edgeVec) >= 0.0) continue;
        size_t lo = i, hi = i + 1;
        while (hi < chainCount && (sA_pts[hi + 1] - sA_pts[hi]).dot(chain[hi].edgeVec) < 0.0) ++hi;
        while (lo > 0 && (sA_pts[lo] - sA_pts[lo - 1]).dot(chain[lo - 1].edgeVec) < 0.0) --lo;
        Geom::Point3D c(0, 0, 0);
        for (size_t k = lo; k <= hi; ++k) { c.x += sA_pts[k].x; c.y += sA_pts[k].y; c.z += sA_pts[k].z; }
        const double n = static_cast<double>(hi - lo + 1);
        c = Geom::Point3D(c.x / n, c.y / n, c.z / n);
        for (size_t k = lo; k <= hi; ++k) sA_pts[k] = c;
        i = hi;
    }

    for (size_t i = 0; i <= chainCount; ++i) {

        v_sA[i] = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(sA_pts[i].x), static_cast<float>(sA_pts[i].y), static_cast<float>(sA_pts[i].z)},
            {static_cast<float>(normalA.x), static_cast<float>(normalA.y), static_cast<float>(normalA.z)}
        });
    }

    // Slide a cross-section point of segment i along its edge direction so it lies on the miter
    // (bisector) plane of the junction at the segment's start or end. Makes adjacent filleted edges
    // meet cleanly at a corner instead of overlapping or leaving gaps.
    auto toMiterPlane = [&](const Geom::Point3D& pt, size_t i, bool atEnd) -> Geom::Point3D {
        const Geom::Vector3D* prevDir = nullptr;
        const Geom::Vector3D* nextDir = nullptr;
        Geom::Point3D J;
        if (atEnd) {
            J = chain[i].p1;
            if (i + 1 < chainCount) nextDir = &chain[i + 1].edgeVec;
            else if (isLoop) nextDir = &chain.front().edgeVec;
            if (!nextDir) return pt;
            prevDir = &chain[i].edgeVec;
        } else {
            J = chain[i].p0;
            if (i > 0) prevDir = &chain[i - 1].edgeVec;
            else if (isLoop) prevDir = &chain.back().edgeVec;
            if (!prevDir) return pt;
            nextDir = &chain[i].edgeVec;
        }
        Geom::Vector3D sum = *prevDir + *nextDir;
        if (sum.length() < 1e-6) return pt;
        Geom::Vector3D nm = sum.normalized();
        const Geom::Vector3D& e = chain[i].edgeVec;
        double denom = e.dot(nm);
        if (std::abs(denom) < 0.2) return pt;
        double s = -((pt - J).dot(nm)) / denom;
        return pt + e * s;
    };

    // Compute setback points on side Face B for each segment
    std::vector<uint32_t> v_sB0(chainCount);
    std::vector<uint32_t> v_sB1(chainCount);
    std::vector<Geom::Point3D> sB0_pts(chainCount);
    std::vector<Geom::Point3D> sB1_pts(chainCount);

    for (size_t i = 0; i < chainCount; ++i) {
        sB0_pts[i] = toMiterPlane(chain[i].p0 + chain[i].uB * setbackD[i], i, false);
        sB1_pts[i] = toMiterPlane(chain[i].p1 + chain[i].uB * setbackD[i], i, true);

        v_sB0[i] = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(sB0_pts[i].x), static_cast<float>(sB0_pts[i].y), static_cast<float>(sB0_pts[i].z)},
            {static_cast<float>(chain[i].normalB.x), static_cast<float>(chain[i].normalB.y), static_cast<float>(chain[i].normalB.z)}
        });

        v_sB1[i] = static_cast<uint32_t>(vertices.size());
        vertices.push_back({
            {static_cast<float>(sB1_pts[i].x), static_cast<float>(sB1_pts[i].y), static_cast<float>(sB1_pts[i].z)},
            {static_cast<float>(chain[i].normalB.x), static_cast<float>(chain[i].normalB.y), static_cast<float>(chain[i].normalB.z)}
        });
    }

    // Update incident triangles belonging to Face A or Face B by cleanly sliding vertices to setback anchors
    for (size_t t = 0; t < numTriangles; ++t) {
        uint32_t a = baseIndices[t * 3 + 0];
        uint32_t b = baseIndices[t * 3 + 1];
        uint32_t c = baseIndices[t * 3 + 2];

        Geom::Point3D pa(baseVerts[a].position[0], baseVerts[a].position[1], baseVerts[a].position[2]);
        Geom::Point3D pb(baseVerts[b].position[0], baseVerts[b].position[1], baseVerts[b].position[2]);
        Geom::Point3D pc(baseVerts[c].position[0], baseVerts[c].position[1], baseVerts[c].position[2]);

        Geom::Vector3D triNorm = (pb - pa).cross(pc - pa).normalized();
        if (triNorm.length() < 0.5) continue;

        double dotA = triNorm.dot(normalA);

        if (dotA > 0.85) {
            for (int k = 0; k < 3; ++k) {
                uint32_t origIdx = baseIndices[t * 3 + k];
                Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                for (size_t j = 0; j <= chainCount; ++j) {
                    Geom::Point3D jPt = (j < chainCount) ? chain[j].p0 : chain.back().p1;
                    if (p.distance_to(jPt) < kTol) {
                        indices[t * 3 + k] = v_sA[j];
                        break;
                    }
                }
            }
        } else {
            // Assign the triangle to the chain segment whose side face it actually belongs to: the best
            // normal match among segments touching it. Taking the first match within tolerance picks the
            // wrong facet on tessellated rounds (neighbouring facets differ by only ~15 degrees).
            auto touches = [&](const Geom::Point3D& q) {
                return pa.distance_to(q) < kTol || pb.distance_to(q) < kTol || pc.distance_to(q) < kTol;
            };
            int bestSeg = -1;
            double bestDot = 0.85;
            for (size_t i = 0; i < chainCount; ++i) {
                double d = triNorm.dot(chain[i].normalB);
                if (d > bestDot && (touches(chain[i].p0) || touches(chain[i].p1))) {
                    bestDot = d;
                    bestSeg = static_cast<int>(i);
                }
            }
            if (bestSeg >= 0) {
                const size_t i = static_cast<size_t>(bestSeg);
                for (int k = 0; k < 3; ++k) {
                    uint32_t origIdx = baseIndices[t * 3 + k];
                    Geom::Point3D p(baseVerts[origIdx].position[0], baseVerts[origIdx].position[1], baseVerts[origIdx].position[2]);
                    if (p.distance_to(chain[i].p0) < kTol) {
                        indices[t * 3 + k] = v_sB0[i];
                    } else if (p.distance_to(chain[i].p1) < kTol) {
                        indices[t * 3 + k] = v_sB1[i];
                    }
                }
            }
        }
    }

    // Sliding vertices to their setback points can turn long sliver triangles of a planar face inside
    // out (e.g. the top face fanned around an inside corner's round). The face is still planar, so
    // when that happens rebuild it from its boundary loop instead.
    auto retriangulateIfFlipped = [&](const Geom::Vector3D& n, const Geom::Point3D& onPlane) {
        auto curPos = [&](uint32_t id) {
            return Geom::Point3D(vertices[id].position[0], vertices[id].position[1], vertices[id].position[2]);
        };
        std::vector<size_t> faceTris;
        bool flipped = false;
        for (size_t t = 0; t < numTriangles; ++t) {
            Geom::Point3D b0 = basePos3(baseVerts, baseIndices[t * 3]);
            Geom::Point3D b1 = basePos3(baseVerts, baseIndices[t * 3 + 1]);
            Geom::Point3D b2 = basePos3(baseVerts, baseIndices[t * 3 + 2]);
            Geom::Vector3D bn = (b1 - b0).cross(b2 - b0);
            if (bn.length() < 1e-12 || bn.normalized().dot(n) < 0.999 || std::abs((b0 - onPlane).dot(n)) > 1e-4) continue;
            faceTris.push_back(t);
            Geom::Vector3D cn = (curPos(indices[t * 3 + 1]) - curPos(indices[t * 3])).cross(curPos(indices[t * 3 + 2]) - curPos(indices[t * 3]));
            if (cn.dot(n) < -1e-12) flipped = true;
        }
        if (!flipped) return;
        std::vector<uint32_t> loop;
        if (!PlanarBoundaryLoop(vertices, indices, faceTris, loop)) return;
        std::vector<uint32_t> tris;
        if (!EarClipPlanar(vertices, loop, n, tris)) return;
        for (size_t t : faceTris) {
            indices[t * 3 + 1] = indices[t * 3];
            indices[t * 3 + 2] = indices[t * 3]; // degenerate; dropped by the cleanup pass below
        }
        indices.insert(indices.end(), tris.begin(), tris.end());
    };
    retriangulateIfFlipped(normalA, chain.front().p0);
    for (size_t i = 0; i < chainCount; ++i) {
        bool seen = false;
        for (size_t j = 0; j < i; ++j) {
            seen |= chain[j].normalB.dot(chain[i].normalB) > 0.999 &&
                    std::abs((chain[i].p0 - chain[j].p0).dot(chain[i].normalB)) < 1e-4;
        }
        if (!seen) retriangulateIfFlipped(chain[i].normalB, chain[i].p0);
    }

    // Generate smooth fillet surface bands connecting Face A to Face B for each segment in the chain
    const int numBands = (kind == EdgeBlendKind::Round) ? 6 : 1; // a bevel is a single flat band
    std::vector<std::vector<uint32_t>> allBand0(chainCount);
    std::vector<std::vector<uint32_t>> allBand1(chainCount);
    // Blend skin triangles (segment, vertex ids); appended last so a bevel can get flat-shaded copies
    // once the end-cap pass has settled the final profile positions.
    std::vector<std::pair<size_t, std::array<uint32_t, 3>>> skinTris;

    for (size_t i = 0; i < chainCount; ++i) {
        std::vector<uint32_t> band0(numBands + 1);
        std::vector<uint32_t> band1(numBands + 1);

        band0[0] = v_sA[i];
        band1[0] = v_sA[i + 1];
        band0[numBands] = v_sB0[i];
        band1[numBands] = v_sB1[i];

        // Convexity check: uA points into Face A, normalB points outward from Face B
        bool isConvex = (chain[i].uA.dot(chain[i].normalB) < 0.0);

        // Rolling-ball center inside the solid for convex edge, outside for concave edge
        // Centres sit on the (mitered, possibly cusp-collapsed) face-A setback points; a mitered point
        // already lies on this segment's setback line, so this matches p + uA*d up to a slide along the edge.
        Geom::Point3D center0 = isConvex
            ? (sA_pts[i] - normalA * seedRadius)
            : (sA_pts[i] + normalA * seedRadius);
        Geom::Point3D center1 = isConvex
            ? (sA_pts[i + 1] - normalA * seedRadius)
            : (sA_pts[i + 1] + normalA * seedRadius);

        // Segment whose setback collapsed to a point (blend wrapping a round of equal or smaller radius):
        // the rolling ball pivots about one centre, so each junction gets a single spherical cross-section
        // built from the junction's averaged side normal, shared by both neighbouring segments.
        // Decided per junction (not per segment) so both segments meeting there build the same section.
        auto segCollapsed = [&](size_t s) { return sA_pts[s].distance_to(sA_pts[s + 1]) < 1e-9; };
        auto junctionPivots = [&](size_t j) {
            bool prev = (j > 0) ? segCollapsed(j - 1) : (isLoop && segCollapsed(chainCount - 1));
            bool next = (j < chainCount) ? segCollapsed(j) : (isLoop && segCollapsed(0));
            return prev || next;
        };
        const bool pivotStart = junctionPivots(i);
        const bool pivotEnd = junctionPivots(i + 1);
        auto junctionNormalB = [&](size_t j) {
            const Geom::Vector3D& prevN = (j > 0) ? chain[j - 1].normalB : (isLoop ? chain.back().normalB : chain[j].normalB);
            const Geom::Vector3D& nextN = (j < chainCount) ? chain[j].normalB : (isLoop ? chain.front().normalB : chain[j - 1].normalB);
            Geom::Vector3D s = prevN + nextN;
            return s.length() > 1e-9 ? s.normalized() : chain[std::min(j, chainCount - 1)].normalB;
        };
        const Geom::Vector3D pivotN0 = junctionNormalB(i);
        const Geom::Vector3D pivotN1 = junctionNormalB(i + 1);

        for (int b = 1; b < numBands; ++b) {
            float t = static_cast<float>(b) / static_cast<float>(numBands);
            Geom::Vector3D filletNorm = (normalA * (1.0f - t) + chain[i].normalB * t).normalized();

            Geom::Point3D pt0, pt1;
            if (pivotStart) {
                Geom::Vector3D n0 = (normalA * (1.0f - t) + pivotN0 * t).normalized();
                pt0 = isConvex ? (center0 + n0 * seedRadius) : (center0 - n0 * seedRadius);
            } else {
                pt0 = toMiterPlane(isConvex
                    ? (center0 + filletNorm * seedRadius)
                    : (center0 - filletNorm * seedRadius), i, false);
            }
            if (pivotEnd) {
                Geom::Vector3D n1 = (normalA * (1.0f - t) + pivotN1 * t).normalized();
                pt1 = isConvex ? (center1 + n1 * seedRadius) : (center1 - n1 * seedRadius);
            } else {
                pt1 = toMiterPlane(isConvex
                    ? (center1 + filletNorm * seedRadius)
                    : (center1 - filletNorm * seedRadius), i, true);
            }

            band0[b] = static_cast<uint32_t>(vertices.size());
            vertices.push_back({
                {static_cast<float>(pt0.x), static_cast<float>(pt0.y), static_cast<float>(pt0.z)},
                {static_cast<float>(filletNorm.x), static_cast<float>(filletNorm.y), static_cast<float>(filletNorm.z)}
            });

            band1[b] = static_cast<uint32_t>(vertices.size());
            vertices.push_back({
                {static_cast<float>(pt1.x), static_cast<float>(pt1.y), static_cast<float>(pt1.z)},
                {static_cast<float>(filletNorm.x), static_cast<float>(filletNorm.y), static_cast<float>(filletNorm.z)}
            });
        }

        allBand0[i] = band0;
        allBand1[i] = band1;

        // Stitch fillet skin quads
        for (int b = 0; b < numBands; ++b) {
            uint32_t p0_a = band0[b];
            uint32_t p1_a = band1[b];
            uint32_t p0_b = band0[b + 1];
            uint32_t p1_b = band1[b + 1];

            Geom::Point3D pa(vertices[p0_a].position[0], vertices[p0_a].position[1], vertices[p0_a].position[2]);
            Geom::Point3D pb(vertices[p1_a].position[0], vertices[p1_a].position[1], vertices[p1_a].position[2]);
            Geom::Point3D pc(vertices[p0_b].position[0], vertices[p0_b].position[1], vertices[p0_b].position[2]);
            Geom::Point3D pd(vertices[p1_b].position[0], vertices[p1_b].position[1], vertices[p1_b].position[2]);
            // Quad orientation from its diagonals (vector area of p0_a,p1_a,p1_b,p0_b): stays valid when one
            // side is collapsed, e.g. the fan around a cusp where p0_a and p1_a coincide.
            Geom::Vector3D n = (pd - pa).cross(pc - pb);
            Geom::Vector3D expectedNorm = (normalA + chain[i].normalB).normalized();

            if (n.dot(expectedNorm) >= 0.0) {
                skinTris.push_back({i, {p0_a, p1_a, p1_b}});
                skinTris.push_back({i, {p0_a, p1_b, p0_b}});
            } else {
                skinTris.push_back({i, {p0_a, p1_b, p1_a}});
                skinTris.push_back({i, {p0_a, p0_b, p1_b}});
            }
        }

        // Connect miter seam between adjacent segments on Face B
        if (i + 1 < chainCount) {
            uint32_t v_currB1 = v_sB1[i];
            uint32_t v_nextB0 = v_sB0[i + 1];
            if (v_currB1 != v_nextB0 && sB1_pts[i].distance_to(sB0_pts[i + 1]) > 1e-4) {
                uint32_t v_midA = v_sA[i + 1];
                Geom::Point3D pA(vertices[v_midA].position[0], vertices[v_midA].position[1], vertices[v_midA].position[2]);
                Geom::Point3D pB1(vertices[v_currB1].position[0], vertices[v_currB1].position[1], vertices[v_currB1].position[2]);
                Geom::Point3D pB0(vertices[v_nextB0].position[0], vertices[v_nextB0].position[1], vertices[v_nextB0].position[2]);
                Geom::Vector3D tn = (pB1 - pA).cross(pB0 - pA);
                Geom::Vector3D expN = (chain[i].normalB + chain[i + 1].normalB).normalized();
                if (tn.dot(expN) >= 0.0) {
                    indices.push_back(v_midA); indices.push_back(v_currB1); indices.push_back(v_nextB0);
                } else {
                    indices.push_back(v_midA); indices.push_back(v_nextB0); indices.push_back(v_currB1);
                }
            }
        } else if (isLoop) {
            uint32_t v_currB1 = v_sB1.back();
            uint32_t v_nextB0 = v_sB0.front();
            if (v_currB1 != v_nextB0 && sB1_pts.back().distance_to(sB0_pts.front()) > 1e-4) {
                uint32_t v_midA = v_sA.front();
                Geom::Point3D pA(vertices[v_midA].position[0], vertices[v_midA].position[1], vertices[v_midA].position[2]);
                Geom::Point3D pB1(vertices[v_currB1].position[0], vertices[v_currB1].position[1], vertices[v_currB1].position[2]);
                Geom::Point3D pB0(vertices[v_nextB0].position[0], vertices[v_nextB0].position[1], vertices[v_nextB0].position[2]);
                Geom::Vector3D tn = (pB1 - pA).cross(pB0 - pA);
                Geom::Vector3D expN = (chain.back().normalB + chain.front().normalB).normalized();
                if (tn.dot(expN) >= 0.0) {
                    indices.push_back(v_midA); indices.push_back(v_currB1); indices.push_back(v_nextB0);
                } else {
                    indices.push_back(v_midA); indices.push_back(v_nextB0); indices.push_back(v_currB1);
                }
            }
        }
    }

    // Trim the end caps (planar faces meeting the open ends of the chain) to the blend profile so no
    // flat, untrimmed piece of the original sharp corner survives where the fillet runs into another edge.
    if (!isLoop) {
        std::vector<char> removedTris(numTriangles, 0);
        auto basePos = [&](uint32_t i) {
            return Geom::Point3D(baseVerts[i].position[0], baseVerts[i].position[1], baseVerts[i].position[2]);
        };

        // Finds the cap face incident to the chain end J. Winding-agnostic: the returned normal is the
        // cap triangles' own geometric normal, so the re-triangulated cap keeps the mesh's orientation.
        auto findCap = [&](const Geom::Point3D& J, const Geom::Vector3D& outDir,
                           const Geom::Vector3D& nB, Geom::Vector3D& capN) -> bool {
            double best = 0.4;
            bool found = false;
            for (size_t t = 0; t < numTriangles; ++t) {
                Geom::Point3D pa = basePos(baseIndices[t * 3]);
                Geom::Point3D pb = basePos(baseIndices[t * 3 + 1]);
                Geom::Point3D pc = basePos(baseIndices[t * 3 + 2]);
                if (pa.distance_to(J) >= kTol && pb.distance_to(J) >= kTol && pc.distance_to(J) >= kTol) continue;
                Geom::Vector3D tn = (pb - pa).cross(pc - pa);
                if (tn.length() < 1e-9) continue;
                tn = tn.normalized();
                if (std::abs(tn.dot(normalA)) > 0.85 || std::abs(tn.dot(nB)) > 0.85) continue;
                double d = std::abs(tn.dot(outDir));
                if (d > best) { best = d; capN = tn; found = true; }
            }
            return found;
        };

        auto processEnd = [&](const Geom::Point3D& J, const Geom::Vector3D& edgeDir, const Geom::Vector3D& outDir,
                              const Geom::Vector3D& nB, const std::vector<uint32_t>& profile) {
            Geom::Vector3D capN;
            if (!findCap(J, outDir, nB, capN)) return;

            // Slide the profile (setback points + arc) along the edge onto the cap plane, so a slanted
            // cap is met exactly and the trimmed cap stays planar.
            double en = edgeDir.dot(capN);
            if (std::abs(en) > 1e-3) {
                for (uint32_t idx : profile) {
                    Geom::Point3D p(vertices[idx].position[0], vertices[idx].position[1], vertices[idx].position[2]);
                    p = p + edgeDir * ((J - p).dot(capN) / en);
                    vertices[idx].position[0] = static_cast<float>(p.x);
                    vertices[idx].position[1] = static_cast<float>(p.y);
                    vertices[idx].position[2] = static_cast<float>(p.z);
                }
            }

            if (TrimCapCorner(baseVerts, baseIndices, vertices, indices, removedTris, J, capN, profile)) return;

            // Concave end: the edge stops at an inside corner, where the cap plane is a wall that carries
            // on past the edge, so no cap material covers the blend's corner region. The removed notch
            // then needs its own end face (J -> setback A -> profile -> setback B), coplanar with that
            // wall; the wall is left as is and RepairTJunctions stitches its edges to the new points.
            auto capCovers = [&](const Geom::Point3D& q) {
                for (size_t t = 0; t < numTriangles; ++t) {
                    Geom::Point3D p[3] = {basePos(baseIndices[t * 3]), basePos(baseIndices[t * 3 + 1]), basePos(baseIndices[t * 3 + 2])};
                    Geom::Vector3D tn = (p[1] - p[0]).cross(p[2] - p[0]);
                    if (tn.length() < 1e-12 || tn.normalized().dot(capN) < 0.99) continue;
                    if (std::abs((q - p[0]).dot(capN)) > 1e-3) continue;
                    bool inside = true;
                    for (int k = 0; k < 3 && inside; ++k) {
                        inside = (p[(k + 1) % 3] - p[k]).cross(q - p[k]).dot(capN) >= -1e-9;
                    }
                    if (inside) return true;
                }
                return false;
            };
            {
                Geom::Point3D pA(vertices[profile.front()].position[0], vertices[profile.front()].position[1], vertices[profile.front()].position[2]);
                Geom::Point3D pB(vertices[profile.back()].position[0], vertices[profile.back()].position[1], vertices[profile.back()].position[2]);
                Geom::Point3D probe((J.x + pA.x + pB.x) / 3.0, (J.y + pA.y + pB.y) / 3.0, (J.z + pA.z + pB.z) / 3.0);
                if (!capCovers(probe)) {
                    // Shading normal from the wall the end face is coplanar with
                    std::array<float, 3> shadeN{static_cast<float>(capN.x), static_cast<float>(capN.y), static_cast<float>(capN.z)};
                    for (size_t t = 0; t < numTriangles; ++t) {
                        Geom::Point3D p[3] = {basePos(baseIndices[t * 3]), basePos(baseIndices[t * 3 + 1]), basePos(baseIndices[t * 3 + 2])};
                        Geom::Vector3D tn = (p[1] - p[0]).cross(p[2] - p[0]);
                        if (tn.length() < 1e-12 || tn.normalized().dot(capN) < 0.99) continue;
                        const auto& n = baseVerts[baseIndices[t * 3]].normal;
                        shadeN = {n[0], n[1], n[2]};
                        break;
                    }
                    auto addCapVertex = [&](const Geom::Point3D& p) {
                        vertices.push_back({{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)},
                                            {shadeN[0], shadeN[1], shadeN[2]}});
                        return static_cast<uint32_t>(vertices.size() - 1);
                    };
                    const uint32_t vJ = addCapVertex(J);
                    std::vector<uint32_t> ring;
                    for (uint32_t idx : profile) {
                        ring.push_back(addCapVertex(Geom::Point3D(vertices[idx].position[0], vertices[idx].position[1], vertices[idx].position[2])));
                    }
                    for (size_t k = 0; k + 1 < ring.size(); ++k) {
                        Geom::Point3D pa(vertices[ring[k]].position[0], vertices[ring[k]].position[1], vertices[ring[k]].position[2]);
                        Geom::Point3D pb(vertices[ring[k + 1]].position[0], vertices[ring[k + 1]].position[1], vertices[ring[k + 1]].position[2]);
                        if ((pa - J).cross(pb - J).dot(capN) >= 0.0) {
                            indices.insert(indices.end(), {vJ, ring[k], ring[k + 1]});
                        } else {
                            indices.insert(indices.end(), {vJ, ring[k + 1], ring[k]});
                        }
                    }
                    return;
                }
            }

            // Fallback: slide the cap corner onto the face-A setback point and fan over the arc segment
            const uint32_t vA = profile.front();
            for (size_t t = 0; t < numTriangles; ++t) {
                if (removedTris[t]) continue;
                Geom::Point3D pa = basePos(baseIndices[t * 3]);
                Geom::Point3D pb = basePos(baseIndices[t * 3 + 1]);
                Geom::Point3D pc = basePos(baseIndices[t * 3 + 2]);
                Geom::Vector3D tn = (pb - pa).cross(pc - pa).normalized();
                if (tn.dot(capN) <= 0.9) continue;
                for (int k = 0; k < 3; ++k) {
                    if (basePos(baseIndices[t * 3 + k]).distance_to(J) < kTol) indices[t * 3 + k] = vA;
                }
            }
            Geom::Point3D pA(vertices[vA].position[0], vertices[vA].position[1], vertices[vA].position[2]);
            for (size_t b = 1; b + 1 < profile.size(); ++b) {
                uint32_t va = profile[b], vb = profile[b + 1];
                Geom::Point3D pb(vertices[va].position[0], vertices[va].position[1], vertices[va].position[2]);
                Geom::Point3D pc(vertices[vb].position[0], vertices[vb].position[1], vertices[vb].position[2]);
                if ((pb - pA).cross(pc - pA).dot(capN) >= 0.0) {
                    indices.push_back(vA); indices.push_back(va); indices.push_back(vb);
                } else {
                    indices.push_back(vA); indices.push_back(vb); indices.push_back(va);
                }
            }
        };

        processEnd(chain.front().p0, chain.front().edgeVec, -chain.front().edgeVec, chain.front().normalB, allBand0.front());
        processEnd(chain.back().p1, chain.back().edgeVec, chain.back().edgeVec, chain.back().normalB, allBand1.back());
        CompactRemovedTriangles(indices, removedTris);
    }


    // Weld the cross-sections of adjacent chain segments at each junction. Each segment slides its own
    // end section onto the junction's miter plane; on curved chains (e.g. a blend running around an
    // earlier round) the two sections differ slightly, which left cracks and a misaligned fan of skin
    // triangles. Matching points are merged into one shared vertex at their midpoint.
    {
        auto posOf = [&](uint32_t id) {
            return Geom::Point3D(vertices[id].position[0], vertices[id].position[1], vertices[id].position[2]);
        };
        std::unordered_map<uint32_t, uint32_t> weld;
        const double weldTol = std::max(1e-4, 0.1 * static_cast<double>(seedRadius));
        auto weldJunction = [&](size_t prev, size_t next) {
            for (int b = 1; b <= numBands; ++b) {
                uint32_t keep = allBand1[prev][b];
                uint32_t drop = allBand0[next][b];
                if (keep == drop) continue;
                Geom::Point3D pk = posOf(keep), pd = posOf(drop);
                if (pk.distance_to(pd) > weldTol) continue;
                Geom::Point3D mid((pk.x + pd.x) * 0.5, (pk.y + pd.y) * 0.5, (pk.z + pd.z) * 0.5);
                vertices[keep].position[0] = static_cast<float>(mid.x);
                vertices[keep].position[1] = static_cast<float>(mid.y);
                vertices[keep].position[2] = static_cast<float>(mid.z);
                weld[drop] = keep;
            }
        };
        for (size_t i = 0; i + 1 < chainCount; ++i) weldJunction(i, i + 1);
        if (isLoop && chainCount > 1) weldJunction(chainCount - 1, 0);

        if (!weld.empty()) {
            for (uint32_t& id : indices) {
                auto it = weld.find(id);
                if (it != weld.end()) id = it->second;
            }
            for (auto& st : skinTris) {
                for (uint32_t& id : st.second) {
                    auto it = weld.find(id);
                    if (it != weld.end()) id = it->second;
                }
            }
        }

        // Drop zero-area triangles left by collapsed seams or by setback points that coincide (e.g. a
        // blend wrapping an equal-radius round, whose offset curve degenerates to a single point).
        auto degenerate = [&](uint32_t a, uint32_t b, uint32_t c) {
            if (a == b || b == c || a == c) return true;
            Geom::Point3D pa = posOf(a), pb = posOf(b), pc = posOf(c);
            return (pb - pa).cross(pc - pa).length() < 1e-9;
        };
        std::vector<uint32_t> kept;
        kept.reserve(indices.size());
        for (size_t t = 0; t + 2 < indices.size(); t += 3) {
            if (degenerate(indices[t], indices[t + 1], indices[t + 2])) continue;
            kept.insert(kept.end(), {indices[t], indices[t + 1], indices[t + 2]});
        }
        indices = std::move(kept);
        skinTris.erase(std::remove_if(skinTris.begin(), skinTris.end(), [&](const auto& st) {
            return degenerate(st.second[0], st.second[1], st.second[2]);
        }), skinTris.end());
    }

    if (kind == EdgeBlendKind::Bevel) {
        // A bevel is a flat face: give each segment its own vertex copies carrying one face normal
        std::unordered_map<uint64_t, uint32_t> flatCopy;
        for (auto& [seg, tri] : skinTris) {
            Geom::Vector3D n = (normalA + chain[seg].normalB).normalized();
            for (uint32_t& id : tri) {
                uint64_t key = (static_cast<uint64_t>(seg) << 32) | id;
                auto it = flatCopy.find(key);
                if (it == flatCopy.end()) {
                    CADVertex v = vertices[id];
                    v.normal[0] = static_cast<float>(n.x);
                    v.normal[1] = static_cast<float>(n.y);
                    v.normal[2] = static_cast<float>(n.z);
                    vertices.push_back(v);
                    it = flatCopy.emplace(key, static_cast<uint32_t>(vertices.size() - 1)).first;
                }
                id = it->second;
            }
        }
    }
    for (const auto& [seg, tri] : skinTris) {
        indices.insert(indices.end(), tri.begin(), tri.end());
    }
    RepairTJunctions(vertices, indices);
    return true;
}

} // namespace Tessera
