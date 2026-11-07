#pragma once

#include "ExtrudeNode.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace Tessera {

/**
 * @brief Trims the sharp corner of a planar end-cap face where a fillet/chamfer terminates.
 *
 * The cap triangles incident to the sharp end vertex J are removed and the cap is re-triangulated
 * so its boundary follows the blend profile (setback point on face A -> arc/chamfer line -> setback
 * point on face B) instead of the original sharp corner. This leaves no flat untrimmed sliver.
 *
 * @param baseVerts      Vertices of the unmodified parent mesh (positions used to locate J).
 * @param baseIndices    Triangle indices of the unmodified parent mesh.
 * @param vertices       Output vertex array (profile copies with the cap normal are appended).
 * @param indices        Output index array (new cap triangles are appended).
 * @param removedTris    Per-base-triangle removal mask; incident cap triangles are flagged here.
 * @param J              Sharp end vertex of the filleted edge.
 * @param endNormal      Unit geometric (winding) normal of the cap triangles; output keeps this winding.
 * @param profile        Ordered vertex indices (into 'vertices') from the face-A setback point
 *                       to the face-B setback point, including interior arc points.
 * @return true when the cap was trimmed.
 */
inline bool TrimCapCorner(
    const std::vector<CADVertex>& baseVerts,
    const std::vector<uint32_t>& baseIndices,
    std::vector<CADVertex>& vertices,
    std::vector<uint32_t>& indices,
    std::vector<char>& removedTris,
    const Geom::Point3D& J,
    const Geom::Vector3D& endNormal,
    const std::vector<uint32_t>& profile)
{
    if (profile.size() < 2) return false;
    const size_t numTriangles = baseIndices.size() / 3;
    auto posOf = [](const CADVertex& v) { return Geom::Point3D(v.position[0], v.position[1], v.position[2]); };

    // 1. Gather cap triangles incident to J, as directed link edges a -> b (ccw around J)
    std::unordered_map<uint32_t, uint32_t> next;
    std::unordered_map<uint32_t, int> inDegree;
    std::vector<size_t> hit;
    for (size_t t = 0; t < numTriangles; ++t) {
        if (removedTris[t]) continue;
        uint32_t id[3] = {baseIndices[t * 3], baseIndices[t * 3 + 1], baseIndices[t * 3 + 2]};
        Geom::Point3D p[3] = {posOf(baseVerts[id[0]]), posOf(baseVerts[id[1]]), posOf(baseVerts[id[2]])};
        Geom::Vector3D tn = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
        if (tn.dot(endNormal) < 0.9) continue;
        for (int k = 0; k < 3; ++k) {
            if (p[k].distance_to(J) < 1e-3) {
                uint32_t a = id[(k + 1) % 3];
                uint32_t b = id[(k + 2) % 3];
                next[a] = b;
                inDegree[b]++;
                inDegree.emplace(a, 0);
                hit.push_back(t);
                break;
            }
        }
    }
    if (hit.empty()) return false;

    // 2. Order the link path L0..Lm
    uint32_t start = 0;
    bool haveStart = false;
    for (const auto& kv : inDegree) {
        if (kv.second == 0) { start = kv.first; haveStart = true; break; }
    }
    if (!haveStart) return false;
    std::vector<uint32_t> path{start};
    while (next.count(path.back()) && path.size() <= next.size() + 1) path.push_back(next[path.back()]);
    if (path.size() != next.size() + 1) return false;

    // 3. Decide which link end lies on the face-A side
    Geom::Point3D pA = posOf(vertices[profile.front()]);
    Geom::Vector3D dirA = (pA - J).normalized();
    Geom::Vector3D d0 = (posOf(baseVerts[path.front()]) - J).normalized();
    Geom::Vector3D dm = (posOf(baseVerts[path.back()]) - J).normalized();
    bool aAtFront;
    if (d0.dot(dirA) > 0.9) aAtFront = true;
    else if (dm.dot(dirA) > 0.9) aAtFront = false;
    else return false;

    // 4. Build the polygon: link path, then the blend profile traversed from the last link vertex back to the first
    auto addProfileVertex = [&](uint32_t srcIdx) {
        CADVertex v = vertices[srcIdx];
        // Shading normal from the existing cap vertices (endNormal is the geometric/winding normal)
        const CADVertex& capV = baseVerts[path.front()];
        v.normal[0] = capV.normal[0];
        v.normal[1] = capV.normal[1];
        v.normal[2] = capV.normal[2];
        vertices.push_back(v);
        return static_cast<uint32_t>(vertices.size() - 1);
    };
    std::vector<uint32_t> poly(path.begin(), path.end());
    if (aAtFront) {
        for (auto it = profile.rbegin(); it != profile.rend(); ++it) poly.push_back(addProfileVertex(*it));
    } else {
        for (auto it = profile.begin(); it != profile.end(); ++it) poly.push_back(addProfileVertex(*it));
    }

    // 5. Ear-clip the polygon in the cap plane (ccw about endNormal)
    Geom::Vector3D e1 = std::abs(endNormal.x) < 0.9 ? Geom::Vector3D(1, 0, 0) : Geom::Vector3D(0, 1, 0);
    e1 = (e1 - endNormal * e1.dot(endNormal)).normalized();
    Geom::Vector3D e2 = endNormal.cross(e1);
    auto to2D = [&](uint32_t idx, double& x, double& y) {
        Geom::Point3D p = posOf(vertices[idx]);
        x = p.x * e1.x + p.y * e1.y + p.z * e1.z;
        y = p.x * e2.x + p.y * e2.y + p.z * e2.z;
    };
    auto cross2 = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };

    std::vector<uint32_t> work = poly;
    std::vector<uint32_t> newTris;
    int guard = 0;
    while (work.size() > 3 && guard++ < 10000) {
        bool clipped = false;
        const size_t n = work.size();
        for (size_t i = 0; i < n; ++i) {
            uint32_t ia = work[(i + n - 1) % n], ib = work[i], ic = work[(i + 1) % n];
            double ax, ay, bx, by, cx, cy;
            to2D(ia, ax, ay); to2D(ib, bx, by); to2D(ic, cx, cy);
            if (cross2(ax, ay, bx, by, cx, cy) <= 1e-9) continue;
            bool inside = false;
            for (size_t j = 0; j < n && !inside; ++j) {
                uint32_t ip = work[j];
                if (ip == ia || ip == ib || ip == ic) continue;
                double px, py;
                to2D(ip, px, py);
                if (cross2(ax, ay, bx, by, px, py) >= -1e-9 &&
                    cross2(bx, by, cx, cy, px, py) >= -1e-9 &&
                    cross2(cx, cy, ax, ay, px, py) >= -1e-9) {
                    inside = true;
                }
            }
            if (inside) continue;
            newTris.push_back(ia); newTris.push_back(ib); newTris.push_back(ic);
            work.erase(work.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) break;
    }
    if (work.size() == 3) {
        double ax, ay, bx, by, cx, cy;
        to2D(work[0], ax, ay); to2D(work[1], bx, by); to2D(work[2], cx, cy);
        if (cross2(ax, ay, bx, by, cx, cy) > 1e-9) {
            newTris.push_back(work[0]); newTris.push_back(work[1]); newTris.push_back(work[2]);
        }
    } else {
        return false; // could not triangulate; leave cap untouched
    }

    for (size_t t : hit) removedTris[t] = 1;
    indices.insert(indices.end(), newTris.begin(), newTris.end());
    return true;
}

/// Removes flagged base triangles from an index buffer whose first numBaseTris triangles mirror the base mesh.
inline void CompactRemovedTriangles(std::vector<uint32_t>& indices, const std::vector<char>& removedTris) {
    std::vector<uint32_t> out;
    out.reserve(indices.size());
    for (size_t t = 0; t * 3 < indices.size(); ++t) {
        if (t < removedTris.size() && removedTris[t]) continue;
        out.push_back(indices[t * 3]);
        out.push_back(indices[t * 3 + 1]);
        out.push_back(indices[t * 3 + 2]);
    }
    indices = std::move(out);
}

} // namespace Tessera

