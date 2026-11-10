#pragma once

#include "Node.hpp"
#include "SketchNode.hpp"
#include "DatumPlaneNode.hpp"
#include "DatumAxisNode.hpp"
#include "engine/concurrency/ThreadPool.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/hal/RHI.hpp"
#include "engine/geom/Transform3D.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <iostream>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

namespace Tessera {

/// Information describing an entire circular or cylindrical rim loop formed by connected segments
struct CircularEdgeInfo {
    bool isCircle{false};
    double radius{0.0};
    double diameter{0.0};
    double circumference{0.0};
    double area{0.0};
    Geom::Point3D center{0.0, 0.0, 0.0};
    Geom::Vector3D normal{0.0, 1.0, 0.0};
    std::vector<int> segments; // Edge indices forming this circular loop
    std::vector<Geom::Point3D> points; // Ordered points along loop
};

/// Pick result for sub-elements (Face, Vertex, or Edge) on a selected 3D model
struct SubElementPickResult {
    uint64_t nodeId{0};
    int faceIndex{-1};
    int vertexIndex{-1};
    int edgeIndex{-1};
    Geom::Point3D edgeP0{0.0, 0.0, 0.0};
    Geom::Point3D edgeP1{0.0, 0.0, 0.0};
    Geom::Plane facePlane;
    Geom::Point3D vertexPos{0.0, 0.0, 0.0};
    double distance{std::numeric_limits<double>::infinity()};
    CircularEdgeInfo circularEdge;
};

/// Filter mode to restrict what sub-elements can be picked
enum class SelectionFilter : uint8_t {
    All = 0,         // Auto: Points -> Edges -> Planes
    PointsOnly = 1,  // Only select vertices / points
    PlanesOnly = 2,  // Only select faces / planes
    EdgesOnly = 3    // Only select corner edges
};

/// Preset resolution for point/edge/plane picking tolerance
enum class SelectionResolutionPreset : uint8_t {
    UltraFine = 0, // 0.05m hit radius (dense meshes / packed points)
    Fine = 1,      // 0.12m hit radius
    Medium = 2,    // 0.25m hit radius (default CAD standard)
    Coarse = 3,    // 0.45m hit radius (relaxed picking)
    Custom = 4     // Custom user tolerance slider
};

/// Möller-Trumbore ray-triangle intersection test (two-sided)
inline bool RayIntersectsTriangle(
    const Geom::Point3D& rayOrigin,
    const Geom::Vector3D& rayDir,
    const Geom::Point3D& v0,
    const Geom::Point3D& v1,
    const Geom::Point3D& v2,
    double& outT
) noexcept {
    constexpr double EPSILON = 1e-8;

    Geom::Vector3D edge1 = v1 - v0;
    Geom::Vector3D edge2 = v2 - v0;
    Geom::Vector3D h = rayDir.cross(edge2);
    double a = edge1.dot(h);

    if (std::abs(a) < EPSILON) {
        return false; // Ray is parallel to triangle
    }

    double f = 1.0 / a;
    Geom::Vector3D s = rayOrigin - v0;
    double u = f * s.dot(h);

    if (u < -EPSILON || u > 1.0 + EPSILON) {
        return false;
    }

    Geom::Vector3D q = s.cross(edge1);
    double v = f * rayDir.dot(q);

    if (v < -EPSILON || (u + v) > 1.0 + EPSILON) {
        return false;
    }

    double t = f * edge2.dot(q);
    if (t > EPSILON) {
        outT = t;
        return true;
    }

    return false;
}

/// Detects if an edge segment is part of a circular loop (e.g. cylinder rim or hole)
inline CircularEdgeInfo DetectCircularLoop(
    const std::vector<CADVertex>& verts,
    const std::vector<uint32_t>& edges,
    int seedEdgeIdx
) {
    CircularEdgeInfo info;
    if (seedEdgeIdx < 0 || static_cast<size_t>(seedEdgeIdx * 2 + 1) >= edges.size()) return info;

    const size_t numEdges = edges.size() / 2;
    if (numEdges < 3) return info;

    // Helper: 3D spatial grid with 20mm cell size for robust vertex connectivity
    // Tolerates floating-point imprecision and split normal vertices across boundaries
    constexpr double GRID_CELL = 0.02; // 20mm
    auto cellCoord = [](double val) -> int32_t {
        return static_cast<int32_t>(std::floor(val / GRID_CELL));
    };
    auto makeCellKey = [](int32_t cx, int32_t cy, int32_t cz) -> uint64_t {
        return (static_cast<uint64_t>(cx & 0x1FFFFF) << 42) |
               (static_cast<uint64_t>(cy & 0x1FFFFF) << 21) |
               (static_cast<uint64_t>(cz & 0x1FFFFF));
    };

    std::unordered_map<uint64_t, std::vector<std::pair<int, int>>> grid;
    grid.reserve(numEdges * 2);

    auto getEdgePt = [&](int eIdx, int endPt) -> Geom::Point3D {
        uint32_t vi = edges[static_cast<size_t>(eIdx * 2 + endPt)];
        return Geom::Point3D(verts[vi].position[0], verts[vi].position[1], verts[vi].position[2]);
    };

    for (size_t e = 0; e < numEdges; ++e) {
        uint32_t i0 = edges[e * 2];
        uint32_t i1 = edges[e * 2 + 1];
        if (i0 >= verts.size() || i1 >= verts.size()) continue;
        Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
        Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
        if (p0.distance_to(p1) < 1e-5) continue;

        grid[makeCellKey(cellCoord(p0.x), cellCoord(p0.y), cellCoord(p0.z))].push_back({static_cast<int>(e), 0});
        grid[makeCellKey(cellCoord(p1.x), cellCoord(p1.y), cellCoord(p1.z))].push_back({static_cast<int>(e), 1});
    }

    // Query 27 neighboring cells for incident edges within 5mm Euclidean distance
    auto findIncidentEdges = [&](const Geom::Point3D& pt, int excludeEdge) {
        std::vector<std::pair<int, int>> result;
        int32_t cx = cellCoord(pt.x);
        int32_t cy = cellCoord(pt.y);
        int32_t cz = cellCoord(pt.z);

        for (int32_t dx = -1; dx <= 1; ++dx) {
            for (int32_t dy = -1; dy <= 1; ++dy) {
                for (int32_t dz = -1; dz <= 1; ++dz) {
                    uint64_t k = makeCellKey(cx + dx, cy + dy, cz + dz);
                    auto it = grid.find(k);
                    if (it != grid.end()) {
                        for (const auto& item : it->second) {
                            if (item.first == excludeEdge) continue;
                            Geom::Point3D candPt = getEdgePt(item.first, item.second);
                            if (candPt.distance_to(pt) <= 0.005) { // 5mm
                                result.push_back(item);
                            }
                        }
                    }
                }
            }
        }
        return result;
    };

    std::vector<int> loopEdges;
    std::unordered_set<int> visitedEdges;
    std::vector<Geom::Point3D> loopPoints;

    Geom::Point3D startP0 = getEdgePt(seedEdgeIdx, 0);
    Geom::Point3D startP1 = getEdgePt(seedEdgeIdx, 1);
    double seedLen = startP0.distance_to(startP1);
    if (seedLen < 1e-4) return info;

    loopEdges.push_back(seedEdgeIdx);
    visitedEdges.insert(seedEdgeIdx);
    loopPoints.push_back(startP0);
    loopPoints.push_back(startP1);

    // Forward walk from startP1
    Geom::Point3D curPt = startP1;
    int curEdge = seedEdgeIdx;
    while (loopEdges.size() < numEdges) {
        auto incident = findIncidentEdges(curPt, curEdge);
        if (incident.empty()) break;

        int nextEdge = -1;
        Geom::Point3D nextPt;
        double bestCos = -2.0;

        for (const auto& [eIdx, endPt] : incident) {
            if (visitedEdges.count(eIdx)) continue;
            Geom::Point3D other = getEdgePt(eIdx, 1 - endPt);

            // Avoid backtrack along coincident reverse edge
            if (other.distance_to(loopPoints[loopPoints.size() - 2]) < 0.005) continue;

            double edgeLen = curPt.distance_to(other);
            if (std::abs(edgeLen - seedLen) > 0.50 * seedLen) continue;

            Geom::Vector3D prevDir = (curPt - loopPoints[loopPoints.size() - 2]).normalized();
            Geom::Vector3D nextDir = (other - curPt).normalized();
            double c = prevDir.dot(nextDir);
            if (c > bestCos) {
                bestCos = c;
                nextEdge = eIdx;
                nextPt = other;
            }
        }

        if (nextEdge < 0 || bestCos < 0.3) break;

        loopEdges.push_back(nextEdge);
        visitedEdges.insert(nextEdge);
        curEdge = nextEdge;
        curPt = nextPt;

        if (curPt.distance_to(startP0) <= 0.005) {
            break; // Closed loop reached!
        }
        loopPoints.push_back(curPt);
    }

    // Backward walk from startP0 if loop not yet closed
    if (curPt.distance_to(startP0) > 0.005) {
        curPt = startP0;
        curEdge = seedEdgeIdx;
        while (loopEdges.size() < numEdges) {
            auto incident = findIncidentEdges(curPt, curEdge);
            if (incident.empty()) break;

            int prevEdge = -1;
            Geom::Point3D prevPt;
            double bestCos = -2.0;

            for (const auto& [eIdx, endPt] : incident) {
                if (visitedEdges.count(eIdx)) continue;
                Geom::Point3D other = getEdgePt(eIdx, 1 - endPt);

                if (other.distance_to(loopPoints[1]) < 0.005) continue;

                double edgeLen = curPt.distance_to(other);
                if (std::abs(edgeLen - seedLen) > 0.50 * seedLen) continue;

                Geom::Vector3D forwardDir = (loopPoints[1] - curPt).normalized();
                Geom::Vector3D backwardDir = (curPt - other).normalized();
                double c = backwardDir.dot(forwardDir);
                if (c > bestCos) {
                    bestCos = c;
                    prevEdge = eIdx;
                    prevPt = other;
                }
            }

            if (prevEdge < 0 || bestCos < 0.3) break;

            loopEdges.insert(loopEdges.begin(), prevEdge);
            visitedEdges.insert(prevEdge);
            curEdge = prevEdge;
            curPt = prevPt;
            loopPoints.insert(loopPoints.begin(), curPt);

            if (curPt.distance_to(loopPoints.back()) <= 0.005) {
                break;
            }
        }
    }

    if (loopEdges.size() < 6) return info;

    std::vector<Geom::Point3D> uniquePts;
    for (const auto& p : loopPoints) {
        bool duplicate = false;
        for (const auto& u : uniquePts) {
            if (u.distance_to(p) < 0.005) { duplicate = true; break; }
        }
        if (!duplicate) uniquePts.push_back(p);
    }
    if (uniquePts.size() < 6) return info;

    // Plane fitting via Newell's method
    Geom::Vector3D normal(0.0, 0.0, 0.0);
    Geom::Point3D centroid(0.0, 0.0, 0.0);
    for (size_t i = 0; i < uniquePts.size(); ++i) {
        const auto& cur = uniquePts[i];
        const auto& next = uniquePts[(i + 1) % uniquePts.size()];
        normal.x += (cur.y - next.y) * (cur.z + next.z);
        normal.y += (cur.z - next.z) * (cur.x + next.x);
        normal.z += (cur.x - next.x) * (cur.y + next.y);
        centroid.x += cur.x;
        centroid.y += cur.y;
        centroid.z += cur.z;
    }
    double nLen = normal.length();
    if (nLen < 1e-6) return info;
    normal = normal / nLen;
    double invSize = 1.0 / static_cast<double>(uniquePts.size());
    centroid.x *= invSize;
    centroid.y *= invSize;
    centroid.z *= invSize;

    // Verify coplanarity
    for (const auto& p : uniquePts) {
        double dist = std::abs((p - centroid).dot(normal));
        if (dist > 0.06) return info;
    }

    // 2D Orthonormal Basis on plane
    Geom::Vector3D uAxis = (std::abs(normal.y) < 0.9) ? Geom::Vector3D(0.0, 1.0, 0.0).cross(normal).normalized()
                                                      : Geom::Vector3D(1.0, 0.0, 0.0).cross(normal).normalized();
    Geom::Vector3D vAxis = normal.cross(uAxis).normalized();

    // 2D Kåsa algebraic circle fit
    double suu = 0.0, svv = 0.0, suv = 0.0;
    double suw = 0.0, svw = 0.0, sw = 0.0;
    const double K = static_cast<double>(uniquePts.size());

    for (const auto& p : uniquePts) {
        Geom::Vector3D diff = p - centroid;
        double u = diff.dot(uAxis);
        double v = diff.dot(vAxis);
        double w = u * u + v * v;
        suu += u * u;
        svv += v * v;
        suv += u * v;
        suw += u * w;
        svw += v * w;
        sw += w;
    }

    double det = suu * svv - suv * suv;
    if (std::abs(det) < 1e-8) return info;

    double uc = (0.5 * suw * svv - 0.5 * svw * suv) / det;
    double vc = (suu * 0.5 * svw - suv * 0.5 * suw) / det;
    double R2 = (sw / K) + uc * uc + vc * vc;
    if (R2 <= 0.0001) return info;

    double R = std::sqrt(R2);
    Geom::Point3D center = centroid + uAxis * uc + vAxis * vc;

    // Check fit residual for circle validity
    for (const auto& p : uniquePts) {
        double d = p.distance_to(center);
        if (std::abs(d - R) > 0.08 * R && std::abs(d - R) > 0.05) {
            return info;
        }
    }

    const double PI_CONST = 3.14159265358979323846;
    info.isCircle = true;
    info.radius = R;
    info.diameter = 2.0 * R;
    info.circumference = 2.0 * PI_CONST * R;
    info.area = PI_CONST * R * R;
    info.center = center;
    info.normal = normal;
    info.segments = loopEdges;
    info.points = uniquePts;

    return info;
}

class ParametricGraph {
public:
    explicit ParametricGraph(std::shared_ptr<ThreadPool> threadPool)
        : m_threadPool(std::move(threadPool)) {}

    void AddNode(std::shared_ptr<DAGNode> node) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        if (node) {
            for (const auto& in : node->GetInputs()) {
                if (in) {
                    node->AddInput(in);
                }
            }
            if (auto targetSolid = node->GetTargetSolid()) {
                node->AddInput(targetSolid);
                targetSolid->SetConsumed(true);
            }
            if (auto plane = std::dynamic_pointer_cast<DatumPlaneNode>(node)) {
                if (auto feat = plane->GetReferencedFeature()) {
                    plane->AddInput(feat);
                }
            } else if (auto axis = std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                if (auto feat = axis->GetReferencedFeature()) {
                    axis->AddInput(feat);
                }
            } else if (auto sketch = std::dynamic_pointer_cast<SketchNode>(node)) {
                if (auto datumPlane = sketch->GetDatumPlaneReference()) {
                    sketch->AddInput(datumPlane);
                }
            }
            if (auto child = node->GetChildSketch()) {
                if (auto sketch = std::dynamic_pointer_cast<SketchNode>(child)) {
                    sketch->SetParentFeature(node);
                    sketch->SetVisible(false);
                    sketch->SetConsumed(true);
                    node->AddInput(sketch);
                }
            }
        }
        m_nodes.push_back(node);
    }

    /// Evaluates any dirty nodes asynchronously via the ThreadPool
    void EvaluateDirtyNodes(RHI::RHIDevice* device, AtomicMeshExchange& exchange) {
        std::vector<std::shared_ptr<DAGNode>> dirtyMetadataNodes;
        std::vector<std::shared_ptr<DAGNode>> dirtyList;
        {
            std::lock_guard<std::mutex> lock(m_graphMutex);
            for (const auto& node : m_nodes) {
                if (node->IsDirty()) {
                    if (std::dynamic_pointer_cast<DatumPlaneNode>(node) ||
                        std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                        dirtyMetadataNodes.push_back(node);
                    } else {
                        dirtyList.push_back(node);
                    }
                }
            }
        }

        // Evaluate lightweight datum planes/axes synchronously first so dependent sketches and features read updated planes
        for (auto& metaNode : dirtyMetadataNodes) {
            metaNode->Evaluate(device, exchange);
        }

        for (auto& node : dirtyList) {
            if (std::dynamic_pointer_cast<SketchNode>(node)) {
                // Evaluate 2D sketches synchronously first so downstream 3D features and the viewport immediately have the updated 3D sketch plane/mesh
                node->Evaluate(device, exchange);
                PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
            } else if (m_threadPool) {
                // Submit heavy 3D tessellations to background thread pool
                m_threadPool->Enqueue([this, node, device, &exchange]() {
                    node->Evaluate(device, exchange);
                    PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                });
            } else {
                node->Evaluate(device, exchange);
                PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
            }
        }
    }

    void SetSelectedNodeId(uint64_t id) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedNodeId = id;
        m_selectedFaceIndex = -1;
        m_selectedVertexIndex = -1;
        m_selectedEdgeIndex = -1;
        m_selectedFaceIndices.clear();
        m_selectedVertexIndices.clear();
        m_selectedCircle = {};
        m_selectedEdgeIndices.clear();
    }

    [[nodiscard]] uint64_t GetSelectedNodeId() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedNodeId;
    }

    void SetSubElementSelection(uint64_t nodeId, int faceIndex, int vertexIndex, int edgeIndex = -1) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedNodeId = nodeId;
        m_selectedFaceIndex = faceIndex;
        m_selectedVertexIndex = vertexIndex;
        m_selectedEdgeIndex = edgeIndex;
        m_selectedFaceIndices.clear();
        m_selectedVertexIndices.clear();
        m_selectedCircle = {};
        m_selectedEdgeIndices.clear();
        if (faceIndex >= 0) m_selectedFaceIndices.push_back(faceIndex);
        if (vertexIndex >= 0) m_selectedVertexIndices.push_back(vertexIndex);
        if (edgeIndex >= 0) m_selectedEdgeIndices.push_back(edgeIndex);
    }

    void SetSubElementMultiSelection(
        uint64_t nodeId,
        const std::vector<int>& faceIndices,
        const std::vector<int>& vertexIndices,
        int edgeIndex = -1
    ) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedNodeId = nodeId;
        m_selectedFaceIndices = faceIndices;
        m_selectedVertexIndices = vertexIndices;
        m_selectedFaceIndex = faceIndices.empty() ? -1 : faceIndices.back();
        m_selectedVertexIndex = vertexIndices.empty() ? -1 : vertexIndices.back();
        m_selectedEdgeIndex = edgeIndex;
        m_selectedCircle = {};
        m_selectedEdgeIndices.clear();
        if (edgeIndex >= 0) m_selectedEdgeIndices.push_back(edgeIndex);
    }

    void ToggleVertexSelection(int vertexIndex) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        auto it = std::find(m_selectedVertexIndices.begin(), m_selectedVertexIndices.end(), vertexIndex);
        if (it != m_selectedVertexIndices.end()) {
            m_selectedVertexIndices.erase(it);
        } else {
            m_selectedVertexIndices.push_back(vertexIndex);
        }
        m_selectedVertexIndex = m_selectedVertexIndices.empty() ? -1 : m_selectedVertexIndices.back();
    }

    void ToggleFaceSelection(int faceIndex) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        auto it = std::find(m_selectedFaceIndices.begin(), m_selectedFaceIndices.end(), faceIndex);
        if (it != m_selectedFaceIndices.end()) {
            m_selectedFaceIndices.erase(it);
        } else {
            m_selectedFaceIndices.push_back(faceIndex);
        }
        m_selectedFaceIndex = m_selectedFaceIndices.empty() ? -1 : m_selectedFaceIndices.back();
    }

    void ClearSubElementSelection() {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedFaceIndex = -1;
        m_selectedVertexIndex = -1;
        m_selectedEdgeIndex = -1;
        m_selectedFaceIndices.clear();
        m_selectedVertexIndices.clear();
        m_selectedCircle = {};
        m_selectedEdgeIndices.clear();
    }

    void GetSubElementSelection(uint64_t& outNodeId, int& outFaceIndex, int& outVertexIndex) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        outNodeId = m_selectedNodeId;
        outFaceIndex = m_selectedFaceIndex;
        outVertexIndex = m_selectedVertexIndex;
    }

    [[nodiscard]] int GetSelectedFaceIndex() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedFaceIndex;
    }

    [[nodiscard]] int GetSelectedVertexIndex() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedVertexIndex;
    }

    [[nodiscard]] int GetSelectedEdgeIndex() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedEdgeIndex;
    }

    [[nodiscard]] std::vector<int> GetSelectedFaceIndices() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedFaceIndices;
    }

    [[nodiscard]] std::vector<int> GetSelectedVertexIndices() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedVertexIndices;
    }

    [[nodiscard]] const CircularEdgeInfo& GetSelectedCircle() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedCircle;
    }

    [[nodiscard]] const std::vector<int>& GetSelectedEdgeIndices() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_selectedEdgeIndices;
    }

    void SetSelectedCircle(const CircularEdgeInfo& circle) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_selectedCircle = circle;
        m_selectedEdgeIndices = circle.isCircle ? circle.segments : (m_selectedEdgeIndex >= 0 ? std::vector<int>{m_selectedEdgeIndex} : std::vector<int>{});
    }

    void PublishCombinedSceneMesh(
        RHI::RHIDevice* device, 
        AtomicMeshExchange& exchange,
        std::optional<uint64_t> selectedNodeIdOverride = std::nullopt,
        std::optional<int> selectedFaceIndexOverride = std::nullopt,
        std::optional<int> selectedVertexIndexOverride = std::nullopt,
        std::optional<int> selectedEdgeIndexOverride = std::nullopt,
        const std::vector<int>* selectedFaceIndicesOverride = nullptr,
        const std::vector<int>* selectedVertexIndicesOverride = nullptr
    ) {
        if (!device) return;
        std::lock_guard<std::mutex> lock(m_graphMutex);
        if (selectedNodeIdOverride.has_value()) {
            if (*selectedNodeIdOverride != m_selectedNodeId) {
                m_selectedNodeId = *selectedNodeIdOverride;
                m_selectedFaceIndex = -1;
                m_selectedVertexIndex = -1;
                m_selectedEdgeIndex = -1;
                m_selectedFaceIndices.clear();
                m_selectedVertexIndices.clear();
            }
        }
        if (selectedFaceIndicesOverride != nullptr) {
            m_selectedFaceIndices = *selectedFaceIndicesOverride;
            m_selectedFaceIndex = m_selectedFaceIndices.empty() ? -1 : m_selectedFaceIndices.back();
        } else if (selectedFaceIndexOverride.has_value()) {
            m_selectedFaceIndex = *selectedFaceIndexOverride;
            m_selectedFaceIndices.clear();
            if (*selectedFaceIndexOverride >= 0) {
                m_selectedFaceIndices.push_back(*selectedFaceIndexOverride);
            }
        }
        if (selectedVertexIndicesOverride != nullptr) {
            m_selectedVertexIndices = *selectedVertexIndicesOverride;
            m_selectedVertexIndex = m_selectedVertexIndices.empty() ? -1 : m_selectedVertexIndices.back();
        } else if (selectedVertexIndexOverride.has_value()) {
            m_selectedVertexIndex = *selectedVertexIndexOverride;
            m_selectedVertexIndices.clear();
            if (*selectedVertexIndexOverride >= 0) {
                m_selectedVertexIndices.push_back(*selectedVertexIndexOverride);
            }
        }
        if (selectedEdgeIndexOverride.has_value()) {
            m_selectedEdgeIndex = *selectedEdgeIndexOverride;
        }

        std::vector<CADVertex> combinedVertices;
        std::vector<uint32_t> combinedIndices;
        std::vector<uint32_t> combinedEdges;

        std::vector<CADVertex> selectedVertices;
        std::vector<uint32_t> selectedIndices;
        std::vector<uint32_t> selectedEdges;

        std::vector<CADVertex> sketchVertices;
        std::vector<uint32_t> sketchIndices;
        std::vector<uint32_t> sketchEdges;

        for (const auto& node : m_nodes) {
            if (!node) continue;

            if (auto sketch = std::dynamic_pointer_cast<SketchNode>(node)) {
                // If this sketch is selected on the model tree, ALWAYS render it with outline and transparent shading
                if (sketch->GetId() != m_selectedNodeId) {
                    continue;
                }

                const auto& verts = sketch->GetCachedVertices();
                const auto& indices = sketch->GetCachedIndices();
                const auto& edges = sketch->GetCachedEdges();

                uint32_t vOffset = static_cast<uint32_t>(sketchVertices.size());
                sketchVertices.insert(sketchVertices.end(), verts.begin(), verts.end());

                for (uint32_t idx : indices) {
                    sketchIndices.push_back(idx + vOffset);
                }
                for (uint32_t edgeIdx : edges) {
                    sketchEdges.push_back(edgeIdx + vOffset);
                }
            } else {
                // Solid 3D features (Extrude, Revolve, Cylinder, Sphere, Fillet, Chamfer, etc.)
                if (node->IsConsumed()) {
                    continue; // Consumed features are superseded by their downstream operations
                }

                // If this node is an earlier sibling targeting the same base feature as an active downstream/selected sibling, skip it
                bool superseded = false;
                for (const auto& other : m_nodes) {
                    if (other && other != node && !other->IsConsumed()) {
                        if (!node->GetInputs().empty() && !other->GetInputs().empty()) {
                            if (node->GetInputs()[0] == other->GetInputs()[0]) {
                                if (other->GetId() == m_selectedNodeId || other->GetId() > node->GetId()) {
                                    superseded = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (superseded) continue;

                const auto& verts = node->GetCachedVertices();
                const auto& indices = node->GetCachedIndices();
                const auto& edges = node->GetCachedEdges();
                if (verts.empty() || indices.empty()) continue;

                bool isSelected = (node->GetId() == m_selectedNodeId);
                if (!isSelected && m_selectedNodeId != 0) {
                    for (const auto& candidate : m_nodes) {
                        if (candidate && candidate->GetId() == m_selectedNodeId) {
                            if (candidate->IsConsumed() && candidate->IsSolidFeature()) {
                                auto leaf = GetActiveLeafForNode(candidate);
                                if (leaf == node) {
                                    isSelected = true;
                                }
                            }
                            break;
                        }
                    }
                }

                if (isSelected && (!m_selectedEdgeIndices.empty() || m_selectedEdgeIndex >= 0)) {
                    // Edge drill-down selection: render whole model + selected corner edge(s) in highlighted selectedEdges buffer!
                    uint32_t unselBase = static_cast<uint32_t>(combinedVertices.size());
                    combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());
                    for (uint32_t idx : indices) combinedIndices.push_back(idx + unselBase);
                    for (uint32_t edgeIdx : edges) combinedEdges.push_back(edgeIdx + unselBase);

                    uint32_t selBase = static_cast<uint32_t>(selectedVertices.size());
                    selectedVertices.insert(selectedVertices.end(), verts.begin(), verts.end());

                    if (m_selectedCircle.isCircle && !m_selectedCircle.segments.empty()) {
                        for (int segIdx : m_selectedCircle.segments) {
                            size_t e = static_cast<size_t>(segIdx * 2);
                            if (e + 1 < edges.size()) {
                                selectedEdges.push_back(edges[e] + selBase);
                                selectedEdges.push_back(edges[e + 1] + selBase);
                            }
                        }
                    } else if (!m_selectedEdgeIndices.empty()) {
                        for (int segIdx : m_selectedEdgeIndices) {
                            size_t e = static_cast<size_t>(segIdx * 2);
                            if (e + 1 < edges.size()) {
                                selectedEdges.push_back(edges[e] + selBase);
                                selectedEdges.push_back(edges[e + 1] + selBase);
                            }
                        }
                    } else if (m_selectedEdgeIndex >= 0) {
                        size_t e = static_cast<size_t>(m_selectedEdgeIndex * 2);
                        if (e + 1 < edges.size()) {
                            selectedEdges.push_back(edges[e] + selBase);
                            selectedEdges.push_back(edges[e + 1] + selBase);
                        }
                    }
                } else if (isSelected && (!m_selectedFaceIndices.empty() || m_selectedFaceIndex >= 0)) {
                    // Face drill-down selection (single or multi-face)
                    std::unordered_set<int> selFaceSet;
                    if (!m_selectedFaceIndices.empty()) {
                        for (int fi : m_selectedFaceIndices) if (fi >= 0) selFaceSet.insert(fi);
                    } else if (m_selectedFaceIndex >= 0) {
                        selFaceSet.insert(m_selectedFaceIndex);
                    }

                    std::unordered_set<size_t> allSelectedTriangles;
                    uint32_t selBase = static_cast<uint32_t>(selectedVertices.size());
                    selectedVertices.insert(selectedVertices.end(), verts.begin(), verts.end());

                    for (int fIdx : selFaceSet) {
                        FaceInfo selFace;
                        if (node->GetFace(fIdx, selFace) && !selFace.triangleIndices.empty()) {
                            for (uint32_t idx : selFace.triangleIndices) {
                                selectedIndices.push_back(idx + selBase);
                            }
                            for (uint32_t edgeIdx : selFace.perimeterEdges) {
                                selectedEdges.push_back(edgeIdx + selBase);
                            }
                            for (size_t k = 0; k < selFace.triangleIndices.size(); k += 3) {
                                uint32_t fa = selFace.triangleIndices[k + 0];
                                uint32_t fb = selFace.triangleIndices[k + 1];
                                uint32_t fc = selFace.triangleIndices[k + 2];
                                for (size_t t = 0; t < indices.size() / 3; ++t) {
                                    if (indices[t * 3 + 0] == fa && indices[t * 3 + 1] == fb && indices[t * 3 + 2] == fc) {
                                        allSelectedTriangles.insert(t);
                                    }
                                }
                            }
                        }
                    }

                    // Put all non-selected triangles into unselected combined buffers
                    uint32_t unselBase = static_cast<uint32_t>(combinedVertices.size());
                    combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());
                    for (size_t t = 0; t < indices.size() / 3; ++t) {
                        if (allSelectedTriangles.find(t) == allSelectedTriangles.end()) {
                            combinedIndices.push_back(indices[t * 3 + 0] + unselBase);
                            combinedIndices.push_back(indices[t * 3 + 1] + unselBase);
                            combinedIndices.push_back(indices[t * 3 + 2] + unselBase);
                        }
                    }
                    for (uint32_t edgeIdx : edges) {
                        combinedEdges.push_back(edgeIdx + unselBase);
                    }
                } else if (isSelected && (!m_selectedVertexIndices.empty() || m_selectedVertexIndex >= 0)) {
                    // Vertex drill-down selection (single or multi-vertex)
                    std::unordered_set<int> selVertSet;
                    if (!m_selectedVertexIndices.empty()) {
                        for (int vi : m_selectedVertexIndices) {
                            if (vi >= 0 && static_cast<size_t>(vi) < verts.size()) {
                                selVertSet.insert(vi);
                            }
                        }
                    } else if (m_selectedVertexIndex >= 0 && static_cast<size_t>(m_selectedVertexIndex) < verts.size()) {
                        selVertSet.insert(m_selectedVertexIndex);
                    }

                    uint32_t unselBase = static_cast<uint32_t>(combinedVertices.size());
                    combinedVertices.insert(combinedVertices.end(), verts.begin(), verts.end());
                    for (uint32_t idx : indices) combinedIndices.push_back(idx + unselBase);
                    for (uint32_t edgeIdx : edges) combinedEdges.push_back(edgeIdx + unselBase);

                    for (int vIdx : selVertSet) {
                        // 1. Incident model edges in vibrant electric cyan
                        uint32_t uV = static_cast<uint32_t>(vIdx);
                        for (size_t e = 0; e + 1 < edges.size(); e += 2) {
                            if (edges[e] == uV || edges[e + 1] == uV) {
                                uint32_t selBase = static_cast<uint32_t>(selectedVertices.size());
                                selectedVertices.push_back(verts[edges[e]]);
                                selectedVertices.push_back(verts[edges[e + 1]]);
                                selectedEdges.push_back(selBase);
                                selectedEdges.push_back(selBase + 1);
                            }
                        }

                        // 2. Subtle 3D point dot marker for selected vertex
                        const auto& v = verts[vIdx];
                        float vx = v.position[0];
                        float vy = v.position[1];
                        float vz = v.position[2];
                        float s = 0.035f;

                        uint32_t markBase = static_cast<uint32_t>(selectedVertices.size());
                        selectedVertices.push_back({.position = {vx, vy + s, vz}, .normal = {0, 1, 0}});
                        selectedVertices.push_back({.position = {vx - s, vy, vz}, .normal = {-1, 0, 0}});
                        selectedVertices.push_back({.position = {vx + s, vy, vz}, .normal = {1, 0, 0}});
                        selectedVertices.push_back({.position = {vx, vy, vz - s}, .normal = {0, 0, -1}});
                        selectedVertices.push_back({.position = {vx, vy, vz + s}, .normal = {0, 0, 1}});
                        selectedVertices.push_back({.position = {vx, vy - s, vz}, .normal = {0, -1, 0}});

                        std::vector<uint32_t> mTris = {
                            0, 1, 3,  0, 3, 2,  0, 2, 4,  0, 4, 1,
                            5, 3, 1,  5, 2, 3,  5, 4, 2,  5, 1, 4
                        };
                        for (uint32_t mi : mTris) selectedIndices.push_back(markBase + mi);
                        std::vector<uint32_t> mEdges = {
                            0, 1,  0, 2,  0, 3,  0, 4,
                            5, 1,  5, 2,  5, 3,  5, 4,
                            1, 3,  3, 2,  2, 4,  4, 1
                        };
                        for (uint32_t me : mEdges) selectedEdges.push_back(markBase + me);

                        // 3. Compact 3D dot crosshair along X, Y, Z at vertex
                        float axisLen = 0.045f;
                        uint32_t axisBase = static_cast<uint32_t>(selectedVertices.size());
                        selectedVertices.push_back({.position = {vx - axisLen, vy, vz}, .normal = {1, 0, 0}});
                        selectedVertices.push_back({.position = {vx + axisLen, vy, vz}, .normal = {1, 0, 0}});
                        selectedVertices.push_back({.position = {vx, vy - axisLen, vz}, .normal = {0, 1, 0}});
                        selectedVertices.push_back({.position = {vx, vy + axisLen, vz}, .normal = {0, 1, 0}});
                        selectedVertices.push_back({.position = {vx, vy, vz - axisLen}, .normal = {0, 0, 1}});
                        selectedVertices.push_back({.position = {vx, vy, vz + axisLen}, .normal = {0, 0, 1}});
                        selectedEdges.push_back(axisBase + 0);
                        selectedEdges.push_back(axisBase + 1);
                        selectedEdges.push_back(axisBase + 2);
                        selectedEdges.push_back(axisBase + 3);
                        selectedEdges.push_back(axisBase + 4);
                        selectedEdges.push_back(axisBase + 5);
                    }
                } else {
                    auto& targetVerts = isSelected ? selectedVertices : combinedVertices;
                    auto& targetIndices = isSelected ? selectedIndices : combinedIndices;
                    auto& targetEdges = isSelected ? selectedEdges : combinedEdges;

                    uint32_t vOffset = static_cast<uint32_t>(targetVerts.size());
                    targetVerts.insert(targetVerts.end(), verts.begin(), verts.end());

                    for (uint32_t idx : indices) {
                        targetIndices.push_back(idx + vOffset);
                    }
                    for (uint32_t edgeIdx : edges) {
                        targetEdges.push_back(edgeIdx + vOffset);
                    }
                }
            }
        }

        std::shared_ptr<RHI::RHIBuffer> vb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> ib = nullptr;
        std::shared_ptr<RHI::RHIBuffer> edgeIb = nullptr;
        uint32_t triCount = 0;
        uint32_t edgeCount = 0;

        if (!combinedVertices.empty() && !combinedIndices.empty()) {
            RHI::RHIBufferDesc vbDesc{
                .sizeInBytes = combinedVertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Scene_Combined_VB"
            };
            RHI::RHIBufferDesc ibDesc{
                .sizeInBytes = combinedIndices.size() * sizeof(uint32_t),
                .usage = RHI::BufferUsage::Index,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Scene_Combined_IB"
            };
            vb = device->CreateBuffer(vbDesc, combinedVertices.data());
            ib = device->CreateBuffer(ibDesc, combinedIndices.data());
            triCount = static_cast<uint32_t>(combinedIndices.size());

            if (!combinedEdges.empty()) {
                RHI::RHIBufferDesc edgeDesc{
                    .sizeInBytes = combinedEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Scene_Combined_EdgeIB"
                };
                edgeIb = device->CreateBuffer(edgeDesc, combinedEdges.data());
                edgeCount = static_cast<uint32_t>(combinedEdges.size());
            }
        }

        // Selected 3D Model Buffers (for subtle surface highlight + outer edge highlight)
        std::shared_ptr<RHI::RHIBuffer> selVb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> selIb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> selEdgeIb = nullptr;
        uint32_t selTriCount = 0;
        uint32_t selEdgeCount = 0;

        if (!selectedVertices.empty()) {
            RHI::RHIBufferDesc selVbDesc{
                .sizeInBytes = selectedVertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Selected_Model_VB"
            };
            selVb = device->CreateBuffer(selVbDesc, selectedVertices.data());

            if (!selectedIndices.empty()) {
                RHI::RHIBufferDesc selIbDesc{
                    .sizeInBytes = selectedIndices.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Selected_Model_IB"
                };
                selIb = device->CreateBuffer(selIbDesc, selectedIndices.data());
                selTriCount = static_cast<uint32_t>(selectedIndices.size());
            }

            if (!selectedEdges.empty()) {
                RHI::RHIBufferDesc selEdgeDesc{
                    .sizeInBytes = selectedEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Selected_Model_EdgeIB"
                };
                selEdgeIb = device->CreateBuffer(selEdgeDesc, selectedEdges.data());
                selEdgeCount = static_cast<uint32_t>(selectedEdges.size());
            }
        }

        std::shared_ptr<RHI::RHIBuffer> sketchVb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> sketchIb = nullptr;
        std::shared_ptr<RHI::RHIBuffer> sketchEdgeIb = nullptr;
        uint32_t sketchTriCount = 0;
        uint32_t sketchEdgeCount = 0;

        if (!sketchVertices.empty()) {
            RHI::RHIBufferDesc svbDesc{
                .sizeInBytes = sketchVertices.size() * sizeof(CADVertex),
                .usage = RHI::BufferUsage::Vertex,
                .memory = RHI::MemoryUsage::CpuToGpu,
                .debugName = "Sketch_Transparent_VB"
            };
            sketchVb = device->CreateBuffer(svbDesc, sketchVertices.data());

            if (!sketchIndices.empty()) {
                RHI::RHIBufferDesc sibDesc{
                    .sizeInBytes = sketchIndices.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Sketch_Transparent_IB"
                };
                sketchIb = device->CreateBuffer(sibDesc, sketchIndices.data());
                sketchTriCount = static_cast<uint32_t>(sketchIndices.size());
            }

            if (!sketchEdges.empty()) {
                RHI::RHIBufferDesc sedgeDesc{
                    .sizeInBytes = sketchEdges.size() * sizeof(uint32_t),
                    .usage = RHI::BufferUsage::Index,
                    .memory = RHI::MemoryUsage::CpuToGpu,
                    .debugName = "Sketch_Outline_EdgeIB"
                };
                sketchEdgeIb = device->CreateBuffer(sedgeDesc, sketchEdges.data());
                sketchEdgeCount = static_cast<uint32_t>(sketchEdges.size());
            }
        }

        exchange.Publish(
            vb, 
            ib, 
            triCount, 
            s_sceneGen.fetch_add(1, std::memory_order_relaxed),
            edgeIb,
            edgeCount,
            sketchVb,
            sketchIb,
            sketchTriCount,
            sketchEdgeIb,
            sketchEdgeCount,
            selVb,
            selIb,
            selTriCount,
            selEdgeIb,
            selEdgeCount
        );
    }

    [[nodiscard]] size_t GetNodeCount() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_nodes.size();
    }

    [[nodiscard]] std::vector<std::shared_ptr<DAGNode>> GetNodes() const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        return m_nodes;
    }

    [[nodiscard]] std::shared_ptr<DAGNode> GetNode(uint64_t id) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        for (const auto& node : m_nodes) {
            if (node && node->GetId() == id) {
                return node;
            }
        }
        return nullptr;
    }

    void RemoveNode(uint64_t id) {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        std::shared_ptr<DAGNode> targetNode = nullptr;
        for (auto& node : m_nodes) {
            if (node && node->GetId() == id) {
                targetNode = node;
                // If this feature has a child sketch, promote the sketch to parentless
                node->DetachChildSketch();
                break;
            }
        }
        if (targetNode) {
            // Unconsume upstream parent features if this node was consuming them
            for (const auto& parent : targetNode->GetInputs()) {
                if (!parent) continue;
                bool stillConsumed = false;
                for (const auto& dep : parent->GetDependents()) {
                    if (dep && dep != targetNode && dep->GetId() != id) {
                        for (const auto& n : m_nodes) {
                            if (n && n == dep && !n->HasConflict()) {
                                stillConsumed = true;
                                break;
                            }
                        }
                    }
                    if (stillConsumed) break;
                }
                if (!stillConsumed) {
                    parent->SetConsumed(false);
                    parent->MarkDirty();
                }
            }

            // If the deleted node is currently selected, update selection to its parent or child
            if (m_selectedNodeId == id) {
                if (targetNode->GetParentFeature()) {
                    m_selectedNodeId = targetNode->GetParentFeature()->GetId();
                } else if (targetNode->GetChildSketch()) {
                    m_selectedNodeId = targetNode->GetChildSketch()->GetId();
                } else {
                    m_selectedNodeId = 0;
                }
            }

            targetNode->NotifyRemoval();
        }
        m_nodes.erase(std::remove_if(m_nodes.begin(), m_nodes.end(), [id](const auto& n) {
            return n->GetId() == id;
        }), m_nodes.end());
    }

    void ClearNodes() {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        m_nodes.clear();
    }

    /// Resolves any consumed/superseded solid feature to its ultimate active leaf feature (end-model state)
    [[nodiscard]] std::shared_ptr<DAGNode> GetActiveLeafForNode(const std::shared_ptr<DAGNode>& node) const {
        if (!node) return nullptr;
        if (!node->IsConsumed() || !node->IsSolidFeature()) return node;

        std::shared_ptr<DAGNode> curr = node;
        bool advanced = true;
        size_t safetyCounter = 0;
        while (advanced && safetyCounter++ < 100) {
            advanced = false;

            // 1. Follow internal dependent chain
            auto leaf = curr->GetActiveLeafFeature();
            if (leaf && leaf != curr && leaf->IsSolidFeature()) {
                curr = leaf;
                advanced = true;
                if (!curr->IsConsumed()) break;
                continue;
            }

            // 2. Scan graph nodes for downstream solid operations targeting curr
            std::shared_ptr<DAGNode> candidate = nullptr;
            for (const auto& n : m_nodes) {
                if (!n || n == curr || n->HasConflict() || !n->IsSolidFeature()) continue;
                if (n->GetTargetSolid() == curr) {
                    candidate = n;
                    if (!n->IsConsumed()) break;
                }
            }
            if (candidate && candidate != curr) {
                curr = candidate;
                advanced = true;
                if (!curr->IsConsumed()) break;
            }
        }
        return curr;
    }

    /// Casts a ray through all DAG nodes and returns the ID of the closest intersected node (0 if none hit)
    [[nodiscard]] uint64_t PickNode(
        const Geom::Point3D& rayOrigin,
        const Geom::Vector3D& rayDir,
        double* outDist = nullptr
    ) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        uint64_t closestNodeId = 0;
        double closestDist = std::numeric_limits<double>::infinity();

        for (const auto& node : m_nodes) {
            if (!node) continue;

            // Never pick consumed solid features (they are hidden/superseded by downstream operations)
            if (node->IsConsumed()) continue;

            // If it's a sketch node:
            // 1) Consumed sketches (driving profiles of features) are picked via the feature's 3D solid
            // 2) Standalone sketches (no parent feature or datum plane) are hidden when not selected
            bool isSketchNode = false;
            if (auto sketch = std::dynamic_pointer_cast<SketchNode>(node)) {
                if (sketch->IsConsumed()) continue;
                if (!sketch->HasParentFeature() && !sketch->HasCustomPlane() && sketch->GetId() != m_selectedNodeId) {
                    continue;
                }
                isSketchNode = true;
            }

            const auto& verts = node->GetCachedVertices();
            const auto& indices = node->GetCachedIndices();

            if (verts.empty() || indices.size() < 3) continue;

            const size_t triCount = indices.size() / 3;
            for (size_t i = 0; i < triCount; ++i) {
                uint32_t idx0 = indices[i * 3 + 0];
                uint32_t idx1 = indices[i * 3 + 1];
                uint32_t idx2 = indices[i * 3 + 2];

                if (idx0 >= verts.size() || idx1 >= verts.size() || idx2 >= verts.size()) {
                    continue;
                }

                Geom::Point3D v0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
                Geom::Point3D v1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
                Geom::Point3D v2(verts[idx2].position[0], verts[idx2].position[1], verts[idx2].position[2]);

                double t = 0.0;
                if (RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, t)) {
                    // Apply a slight depth priority bias for sketches drawn directly on faces
                    double effectiveT = isSketchNode ? (t - 0.01) : t;
                    if (effectiveT < closestDist) {
                        closestDist = effectiveT;
                        closestNodeId = node->GetId();
                        if (outDist) *outDist = t;
                    }
                }
            }
        }

        if (outDist && closestNodeId != 0) {
            *outDist = closestDist;
        }

        return closestNodeId;
    }

    /// Casts a ray specifically against a selected node to pick its sub-element (Vertex, Edge, or Face)
    [[nodiscard]] SubElementPickResult PickSubElement(
        const Geom::Point3D& rayOrigin,
        const Geom::Vector3D& rayDir,
        uint64_t targetNodeId,
        double vertexTolerance = 0.25,
        double edgeTolerance = 0.25,
        SelectionFilter filter = SelectionFilter::All
    ) const {
        std::lock_guard<std::mutex> lock(m_graphMutex);
        SubElementPickResult result;
        result.nodeId = targetNodeId;

        std::shared_ptr<DAGNode> targetNode = nullptr;
        for (const auto& node : m_nodes) {
            if (node && node->GetId() == targetNodeId) {
                targetNode = node;
                break;
            }
        }
        if (!targetNode) return result;

        if (targetNode->IsConsumed()) {
            auto leaf = GetActiveLeafForNode(targetNode);
            if (leaf) {
                targetNode = leaf;
                result.nodeId = leaf->GetId();
            }
        }

        const auto& verts = targetNode->GetCachedVertices();
        const auto& indices = targetNode->GetCachedIndices();
        if (verts.empty()) return result;

        // 0. Surface Occlusion & View-Facing Normal Filtering
        // Find front-most surface hit distance to prevent picking occluded back-facing vertices or edges
        const size_t triCount = indices.size() / 3;
        double closestFrontHitT = std::numeric_limits<double>::infinity();
        std::vector<bool> vertFacingView(verts.size(), false);
        std::vector<size_t> frontTriIndices;
        frontTriIndices.reserve(triCount);

        for (size_t i = 0; i < triCount; ++i) {
            uint32_t idx0 = indices[i * 3 + 0];
            uint32_t idx1 = indices[i * 3 + 1];
            uint32_t idx2 = indices[i * 3 + 2];
            if (idx0 >= verts.size() || idx1 >= verts.size() || idx2 >= verts.size()) continue;

            Geom::Point3D v0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
            Geom::Point3D v1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
            Geom::Point3D v2(verts[idx2].position[0], verts[idx2].position[1], verts[idx2].position[2]);

            Geom::Vector3D triNormal = (v1 - v0).cross(v2 - v0);
            Geom::Vector3D avgNormal(
                verts[idx0].normal[0] + verts[idx1].normal[0] + verts[idx2].normal[0],
                verts[idx0].normal[1] + verts[idx1].normal[1] + verts[idx2].normal[1],
                verts[idx0].normal[2] + verts[idx1].normal[2] + verts[idx2].normal[2]
            );
            if (avgNormal.length_squared() > 1e-4 && triNormal.dot(avgNormal) < 0.0) {
                triNormal = -triNormal;
            }

            bool isFacing = (triNormal.dot(rayDir) < -1e-6) || (avgNormal.length_squared() > 1e-4 && avgNormal.dot(rayDir) < -1e-6);
            if (isFacing) { // Strictly facing the camera
                frontTriIndices.push_back(i);
                vertFacingView[idx0] = true;
                vertFacingView[idx1] = true;
                vertFacingView[idx2] = true;

                double hitT = 0.0;
                if (RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, hitT)) {
                    if (hitT > 1e-4 && hitT < closestFrontHitT) {
                        closestFrontHitT = hitT;
                    }
                }
            }
        }

        // Map edges to whether at least one incident triangle faces the camera
        std::unordered_map<uint64_t, bool> edgeHasFrontFace;
        for (size_t t : frontTriIndices) {
            uint32_t i0 = indices[t * 3 + 0];
            uint32_t i1 = indices[t * 3 + 1];
            uint32_t i2 = indices[t * 3 + 2];
            auto addEdgeFront = [&](uint32_t a, uint32_t b) {
                uint64_t k = (static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
                edgeHasFrontFace[k] = true;
            };
            addEdgeFront(i0, i1);
            addEdgeFront(i1, i2);
            addEdgeFront(i2, i0);
        }

        // Ray occlusion test: check if point Q is occluded by any front-facing triangle
        auto isPointOccluded = [&](const Geom::Point3D& q) -> bool {
            Geom::Vector3D toQ = q - rayOrigin;
            double distToQ = toQ.length();
            if (distToQ < 1e-4) return false;
            Geom::Vector3D dirToQ = toQ / distToQ;

            for (size_t t : frontTriIndices) {
                uint32_t idx0 = indices[t * 3 + 0];
                uint32_t idx1 = indices[t * 3 + 1];
                uint32_t idx2 = indices[t * 3 + 2];
                Geom::Point3D v0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
                Geom::Point3D v1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
                Geom::Point3D v2(verts[idx2].position[0], verts[idx2].position[1], verts[idx2].position[2]);

                double hitT = 0.0;
                if (RayIntersectsTriangle(rayOrigin, dirToQ, v0, v1, v2, hitT)) {
                    if (hitT > 1e-4 && hitT < distToQ - 0.03) {
                        return true;
                    }
                }
            }
            return false;
        };

        // 1. Proximity check to vertices
        int bestVertexIdx = -1;
        double bestVertexDist = vertexTolerance;
        double bestVertexRayT = std::numeric_limits<double>::infinity();

        if (filter == SelectionFilter::All || filter == SelectionFilter::PointsOnly) {
            for (size_t i = 0; i < verts.size(); ++i) {
                // Must be facing the view
                if (!vertFacingView[i]) {
                    Geom::Vector3D vn(verts[i].normal[0], verts[i].normal[1], verts[i].normal[2]);
                    if (vn.dot(rayDir) > -0.05) continue; // back-facing
                }

                Geom::Point3D p(verts[i].position[0], verts[i].position[1], verts[i].position[2]);
                Geom::Vector3D toP = p - rayOrigin;
                double t = toP.dot(rayDir);
                if (t > 1e-4) {
                    if (closestFrontHitT < 1e9 && t > closestFrontHitT + 0.15) {
                        continue;
                    }
                    if (isPointOccluded(p)) {
                        continue;
                    }

                    Geom::Point3D rayPt = rayOrigin + rayDir * t;
                    double d = p.distance_to(rayPt);
                    if (d <= vertexTolerance && (d < bestVertexDist || (std::abs(d - bestVertexDist) < 0.02 && t < bestVertexRayT))) {
                        bestVertexDist = d;
                        bestVertexRayT = t;
                        bestVertexIdx = static_cast<int>(i);
                    }
                }
            }

            if (filter == SelectionFilter::PointsOnly && bestVertexIdx >= 0) {
                result.vertexIndex = bestVertexIdx;
                result.vertexPos = Geom::Point3D(verts[bestVertexIdx].position[0], verts[bestVertexIdx].position[1], verts[bestVertexIdx].position[2]);
                result.distance = bestVertexRayT;
                return result;
            }
        }

        // 2. Proximity check to feature edges (corner edges)
        int bestEdgeIdx = -1;
        double bestEdgeDist = edgeTolerance;
        double bestEdgeRayT = std::numeric_limits<double>::infinity();
        Geom::Point3D bestEdgeP0, bestEdgeP1;

        if (filter == SelectionFilter::All || filter == SelectionFilter::EdgesOnly) {
            const auto& edges = targetNode->GetCachedEdges();
            for (size_t e = 0; e + 1 < edges.size(); e += 2) {
                uint32_t i0 = edges[e];
                uint32_t i1 = edges[e + 1];
                if (i0 >= verts.size() || i1 >= verts.size()) continue;

                // Edge must border at least one front-facing triangle or have view-facing endpoints
                uint64_t eKey = (static_cast<uint64_t>(std::min(i0, i1)) << 32) | std::max(i0, i1);
                bool hasFront = (edgeHasFrontFace.find(eKey) != edgeHasFrontFace.end());
                if (!hasFront && !vertFacingView[i0] && !vertFacingView[i1]) {
                    continue; // Completely back-facing edge
                }

                Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
                Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
                Geom::Point3D midPt((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5, (p0.z + p1.z) * 0.5);

                Geom::Vector3D u = rayDir;
                Geom::Vector3D v = p1 - p0;
                Geom::Vector3D w = rayOrigin - p0;

                double b = u.dot(v);
                double c = v.dot(v);
                double d = u.dot(w);
                double e_dot = v.dot(w);

                double denom = c - b * b;
                double s = (denom > 1e-6) ? (e_dot - b * d) / denom : 0.5;
                s = std::clamp(s, 0.0, 1.0);

                double t = b * s - d;
                if (t > 1e-4) {
                    if (closestFrontHitT < 1e9 && t > closestFrontHitT + 0.15) {
                        continue;
                    }
                    if (isPointOccluded(midPt)) {
                        continue;
                    }

                    Geom::Point3D rayPt = rayOrigin + u * t;
                    Geom::Point3D segPt = p0 + v * s;
                    double dist = rayPt.distance_to(segPt);
                    if (dist <= bestEdgeDist && (dist < bestEdgeDist || (std::abs(dist - bestEdgeDist) < 0.02 && t < bestEdgeRayT))) {
                        bestEdgeDist = dist;
                        bestEdgeRayT = t;
                        bestEdgeIdx = static_cast<int>(e / 2);
                        bestEdgeP0 = p0;
                        bestEdgeP1 = p1;
                    }
                }
            }

            if (filter == SelectionFilter::EdgesOnly && bestEdgeIdx >= 0) {
                result.edgeIndex = bestEdgeIdx;
                result.edgeP0 = bestEdgeP0;
                result.edgeP1 = bestEdgeP1;
                result.distance = bestEdgeRayT;
                result.circularEdge = DetectCircularLoop(verts, edges, bestEdgeIdx);
                return result;
            }
        }

        // Resolve ambiguity between Vertex and Edge in SelectionFilter::All
        if (filter == SelectionFilter::All) {
            if (bestEdgeIdx >= 0 && bestVertexIdx >= 0) {
                const auto& edges = targetNode->GetCachedEdges();
                auto circ = DetectCircularLoop(verts, edges, bestEdgeIdx);

                // If clicking along a circular loop, prefer circle edge over vertex
                if (circ.isCircle) {
                    result.edgeIndex = bestEdgeIdx;
                    result.edgeP0 = bestEdgeP0;
                    result.edgeP1 = bestEdgeP1;
                    result.distance = bestEdgeRayT;
                    result.circularEdge = std::move(circ);
                    return result;
                }

                // If user clicked directly along an edge segment far from any vertex:
                if (bestEdgeDist < bestVertexDist - 0.12) {
                    result.edgeIndex = bestEdgeIdx;
                    result.edgeP0 = bestEdgeP0;
                    result.edgeP1 = bestEdgeP1;
                    result.distance = bestEdgeRayT;
                    result.circularEdge = std::move(circ);
                    return result;
                } else {
                    // Clicking near a corner vertex gives precedence to the vertex
                    result.vertexIndex = bestVertexIdx;
                    result.vertexPos = Geom::Point3D(verts[bestVertexIdx].position[0], verts[bestVertexIdx].position[1], verts[bestVertexIdx].position[2]);
                    result.distance = bestVertexRayT;
                    return result;
                }
            } else if (bestEdgeIdx >= 0) {
                const auto& edges = targetNode->GetCachedEdges();
                result.edgeIndex = bestEdgeIdx;
                result.edgeP0 = bestEdgeP0;
                result.edgeP1 = bestEdgeP1;
                result.distance = bestEdgeRayT;
                result.circularEdge = DetectCircularLoop(verts, edges, bestEdgeIdx);
                return result;
            } else if (bestVertexIdx >= 0) {
                result.vertexIndex = bestVertexIdx;
                result.vertexPos = Geom::Point3D(verts[bestVertexIdx].position[0], verts[bestVertexIdx].position[1], verts[bestVertexIdx].position[2]);
                result.distance = bestVertexRayT;
                return result;
            }
        }

        // 3. Intersected face check (if All or PlanesOnly)
        if (filter == SelectionFilter::All || filter == SelectionFilter::PlanesOnly) {
            double closestTriT = std::numeric_limits<double>::infinity();
            int hitTriIdx = -1;

            for (size_t i = 0; i < triCount; ++i) {
                uint32_t idx0 = indices[i * 3 + 0];
                uint32_t idx1 = indices[i * 3 + 1];
                uint32_t idx2 = indices[i * 3 + 2];
                if (idx0 >= verts.size() || idx1 >= verts.size() || idx2 >= verts.size()) continue;

                Geom::Point3D v0(verts[idx0].position[0], verts[idx0].position[1], verts[idx0].position[2]);
                Geom::Point3D v1(verts[idx1].position[0], verts[idx1].position[1], verts[idx1].position[2]);
                Geom::Point3D v2(verts[idx2].position[0], verts[idx2].position[1], verts[idx2].position[2]);

                double t = 0.0;
                if (RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, t)) {
                    if (t < closestTriT) {
                        closestTriT = t;
                        hitTriIdx = static_cast<int>(i);
                    }
                }
            }

            if (hitTriIdx >= 0) {
                int faceIdx = targetNode->GetFaceIndexForTriangle(static_cast<size_t>(hitTriIdx));
                FaceInfo face;
                if (faceIdx >= 0 && targetNode->GetFace(faceIdx, face)) {
                    result.faceIndex = faceIdx;
                    result.facePlane = face.plane;
                } else {
                    result.faceIndex = 0;
                    uint32_t i0 = indices[hitTriIdx * 3 + 0];
                    uint32_t i1 = indices[hitTriIdx * 3 + 1];
                    uint32_t i2 = indices[hitTriIdx * 3 + 2];
                    Geom::Point3D v0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
                    Geom::Point3D v1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
                    Geom::Point3D v2(verts[i2].position[0], verts[i2].position[1], verts[i2].position[2]);
                    result.facePlane = Geom::Plane(v0, v1, v2);
                }
                result.distance = closestTriT;
                return result;
            }
        }

        return result;
    }

private:
    std::shared_ptr<ThreadPool> m_threadPool;
    std::vector<std::shared_ptr<DAGNode>> m_nodes;
    uint64_t m_selectedNodeId{0};
    int m_selectedFaceIndex{-1};
    int m_selectedVertexIndex{-1};
    int m_selectedEdgeIndex{-1};
    CircularEdgeInfo m_selectedCircle;
    std::vector<int> m_selectedEdgeIndices;
    std::vector<int> m_selectedFaceIndices;
    std::vector<int> m_selectedVertexIndices;
    mutable std::mutex m_graphMutex;
    static inline std::atomic<uint64_t> s_sceneGen{1};
};

} // namespace Tessera

