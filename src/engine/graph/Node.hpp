#pragma once

#include "GeometryTypes.hpp"
#include "ExtrusionCommon.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include "engine/geom/Plane.hpp"
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <cstdint>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace Tessera {

/// Sub-element representation of a topological / tessellated face on a 3D CAD model
struct FaceInfo {
    int faceIndex{0};
    std::vector<uint32_t> triangleIndices; // 3 * K indices into node's cached vertices
    std::vector<uint32_t> perimeterEdges;  // Line pairs of vertex indices (e0, e1, e2, e3...)
    Geom::Plane plane;                     // Coordinate frame & analytical plane
    Geom::Point3D center{0.0, 0.0, 0.0};
    Geom::Vector3D normal{0.0, 1.0, 0.0};
    bool isPlanar{true};
};

class DAGNode : public std::enable_shared_from_this<DAGNode> {
public:
    explicit DAGNode(std::string name)
        : m_name(std::move(name)),
          m_id(s_nextId.fetch_add(1, std::memory_order_relaxed)),
          m_isDirty(true),
          m_generation(1) {}

    virtual ~DAGNode() = default;

    [[nodiscard]] uint64_t GetId() const noexcept { return m_id; }
    [[nodiscard]] const std::string& GetName() const noexcept { return m_name; }
    [[nodiscard]] bool IsDirty() const noexcept { return m_isDirty.load(std::memory_order_acquire); }
    [[nodiscard]] uint64_t GetGeneration() const noexcept { return m_generation.load(std::memory_order_acquire); }

    [[nodiscard]] const std::vector<CADVertex>& GetCachedVertices() const noexcept { return m_cachedVertices; }
    [[nodiscard]] const std::vector<uint32_t>& GetCachedIndices() const noexcept { return m_cachedIndices; }
    [[nodiscard]] const std::vector<uint32_t>& GetCachedEdges() const noexcept { return m_cachedEdges; }

    void SetPosition(float x, float y, float z) {
        m_position[0] = x;
        m_position[1] = y;
        m_position[2] = z;
        MarkDirty();
    }
    [[nodiscard]] const float* GetPosition() const noexcept { return m_position; }

    /// Child sketch association for feature nodes (Extrude, Revolve)
    [[nodiscard]] virtual std::shared_ptr<DAGNode> GetParentFeature() const noexcept { return nullptr; }
    [[nodiscard]] virtual std::shared_ptr<DAGNode> GetChildSketch() const noexcept { return nullptr; }
    [[nodiscard]] virtual bool HasParentFeature() const noexcept { return false; }
    virtual void DetachChildSketch() noexcept {}

    /// Conflict status for associative parametric references
    [[nodiscard]] virtual bool HasConflict() const noexcept { return false; }
    [[nodiscard]] virtual std::string GetConflictMessage() const noexcept { return ""; }

    /// Consumed status for features modified by downstream operations (e.g. Fillet, Chamfer, Extrude)
    [[nodiscard]] virtual bool IsConsumed() const noexcept { return m_isConsumed; }
    virtual void SetConsumed(bool consumed) noexcept { m_isConsumed = consumed; }

    /// Extrusion profile & coordinate metadata query for chaining 3D solid operations (Fillet, Chamfer, Boss)
    virtual bool IsExtrusionBased(ExtrusionData& out) const { (void)out; return false; }

    [[nodiscard]] const std::vector<std::shared_ptr<DAGNode>>& GetInputs() const noexcept { return m_inputs; }

    [[nodiscard]] std::vector<std::shared_ptr<DAGNode>> GetDependents() const {
        std::vector<std::shared_ptr<DAGNode>> result;
        for (const auto& depWeak : m_dependents) {
            if (auto dep = depWeak.lock()) {
                result.push_back(dep);
            }
        }
        return result;
    }

    /// Recursively traces downstream solid feature dependencies to find the latest active leaf feature
    [[nodiscard]] std::shared_ptr<DAGNode> GetActiveLeafFeature() {
        std::shared_ptr<DAGNode> curr = shared_from_this();
        bool advanced = true;
        while (advanced) {
            advanced = false;
            std::shared_ptr<DAGNode> bestNext = nullptr;
            for (const auto& depWeak : curr->m_dependents) {
                if (auto dep = depWeak.lock()) {
                    if (dep->HasConflict()) continue;
                    bestNext = dep;
                    if (!dep->IsConsumed()) {
                        break;
                    }
                }
            }
            if (bestNext && bestNext != curr) {
                curr = bestNext;
                advanced = true;
            }
        }
        return curr;
    }

    /// Connects this node to depend on an upstream parent
    void AddInput(std::shared_ptr<DAGNode> parent) {
        if (!parent) return;
        m_inputs.push_back(parent);
        parent->m_dependents.push_back(weak_from_this());
    }

    /// Notifies all downstream dependent nodes that this node is being removed from the graph
    virtual void NotifyRemoval() {
        for (auto& depWeak : m_dependents) {
            if (auto dep = depWeak.lock()) {
                dep->OnParentRemoved(m_id);
            }
        }
        m_dependents.clear();
    }

    /// Called on a dependent node when one of its upstream parents is removed
    virtual void OnParentRemoved(uint64_t parentId) {
        m_inputs.erase(std::remove_if(m_inputs.begin(), m_inputs.end(), [parentId](const auto& in) {
            return in && in->GetId() == parentId;
        }), m_inputs.end());
        MarkDirty();
    }

    /// Flags this node as dirty and propagates downstream through the DAG
    virtual void MarkDirty() {
        m_isDirty.store(true, std::memory_order_release);
        m_generation.fetch_add(1, std::memory_order_release);
        m_facesDirty = true;

        for (auto it = m_dependents.begin(); it != m_dependents.end();) {
            if (auto dependent = it->lock()) {
                dependent->MarkDirty();
                ++it;
            } else {
                it = m_dependents.erase(it);
            }
        }
    }

    /// Clear dirty flag after evaluation completes
    void ClearDirty() noexcept {
        m_isDirty.store(false, std::memory_order_release);
    }

    /// Pure virtual evaluation invoked asynchronously on the Worker Thread Pool
    virtual void Evaluate(RHI::RHIDevice* device, AtomicMeshExchange& exchange) = 0;

    // =========================================================================
    // Sub-Element Face & Vertex Querying
    // =========================================================================

    virtual const std::vector<FaceInfo>& GetFaces() {
        if (m_facesDirty) {
            ComputeFaces();
            m_facesDirty = false;
        }
        return m_cachedFaces;
    }

    virtual bool GetFace(int faceIndex, FaceInfo& outFace) {
        const auto& faces = GetFaces();
        if (faceIndex >= 0 && static_cast<size_t>(faceIndex) < faces.size()) {
            outFace = faces[faceIndex];
            return true;
        }
        return false;
    }

    virtual int GetFaceIndexForTriangle(size_t triangleIndex) {
        const auto& faces = GetFaces();
        uint32_t triStart = static_cast<uint32_t>(triangleIndex * 3);
        for (const auto& face : faces) {
            for (size_t i = 0; i < face.triangleIndices.size(); i += 3) {
                // Check if the first index of the triangle matches
                if (triStart < m_cachedIndices.size() && 
                    face.triangleIndices[i] == m_cachedIndices[triStart] &&
                    face.triangleIndices[i + 1] == m_cachedIndices[triStart + 1] &&
                    face.triangleIndices[i + 2] == m_cachedIndices[triStart + 2]) {
                    return face.faceIndex;
                }
            }
        }
        return -1;
    }

    virtual bool GetVertexPosition(int vertexIndex, Geom::Point3D& outPos) const {
        if (vertexIndex >= 0 && static_cast<size_t>(vertexIndex) < m_cachedVertices.size()) {
            const auto& v = m_cachedVertices[vertexIndex];
            outPos = Geom::Point3D(v.position[0], v.position[1], v.position[2]);
            return true;
        }
        return false;
    }

protected:
    virtual void ComputeFaces() {
        m_cachedFaces.clear();
        if (m_cachedVertices.empty() || m_cachedIndices.size() < 3) return;

        const size_t triCount = m_cachedIndices.size() / 3;
        std::vector<bool> visited(triCount, false);

        struct TriPlane {
            Geom::Vector3D normal;
            double d{0.0};
            bool valid{false};
        };

        std::vector<TriPlane> triPlanes(triCount);
        for (size_t i = 0; i < triCount; ++i) {
            uint32_t i0 = m_cachedIndices[i * 3 + 0];
            uint32_t i1 = m_cachedIndices[i * 3 + 1];
            uint32_t i2 = m_cachedIndices[i * 3 + 2];
            if (i0 >= m_cachedVertices.size() || i1 >= m_cachedVertices.size() || i2 >= m_cachedVertices.size()) {
                continue;
            }

            Geom::Point3D p0(m_cachedVertices[i0].position[0], m_cachedVertices[i0].position[1], m_cachedVertices[i0].position[2]);
            Geom::Point3D p1(m_cachedVertices[i1].position[0], m_cachedVertices[i1].position[1], m_cachedVertices[i1].position[2]);
            Geom::Point3D p2(m_cachedVertices[i2].position[0], m_cachedVertices[i2].position[1], m_cachedVertices[i2].position[2]);

            Geom::Vector3D n = (p1 - p0).cross(p2 - p0);
            double len = n.length();
            if (len > 1e-6) {
                n = n / len;
                Geom::Vector3D vn(m_cachedVertices[i0].normal[0], m_cachedVertices[i0].normal[1], m_cachedVertices[i0].normal[2]);
                if (vn.length_squared() > 1e-4 && n.dot(vn) < 0.0) {
                    n = -n;
                }
                triPlanes[i].normal = n;
                triPlanes[i].d = -(triPlanes[i].normal.x * p0.x + triPlanes[i].normal.y * p0.y + triPlanes[i].normal.z * p0.z);
                triPlanes[i].valid = true;
            }
        }

        int faceIdx = 0;
        for (size_t i = 0; i < triCount; ++i) {
            if (visited[i] || !triPlanes[i].valid) continue;

            FaceInfo fi;
            fi.faceIndex = faceIdx++;
            fi.normal = triPlanes[i].normal;
            fi.isPlanar = true;

            for (size_t j = i; j < triCount; ++j) {
                if (visited[j] || !triPlanes[j].valid) continue;
                double dotN = triPlanes[i].normal.dot(triPlanes[j].normal);
                double diffD = std::abs(triPlanes[i].d - triPlanes[j].d);
                if (dotN > 0.999 && diffD < 0.01) {
                    visited[j] = true;
                    fi.triangleIndices.push_back(m_cachedIndices[j * 3 + 0]);
                    fi.triangleIndices.push_back(m_cachedIndices[j * 3 + 1]);
                    fi.triangleIndices.push_back(m_cachedIndices[j * 3 + 2]);
                }
            }

            // Fallback for curved triangles not grouped with plane: take individual triangle or curved patch
            if (fi.triangleIndices.empty()) {
                visited[i] = true;
                fi.triangleIndices.push_back(m_cachedIndices[i * 3 + 0]);
                fi.triangleIndices.push_back(m_cachedIndices[i * 3 + 1]);
                fi.triangleIndices.push_back(m_cachedIndices[i * 3 + 2]);
                fi.isPlanar = false;
            }

            // Compute centroid
            Geom::Vector3D sumPos(0, 0, 0);
            size_t vCount = 0;
            for (uint32_t vi : fi.triangleIndices) {
                sumPos = sumPos + Geom::Vector3D(m_cachedVertices[vi].position[0], m_cachedVertices[vi].position[1], m_cachedVertices[vi].position[2]);
                vCount++;
            }
            if (vCount > 0) {
                fi.center = Geom::Point3D(sumPos.x / vCount, sumPos.y / vCount, sumPos.z / vCount);
            }

            fi.plane = Geom::Plane(fi.center, fi.normal);

            // Perimeter edges: edges that appear with multiplicity == 1
            struct EdgeKey {
                uint32_t a, b;
                bool operator==(const EdgeKey& o) const noexcept {
                    return (a == o.a && b == o.b) || (a == o.b && b == o.a);
                }
            };
            struct EdgeHash {
                size_t operator()(const EdgeKey& k) const noexcept {
                    uint32_t lo = std::min(k.a, k.b);
                    uint32_t hi = std::max(k.a, k.b);
                    return (static_cast<size_t>(lo) << 32) ^ hi;
                }
            };
            std::unordered_map<EdgeKey, int, EdgeHash> edgeMap;
            for (size_t k = 0; k < fi.triangleIndices.size(); k += 3) {
                uint32_t v0 = fi.triangleIndices[k + 0];
                uint32_t v1 = fi.triangleIndices[k + 1];
                uint32_t v2 = fi.triangleIndices[k + 2];
                edgeMap[{v0, v1}]++;
                edgeMap[{v1, v2}]++;
                edgeMap[{v2, v0}]++;
            }
            for (const auto& [ek, count] : edgeMap) {
                if (count == 1) {
                    fi.perimeterEdges.push_back(ek.a);
                    fi.perimeterEdges.push_back(ek.b);
                }
            }

            m_cachedFaces.push_back(std::move(fi));
        }
    }

    std::string m_name;
    uint64_t m_id;
    std::atomic<bool> m_isDirty;
    std::atomic<uint64_t> m_generation;

    float m_position[3]{0.0f, 0.0f, 0.0f};
    std::vector<CADVertex> m_cachedVertices;
    std::vector<uint32_t> m_cachedIndices;
    std::vector<uint32_t> m_cachedEdges;

    std::vector<FaceInfo> m_cachedFaces;
    bool m_facesDirty{true};
    bool m_isConsumed{false};

    std::vector<std::shared_ptr<DAGNode>> m_inputs;
    std::vector<std::weak_ptr<DAGNode>> m_dependents;

private:
    static inline std::atomic<uint64_t> s_nextId{1};
};

inline bool ExtractExtrusionData(const std::shared_ptr<DAGNode>& node, ExtrusionData& out) {
    if (!node) return false;
    auto leaf = node->GetActiveLeafFeature();
    if (leaf && leaf != node && leaf->IsExtrusionBased(out)) {
        return true;
    }
    if (node->IsExtrusionBased(out)) return true;
    return false;
}

} // namespace Tessera


