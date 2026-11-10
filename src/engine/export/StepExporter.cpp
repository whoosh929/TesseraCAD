#include "StepExporter.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <filesystem>

namespace Tessera::Export {

namespace {

// Format floating point numbers to ISO 10303-21 standard clear text representation
std::string FormatReal(double val) {
    if (std::abs(val) < 1e-12) val = 0.0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", val);
    std::string s(buf);
    if (s.find('.') == std::string::npos && s.find('e') == std::string::npos && s.find('E') == std::string::npos) {
        s += ".";
    }
    return s;
}

std::string GetCurrentTimestampIso() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
    return std::string(buf);
}

// Spatial quantization key for point deduplication (1 micrometer resolution)
struct QuantizedPoint {
    int64_t x{0};
    int64_t y{0};
    int64_t z{0};

    bool operator==(const QuantizedPoint& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct QuantizedPointHash {
    size_t operator()(const QuantizedPoint& p) const noexcept {
        size_t h1 = std::hash<int64_t>{}(p.x);
        size_t h2 = std::hash<int64_t>{}(p.y);
        size_t h3 = std::hash<int64_t>{}(p.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] double length_sq() const noexcept { return x * x + y * y + z * z; }
    [[nodiscard]] double length() const noexcept { return std::sqrt(length_sq()); }

    [[nodiscard]] Vec3 normalized() const noexcept {
        double len = length();
        if (len < 1e-12) return {0.0, 0.0, 1.0};
        return {x / len, y / len, z / len};
    }

    [[nodiscard]] double dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }

    [[nodiscard]] Vec3 cross(const Vec3& o) const noexcept {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const noexcept { return {x * s, y * s, z * s}; }
};

struct EdgeKey {
    uint32_t v0{0};
    uint32_t v1{0};

    bool operator==(const EdgeKey& o) const noexcept {
        return v0 == o.v0 && v1 == o.v1;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& k) const noexcept {
        return (static_cast<size_t>(k.v0) << 32) | static_cast<size_t>(k.v1);
    }
};

// Builder for a single Solid's STEP entities
struct SolidGeometryData {
    std::string solidName;
    std::vector<Vec3> uniquePoints;
    std::vector<uint32_t> pointEntityIds;
    std::vector<uint32_t> vertexEntityIds;

    struct StepEdge {
        uint32_t vMinIdx{0};
        uint32_t vMaxIdx{0};
        uint32_t edgeCurveId{0};
    };
    std::vector<StepEdge> uniqueEdges;

    struct StepFace {
        uint32_t faceId{0};
        uint32_t orientedEdgeIds[3]{0, 0, 0};
        uint32_t p0Idx{0};
        Vec3 normal;
        Vec3 refDir;
    };
    std::vector<StepFace> faces;
    uint32_t shellId{0};
    uint32_t solidBrepId{0};
};

} // namespace

StepExportResult StepExporter::ExportSolid(
    const std::string& filepath,
    const std::shared_ptr<DAGNode>& solidNode,
    const StepExportOptions& options
) {
    StepExportResult res;
    res.filePath = filepath;

    if (!solidNode) {
        res.errorMessage = "Null solid node provided";
        return res;
    }

    auto target = solidNode->IsConsumed() ? solidNode->GetActiveLeafFeature() : solidNode;
    if (!target) {
        res.errorMessage = "No active leaf solid found for feature";
        return res;
    }

    const auto& verts = target->GetCachedVertices();
    const auto& inds = target->GetCachedIndices();

    if (verts.empty() || inds.empty()) {
        res.errorMessage = "Solid feature has no geometry (empty mesh)";
        return res;
    }

    return ExportMesh(filepath, target->GetName(), verts, inds, options);
}

StepExportResult StepExporter::ExportModel(
    const std::string& filepath,
    const ParametricGraph& graph,
    const StepExportOptions& options
) {
    StepExportResult res;
    try {
        res.filePath = std::filesystem::absolute(filepath).string();
    } catch (...) {
        res.filePath = filepath;
    }

    auto allNodes = graph.GetNodes();
    std::vector<std::pair<std::string, const DAGNode*>> solidList;

    for (const auto& node : allNodes) {
        if (!node) continue;
        if (!node->IsSolidFeature()) continue;
        if (node->IsConsumed()) continue;
        if (node->HasConflict()) continue;
        if (node->GetCachedVertices().empty() || node->GetCachedIndices().empty()) continue;

        solidList.push_back({node->GetName(), node.get()});
    }

    if (solidList.empty()) {
        res.errorMessage = "No active 3D solid models found in scene to export";
        return res;
    }

    std::string stepContent = GenerateStepString(solidList, options);
    if (stepContent.empty()) {
        res.errorMessage = "Failed to generate STEP representation for model";
        return res;
    }

    // Ensure output directory exists
    try {
        std::filesystem::path p(filepath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (...) {}

    std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
    if (!ofs.is_open()) {
        res.errorMessage = "Failed to open output file: " + filepath;
        return res;
    }

    ofs.write(stepContent.data(), stepContent.size());
    ofs.close();

    res.success = true;
    res.solidCount = solidList.size();
    res.fileSizeBytes = stepContent.size();

    for (const auto& s : solidList) {
        res.faceCount += s.second->GetCachedIndices().size() / 3;
        res.vertexCount += s.second->GetCachedVertices().size();
        res.edgeCount += s.second->GetCachedEdges().size() / 2;
    }

    return res;
}

StepExportResult StepExporter::ExportMesh(
    const std::string& filepath,
    const std::string& solidName,
    const std::vector<CADVertex>& vertices,
    const std::vector<uint32_t>& indices,
    const StepExportOptions& options
) {
    StepExportResult res;
    try {
        res.filePath = std::filesystem::absolute(filepath).string();
    } catch (...) {
        res.filePath = filepath;
    }

    if (vertices.empty() || indices.size() < 3) {
        res.errorMessage = "Mesh has no geometry (vertices or triangles empty)";
        return res;
    }

    std::string stepContent = GenerateStepStringFromMesh(solidName, vertices, indices, options);
    if (stepContent.empty()) {
        res.errorMessage = "Failed to generate STEP content from mesh";
        return res;
    }

    try {
        std::filesystem::path p(filepath);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (...) {}

    std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
    if (!ofs.is_open()) {
        res.errorMessage = "Failed to open output file: " + filepath;
        return res;
    }

    ofs.write(stepContent.data(), stepContent.size());
    ofs.close();

    res.success = true;
    res.solidCount = 1;
    res.faceCount = indices.size() / 3;
    res.vertexCount = vertices.size();
    res.edgeCount = (indices.size() / 3) * 3 / 2;
    res.fileSizeBytes = stepContent.size();
    return res;
}

std::string StepExporter::GenerateStepStringFromMesh(
    const std::string& solidName,
    const std::vector<CADVertex>& vertices,
    const std::vector<uint32_t>& indices,
    const StepExportOptions& options
) {
    // Wrap in dummy node-like structure for uniform generation
    class DummyNode : public DAGNode {
    public:
        DummyNode(std::string name, const std::vector<CADVertex>& v, const std::vector<uint32_t>& i)
            : DAGNode(std::move(name)) {
            m_cachedVertices = v;
            m_cachedIndices = i;
        }
        void Evaluate(RHI::RHIDevice*, AtomicMeshExchange&) override {}
    };

    auto dummy = std::make_unique<DummyNode>(solidName, vertices, indices);
    std::vector<std::pair<std::string, const DAGNode*>> solids = {{solidName, dummy.get()}};
    return GenerateStepString(solids, options);
}

std::string StepExporter::GenerateStepString(
    const std::vector<std::pair<std::string, const DAGNode*>>& solids,
    const StepExportOptions& options
) {
    if (solids.empty()) return "";

    std::stringstream ss;
    uint32_t nextId = 1;

    // -------------------------------------------------------------------------
    // 1. STEP ISO 10303-21 Header Section
    // -------------------------------------------------------------------------
    ss << "ISO-10303-21;\n";
    ss << "HEADER;\n";
    ss << "FILE_DESCRIPTION(('" << options.description << "'),'2;1');\n";
    ss << "FILE_NAME('" << options.modelName << ".stp','"
       << GetCurrentTimestampIso() << "',('"
       << options.author << "'),('"
       << options.organization << "'),'TesseraCAD Engine 0.1.0','TesseraCAD','');\n";
    ss << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n";
    ss << "ENDSEC;\n";
    ss << "DATA;\n";

    // -------------------------------------------------------------------------
    // 2. Global Context, Product Definition & Units
    // -------------------------------------------------------------------------
    uint32_t appContextId = nextId++;
    ss << "#" << appContextId << " = APPLICATION_CONTEXT('core data for automotive mechanical design processes');\n";

    uint32_t appProtoDefId = nextId++;
    ss << "#" << appProtoDefId << " = APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#" << appContextId << ");\n";

    uint32_t prodContextId = nextId++;
    ss << "#" << prodContextId << " = PRODUCT_CONTEXT('',#" << appContextId << ",'mechanical');\n";

    uint32_t productId = nextId++;
    ss << "#" << productId << " = PRODUCT('" << options.modelName << "','" << options.modelName << "','',(#" << prodContextId << "));\n";

    uint32_t prodDefFormId = nextId++;
    ss << "#" << prodDefFormId << " = PRODUCT_DEFINITION_FORMATION('','',#" << productId << ");\n";

    uint32_t prodDefId = nextId++;
    ss << "#" << prodDefId << " = PRODUCT_DEFINITION('design','',#" << prodDefFormId << ",#" << prodContextId << ");\n";

    uint32_t prodDefShapeId = nextId++;
    ss << "#" << prodDefShapeId << " = PRODUCT_DEFINITION_SHAPE('','',#" << prodDefId << ");\n";

    // SI Units (Length: Metre, Angle: Radian, Solid Angle: Steradian)
    uint32_t lengthUnitId = nextId++;
    ss << "#" << lengthUnitId << " = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );\n";

    uint32_t angleUnitId = nextId++;
    ss << "#" << angleUnitId << " = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n";

    uint32_t solidAngleUnitId = nextId++;
    ss << "#" << solidAngleUnitId << " = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n";

    uint32_t uncertaintyId = nextId++;
    ss << "#" << uncertaintyId << " = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#"
       << lengthUnitId << ",'closure','Maximum model space distance between geometric elements at their paths and surfaces');\n";

    uint32_t geomContextId = nextId++;
    ss << "#" << geomContextId << " = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#"
       << uncertaintyId << ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#"
       << lengthUnitId << ",#" << angleUnitId << ",#" << solidAngleUnitId << ")) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') );\n";

    // World Axis Placement 3D at Origin (0, 0, 0)
    uint32_t originPointId = nextId++;
    ss << "#" << originPointId << " = CARTESIAN_POINT('',(" << FormatReal(0.0) << "," << FormatReal(0.0) << "," << FormatReal(0.0) << "));\n";

    uint32_t zAxisDirId = nextId++;
    ss << "#" << zAxisDirId << " = DIRECTION('',(" << FormatReal(0.0) << "," << FormatReal(0.0) << "," << FormatReal(1.0) << "));\n";

    uint32_t xAxisDirId = nextId++;
    ss << "#" << xAxisDirId << " = DIRECTION('',(" << FormatReal(1.0) << "," << FormatReal(0.0) << "," << FormatReal(0.0) << "));\n";

    uint32_t worldPlacementId = nextId++;
    ss << "#" << worldPlacementId << " = AXIS2_PLACEMENT_3D('',#" << originPointId << ",#" << zAxisDirId << ",#" << xAxisDirId << ");\n";

    // -------------------------------------------------------------------------
    // 3. Process Each Solid and Generate Watertight B-Rep Topology
    // -------------------------------------------------------------------------
    std::vector<uint32_t> manifoldSolidBrepIds;

    for (const auto& solidPair : solids) {
        const std::string& solidName = solidPair.first;
        const DAGNode* node = solidPair.second;
        if (!node) continue;

        const auto& verts = node->GetCachedVertices();
        const auto& inds = node->GetCachedIndices();
        if (verts.empty() || inds.size() < 3) continue;

        SolidGeometryData geom;
        geom.solidName = solidName;

        // A. Deduplicate points
        std::unordered_map<QuantizedPoint, uint32_t, QuantizedPointHash> ptMap;
        std::vector<uint32_t> meshToUniquePt(verts.size());

        for (size_t i = 0; i < verts.size(); ++i) {
            double x = verts[i].position[0];
            double y = verts[i].position[1];
            double z = verts[i].position[2];

            QuantizedPoint qp{
                static_cast<int64_t>(std::llround(x * 1e6)),
                static_cast<int64_t>(std::llround(y * 1e6)),
                static_cast<int64_t>(std::llround(z * 1e6))
            };

            auto it = ptMap.find(qp);
            if (it == ptMap.end()) {
                uint32_t uIdx = static_cast<uint32_t>(geom.uniquePoints.size());
                ptMap[qp] = uIdx;
                meshToUniquePt[i] = uIdx;
                geom.uniquePoints.push_back({x, y, z});
            } else {
                meshToUniquePt[i] = it->second;
            }
        }

        // Emit CARTESIAN_POINT and VERTEX_POINT entities
        geom.pointEntityIds.resize(geom.uniquePoints.size());
        geom.vertexEntityIds.resize(geom.uniquePoints.size());
        for (size_t i = 0; i < geom.uniquePoints.size(); ++i) {
            uint32_t pId = nextId++;
            geom.pointEntityIds[i] = pId;
            ss << "#" << pId << " = CARTESIAN_POINT('',("
               << FormatReal(geom.uniquePoints[i].x) << ","
               << FormatReal(geom.uniquePoints[i].y) << ","
               << FormatReal(geom.uniquePoints[i].z) << "));\n";

            uint32_t vId = nextId++;
            geom.vertexEntityIds[i] = vId;
            ss << "#" << vId << " = VERTEX_POINT('',#" << pId << ");\n";
        }

        // B. Deduplicate edges
        std::unordered_map<EdgeKey, uint32_t, EdgeKeyHash> edgeMap;

        auto GetOrCreateEdgeCurve = [&](uint32_t vA, uint32_t vB) -> std::pair<uint32_t, bool> {
            bool forward = (vA < vB);
            uint32_t vMin = forward ? vA : vB;
            uint32_t vMax = forward ? vB : vA;
            EdgeKey key{vMin, vMax};

            auto it = edgeMap.find(key);
            if (it != edgeMap.end()) {
                return {geom.uniqueEdges[it->second].edgeCurveId, forward};
            }

            // Create LINE geometry along the edge
            Vec3 pMin = geom.uniquePoints[vMin];
            Vec3 pMax = geom.uniquePoints[vMax];
            Vec3 delta = pMax - pMin;
            double len = delta.length();
            Vec3 dir = delta.normalized();

            uint32_t dirId = nextId++;
            ss << "#" << dirId << " = DIRECTION('',("
               << FormatReal(dir.x) << "," << FormatReal(dir.y) << "," << FormatReal(dir.z) << "));\n";

            uint32_t vecId = nextId++;
            ss << "#" << vecId << " = VECTOR('',#" << dirId << "," << FormatReal((len > 1e-9) ? len : 1.0) << ");\n";

            uint32_t lineId = nextId++;
            ss << "#" << lineId << " = LINE('',#" << geom.pointEntityIds[vMin] << ",#" << vecId << ");\n";

            uint32_t edgeCurveId = nextId++;
            ss << "#" << edgeCurveId << " = EDGE_CURVE('',#"
               << geom.vertexEntityIds[vMin] << ",#"
               << geom.vertexEntityIds[vMax] << ",#"
               << lineId << ",.T.);\n";

            uint32_t eIdx = static_cast<uint32_t>(geom.uniqueEdges.size());
            geom.uniqueEdges.push_back({vMin, vMax, edgeCurveId});
            edgeMap[key] = eIdx;

            return {edgeCurveId, forward};
        };

        // C. Process each triangle as an ADVANCED_FACE
        std::vector<uint32_t> faceIds;

        for (size_t t = 0; t + 2 < inds.size(); t += 3) {
            uint32_t origI0 = inds[t + 0];
            uint32_t origI1 = inds[t + 1];
            uint32_t origI2 = inds[t + 2];

            if (origI0 >= verts.size() || origI1 >= verts.size() || origI2 >= verts.size()) continue;

            uint32_t u0 = meshToUniquePt[origI0];
            uint32_t u1 = meshToUniquePt[origI1];
            uint32_t u2 = meshToUniquePt[origI2];

            // Skip degenerate triangle
            if (u0 == u1 || u1 == u2 || u2 == u0) continue;

            Vec3 p0 = geom.uniquePoints[u0];
            Vec3 p1 = geom.uniquePoints[u1];
            Vec3 p2 = geom.uniquePoints[u2];

            Vec3 e1 = p1 - p0;
            Vec3 e2 = p2 - p0;
            Vec3 normal = e1.cross(e2);
            double nLen = normal.length();
            if (nLen < 1e-12) continue; // Collinear vertices
            normal = normal * (1.0 / nLen);

            // Check consistency with mesh vertex normal
            Vec3 vNorm{verts[origI0].normal[0], verts[origI0].normal[1], verts[origI0].normal[2]};
            if (vNorm.length_sq() > 0.01 && normal.dot(vNorm) < 0.0) {
                // Reverse winding to ensure outward normal
                std::swap(u1, u2);
                std::swap(p1, p2);
                normal = normal * -1.0;
                e1 = p1 - p0;
            }

            // Reference direction in plane (orthogonal to normal)
            Vec3 refDir = e1.normalized();

            // Create 3 ORIENTED_EDGEs for the perimeter loop
            auto [edge01, fwd01] = GetOrCreateEdgeCurve(u0, u1);
            auto [edge12, fwd12] = GetOrCreateEdgeCurve(u1, u2);
            auto [edge20, fwd20] = GetOrCreateEdgeCurve(u2, u0);

            uint32_t oe01 = nextId++;
            ss << "#" << oe01 << " = ORIENTED_EDGE('',*,*,#" << edge01 << "," << (fwd01 ? ".T." : ".F.") << ");\n";

            uint32_t oe12 = nextId++;
            ss << "#" << oe12 << " = ORIENTED_EDGE('',*,*,#" << edge12 << "," << (fwd12 ? ".T." : ".F.") << ");\n";

            uint32_t oe20 = nextId++;
            ss << "#" << oe20 << " = ORIENTED_EDGE('',*,*,#" << edge20 << "," << (fwd20 ? ".T." : ".F.") << ");\n";

            // EDGE_LOOP and FACE_OUTER_BOUND
            uint32_t edgeLoopId = nextId++;
            ss << "#" << edgeLoopId << " = EDGE_LOOP('',(#" << oe01 << ",#" << oe12 << ",#" << oe20 << "));\n";

            uint32_t faceBoundId = nextId++;
            ss << "#" << faceBoundId << " = FACE_OUTER_BOUND('',#" << edgeLoopId << ",.T.);\n";

            // Surface: PLANE with AXIS2_PLACEMENT_3D
            uint32_t faceNormDirId = nextId++;
            ss << "#" << faceNormDirId << " = DIRECTION('',("
               << FormatReal(normal.x) << "," << FormatReal(normal.y) << "," << FormatReal(normal.z) << "));\n";

            uint32_t faceRefDirId = nextId++;
            ss << "#" << faceRefDirId << " = DIRECTION('',("
               << FormatReal(refDir.x) << "," << FormatReal(refDir.y) << "," << FormatReal(refDir.z) << "));\n";

            uint32_t facePlacementId = nextId++;
            ss << "#" << facePlacementId << " = AXIS2_PLACEMENT_3D('',#"
               << geom.pointEntityIds[u0] << ",#" << faceNormDirId << ",#" << faceRefDirId << ");\n";

            uint32_t planeId = nextId++;
            ss << "#" << planeId << " = PLANE('',#" << facePlacementId << ");\n";

            // ADVANCED_FACE
            uint32_t advFaceId = nextId++;
            ss << "#" << advFaceId << " = ADVANCED_FACE('',(#" << faceBoundId << "),#" << planeId << ",.T.);\n";

            faceIds.push_back(advFaceId);
        }

        if (faceIds.empty()) continue;

        // D. CLOSED_SHELL
        uint32_t shellId = nextId++;
        ss << "#" << shellId << " = CLOSED_SHELL('',(";
        for (size_t f = 0; f < faceIds.size(); ++f) {
            ss << "#" << faceIds[f] << (f + 1 < faceIds.size() ? "," : "");
        }
        ss << "));\n";

        // E. MANIFOLD_SOLID_BREP
        uint32_t manifoldSolidId = nextId++;
        ss << "#" << manifoldSolidId << " = MANIFOLD_SOLID_BREP('" << solidName << "',#" << shellId << ");\n";
        manifoldSolidBrepIds.push_back(manifoldSolidId);
    }

    if (manifoldSolidBrepIds.empty()) {
        return "";
    }

    // -------------------------------------------------------------------------
    // 4. Shape Definition Representation & Advanced Brep Shape Representation
    // -------------------------------------------------------------------------
    uint32_t shapeRepId = nextId++;
    ss << "#" << shapeRepId << " = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#" << worldPlacementId;
    for (uint32_t sid : manifoldSolidBrepIds) {
        ss << ",#" << sid;
    }
    ss << "),#" << geomContextId << ");\n";

    uint32_t shapeDefRepId = nextId++;
    ss << "#" << shapeDefRepId << " = SHAPE_DEFINITION_REPRESENTATION(#" << prodDefShapeId << ",#" << shapeRepId << ");\n";

    // -------------------------------------------------------------------------
    // 5. Close DATA section and EOF
    // -------------------------------------------------------------------------
    ss << "ENDSEC;\n";
    ss << "END-ISO-10303-21;\n";

    return ss.str();
}

} // namespace Tessera::Export

