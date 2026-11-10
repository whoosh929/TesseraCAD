#pragma once

#include "engine/graph/Node.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include <string>
#include <vector>
#include <memory>
#include <utility>

namespace Tessera::Export {

struct StepExportOptions {
    std::string author{"User"};
    std::string organization{"TesseraCAD"};
    std::string description{"TesseraCAD Model STEP AP214"};
    std::string modelName{"SolidModel"};
    bool exportAllSolids{true}; // true: export all active solid features in scene; false: export selected
};

struct StepExportResult {
    bool success{false};
    std::string errorMessage;
    std::string filePath;
    size_t solidCount{0};
    size_t faceCount{0};
    size_t vertexCount{0};
    size_t edgeCount{0};
    size_t fileSizeBytes{0};
};

class StepExporter {
public:
    /// Export a single solid DAGNode to a standard STEP AP214 file (.stp / .step)
    static StepExportResult ExportSolid(
        const std::string& filepath,
        const std::shared_ptr<DAGNode>& solidNode,
        const StepExportOptions& options = {}
    );

    /// Export all active (non-consumed) solid features in the parametric graph to a .stp file
    static StepExportResult ExportModel(
        const std::string& filepath,
        const ParametricGraph& graph,
        const StepExportOptions& options = {}
    );

    /// Export raw mesh data (vertices + triangle indices) as a standard STEP AP214 B-Rep solid
    static StepExportResult ExportMesh(
        const std::string& filepath,
        const std::string& solidName,
        const std::vector<CADVertex>& vertices,
        const std::vector<uint32_t>& indices,
        const StepExportOptions& options = {}
    );

    /// Generate STEP AP214 clear text content for given solid nodes
    static std::string GenerateStepString(
        const std::vector<std::pair<std::string, const DAGNode*>>& solids,
        const StepExportOptions& options = {}
    );

    /// Generate STEP AP214 clear text content for raw mesh
    static std::string GenerateStepStringFromMesh(
        const std::string& solidName,
        const std::vector<CADVertex>& vertices,
        const std::vector<uint32_t>& indices,
        const StepExportOptions& options = {}
    );
};

} // namespace Tessera::Export

