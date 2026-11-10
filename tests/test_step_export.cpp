#include "engine/export/StepExporter.hpp"
#include "engine/graph/ParametricGraph.hpp"
#include "engine/graph/ExtrudeNode.hpp"
#include "engine/graph/SketchNode.hpp"
#include "engine/concurrency/ThreadPool.hpp"
#include "engine/concurrency/AtomicMeshExchange.hpp"
#include <iostream>
#include <cassert>
#include <fstream>
#include <sstream>
#include <filesystem>

void AssertMsg(bool condition, const std::string& msg) {
    if (!condition) {
        std::cerr << "[-] Assertion FAILED: " << msg << std::endl;
        std::exit(1);
    }
}

int main() {
    std::cout << "[+] Running STEP Exporter Unit Tests (ISO 10303-21 AP214)..." << std::endl;

    std::string tempDir = "test_step_output";
    std::filesystem::create_directories(tempDir);

    // =========================================================================
    // Test 1: Export Raw Mesh to STEP
    // =========================================================================
    std::cout << "[+] Test 1: Exporting simple mesh (triangulated cube/prism) to STEP..." << std::endl;
    {
        // Simple triangular prism (5 faces: 2 triangle caps, 3 quad sides = 8 triangles total)
        std::vector<Tessera::CADVertex> vertices = {
            // Bottom cap (z = 0)
            {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}},
            {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}},
            {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}},
            // Top cap (z = 1)
            {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
            {{1.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
            {{0.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
        };

        std::vector<uint32_t> indices = {
            // Bottom cap (reversed for outward normal)
            0, 2, 1,
            // Top cap
            3, 4, 5,
            // Side 1 (0-1-4-3)
            0, 1, 4,  0, 4, 3,
            // Side 2 (1-2-5-4)
            1, 2, 5,  1, 5, 4,
            // Side 3 (2-0-3-5)
            2, 0, 3,  2, 3, 5
        };

        std::string testPath = tempDir + "/test_prism.stp";
        auto res = Tessera::Export::StepExporter::ExportMesh(testPath, "PrismPart", vertices, indices);
        AssertMsg(res.success, "ExportMesh should succeed: " + res.errorMessage);
        AssertMsg(res.faceCount == 8, "Expected 8 facet faces exported, got: " + std::to_string(res.faceCount));
        AssertMsg(res.fileSizeBytes > 500, "STEP file size should be substantial");

        // Read and verify STEP content
        std::ifstream ifs(testPath);
        AssertMsg(ifs.is_open(), "Exported STEP file must exist and be readable");
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        std::string content = buffer.str();

        AssertMsg(content.find("ISO-10303-21;") != std::string::npos, "Must start with ISO-10303-21; header");
        AssertMsg(content.find("HEADER;") != std::string::npos, "Must have HEADER section");
        AssertMsg(content.find("AUTOMOTIVE_DESIGN") != std::string::npos, "Must contain AP214 schema");
        AssertMsg(content.find("DATA;") != std::string::npos, "Must have DATA section");
        AssertMsg(content.find("MANIFOLD_SOLID_BREP('PrismPart',") != std::string::npos, "Must contain MANIFOLD_SOLID_BREP with solid name");
        AssertMsg(content.find("CLOSED_SHELL") != std::string::npos, "Must contain CLOSED_SHELL entity");
        AssertMsg(content.find("ADVANCED_FACE") != std::string::npos, "Must contain ADVANCED_FACE entities");
        AssertMsg(content.find("EDGE_LOOP") != std::string::npos, "Must contain EDGE_LOOP entities");
        AssertMsg(content.find("ORIENTED_EDGE") != std::string::npos, "Must contain ORIENTED_EDGE entities");
        AssertMsg(content.find("EDGE_CURVE") != std::string::npos, "Must contain EDGE_CURVE entities");
        AssertMsg(content.find("CARTESIAN_POINT") != std::string::npos, "Must contain CARTESIAN_POINT entities");
        AssertMsg(content.find("END-ISO-10303-21;") != std::string::npos, "Must end with END-ISO-10303-21;");
        std::cout << "[+] Test 1 PASSED! (Exported " << res.faceCount << " faces, " << res.fileSizeBytes << " bytes)" << std::endl;
    }

    // =========================================================================
    // Test 2: Export Solid Feature (ExtrudeNode) to STEP
    // =========================================================================
    std::cout << "[+] Test 2: Exporting ExtrudeNode solid to STEP..." << std::endl;
    {
        auto threadPool = std::make_shared<Tessera::ThreadPool>(2);
        Tessera::ParametricGraph graph(threadPool);
        Tessera::AtomicMeshExchange exchange;

        // Create rectangular 2D sketch
        auto sketch = std::make_shared<Tessera::SketchNode>("Sketch_Box", Tessera::SketchPlane::XZ_Ground);
        sketch->SetPresetRectangle(4.0f, 2.0f);
        sketch->Evaluate(nullptr, exchange);
        graph.AddNode(sketch);

        // Extrude 3D solid
        auto extrude = std::make_shared<Tessera::ExtrudeNode>("Extrude_Box", sketch, 3.0f, Tessera::ExtrudeOperation::Boss);
        extrude->Evaluate(nullptr, exchange);
        graph.AddNode(extrude);

        AssertMsg(!extrude->GetCachedVertices().empty(), "ExtrudeNode must produce cached vertices");
        AssertMsg(!extrude->GetCachedIndices().empty(), "ExtrudeNode must produce cached triangle indices");

        std::string testPath = tempDir + "/test_extrude_box.stp";
        auto res = Tessera::Export::StepExporter::ExportSolid(testPath, extrude);
        AssertMsg(res.success, "ExportSolid should succeed: " + res.errorMessage);
        AssertMsg(res.solidCount == 1, "ExportSolid should export 1 solid");
        AssertMsg(res.fileSizeBytes > 1000, "STEP file should have valid non-empty size");

        std::ifstream ifs(testPath);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        std::string content = buffer.str();
        AssertMsg(content.find("MANIFOLD_SOLID_BREP('Extrude_Box',") != std::string::npos, "Must have MANIFOLD_SOLID_BREP with node name");
        std::cout << "[+] Test 2 PASSED! (Exported " << res.solidCount << " solid, " << res.fileSizeBytes << " bytes)" << std::endl;
    }

    // =========================================================================
    // Test 3: Export Whole Parametric Graph (Multiple Active Solids) to STEP
    // =========================================================================
    std::cout << "[+] Test 3: Exporting entire model with multiple solids to STEP..." << std::endl;
    {
        auto threadPool = std::make_shared<Tessera::ThreadPool>(2);
        Tessera::ParametricGraph graph(threadPool);
        Tessera::AtomicMeshExchange exchange;

        // Solid 1: Box
        auto sketch1 = std::make_shared<Tessera::SketchNode>("Sketch_Part1", Tessera::SketchPlane::XZ_Ground);
        sketch1->SetPresetRectangle(2.0f, 2.0f);
        sketch1->Evaluate(nullptr, exchange);
        graph.AddNode(sketch1);

        auto extrude1 = std::make_shared<Tessera::ExtrudeNode>("Extrude_Part1", sketch1, 2.0f, Tessera::ExtrudeOperation::Boss);
        extrude1->Evaluate(nullptr, exchange);
        graph.AddNode(extrude1);

        // Solid 2: L-Bracket offset in space
        auto sketch2 = std::make_shared<Tessera::SketchNode>("Sketch_Part2", Tessera::SketchPlane::XY_Front);
        sketch2->SetPresetLBracket(3.0f, 3.0f, 1.0f);
        sketch2->SetPosition(5.0f, 0.0f, 0.0f);
        sketch2->Evaluate(nullptr, exchange);
        graph.AddNode(sketch2);

        auto extrude2 = std::make_shared<Tessera::ExtrudeNode>("Extrude_Part2", sketch2, 1.5f, Tessera::ExtrudeOperation::Boss);
        extrude2->Evaluate(nullptr, exchange);
        graph.AddNode(extrude2);

        std::string testPath = tempDir + "/test_full_model.stp";
        auto res = Tessera::Export::StepExporter::ExportModel(testPath, graph);
        AssertMsg(res.success, "ExportModel should succeed: " + res.errorMessage);
        AssertMsg(res.solidCount == 2, "Expected 2 active solid bodies exported, got: " + std::to_string(res.solidCount));

        std::ifstream ifs(testPath);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        std::string content = buffer.str();

        AssertMsg(content.find("Extrude_Part1") != std::string::npos, "Model STEP file must contain Part 1");
        AssertMsg(content.find("Extrude_Part2") != std::string::npos, "Model STEP file must contain Part 2");
        AssertMsg(content.find("ADVANCED_BREP_SHAPE_REPRESENTATION") != std::string::npos, "Must contain ADVANCED_BREP_SHAPE_REPRESENTATION");
        std::cout << "[+] Test 3 PASSED! (Exported " << res.solidCount << " solids, " << res.fileSizeBytes << " bytes)" << std::endl;
    }

    // Clean up
    try {
        std::filesystem::remove_all(tempDir);
    } catch (...) {}

    std::cout << "[+] ALL STEP Exporter Unit Tests PASSED Successfully!" << std::endl;
    return 0;
}
