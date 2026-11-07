#include "CADGui.hpp"
#include "engine/sketch/PolygonBool2D.hpp"
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_vulkan.h"
#include "engine/hal/vulkan/VulkanDevice.hpp"
#include <GLFW/glfw3.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace Tessera {

CADGui::~CADGui() {
    Shutdown();
}

void CADGui::Init(GLFWwindow* window, RHI::RHIDevice* rhiDevice) {
    if (m_initialized) return;
    m_rhiDevice = rhiDevice;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Apply sleek modern dark CAD workstation styling
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.GrabRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.ItemSpacing = ImVec2(8.0f, 6.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]         = ImVec4(0.12f, 0.13f, 0.15f, 0.95f);
    colors[ImGuiCol_Header]           = ImVec4(0.20f, 0.25f, 0.32f, 1.00f);
    colors[ImGuiCol_HeaderHovered]    = ImVec4(0.28f, 0.35f, 0.45f, 1.00f);
    colors[ImGuiCol_HeaderActive]     = ImVec4(0.24f, 0.40f, 0.58f, 1.00f);
    colors[ImGuiCol_Button]           = ImVec4(0.22f, 0.28f, 0.36f, 1.00f);
    colors[ImGuiCol_ButtonHovered]    = ImVec4(0.28f, 0.42f, 0.60f, 1.00f);
    colors[ImGuiCol_ButtonActive]     = ImVec4(0.20f, 0.50f, 0.78f, 1.00f);
    colors[ImGuiCol_FrameBg]          = ImVec4(0.18f, 0.20f, 0.24f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]   = ImVec4(0.24f, 0.28f, 0.34f, 1.00f);
    colors[ImGuiCol_FrameBgActive]    = ImVec4(0.28f, 0.35f, 0.44f, 1.00f);
    colors[ImGuiCol_TitleBg]          = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
    colors[ImGuiCol_TitleBgActive]    = ImVec4(0.16f, 0.22f, 0.30f, 1.00f);
    colors[ImGuiCol_SliderGrab]       = ImVec4(0.35f, 0.55f, 0.85f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.45f, 0.70f, 1.00f, 1.00f);
    colors[ImGuiCol_CheckMark]        = ImVec4(0.40f, 0.75f, 1.00f, 1.00f);

    // Initialize backend wrappers (false = manual forwarding via Application::SetupCallbacks)
    ImGui_ImplGlfw_InitForVulkan(window, false);

    if (auto* vkDevice = dynamic_cast<RHI::VulkanDevice*>(rhiDevice)) {
        ImGui_ImplVulkan_InitInfo initInfo{};
        initInfo.Instance = vkDevice->GetVkInstance();
        initInfo.PhysicalDevice = vkDevice->GetVkPhysicalDevice();
        initInfo.Device = vkDevice->GetVkDevice();
        initInfo.QueueFamily = vkDevice->GetGraphicsQueueFamily();
        initInfo.Queue = vkDevice->GetGraphicsQueue();
        initInfo.DescriptorPool = vkDevice->GetDescriptorPool();
        initInfo.MinImageCount = 2;
        initInfo.ImageCount = vkDevice->GetImageCount();
        initInfo.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        initInfo.RenderPass = vkDevice->GetRenderPass();

        ImGui_ImplVulkan_Init(&initInfo);
    }

    m_initialized = true;
    std::cout << "[CADGui] Dear ImGui docking CAD interface initialized.\n";
}

void CADGui::Shutdown() {
    if (!m_initialized) return;

    if (dynamic_cast<RHI::VulkanDevice*>(m_rhiDevice)) {
        ImGui_ImplVulkan_Shutdown();
    }
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    m_initialized = false;
}

void CADGui::BeginFrame() {
    if (!m_initialized) return;

    if (dynamic_cast<RHI::VulkanDevice*>(m_rhiDevice)) {
        ImGui_ImplVulkan_NewFrame();
    }
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

bool CADGui::WantCaptureMouse() const noexcept {
    return m_initialized && ImGui::GetIO().WantCaptureMouse;
}

bool CADGui::WantCaptureKeyboard() const noexcept {
    return m_initialized && ImGui::GetIO().WantCaptureKeyboard;
}

void CADGui::RenderTopBar(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes,
    std::shared_ptr<DAGNode> selectedNode
) {
    (void)nodes;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, 52.0f));

    ImGuiWindowFlags topBarFlags = ImGuiWindowFlags_NoTitleBar |
                                   ImGuiWindowFlags_NoResize |
                                   ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoScrollbar |
                                   ImGuiWindowFlags_NoSavedSettings;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 8.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.11f, 0.12f, 0.15f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.22f, 0.25f, 0.32f, 1.0f));

    if (ImGui::Begin("##CADTopBar", nullptr, topBarFlags)) {
        // App Title / Branding
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ImVec4(0.25f, 0.75f, 1.0f, 1.0f), "⬡ TesseraCAD");
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        // -------------------------------------------------------------
        // 1. Create Reference Plane
        // -------------------------------------------------------------
        if (ImGui::Button("⊞ Ref Plane", ImVec2(100, 32))) {
            ImGui::OpenPopup("RefPlanePopup");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("1. Create Reference Datum Plane (XY Front, XZ Top/Ground, YZ Right, or Offset)");
        }
        if (ImGui::BeginPopup("RefPlanePopup")) {
            ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Create Reference Plane");
            ImGui::Separator();
            if (m_selectedFaceIndex >= 0 && selectedNode) {
                if (ImGui::MenuItem("★ Tie to Selected Model Face")) {
                    std::string name = "Plane_Face_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumPlaneNode>(name, selectedNode, m_selectedFaceIndex, m_selectedFacePlane);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                if (ImGui::MenuItem("★ Tie to Selected Model Vertex")) {
                    std::string name = "Plane_Vertex_" + std::to_string(m_shapeCounter++);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    auto node = std::make_shared<DatumPlaneNode>(name, selectedNode, m_selectedVertexIndex, pl, true);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            }
            if (ImGui::MenuItem("Top Plane (XZ Ground)")) {
                std::string name = "Plane_XZ_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumPlaneNode>(name, Geom::Plane::XZ());
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("Front Plane (XY)")) {
                std::string name = "Plane_XY_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumPlaneNode>(name, Geom::Plane::XY());
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("Right Plane (YZ)")) {
                std::string name = "Plane_YZ_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumPlaneNode>(name, Geom::Plane::YZ());
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (auto selectedPlane = std::dynamic_pointer_cast<DatumPlaneNode>(selectedNode)) {
                ImGui::Separator();
                if (ImGui::MenuItem("Offset Plane (+1.0m from Selected)")) {
                    std::string name = "Plane_Offset_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumPlaneNode>(name, selectedPlane->GetPlane().offset(1.0));
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
            }
            ImGui::EndPopup();
        }

        ImGui::SameLine();

        // -------------------------------------------------------------
        // 2. Create Reference Axis
        // -------------------------------------------------------------
        if (ImGui::Button("⌖ Ref Axis", ImVec2(96, 32))) {
            ImGui::OpenPopup("RefAxisPopup");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("2. Create Reference Datum Axis (Y Up, X Horizontal, Z Depth)");
        }
        if (ImGui::BeginPopup("RefAxisPopup")) {
            ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Create Reference Axis");
            ImGui::Separator();
            if (m_selectedVertexIndex >= 0 && selectedNode) {
                if (ImGui::MenuItem("★ Tie to Selected Model Vertex")) {
                    std::string name = "Axis_Vertex_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumAxisNode>(name, selectedNode, m_selectedVertexIndex, Geom::Vector3D(0.0, 1.0, 0.0));
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            }
            if (ImGui::MenuItem("Y-Axis (Vertical Up)")) {
                std::string name = "Axis_Y_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumAxisNode>(name, Geom::Axis3D::Y());
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("X-Axis (Horizontal)")) {
                std::string name = "Axis_X_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumAxisNode>(name, Geom::Axis3D::X());
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("Z-Axis (Depth)")) {
                std::string name = "Axis_Z_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumAxisNode>(name, Geom::Axis3D::Z());
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            ImGui::EndPopup();
        }

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        // -------------------------------------------------------------
        // 3. Create 2D Sketch
        // -------------------------------------------------------------
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.35f, 0.55f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.48f, 0.75f, 1.0f));
        if (ImGui::Button("✎ 2D Sketch", ImVec2(105, 32))) {
            // When creating a 2d sketch:
            // 1) If a face on a model is selected, automatically use that face as datum plane!
            // 2) If a datum plane is selected, use that datum plane.
            // 3) If no plane is selected, ask to identify either by clicking on model or from tree.
            if (m_selectedFaceIndex >= 0 && selectedNode) {
                std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                auto planeNode = std::make_shared<DatumPlaneNode>(planeName, selectedNode, m_selectedFaceIndex, m_selectedFacePlane);
                graph.AddNode(planeNode);

                std::string name = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                sketch->SetPosition(static_cast<float>(m_selectedFacePlane.origin.x), static_cast<float>(m_selectedFacePlane.origin.y), static_cast<float>(m_selectedFacePlane.origin.z));
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                m_activeSketchId = sketch->GetId();
                m_sketchModeActive = true;
                m_showSketchPlanePrompt = false;
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                std::string planeName = "Plane_Vertex_" + std::to_string(m_shapeCounter);
                Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                auto planeNode = std::make_shared<DatumPlaneNode>(planeName, selectedNode, m_selectedVertexIndex, pl, true);
                graph.AddNode(planeNode);

                std::string name = "Sketch_Vertex_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                sketch->SetPosition(static_cast<float>(m_selectedVertexPos.x), static_cast<float>(m_selectedVertexPos.y), static_cast<float>(m_selectedVertexPos.z));
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                m_activeSketchId = sketch->GetId();
                m_sketchModeActive = true;
                m_showSketchPlanePrompt = false;
            } else if (auto planeNode = std::dynamic_pointer_cast<DatumPlaneNode>(selectedNode)) {
                Geom::Plane pl = planeNode->GetPlane();
                std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                sketch->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                m_activeSketchId = sketch->GetId();
                m_sketchModeActive = true;
                m_showSketchPlanePrompt = false;
            } else {
                m_showSketchPlanePrompt = true;
                m_promptInitialSelectedId = m_selectedNodeId;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("3. Create 2D Sketch:\n- If a plane is selected, automatically uses it as sketch plane.\n- If no plane is selected, prompts to identify plane on model or tree.");
        }

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        // -------------------------------------------------------------
        // 4. Create Model Features (Extrude, Revolve, etc.)
        // -------------------------------------------------------------
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.38f, 0.28f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.52f, 0.38f, 1.0f));
        if (ImGui::Button("⤊ Extrude", ImVec2(92, 32))) {
            // If a 2d sketch is selected, try to make the feature from that sketch automatically.
            // If a 2d sketch is not selected, ask to reference one in the model tree or allow you to jump into 2d sketch creation.
            auto sketchNode = std::dynamic_pointer_cast<SketchNode>(selectedNode);
            if (sketchNode && !sketchNode->HasParentFeature() && !sketchNode->IsConsumed()) {
                std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                auto extrude = std::make_shared<ExtrudeNode>(name, sketchNode, m_newExtrudeHeight);
                graph.AddNode(extrude);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(extrude->GetId());
                m_showExtrudePrompt = false;
            } else {
                m_showExtrudePrompt = true;
                m_promptInitialSelectedId = m_selectedNodeId;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("4. Create Extrude Feature:\n- Automatically creates from selected sketch.\n- If none selected, prompts to pick from model tree or jump into sketch creation.");
        }

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.38f, 0.32f, 0.18f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.52f, 0.44f, 0.26f, 1.0f));
        if (ImGui::Button("↻ Revolve", ImVec2(92, 32))) {
            auto sketchNode = std::dynamic_pointer_cast<SketchNode>(selectedNode);
            if (sketchNode && !sketchNode->HasParentFeature() && !sketchNode->IsConsumed()) {
                std::string name = "Revolve_" + std::to_string(m_shapeCounter++);
                auto revolve = std::make_shared<RevolveNode>(name, sketchNode, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis);
                graph.AddNode(revolve);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(revolve->GetId());
                m_showRevolvePrompt = false;
            } else {
                m_showRevolvePrompt = true;
                m_promptInitialSelectedId = m_selectedNodeId;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("4. Create Revolve Feature:\n- Revolve sketch around an axis to create a cylindrical 3D model.\n- Cylindrical wall thickness is directly dictated by 2D sketch drawing.");
        }

        // Fillet & Chamfer features
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.35f, 0.48f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.48f, 0.64f, 1.0f));
        if (ImGui::Button("⌒ Fillet", ImVec2(80, 32))) {
            if (m_selectedEdgeIndex >= 0 && selectedNode) {
                std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                auto fillet = std::make_shared<FilletNode>(name, selectedNode, m_selectedEdgeIndex, m_newFilletRadius);
                graph.AddNode(fillet);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(fillet->GetId());
                m_showFilletPrompt = false;
            } else {
                m_showFilletPrompt = true;
                m_showChamferPrompt = false;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Create Fillet Feature:\n- Rounds a model corner edge with input radius.\n- Click on any model corner edge in 3D viewport, then apply Fillet.");
        }

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.36f, 0.24f, 0.44f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.48f, 0.32f, 0.58f, 1.0f));
        if (ImGui::Button("⟍ Chamfer", ImVec2(90, 32))) {
            if (m_selectedEdgeIndex >= 0 && selectedNode) {
                std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                auto chamfer = std::make_shared<ChamferNode>(name, selectedNode, m_selectedEdgeIndex, m_newChamferDistance);
                graph.AddNode(chamfer);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(chamfer->GetId());
                m_showChamferPrompt = false;
            } else {
                m_showChamferPrompt = true;
                m_showFilletPrompt = false;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Create Chamfer Feature:\n- Bevels a model corner edge with input distance.\n- Click on any model corner edge in 3D viewport, then apply Chamfer.");
        }

        // Right side: Selection status and quick deselect
        float rightContentWidth = 210.0f;
        if (ImGui::GetContentRegionAvail().x > rightContentWidth) {
            ImGui::SameLine(ImGui::GetWindowWidth() - rightContentWidth);
            if (selectedNode) {
                ImGui::AlignTextToFramePadding();
                std::string selText = "Selected: " + selectedNode->GetName();
                if (selText.length() > 20) selText = selText.substr(0, 18) + "..";
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "%s", selText.c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton("✕ Deselect")) {
                    SetSelectedNodeId(0);
                }
            } else {
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("Selected: None");
            }
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void CADGui::RenderSketchPlanePromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showSketchPlanePrompt) return;

    // Check if user clicked a node on the model or tree while prompt was active
    if (m_selectedNodeId != 0 && (m_selectedNodeId != m_promptInitialSelectedId || m_selectedFaceIndex >= 0 || m_selectedVertexIndex >= 0)) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (m_selectedFaceIndex >= 0) {
                    // Clicked on a specific model face: bind associative datum plane and sketch!
                    std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, n, m_selectedFaceIndex, m_selectedFacePlane);
                    graph.AddNode(planeNode);

                    std::string name = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetPosition(static_cast<float>(m_selectedFacePlane.origin.x), static_cast<float>(m_selectedFacePlane.origin.y), static_cast<float>(m_selectedFacePlane.origin.z));
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    m_activeSketchId = newSketch->GetId();
                    m_sketchModeActive = true;
                    m_showSketchPlanePrompt = false;
                    return;
                } else if (m_selectedVertexIndex >= 0) {
                    // Clicked on a model vertex: bind vertex datum plane and sketch centered at vertex!
                    std::string planeName = "Plane_Vertex_" + std::to_string(m_shapeCounter);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, n, m_selectedVertexIndex, pl, true);
                    graph.AddNode(planeNode);

                    std::string name = "Sketch_Vertex_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetPosition(static_cast<float>(m_selectedVertexPos.x), static_cast<float>(m_selectedVertexPos.y), static_cast<float>(m_selectedVertexPos.z));
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    m_activeSketchId = newSketch->GetId();
                    m_sketchModeActive = true;
                    m_showSketchPlanePrompt = false;
                    return;
                } else if (auto p = std::dynamic_pointer_cast<DatumPlaneNode>(n)) {
                    Geom::Plane pl = p->GetPlane();
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, p);
                    newSketch->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    m_activeSketchId = newSketch->GetId();
                    m_sketchModeActive = true;
                    m_showSketchPlanePrompt = false;
                    return;
                } else {
                    // Clicked on a solid model in 3D: use its position as sketch plane!
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XZ_Ground);
                    sketch->SetPosition(n->GetPosition()[0], n->GetPosition()[1], n->GetPosition()[2]);
                    graph.AddNode(sketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(sketch->GetId());
                    m_activeSketchId = sketch->GetId();
                    m_sketchModeActive = true;
                    m_showSketchPlanePrompt = false;
                    return;
                }
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.13f, 0.15f, 0.20f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.25f, 0.65f, 1.0f, 1.0f));

    if (ImGui::Begin("Select Sketch Plane##Modal", &m_showSketchPlanePrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.25f, 0.8f, 1.0f, 1.0f), "✎ Select a Sketch Plane");
        ImGui::Separator();
        ImGui::TextWrapped("Identify a plane by clicking on the model or datum plane in the 3D viewport, or choose from the Model Tree / datum planes below:");
        ImGui::Spacing();

        // 1. Standard Datum Planes
        ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Standard Datum Planes:");
        if (ImGui::Button("⬛ Top Plane (XZ Ground)", ImVec2(146, 32))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XZ_Ground);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            m_activeSketchId = sketch->GetId();
            m_sketchModeActive = true;
            m_showSketchPlanePrompt = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("⬛ Front Plane (XY)", ImVec2(146, 32))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XY_Front);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            m_activeSketchId = sketch->GetId();
            m_sketchModeActive = true;
            m_showSketchPlanePrompt = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("⬛ Right Plane (YZ)", ImVec2(146, 32))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::YZ_Right);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            m_activeSketchId = sketch->GetId();
            m_sketchModeActive = true;
            m_showSketchPlanePrompt = false;
        }

        // 2. User-Created Datum Planes in Model Tree
        std::vector<std::shared_ptr<DatumPlaneNode>> userPlanes;
        for (const auto& n : nodes) {
            if (auto p = std::dynamic_pointer_cast<DatumPlaneNode>(n)) {
                userPlanes.push_back(p);
            }
        }
        if (!userPlanes.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Reference Planes from Model Tree:");
            for (const auto& p : userPlanes) {
                std::string btnLabel = "⊞ Use: " + p->GetName() + " (Offset: " + std::to_string(p->GetOffset()) + "m)";
                if (ImGui::Button(btnLabel.c_str(), ImVec2(-1, 28))) {
                    Geom::Plane pl = p->GetPlane();
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, p);
                    newSketch->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    m_activeSketchId = newSketch->GetId();
                    m_sketchModeActive = true;
                    m_showSketchPlanePrompt = false;
                    break;
                }
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showSketchPlanePrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderExtrudePromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showExtrudePrompt) return;

    // Check if user selected an available sketch from the Model Tree while prompt was open
    if (m_selectedNodeId != 0 && m_selectedNodeId != m_promptInitialSelectedId) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    if (!s->HasParentFeature() && !s->IsConsumed()) {
                        std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                        auto extrude = std::make_shared<ExtrudeNode>(name, s, m_newExtrudeHeight);
                        graph.AddNode(extrude);
                        graph.EvaluateDirtyNodes(device, exchange);
                        SetSelectedNodeId(extrude->GetId());
                        m_showExtrudePrompt = false;
                        return;
                    }
                }
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.13f, 0.15f, 0.20f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.35f, 0.75f, 0.45f, 1.0f));

    if (ImGui::Begin("Extrude Feature - Select Reference Sketch##Modal", &m_showExtrudePrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 0.5f, 1.0f), "⤊ Extrude from Reference Sketch");
        ImGui::Separator();
        ImGui::TextWrapped("No 2D sketch is currently selected. Select an existing sketch from the Model Tree / list, or jump directly into 2D sketch creation:");
        ImGui::Spacing();

        std::vector<std::shared_ptr<SketchNode>> availableSketches;
        for (const auto& n : nodes) {
            if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                if (!s->HasParentFeature() && !s->IsConsumed()) {
                    availableSketches.push_back(s);
                }
            }
        }

        if (!availableSketches.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Available Sketches in Model Tree:");
            ImGui::InputFloat("Extrude Depth", &m_newExtrudeHeight, 0.1f, 1.0f, "%.2f m");
            if (std::abs(m_newExtrudeHeight) < 0.001f) m_newExtrudeHeight = (m_newExtrudeHeight < 0.0f ? -0.01f : 0.01f);
            ImGui::Spacing();

            for (const auto& s : availableSketches) {
                std::string btnLabel = "⤊ Extrude Sketch: '" + s->GetName() + "' (ID " + std::to_string(s->GetId()) + ", " + std::to_string(s->GetProfile().size()) + " pts)";
                if (ImGui::Button(btnLabel.c_str(), ImVec2(-1, 30))) {
                    std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                    auto extrude = std::make_shared<ExtrudeNode>(name, s, m_newExtrudeHeight);
                    graph.AddNode(extrude);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(extrude->GetId());
                    m_showExtrudePrompt = false;
                    break;
                }
            }
            ImGui::Spacing();
        } else {
            ImGui::TextDisabled("No available (unconsumed) 2D sketches exist in the model tree.");
            ImGui::Spacing();
        }

        // Jump into 2D sketch creation
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.38f, 0.65f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.50f, 0.85f, 1.0f));
        if (ImGui::Button("✎ Jump into 2D Sketch Creation", ImVec2(-1, 34))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XZ_Ground);
            sketch->SetPresetRectangle(3.5f, 2.5f);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            m_activeSketchId = sketch->GetId();
            m_sketchModeActive = true;
            m_showExtrudePrompt = false;
        }
        ImGui::PopStyleColor(2);

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showExtrudePrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderRevolvePromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showRevolvePrompt) return;

    if (m_selectedNodeId != 0 && m_selectedNodeId != m_promptInitialSelectedId) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    if (!s->HasParentFeature() && !s->IsConsumed()) {
                        std::string name = "Revolve_" + std::to_string(m_shapeCounter++);
                        auto revolve = std::make_shared<RevolveNode>(name, s, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis);
                        graph.AddNode(revolve);
                        graph.EvaluateDirtyNodes(device, exchange);
                        SetSelectedNodeId(revolve->GetId());
                        m_showRevolvePrompt = false;
                        return;
                    }
                }
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.13f, 0.15f, 0.20f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.85f, 0.65f, 0.25f, 1.0f));

    if (ImGui::Begin("Revolve Feature - Select Reference Sketch##Modal", &m_showRevolvePrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "↻ Revolve from Reference Sketch");
        ImGui::Separator();
        ImGui::TextWrapped("No 2D sketch is currently selected. Select an existing sketch from the Model Tree / list, or jump directly into 2D sketch creation:");
        ImGui::Spacing();

        std::vector<std::shared_ptr<SketchNode>> availableSketches;
        for (const auto& n : nodes) {
            if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                if (!s->HasParentFeature() && !s->IsConsumed()) {
                    availableSketches.push_back(s);
                }
            }
        }

        if (!availableSketches.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Available Sketches in Model Tree:");
            ImGui::InputFloat("Revolve Angle", &m_newRevolveAngle, 1.0f, 15.0f, "%.1f deg");
            if (m_newRevolveAngle < 0.1f) m_newRevolveAngle = 0.1f;
            if (m_newRevolveAngle > 360.0f) m_newRevolveAngle = 360.0f;
            ImGui::Spacing();

            for (const auto& s : availableSketches) {
                std::string btnLabel = "↻ Revolve Sketch: '" + s->GetName() + "' (ID " + std::to_string(s->GetId()) + ")";
                if (ImGui::Button(btnLabel.c_str(), ImVec2(-1, 30))) {
                    std::string name = "Revolve_" + std::to_string(m_shapeCounter++);
                    auto revolve = std::make_shared<RevolveNode>(name, s, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis);
                    graph.AddNode(revolve);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(revolve->GetId());
                    m_showRevolvePrompt = false;
                    break;
                }
            }
            ImGui::Spacing();
        } else {
            ImGui::TextDisabled("No available (unconsumed) 2D sketches exist in the model tree.");
            ImGui::Spacing();
        }

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.38f, 0.65f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.50f, 0.85f, 1.0f));
        if (ImGui::Button("✎ Jump into 2D Sketch Creation", ImVec2(-1, 34))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XY_Front);
            // Profile: rectangle from u=1.0 to 2.5, v=0.0 to 3.0 (cylinder with wall thickness 1.5m)
            sketch->SetProfile({ {1.0f, 0.0f}, {2.5f, 0.0f}, {2.5f, 3.0f}, {1.0f, 3.0f} });
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            m_activeSketchId = sketch->GetId();
            m_sketchModeActive = true;
            m_showRevolvePrompt = false;
        }
        ImGui::PopStyleColor(2);

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showRevolvePrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderFilletPromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showFilletPrompt) return;

    if (m_selectedEdgeIndex >= 0 && m_selectedNodeId != 0) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                auto fillet = std::make_shared<FilletNode>(name, n, m_selectedEdgeIndex, m_newFilletRadius);
                graph.AddNode(fillet);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(fillet->GetId());
                m_showFilletPrompt = false;
                return;
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.12f, 0.16f, 0.22f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.25f, 0.55f, 0.85f, 1.0f));
    if (ImGui::Begin("⌒ Create Fillet Feature###FilletPrompt", &m_showFilletPrompt, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Corner Edge Fillet Feature:");
        ImGui::TextWrapped("Click any corner edge on a 3D model in the viewport, or select one below.");
        ImGui::Spacing();

        ImGui::InputFloat("Fillet Radius", &m_newFilletRadius, 0.05f, 0.2f, "%.2f m");
        if (m_newFilletRadius < 0.01f) m_newFilletRadius = 0.01f;
        ImGui::Spacing();

        bool hasModels = false;
        for (const auto& n : nodes) {
            if (!n || n->IsConsumed() || std::dynamic_pointer_cast<SketchNode>(n) || std::dynamic_pointer_cast<DatumPlaneNode>(n) || std::dynamic_pointer_cast<DatumAxisNode>(n)) continue;
            const auto& edges = n->GetCachedEdges();
            if (edges.empty()) continue;

            hasModels = true;
            size_t numEdges = edges.size() / 2;
            std::string btn = "⌒ Apply Fillet to Edge #0 of '" + n->GetName() + "' (" + std::to_string(numEdges) + " edges)";
            if (ImGui::Button(btn.c_str(), ImVec2(-1, 28))) {
                std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                auto fillet = std::make_shared<FilletNode>(name, n, 0, m_newFilletRadius);
                graph.AddNode(fillet);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(fillet->GetId());
                m_showFilletPrompt = false;
                break;
            }
        }
        if (!hasModels) {
            ImGui::TextDisabled("No 3D solid models exist in the scene. Create an Extrude or Revolve first.");
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showFilletPrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderChamferPromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showChamferPrompt) return;

    if (m_selectedEdgeIndex >= 0 && m_selectedNodeId != 0) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                auto chamfer = std::make_shared<ChamferNode>(name, n, m_selectedEdgeIndex, m_newChamferDistance);
                graph.AddNode(chamfer);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(chamfer->GetId());
                m_showChamferPrompt = false;
                return;
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.18f, 0.14f, 0.22f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.65f, 0.40f, 0.85f, 1.0f));
    if (ImGui::Begin("⟍ Create Chamfer Feature###ChamferPrompt", &m_showChamferPrompt, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.75f, 0.55f, 1.0f, 1.0f), "Corner Edge Chamfer Feature:");
        ImGui::TextWrapped("Click any corner edge on a 3D model in the viewport, or select one below.");
        ImGui::Spacing();

        ImGui::InputFloat("Chamfer Distance", &m_newChamferDistance, 0.05f, 0.2f, "%.2f m");
        if (m_newChamferDistance < 0.01f) m_newChamferDistance = 0.01f;
        ImGui::Spacing();

        bool hasModels = false;
        for (const auto& n : nodes) {
            if (!n || n->IsConsumed() || std::dynamic_pointer_cast<SketchNode>(n) || std::dynamic_pointer_cast<DatumPlaneNode>(n) || std::dynamic_pointer_cast<DatumAxisNode>(n)) continue;
            const auto& edges = n->GetCachedEdges();
            if (edges.empty()) continue;

            hasModels = true;
            size_t numEdges = edges.size() / 2;
            std::string btn = "⟍ Apply Chamfer to Edge #0 of '" + n->GetName() + "' (" + std::to_string(numEdges) + " edges)";
            if (ImGui::Button(btn.c_str(), ImVec2(-1, 28))) {
                std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                auto chamfer = std::make_shared<ChamferNode>(name, n, 0, m_newChamferDistance);
                graph.AddNode(chamfer);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(chamfer->GetId());
                m_showChamferPrompt = false;
                break;
            }
        }
        if (!hasModels) {
            ImGui::TextDisabled("No 3D solid models exist in the scene. Create an Extrude or Revolve first.");
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showChamferPrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderConflictPromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showConflictPrompt) return;

    std::shared_ptr<DAGNode> conflictedNode = nullptr;
    for (const auto& n : nodes) {
        if (n->GetId() == m_conflictedNodeId) {
            conflictedNode = n;
            break;
        }
        if (auto child = n->GetChildSketch()) {
            if (child->GetId() == m_conflictedNodeId) {
                conflictedNode = child;
                break;
            }
        }
    }

    if (!conflictedNode || !conflictedNode->HasConflict()) {
        m_showConflictPrompt = false;
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 120.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.18f, 0.10f, 0.10f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.35f, 0.35f, 1.0f));

    if (ImGui::Begin("Parametric Relationship Conflict##Modal", &m_showConflictPrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "⚠ Parametric Dependency Broken");
        ImGui::Separator();
        ImGui::TextWrapped("Node '%s' (ID %llu) has an unresolved relationship conflict:", conflictedNode->GetName().c_str(), conflictedNode->GetId());
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", conflictedNode->GetConflictMessage().c_str());
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Options to fix this relationship:");
        ImGui::Spacing();

        if (auto plane = std::dynamic_pointer_cast<DatumPlaneNode>(conflictedNode)) {
            if (plane->IsFaceAssociative()) {
                if (m_selectedFaceIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != plane->GetId()) {
                    std::shared_ptr<DAGNode> selModel = nullptr;
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) { selModel = n; break; }
                    }
                    if (selModel) {
                        std::string reattachLabel = "Re-attach to Selected Face #" + std::to_string(m_selectedFaceIndex) + " on '" + selModel->GetName() + "'";
                        if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 30))) {
                            plane->SetFaceReference(selModel, m_selectedFaceIndex, m_selectedFacePlane);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_showConflictPrompt = false;
                        }
                    }
                } else {
                    ImGui::TextDisabled("(To re-attach: select a face on any 3D model in viewport)");
                }
            } else if (plane->IsVertexAssociative()) {
                if (m_selectedVertexIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != plane->GetId()) {
                    std::shared_ptr<DAGNode> selModel = nullptr;
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) { selModel = n; break; }
                    }
                    if (selModel) {
                        std::string reattachLabel = "Re-attach to Selected Vertex #" + std::to_string(m_selectedVertexIndex) + " on '" + selModel->GetName() + "'";
                        if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 30))) {
                            plane->SetVertexReference(selModel, m_selectedVertexIndex, Geom::Plane::XZ(m_selectedVertexPos));
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_showConflictPrompt = false;
                        }
                    }
                } else {
                    ImGui::TextDisabled("(To re-attach: select a vertex on any 3D model in viewport)");
                }
            }

            if (ImGui::Button("Convert to Fixed Datum Plane (Keep Current 3D Position)", ImVec2(-1, 30))) {
                plane->ConvertToFixed();
                graph.EvaluateDirtyNodes(device, exchange);
                m_showConflictPrompt = false;
            }

            if (ImGui::Button("Delete Datum Plane", ImVec2(-1, 26))) {
                graph.RemoveNode(plane->GetId());
                graph.EvaluateDirtyNodes(device, exchange);
                m_showConflictPrompt = false;
            }
        } else if (auto axis = std::dynamic_pointer_cast<DatumAxisNode>(conflictedNode)) {
            if (m_selectedVertexIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != axis->GetId()) {
                std::shared_ptr<DAGNode> selModel = nullptr;
                for (const auto& n : nodes) {
                    if (n->GetId() == m_selectedNodeId) { selModel = n; break; }
                }
                if (selModel) {
                    std::string reattachLabel = "Re-attach to Selected Vertex #" + std::to_string(m_selectedVertexIndex) + " on '" + selModel->GetName() + "'";
                    if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 30))) {
                        axis->SetVertexReference(selModel, m_selectedVertexIndex);
                        graph.EvaluateDirtyNodes(device, exchange);
                        m_showConflictPrompt = false;
                    }
                }
            } else {
                ImGui::TextDisabled("(To re-attach: select a vertex on any 3D model in viewport)");
            }

            if (ImGui::Button("Convert to Fixed Reference Axis (Keep Current Coordinates)", ImVec2(-1, 30))) {
                axis->ConvertToFixed();
                graph.EvaluateDirtyNodes(device, exchange);
                m_showConflictPrompt = false;
            }

            if (ImGui::Button("Delete Reference Axis", ImVec2(-1, 26))) {
                graph.RemoveNode(axis->GetId());
                graph.EvaluateDirtyNodes(device, exchange);
                m_showConflictPrompt = false;
            }
        } else if (auto sk = std::dynamic_pointer_cast<SketchNode>(conflictedNode)) {
            if (ImGui::Button("Convert to Standalone Sketch (Detach Datum Plane)", ImVec2(-1, 30))) {
                sk->SetPlane(sk->GetPlane());
                graph.EvaluateDirtyNodes(device, exchange);
                m_showConflictPrompt = false;
            }
        }

        ImGui::Spacing();
        if (ImGui::Button("Close", ImVec2(-1, 24))) {
            m_showConflictPrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderEmptySketchPromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange
) {
    if (!m_showEmptySketchExitPrompt) return;

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 220.0f, ImGui::GetIO().DisplaySize.y * 0.5f - 85.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(440.0f, 0.0f), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.14f, 0.16f, 0.20f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.45f, 0.2f, 1.0f));

    if (ImGui::Begin("Empty Sketch Confirmation###EmptySketchModal", &m_showEmptySketchExitPrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.2f, 1.0f), "⚠ Sketch Is Empty");
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextWrapped("This 2D sketch contains no shapes. Exiting will delete this empty sketch from the model tree. Do you wish to delete it?");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.28f, 0.28f, 1.0f));
        if (ImGui::Button("🗑 Delete Sketch", ImVec2(140, 30))) {
            if (m_emptySketchToExitId != 0) {
                graph.RemoveNode(m_emptySketchToExitId);
                graph.EvaluateDirtyNodes(device, exchange);
                m_emptySketchToExitId = 0;
            }
            m_canvasShapes.clear();
            m_sketchModeActive = false;
            m_showEmptySketchExitPrompt = false;
            m_selectedNodeId = 0;
            m_lastPublishedSelectedNodeId = 0;
            graph.PublishCombinedSceneMesh(device, exchange, 0);
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();
        if (ImGui::Button("Keep Editing", ImVec2(130, 30))) {
            m_showEmptySketchExitPrompt = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 30))) {
            m_showEmptySketchExitPrompt = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
}

void CADGui::RenderUI(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    Camera& camera,
    ViewportRenderer& renderer
) {
    if (!m_initialized) return;

    if (m_selectedNodeId != m_lastPublishedSelectedNodeId ||
        m_selectedFaceIndex != m_lastPublishedSelectedFaceIndex ||
        m_selectedVertexIndex != m_lastPublishedSelectedVertexIndex ||
        m_selectedEdgeIndex != m_lastPublishedSelectedEdgeIndex) {
        m_lastPublishedSelectedNodeId = m_selectedNodeId;
        m_lastPublishedSelectedFaceIndex = m_selectedFaceIndex;
        m_lastPublishedSelectedVertexIndex = m_selectedVertexIndex;
        m_lastPublishedSelectedEdgeIndex = m_selectedEdgeIndex;
        graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, m_selectedFaceIndex, m_selectedVertexIndex, m_selectedEdgeIndex);
    }

    auto nodes = graph.GetNodes();
    std::shared_ptr<DAGNode> selectedNode = nullptr;
    std::shared_ptr<DAGNode> anyConflictedNode = nullptr;
    for (const auto& node : nodes) {
        if (node->GetId() == m_selectedNodeId) {
            selectedNode = node;
        }
        if (node->HasConflict() && !anyConflictedNode) {
            anyConflictedNode = node;
        }
        if (auto child = node->GetChildSketch()) {
            if (child->GetId() == m_selectedNodeId) {
                selectedNode = child;
            }
            if (child->HasConflict() && !anyConflictedNode) {
                anyConflictedNode = child;
            }
        }
    }

    // -------------------------------------------------------------------------
    // TOP BAR: Main Feature & Reference Geometry Toolbar
    // -------------------------------------------------------------------------
    RenderTopBar(graph, device, exchange, nodes, selectedNode);

    // Conflict Warning Banner (if any node has broken relationship)
    if (anyConflictedNode) {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 390.0f, vp->WorkPos.y + 60.0f));
        ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x - 402.0f, 42.0f));
        ImGuiWindowFlags bannerFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar;
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.35f, 0.08f, 0.08f, 0.95f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.25f, 0.25f, 1.0f));
        if (ImGui::Begin("##ConflictBanner", nullptr, bannerFlags)) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "⚠ Parametric Conflict: '%s' has a broken relationship!", anyConflictedNode->GetName().c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Resolve Conflict Now")) {
                OpenConflictPrompt(anyConflictedNode->GetId());
            }
        }
        ImGui::End();
        ImGui::PopStyleColor(2);
    }

    // Interactive identification modals for Sketch Planes and Feature References
    RenderSketchPlanePromptModal(graph, device, exchange, nodes);
    RenderExtrudePromptModal(graph, device, exchange, nodes);
    RenderRevolvePromptModal(graph, device, exchange, nodes);
    RenderFilletPromptModal(graph, device, exchange, nodes);
    RenderChamferPromptModal(graph, device, exchange, nodes);
    RenderConflictPromptModal(graph, device, exchange, nodes);
    RenderEmptySketchPromptModal(graph, device, exchange);



    // -------------------------------------------------------------------------
    // PANEL 2: Model Tree & Interactive Property Inspector
    // -------------------------------------------------------------------------
    const ImGuiViewport* mainVp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(12.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380.0f, mainVp->WorkSize.y - 72.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Model Tree & Property Inspector", nullptr)) {
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "CAD Model Tree (%zu Objects)", graph.GetNodeCount());
        ImGui::Separator();

        ImGui::BeginChild("NodeList", ImVec2(0, 180), true);
        for (const auto& node : nodes) {
            // Child sketches are displayed underneath their parent feature in the hierarchy
            if (node->HasParentFeature()) {
                continue;
            }

            auto childSketch = node->GetChildSketch();
            bool isSelected = (node->GetId() == m_selectedNodeId);

            std::string prefix = "[Solid] ";
            if (node->HasConflict()) {
                prefix = "[!] CONFLICT: ";
            } else if (std::dynamic_pointer_cast<SketchNode>(node)) {
                if (isSelected) {
                    prefix = "[Sketch ✦ Outline/Shaded] ";
                } else {
                    prefix = "[Sketch 👁 Hidden] ";
                }
            } else if (auto e = std::dynamic_pointer_cast<ExtrudeNode>(node)) {
                prefix = "[Extrude] ";
            } else if (auto r = std::dynamic_pointer_cast<RevolveNode>(node)) {
                prefix = "[Revolve] ";
            } else if (auto p = std::dynamic_pointer_cast<DatumPlaneNode>(node)) {
                prefix = p->IsFaceAssociative() ? "[DatumPlane (Face Tied)] " : (p->IsVertexAssociative() ? "[DatumPlane (Vertex Tied)] " : "[DatumPlane] ");
            } else if (auto a = std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                prefix = a->IsVertexAssociative() ? "[DatumAxis (Vertex Tied)] " : "[DatumAxis] ";
            } else if (std::dynamic_pointer_cast<FilletNode>(node)) {
                prefix = "[⌒ Fillet] ";
            } else if (std::dynamic_pointer_cast<ChamferNode>(node)) {
                prefix = "[⟍ Chamfer] ";
            }

            std::string label = prefix + node->GetName() + " (ID " + std::to_string(node->GetId()) + ")";

            if (childSketch) {
                ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
                if (isSelected) {
                    flags |= ImGuiTreeNodeFlags_Selected;
                }

                bool nodeOpen = ImGui::TreeNodeEx(reinterpret_cast<void*>(node->GetId()), flags, "%s", label.c_str());
                if (ImGui::IsItemClicked()) {
                    m_selectedNodeId = node->GetId();
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                }
                if (isSelected && m_selectionScrollPending) {
                    ImGui::SetScrollHereY(0.5f);
                }

                if (nodeOpen) {
                    bool isChildSelected = (childSketch->GetId() == m_selectedNodeId);
                    std::string childPrefix = childSketch->HasConflict() ? "↳ [!] CONFLICT: " : "↳ [Sketch] ";
                    std::string childLabel = childPrefix + childSketch->GetName() + " (ID " + std::to_string(childSketch->GetId()) + ")";

                    if (ImGui::Selectable(childLabel.c_str(), isChildSelected)) {
                        m_selectedNodeId = childSketch->GetId();
                        ClearSubElementSelection();
                        graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                    }
                    if (isChildSelected && m_selectionScrollPending) {
                        ImGui::SetScrollHereY(0.5f);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Selectable(label.c_str(), isSelected)) {
                    m_selectedNodeId = node->GetId();
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                }
                if (std::dynamic_pointer_cast<SketchNode>(node) && ImGui::IsItemHovered()) {
                    if (isSelected) {
                        ImGui::SetTooltip("Standalone Sketch is SELECTED:\nRendered in 3D viewport as a vibrant outline with transparent shading.");
                    } else {
                        ImGui::SetTooltip("Standalone Sketch is NOT selected:\nHidden from 3D viewport. Click to select on feature tree and display with outline & transparent shading.");
                    }
                }
                if (isSelected && m_selectionScrollPending) {
                    ImGui::SetScrollHereY(0.5f);
                }
            }
        }
        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0) && !ImGui::IsAnyItemHovered()) {
            m_selectedNodeId = 0; // Deselect on clicking empty background in Model Tree
            ClearSubElementSelection();
            graph.SetSubElementSelection(0, -1, -1);
        }
        m_selectionScrollPending = false;
        ImGui::EndChild();

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Live Feature / Sketch Inspector:");
        ImGui::Separator();

        if (selectedNode) {
            ImGui::Text("Selected: %s (Gen %llu)", selectedNode->GetName().c_str(), selectedNode->GetGeneration());

            if (selectedNode->HasConflict()) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.25f, 0.25f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.28f, 0.08f, 0.08f, 0.95f));
                ImGui::BeginChild("ConflictAlertCard", ImVec2(0, 72), true);
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "⚠ Parametric Conflict Detected!");
                ImGui::TextWrapped("%s", selectedNode->GetConflictMessage().c_str());
                if (ImGui::SmallButton("Resolve Conflict Now...")) {
                    OpenConflictPrompt(selectedNode->GetId());
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            }

            if (m_selectedFaceIndex >= 0) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.0f, 0.85f, 1.0f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.16f, 0.22f, 0.9f));
                ImGui::BeginChild("FaceSubElementCard", ImVec2(0, 145), true);
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "● Sub-Element Selected: Face #%d", m_selectedFaceIndex);
                ImGui::Text("Normal: (%.2f, %.2f, %.2f)", m_selectedFacePlane.normal.x, m_selectedFacePlane.normal.y, m_selectedFacePlane.normal.z);
                ImGui::Text("Center: (%.2f, %.2f, %.2f)", m_selectedFacePlane.origin.x, m_selectedFacePlane.origin.y, m_selectedFacePlane.origin.z);
                ImGui::Spacing();

                if (ImGui::Button("✎ Sketch on this Face", ImVec2(-1, 26))) {
                    std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, selectedNode, m_selectedFaceIndex, m_selectedFacePlane);
                    graph.AddNode(planeNode);

                    std::string sketchName = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                    auto sketch = std::make_shared<SketchNode>(sketchName, planeNode);
                    sketch->SetPosition(static_cast<float>(m_selectedFacePlane.origin.x), static_cast<float>(m_selectedFacePlane.origin.y), static_cast<float>(m_selectedFacePlane.origin.z));
                    graph.AddNode(sketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(sketch->GetId());
                    m_activeSketchId = sketch->GetId();
                    m_sketchModeActive = true;
                }
                if (ImGui::Button("⊞ Create Associative Datum Plane", ImVec2(-1, 24))) {
                    std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter++);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, selectedNode, m_selectedFaceIndex, m_selectedFacePlane);
                    graph.AddNode(planeNode);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(planeNode->GetId());
                }
                if (ImGui::Button("✕ Deselect Face (Select Whole Model)", ImVec2(-1, 22))) {
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            } else if (m_selectedVertexIndex >= 0) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.2f, 0.9f, 0.4f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.20f, 0.12f, 0.9f));
                ImGui::BeginChild("VertexSubElementCard", ImVec2(0, 175), true);
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "● Sub-Element Selected: Vertex #%d", m_selectedVertexIndex);
                ImGui::Text("Position: (%.2f, %.2f, %.2f)", m_selectedVertexPos.x, m_selectedVertexPos.y, m_selectedVertexPos.z);
                ImGui::Spacing();

                if (ImGui::Button("✎ Sketch on this Vertex", ImVec2(-1, 26))) {
                    std::string planeName = "Plane_Vertex_" + std::to_string(m_shapeCounter);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, selectedNode, m_selectedVertexIndex, pl, true);
                    graph.AddNode(planeNode);

                    std::string sketchName = "Sketch_Vertex_" + std::to_string(m_shapeCounter++);
                    auto sketch = std::make_shared<SketchNode>(sketchName, planeNode);
                    sketch->SetPosition(static_cast<float>(m_selectedVertexPos.x), static_cast<float>(m_selectedVertexPos.y), static_cast<float>(m_selectedVertexPos.z));
                    graph.AddNode(sketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(sketch->GetId());
                    m_activeSketchId = sketch->GetId();
                    m_sketchModeActive = true;
                }
                if (ImGui::Button("⊞ Create Associative Datum Plane", ImVec2(-1, 24))) {
                    std::string planeName = "Plane_Vertex_" + std::to_string(m_shapeCounter++);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, selectedNode, m_selectedVertexIndex, pl, true);
                    graph.AddNode(planeNode);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(planeNode->GetId());
                }
                if (ImGui::Button("⌖ Create Associative Datum Axis", ImVec2(-1, 24))) {
                    std::string axisName = "Axis_Vertex_" + std::to_string(m_shapeCounter++);
                    auto axisNode = std::make_shared<DatumAxisNode>(axisName, selectedNode, m_selectedVertexIndex, Geom::Vector3D(0.0, 1.0, 0.0));
                    graph.AddNode(axisNode);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(axisNode->GetId());
                }
                if (ImGui::Button("✕ Deselect Vertex (Select Whole Model)", ImVec2(-1, 22))) {
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            } else if (m_selectedEdgeIndex >= 0) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.75f, 0.2f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.22f, 0.17f, 0.05f, 0.9f));
                ImGui::BeginChild("EdgeSubElementCard", ImVec2(0, 185), true);
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.3f, 1.0f), "● Sub-Element Selected: Corner Edge #%d", m_selectedEdgeIndex);
                float edgeLen = static_cast<float>(m_selectedEdgeP0.distance_to(m_selectedEdgeP1));
                ImGui::Text("Length: %.2f m", edgeLen);
                ImGui::Text("From: (%.2f, %.2f, %.2f)", m_selectedEdgeP0.x, m_selectedEdgeP0.y, m_selectedEdgeP0.z);
                ImGui::Text("To:   (%.2f, %.2f, %.2f)", m_selectedEdgeP1.x, m_selectedEdgeP1.y, m_selectedEdgeP1.z);
                ImGui::Spacing();

                ImGui::SetNextItemWidth(90.0f);
                ImGui::InputFloat("##EdgeFilletR", &m_newFilletRadius, 0.05f, 0.2f, "%.2f m");
                if (m_newFilletRadius < 0.01f) m_newFilletRadius = 0.01f;
                ImGui::SameLine();
                if (ImGui::Button("⌒ Create Fillet", ImVec2(-1, 24))) {
                    std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                    auto fillet = std::make_shared<FilletNode>(name, selectedNode, m_selectedEdgeIndex, m_newFilletRadius);
                    graph.AddNode(fillet);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(fillet->GetId());
                }

                ImGui::SetNextItemWidth(90.0f);
                ImGui::InputFloat("##EdgeChamferD", &m_newChamferDistance, 0.05f, 0.2f, "%.2f m");
                if (m_newChamferDistance < 0.01f) m_newChamferDistance = 0.01f;
                ImGui::SameLine();
                if (ImGui::Button("⟍ Create Chamfer", ImVec2(-1, 24))) {
                    std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                    auto chamfer = std::make_shared<ChamferNode>(name, selectedNode, m_selectedEdgeIndex, m_newChamferDistance);
                    graph.AddNode(chamfer);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(chamfer->GetId());
                }

                if (ImGui::Button("✕ Deselect Edge (Select Whole Model)", ImVec2(-1, 22))) {
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1, -1);
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1, -1);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            }

            float pos[3]{selectedNode->GetPosition()[0], selectedNode->GetPosition()[1], selectedNode->GetPosition()[2]};
            if (ImGui::DragFloat3("Position (X, Y, Z)", pos, 0.1f, -50.0f, 50.0f, "%.2f m")) {
                selectedNode->SetPosition(pos[0], pos[1], pos[2]);
                graph.EvaluateDirtyNodes(device, exchange);
            }
            ImGui::Spacing();

            // -----------------------------------------------------------------
            // A. Sketch Node Inspector
            // -----------------------------------------------------------------
            if (auto sketch = std::dynamic_pointer_cast<SketchNode>(selectedNode)) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "2D Sketch Properties:");
                
                if (!m_sketchModeActive || m_activeSketchId != sketch->GetId()) {
                    if (ImGui::Button("✏ Edit in 2D Mouse Sketch Canvas", ImVec2(-1, 28))) {
                        m_sketchModeActive = true;
                        m_activeSketchId = sketch->GetId();
                    }
                } else {
                    if (ImGui::Button("✓ Close 2D Sketch Canvas", ImVec2(-1, 28))) {
                        if (m_canvasShapes.empty() && sketch->GetShapes().empty()) {
                            m_showEmptySketchExitPrompt = true;
                            m_emptySketchToExitId = sketch->GetId();
                        } else {
                            m_sketchModeActive = false;
                        }
                    }
                }
                ImGui::Spacing();

                int currentPlane = static_cast<int>(sketch->GetPlane());
                const char* planes[] = { "XZ Ground Plane (Top)", "XY Front Plane", "YZ Right Plane" };
                if (ImGui::Combo("Datum Plane", &currentPlane, planes, IM_ARRAYSIZE(planes))) {
                    sketch->SetPlane(static_cast<SketchPlane>(currentPlane));
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (!sketch->HasParentFeature()) {
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.0f, 0.85f, 1.0f, 1.0f), "● 3D Viewport Display:");
                    ImGui::BulletText("Outline with transparent shading (active when selected).");
                    ImGui::BulletText("Deselect in feature tree to hide from 3D viewport.");
                } else if (auto parent = sketch->GetParentFeature()) {
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.5f, 0.8f, 0.5f, 1.0f), "● Associated with 3D Feature:");
                    ImGui::BulletText("Child sketch of: %s", parent->GetName().c_str());
                }

                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.3f, 1.0f), "Profile Presets Quick-Swap:");
                if (ImGui::Button("Rectangle")) {
                    sketch->SetPresetRectangle(3.0f, 2.0f);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::Button("Hexagon")) {
                    sketch->SetPresetRegularPolygon(6, 2.0f);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::Button("L-Bracket")) {
                    sketch->SetPresetLBracket(3.5f, 3.5f, 1.0f);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::Button("Star")) {
                    sketch->SetPresetStar(5, 2.5f, 1.2f);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                // Interactive 2D Shapes & Points Editor
                ImGui::Spacing();
                const auto& shapes = sketch->GetShapes();
                if (shapes.size() > 1) {
                    ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Shapes in Sketch (%zu):", shapes.size());
                    for (size_t s = 0; s < shapes.size(); ++s) {
                        ImGui::PushID(static_cast<int>(s + 1000));
                        ImGui::BulletText("Shape %zu (%zu vertices)", s + 1, shapes[s].size());
                        ImGui::SameLine();
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.2f, 0.2f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.25f, 0.25f, 1.0f));
                        if (ImGui::Button("🗑 Delete Shape")) {
                            sketch->RemoveShape(s);
                            graph.EvaluateDirtyNodes(device, exchange);
                            ImGui::PopStyleColor(2);
                            ImGui::PopID();
                            break;
                        }
                        ImGui::PopStyleColor(2);
                        ImGui::PopID();
                    }
                    ImGui::Spacing();
                }

                const auto& profile = sketch->GetProfile();
                ImGui::Text("Active Profile Vertices (%zu points):", profile.size());

                ImGui::BeginChild("PointEditor", ImVec2(0, 115), true);
                for (size_t i = 0; i < profile.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    float pt[2]{profile[i][0], profile[i][1]};
                    std::string label = "Pt #" + std::to_string(i);
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 36.0f);
                    if (ImGui::DragFloat2(label.c_str(), pt, 0.05f, -20.0f, 20.0f, "%.2f")) {
                        sketch->UpdatePoint(i, pt[0], pt[1]);
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
                    ImGui::SameLine();
                    if (profile.size() > 3) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.2f, 0.2f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.25f, 0.25f, 1.0f));
                        if (ImGui::Button("✕", ImVec2(24, 0))) {
                            sketch->RemovePoint(i);
                            graph.EvaluateDirtyNodes(device, exchange);
                            ImGui::PopStyleColor(2);
                            ImGui::PopID();
                            break;
                        }
                        ImGui::PopStyleColor(2);
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("Delete vertex #%zu", i);
                        }
                    } else {
                        ImGui::BeginDisabled();
                        ImGui::Button("✕", ImVec2(24, 0));
                        ImGui::EndDisabled();
                        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                            ImGui::SetTooltip("A polygon must have at least 3 vertices.");
                        }
                    }
                    ImGui::PopID();
                }
                ImGui::EndChild();

                if (ImGui::Button("+ Add Vertex")) {
                    sketch->AddPoint(m_newPointCoord[0], m_newPointCoord[1]);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::Button("- Delete Last Vertex") && profile.size() > 3) {
                    sketch->RemovePoint(profile.size() - 1);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }
            // -----------------------------------------------------------------
            // B. Extrude Node Inspector
            // -----------------------------------------------------------------
            else if (auto extrude = std::dynamic_pointer_cast<ExtrudeNode>(selectedNode)) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Extrude Feature Properties:");
                
                if (auto refSketch = extrude->GetSketchReference()) {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Linked to Sketch: %s", refSketch->GetName().c_str());
                    if (ImGui::Button("Jump to Reference Sketch", ImVec2(-1, 24))) {
                        m_selectedNodeId = refSketch->GetId();
                    }
                } else {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Standalone Profile (No Sketch Ref)");
                }

                float h = extrude->GetHeight();
                if (ImGui::InputFloat("Extrude Depth", &h, 0.1f, 1.0f, "%.2f m")) {
                    if (std::abs(h) < 0.001f) h = (h < 0.0f ? -0.01f : 0.01f);
                    extrude->SetHeight(h);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }
            // -----------------------------------------------------------------
            // B2. Revolve Node Inspector
            // -----------------------------------------------------------------
            else if (auto revolve = std::dynamic_pointer_cast<RevolveNode>(selectedNode)) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Revolve Feature Properties:");
                
                if (auto refSketch = revolve->GetSketchReference()) {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Linked to Sketch: %s", refSketch->GetName().c_str());
                    if (ImGui::Button("Jump to Reference Sketch", ImVec2(-1, 24))) {
                        m_selectedNodeId = refSketch->GetId();
                    }
                } else {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Standalone Profile (No Sketch Ref)");
                }

                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f), "Cylindrical Model Dimensions:");
                ImGui::BulletText("Wall Thickness: %.2f m (Dictated by Sketch)", revolve->GetWallThickness());
                ImGui::BulletText("Inner Radius:   %.2f m", revolve->GetInnerRadius());
                ImGui::BulletText("Outer Radius:   %.2f m", revolve->GetOuterRadius());
                ImGui::BulletText("Height:         %.2f m", revolve->GetCylinderHeight());
                ImGui::Spacing();

                // Axis selection
                int currentMode = static_cast<int>(revolve->GetAxisMode());
                const char* axisModes[] = {
                    "Sketch V-Axis (Vertical)",
                    "Sketch U-Axis (Horizontal)",
                    "World Y-Axis (0, 1, 0)",
                    "World X-Axis (1, 0, 0)",
                    "World Z-Axis (0, 0, 1)",
                    "Custom Datum Axis"
                };
                if (ImGui::Combo("Revolution Axis", &currentMode, axisModes, IM_ARRAYSIZE(axisModes))) {
                    revolve->SetAxisMode(static_cast<RevolveAxisMode>(currentMode));
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                float ang = revolve->GetAngleDegrees();
                if (ImGui::InputFloat("Revolve Angle", &ang, 1.0f, 15.0f, "%.1f deg")) {
                    ang = std::clamp(ang, 0.1f, 360.0f);
                    revolve->SetAngleDegrees(ang);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                int segs = revolve->GetAngularSegments();
                if (ImGui::InputInt("Angular Slices", &segs, 1, 4)) {
                    segs = std::clamp(segs, 8, 96);
                    revolve->SetAngularSegments(segs);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                bool solid = revolve->IsSolid();
                if (ImGui::Checkbox("Solid Body (Caps)", &solid)) {
                    revolve->SetSolid(solid);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }
            // -----------------------------------------------------------------
            // C. Primitive Nodes (Cylinder, Sphere)
            // -----------------------------------------------------------------
            else if (auto cyl = std::dynamic_pointer_cast<CylinderNode>(selectedNode)) {
                float r = cyl->GetRadius();
                float h = cyl->GetHeight();
                int segs = static_cast<int>(cyl->GetSegments());
                bool changed = false;

                changed |= ImGui::InputFloat("Radius",   &r, 0.1f, 1.0f, "%.2f m");
                changed |= ImGui::InputFloat("Height",   &h, 0.1f, 1.0f, "%.2f m");
                changed |= ImGui::InputInt("Segments",   &segs, 1, 4);

                if (changed) {
                    r = std::max(0.01f, r);
                    h = std::max(0.01f, h);
                    segs = std::clamp(segs, 8, 96);
                    cyl->SetParameters(r, h, segs);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            } else if (auto sphere = std::dynamic_pointer_cast<SphereNode>(selectedNode)) {
                float r = sphere->GetRadius();
                int rings = static_cast<int>(sphere->GetRings());
                int sectors = static_cast<int>(sphere->GetSectors());
                bool changed = false;

                changed |= ImGui::InputFloat("Radius",  &r, 0.1f, 1.0f, "%.2f m");
                changed |= ImGui::InputInt("Rings",    &rings, 1, 4);
                changed |= ImGui::InputInt("Sectors",  &sectors, 1, 4);

                if (changed) {
                    r = std::max(0.01f, r);
                    rings = std::clamp(rings, 6, 48);
                    sectors = std::clamp(sectors, 8, 64);
                    sphere->SetParameters(r, rings, sectors);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            } else if (auto planeNode = std::dynamic_pointer_cast<DatumPlaneNode>(selectedNode)) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Datum Plane Inspector");
                ImGui::Separator();

                if (planeNode->IsFaceAssociative()) {
                    if (auto parent = planeNode->GetReferencedFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Associative Link: Face #%d on '%s'", planeNode->GetReferencedFaceIndex(), parent->GetName().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Associative Link: Feature MISSING!");
                    }
                    if (planeNode->HasConflict()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[!] CONFLICT: %s", planeNode->GetConflictMessage().c_str());
                        if (ImGui::Button("Resolve Relationship Conflict...", ImVec2(-1, 28))) {
                            OpenConflictPrompt(planeNode->GetId());
                        }
                    } else {
                        if (ImGui::Button("Break Link (Convert to Fixed Plane)", ImVec2(-1, 24))) {
                            planeNode->ConvertToFixed();
                            graph.EvaluateDirtyNodes(device, exchange);
                        }
                    }
                } else if (planeNode->IsVertexAssociative()) {
                    if (auto parent = planeNode->GetReferencedFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Associative Link: Vertex #%d on '%s'", planeNode->GetReferencedVertexIndex(), parent->GetName().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Associative Link: Feature MISSING!");
                    }
                    if (planeNode->HasConflict()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[!] CONFLICT: %s", planeNode->GetConflictMessage().c_str());
                        if (ImGui::Button("Resolve Relationship Conflict...", ImVec2(-1, 28))) {
                            OpenConflictPrompt(planeNode->GetId());
                        }
                    } else {
                        if (ImGui::Button("Break Link (Convert to Fixed Plane)", ImVec2(-1, 24))) {
                            planeNode->ConvertToFixed();
                            graph.EvaluateDirtyNodes(device, exchange);
                        }
                    }
                } else {
                    ImGui::TextDisabled("Type: Fixed Datum Plane");
                }

                Geom::Plane pl = planeNode->GetPlane();
                ImGui::Text("Plane Normal: (%.2f, %.2f, %.2f)", pl.normal.x, pl.normal.y, pl.normal.z);
                ImGui::Text("Plane Origin: (%.2f, %.2f, %.2f)", pl.origin.x, pl.origin.y, pl.origin.z);

                float offset = static_cast<float>(planeNode->GetOffset());
                if (ImGui::InputFloat("Plane Offset", &offset, 0.1f, 1.0f, "%.2f m")) {
                    planeNode->SetOffset(offset);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                ImGui::Spacing();
                if (ImGui::Button("✎ Create 2D Sketch on this Plane", ImVec2(-1, 30))) {
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    m_activeSketchId = newSketch->GetId();
                    m_sketchModeActive = true;
                }
            } else if (auto axisNode = std::dynamic_pointer_cast<DatumAxisNode>(selectedNode)) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Datum Axis Inspector");
                ImGui::Separator();

                if (axisNode->IsVertexAssociative()) {
                    if (auto parent = axisNode->GetReferencedFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Associative Link: Vertex #%d on '%s'", axisNode->GetReferencedVertexIndex(), parent->GetName().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Associative Link: Feature MISSING!");
                    }
                    if (axisNode->HasConflict()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[!] CONFLICT: %s", axisNode->GetConflictMessage().c_str());
                        if (ImGui::Button("Resolve Relationship Conflict...", ImVec2(-1, 28))) {
                            OpenConflictPrompt(axisNode->GetId());
                        }
                    } else {
                        if (ImGui::Button("Break Link (Convert to Fixed Axis)", ImVec2(-1, 24))) {
                            axisNode->ConvertToFixed();
                            graph.EvaluateDirtyNodes(device, exchange);
                        }
                    }
                } else {
                    ImGui::TextDisabled("Type: Fixed Reference Axis");
                }

                Geom::Axis3D ax = axisNode->GetAxis();
                ImGui::Text("Axis Direction: (%.2f, %.2f, %.2f)", ax.direction.x, ax.direction.y, ax.direction.z);
                ImGui::Text("Axis Origin:    (%.2f, %.2f, %.2f)", ax.origin.x, ax.origin.y, ax.origin.z);

                int axisPreset = (std::abs(ax.direction.x) > 0.8) ? 0 :
                                 (std::abs(ax.direction.z) > 0.8) ? 2 : 1;
                const char* axes[] = { "X-Axis (Horizontal)", "Y-Axis (Vertical Up)", "Z-Axis (Depth)" };
                if (ImGui::Combo("Axis Alignment", &axisPreset, axes, IM_ARRAYSIZE(axes))) {
                    Geom::Axis3D newAx = (axisPreset == 0) ? Geom::Axis3D::X(ax.origin) :
                                         (axisPreset == 2) ? Geom::Axis3D::Z(ax.origin) :
                                                             Geom::Axis3D::Y(ax.origin);
                    axisNode->SetAxis(newAx);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            } else if (auto fillet = std::dynamic_pointer_cast<FilletNode>(selectedNode)) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "⌒ Fillet Feature Properties:");
                ImGui::Separator();

                if (auto target = fillet->GetTargetFeature()) {
                    ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Linked to Model: '%s' (Edge #%d)", target->GetName().c_str(), fillet->GetReferencedEdgeIndex());
                    if (ImGui::Button("Jump to Target Model", ImVec2(-1, 24))) {
                        SetSelectedNodeId(target->GetId());
                    }
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Target Model: MISSING / DELETED!");
                }

                if (fillet->HasConflict()) {
                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[!] CONFLICT: %s", fillet->GetConflictMessage().c_str());
                }

                float r = fillet->GetRadius();
                if (ImGui::InputFloat("Fillet Radius", &r, 0.05f, 0.2f, "%.2f m")) {
                    fillet->SetRadius(r);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (m_selectedEdgeIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != fillet->GetId()) {
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) {
                            std::string reattachLabel = "Re-attach Fillet to Selected Edge #" + std::to_string(m_selectedEdgeIndex) + " on '" + n->GetName() + "'";
                            if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 26))) {
                                fillet->SetEdgeReference(n, m_selectedEdgeIndex);
                                graph.EvaluateDirtyNodes(device, exchange);
                            }
                            break;
                        }
                    }
                }
            } else if (auto chamfer = std::dynamic_pointer_cast<ChamferNode>(selectedNode)) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.75f, 0.55f, 1.0f, 1.0f), "⟍ Chamfer Feature Properties:");
                ImGui::Separator();

                if (auto target = chamfer->GetTargetFeature()) {
                    ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Linked to Model: '%s' (Edge #%d)", target->GetName().c_str(), chamfer->GetReferencedEdgeIndex());
                    if (ImGui::Button("Jump to Target Model", ImVec2(-1, 24))) {
                        SetSelectedNodeId(target->GetId());
                    }
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Target Model: MISSING / DELETED!");
                }

                if (chamfer->HasConflict()) {
                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[!] CONFLICT: %s", chamfer->GetConflictMessage().c_str());
                }

                float d = chamfer->GetDistance();
                if (ImGui::InputFloat("Chamfer Distance", &d, 0.05f, 0.2f, "%.2f m")) {
                    chamfer->SetDistance(d);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (m_selectedEdgeIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != chamfer->GetId()) {
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) {
                            std::string reattachLabel = "Re-attach Chamfer to Selected Edge #" + std::to_string(m_selectedEdgeIndex) + " on '" + n->GetName() + "'";
                            if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 26))) {
                                chamfer->SetEdgeReference(n, m_selectedEdgeIndex);
                                graph.EvaluateDirtyNodes(device, exchange);
                            }
                            break;
                        }
                    }
                }
            }

            ImGui::Spacing();
            ImGui::Separator();

            bool isChildSketch = false;
            if (auto sketch = std::dynamic_pointer_cast<SketchNode>(selectedNode)) {
                if (sketch->HasParentFeature()) {
                    isChildSketch = true;
                    if (auto parent = sketch->GetParentFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Child of feature: '%s'", parent->GetName().c_str());
                    }
                    ImGui::TextDisabled("(Deleting the parent feature will promote this sketch)");
                }
            }

            std::string delBtnText = "Delete Selected Object";
            if (selectedNode->GetChildSketch()) {
                delBtnText = "🗑 Delete Feature (Promotes Child Sketch)";
            } else if (isChildSketch) {
                delBtnText = "";
            } else if (std::dynamic_pointer_cast<SketchNode>(selectedNode)) {
                delBtnText = "🗑 Delete 2D Sketch";
            } else {
                delBtnText = "🗑 Delete Feature";
            }

            if (!delBtnText.empty()) {
                if (ImGui::Button(delBtnText.c_str(), ImVec2(-1, 28))) {
                    uint64_t delId = selectedNode->GetId();
                    auto child = selectedNode->GetChildSketch();
                    graph.RemoveNode(delId);
                    graph.EvaluateDirtyNodes(device, exchange);
                    if (child) {
                        m_selectedNodeId = child->GetId();
                    } else {
                        m_selectedNodeId = 0;
                    }
                    m_lastPublishedSelectedNodeId = m_selectedNodeId;
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                }
            }

            if (!m_sketchModeActive && !ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
                if (!isChildSketch) {
                    uint64_t delId = selectedNode->GetId();
                    auto child = selectedNode->GetChildSketch();
                    graph.RemoveNode(delId);
                    graph.EvaluateDirtyNodes(device, exchange);
                    if (child) {
                        m_selectedNodeId = child->GetId();
                    } else {
                        m_selectedNodeId = 0;
                    }
                    m_lastPublishedSelectedNodeId = m_selectedNodeId;
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                }
            }
        } else {
            ImGui::TextDisabled("Select an object or sketch from the Model Tree above to inspect.");
        }
    }
    ImGui::End();

    // -------------------------------------------------------------------------
    // PANEL 3: CAD Viewport Controls & HUD
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 360.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(344.0f, 320.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("CAD Viewport & Performance HUD", nullptr)) {
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Camera Presets");
        ImGui::Separator();

        if (ImGui::Button("Top View", ImVec2(76, 26))) {
            camera.Orbit(0.0f, 1.55f);
        }
        ImGui::SameLine();
        if (ImGui::Button("Front View", ImVec2(76, 26))) {
            camera.Orbit(0.0f, 0.0f);
        }
        ImGui::SameLine();
        if (ImGui::Button("Right View", ImVec2(76, 26))) {
            camera.Orbit(1.5707f, 0.0f);
        }
        ImGui::SameLine();
        if (ImGui::Button("Isometric", ImVec2(76, 26))) {
            camera.Orbit(0.785f, 0.523f);
        }

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Display Options");
        ImGui::Separator();
        ImGui::Checkbox("Sharp Feature Edges / Wireframe", &m_showWireframe);
        ImGui::Checkbox("Infinite Datum Ground Grid", &m_showGrid);

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Engine Telemetry (Decoupled)");
        ImGui::Separator();

        double frameTimeMs = renderer.GetAverageFrameTimeMs();
        double fps = (frameTimeMs > 0.0) ? (1000.0 / frameTimeMs) : 0.0;

        ImGui::Text("Render Cadence: %.2f ms (%.1f FPS)", frameTimeMs, fps);
        ImGui::Text("Render Frames:  %llu", renderer.GetRenderedFrames());
        ImGui::Text("Active Gen ID:  %llu", exchange.GetPublishedGeneration());
        
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.4f, 1.0f), "[Render: 120Hz Unblocked] [Worker: Active]");
    }
    ImGui::End();

    // -------------------------------------------------------------------------
    // PANEL 4: 2D Sketch Interactive Canvas (When Active)
    // -------------------------------------------------------------------------
    if (m_sketchModeActive) {
        std::shared_ptr<SketchNode> activeSketch = nullptr;
        for (const auto& n : graph.GetNodes()) {
            if (n->GetId() == m_activeSketchId) {
                activeSketch = std::dynamic_pointer_cast<SketchNode>(n);
                break;
            }
        }
        if (!activeSketch) {
            // Fallback: pick any sketch if m_activeSketchId wasn't found
            for (const auto& n : graph.GetNodes()) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    activeSketch = s;
                    m_activeSketchId = s->GetId();
                    break;
                }
            }
        }

        if (activeSketch) {
            RenderSketchCanvas(graph, device, exchange, activeSketch);
        } else {
            m_sketchModeActive = false;
        }
    }
}

// -----------------------------------------------------------------------------
// Catmull-Rom Spline Evaluator
// -----------------------------------------------------------------------------
static std::vector<std::array<float, 2>> EvaluateCatmullRomSpline(
    const std::vector<std::array<float, 2>>& controls, 
    int samplesPerSegment = 12
) {
    if (controls.size() < 3) return controls;
    std::vector<std::array<float, 2>> result;
    size_t n = controls.size();
    result.reserve(n * samplesPerSegment);

    for (size_t i = 0; i < n; ++i) {
        const auto& p0 = controls[(i + n - 1) % n];
        const auto& p1 = controls[i];
        const auto& p2 = controls[(i + 1) % n];
        const auto& p3 = controls[(i + 2) % n];

        for (int s = 0; s < samplesPerSegment; ++s) {
            float t = static_cast<float>(s) / static_cast<float>(samplesPerSegment);
            float t2 = t * t;
            float t3 = t2 * t;

            float u = 0.5f * (
                (2.0f * p1[0]) +
                (-p0[0] + p2[0]) * t +
                (2.0f * p0[0] - 5.0f * p1[0] + 4.0f * p2[0] - p3[0]) * t2 +
                (-p0[0] + 3.0f * p1[0] - 3.0f * p2[0] + p3[0]) * t3
            );
            float v = 0.5f * (
                (2.0f * p1[1]) +
                (-p0[1] + p2[1]) * t +
                (2.0f * p0[1] - 5.0f * p1[1] + 4.0f * p2[1] - p3[1]) * t2 +
                (-p0[1] + 3.0f * p1[1] - 3.0f * p2[1] + p3[1]) * t3
            );
            result.push_back({u, v});
        }
    }
    return result;
}

// -----------------------------------------------------------------------------
// Interactive 2D Mouse Sketch Canvas
// -----------------------------------------------------------------------------
void CADGui::RenderSketchCanvas(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    std::shared_ptr<SketchNode> activeSketch
) {
    if (!activeSketch) return;

    // -------------------------------------------------------------------------
    // Synchronize Canvas Shapes from Active Sketch Node
    // -------------------------------------------------------------------------
    if (m_currentEditingSketchId != activeSketch->GetId()) {
        m_currentEditingSketchId = activeSketch->GetId();
        m_canvasShapes.clear();
        m_inProgressPoints.clear();
        m_inProgressSplineControls.clear();
        m_draggedShapeIdx = -1;
        m_draggedVertexIdx = -1;
        m_hoveredShapeIdx = -1;
        m_hoveredVertexIdx = -1;

        const auto& shapes = activeSketch->GetShapes();
        if (!shapes.empty()) {
            for (size_t i = 0; i < shapes.size(); ++i) {
                m_canvasShapes.push_back({
                    "Shape " + std::to_string(i + 1),
                    shapes[i],
                    true
                });
            }
        } else if (!activeSketch->GetProfile().empty()) {
            m_canvasShapes.push_back({
                "Shape 1",
                activeSketch->GetProfile(),
                true
            });
        }
    }

    // -------------------------------------------------------------------------
    // Evaluate Sketch Closure
    // -------------------------------------------------------------------------
    bool isSketchClosed = true;
    std::string unclosedReason;

    if (!m_inProgressPoints.empty()) {
        isSketchClosed = false;
        unclosedReason = "Line/Polyline drawing in progress is not closed.";
    } else if (!m_inProgressSplineControls.empty()) {
        isSketchClosed = false;
        unclosedReason = "Spline drawing in progress is not closed.";
    } else if (m_canvasShapes.empty()) {
        isSketchClosed = false;
        unclosedReason = "Sketch has no shapes yet. Draw an enclosed shape to confirm.";
    } else {
        for (size_t i = 0; i < m_canvasShapes.size(); ++i) {
            if (!m_canvasShapes[i].isClosed || m_canvasShapes[i].points.size() < 3) {
                isSketchClosed = false;
                unclosedReason = m_canvasShapes[i].name + " is open (not a closed loop).";
                break;
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(390.0f, 16.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(840.0f, 660.0f), ImGuiCond_FirstUseEver);

    std::string winTitle = "2D Sketch Canvas - " + activeSketch->GetName() + " (" + GetSketchPlaneName(activeSketch->GetPlane()) + ")###SketchCanvas";
    bool isOpen = true;
    if (!ImGui::Begin(winTitle.c_str(), &isOpen)) {
        if (!isOpen) {
            if (m_canvasShapes.empty() && activeSketch->GetShapes().empty()) {
                m_showEmptySketchExitPrompt = true;
                m_emptySketchToExitId = activeSketch->GetId();
            } else {
                m_sketchModeActive = false;
            }
        }
        ImGui::End();
        return;
    }
    if (!isOpen) {
        if (m_canvasShapes.empty() && activeSketch->GetShapes().empty()) {
            m_showEmptySketchExitPrompt = true;
            m_emptySketchToExitId = activeSketch->GetId();
        } else {
            m_sketchModeActive = false;
        }
        ImGui::End();
        return;
    }

    // -------------------------------------------------------------------------
    // Top Tool Ribbon
    // -------------------------------------------------------------------------
    auto ToolBtn = [&](SketchMouseTool tool, const char* label, const char* tooltip) {
        bool isSel = (m_mouseTool == tool);
        if (isSel) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.58f, 0.95f, 1.0f));
        }
        if (ImGui::Button(label, ImVec2(80, 28))) {
            if (m_mouseTool != tool) {
                m_mouseTool = tool;
                m_isDrawing = false;
                m_inProgressPoints.clear();
                m_inProgressSplineControls.clear();
                m_draggedShapeIdx = -1;
                m_draggedVertexIdx = -1;
            }
        }
        if (isSel) {
            ImGui::PopStyleColor(2);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s", tooltip);
        }
    };

    ToolBtn(SketchMouseTool::SelectMove, "Select/Move", "Select & drag vertices to reshape (V)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Rectangle, "Rectangle", "Click and drag diagonally to draw rectangle (R)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Circle, "Circle", "Click center and drag outward to draw circle (C)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Line, "Line/Poly", "Click vertices to draw polyline, click first point / double click to close (L)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Spline, "Spline", "Click control points for smooth spline, click first / double click to close (S)");
    ImGui::SameLine();

    // Stitching button
    if (ImGui::Button("⚡ Stitch", ImVec2(76, 28))) {
        if (m_canvasShapes.size() > 1) {
            std::vector<std::vector<std::array<float, 2>>> loops;
            for (const auto& sh : m_canvasShapes) {
                if (sh.isClosed && sh.points.size() >= 3) {
                    loops.push_back(sh.points);
                }
            }
            if (loops.size() > 1) {
                auto stitched = Sketch::StitchMultipleShapes(loops);
                if (stitched.size() >= 3) {
                    m_canvasShapes.clear();
                    m_canvasShapes.push_back({ "Stitched Boundary", std::move(stitched), true });
                }
            }
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Boolean stitch & union overlapping shapes into a single closed boundary profile");
    }
    ImGui::SameLine();

    ImGui::Checkbox("Snap", &m_gridSnapEnable);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(76.0f);
    ImGui::InputFloat("##Grid", &m_gridSnapSize, 0.1f, 0.5f, "%.1fm");
    if (m_gridSnapSize < 0.05f) m_gridSnapSize = 0.05f;
    ImGui::SameLine();

    if (ImGui::Button("Center", ImVec2(56, 28))) {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        m_canvasPanX = avail.x * 0.5f;
        m_canvasPanY = avail.y * 0.5f;
        m_canvasZoom = 35.0f;
    }
    ImGui::SameLine();

    ImGui::Checkbox("👁 Ref Feature", &m_showReferenceFeature);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Project 3D feature edges and vertices onto 2D sketch canvas as reference geometry");
    }
    ImGui::SameLine();

    // Delete controls in ribbon for selected vertex or shape
    if (m_selectedShapeIdx >= 0 && m_selectedShapeIdx < static_cast<int>(m_canvasShapes.size())) {
        auto& selShape = m_canvasShapes[m_selectedShapeIdx];
        if (m_selectedVertexIdx >= 0 && m_selectedVertexIdx < static_cast<int>(selShape.points.size())) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.28f, 0.28f, 1.0f));
            if (ImGui::Button("🗑 Del Vertex", ImVec2(96, 28))) {
                selShape.points.erase(selShape.points.begin() + m_selectedVertexIdx);
                m_selectedVertexIdx = -1;
                if (selShape.points.size() < 3) selShape.isClosed = false;
                if (selShape.points.empty()) {
                    m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                    m_selectedShapeIdx = -1;
                }
            }
            ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Delete selected vertex handle (Del)");
            ImGui::SameLine();

            ImGui::SetNextItemWidth(70.0f);
            ImGui::InputFloat("##SkFilletR", &m_sketchFilletRadius, 0.05f, 0.1f, "%.2fm");
            if (m_sketchFilletRadius < 0.01f) m_sketchFilletRadius = 0.01f;
            ImGui::SameLine();
            if (ImGui::Button("⌒ Fillet", ImVec2(66, 28))) {
                if (Sketch::ApplyFillet2D(selShape.points, static_cast<size_t>(m_selectedVertexIdx), m_sketchFilletRadius, selShape.isClosed, 6)) {
                    m_selectedVertexIdx = -1;
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply round fillet arc of radius %.2fm to selected corner vertex", m_sketchFilletRadius);
            ImGui::SameLine();

            ImGui::SetNextItemWidth(70.0f);
            ImGui::InputFloat("##SkChamferD", &m_sketchChamferDist, 0.05f, 0.1f, "%.2fm");
            if (m_sketchChamferDist < 0.01f) m_sketchChamferDist = 0.01f;
            ImGui::SameLine();
            if (ImGui::Button("⟍ Chamfer", ImVec2(78, 28))) {
                if (Sketch::ApplyChamfer2D(selShape.points, static_cast<size_t>(m_selectedVertexIdx), m_sketchChamferDist, selShape.isClosed)) {
                    m_selectedVertexIdx = -1;
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply beveled chamfer of distance %.2fm to selected corner vertex", m_sketchChamferDist);
            ImGui::SameLine();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.28f, 0.28f, 1.0f));
            if (ImGui::Button("🗑 Del Shape", ImVec2(92, 28))) {
                m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                m_selectedShapeIdx = -1;
                m_selectedVertexIdx = -1;
            }
            ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Delete selected shape (Del)");
            ImGui::SameLine();

            ImGui::SetNextItemWidth(70.0f);
            ImGui::InputFloat("##AllFilletR", &m_sketchFilletRadius, 0.05f, 0.1f, "%.2fm");
            if (m_sketchFilletRadius < 0.01f) m_sketchFilletRadius = 0.01f;
            ImGui::SameLine();
            if (ImGui::Button("⌒ Fillet All", ImVec2(80, 28))) {
                size_t origSize = selShape.points.size();
                for (int c = static_cast<int>(origSize) - 1; c >= 0; --c) {
                    Sketch::ApplyFillet2D(selShape.points, static_cast<size_t>(c), m_sketchFilletRadius, selShape.isClosed, 6);
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply fillet arc to all corners of this shape");
            ImGui::SameLine();

            ImGui::SetNextItemWidth(70.0f);
            ImGui::InputFloat("##AllChamferD", &m_sketchChamferDist, 0.05f, 0.1f, "%.2fm");
            if (m_sketchChamferDist < 0.01f) m_sketchChamferDist = 0.01f;
            ImGui::SameLine();
            if (ImGui::Button("⟍ Chamfer All", ImVec2(92, 28))) {
                size_t origSize = selShape.points.size();
                for (int c = static_cast<int>(origSize) - 1; c >= 0; --c) {
                    Sketch::ApplyChamfer2D(selShape.points, static_cast<size_t>(c), m_sketchChamferDist, selShape.isClosed);
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply beveled chamfer to all corners of this shape");
            ImGui::SameLine();
        }
    }

    if (!m_canvasShapes.empty()) {
        if (ImGui::Button("Clear All", ImVec2(70, 28))) {
            m_canvasShapes.clear();
            m_selectedShapeIdx = -1;
            m_selectedVertexIdx = -1;
            m_inProgressPoints.clear();
            m_inProgressSplineControls.clear();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Clear all shapes from sketch canvas");
        ImGui::SameLine();
    }

    // Closure Status Badge
    if (isSketchClosed) {
        ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.4f, 1.0f), "● Closed Loop(s)");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.25f, 1.0f), "▲ Unclosed");
    }
    ImGui::SameLine();

    // Quick loop close button if in-progress has points
    if (!m_inProgressPoints.empty() && m_inProgressPoints.size() >= 3) {
        if (ImGui::Button("Close Loop", ImVec2(80, 28))) {
            m_canvasShapes.push_back({
                "Polyline " + std::to_string(m_canvasShapes.size() + 1),
                m_inProgressPoints,
                true
            });
            m_inProgressPoints.clear();
        }
        ImGui::SameLine();
    } else if (!m_inProgressSplineControls.empty() && m_inProgressSplineControls.size() >= 3) {
        if (ImGui::Button("Close Spline", ImVec2(84, 28))) {
            auto denseSpline = EvaluateCatmullRomSpline(m_inProgressSplineControls, 12);
            m_canvasShapes.push_back({
                "Spline " + std::to_string(m_canvasShapes.size() + 1),
                std::move(denseSpline),
                true
            });
            m_inProgressSplineControls.clear();
        }
        ImGui::SameLine();
    }

    // Confirmation Button: Disabled when unclosed!
    ImGui::BeginDisabled(!isSketchClosed);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.62f, 0.35f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.72f, 0.42f, 1.0f));
    if (ImGui::Button("✓ Confirm Changes", ImVec2(130, 28))) {
        std::vector<std::vector<std::array<float, 2>>> outShapes;
        for (const auto& sh : m_canvasShapes) {
            if (sh.isClosed && sh.points.size() >= 3) {
                outShapes.push_back(sh.points);
            }
        }
        activeSketch->SetShapes(std::move(outShapes));
        graph.EvaluateDirtyNodes(device, exchange);
        m_sketchModeActive = false;
    }
    ImGui::PopStyleColor(2);
    ImGui::EndDisabled();

    if (!isSketchClosed && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Cannot confirm sketch:\n%s\nAll shapes must be closed loops to apply.", unclosedReason.c_str());
    }
    ImGui::SameLine();

    if (ImGui::Button("✕ Cancel", ImVec2(65, 28))) {
        if (m_canvasShapes.empty() && activeSketch->GetShapes().empty()) {
            m_showEmptySketchExitPrompt = true;
            m_emptySketchToExitId = activeSketch->GetId();
        } else {
            m_sketchModeActive = false;
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Discard changes and close sketch canvas.");
    }

    ImGui::Separator();

    // -------------------------------------------------------------------------
    // Canvas Area
    // -------------------------------------------------------------------------
    ImVec2 canvasSz = ImGui::GetContentRegionAvail();
    if (canvasSz.x < 150.0f) canvasSz.x = 150.0f;
    if (canvasSz.y < 150.0f) canvasSz.y = 150.0f;

    ImVec2 canvasP0 = ImGui::GetCursorScreenPos();
    ImVec2 canvasP1 = ImVec2(canvasP0.x + canvasSz.x, canvasP0.y + canvasSz.y);

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(canvasP0, canvasP1, IM_COL32(20, 22, 26, 255));
    drawList->AddRect(canvasP0, canvasP1, IM_COL32(50, 56, 68, 255), 0.0f, 0, 1.5f);

    ImGui::InvisibleButton("SketchCanvasInteractiveArea", canvasSz, 
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    bool isHovered = ImGui::IsItemHovered();
    ImGuiIO& io = ImGui::GetIO();

    // Canvas navigation: Pan & Zoom
    if (isHovered) {
        if (ImGui::IsKeyPressed(ImGuiKey_V)) m_mouseTool = SketchMouseTool::SelectMove;
        if (ImGui::IsKeyPressed(ImGuiKey_R)) m_mouseTool = SketchMouseTool::Rectangle;
        if (ImGui::IsKeyPressed(ImGuiKey_C)) m_mouseTool = SketchMouseTool::Circle;
        if (ImGui::IsKeyPressed(ImGuiKey_L)) m_mouseTool = SketchMouseTool::Line;
        if (ImGui::IsKeyPressed(ImGuiKey_S)) m_mouseTool = SketchMouseTool::Spline;

        // Delete / Backspace key to delete selected vertex or selected shape
        if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
            if (m_selectedShapeIdx >= 0 && m_selectedShapeIdx < static_cast<int>(m_canvasShapes.size())) {
                auto& selShape = m_canvasShapes[m_selectedShapeIdx];
                if (m_selectedVertexIdx >= 0 && m_selectedVertexIdx < static_cast<int>(selShape.points.size())) {
                    selShape.points.erase(selShape.points.begin() + m_selectedVertexIdx);
                    m_selectedVertexIdx = -1;
                    if (selShape.points.size() < 3) selShape.isClosed = false;
                    if (selShape.points.empty()) {
                        m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                        m_selectedShapeIdx = -1;
                    }
                } else {
                    m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                    m_selectedShapeIdx = -1;
                    m_selectedVertexIdx = -1;
                }
            }
        }

        if (io.MouseWheel != 0.0f) {
            float oldZoom = m_canvasZoom;
            float factor = (io.MouseWheel > 0.0f) ? 1.15f : (1.0f / 1.15f);
            float newZoom = std::clamp(m_canvasZoom * factor, 5.0f, 300.0f);

            float mouseU = (io.MousePos.x - canvasP0.x - m_canvasPanX) / oldZoom;
            float mouseV = -(io.MousePos.y - canvasP0.y - m_canvasPanY) / oldZoom;

            m_canvasZoom = newZoom;
            m_canvasPanX = io.MousePos.x - canvasP0.x - mouseU * newZoom;
            m_canvasPanY = io.MousePos.y - canvasP0.y + mouseV * newZoom;
        }

        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
            m_canvasPanX += io.MouseDelta.x;
            m_canvasPanY += io.MouseDelta.y;
        }
    }

    // Transforms
    auto ToScreen = [&](float u, float v) -> ImVec2 {
        return ImVec2(canvasP0.x + m_canvasPanX + u * m_canvasZoom,
                      canvasP0.y + m_canvasPanY - v * m_canvasZoom);
    };

    drawList->PushClipRect(canvasP0, canvasP1, true);

    // Grid rendering
    float uMin = -m_canvasPanX / m_canvasZoom;
    float uMax = (canvasSz.x - m_canvasPanX) / m_canvasZoom;
    float vMax = m_canvasPanY / m_canvasZoom;
    float vMin = -(canvasSz.y - m_canvasPanY) / m_canvasZoom;

    float gridSpacing = (m_gridSnapEnable && m_gridSnapSize > 0.01f) ? m_gridSnapSize : 1.0f;
    while (gridSpacing * m_canvasZoom < 24.0f) {
        gridSpacing *= 2.0f;
    }

    float startU = std::floor(uMin / gridSpacing) * gridSpacing;
    for (float u = startU; u <= uMax; u += gridSpacing) {
        float scrX = canvasP0.x + m_canvasPanX + u * m_canvasZoom;
        drawList->AddLine(ImVec2(scrX, canvasP0.y), ImVec2(scrX, canvasP1.y), IM_COL32(36, 40, 48, 255), 1.0f);
    }

    float startV = std::floor(vMin / gridSpacing) * gridSpacing;
    for (float v = startV; v <= vMax; v += gridSpacing) {
        float scrY = canvasP0.y + m_canvasPanY - v * m_canvasZoom;
        drawList->AddLine(ImVec2(canvasP0.x, scrY), ImVec2(canvasP1.x, scrY), IM_COL32(36, 40, 48, 255), 1.0f);
    }

    // Datum Axes: U = Red, V = Green
    float originScrX = canvasP0.x + m_canvasPanX;
    float originScrY = canvasP0.y + m_canvasPanY;

    if (originScrY >= canvasP0.y && originScrY <= canvasP1.y) {
        drawList->AddLine(ImVec2(canvasP0.x, originScrY), ImVec2(canvasP1.x, originScrY), IM_COL32(220, 60, 60, 220), 2.0f);
        drawList->AddText(ImVec2(canvasP1.x - 36.0f, originScrY - 18.0f), IM_COL32(230, 80, 80, 255), "+U");
    }
    if (originScrX >= canvasP0.x && originScrX <= canvasP1.x) {
        drawList->AddLine(ImVec2(originScrX, canvasP0.y), ImVec2(originScrX, canvasP1.y), IM_COL32(60, 220, 80, 220), 2.0f);
        drawList->AddText(ImVec2(originScrX + 6.0f, canvasP0.y + 6.0f), IM_COL32(80, 230, 80, 255), "+V");
    }
    if (originScrX >= canvasP0.x && originScrX <= canvasP1.x && originScrY >= canvasP0.y && originScrY <= canvasP1.y) {
        drawList->AddCircleFilled(ImVec2(originScrX, originScrY), 4.5f, IM_COL32(255, 255, 255, 255));
        drawList->AddText(ImVec2(originScrX + 6.0f, originScrY + 4.0f), IM_COL32(200, 200, 200, 220), "(0, 0)");
    }

    // -------------------------------------------------------------------------
    // Render Reference 3D Feature Geometry (If Enabled)
    // -------------------------------------------------------------------------
    std::vector<std::array<float, 2>> refSnapPoints;
    if (m_showReferenceFeature) {
        std::shared_ptr<DAGNode> refFeature = nullptr;
        if (activeSketch->GetDatumPlaneReference()) {
            refFeature = activeSketch->GetDatumPlaneReference()->GetReferencedFeature();
        }
        if (!refFeature && activeSketch->GetParentFeature()) {
            refFeature = activeSketch->GetParentFeature();
        }
        if (!refFeature && m_selectedNodeId != 0 && m_selectedNodeId != activeSketch->GetId()) {
            for (const auto& n : graph.GetNodes()) {
                if (n->GetId() == m_selectedNodeId) {
                    refFeature = n;
                    break;
                }
            }
        }
        if (!refFeature) {
            for (const auto& n : graph.GetNodes()) {
                if (std::dynamic_pointer_cast<ExtrudeNode>(n) || 
                    std::dynamic_pointer_cast<RevolveNode>(n) ||
                    std::dynamic_pointer_cast<CylinderNode>(n) ||
                    std::dynamic_pointer_cast<SphereNode>(n)) {
                    refFeature = n;
                }
            }
        }

        if (refFeature) {
            Geom::Plane sketchPlane;
            if (activeSketch->HasCustomPlane()) {
                sketchPlane = activeSketch->GetCustomPlane();
            } else {
                Geom::Point3D orig(activeSketch->GetPosition()[0], activeSketch->GetPosition()[1], activeSketch->GetPosition()[2]);
                switch (activeSketch->GetPlane()) {
                    case SketchPlane::XZ_Ground: sketchPlane = Geom::Plane::XZ(orig); break;
                    case SketchPlane::XY_Front:  sketchPlane = Geom::Plane::XY(orig); break;
                    case SketchPlane::YZ_Right:  sketchPlane = Geom::Plane::YZ(orig); break;
                }
            }

            const auto& rVerts = refFeature->GetCachedVertices();
            const auto& rEdges = refFeature->GetCachedEdges();

            for (size_t e = 0; e + 1 < rEdges.size(); e += 2) {
                uint32_t i0 = rEdges[e];
                uint32_t i1 = rEdges[e + 1];
                if (i0 < rVerts.size() && i1 < rVerts.size()) {
                    Geom::Point3D p0(rVerts[i0].position[0], rVerts[i0].position[1], rVerts[i0].position[2]);
                    Geom::Point3D p1(rVerts[i1].position[0], rVerts[i1].position[1], rVerts[i1].position[2]);

                    Geom::Point2D uv0 = sketchPlane.to_local(p0);
                    Geom::Point2D uv1 = sketchPlane.to_local(p1);

                    refSnapPoints.push_back({ static_cast<float>(uv0.u), static_cast<float>(uv0.v) });
                    refSnapPoints.push_back({ static_cast<float>(uv1.u), static_cast<float>(uv1.v) });

                    ImVec2 s0 = ToScreen(static_cast<float>(uv0.u), static_cast<float>(uv0.v));
                    ImVec2 s1 = ToScreen(static_cast<float>(uv1.u), static_cast<float>(uv1.v));

                    // Draw reference silhouette line in cyan/slate-blue
                    drawList->AddLine(s0, s1, IM_COL32(75, 175, 235, 180), 1.8f);
                }
            }

            // Draw vertex dots for vertices near the sketch plane
            for (const auto& v : rVerts) {
                Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
                if (std::abs(sketchPlane.distance_to(pt)) < 0.05) {
                    Geom::Point2D uv = sketchPlane.to_local(pt);
                    ImVec2 sPos = ToScreen(static_cast<float>(uv.u), static_cast<float>(uv.v));
                    drawList->AddCircleFilled(sPos, 3.5f, IM_COL32(100, 215, 255, 220));
                }
            }

            // Draw reference feature tag
            std::string refBadge = "👁 Ref Feature: " + refFeature->GetName();
            drawList->AddText(ImVec2(canvasP0.x + 12.0f, canvasP1.y - 24.0f), IM_COL32(110, 195, 250, 220), refBadge.c_str());
        }
    }

    // -------------------------------------------------------------------------
    // Render Canvas Shapes (Enclosed shapes signified by shaded area)
    // -------------------------------------------------------------------------
    for (size_t s = 0; s < m_canvasShapes.size(); ++s) {
        const auto& shape = m_canvasShapes[s];
        if (shape.points.empty()) continue;

        std::vector<ImVec2> screenPts;
        screenPts.reserve(shape.points.size());
        for (const auto& pt : shape.points) {
            screenPts.push_back(ToScreen(pt[0], pt[1]));
        }

        bool isShapeSelected = (m_selectedShapeIdx == static_cast<int>(s));

        if (shape.isClosed && shape.points.size() >= 3) {
            // ENCLOSED SHAPE: Signified by shaded area!
            auto triIndices = TessellationUtils::TriangulatePolygon2D(shape.points);
            for (size_t t = 0; t + 2 < triIndices.size(); t += 3) {
                drawList->AddTriangleFilled(
                    screenPts[triIndices[t]],
                    screenPts[triIndices[t + 1]],
                    screenPts[triIndices[t + 2]],
                    isShapeSelected ? IM_COL32(255, 195, 30, 48) : IM_COL32(0, 160, 255, 45)
                );
            }
            // Boundary contour
            ImU32 boundaryColor = isShapeSelected ? IM_COL32(255, 215, 0, 255) : IM_COL32(0, 210, 255, 255);
            float boundaryThickness = isShapeSelected ? 3.5f : 2.5f;
            drawList->AddPolyline(screenPts.data(), static_cast<int>(screenPts.size()), boundaryColor, ImDrawFlags_Closed, boundaryThickness);
        } else {
            // OPEN / UNCLOSED SHAPE: NO shaded area!
            ImU32 openColor = isShapeSelected ? IM_COL32(255, 215, 0, 255) : IM_COL32(255, 140, 20, 240);
            drawList->AddPolyline(screenPts.data(), static_cast<int>(screenPts.size()), openColor, 0, 2.5f);
            // Highlight open endpoints
            drawList->AddCircleFilled(screenPts.front(), 6.0f, IM_COL32(255, 60, 60, 255));
            drawList->AddCircleFilled(screenPts.back(), 6.0f, IM_COL32(255, 60, 60, 255));
            drawList->AddText(ImVec2(screenPts.front().x + 8.0f, screenPts.front().y - 14.0f), IM_COL32(255, 90, 90, 255), "Open");
            drawList->AddText(ImVec2(screenPts.back().x + 8.0f, screenPts.back().y - 14.0f), IM_COL32(255, 90, 90, 255), "Open");
        }

        // Selected shape banner
        if (isShapeSelected && m_selectedVertexIdx < 0 && !screenPts.empty()) {
            drawList->AddText(ImVec2(screenPts[0].x - 10.0f, screenPts[0].y - 28.0f), IM_COL32(255, 220, 50, 255), "[Selected Shape: Del to remove]");
        }

        // Draw vertex handles
        for (size_t i = 0; i < shape.points.size(); ++i) {
            ImVec2 ptScr = screenPts[i];
            bool isHoveredPt = (m_hoveredShapeIdx == static_cast<int>(s) && m_hoveredVertexIdx == static_cast<int>(i));
            bool isDraggedPt = (m_draggedShapeIdx == static_cast<int>(s) && m_draggedVertexIdx == static_cast<int>(i));
            bool isSelectedPt = (m_selectedShapeIdx == static_cast<int>(s) && m_selectedVertexIdx == static_cast<int>(i));

            float radius = (isHoveredPt || isDraggedPt || isSelectedPt) ? 7.0f : 4.5f;
            ImU32 color = isDraggedPt ? IM_COL32(255, 80, 80, 255) : 
                          isSelectedPt ? IM_COL32(255, 230, 0, 255) :
                          isHoveredPt ? IM_COL32(255, 215, 0, 255) : 
                                        IM_COL32(255, 255, 255, 230);

            drawList->AddCircleFilled(ptScr, radius, color);
            drawList->AddCircle(ptScr, radius + 1.5f, isSelectedPt ? IM_COL32(255, 230, 0, 255) : IM_COL32(10, 10, 10, 200), 16, 1.5f);

            if (isSelectedPt) {
                drawList->AddText(ImVec2(ptScr.x + 10.0f, ptScr.y - 16.0f), IM_COL32(255, 230, 50, 255), "Selected (Del/RMB to remove)");
            } else if (isHoveredPt || isDraggedPt) {
                char buf[64];
                snprintf(buf, sizeof(buf), "%s #%zu (%.2f, %.2f) [RMB: Del]", shape.name.c_str(), i, shape.points[i][0], shape.points[i][1]);
                drawList->AddText(ImVec2(ptScr.x + 8.0f, ptScr.y - 14.0f), IM_COL32(255, 255, 100, 255), buf);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Cursor coordinates & Snapping (Grid Snap + Vertex Snapping to any shape)
    // -------------------------------------------------------------------------
    float rawU = (io.MousePos.x - canvasP0.x - m_canvasPanX) / m_canvasZoom;
    float rawV = -(io.MousePos.y - canvasP0.y - m_canvasPanY) / m_canvasZoom;
    float snapU = m_gridSnapEnable ? (std::round(rawU / m_gridSnapSize) * m_gridSnapSize) : rawU;
    float snapV = m_gridSnapEnable ? (std::round(rawV / m_gridSnapSize) * m_gridSnapSize) : rawV;

    // Check vertex snapping across all existing shapes (enabling vertex stitching)
    bool snappedToVertex = false;
    bool snappedToRef = false;
    float bestSnapDist = 14.0f; // Screen pixel snap distance
    for (size_t s = 0; s < m_canvasShapes.size(); ++s) {
        for (size_t v = 0; v < m_canvasShapes[s].points.size(); ++v) {
            ImVec2 scr = ToScreen(m_canvasShapes[s].points[v][0], m_canvasShapes[s].points[v][1]);
            float d = std::hypot(io.MousePos.x - scr.x, io.MousePos.y - scr.y);
            if (d < bestSnapDist) {
                bestSnapDist = d;
                snapU = m_canvasShapes[s].points[v][0];
                snapV = m_canvasShapes[s].points[v][1];
                snappedToVertex = true;
                snappedToRef = false;
            }
        }
    }
    for (const auto& rpt : refSnapPoints) {
        ImVec2 scr = ToScreen(rpt[0], rpt[1]);
        float d = std::hypot(io.MousePos.x - scr.x, io.MousePos.y - scr.y);
        if (d < bestSnapDist) {
            bestSnapDist = d;
            snapU = rpt[0];
            snapV = rpt[1];
            snappedToVertex = true;
            snappedToRef = true;
        }
    }
    ImVec2 snapPosScr = ToScreen(snapU, snapV);

    if (isHovered) {
        if (snappedToVertex) {
            ImU32 snapCol = snappedToRef ? IM_COL32(80, 210, 255, 255) : IM_COL32(255, 230, 0, 255);
            ImU32 rectCol = snappedToRef ? IM_COL32(80, 210, 255, 220) : IM_COL32(255, 230, 0, 220);
            drawList->AddCircle(snapPosScr, 7.0f, snapCol, 16, 2.0f);
            drawList->AddRect(ImVec2(snapPosScr.x - 5.0f, snapPosScr.y - 5.0f),
                              ImVec2(snapPosScr.x + 5.0f, snapPosScr.y + 5.0f),
                              rectCol, 0.0f, 0, 1.5f);
            drawList->AddText(ImVec2(snapPosScr.x + 9.0f, snapPosScr.y - 12.0f), snapCol, snappedToRef ? "Snap to Ref Feature" : "Snap to Vertex");
        } else {
            drawList->AddCircle(snapPosScr, 4.0f, IM_COL32(0, 255, 255, 180), 8, 1.2f);
        }
    }

    // -------------------------------------------------------------------------
    // Tool Interactions
    // -------------------------------------------------------------------------
    auto DistToSegment = [](const ImVec2& p, const ImVec2& a, const ImVec2& b) -> float {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float lenSq = dx * dx + dy * dy;
        if (lenSq < 1e-6f) return std::hypot(p.x - a.x, p.y - a.y);
        float t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq, 0.0f, 1.0f);
        float projX = a.x + t * dx;
        float projY = a.y + t * dy;
        return std::hypot(p.x - projX, p.y - projY);
    };

    // 1. Select / Move Tool
    if (m_mouseTool == SketchMouseTool::SelectMove) {
        if (m_draggedShapeIdx < 0) {
            m_hoveredShapeIdx = -1;
            m_hoveredVertexIdx = -1;
            float bestDist = 14.0f;
            for (size_t s = 0; s < m_canvasShapes.size(); ++s) {
                for (size_t i = 0; i < m_canvasShapes[s].points.size(); ++i) {
                    ImVec2 ptScr = ToScreen(m_canvasShapes[s].points[i][0], m_canvasShapes[s].points[i][1]);
                    float dist = std::hypot(io.MousePos.x - ptScr.x, io.MousePos.y - ptScr.y);
                    if (dist < bestDist) {
                        bestDist = dist;
                        m_hoveredShapeIdx = static_cast<int>(s);
                        m_hoveredVertexIdx = static_cast<int>(i);
                    }
                }
            }

            // If no vertex handle is hovered, check if cursor is over a shape body or edge
            if (m_hoveredVertexIdx < 0) {
                for (int s = static_cast<int>(m_canvasShapes.size()) - 1; s >= 0; --s) {
                    const auto& sh = m_canvasShapes[s];
                    if (sh.points.size() >= 3 && Sketch::PointInPolygon2D({rawU, rawV}, sh.points)) {
                        m_hoveredShapeIdx = s;
                        break;
                    }
                    for (size_t i = 0; i < sh.points.size(); ++i) {
                        size_t next = (i + 1) % sh.points.size();
                        ImVec2 p0 = ToScreen(sh.points[i][0], sh.points[i][1]);
                        ImVec2 p1 = ToScreen(sh.points[next][0], sh.points[next][1]);
                        if (DistToSegment(io.MousePos, p0, p1) < 10.0f) {
                            m_hoveredShapeIdx = s;
                            break;
                        }
                    }
                    if (m_hoveredShapeIdx >= 0) break;
                }
            }

            // Left Click: select / start dragging
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
                if (m_hoveredVertexIdx >= 0) {
                    m_selectedShapeIdx = m_hoveredShapeIdx;
                    m_selectedVertexIdx = m_hoveredVertexIdx;
                    m_draggedShapeIdx = m_hoveredShapeIdx;
                    m_draggedVertexIdx = m_hoveredVertexIdx;
                } else if (m_hoveredShapeIdx >= 0) {
                    m_selectedShapeIdx = m_hoveredShapeIdx;
                    m_selectedVertexIdx = -1;
                } else {
                    m_selectedShapeIdx = -1;
                    m_selectedVertexIdx = -1;
                }
            }

            // Right Click in SelectMove mode: instant delete of hovered vertex or hovered shape
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && isHovered) {
                if (m_hoveredShapeIdx >= 0 && m_hoveredShapeIdx < static_cast<int>(m_canvasShapes.size())) {
                    auto& shape = m_canvasShapes[m_hoveredShapeIdx];
                    if (m_hoveredVertexIdx >= 0 && m_hoveredVertexIdx < static_cast<int>(shape.points.size())) {
                        shape.points.erase(shape.points.begin() + m_hoveredVertexIdx);
                        if (m_selectedShapeIdx == m_hoveredShapeIdx) m_selectedVertexIdx = -1;
                        if (shape.points.size() < 3) shape.isClosed = false;
                        if (shape.points.empty()) {
                            m_canvasShapes.erase(m_canvasShapes.begin() + m_hoveredShapeIdx);
                            m_selectedShapeIdx = -1;
                        }
                        m_hoveredShapeIdx = -1;
                        m_hoveredVertexIdx = -1;
                    } else {
                        m_canvasShapes.erase(m_canvasShapes.begin() + m_hoveredShapeIdx);
                        m_selectedShapeIdx = -1;
                        m_selectedVertexIdx = -1;
                        m_hoveredShapeIdx = -1;
                    }
                }
            }
        } else {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                if (m_draggedShapeIdx < static_cast<int>(m_canvasShapes.size()) &&
                    m_draggedVertexIdx < static_cast<int>(m_canvasShapes[m_draggedShapeIdx].points.size())) {
                    m_canvasShapes[m_draggedShapeIdx].points[m_draggedVertexIdx] = { snapU, snapV };
                }
            } else {
                m_draggedShapeIdx = -1;
                m_draggedVertexIdx = -1;
            }
        }
    }
    // 2. Rectangle Tool
    else if (m_mouseTool == SketchMouseTool::Rectangle) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
            m_isDrawing = true;
            m_drawStartU = snapU;
            m_drawStartV = snapV;
        }

        if (m_isDrawing) {
            ImVec2 pA = ToScreen(m_drawStartU, m_drawStartV);
            ImVec2 pB = ToScreen(snapU, snapV);
            ImVec2 minScr = ImVec2(std::min(pA.x, pB.x), std::min(pA.y, pB.y));
            ImVec2 maxScr = ImVec2(std::max(pA.x, pB.x), std::max(pA.y, pB.y));

            // Shaded enclosed shape area during sketching!
            drawList->AddRectFilled(minScr, maxScr, IM_COL32(30, 144, 255, 55));
            drawList->AddRect(minScr, maxScr, IM_COL32(0, 220, 255, 255), 0.0f, 0, 2.0f);

            float w = std::abs(snapU - m_drawStartU);
            float h = std::abs(snapV - m_drawStartV);
            char dimBuf[64];
            snprintf(dimBuf, sizeof(dimBuf), "%.2f m x %.2f m", w, h);
            drawList->AddText(ImVec2((minScr.x + maxScr.x) * 0.5f - 40.0f, (minScr.y + maxScr.y) * 0.5f - 8.0f), IM_COL32(255, 255, 255, 255), dimBuf);

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                m_isDrawing = false;
                float minU = std::min(m_drawStartU, snapU);
                float maxU = std::max(m_drawStartU, snapU);
                float minV = std::min(m_drawStartV, snapV);
                float maxV = std::max(m_drawStartV, snapV);
                if ((maxU - minU) > 0.05f && (maxV - minV) > 0.05f) {
                    m_canvasShapes.push_back({
                        "Rectangle " + std::to_string(m_canvasShapes.size() + 1),
                        { {minU, minV}, {maxU, minV}, {maxU, maxV}, {minU, maxV} },
                        true
                    });
                }
            }
        }
    }
    // 3. Circle Tool
    else if (m_mouseTool == SketchMouseTool::Circle) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
            m_isDrawing = true;
            m_drawStartU = snapU;
            m_drawStartV = snapV;
        }

        if (m_isDrawing) {
            float radius = std::hypot(snapU - m_drawStartU, snapV - m_drawStartV);
            ImVec2 centerScr = ToScreen(m_drawStartU, m_drawStartV);
            float radiusScr = radius * m_canvasZoom;

            // Shaded enclosed shape area during sketching!
            drawList->AddCircleFilled(centerScr, radiusScr, IM_COL32(30, 144, 255, 55), 48);
            drawList->AddCircle(centerScr, radiusScr, IM_COL32(0, 220, 255, 255), 48, 2.0f);
            drawList->AddLine(centerScr, snapPosScr, IM_COL32(255, 255, 100, 220), 1.5f);

            char dimBuf[64];
            snprintf(dimBuf, sizeof(dimBuf), "R: %.2f m", radius);
            drawList->AddText(ImVec2(centerScr.x + 8.0f, centerScr.y - 16.0f), IM_COL32(255, 255, 255, 255), dimBuf);

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                m_isDrawing = false;
                if (radius > 0.05f) {
                    const int segs = 32;
                    std::vector<std::array<float, 2>> circleProfile;
                    circleProfile.reserve(segs);
                    for (int i = 0; i < segs; ++i) {
                        float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 6.2831853f;
                        circleProfile.push_back({m_drawStartU + radius * std::cos(theta), m_drawStartV + radius * std::sin(theta)});
                    }
                    m_canvasShapes.push_back({
                        "Circle " + std::to_string(m_canvasShapes.size() + 1),
                        std::move(circleProfile),
                        true
                    });
                }
            }
        }
    }
    // 4. Line / Polyline Tool
    else if (m_mouseTool == SketchMouseTool::Line) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            m_inProgressPoints.clear();
        }

        bool isCloseHovered = false;
        if (!m_inProgressPoints.empty() && m_inProgressPoints.size() >= 3) {
            ImVec2 firstPtScr = ToScreen(m_inProgressPoints[0][0], m_inProgressPoints[0][1]);
            float distToFirst = std::hypot(io.MousePos.x - firstPtScr.x, io.MousePos.y - firstPtScr.y);
            if (distToFirst < 16.0f) isCloseHovered = true;
        }

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
            if (m_inProgressPoints.empty()) {
                m_inProgressPoints.push_back({snapU, snapV});
            } else {
                if ((isCloseHovered || ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) && m_inProgressPoints.size() >= 3) {
                    // Loop is enclosed and closed!
                    m_canvasShapes.push_back({
                        "Polyline " + std::to_string(m_canvasShapes.size() + 1),
                        m_inProgressPoints,
                        true
                    });
                    m_inProgressPoints.clear();
                } else if (!isCloseHovered) {
                    auto last = m_inProgressPoints.back();
                    if (std::hypot(snapU - last[0], snapV - last[1]) > 0.01f) {
                        m_inProgressPoints.push_back({snapU, snapV});
                    }
                }
            }
        }

        if (!m_inProgressPoints.empty()) {
            // As sketching is happening, an enclosed shape is signified by a shaded area!
            if (isCloseHovered && m_inProgressPoints.size() >= 3) {
                auto triIndices = TessellationUtils::TriangulatePolygon2D(m_inProgressPoints);
                for (size_t t = 0; t + 2 < triIndices.size(); t += 3) {
                    drawList->AddTriangleFilled(
                        ToScreen(m_inProgressPoints[triIndices[t]][0], m_inProgressPoints[triIndices[t]][1]),
                        ToScreen(m_inProgressPoints[triIndices[t + 1]][0], m_inProgressPoints[triIndices[t + 1]][1]),
                        ToScreen(m_inProgressPoints[triIndices[t + 2]][0], m_inProgressPoints[triIndices[t + 2]][1]),
                        IM_COL32(30, 220, 140, 55) // Greenish/cyan shaded enclosed shape preview
                    );
                }
            }

            for (size_t i = 0; i + 1 < m_inProgressPoints.size(); ++i) {
                drawList->AddLine(ToScreen(m_inProgressPoints[i][0], m_inProgressPoints[i][1]),
                                  ToScreen(m_inProgressPoints[i+1][0], m_inProgressPoints[i+1][1]),
                                  IM_COL32(255, 180, 0, 255), 2.5f);
                drawList->AddCircleFilled(ToScreen(m_inProgressPoints[i][0], m_inProgressPoints[i][1]), 4.0f, IM_COL32(255, 200, 50, 255));
            }
            ImVec2 lastPtScr = ToScreen(m_inProgressPoints.back()[0], m_inProgressPoints.back()[1]);
            drawList->AddLine(lastPtScr, snapPosScr, IM_COL32(255, 220, 100, 200), 2.0f);
            drawList->AddCircleFilled(lastPtScr, 4.0f, IM_COL32(255, 200, 50, 255));

            ImVec2 firstPtScr = ToScreen(m_inProgressPoints[0][0], m_inProgressPoints[0][1]);
            if (m_inProgressPoints.size() >= 3) {
                drawList->AddCircle(firstPtScr, 9.0f, isCloseHovered ? IM_COL32(50, 255, 50, 255) : IM_COL32(255, 180, 0, 180), 16, 2.0f);
                if (isCloseHovered) {
                    drawList->AddText(ImVec2(firstPtScr.x + 12.0f, firstPtScr.y - 12.0f), IM_COL32(50, 255, 50, 255), "Click to Close & Enclose Loop");
                }
            }

            float segLen = std::hypot(snapU - m_inProgressPoints.back()[0], snapV - m_inProgressPoints.back()[1]);
            char lenBuf[64];
            snprintf(lenBuf, sizeof(lenBuf), "L: %.2f m", segLen);
            drawList->AddText(ImVec2((lastPtScr.x + snapPosScr.x) * 0.5f + 6.0f, (lastPtScr.y + snapPosScr.y) * 0.5f - 14.0f), IM_COL32(255, 255, 255, 220), lenBuf);
        }
    }
    // 5. Spline Tool
    else if (m_mouseTool == SketchMouseTool::Spline) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            m_inProgressSplineControls.clear();
        }

        bool isCloseHovered = false;
        if (!m_inProgressSplineControls.empty() && m_inProgressSplineControls.size() >= 3) {
            ImVec2 firstPtScr = ToScreen(m_inProgressSplineControls[0][0], m_inProgressSplineControls[0][1]);
            float distToFirst = std::hypot(io.MousePos.x - firstPtScr.x, io.MousePos.y - firstPtScr.y);
            if (distToFirst < 16.0f) isCloseHovered = true;
        }

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
            if (m_inProgressSplineControls.empty()) {
                m_inProgressSplineControls.push_back({snapU, snapV});
            } else {
                if ((isCloseHovered || ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) && m_inProgressSplineControls.size() >= 3) {
                    auto denseSpline = EvaluateCatmullRomSpline(m_inProgressSplineControls, 12);
                    m_canvasShapes.push_back({
                        "Spline " + std::to_string(m_canvasShapes.size() + 1),
                        std::move(denseSpline),
                        true
                    });
                    m_inProgressSplineControls.clear();
                } else if (!isCloseHovered) {
                    auto last = m_inProgressSplineControls.back();
                    if (std::hypot(snapU - last[0], snapV - last[1]) > 0.01f) {
                        m_inProgressSplineControls.push_back({snapU, snapV});
                    }
                }
            }
        }

        if (!m_inProgressSplineControls.empty()) {
            if (isCloseHovered && m_inProgressSplineControls.size() >= 3) {
                auto denseSpline = EvaluateCatmullRomSpline(m_inProgressSplineControls, 8);
                auto triIndices = TessellationUtils::TriangulatePolygon2D(denseSpline);
                for (size_t t = 0; t + 2 < triIndices.size(); t += 3) {
                    drawList->AddTriangleFilled(
                        ToScreen(denseSpline[triIndices[t]][0], denseSpline[triIndices[t]][1]),
                        ToScreen(denseSpline[triIndices[t + 1]][0], denseSpline[triIndices[t + 1]][1]),
                        ToScreen(denseSpline[triIndices[t + 2]][0], denseSpline[triIndices[t + 2]][1]),
                        IM_COL32(30, 220, 140, 55)
                    );
                }
            }

            for (size_t i = 0; i < m_inProgressSplineControls.size(); ++i) {
                ImVec2 pScr = ToScreen(m_inProgressSplineControls[i][0], m_inProgressSplineControls[i][1]);
                drawList->AddCircleFilled(pScr, 5.0f, IM_COL32(255, 100, 200, 255));
                if (i > 0) {
                    ImVec2 prevScr = ToScreen(m_inProgressSplineControls[i-1][0], m_inProgressSplineControls[i-1][1]);
                    drawList->AddLine(prevScr, pScr, IM_COL32(180, 180, 180, 100), 1.0f);
                }
            }
            ImVec2 lastCtrlScr = ToScreen(m_inProgressSplineControls.back()[0], m_inProgressSplineControls.back()[1]);
            drawList->AddLine(lastCtrlScr, snapPosScr, IM_COL32(180, 180, 180, 100), 1.0f);

            if (m_inProgressSplineControls.size() >= 3) {
                ImVec2 firstPtScr = ToScreen(m_inProgressSplineControls[0][0], m_inProgressSplineControls[0][1]);
                drawList->AddCircle(firstPtScr, 9.0f, isCloseHovered ? IM_COL32(50, 255, 50, 255) : IM_COL32(255, 100, 200, 200), 16, 2.0f);
                if (isCloseHovered) {
                    drawList->AddText(ImVec2(firstPtScr.x + 12.0f, firstPtScr.y - 12.0f), IM_COL32(50, 255, 50, 255), "Click to Close & Enclose Spline");
                }
            }

            std::vector<std::array<float, 2>> previewControls = m_inProgressSplineControls;
            previewControls.push_back({snapU, snapV});
            if (previewControls.size() >= 2) {
                size_t n = previewControls.size();
                std::vector<ImVec2> splineCurvePts;
                for (size_t i = 0; i + 1 < n; ++i) {
                    const auto& p0 = (i == 0) ? previewControls[0] : previewControls[i - 1];
                    const auto& p1 = previewControls[i];
                    const auto& p2 = previewControls[i + 1];
                    const auto& p3 = (i + 2 >= n) ? previewControls[i + 1] : previewControls[i + 2];
                    for (int s = 0; s < 10; ++s) {
                        float t = static_cast<float>(s) / 10.0f;
                        float t2 = t * t;
                        float t3 = t2 * t;
                        float u = 0.5f * ((2.0f * p1[0]) + (-p0[0] + p2[0]) * t + (2.0f * p0[0] - 5.0f * p1[0] + 4.0f * p2[0] - p3[0]) * t2 + (-p0[0] + 3.0f * p1[0] - 3.0f * p2[0] + p3[0]) * t3);
                        float v = 0.5f * ((2.0f * p1[1]) + (-p0[1] + p2[1]) * t + (2.0f * p0[1] - 5.0f * p1[1] + 4.0f * p2[1] - p3[1]) * t2 + (-p0[1] + 3.0f * p1[1] - 3.0f * p2[1] + p3[1]) * t3);
                        splineCurvePts.push_back(ToScreen(u, v));
                    }
                }
                splineCurvePts.push_back(snapPosScr);
                drawList->AddPolyline(splineCurvePts.data(), static_cast<int>(splineCurvePts.size()), IM_COL32(255, 215, 0, 255), 0, 2.5f);
            }
        }
    }

    // -------------------------------------------------------------------------
    // HUD Overlays
    // -------------------------------------------------------------------------
    // 1. Coordinate Readout HUD
    char coordBuf[128];
    snprintf(coordBuf, sizeof(coordBuf), "Cursor: (U: %+.2f m, V: %+.2f m) | Grid Snap: %.2f m", snapU, snapV, m_gridSnapSize);
    drawList->AddRectFilled(ImVec2(canvasP0.x + 8.0f, canvasP1.y - 28.0f), ImVec2(canvasP0.x + 380.0f, canvasP1.y - 6.0f), IM_COL32(15, 17, 20, 200), 4.0f);
    drawList->AddText(ImVec2(canvasP0.x + 14.0f, canvasP1.y - 25.0f), IM_COL32(0, 230, 255, 255), coordBuf);

    // 2. Tool quick tip
    const char* toolTipText = "";
    switch (m_mouseTool) {
        case SketchMouseTool::SelectMove: toolTipText = "[Select/Move] Click vertex/shape to select. Drag to move. Del/RMB: Delete"; break;
        case SketchMouseTool::Rectangle:  toolTipText = "[Rectangle] Click & drag diagonally to draw rectangle (R)"; break;
        case SketchMouseTool::Circle:     toolTipText = "[Circle] Click center & drag outward to draw circle (C)"; break;
        case SketchMouseTool::Line:       toolTipText = "[Line/Poly] Click vertices. Click first / double-click to close (RMB: Cancel)"; break;
        case SketchMouseTool::Spline:     toolTipText = "[Spline] Click control points. Click first / double-click to close (RMB: Cancel)"; break;
    }
    ImVec2 tipSz = ImGui::CalcTextSize(toolTipText);
    drawList->AddRectFilled(ImVec2(canvasP1.x - tipSz.x - 20.0f, canvasP0.y + 8.0f), ImVec2(canvasP1.x - 8.0f, canvasP0.y + 28.0f), IM_COL32(15, 17, 20, 190), 4.0f);
    drawList->AddText(ImVec2(canvasP1.x - tipSz.x - 14.0f, canvasP0.y + 11.0f), IM_COL32(230, 230, 230, 230), toolTipText);

    drawList->PopClipRect();

    // -------------------------------------------------------------------------
    // Interactive Shape Chips Bar (Click chip to select, ✕ to delete shape)
    // -------------------------------------------------------------------------
    ImGui::SetCursorScreenPos(ImVec2(canvasP0.x + 8.0f, canvasP1.y - 68.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(18, 22, 28, 225));
    ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(50, 60, 75, 200));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
    ImGui::BeginChild("CanvasShapeChips", ImVec2(canvasSz.x - 16.0f, 34.0f), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Shapes (%zu):", m_canvasShapes.size());
    ImGui::SameLine();

    for (size_t s = 0; s < m_canvasShapes.size(); ++s) {
        ImGui::PushID(static_cast<int>(s + 7000));
        bool isSel = (m_selectedShapeIdx == static_cast<int>(s));
        if (isSel) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.52f, 0.12f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.82f, 0.62f, 0.18f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.24f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.34f, 0.42f, 1.0f));
        }

        std::string label = m_canvasShapes[s].name + " (" + std::to_string(m_canvasShapes[s].points.size()) + " pts)" + 
                            (m_canvasShapes[s].isClosed ? " ✓" : " ⚠");
        if (ImGui::Button(label.c_str())) {
            m_selectedShapeIdx = static_cast<int>(s);
            m_selectedVertexIdx = -1;
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Select %s on canvas (Del to delete)", m_canvasShapes[s].name.c_str());
        }
        ImGui::SameLine(0.0f, 2.0f);

        // Delete button for this shape
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.18f, 0.9f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
        if (ImGui::Button("✕", ImVec2(22, 0))) {
            m_canvasShapes.erase(m_canvasShapes.begin() + s);
            if (m_selectedShapeIdx == static_cast<int>(s)) {
                m_selectedShapeIdx = -1;
                m_selectedVertexIdx = -1;
            } else if (m_selectedShapeIdx > static_cast<int>(s)) {
                m_selectedShapeIdx--;
            }
            ImGui::PopStyleColor(2);
            ImGui::PopID();
            break;
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Delete %s", m_canvasShapes[s].name.c_str());
        }
        ImGui::SameLine(0.0f, 8.0f);
        ImGui::PopID();
    }
    if (m_canvasShapes.empty()) {
        ImGui::TextDisabled("No shapes on canvas. Use Rectangle, Circle, or Line tools above to draw.");
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);

    ImGui::End();
}

void CADGui::EndFrameAndDraw(RHI::RHICommandList* cmdList) {
    if (!m_initialized) return;

    ImGui::Render();
    ImDrawData* drawData = ImGui::GetDrawData();

    void* nativeCmd = cmdList ? cmdList->GetNativeCommandBuffer() : nullptr;
    if (nativeCmd) {
        ImGui_ImplVulkan_RenderDrawData(drawData, static_cast<VkCommandBuffer>(nativeCmd));
    }
}

} // namespace Tessera
