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
#include <filesystem>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
#endif

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
            CloseMeasurement();
            ImGui::OpenPopup("RefPlanePopup");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("1. Create Reference Datum Plane (from Selected Face, Corner Edge/Vertex, or Standard Planes with Offset)");
        }
        if (ImGui::BeginPopup("RefPlanePopup")) {
            ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Create Reference Plane");
            ImGui::Separator();
            ImGui::SetNextItemWidth(120.0f);
            ImGui::InputFloat("Offset Distance##RefPlane", &m_newRefPlaneOffset, 0.1f, 0.5f, "%.2f m");
            ImGui::Separator();

            if (m_selectedFaceIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                char itemBuf[128];
                std::snprintf(itemBuf, sizeof(itemBuf), "★ From Selected Model Face (Offset: %+.2f m)", m_newRefPlaneOffset);
                if (ImGui::MenuItem(itemBuf)) {
                    std::string name = "Plane_Face_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumPlaneNode>(name, target, m_selectedFaceIndex, m_selectedFacePlane, m_newRefPlaneOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            } else if (m_selectedEdgeIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                char itemBuf[128];
                std::snprintf(itemBuf, sizeof(itemBuf), "★ From Selected Corner Edge (Offset: %+.2f m)", m_newRefPlaneOffset);
                if (ImGui::MenuItem(itemBuf)) {
                    Geom::Plane edgePlane = Geom::Plane::XZ(Geom::Point3D(
                        (m_selectedEdgeP0.x + m_selectedEdgeP1.x) * 0.5,
                        (m_selectedEdgeP0.y + m_selectedEdgeP1.y) * 0.5,
                        (m_selectedEdgeP0.z + m_selectedEdgeP1.z) * 0.5
                    ));
                    DatumPlaneNode::ComputeCornerEdgePlane(target, m_selectedEdgeIndex, edgePlane);
                    std::string name = "Plane_CornerEdge_" + std::to_string(m_shapeCounter++);
                    auto node = DatumPlaneNode::CreateFromEdge(name, target, m_selectedEdgeIndex, edgePlane, m_newRefPlaneOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                char itemNormBuf[128];
                std::snprintf(itemNormBuf, sizeof(itemNormBuf), "★ Normal to Selected Corner Edge (Offset: %+.2f m)", m_newRefPlaneOffset);
                if (ImGui::MenuItem(itemNormBuf)) {
                    Geom::Point3D mid(
                        (m_selectedEdgeP0.x + m_selectedEdgeP1.x) * 0.5,
                        (m_selectedEdgeP0.y + m_selectedEdgeP1.y) * 0.5,
                        (m_selectedEdgeP0.z + m_selectedEdgeP1.z) * 0.5
                    );
                    Geom::Vector3D edgeDir = (m_selectedEdgeP1 - m_selectedEdgeP0).normalized();
                    Geom::Plane normPlane(mid, edgeDir);
                    std::string name = "Plane_EdgeNormal_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumPlaneNode>(name, normPlane, m_newRefPlaneOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                char itemBuf[128];
                std::snprintf(itemBuf, sizeof(itemBuf), "★ From Selected Corner Vertex (Offset: %+.2f m)", m_newRefPlaneOffset);
                if (ImGui::MenuItem(itemBuf)) {
                    std::string name = "Plane_Vertex_" + std::to_string(m_shapeCounter++);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    DatumPlaneNode::ComputeCornerVertexPlane(target, m_selectedVertexIndex, pl);
                    auto node = std::make_shared<DatumPlaneNode>(name, target, m_selectedVertexIndex, pl, true, m_newRefPlaneOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                char itemXZBuf[128];
                std::snprintf(itemXZBuf, sizeof(itemXZBuf), "★ Horizontal (XZ) at Selected Vertex (Offset: %+.2f m)", m_newRefPlaneOffset);
                if (ImGui::MenuItem(itemXZBuf)) {
                    std::string name = "Plane_VertexXZ_" + std::to_string(m_shapeCounter++);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    auto node = std::make_shared<DatumPlaneNode>(name, target, m_selectedVertexIndex, pl, true, m_newRefPlaneOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            }
            if (ImGui::MenuItem("Top Plane (XZ Ground)")) {
                std::string name = "Plane_XZ_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumPlaneNode>(name, Geom::Plane::XZ(), m_newRefPlaneOffset);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("Front Plane (XY)")) {
                std::string name = "Plane_XY_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumPlaneNode>(name, Geom::Plane::XY(), m_newRefPlaneOffset);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("Right Plane (YZ)")) {
                std::string name = "Plane_YZ_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumPlaneNode>(name, Geom::Plane::YZ(), m_newRefPlaneOffset);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (auto selectedPlane = std::dynamic_pointer_cast<DatumPlaneNode>(selectedNode)) {
                ImGui::Separator();
                double offVal = (std::abs(m_newRefPlaneOffset) > 1e-4f) ? m_newRefPlaneOffset : 1.0;
                char offLabel[128];
                std::snprintf(offLabel, sizeof(offLabel), "Offset Plane (%+.2f m from Selected)", offVal);
                if (ImGui::MenuItem(offLabel)) {
                    std::string name = "Plane_Offset_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumPlaneNode>(name, selectedPlane->GetPlane().offset(offVal));
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
            CloseMeasurement();
            ImGui::OpenPopup("RefAxisPopup");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("2. Create Reference Datum Axis (from Selected Face, Corner Edge/Vertex, or Standard Axes with Offset)");
        }
        if (ImGui::BeginPopup("RefAxisPopup")) {
            ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Create Reference Axis");
            ImGui::Separator();
            ImGui::SetNextItemWidth(120.0f);
            ImGui::InputFloat("Offset Distance##RefAxis", &m_newRefAxisOffset, 0.1f, 0.5f, "%.2f m");
            ImGui::Separator();

            if (m_selectedFaceIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                char normBuf[128];
                std::snprintf(normBuf, sizeof(normBuf), "★ Normal Axis from Selected Face (Offset: %+.2f m)", m_newRefAxisOffset);
                if (ImGui::MenuItem(normBuf)) {
                    std::string name = "Axis_FaceNormal_" + std::to_string(m_shapeCounter++);
                    auto node = DatumAxisNode::CreateFromFace(name, target, m_selectedFaceIndex, m_selectedFacePlane.normal, m_newRefAxisOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                char uBuf[128];
                std::snprintf(uBuf, sizeof(uBuf), "★ In-Plane U-Axis from Selected Face (Offset: %+.2f m)", m_newRefAxisOffset);
                if (ImGui::MenuItem(uBuf)) {
                    std::string name = "Axis_FaceU_" + std::to_string(m_shapeCounter++);
                    auto node = DatumAxisNode::CreateFromFace(name, target, m_selectedFaceIndex, m_selectedFacePlane.u_axis, m_newRefAxisOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                char vBuf[128];
                std::snprintf(vBuf, sizeof(vBuf), "★ In-Plane V-Axis from Selected Face (Offset: %+.2f m)", m_newRefAxisOffset);
                if (ImGui::MenuItem(vBuf)) {
                    std::string name = "Axis_FaceV_" + std::to_string(m_shapeCounter++);
                    auto node = DatumAxisNode::CreateFromFace(name, target, m_selectedFaceIndex, m_selectedFacePlane.v_axis, m_newRefAxisOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            } else if (m_selectedEdgeIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                char edgeBuf[128];
                std::snprintf(edgeBuf, sizeof(edgeBuf), "★ Along Selected Corner Edge (Offset: %+.2f m)", m_newRefAxisOffset);
                if (ImGui::MenuItem(edgeBuf)) {
                    std::string name = "Axis_CornerEdge_" + std::to_string(m_shapeCounter++);
                    auto node = DatumAxisNode::CreateFromEdge(name, target, m_selectedEdgeIndex, m_newRefAxisOffset, true);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                char edgeNormBuf[128];
                std::snprintf(edgeNormBuf, sizeof(edgeNormBuf), "★ Normal to Selected Corner Edge (Offset: %+.2f m)", m_newRefAxisOffset);
                if (ImGui::MenuItem(edgeNormBuf)) {
                    std::string name = "Axis_EdgeNormal_" + std::to_string(m_shapeCounter++);
                    auto node = DatumAxisNode::CreateFromEdge(name, target, m_selectedEdgeIndex, m_newRefAxisOffset, false);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                char vertBuf[128];
                std::snprintf(vertBuf, sizeof(vertBuf), "★ From Selected Corner Vertex (Offset: %+.2f m)", m_newRefAxisOffset);
                if (ImGui::MenuItem(vertBuf)) {
                    std::string name = "Axis_Vertex_" + std::to_string(m_shapeCounter++);
                    auto node = std::make_shared<DatumAxisNode>(name, target, m_selectedVertexIndex, Geom::Vector3D(0.0, 1.0, 0.0), m_newRefAxisOffset);
                    graph.AddNode(node);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(node->GetId());
                }
                ImGui::Separator();
            }
            if (ImGui::MenuItem("Y-Axis (Vertical Up)")) {
                std::string name = "Axis_Y_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumAxisNode>(name, Geom::Axis3D::Y(), m_newRefAxisOffset);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("X-Axis (Horizontal)")) {
                std::string name = "Axis_X_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumAxisNode>(name, Geom::Axis3D::X(), m_newRefAxisOffset);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(node->GetId());
            }
            if (ImGui::MenuItem("Z-Axis (Depth)")) {
                std::string name = "Axis_Z_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<DatumAxisNode>(name, Geom::Axis3D::Z(), m_newRefAxisOffset);
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
            CloseMeasurement();
            // When creating a 2d sketch:
            // 1) If a face or corner edge/vertex on a model is selected, automatically use that as datum plane!
            // 2) If a datum plane is selected, use that datum plane.
            // 3) If no plane is selected, ask to identify either by clicking on model or from tree.
            if (m_selectedFaceIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                FaceInfo faceInfo;
                if (target && target->GetFace(m_selectedFaceIndex, faceInfo)) {
                    m_selectedFacePlane = faceInfo.plane;
                }
                std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                auto planeNode = std::make_shared<DatumPlaneNode>(planeName, target, m_selectedFaceIndex, m_selectedFacePlane, 0.0);
                planeNode->Evaluate(device, exchange);
                graph.AddNode(planeNode);

                Geom::Plane effPl = planeNode->GetPlane();
                std::string name = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                sketch->SetParentFeature(target);
                sketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                sketch->Evaluate(device, exchange);
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                EnterSketchMode(sketch->GetId());
                m_showSketchPlanePrompt = false;
            } else if (m_selectedEdgeIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                Geom::Plane edgePlane = Geom::Plane::XZ(Geom::Point3D(
                    (m_selectedEdgeP0.x + m_selectedEdgeP1.x) * 0.5,
                    (m_selectedEdgeP0.y + m_selectedEdgeP1.y) * 0.5,
                    (m_selectedEdgeP0.z + m_selectedEdgeP1.z) * 0.5
                ));
                DatumPlaneNode::ComputeCornerEdgePlane(target, m_selectedEdgeIndex, edgePlane);
                std::string planeName = "Plane_CornerEdge_" + std::to_string(m_shapeCounter);
                auto planeNode = DatumPlaneNode::CreateFromEdge(planeName, target, m_selectedEdgeIndex, edgePlane, 0.0);
                planeNode->Evaluate(device, exchange);
                graph.AddNode(planeNode);

                Geom::Plane effPl = planeNode->GetPlane();
                std::string name = "Sketch_CornerEdge_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                sketch->SetParentFeature(target);
                sketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                sketch->Evaluate(device, exchange);
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                EnterSketchMode(sketch->GetId());
                m_showSketchPlanePrompt = false;
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                std::string planeName = "Plane_Vertex_" + std::to_string(m_shapeCounter);
                Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                DatumPlaneNode::ComputeCornerVertexPlane(target, m_selectedVertexIndex, pl);
                auto planeNode = std::make_shared<DatumPlaneNode>(planeName, target, m_selectedVertexIndex, pl, true, 0.0);
                planeNode->Evaluate(device, exchange);
                graph.AddNode(planeNode);

                Geom::Plane effPl = planeNode->GetPlane();
                std::string name = "Sketch_Vertex_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                sketch->SetParentFeature(target);
                sketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                sketch->Evaluate(device, exchange);
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                EnterSketchMode(sketch->GetId());
                m_showSketchPlanePrompt = false;
            } else if (auto planeNode = std::dynamic_pointer_cast<DatumPlaneNode>(selectedNode)) {
                planeNode->Evaluate(device, exchange);
                Geom::Plane pl = planeNode->GetPlane();
                std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                auto sketch = std::make_shared<SketchNode>(name, planeNode);
                if (planeNode->GetTargetFeature()) {
                    sketch->SetParentFeature(planeNode->GetTargetFeature());
                }
                sketch->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
                sketch->Evaluate(device, exchange);
                graph.AddNode(sketch);
                graph.EvaluateDirtyNodes(device, exchange);
                SetSelectedNodeId(sketch->GetId());
                EnterSketchMode(sketch->GetId());
                m_showSketchPlanePrompt = false;
            } else {
                m_showSketchPlanePrompt = true;
                m_promptInitialSelectedId = m_selectedNodeId;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("3. Create 2D Sketch:\n- If a plane, face, or corner is selected, automatically uses it as sketch plane.\n- If none is selected, prompts to identify plane on model or tree.");
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
            CloseMeasurement();
            // If a 2d sketch is selected, start live feature preview from that sketch.
            // If a 2d sketch is not selected, prompt to pick one or jump into 2d sketch creation.
            auto sketchNode = std::dynamic_pointer_cast<SketchNode>(selectedNode);
            if (!sketchNode && m_selectedNodeId != 0) {
                sketchNode = std::dynamic_pointer_cast<SketchNode>(graph.GetNode(m_selectedNodeId));
            }
            if (!sketchNode && selectedNode) {
                // If a solid model or face datum plane is selected, find any unconsumed sketch associated with it
                for (const auto& n : nodes) {
                    if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                        if (!s->IsConsumed()) {
                            if (s->GetParentFeature() == selectedNode) {
                                sketchNode = s;
                                break;
                            }
                            if (auto dp = s->GetDatumPlaneReference()) {
                                if (dp == selectedNode || dp->GetTargetFeature() == selectedNode) {
                                    sketchNode = s;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            if (sketchNode && !sketchNode->IsConsumed()) {
                std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                auto targetSolid = ResolveTargetSolid(sketchNode, graph);
                auto extrude = std::make_shared<ExtrudeNode>(name, sketchNode, m_newExtrudeHeight, ExtrudeOperation::Boss, targetSolid);
                StartFeaturePreview(PreviewFeatureType::Extrude, extrude, targetSolid, graph, device, exchange);
            } else {
                m_showExtrudePrompt = true;
                m_promptInitialSelectedId = m_selectedNodeId;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("4. Create Extrude Feature:\n- Previews 3D extrusion (Boss/Cut) live in viewport with confirm/cancel.\n- If none selected, prompts to pick from model tree or jump into sketch creation.");
        }

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.38f, 0.32f, 0.18f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.52f, 0.44f, 0.26f, 1.0f));
        if (ImGui::Button("↻ Revolve", ImVec2(92, 32))) {
            CloseMeasurement();
            auto sketchNode = std::dynamic_pointer_cast<SketchNode>(selectedNode);
            if (!sketchNode && m_selectedNodeId != 0) {
                sketchNode = std::dynamic_pointer_cast<SketchNode>(graph.GetNode(m_selectedNodeId));
            }
            if (!sketchNode && selectedNode) {
                for (const auto& n : nodes) {
                    if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                        if (!s->IsConsumed()) {
                            if (s->GetParentFeature() == selectedNode) {
                                sketchNode = s;
                                break;
                            }
                            if (auto dp = s->GetDatumPlaneReference()) {
                                if (dp == selectedNode || dp->GetTargetFeature() == selectedNode) {
                                    sketchNode = s;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            if (sketchNode && !sketchNode->IsConsumed()) {
                std::string name = "Revolve_" + std::to_string(m_shapeCounter++);
                auto targetSolid = ResolveTargetSolid(sketchNode, graph);
                auto revolve = std::make_shared<RevolveNode>(name, sketchNode, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis, RevolveOperation::Boss, targetSolid);
                StartFeaturePreview(PreviewFeatureType::Revolve, revolve, targetSolid, graph, device, exchange);
            } else {
                m_showRevolvePrompt = true;
                m_promptInitialSelectedId = m_selectedNodeId;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("5. Create Revolve Feature:\n- Previews 3D revolution (Boss/Cut) live in viewport with confirm/cancel.\n- Cylindrical wall thickness is directly dictated by 2D sketch drawing.");
        }

        // Fillet & Chamfer features
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.35f, 0.48f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.48f, 0.64f, 1.0f));
        if (ImGui::Button("⌒ Fillet", ImVec2(80, 32))) {
            CloseMeasurement();
            if (m_selectedEdgeIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                float effRadius = (m_filletChoice == FilletDimensionChoice::Angle)
                    ? (0.25f * std::tan(m_newFilletAngle * 3.14159265f / 360.0f))
                    : m_newFilletRadius;
                auto fillet = std::make_shared<FilletNode>(name, target, m_selectedEdgeIndex, effRadius);
                if (m_selectedCircle.isCircle && m_selectedCircle.segments.size() > 1) {
                    const auto& verts = target->GetCachedVertices();
                    const auto& edges = target->GetCachedEdges();
                    std::vector<std::pair<Geom::Point3D, Geom::Point3D>> addEdges;
                    for (int segIdx : m_selectedCircle.segments) {
                        if (segIdx == m_selectedEdgeIndex) continue;
                        size_t e = static_cast<size_t>(segIdx * 2);
                        if (e + 1 < edges.size()) {
                            uint32_t i0 = edges[e];
                            uint32_t i1 = edges[e + 1];
                            if (i0 < verts.size() && i1 < verts.size()) {
                                addEdges.push_back({
                                    Geom::Point3D(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]),
                                    Geom::Point3D(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2])
                                });
                            }
                        }
                    }
                    fillet->SetAdditionalEdges(addEdges);
                }
                if (m_filletChoice == FilletDimensionChoice::Angle) {
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
                    fillet->SetAngle(m_newFilletAngle);
                }
                StartFeaturePreview(PreviewFeatureType::Fillet, fillet, target, graph, device, exchange);
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                const auto& edges = target->GetCachedEdges();
                const auto& verts = target->GetCachedVertices();

                ExtrusionData exData;
                Geom::Vector3D extrudeNormal(0.0, 1.0, 0.0);
                if (ExtractExtrusionData(target, exData)) {
                    if (exData.useCustomPlane) {
                        extrudeNormal = exData.customPlane.normal;
                    } else {
                        switch (exData.plane) {
                            case SketchPlane::XZ_Ground: extrudeNormal = Geom::Vector3D(0, 1, 0); break;
                            case SketchPlane::XY_Front:  extrudeNormal = Geom::Vector3D(0, 0, 1); break;
                            case SketchPlane::YZ_Right:  extrudeNormal = Geom::Vector3D(1, 0, 0); break;
                        }
                    }
                }

                VertexCornerEdges corner = CollectVertexCornerEdges(verts, edges, m_selectedVertexPos, extrudeNormal);
                if (corner.primaryEdge >= 0) {
                    std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                    float effRadius = (m_filletChoice == FilletDimensionChoice::Angle)
                        ? (0.25f * std::tan(m_newFilletAngle * 3.14159265f / 360.0f))
                        : m_newFilletRadius;
                    auto fillet = std::make_shared<FilletNode>(name, target, corner.primaryEdge, effRadius);
                    fillet->SetAdditionalEdges(corner.otherEdges);
                    if (m_filletChoice == FilletDimensionChoice::Angle) {
                        fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
                        fillet->SetAngle(m_newFilletAngle);
                    }
                    StartFeaturePreview(PreviewFeatureType::Fillet, fillet, target, graph, device, exchange);
                }
            } else {
                m_showFilletPrompt = true;
                m_showChamferPrompt = false;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Create Fillet Feature:\n- Previews rounded model corner edge or vertex live with confirm/cancel.\n- Click on any model corner edge or vertex in 3D viewport, then apply Fillet.");
        }

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.36f, 0.24f, 0.44f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.48f, 0.32f, 0.58f, 1.0f));
        if (ImGui::Button("⟍ Chamfer", ImVec2(90, 32))) {
            CloseMeasurement();
            if (m_selectedEdgeIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                auto chamfer = std::make_shared<ChamferNode>(name, target, m_selectedEdgeIndex, m_newChamferDistance);
                if (m_selectedCircle.isCircle && m_selectedCircle.segments.size() > 1) {
                    const auto& verts = target->GetCachedVertices();
                    const auto& edges = target->GetCachedEdges();
                    std::vector<std::pair<Geom::Point3D, Geom::Point3D>> addEdges;
                    for (int segIdx : m_selectedCircle.segments) {
                        if (segIdx == m_selectedEdgeIndex) continue;
                        size_t e = static_cast<size_t>(segIdx * 2);
                        if (e + 1 < edges.size()) {
                            uint32_t i0 = edges[e];
                            uint32_t i1 = edges[e + 1];
                            if (i0 < verts.size() && i1 < verts.size()) {
                                addEdges.push_back({
                                    Geom::Point3D(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]),
                                    Geom::Point3D(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2])
                                });
                            }
                        }
                    }
                    chamfer->SetAdditionalEdges(addEdges);
                }
                StartFeaturePreview(PreviewFeatureType::Chamfer, chamfer, target, graph, device, exchange);
            } else if (m_selectedVertexIndex >= 0 && selectedNode) {
                auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
                const auto& edges = target->GetCachedEdges();
                const auto& verts = target->GetCachedVertices();

                ExtrusionData exData;
                Geom::Vector3D extrudeNormal(0.0, 1.0, 0.0);
                if (ExtractExtrusionData(target, exData)) {
                    if (exData.useCustomPlane) {
                        extrudeNormal = exData.customPlane.normal;
                    } else {
                        switch (exData.plane) {
                            case SketchPlane::XZ_Ground: extrudeNormal = Geom::Vector3D(0, 1, 0); break;
                            case SketchPlane::XY_Front:  extrudeNormal = Geom::Vector3D(0, 0, 1); break;
                            case SketchPlane::YZ_Right:  extrudeNormal = Geom::Vector3D(1, 0, 0); break;
                        }
                    }
                }

                VertexCornerEdges corner = CollectVertexCornerEdges(verts, edges, m_selectedVertexPos, extrudeNormal);
                if (corner.primaryEdge >= 0) {
                    std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                    auto chamfer = std::make_shared<ChamferNode>(name, target, corner.primaryEdge, m_newChamferDistance);
                    StartFeaturePreview(PreviewFeatureType::Chamfer, chamfer, target, graph, device, exchange);
                }
            } else {
                m_showChamferPrompt = true;
                m_showFilletPrompt = false;
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Create Chamfer Feature:\n- Previews beveled model corner edge or vertex live with confirm/cancel.\n- Click on any model corner edge or vertex in 3D viewport, then apply Chamfer.");
        }

        // ---------------------------------------------------------------------
        // 3D Measure Tool Button
        // ---------------------------------------------------------------------
        ImGui::SameLine();
        if (m_measureToolActive) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.52f, 0.72f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.62f, 0.84f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.26f, 0.34f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.35f, 0.46f, 1.0f));
        }
        if (ImGui::Button(m_measureToolActive ? "📏 Measuring [M]" : "📏 Measure [M]", ImVec2(120, 32))) {
            ToggleMeasureTool();
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("3D Measure Tool (Shortcut: M):\n- Click any two points/vertices on 3D models to calculate\n  direct 3D distance, orthogonal X/Y/Z deltas, and angles.");
        }

        // ---------------------------------------------------------------------
        // Export Model to STP (STEP ISO 10303-21 AP214) Button
        // ---------------------------------------------------------------------
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.24f, 0.35f, 0.46f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.32f, 0.46f, 0.60f, 1.0f));
        if (ImGui::Button("💾 Export STP", ImVec2(110, 32))) {
            CloseMeasurement();
            m_showExportStepModal = true;
            m_exportStepStatus.clear();
            std::string defFile = (selectedNode && selectedNode->IsSolidFeature()) ? (selectedNode->GetName() + ".stp") : "model.stp";
            try {
                std::string absPath = std::filesystem::absolute(defFile).string();
                std::snprintf(m_exportStepPath, sizeof(m_exportStepPath), "%s", absPath.c_str());
            } catch (...) {
                std::snprintf(m_exportStepPath, sizeof(m_exportStepPath), "%s", defFile.c_str());
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Export CAD Model to STEP (.stp / .step):\n- Generates industry-standard ISO 10303-21 AP214 B-Rep solid file.\n- Fully compatible with SolidWorks, Fusion 360, FreeCAD, and Siemens NX.");
        }

        // If a 3D feature preview is active, show Confirm & Cancel buttons right in the topbar!
        if (m_featurePreviewActive && m_previewNode) {
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();

            std::string confirmLabel = std::string("✓ Confirm ") + GetPreviewFeatureTypeName(m_previewFeatureType) + "##TopBar";
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.62f, 0.35f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.72f, 0.42f, 1.0f));
            if (ImGui::Button(confirmLabel.c_str(), ImVec2(145, 32))) {
                ConfirmFeaturePreview();
            }
            ImGui::PopStyleColor(2);

            ImGui::SameLine();
            std::string cancelLabel = "✕ Cancel##TopBar";
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.28f, 0.28f, 1.0f));
            if (ImGui::Button(cancelLabel.c_str(), ImVec2(80, 32))) {
                CancelFeaturePreview(graph, device, exchange);
            }
            ImGui::PopStyleColor(2);
        }

        // If 2D Sketch mode is active, always show Confirm / Cancel Sketch right in the topbar as well!
        if (m_sketchModeActive) {
            std::shared_ptr<SketchNode> activeSketch = nullptr;
            for (const auto& n : nodes) {
                if (n && n->GetId() == m_activeSketchId) {
                    activeSketch = std::dynamic_pointer_cast<SketchNode>(n);
                    break;
                }
            }
            bool canConfirm = m_inProgressPoints.empty() && m_inProgressSplineControls.empty();
            for (const auto& sh : m_canvasShapes) {
                if (!sh.isClosed || sh.points.size() < 3) {
                    canConfirm = false;
                    break;
                }
            }

            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();

            ImGui::BeginDisabled(!canConfirm);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.62f, 0.35f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.72f, 0.42f, 1.0f));
            if (ImGui::Button("✓ Confirm Sketch##TopBar", ImVec2(128, 32)) && activeSketch) {
                std::vector<std::vector<std::array<float, 2>>> outShapes;
                for (const auto& sh : m_canvasShapes) {
                    if (sh.isClosed && sh.points.size() >= 3) {
                        outShapes.push_back(sh.points);
                    }
                }
                if (outShapes.empty()) {
                    m_showEmptySketchExitPrompt = true;
                    m_emptySketchToExitId = activeSketch->GetId();
                } else {
                    activeSketch->SetShapes(std::move(outShapes));
                    graph.EvaluateDirtyNodes(device, exchange);
                    m_sketchModeActive = false;
                }
            }
            ImGui::PopStyleColor(2);
            ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.28f, 0.28f, 1.0f));
            if (ImGui::Button("✕ Cancel Sketch##TopBar", ImVec2(118, 32))) {
                if (activeSketch && m_canvasShapes.empty() && activeSketch->GetShapes().empty()) {
                    m_showEmptySketchExitPrompt = true;
                    m_emptySketchToExitId = activeSketch->GetId();
                } else {
                    m_sketchModeActive = false;
                }
            }
            ImGui::PopStyleColor(2);
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
    if (m_selectedNodeId != 0 && (m_selectedNodeId != m_promptInitialSelectedId || m_selectedFaceIndex >= 0 || m_selectedEdgeIndex >= 0 || m_selectedVertexIndex >= 0)) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                auto target = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                if (m_selectedFaceIndex >= 0) {
                    // Clicked on a specific model face: bind associative datum plane and sketch directly on the face surface!
                    FaceInfo faceInfo;
                    if (target && target->GetFace(m_selectedFaceIndex, faceInfo)) {
                        m_selectedFacePlane = faceInfo.plane;
                    }
                    std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, target, m_selectedFaceIndex, m_selectedFacePlane, 0.0);
                    planeNode->Evaluate(device, exchange);
                    graph.AddNode(planeNode);

                    Geom::Plane effPl = planeNode->GetPlane();
                    std::string name = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetParentFeature(target);
                    newSketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                    newSketch->Evaluate(device, exchange);
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    EnterSketchMode(newSketch->GetId());
                    m_showSketchPlanePrompt = false;
                    return;
                } else if (m_selectedEdgeIndex >= 0) {
                    // Clicked on a model corner edge: bind edge datum plane and sketch!
                    Geom::Plane edgePlane = Geom::Plane::XZ(Geom::Point3D(
                        (m_selectedEdgeP0.x + m_selectedEdgeP1.x) * 0.5,
                        (m_selectedEdgeP0.y + m_selectedEdgeP1.y) * 0.5,
                        (m_selectedEdgeP0.z + m_selectedEdgeP1.z) * 0.5
                    ));
                    DatumPlaneNode::ComputeCornerEdgePlane(target, m_selectedEdgeIndex, edgePlane);
                    std::string planeName = "Plane_CornerEdge_" + std::to_string(m_shapeCounter);
                    auto planeNode = DatumPlaneNode::CreateFromEdge(planeName, target, m_selectedEdgeIndex, edgePlane, 0.0);
                    planeNode->Evaluate(device, exchange);
                    graph.AddNode(planeNode);

                    Geom::Plane effPl = planeNode->GetPlane();
                    std::string name = "Sketch_CornerEdge_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetParentFeature(target);
                    newSketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                    newSketch->Evaluate(device, exchange);
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    EnterSketchMode(newSketch->GetId());
                    m_showSketchPlanePrompt = false;
                    return;
                } else if (m_selectedVertexIndex >= 0) {
                    // Clicked on a model vertex: bind vertex datum plane and sketch centered at vertex!
                    std::string planeName = "Plane_Vertex_" + std::to_string(m_shapeCounter);
                    Geom::Plane pl = Geom::Plane::XZ(m_selectedVertexPos);
                    DatumPlaneNode::ComputeCornerVertexPlane(target, m_selectedVertexIndex, pl);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, target, m_selectedVertexIndex, pl, true, 0.0);
                    planeNode->Evaluate(device, exchange);
                    graph.AddNode(planeNode);

                    Geom::Plane effPl = planeNode->GetPlane();
                    std::string name = "Sketch_Vertex_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetParentFeature(target);
                    newSketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                    newSketch->Evaluate(device, exchange);
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    EnterSketchMode(newSketch->GetId());
                    m_showSketchPlanePrompt = false;
                    return;
                } else if (auto p = std::dynamic_pointer_cast<DatumPlaneNode>(n)) {
                    p->Evaluate(device, exchange);
                    Geom::Plane pl = p->GetPlane();
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, p);
                    if (p->GetTargetFeature()) {
                        newSketch->SetParentFeature(p->GetTargetFeature());
                    }
                    newSketch->SetPosition(static_cast<float>(pl.origin.x), static_cast<float>(pl.origin.y), static_cast<float>(pl.origin.z));
                    newSketch->Evaluate(device, exchange);
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    EnterSketchMode(newSketch->GetId());
                    m_showSketchPlanePrompt = false;
                    return;
                } else if (target && !target->GetFaces().empty()) {
                    // Selected a solid model (e.g. from Model Tree): bind to its primary planar face (#0) rather than origin!
                    const auto& faces = target->GetFaces();
                    std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                    auto planeNode = std::make_shared<DatumPlaneNode>(planeName, target, faces[0].faceIndex, faces[0].plane, 0.0);
                    planeNode->Evaluate(device, exchange);
                    graph.AddNode(planeNode);

                    Geom::Plane effPl = planeNode->GetPlane();
                    std::string name = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                    auto newSketch = std::make_shared<SketchNode>(name, planeNode);
                    newSketch->SetParentFeature(target);
                    newSketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                    newSketch->Evaluate(device, exchange);
                    graph.AddNode(newSketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(newSketch->GetId());
                    EnterSketchMode(newSketch->GetId());
                    m_showSketchPlanePrompt = false;
                    return;
                } else {
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XZ_Ground);
                    sketch->SetPosition(n->GetPosition()[0], n->GetPosition()[1], n->GetPosition()[2]);
                    sketch->Evaluate(device, exchange);
                    graph.AddNode(sketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    SetSelectedNodeId(sketch->GetId());
                    EnterSketchMode(sketch->GetId());
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
            EnterSketchMode(sketch->GetId());
            m_showSketchPlanePrompt = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("⬛ Front Plane (XY)", ImVec2(146, 32))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XY_Front);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            EnterSketchMode(sketch->GetId());
            m_showSketchPlanePrompt = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("⬛ Right Plane (YZ)", ImVec2(146, 32))) {
            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::YZ_Right);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            EnterSketchMode(sketch->GetId());
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
                    EnterSketchMode(newSketch->GetId());
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

std::shared_ptr<DAGNode> CADGui::ResolveTargetSolid(
    const std::shared_ptr<DAGNode>& featureOrSketch,
    const ParametricGraph& graph
) const {
    auto isSolid = [](const std::shared_ptr<DAGNode>& n) -> bool {
        if (!n) return false;
        return !std::dynamic_pointer_cast<SketchNode>(n) &&
               !std::dynamic_pointer_cast<DatumPlaneNode>(n) &&
               !std::dynamic_pointer_cast<DatumAxisNode>(n);
    };

    if (featureOrSketch) {
        if (isSolid(featureOrSketch)) {
            if (featureOrSketch->IsConsumed()) {
                auto leaf = featureOrSketch->GetActiveLeafFeature();
                if (leaf && isSolid(leaf)) return leaf;
            }
            return featureOrSketch;
        }

        if (auto sketch = std::dynamic_pointer_cast<SketchNode>(featureOrSketch)) {
            if (auto p = sketch->GetParentFeature()) {
                if (isSolid(p)) return p->IsConsumed() ? p->GetActiveLeafFeature() : p;
            }
            if (auto dp = sketch->GetDatumPlaneReference()) {
                if (auto t = dp->GetTargetFeature()) {
                    if (isSolid(t)) return t->IsConsumed() ? t->GetActiveLeafFeature() : t;
                }
            }
        }

        if (auto dp = std::dynamic_pointer_cast<DatumPlaneNode>(featureOrSketch)) {
            if (auto t = dp->GetTargetFeature()) {
                if (isSolid(t)) return t->IsConsumed() ? t->GetActiveLeafFeature() : t;
            }
        }
    }

    // Fall back to scanning graph nodes in reverse order for an active (unconsumed) solid
    const auto& nodes = graph.GetNodes();
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
        if (*it && isSolid(*it) && !(*it)->IsConsumed() && (*it) != featureOrSketch) {
            return *it;
        }
    }
    // Or any solid leaf
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
        if (*it && isSolid(*it) && (*it) != featureOrSketch) {
            auto leaf = (*it)->GetActiveLeafFeature();
            return (leaf && leaf != featureOrSketch) ? leaf : *it;
        }
    }
    return nullptr;
}

const char* CADGui::GetPreviewFeatureTypeName(CADGui::PreviewFeatureType type) noexcept {
    switch (type) {
        case PreviewFeatureType::Extrude:    return "Extrude";
        case PreviewFeatureType::CutExtrude: return "Cut Extrude";
        case PreviewFeatureType::Revolve:    return "Revolve";
        case PreviewFeatureType::CutRevolve: return "Cut Revolve";
        case PreviewFeatureType::Fillet:     return "Fillet";
        case PreviewFeatureType::Chamfer:    return "Chamfer";
        case PreviewFeatureType::None:       return "Feature";
    }
    return "Feature";
}

void CADGui::StartFeaturePreview(
    PreviewFeatureType type,
    std::shared_ptr<DAGNode> previewNode,
    std::shared_ptr<DAGNode> target,
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange
) {
    if (!previewNode) return;
    CloseMeasurement();
    if (m_featurePreviewActive && m_previewNode) {
        ConfirmFeaturePreview();
    }

    m_previewSavedSelectionId = m_selectedNodeId;
    m_previewSavedFaceIndex = m_selectedFaceIndex;
    m_previewSavedVertexIndex = m_selectedVertexIndex;
    m_previewSavedEdgeIndex = m_selectedEdgeIndex;

    m_previewFeatureType = type;
    m_previewNode = previewNode;
    m_previewTarget = target;
    m_featurePreviewActive = true;

    graph.AddNode(previewNode);
    graph.EvaluateDirtyNodes(device, exchange);
    SetSelectedNodeId(previewNode->GetId());
    ClearSubElementSelection();

    m_showExtrudePrompt = false;
    m_showRevolvePrompt = false;
    m_showFilletPrompt = false;
    m_showChamferPrompt = false;
}

void CADGui::ConfirmFeaturePreview() {
    m_featurePreviewActive = false;
    m_previewFeatureType = PreviewFeatureType::None;
    m_previewNode = nullptr;
    m_previewTarget = nullptr;
}

void CADGui::CancelFeaturePreview(ParametricGraph& graph, RHI::RHIDevice* device, AtomicMeshExchange& exchange) {
    if (!m_featurePreviewActive || !m_previewNode) return;

    uint64_t prevId = m_previewNode->GetId();
    graph.RemoveNode(prevId);
    graph.EvaluateDirtyNodes(device, exchange);

    if (m_previewTarget && graph.GetNode(m_previewTarget->GetId())) {
        SetSelectedNodeId(m_previewTarget->GetId());
    } else if (m_previewSavedSelectionId != 0 && graph.GetNode(m_previewSavedSelectionId)) {
        SetSelectedNodeId(m_previewSavedSelectionId);
    } else {
        SetSelectedNodeId(0);
    }

    if (m_previewSavedFaceIndex >= 0) {
        m_selectedFaceIndex = m_previewSavedFaceIndex;
    } else if (m_previewSavedEdgeIndex >= 0) {
        m_selectedEdgeIndex = m_previewSavedEdgeIndex;
    } else if (m_previewSavedVertexIndex >= 0) {
        m_selectedVertexIndex = m_previewSavedVertexIndex;
    }

    m_lastPublishedSelectedNodeId = m_selectedNodeId;
    m_lastPublishedSelectedFaceIndex = m_selectedFaceIndex;
    m_lastPublishedSelectedVertexIndex = m_selectedVertexIndex;
    m_lastPublishedSelectedEdgeIndex = m_selectedEdgeIndex;
    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, m_selectedFaceIndex, m_selectedVertexIndex, m_selectedEdgeIndex);

    m_featurePreviewActive = false;
    m_previewFeatureType = PreviewFeatureType::None;
    m_previewNode = nullptr;
    m_previewTarget = nullptr;
}

void CADGui::RenderFeaturePreviewDialog(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange
) {
    if (!m_featurePreviewActive || !m_previewNode) return;

    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantTextInput) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            CancelFeaturePreview(graph, device, exchange);
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
            ConfirmFeaturePreview();
            return;
        }
    }

    ImGui::SetNextWindowPos(ImVec2(390.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(540.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.12f, 0.16f, 0.22f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.20f, 0.70f, 0.95f, 1.0f));

    std::string winTitle = std::string("3D Feature Preview: ") + GetPreviewFeatureTypeName(m_previewFeatureType) + "###FeaturePreviewModal";
    bool isOpen = true;
    if (ImGui::Begin(winTitle.c_str(), &isOpen, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.25f, 0.85f, 1.0f, 1.0f), "● 3D Feature Preview: %s", m_previewNode->GetName().c_str());
        ImGui::TextDisabled("Previewing live in viewport. Confirm to apply chain, or Cancel to discard.");
        ImGui::Separator();
        ImGui::Spacing();

        if (m_previewFeatureType == PreviewFeatureType::Extrude || m_previewFeatureType == PreviewFeatureType::CutExtrude) {
            auto extrude = std::dynamic_pointer_cast<ExtrudeNode>(m_previewNode);
            if (extrude) {
                bool isCut = extrude->IsCut();
                if (ImGui::RadioButton("Add Material (Boss)##ExtPrev", !isCut)) {
                    extrude->SetOperation(ExtrudeOperation::Boss);
                    m_previewFeatureType = PreviewFeatureType::Extrude;
                    float h = extrude->GetHeight();
                    if (h < 0.0f) {
                        h = -h;
                        m_newExtrudeHeight = h;
                        extrude->SetHeight(h);
                    }
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Cut Material (Subtract)##ExtPrev", isCut)) {
                    auto tgt = ResolveTargetSolid(m_previewTarget ? m_previewTarget : extrude->GetSketchReference(), graph);
                    if (tgt) extrude->SetTargetFeature(tgt);
                    extrude->SetOperation(ExtrudeOperation::Cut);
                    m_previewFeatureType = PreviewFeatureType::CutExtrude;
                    float h = extrude->GetHeight();
                    if (h > 0.0f) {
                        h = -h;
                        m_newExtrudeHeight = h;
                        extrude->SetHeight(h);
                    }
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::Spacing();

                float h = extrude->GetHeight();
                const char* depthLabel = isCut ? "Cut Depth:" : "Extrude Depth:";
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%s", depthLabel);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(150.0f);
                std::string inputId = std::string("##Depth_") + (isCut ? "Cut" : "Extrude");
                if (ImGui::InputFloat(inputId.c_str(), &h, 0.1f, 1.0f, "%.2f m")) {
                    if (std::abs(h) < 0.001f) h = (h < 0.0f ? -0.01f : 0.01f);
                    m_newExtrudeHeight = h;
                    extrude->SetHeight(h);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::Button("⇄ Flip", ImVec2(80.0f, 0.0f))) {
                    h = -h;
                    m_newExtrudeHeight = h;
                    extrude->SetHeight(h);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (isCut) {
                    ImGui::SameLine();
                    bool wireframe = m_showWireframe;
                    if (ImGui::Checkbox("X-Ray Wireframe (See-through Model)##ExtCutPrev", &wireframe)) {
                        m_showWireframe = wireframe;
                    }
                    if (auto tgt = extrude->GetTargetFeature()) {
                        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "Cutting from Target: %s", tgt->GetName().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "No target model specified to cut from.");
                    }
                }
            }
        } else if (m_previewFeatureType == PreviewFeatureType::Chamfer) {
            auto chamfer = std::dynamic_pointer_cast<ChamferNode>(m_previewNode);
            if (chamfer) {
                float d = chamfer->GetDistance();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Chamfer Distance:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(150.0f);
                if (ImGui::InputFloat("##ChamferDist", &d, 0.05f, 0.2f, "%.2f m")) {
                    if (d < 0.01f) d = 0.01f;
                    m_newChamferDistance = d;
                    chamfer->SetDistance(d);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }
        } else if (m_previewFeatureType == PreviewFeatureType::Fillet) {
            auto fillet = std::dynamic_pointer_cast<FilletNode>(m_previewNode);
            if (fillet) {
                bool isDist = (m_filletChoice == FilletDimensionChoice::Distance);
                if (ImGui::RadioButton("Distance##PrevFilletChoice", isDist)) {
                    m_filletChoice = FilletDimensionChoice::Distance;
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Distance);
                    fillet->SetRadius(m_newFilletRadius);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Angle##PrevFilletChoice", !isDist)) {
                    m_filletChoice = FilletDimensionChoice::Angle;
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
                    fillet->SetAngle(m_newFilletAngle);
                    float effRadius = 0.25f * std::tan(m_newFilletAngle * 3.14159265f / 360.0f);
                    fillet->SetRadius(effRadius);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (m_filletChoice == FilletDimensionChoice::Distance) {
                    float r = fillet->GetRadius();
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("Fillet Radius:");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(150.0f);
                    if (ImGui::InputFloat("##FilletRadius", &r, 0.05f, 0.2f, "%.2f m")) {
                        if (r < 0.01f) r = 0.01f;
                        m_newFilletRadius = r;
                        fillet->SetRadius(r);
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
                } else {
                    float a = fillet->GetAngle();
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("Fillet Angle:");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(150.0f);
                    if (ImGui::InputFloat("##FilletAngle", &a, 1.0f, 5.0f, "%.1f °")) {
                        a = std::clamp(a, 5.0f, 175.0f);
                        m_newFilletAngle = a;
                        fillet->SetAngle(a);
                        float effRadius = 0.25f * std::tan(a * 3.14159265f / 360.0f);
                        fillet->SetRadius(effRadius);
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
                }
            }
        } else if (m_previewFeatureType == PreviewFeatureType::Revolve || m_previewFeatureType == PreviewFeatureType::CutRevolve) {
            auto revolve = std::dynamic_pointer_cast<RevolveNode>(m_previewNode);
            if (revolve) {
                bool isCut = revolve->IsCut();
                if (ImGui::RadioButton("Add Material (Boss)##RevPrev", !isCut)) {
                    revolve->SetOperation(RevolveOperation::Boss);
                    m_previewFeatureType = PreviewFeatureType::Revolve;
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Cut Material (Subtract)##RevPrev", isCut)) {
                    auto tgt = ResolveTargetSolid(m_previewTarget ? m_previewTarget : revolve->GetSketchReference(), graph);
                    if (tgt) revolve->SetTargetFeature(tgt);
                    revolve->SetOperation(RevolveOperation::Cut);
                    m_previewFeatureType = PreviewFeatureType::CutRevolve;
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::Spacing();

                float ang = revolve->GetAngleDegrees();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Revolve Angle:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(150.0f);
                if (ImGui::InputFloat("##RevolveAngle", &ang, 1.0f, 15.0f, "%.1f deg")) {
                    ang = std::clamp(ang, 0.1f, 360.0f);
                    m_newRevolveAngle = ang;
                    revolve->SetAngleDegrees(ang);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                int currentMode = static_cast<int>(revolve->GetAxisMode());
                const char* axisModes[] = {
                    "Sketch V-Axis (Vertical)",
                    "Sketch U-Axis (Horizontal)",
                    "World Y-Axis (0, 1, 0)",
                    "World X-Axis (1, 0, 0)",
                    "World Z-Axis (0, 0, 1)"
                };
                if (ImGui::Combo("Axis Mode", &currentMode, axisModes, IM_ARRAYSIZE(axisModes))) {
                    revolve->SetAxisMode(static_cast<RevolveAxisMode>(currentMode));
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (isCut) {
                    ImGui::SameLine();
                    bool wireframe = m_showWireframe;
                    if (ImGui::Checkbox("X-Ray Wireframe (See-through Model)##RevCutPrev", &wireframe)) {
                        m_showWireframe = wireframe;
                    }
                    if (auto tgt = revolve->GetTargetFeature()) {
                        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "Cutting from Target: %s", tgt->GetName().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "No target model specified to cut from.");
                    }
                }
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.62f, 0.35f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.72f, 0.42f, 1.0f));
        std::string confirmBtnText = std::string("✓ Confirm ") + GetPreviewFeatureTypeName(m_previewFeatureType) + " (Enter)";
        if (ImGui::Button(confirmBtnText.c_str(), ImVec2(200, 32))) {
            ConfirmFeaturePreview();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.60f, 0.20f, 0.20f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.26f, 0.26f, 1.0f));
        if (ImGui::Button("✕ Cancel (Esc)", ImVec2(140, 32))) {
            CancelFeaturePreview(graph, device, exchange);
        }
        ImGui::PopStyleColor(2);
    }
    if (!isOpen) {
        CancelFeaturePreview(graph, device, exchange);
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

    // Check if user selected an available sketch from the Model Tree or viewport while prompt was open
    if (m_selectedNodeId != 0) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    if (!s->IsConsumed()) {
                        std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                        auto targetSolid = ResolveTargetSolid(s, graph);
                        auto extrude = std::make_shared<ExtrudeNode>(name, s, m_newExtrudeHeight, ExtrudeOperation::Boss, targetSolid);
                        StartFeaturePreview(PreviewFeatureType::Extrude, extrude, targetSolid, graph, device, exchange);
                        m_showExtrudePrompt = false;
                        return;
                    }
                }
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 270.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(540.0f, 0.0f), ImGuiCond_Always);

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
                if (!s->IsConsumed()) {
                    availableSketches.push_back(s);
                }
            }
        }

        if (!availableSketches.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Available Sketches in Model Tree:");
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Extrude Depth:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::InputFloat("##PromptExtrudeDepth", &m_newExtrudeHeight, 0.1f, 1.0f, "%.2f m")) {
                if (std::abs(m_newExtrudeHeight) < 0.001f) m_newExtrudeHeight = (m_newExtrudeHeight < 0.0f ? -0.01f : 0.01f);
            }
            ImGui::SameLine();
            if (ImGui::Button("⇄ Flip##PromptExtrudeFlip", ImVec2(80.0f, 0.0f))) {
                m_newExtrudeHeight = -m_newExtrudeHeight;
            }
            ImGui::Spacing();

            for (const auto& s : availableSketches) {
                std::string btnLabel = "⤊ Extrude Sketch: '" + s->GetName() + "' (ID " + std::to_string(s->GetId()) + ", " + std::to_string(s->GetProfile().size()) + " pts)";
                if (ImGui::Button(btnLabel.c_str(), ImVec2(-1, 30))) {
                    std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                    auto targetSolid = ResolveTargetSolid(s, graph);
                    auto extrude = std::make_shared<ExtrudeNode>(name, s, m_newExtrudeHeight, ExtrudeOperation::Boss, targetSolid);
                    StartFeaturePreview(PreviewFeatureType::Extrude, extrude, targetSolid, graph, device, exchange);
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
            std::shared_ptr<SketchNode> sketch = nullptr;
            if (m_selectedFaceIndex >= 0 && m_selectedNodeId != 0) {
                for (const auto& n : nodes) {
                    if (n->GetId() == m_selectedNodeId) {
                        auto target = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                        FaceInfo faceInfo;
                        if (target && target->GetFace(m_selectedFaceIndex, faceInfo)) {
                            m_selectedFacePlane = faceInfo.plane;
                        }
                        std::string planeName = "Plane_Face_" + std::to_string(m_shapeCounter);
                        auto planeNode = std::make_shared<DatumPlaneNode>(planeName, target, m_selectedFaceIndex, m_selectedFacePlane, 0.0);
                        planeNode->Evaluate(device, exchange);
                        graph.AddNode(planeNode);

                        Geom::Plane effPl = planeNode->GetPlane();
                        std::string name = "Sketch_Face_" + std::to_string(m_shapeCounter++);
                        sketch = std::make_shared<SketchNode>(name, planeNode);
                        sketch->SetParentFeature(target);
                        sketch->SetPosition(static_cast<float>(effPl.origin.x), static_cast<float>(effPl.origin.y), static_cast<float>(effPl.origin.z));
                        break;
                    }
                }
            }
            if (!sketch) {
                std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                sketch = std::make_shared<SketchNode>(name, SketchPlane::XZ_Ground);
            }
            sketch->Evaluate(device, exchange);
            graph.AddNode(sketch);
            graph.EvaluateDirtyNodes(device, exchange);
            SetSelectedNodeId(sketch->GetId());
            EnterSketchMode(sketch->GetId());
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

void CADGui::RenderCutExtrudePromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showCutExtrudePrompt) return;

    std::shared_ptr<DAGNode> target = ResolveTargetSolid(nullptr, graph);

    if (m_selectedNodeId != 0) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    if (!s->IsConsumed() && target) {
                        std::string name = "Cut_Extrude_" + std::to_string(m_shapeCounter++);
                        auto extrude = std::make_shared<ExtrudeNode>(name, s, m_newExtrudeHeight, ExtrudeOperation::Cut, target);
                        StartFeaturePreview(PreviewFeatureType::CutExtrude, extrude, target, graph, device, exchange);
                        m_showCutExtrudePrompt = false;
                        return;
                    }
                }
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 270.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(540.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15f, 0.12f, 0.13f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.85f, 0.35f, 0.30f, 1.0f));

    if (ImGui::Begin("Cut Extrude Feature - Select Reference Sketch##Modal", &m_showCutExtrudePrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "⤋ Cut Extrude (Subtract Material)");
        ImGui::Separator();
        if (target) {
            ImGui::TextColored(ImVec4(0.2f, 0.85f, 1.0f, 1.0f), "Target Model: '%s' (ID %llu)", target->GetName().c_str(), target->GetId());
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Warning: No solid model exists in scene to cut away material from.");
        }
        ImGui::Spacing();

        std::vector<std::shared_ptr<SketchNode>> availableSketches;
        for (const auto& n : nodes) {
            if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                if (!s->IsConsumed()) {
                    availableSketches.push_back(s);
                }
            }
        }

        if (!availableSketches.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Available Sketches in Model Tree:");
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Cut Depth:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::InputFloat("##PromptCutDepth", &m_newExtrudeHeight, 0.1f, 1.0f, "%.2f m")) {
                if (std::abs(m_newExtrudeHeight) < 0.001f) m_newExtrudeHeight = (m_newExtrudeHeight < 0.0f ? -0.01f : 0.01f);
            }
            ImGui::SameLine();
            if (ImGui::Button("⇄ Flip##PromptCutFlip", ImVec2(80.0f, 0.0f))) {
                m_newExtrudeHeight = -m_newExtrudeHeight;
            }
            ImGui::Spacing();

            for (const auto& s : availableSketches) {
                std::string btnLabel = "⤋ Cut with Sketch: '" + s->GetName() + "' (ID " + std::to_string(s->GetId()) + ")";
                if (ImGui::Button(btnLabel.c_str(), ImVec2(-1, 30))) {
                    if (target) {
                        std::string name = "Cut_Extrude_" + std::to_string(m_shapeCounter++);
                        auto extrude = std::make_shared<ExtrudeNode>(name, s, m_newExtrudeHeight, ExtrudeOperation::Cut, target);
                        StartFeaturePreview(PreviewFeatureType::CutExtrude, extrude, target, graph, device, exchange);
                        m_showCutExtrudePrompt = false;
                        break;
                    }
                }
            }
            ImGui::Spacing();
        } else {
            ImGui::TextDisabled("No available (unconsumed) 2D sketches exist in the model tree.");
            ImGui::Spacing();
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showCutExtrudePrompt = false;
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

    if (m_selectedNodeId != 0) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    if (!s->IsConsumed()) {
                        std::string name = "Revolve_" + std::to_string(m_shapeCounter++);
                        auto targetSolid = ResolveTargetSolid(s, graph);
                        auto revolve = std::make_shared<RevolveNode>(name, s, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis, RevolveOperation::Boss, targetSolid);
                        StartFeaturePreview(PreviewFeatureType::Revolve, revolve, targetSolid, graph, device, exchange);
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
                if (!s->IsConsumed()) {
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
                    auto targetSolid = ResolveTargetSolid(s, graph);
                    auto revolve = std::make_shared<RevolveNode>(name, s, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis, RevolveOperation::Boss, targetSolid);
                    StartFeaturePreview(PreviewFeatureType::Revolve, revolve, targetSolid, graph, device, exchange);
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
            EnterSketchMode(sketch->GetId());
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

void CADGui::RenderCutRevolvePromptModal(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    const std::vector<std::shared_ptr<DAGNode>>& nodes
) {
    if (!m_showCutRevolvePrompt) return;

    std::shared_ptr<DAGNode> target = ResolveTargetSolid(nullptr, graph);

    if (m_selectedNodeId != 0) {
        for (const auto& n : nodes) {
            if (n->GetId() == m_selectedNodeId) {
                if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                    if (!s->IsConsumed() && target) {
                        std::string name = "Cut_Revolve_" + std::to_string(m_shapeCounter++);
                        auto revolve = std::make_shared<RevolveNode>(name, s, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis, RevolveOperation::Cut, target);
                        StartFeaturePreview(PreviewFeatureType::CutRevolve, revolve, target, graph, device, exchange);
                        m_showCutRevolvePrompt = false;
                        return;
                    }
                }
            }
        }
    }

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f - 240.0f, 68.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.16f, 0.12f, 0.14f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.85f, 0.35f, 0.40f, 1.0f));

    if (ImGui::Begin("Cut Revolve Feature - Select Reference Sketch##Modal", &m_showCutRevolvePrompt, ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f), "⤋ Cut Revolve (Subtract Material via Revolution)");
        ImGui::Separator();
        if (target) {
            ImGui::TextColored(ImVec4(0.2f, 0.85f, 1.0f, 1.0f), "Target Model: '%s' (ID %llu)", target->GetName().c_str(), target->GetId());
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Warning: No solid model exists in scene to cut away material from.");
        }
        ImGui::Spacing();

        std::vector<std::shared_ptr<SketchNode>> availableSketches;
        for (const auto& n : nodes) {
            if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                if (!s->IsConsumed()) {
                    availableSketches.push_back(s);
                }
            }
        }

        if (!availableSketches.empty()) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Available Sketches in Model Tree:");
            ImGui::InputFloat("Revolve Angle", &m_newRevolveAngle, 1.0f, 15.0f, "%.1f deg");
            m_newRevolveAngle = std::clamp(m_newRevolveAngle, 0.1f, 360.0f);
            ImGui::Spacing();

            for (const auto& s : availableSketches) {
                std::string btnLabel = "⤋ Cut Revolve with Sketch: '" + s->GetName() + "' (ID " + std::to_string(s->GetId()) + ")";
                if (ImGui::Button(btnLabel.c_str(), ImVec2(-1, 30))) {
                    if (target) {
                        std::string name = "Cut_Revolve_" + std::to_string(m_shapeCounter++);
                        auto revolve = std::make_shared<RevolveNode>(name, s, m_newRevolveAngle, RevolveAxisMode::Sketch_V_Axis, RevolveOperation::Cut, target);
                        StartFeaturePreview(PreviewFeatureType::CutRevolve, revolve, target, graph, device, exchange);
                        m_showCutRevolvePrompt = false;
                        break;
                    }
                }
            }
            ImGui::Spacing();
        } else {
            ImGui::TextDisabled("No available (unconsumed) 2D sketches exist in the model tree.");
            ImGui::Spacing();
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(-1, 26))) {
            m_showCutRevolvePrompt = false;
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
                auto target = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                float effRadius = (m_filletChoice == FilletDimensionChoice::Angle)
                    ? (0.25f * std::tan(m_newFilletAngle * 3.14159265f / 360.0f))
                    : m_newFilletRadius;
                auto fillet = std::make_shared<FilletNode>(name, target, m_selectedEdgeIndex, effRadius);
                if (m_selectedCircle.isCircle && m_selectedCircle.segments.size() > 1) {
                    const auto& verts = target->GetCachedVertices();
                    const auto& edges = target->GetCachedEdges();
                    std::vector<std::pair<Geom::Point3D, Geom::Point3D>> addEdges;
                    for (int segIdx : m_selectedCircle.segments) {
                        if (segIdx == m_selectedEdgeIndex) continue;
                        size_t e = static_cast<size_t>(segIdx * 2);
                        if (e + 1 < edges.size()) {
                            uint32_t i0 = edges[e];
                            uint32_t i1 = edges[e + 1];
                            if (i0 < verts.size() && i1 < verts.size()) {
                                addEdges.push_back({
                                    Geom::Point3D(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]),
                                    Geom::Point3D(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2])
                                });
                            }
                        }
                    }
                    fillet->SetAdditionalEdges(addEdges);
                }
                if (m_filletChoice == FilletDimensionChoice::Angle) {
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
                    fillet->SetAngle(m_newFilletAngle);
                }
                StartFeaturePreview(PreviewFeatureType::Fillet, fillet, target, graph, device, exchange);
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

        // Choice for Distance or Angle
        if (ImGui::RadioButton("Distance##ModalFilletChoice", m_filletChoice == FilletDimensionChoice::Distance)) {
            m_filletChoice = FilletDimensionChoice::Distance;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Angle##ModalFilletChoice", m_filletChoice == FilletDimensionChoice::Angle)) {
            m_filletChoice = FilletDimensionChoice::Angle;
        }

        if (m_filletChoice == FilletDimensionChoice::Distance) {
            ImGui::InputFloat("Fillet Radius / Distance", &m_newFilletRadius, 0.05f, 0.2f, "%.2f m");
            if (m_newFilletRadius < 0.01f) m_newFilletRadius = 0.01f;
        } else {
            ImGui::InputFloat("Fillet Angle", &m_newFilletAngle, 1.0f, 5.0f, "%.1f °");
            m_newFilletAngle = std::clamp(m_newFilletAngle, 5.0f, 175.0f);
        }
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
                auto target = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                std::string name = "Fillet_" + std::to_string(m_shapeCounter++);
                float effRadius = (m_filletChoice == FilletDimensionChoice::Angle)
                    ? (0.25f * std::tan(m_newFilletAngle * 3.14159265f / 360.0f))
                    : m_newFilletRadius;
                auto fillet = std::make_shared<FilletNode>(name, target, 0, effRadius);
                if (m_filletChoice == FilletDimensionChoice::Angle) {
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
                    fillet->SetAngle(m_newFilletAngle);
                }
                StartFeaturePreview(PreviewFeatureType::Fillet, fillet, target, graph, device, exchange);
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
                auto target = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                auto chamfer = std::make_shared<ChamferNode>(name, target, m_selectedEdgeIndex, m_newChamferDistance);
                if (m_selectedCircle.isCircle && m_selectedCircle.segments.size() > 1) {
                    const auto& verts = target->GetCachedVertices();
                    const auto& edges = target->GetCachedEdges();
                    std::vector<std::pair<Geom::Point3D, Geom::Point3D>> addEdges;
                    for (int segIdx : m_selectedCircle.segments) {
                        if (segIdx == m_selectedEdgeIndex) continue;
                        size_t e = static_cast<size_t>(segIdx * 2);
                        if (e + 1 < edges.size()) {
                            uint32_t i0 = edges[e];
                            uint32_t i1 = edges[e + 1];
                            if (i0 < verts.size() && i1 < verts.size()) {
                                addEdges.push_back({
                                    Geom::Point3D(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]),
                                    Geom::Point3D(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2])
                                });
                            }
                        }
                    }
                    chamfer->SetAdditionalEdges(addEdges);
                }
                StartFeaturePreview(PreviewFeatureType::Chamfer, chamfer, target, graph, device, exchange);
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
                auto target = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                std::string name = "Chamfer_" + std::to_string(m_shapeCounter++);
                auto chamfer = std::make_shared<ChamferNode>(name, target, 0, m_newChamferDistance);
                StartFeaturePreview(PreviewFeatureType::Chamfer, chamfer, target, graph, device, exchange);
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
            } else if (plane->IsEdgeAssociative()) {
                if (m_selectedEdgeIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != plane->GetId()) {
                    std::shared_ptr<DAGNode> selModel = nullptr;
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) { selModel = n; break; }
                    }
                    if (selModel) {
                        std::string reattachLabel = "Re-attach to Selected Edge #" + std::to_string(m_selectedEdgeIndex) + " on '" + selModel->GetName() + "'";
                        if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 30))) {
                            Geom::Plane edgePl = Geom::Plane::XZ(m_selectedEdgeP0);
                            DatumPlaneNode::ComputeCornerEdgePlane(selModel, m_selectedEdgeIndex, edgePl);
                            plane->SetEdgeReference(selModel, m_selectedEdgeIndex, edgePl);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_showConflictPrompt = false;
                        }
                    }
                } else {
                    ImGui::TextDisabled("(To re-attach: select a corner edge on any 3D model in viewport)");
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
                            Geom::Plane vertPl = Geom::Plane::XZ(m_selectedVertexPos);
                            DatumPlaneNode::ComputeCornerVertexPlane(selModel, m_selectedVertexIndex, vertPl);
                            plane->SetVertexReference(selModel, m_selectedVertexIndex, vertPl);
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
            if (axis->IsFaceAssociative()) {
                if (m_selectedFaceIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != axis->GetId()) {
                    std::shared_ptr<DAGNode> selModel = nullptr;
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) { selModel = n; break; }
                    }
                    if (selModel) {
                        std::string reattachLabel = "Re-attach to Selected Face #" + std::to_string(m_selectedFaceIndex) + " on '" + selModel->GetName() + "'";
                        if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 30))) {
                            axis->SetFaceReference(selModel, m_selectedFaceIndex, m_selectedFacePlane.normal, axis->GetOffset());
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_showConflictPrompt = false;
                        }
                    }
                } else {
                    ImGui::TextDisabled("(To re-attach: select a face on any 3D model in viewport)");
                }
            } else if (axis->IsEdgeAssociative()) {
                if (m_selectedEdgeIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != axis->GetId()) {
                    std::shared_ptr<DAGNode> selModel = nullptr;
                    for (const auto& n : nodes) {
                        if (n->GetId() == m_selectedNodeId) { selModel = n; break; }
                    }
                    if (selModel) {
                        std::string reattachLabel = "Re-attach to Selected Edge #" + std::to_string(m_selectedEdgeIndex) + " on '" + selModel->GetName() + "'";
                        if (ImGui::Button(reattachLabel.c_str(), ImVec2(-1, 30))) {
                            axis->SetEdgeReference(selModel, m_selectedEdgeIndex, axis->GetOffset(), true);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_showConflictPrompt = false;
                        }
                    }
                } else {
                    ImGui::TextDisabled("(To re-attach: select a corner edge on any 3D model in viewport)");
                }
            } else if (m_selectedVertexIndex >= 0 && m_selectedNodeId != 0 && m_selectedNodeId != axis->GetId()) {
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
        ImGui::TextWrapped("This 2D sketch contains no shapes. Empty 2D sketches cannot be created and will be deleted from the model tree. Do you wish to delete it?");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.28f, 0.28f, 1.0f));
        if (ImGui::Button("🗑 Delete Empty Sketch", ImVec2(160, 30))) {
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

void CADGui::EnforceShapeConstraints(CanvasShape& shape, int pinnedVertexIdx) {
    SketchNode::EnforceConstraints(shape.points, shape.edgeConstraints, shape.isClosed, pinnedVertexIdx);
}

void CADGui::RenderDimensionEditModal() {
    if (!m_showDimensionEditPopup) return;
    if (m_dimEditShapeIdx < 0 || m_dimEditShapeIdx >= static_cast<int>(m_canvasShapes.size())) {
        m_showDimensionEditPopup = false;
        return;
    }
    auto& shape = m_canvasShapes[m_dimEditShapeIdx];
    size_t n = shape.points.size();
    if (m_dimEditEdgeIdx < 0 || m_dimEditEdgeIdx >= static_cast<int>(n)) {
        m_showDimensionEditPopup = false;
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(m_dimEditPopupPos[0] + 8.0f, m_dimEditPopupPos[1] - 35.0f), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(350.0f, 0.0f), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.12f, 0.15f, 0.20f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.95f, 0.70f, 0.15f, 1.0f));

    bool isOpen = true;
    if (ImGui::Begin("Edit Dimension Constraint###DimEditModal", &isOpen, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "📐 Line Dimension (Edge #%d of '%s'):", m_dimEditEdgeIdx, shape.name.c_str());
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::IsWindowAppearing()) {
            ImGui::SetKeyboardFocusHere();
        }
        ImGui::SetNextItemWidth(130.0f);
        bool enterPressed = ImGui::InputFloat("Dimension (Length)##DimValInput", &m_dimEditValue, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
        if (m_dimEditValue < 0.01f) m_dimEditValue = 0.01f;

        ImGui::Spacing();
        ImGui::Checkbox("Lock & Constrain Dimension (🔒)", &m_dimEditConstrained);
        ImGui::TextDisabled("Constrained dimensions will not change when modifying\nother dimensions or dragging elements.");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.62f, 0.35f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.72f, 0.42f, 1.0f));
        bool apply = ImGui::Button("✓ Apply (Enter)", ImVec2(130, 28)) || enterPressed;
        ImGui::PopStyleColor(2);

        ImGui::SameLine();
        if (ImGui::Button("✕ Cancel (Esc)", ImVec2(100, 28)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            m_showDimensionEditPopup = false;
        }

        if (apply) {
            size_t i0 = m_dimEditEdgeIdx;
            size_t i1 = (i0 + 1) % n;
            float dU = shape.points[i1][0] - shape.points[i0][0];
            float dV = shape.points[i1][1] - shape.points[i0][1];
            float curLen = std::hypot(dU, dV);
            float ux = (curLen > 1e-6f) ? (dU / curLen) : 1.0f;
            float uy = (curLen > 1e-6f) ? (dV / curLen) : 0.0f;
            float deltaL = m_dimEditValue - curLen;

            if (shape.isClosed && shape.points.size() == 4) {
                size_t i2 = (i0 + 2) % 4;
                shape.points[i1][0] += ux * deltaL;
                shape.points[i1][1] += uy * deltaL;
                shape.points[i2][0] += ux * deltaL;
                shape.points[i2][1] += uy * deltaL;
            } else {
                shape.points[i1][0] = shape.points[i0][0] + ux * m_dimEditValue;
                shape.points[i1][1] = shape.points[i0][1] + uy * m_dimEditValue;
            }

            if (m_dimEditConstrained) {
                shape.SetEdgeConstraint(m_dimEditEdgeIdx, m_dimEditValue);
            } else {
                shape.RemoveEdgeConstraint(m_dimEditEdgeIdx);
            }

            EnforceShapeConstraints(shape, static_cast<int>(i0));
            m_showDimensionEditPopup = false;
        }
    }
    if (!isOpen) {
        m_showDimensionEditPopup = false;
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
    RenderCutExtrudePromptModal(graph, device, exchange, nodes);
    RenderRevolvePromptModal(graph, device, exchange, nodes);
    RenderCutRevolvePromptModal(graph, device, exchange, nodes);
    RenderFilletPromptModal(graph, device, exchange, nodes);
    RenderChamferPromptModal(graph, device, exchange, nodes);
    RenderConflictPromptModal(graph, device, exchange, nodes);
    RenderEmptySketchPromptModal(graph, device, exchange);
    RenderFeaturePreviewDialog(graph, device, exchange);
    RenderDimensionEditModal();
    RenderMeasureToolDialog();
    RenderExportStepModal(graph, selectedNode);

    // Close measurement on ESC if active/open and not in feature preview or text input
    {
        ImGuiIO& io = ImGui::GetIO();
        if (!io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            if (!m_featurePreviewActive && !m_showDimensionEditPopup && (m_measureToolActive || m_showMeasureDialog || m_measureHasP1)) {
                CloseMeasurement();
            }
        }
    }

    // Refresh nodes & selectedNode if the top bar or prompt modals added/selected a new node this frame
    nodes = graph.GetNodes();
    selectedNode = nullptr;
    for (const auto& node : nodes) {
        if (node->GetId() == m_selectedNodeId) {
            selectedNode = node;
        }
        if (auto child = node->GetChildSketch()) {
            if (child->GetId() == m_selectedNodeId) {
                selectedNode = child;
            }
        }
    }



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

        // Recursive tree renderer for nodes, child sketches, and child features (Fillets, Chamfers)
        std::function<void(const std::shared_ptr<DAGNode>&, int)> renderNodeTree =
            [&](const std::shared_ptr<DAGNode>& node, int depth) {
                if (!node) return;

                bool isSelected = (node->GetId() == m_selectedNodeId);
                auto childSketch = node->GetChildSketch();

                // Find all child features whose parent feature is this node (excluding childSketch which is rendered directly above)
                std::vector<std::shared_ptr<DAGNode>> childFeatures;
                for (const auto& n : nodes) {
                    if (n && (!childSketch || n->GetId() != childSketch->GetId()) && n->GetParentFeature() == node) {
                        childFeatures.push_back(n);
                    }
                }

                bool hasChildren = (childSketch != nullptr) || !childFeatures.empty();

                std::string prefix = (depth > 0) ? "↳ [Solid] " : "[Solid] ";
                if (m_featurePreviewActive && m_previewNode && node == m_previewNode) {
                    if (auto e = std::dynamic_pointer_cast<ExtrudeNode>(node)) {
                        prefix = (depth > 0) ? (e->IsCut() ? "↳ [PREVIEW: Extrude (Cut)] " : "↳ [PREVIEW: Extrude] ") 
                                             : (e->IsCut() ? "[PREVIEW: Extrude (Cut)] " : "[PREVIEW: Extrude] ");
                    } else if (auto r = std::dynamic_pointer_cast<RevolveNode>(node)) {
                        prefix = (depth > 0) ? (r->IsCut() ? "↳ [PREVIEW: Revolve (Cut)] " : "↳ [PREVIEW: Revolve] ") 
                                             : (r->IsCut() ? "[PREVIEW: Revolve (Cut)] " : "[PREVIEW: Revolve] ");
                    } else {
                        prefix = (depth > 0) ? "↳ [PREVIEW] " : "[PREVIEW] ";
                    }
                } else if (node->HasConflict()) {
                    prefix = (depth > 0) ? "↳ [!] CONFLICT: " : "[!] CONFLICT: ";
                } else if (std::dynamic_pointer_cast<SketchNode>(node)) {
                    if (isSelected) {
                        prefix = (depth > 0) ? "↳ [Sketch ✦ Outline/Shaded] " : "[Sketch ✦ Outline/Shaded] ";
                    } else {
                        prefix = (depth > 0) ? "↳ [Sketch 👁 Hidden] " : "[Sketch 👁 Hidden] ";
                    }
                } else if (auto e = std::dynamic_pointer_cast<ExtrudeNode>(node)) {
                    prefix = (depth > 0) ? (e->IsCut() ? "↳ [Extrude (Cut)] " : "↳ [Extrude] ") : (e->IsCut() ? "[Extrude (Cut)] " : "[Extrude] ");
                } else if (auto r = std::dynamic_pointer_cast<RevolveNode>(node)) {
                    prefix = (depth > 0) ? (r->IsCut() ? "↳ [Revolve (Cut)] " : "↳ [Revolve] ") : (r->IsCut() ? "[Revolve (Cut)] " : "[Revolve] ");
                } else if (auto p = std::dynamic_pointer_cast<DatumPlaneNode>(node)) {
                    if (p->IsFaceAssociative()) prefix = "↳ [DatumPlane (Face Tied)] ";
                    else if (p->IsEdgeAssociative()) prefix = "↳ [DatumPlane (Edge Tied)] ";
                    else if (p->IsVertexAssociative()) prefix = "↳ [DatumPlane (Vertex Tied)] ";
                    else prefix = "↳ [DatumPlane] ";
                } else if (auto a = std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                    if (a->IsFaceAssociative()) prefix = "↳ [DatumAxis (Face Tied)] ";
                    else if (a->IsEdgeAssociative()) prefix = "↳ [DatumAxis (Edge Tied)] ";
                    else if (a->IsVertexAssociative()) prefix = "↳ [DatumAxis (Vertex Tied)] ";
                    else prefix = "↳ [DatumAxis] ";
                } else if (std::dynamic_pointer_cast<FilletNode>(node)) {
                    prefix = (depth > 0) ? "↳ [⌒ Fillet] " : "[⌒ Fillet] ";
                } else if (std::dynamic_pointer_cast<ChamferNode>(node)) {
                    prefix = (depth > 0) ? "↳ [⟍ Chamfer] " : "[⟍ Chamfer] ";
                }

                std::string label = prefix + node->GetName() + " (ID " + std::to_string(node->GetId()) + ")";

                if (hasChildren) {
                    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
                    if (isSelected) {
                        flags |= ImGuiTreeNodeFlags_Selected;
                    }

                    bool nodeOpen = ImGui::TreeNodeEx(reinterpret_cast<void*>(node->GetId()), flags, "%s", label.c_str());
                    if (ImGui::IsItemClicked()) {
                        m_selectedNodeId = node->GetId();
                        ClearSubElementSelection();
                        graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                        graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1, -1);
                    }
                    if (ImGui::BeginPopupContextItem()) {
                        std::string menuDelText = std::dynamic_pointer_cast<SketchNode>(node) ? "🗑 Delete 2D Sketch" : "🗑 Delete Feature";
                        if (ImGui::MenuItem(menuDelText.c_str())) {
                            uint64_t delId = node->GetId();
                            auto child = node->GetChildSketch();
                            auto parent = node->GetParentFeature();
                            if (m_sketchModeActive && m_activeSketchId == delId) m_sketchModeActive = false;
                            graph.RemoveNode(delId);
                            graph.EvaluateDirtyNodes(device, exchange);
                            if (m_selectedNodeId == delId) {
                                if (parent && graph.GetNode(parent->GetId())) m_selectedNodeId = parent->GetId();
                                else if (child && graph.GetNode(child->GetId())) m_selectedNodeId = child->GetId();
                                else m_selectedNodeId = 0;
                                ClearSubElementSelection();
                                graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                                m_lastPublishedSelectedNodeId = m_selectedNodeId;
                                graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                            }
                        }
                        ImGui::EndPopup();
                    }
                    if (isSelected && m_selectionScrollPending) {
                        ImGui::SetScrollHereY(0.5f);
                    }

                    if (nodeOpen) {
                        if (childSketch) {
                            bool isChildSelected = (childSketch->GetId() == m_selectedNodeId);
                            std::string childPrefix = childSketch->HasConflict() ? "↳ [!] CONFLICT: " : "↳ [Sketch] ";
                            std::string childLabel = childPrefix + childSketch->GetName() + " (ID " + std::to_string(childSketch->GetId()) + ")";

                            if (ImGui::Selectable(childLabel.c_str(), isChildSelected)) {
                                m_selectedNodeId = childSketch->GetId();
                                ClearSubElementSelection();
                                graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                                graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1, -1);
                            }
                            if (ImGui::BeginPopupContextItem()) {
                                if (ImGui::MenuItem("🗑 Delete 2D Sketch")) {
                                    uint64_t delId = childSketch->GetId();
                                    auto parent = childSketch->GetParentFeature();
                                    if (m_sketchModeActive && m_activeSketchId == delId) m_sketchModeActive = false;
                                    graph.RemoveNode(delId);
                                    graph.EvaluateDirtyNodes(device, exchange);
                                    if (m_selectedNodeId == delId) {
                                        m_selectedNodeId = (parent && graph.GetNode(parent->GetId())) ? parent->GetId() : 0;
                                        ClearSubElementSelection();
                                        graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                                        m_lastPublishedSelectedNodeId = m_selectedNodeId;
                                        graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                                    }
                                }
                                ImGui::EndPopup();
                            }
                            if (isChildSelected && m_selectionScrollPending) {
                                ImGui::SetScrollHereY(0.5f);
                            }
                        }
                        for (const auto& cf : childFeatures) {
                            renderNodeTree(cf, depth + 1);
                        }
                        ImGui::TreePop();
                    }
                } else {
                    if (ImGui::Selectable(label.c_str(), isSelected)) {
                        m_selectedNodeId = node->GetId();
                        ClearSubElementSelection();
                        graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                        graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1, -1);
                    }
                    if (ImGui::BeginPopupContextItem()) {
                        std::string menuDelText = std::dynamic_pointer_cast<SketchNode>(node) ? "🗑 Delete 2D Sketch" : "🗑 Delete Feature";
                        if (ImGui::MenuItem(menuDelText.c_str())) {
                            uint64_t delId = node->GetId();
                            auto child = node->GetChildSketch();
                            auto parent = node->GetParentFeature();
                            if (m_sketchModeActive && m_activeSketchId == delId) m_sketchModeActive = false;
                            graph.RemoveNode(delId);
                            graph.EvaluateDirtyNodes(device, exchange);
                            if (m_selectedNodeId == delId) {
                                if (parent && graph.GetNode(parent->GetId())) m_selectedNodeId = parent->GetId();
                                else if (child && graph.GetNode(child->GetId())) m_selectedNodeId = child->GetId();
                                else m_selectedNodeId = 0;
                                ClearSubElementSelection();
                                graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                                m_lastPublishedSelectedNodeId = m_selectedNodeId;
                                graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                            }
                        }
                        ImGui::EndPopup();
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
            };

        auto hasParentInNodes = [&](const std::shared_ptr<DAGNode>& n) -> bool {
            if (!n || !n->HasParentFeature()) return false;
            auto p = n->GetParentFeature();
            for (const auto& candidate : nodes) {
                if (candidate == p) return true;
            }
            return false;
        };

        for (const auto& node : nodes) {
            // Child features & child sketches are displayed underneath their parent feature in the hierarchy
            if (hasParentInNodes(node)) {
                continue;
            }
            renderNodeTree(node, 0);
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

            if (m_featurePreviewActive && m_previewNode && selectedNode == m_previewNode) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.35f, 0.75f, 0.45f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.22f, 0.15f, 0.95f));
                ImGui::BeginChild("FeaturePreviewCard", ImVec2(0, 78), true);
                ImGui::TextColored(ImVec4(0.4f, 0.95f, 0.55f, 1.0f), "● PREVIEW MODE: %s", GetPreviewFeatureTypeName(m_previewFeatureType));
                ImGui::TextWrapped("Previewing feature in 3D viewport. Adjust parameters above/below, then confirm or cancel.");
                if (ImGui::Button("✓ Confirm", ImVec2(100, 24))) {
                    ConfirmFeaturePreview();
                }
                ImGui::SameLine();
                if (ImGui::Button("✕ Cancel", ImVec2(80, 24))) {
                    CancelFeaturePreview(graph, device, exchange);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            }

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

            if (m_selectedFaceIndices.size() > 1) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.0f, 0.85f, 1.0f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.16f, 0.22f, 0.9f));
                ImGui::BeginChild("MultiFaceSubElementCard", ImVec2(0, 118), true);
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "● Multi-Plane Selection: %zu Planes Selected", m_selectedFaceIndices.size());
                ImGui::TextDisabled("Hold Shift / Ctrl to add or remove planes");
                for (size_t i = 0; i < std::min(m_selectedFaceIndices.size(), size_t(3)); ++i) {
                    if (i < m_selectedFacePlanes.size()) {
                        const auto& pl = m_selectedFacePlanes[i];
                        ImGui::Text("  Face #%d: N(%.2f, %.2f, %.2f)", m_selectedFaceIndices[i], pl.normal.x, pl.normal.y, pl.normal.z);
                    }
                }
                if (m_selectedFaceIndices.size() > 3) {
                    ImGui::TextDisabled("  ... and %zu more", m_selectedFaceIndices.size() - 3);
                }
                ImGui::Spacing();
                if (ImGui::Button("✕ Clear Multi-Plane Selection", ImVec2(-1, 22))) {
                    ClearSubElementSelection();
                    graph.ClearSubElementSelection();
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            } else if (m_selectedFaceIndex >= 0) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.0f, 0.85f, 1.0f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.16f, 0.22f, 0.9f));
                ImGui::BeginChild("FaceSubElementCard", ImVec2(0, 92), true);
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "● Sub-Element Selected: Face #%d", m_selectedFaceIndex);
                ImGui::Text("Normal: (%.2f, %.2f, %.2f)", m_selectedFacePlane.normal.x, m_selectedFacePlane.normal.y, m_selectedFacePlane.normal.z);
                ImGui::Text("Center: (%.2f, %.2f, %.2f)", m_selectedFacePlane.origin.x, m_selectedFacePlane.origin.y, m_selectedFacePlane.origin.z);
                ImGui::Spacing();

                if (ImGui::Button("✕ Deselect Face (Select Whole Model)", ImVec2(-1, 22))) {
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            } else if (m_selectedVertexIndices.size() > 1) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.2f, 0.9f, 0.4f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.20f, 0.12f, 0.9f));
                ImGui::BeginChild("MultiVertexSubElementCard", ImVec2(0, 195), true);
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "● Multi-Point Selection: %zu Points Selected", m_selectedVertexIndices.size());
                ImGui::TextDisabled("Hold Shift / Ctrl to add or remove points");
                for (size_t i = 0; i < std::min(m_selectedVertexIndices.size(), size_t(3)); ++i) {
                    if (i < m_selectedVertexPositions.size()) {
                        const auto& vp = m_selectedVertexPositions[i];
                        ImGui::Text("  P%zu [Vertex #%d]: (%.2f, %.2f, %.2f)", i + 1, m_selectedVertexIndices[i], vp.x, vp.y, vp.z);
                    }
                }
                if (m_selectedVertexIndices.size() == 2 && m_selectedVertexPositions.size() >= 2) {
                    auto m = Geom::ComputeMeasurement(m_selectedVertexPositions[0], m_selectedVertexPositions[1]);
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 1.0f, 1.0f), "  Direct 3D Distance: %.4f m", m.directDistance);
                    ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.55f, 1.0f), "  ΔX: %.3f", m.absDeltaX);
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.55f, 1.0f, 0.55f, 1.0f), "  ΔY: %.3f", m.absDeltaY);
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.55f, 0.75f, 1.0f, 1.0f), "  ΔZ: %.3f", m.absDeltaZ);
                    ImGui::TextColored(ImVec4(0.85f, 0.95f, 0.50f, 1.0f), "  Incline: %+.1f°  Ground: %+.1f°", m.elevationDeg, m.azimuthDeg);
                    if (ImGui::SmallButton("📏 Open Full Measure Inspector")) {
                        SetMeasurePoint1(m_selectedVertexPositions[0]);
                        SetMeasurePoint2(m_selectedVertexPositions[1]);
                        SetMeasureToolActive(true);
                    }
                }
                if (m_selectedVertexIndices.size() > 3) {
                    ImGui::TextDisabled("  ... and %zu more", m_selectedVertexIndices.size() - 3);
                }
                ImGui::Spacing();
                if (ImGui::Button("✕ Clear Multi-Point Selection", ImVec2(-1, 22))) {
                    ClearSubElementSelection();
                    graph.ClearSubElementSelection();
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1);
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            } else if (m_selectedVertexIndex >= 0) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.2f, 0.9f, 0.4f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.20f, 0.12f, 0.9f));
                ImGui::BeginChild("VertexSubElementCard", ImVec2(0, 74), true);
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "● Sub-Element Selected: Vertex #%d", m_selectedVertexIndex);
                ImGui::Text("Position: (%.2f, %.2f, %.2f)", m_selectedVertexPos.x, m_selectedVertexPos.y, m_selectedVertexPos.z);
                ImGui::Spacing();

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
                if (m_selectedCircle.isCircle) {
                    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.2f, 0.85f, 1.0f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.20f, 0.28f, 0.95f));
                    ImGui::BeginChild("CircleEdgeSubElementCard", ImVec2(0, 168), true);
                    ImGui::TextColored(ImVec4(0.25f, 0.95f, 1.0f, 1.0f), "◯ Circular Edge Loop Selected (%zu Segments)", m_selectedCircle.segments.size());
                    ImGui::Separator();
                    ImGui::Text("Diameter:      Ø %.3f m", m_selectedCircle.diameter);
                    ImGui::Text("Radius:        R %.3f m", m_selectedCircle.radius);
                    ImGui::Text("Circumference: %.3f m", m_selectedCircle.circumference);
                    ImGui::Text("Enclosed Area: %.3f m²", m_selectedCircle.area);
                    ImGui::Text("Center:        (%.2f, %.2f, %.2f)", m_selectedCircle.center.x, m_selectedCircle.center.y, m_selectedCircle.center.z);
                    ImGui::Spacing();

                    if (ImGui::Button("✕ Deselect Circle (Select Whole Model)", ImVec2(-1, 22))) {
                        ClearSubElementSelection();
                        graph.SetSubElementSelection(m_selectedNodeId, -1, -1, -1);
                        graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1, -1);
                    }
                    ImGui::EndChild();
                    ImGui::PopStyleColor(2);
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.75f, 0.2f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.22f, 0.17f, 0.05f, 0.9f));
                    ImGui::BeginChild("EdgeSubElementCard", ImVec2(0, 112), true);
                    ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.3f, 1.0f), "● Sub-Element Selected: Corner Edge #%d", m_selectedEdgeIndex);
                    float edgeLen = static_cast<float>(m_selectedEdgeP0.distance_to(m_selectedEdgeP1));
                    ImGui::Text("Length: %.2f m", edgeLen);
                    ImGui::Text("From: (%.2f, %.2f, %.2f)", m_selectedEdgeP0.x, m_selectedEdgeP0.y, m_selectedEdgeP0.z);
                    ImGui::Text("To:   (%.2f, %.2f, %.2f)", m_selectedEdgeP1.x, m_selectedEdgeP1.y, m_selectedEdgeP1.z);
                    ImGui::Spacing();

                    if (ImGui::Button("✕ Deselect Edge (Select Whole Model)", ImVec2(-1, 22))) {
                        ClearSubElementSelection();
                        graph.SetSubElementSelection(m_selectedNodeId, -1, -1, -1);
                        graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId, -1, -1, -1);
                    }
                    ImGui::EndChild();
                    ImGui::PopStyleColor(2);
                }
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
                        EnterSketchMode(sketch->GetId());
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
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.25f, 0.25f, 1.0f));
                if (ImGui::Button("🗑 Delete 2D Sketch##SketchProps", ImVec2(-1, 26))) {
                    uint64_t delId = sketch->GetId();
                    auto parent = sketch->GetParentFeature();
                    if (m_sketchModeActive && m_activeSketchId == delId) {
                        m_sketchModeActive = false;
                    }
                    graph.RemoveNode(delId);
                    graph.EvaluateDirtyNodes(device, exchange);
                    m_selectedNodeId = (parent && graph.GetNode(parent->GetId())) ? parent->GetId() : 0;
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                    m_lastPublishedSelectedNodeId = m_selectedNodeId;
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                }
                ImGui::PopStyleColor(2);

                if (!sketch->IsConsumed()) {
                    ImGui::Spacing();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.45f, 0.30f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.58f, 0.40f, 1.0f));
                    if (ImGui::Button("⤊ Extrude This Sketch...", ImVec2(-1, 28))) {
                        CloseMeasurement();
                        std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                        auto targetSolid = ResolveTargetSolid(sketch, graph);
                        auto extrude = std::make_shared<ExtrudeNode>(name, sketch, m_newExtrudeHeight, ExtrudeOperation::Boss, targetSolid);
                        StartFeaturePreview(PreviewFeatureType::Extrude, extrude, targetSolid, graph, device, exchange);
                    }
                    ImGui::PopStyleColor(2);
                }
                ImGui::Spacing();

                if (sketch->HasCustomPlane()) {
                    Geom::Plane cpl = sketch->GetCustomPlane();
                    if (auto dp = sketch->GetDatumPlaneReference()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Reference Plane: %s", dp->GetName().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Reference Plane: Custom Face Plane");
                    }
                    ImGui::Text("Plane Origin: (%.2f, %.2f, %.2f)", cpl.origin.x, cpl.origin.y, cpl.origin.z);
                    ImGui::Text("Plane Normal: (%.2f, %.2f, %.2f)", cpl.normal.x, cpl.normal.y, cpl.normal.z);
                    if (ImGui::Button("Detach from Reference Plane", ImVec2(-1, 24))) {
                        sketch->SetPlane(sketch->GetPlane());
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
                } else {
                    int currentPlane = static_cast<int>(sketch->GetPlane());
                    const char* planes[] = { "XZ Ground Plane (Top)", "XY Front Plane", "YZ Right Plane" };
                    if (ImGui::Combo("Datum Plane", &currentPlane, planes, IM_ARRAYSIZE(planes))) {
                        sketch->SetPlane(static_cast<SketchPlane>(currentPlane));
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
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
                bool isCut = extrude->IsCut();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s Feature Properties:", isCut ? "Extrude (Cut)" : "Extrude");
                
                ImGui::Text("Operation:");
                ImGui::SameLine();
                if (ImGui::RadioButton("Boss (Add)##ExtInsp", !isCut)) {
                    extrude->SetOperation(ExtrudeOperation::Boss);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Cut (Subtract)##ExtInsp", isCut)) {
                    if (!extrude->GetTargetFeature()) {
                        std::shared_ptr<DAGNode> tgt = nullptr;
                        if (auto sk = extrude->GetSketchReference()) {
                            if (auto p = sk->GetParentFeature()) tgt = p;
                            else if (auto dp = sk->GetDatumPlaneReference()) tgt = dp->GetTargetFeature();
                        }
                        if (!tgt) {
                            for (const auto& n : graph.GetNodes()) {
                                if (n && n != extrude && n != extrude->GetSketchReference() && !n->IsConsumed()) {
                                    if (!std::dynamic_pointer_cast<SketchNode>(n) && !std::dynamic_pointer_cast<DatumPlaneNode>(n) && !std::dynamic_pointer_cast<DatumAxisNode>(n)) {
                                        tgt = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                                        break;
                                    }
                                }
                            }
                        }
                        if (tgt) extrude->SetTargetFeature(tgt);
                    }
                    extrude->SetOperation(ExtrudeOperation::Cut);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::Spacing();
                
                if (auto refSketch = extrude->GetSketchReference()) {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Linked to Sketch: %s", refSketch->GetName().c_str());
                    if (ImGui::Button("Jump to Reference Sketch", ImVec2(-1, 24))) {
                        m_selectedNodeId = refSketch->GetId();
                    }
                } else {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Standalone Profile (No Sketch Ref)");
                }

                if (isCut) {
                    if (auto tgt = extrude->GetTargetFeature()) {
                        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "Cutting from Target: %s", tgt->GetName().c_str());
                    }
                }

                float h = extrude->GetHeight();
                const char* depthLabel = isCut ? "Cut Depth" : "Extrude Depth";
                if (ImGui::InputFloat(depthLabel, &h, 0.1f, 1.0f, "%.2f m")) {
                    if (std::abs(h) < 0.001f) h = (h < 0.0f ? -0.01f : 0.01f);
                    extrude->SetHeight(h);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }
            // -----------------------------------------------------------------
            // B2. Revolve Node Inspector
            // -----------------------------------------------------------------
            else if (auto revolve = std::dynamic_pointer_cast<RevolveNode>(selectedNode)) {
                bool isCut = revolve->IsCut();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "%s Feature Properties:", isCut ? "Revolve (Cut)" : "Revolve");
                
                ImGui::Text("Operation:");
                ImGui::SameLine();
                if (ImGui::RadioButton("Boss (Add)##RevInsp", !isCut)) {
                    revolve->SetOperation(RevolveOperation::Boss);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Cut (Subtract)##RevInsp", isCut)) {
                    if (!revolve->GetTargetFeature()) {
                        std::shared_ptr<DAGNode> tgt = nullptr;
                        if (auto sk = revolve->GetSketchReference()) {
                            if (auto p = sk->GetParentFeature()) tgt = p;
                            else if (auto dp = sk->GetDatumPlaneReference()) tgt = dp->GetTargetFeature();
                        }
                        if (!tgt) {
                            for (const auto& n : graph.GetNodes()) {
                                if (n && n != revolve && n != revolve->GetSketchReference() && !n->IsConsumed()) {
                                    if (!std::dynamic_pointer_cast<SketchNode>(n) && !std::dynamic_pointer_cast<DatumPlaneNode>(n) && !std::dynamic_pointer_cast<DatumAxisNode>(n)) {
                                        tgt = n->IsConsumed() ? n->GetActiveLeafFeature() : n;
                                        break;
                                    }
                                }
                            }
                        }
                        if (tgt) revolve->SetTargetFeature(tgt);
                    }
                    revolve->SetOperation(RevolveOperation::Cut);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::Spacing();
                
                if (auto refSketch = revolve->GetSketchReference()) {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Linked to Sketch: %s", refSketch->GetName().c_str());
                    if (ImGui::Button("Jump to Reference Sketch", ImVec2(-1, 24))) {
                        m_selectedNodeId = refSketch->GetId();
                    }
                } else {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Standalone Profile (No Sketch Ref)");
                }

                if (isCut) {
                    if (auto tgt = revolve->GetTargetFeature()) {
                        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "Cutting from Target: %s", tgt->GetName().c_str());
                    }
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
                } else if (planeNode->IsEdgeAssociative()) {
                    if (auto parent = planeNode->GetReferencedFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Associative Link: Corner Edge #%d on '%s'", planeNode->GetReferencedEdgeIndex(), parent->GetName().c_str());
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
                    EnterSketchMode(newSketch->GetId());
                }
            } else if (auto axisNode = std::dynamic_pointer_cast<DatumAxisNode>(selectedNode)) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Datum Axis Inspector");
                ImGui::Separator();

                if (axisNode->IsFaceAssociative()) {
                    if (auto parent = axisNode->GetReferencedFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Associative Link: Face #%d on '%s'", axisNode->GetReferencedFaceIndex(), parent->GetName().c_str());
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
                } else if (axisNode->IsEdgeAssociative()) {
                    if (auto parent = axisNode->GetReferencedFeature()) {
                        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Associative Link: Corner Edge #%d on '%s'", axisNode->GetReferencedEdgeIndex(), parent->GetName().c_str());
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
                } else if (axisNode->IsVertexAssociative()) {
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

                float axisOffset = static_cast<float>(axisNode->GetOffset());
                if (ImGui::InputFloat("Axis Offset", &axisOffset, 0.1f, 1.0f, "%.2f m")) {
                    axisNode->SetOffset(axisOffset);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

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

                // Choice between Distance and Angle for FilletNode
                bool isDist = (fillet->GetDimensionChoice() == FilletNode::DimensionChoice::Distance);
                if (ImGui::RadioButton("Distance##FilletNodeChoice", isDist)) {
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Distance);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Angle##FilletNodeChoice", !isDist)) {
                    fillet->SetDimensionChoice(FilletNode::DimensionChoice::Angle);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                if (fillet->GetDimensionChoice() == FilletNode::DimensionChoice::Distance) {
                    float r = fillet->GetRadius();
                    if (ImGui::InputFloat("Fillet Distance / Radius", &r, 0.05f, 0.2f, "%.2f m")) {
                        fillet->SetRadius(r);
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
                } else {
                    float a = fillet->GetAngle();
                    if (ImGui::InputFloat("Fillet Angle", &a, 1.0f, 5.0f, "%.1f °")) {
                        fillet->SetAngle(a);
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
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
            } else if (selectedNode->HasParentFeature()) {
                if (auto parent = selectedNode->GetParentFeature()) {
                    ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Child feature of: '%s'", parent->GetName().c_str());
                }
            }

            if (selectedNode->IsSolidFeature() && !selectedNode->GetCachedVertices().empty()) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.40f, 0.55f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.52f, 0.70f, 1.0f));
                if (ImGui::Button("💾 Export Feature as .STP", ImVec2(-1, 28))) {
                    m_showExportStepModal = true;
                    m_exportStepStatus.clear();
                    std::string defFile = selectedNode->GetName() + ".stp";
                    try {
                        std::string absPath = std::filesystem::absolute(defFile).string();
                        std::snprintf(m_exportStepPath, sizeof(m_exportStepPath), "%s", absPath.c_str());
                    } catch (...) {
                        std::snprintf(m_exportStepPath, sizeof(m_exportStepPath), "%s", defFile.c_str());
                    }
                    m_exportStepScope = 1; // Selected solid only
                }
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            }

            std::string delBtnText = "Delete Selected Object";
            if (std::dynamic_pointer_cast<SketchNode>(selectedNode)) {
                delBtnText = "🗑 Delete 2D Sketch";
            } else if (selectedNode->GetChildSketch()) {
                delBtnText = "🗑 Delete Feature (Promotes Child Sketch)";
            } else {
                delBtnText = "🗑 Delete Feature";
            }

            if (!delBtnText.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.25f, 0.25f, 1.0f));
                if (ImGui::Button(delBtnText.c_str(), ImVec2(-1, 28))) {
                    uint64_t delId = selectedNode->GetId();
                    auto child = selectedNode->GetChildSketch();
                    auto parent = selectedNode->GetParentFeature();
                    if (m_sketchModeActive && m_activeSketchId == delId) {
                        m_sketchModeActive = false;
                    }
                    graph.RemoveNode(delId);
                    graph.EvaluateDirtyNodes(device, exchange);
                    if (parent && graph.GetNode(parent->GetId())) {
                        m_selectedNodeId = parent->GetId();
                    } else if (child && graph.GetNode(child->GetId())) {
                        m_selectedNodeId = child->GetId();
                    } else {
                        m_selectedNodeId = 0;
                    }
                    ClearSubElementSelection();
                    graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                    m_lastPublishedSelectedNodeId = m_selectedNodeId;
                    graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
                }
                ImGui::PopStyleColor(2);
            }

            if (!m_sketchModeActive && !ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
                uint64_t delId = selectedNode->GetId();
                auto child = selectedNode->GetChildSketch();
                auto parent = selectedNode->GetParentFeature();
                if (m_sketchModeActive && m_activeSketchId == delId) {
                    m_sketchModeActive = false;
                }
                graph.RemoveNode(delId);
                graph.EvaluateDirtyNodes(device, exchange);
                if (parent && graph.GetNode(parent->GetId())) {
                    m_selectedNodeId = parent->GetId();
                } else if (child && graph.GetNode(child->GetId())) {
                    m_selectedNodeId = child->GetId();
                } else {
                    m_selectedNodeId = 0;
                }
                ClearSubElementSelection();
                graph.SetSubElementSelection(m_selectedNodeId, -1, -1);
                m_lastPublishedSelectedNodeId = m_selectedNodeId;
                graph.PublishCombinedSceneMesh(device, exchange, m_selectedNodeId);
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
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Camera Presets (Recenter on Origin)");
        ImGui::Separator();

        if (ImGui::Button("Top View", ImVec2(76, 26))) {
            camera.SetTopView(true);
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter on Top View (+Y down onto XZ plane)");
        ImGui::SameLine();
        if (ImGui::Button("Side View", ImVec2(76, 26))) {
            camera.SetSideView(true);
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter on Side View (+X looking towards YZ plane)");
        ImGui::SameLine();
        if (ImGui::Button("Isometric", ImVec2(76, 26))) {
            camera.SetIsometricView(true);
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter on Isometric View (CAD corner angle)");
        ImGui::SameLine();
        if (ImGui::Button("Front View", ImVec2(76, 26))) {
            camera.SetFrontView(true);
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter on Front View (+Z looking towards XY plane)");

        if (ImGui::Button("⌂ Recenter Origin", ImVec2(140, 24))) {
            camera.RecenterOrigin();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter camera target on (0, 0, 0) origin without altering orbit angle");

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Display Options");
        ImGui::Separator();
        ImGui::Checkbox("Sharp Feature Edges / Wireframe", &m_showWireframe);
        ImGui::Checkbox("Infinite Datum Ground Grid", &m_showGrid);
        ImGui::Checkbox("X/Y/Z Axes Reference & Triad", &m_showAxesReference);

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

    // -------------------------------------------------------------------------
    // 3D Viewport Measure Tool Visual Overlay (Direct line, Deltas, Angles, Badges)
    // -------------------------------------------------------------------------
    RenderMeasureVisualOverlay(camera, renderer);

    // -------------------------------------------------------------------------
    // 3D Viewport Cut Feature Preview Visual Overlay (X-Ray Cutter Volume, Pocket Floor & Depth Badge)
    // -------------------------------------------------------------------------
    RenderCutPreviewVisualOverlay(camera, renderer);

    // -------------------------------------------------------------------------
    // 3D Viewport Vertex Selection Highlight (Reticle + Callout Badge for all selected points)
    // -------------------------------------------------------------------------
    if ((!m_selectedVertexIndices.empty() || m_selectedVertexIndex >= 0) && selectedNode && !m_sketchModeActive) {
        const auto& verts = selectedNode->GetCachedVertices();
        float vpW = static_cast<float>(renderer.GetWidth());
        float vpH = static_cast<float>(renderer.GetHeight());
        if (vpW <= 0.0f || vpH <= 0.0f) {
            ImGuiIO& io = ImGui::GetIO();
            vpW = io.DisplaySize.x;
            vpH = io.DisplaySize.y;
        }

        std::vector<int> displayVertIndices = m_selectedVertexIndices;
        if (displayVertIndices.empty() && m_selectedVertexIndex >= 0) {
            displayVertIndices.push_back(m_selectedVertexIndex);
        }

        ImDrawList* fg = ImGui::GetForegroundDrawList();

        for (size_t ptIdx = 0; ptIdx < displayVertIndices.size(); ++ptIdx) {
            int vIdx = displayVertIndices[ptIdx];
            if (vIdx < 0 || static_cast<size_t>(vIdx) >= verts.size()) continue;

            Geom::Point3D vPos(
                verts[vIdx].position[0],
                verts[vIdx].position[1],
                verts[vIdx].position[2]
            );

            float sx = 0.0f, sy = 0.0f;
            if (camera.WorldToScreenPoint(vPos, vpW, vpH, sx, sy)) {
                ImVec2 center(sx, sy);

                // Distinguishable selected 'point' dot:
                // 1. Dark contrasting outer border/halo for crisp visibility against any model geometry
                fg->AddCircleFilled(center, 5.5f, IM_COL32(10, 15, 20, 230), 16);

                // 2. Vibrant selected dot fill (electric cyan CAD selection color)
                fg->AddCircleFilled(center, 4.0f, IM_COL32(0, 225, 255, 255), 16);

                // 3. Crisp bright white center core highlight
                fg->AddCircleFilled(center, 1.8f, IM_COL32(255, 255, 255, 255), 16);
            }
        }
    }

    // Multi-plane selection summary banner
    if (m_selectedFaceIndices.size() > 1 && selectedNode && !m_sketchModeActive) {
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        char planeSumText[128];
        std::snprintf(planeSumText, sizeof(planeSumText), "● Multi-Plane Selection: %zu Planes (Hold Shift/Ctrl to add/remove)", m_selectedFaceIndices.size());
        ImVec2 sumSize = ImGui::CalcTextSize(planeSumText);
        float sumPad = 8.0f;
        ImVec2 sumP0(18.0f, 65.0f);
        ImVec2 sumP1(sumP0.x + sumSize.x + sumPad * 2.0f, sumP0.y + sumSize.y + sumPad * 2.0f);
        fg->AddRectFilled(sumP0, sumP1, IM_COL32(10, 24, 34, 230), 5.0f);
        fg->AddRect(sumP0, sumP1, IM_COL32(0, 200, 255, 220), 5.0f, 0, 1.5f);
        fg->AddText(ImVec2(sumP0.x + sumPad, sumP0.y + sumPad), IM_COL32(200, 245, 255, 255), planeSumText);
    }

    // -------------------------------------------------------------------------
    // 3D Viewport Edge Selection Highlight (Glowing Line + Endpins + Badge)
    // -------------------------------------------------------------------------
    if (m_selectedEdgeIndex >= 0 && selectedNode && !m_sketchModeActive) {
        const auto& verts = selectedNode->GetCachedVertices();
        const auto& edges = selectedNode->GetCachedEdges();
        Geom::Point3D p0 = m_selectedEdgeP0;
        Geom::Point3D p1 = m_selectedEdgeP1;

        size_t e = static_cast<size_t>(m_selectedEdgeIndex * 2);
        if (e + 1 < edges.size()) {
            uint32_t i0 = edges[e];
            uint32_t i1 = edges[e + 1];
            if (i0 < verts.size() && i1 < verts.size()) {
                p0 = Geom::Point3D(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
                p1 = Geom::Point3D(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
                m_selectedEdgeP0 = p0;
                m_selectedEdgeP1 = p1;
            }
        }

        float sx0 = 0.0f, sy0 = 0.0f;
        float sx1 = 0.0f, sy1 = 0.0f;
        float vpW = static_cast<float>(renderer.GetWidth());
        float vpH = static_cast<float>(renderer.GetHeight());
        if (vpW <= 0.0f || vpH <= 0.0f) {
            ImGuiIO& io = ImGui::GetIO();
            vpW = io.DisplaySize.x;
            vpH = io.DisplaySize.y;
        }

        bool vis0 = camera.WorldToScreenPoint(p0, vpW, vpH, sx0, sy0);
        bool vis1 = camera.WorldToScreenPoint(p1, vpW, vpH, sx1, sy1);

        if (vis0 && vis1) {
            ImDrawList* fg = ImGui::GetForegroundDrawList();
            ImVec2 pt0(sx0, sy0);
            ImVec2 pt1(sx1, sy1);

            // 1. Dark outer drop shadow line
            fg->AddLine(pt0, pt1, IM_COL32(0, 0, 0, 220), 6.0f);

            // 2. Vibrant electric cyan edge line
            fg->AddLine(pt0, pt1, IM_COL32(0, 230, 255, 255), 3.5f);

            // 3. Bright white core line
            fg->AddLine(pt0, pt1, IM_COL32(235, 250, 255, 255), 1.5f);

            // 4. Endpin rings at the edge endpoints
            fg->AddCircleFilled(pt0, 5.0f, IM_COL32(0, 230, 255, 255), 16);
            fg->AddCircleFilled(pt0, 2.5f, IM_COL32(255, 255, 255, 255), 16);
            fg->AddCircleFilled(pt1, 5.0f, IM_COL32(0, 230, 255, 255), 16);
            fg->AddCircleFilled(pt1, 2.5f, IM_COL32(255, 255, 255, 255), 16);

            // If a circular loop was detected, draw all other connected segments around the loop
            if (m_selectedCircle.isCircle) {
                for (int segIdx : m_selectedCircle.segments) {
                    if (segIdx == m_selectedEdgeIndex) continue;
                    size_t se = static_cast<size_t>(segIdx * 2);
                    if (se + 1 < edges.size()) {
                        uint32_t si0 = edges[se];
                        uint32_t si1 = edges[se + 1];
                        if (si0 < verts.size() && si1 < verts.size()) {
                            Geom::Point3D sp0(verts[si0].position[0], verts[si0].position[1], verts[si0].position[2]);
                            Geom::Point3D sp1(verts[si1].position[0], verts[si1].position[1], verts[si1].position[2]);
                            float ssx0, ssy0, ssx1, ssy1;
                            if (camera.WorldToScreenPoint(sp0, vpW, vpH, ssx0, ssy0) &&
                                camera.WorldToScreenPoint(sp1, vpW, vpH, ssx1, ssy1)) {
                                fg->AddLine(ImVec2(ssx0, ssy0), ImVec2(ssx1, ssy1), IM_COL32(0, 0, 0, 200), 5.0f);
                                fg->AddLine(ImVec2(ssx0, ssy0), ImVec2(ssx1, ssy1), IM_COL32(0, 230, 255, 255), 3.0f);
                            }
                        }
                    }
                }
            }

            // 5. Edge Callout Badge at midpoint
            ImVec2 midPt((pt0.x + pt1.x) * 0.5f, (pt0.y + pt1.y) * 0.5f);
            char edgeBadge[128];
            if (m_selectedCircle.isCircle) {
                std::snprintf(edgeBadge, sizeof(edgeBadge), "◯ Circle Loop (Ø %.2fm, R %.2fm | %zu segs)",
                              m_selectedCircle.diameter, m_selectedCircle.radius, m_selectedCircle.segments.size());
            } else {
                float length = static_cast<float>(p0.distance_to(p1));
                std::snprintf(edgeBadge, sizeof(edgeBadge), "Edge #%d  |  L = %.2f m", m_selectedEdgeIndex, length);
            }
            ImVec2 badgeSz = ImGui::CalcTextSize(edgeBadge);
            float padX = 7.0f;
            float padY = 3.0f;
            float bW = badgeSz.x + padX * 2.0f;
            float bH = badgeSz.y + padY * 2.0f;

            ImVec2 bP0(midPt.x - bW * 0.5f, midPt.y - bH - 12.0f);
            ImVec2 bP1(bP0.x + bW, bP0.y + bH);
            fg->AddLine(midPt, ImVec2(midPt.x, bP1.y), IM_COL32(0, 220, 255, 200), 1.5f);
            fg->AddRectFilled(bP0, bP1, IM_COL32(14, 18, 26, 230), 4.0f);
            fg->AddRect(bP0, bP1, IM_COL32(0, 220, 255, 220), 4.0f, 0, 1.5f);
            fg->AddText(ImVec2(bP0.x + padX, bP0.y + padY), IM_COL32(230, 245, 255, 255), edgeBadge);
        }
    }

    // -------------------------------------------------------------------------
    // 3D Viewport Datum Plane & Datum Axis Visual Overlays
    // -------------------------------------------------------------------------
    if (!m_sketchModeActive) {
        float vpW = static_cast<float>(renderer.GetWidth());
        float vpH = static_cast<float>(renderer.GetHeight());
        if (vpW <= 0.0f || vpH <= 0.0f) {
            ImGuiIO& io = ImGui::GetIO();
            vpW = io.DisplaySize.x;
            vpH = io.DisplaySize.y;
        }
        ImDrawList* bg = ImGui::GetBackgroundDrawList();

        for (const auto& node : nodes) {
            bool isSel = (node->GetId() == m_selectedNodeId);
            if (auto planeNode = std::dynamic_pointer_cast<DatumPlaneNode>(node)) {
                Geom::Plane pl = planeNode->GetPlane();
                Geom::Vector3D n = pl.normal.normalized();
                Geom::Vector3D ref = (std::abs(n.y) < 0.9) ? Geom::Vector3D(0.0, 1.0, 0.0) : Geom::Vector3D(1.0, 0.0, 0.0);
                Geom::Vector3D u = ref.cross(n).normalized();
                Geom::Vector3D v = n.cross(u).normalized();

                double halfExtent = 1.6;
                Geom::Point3D c0 = pl.origin + (-u - v) * halfExtent;
                Geom::Point3D c1 = pl.origin + ( u - v) * halfExtent;
                Geom::Point3D c2 = pl.origin + ( u + v) * halfExtent;
                Geom::Point3D c3 = pl.origin + (-u + v) * halfExtent;

                float sx[4], sy[4];
                if (camera.WorldToScreenPoint(c0, vpW, vpH, sx[0], sy[0]) &&
                    camera.WorldToScreenPoint(c1, vpW, vpH, sx[1], sy[1]) &&
                    camera.WorldToScreenPoint(c2, vpW, vpH, sx[2], sy[2]) &&
                    camera.WorldToScreenPoint(c3, vpW, vpH, sx[3], sy[3])) {
                    ImVec2 q0(sx[0], sy[0]), q1(sx[1], sy[1]), q2(sx[2], sy[2]), q3(sx[3], sy[3]);
                    ImU32 fillCol = isSel ? IM_COL32(0, 210, 255, 55) : IM_COL32(80, 175, 255, 28);
                    ImU32 borderCol = isSel ? IM_COL32(0, 235, 255, 235) : IM_COL32(90, 185, 255, 150);
                    float borderThick = isSel ? 2.2f : 1.3f;

                    bg->AddQuadFilled(q0, q1, q2, q3, fillCol);
                    bg->AddQuad(q0, q1, q2, q3, borderCol, borderThick);

                    float ox = 0.0f, oy = 0.0f, nx = 0.0f, ny = 0.0f;
                    if (camera.WorldToScreenPoint(pl.origin, vpW, vpH, ox, oy) &&
                        camera.WorldToScreenPoint(pl.origin + n * 0.8, vpW, vpH, nx, ny)) {
                        bg->AddLine(ImVec2(ox, oy), ImVec2(nx, ny), borderCol, borderThick);
                        bg->AddCircleFilled(ImVec2(ox, oy), isSel ? 4.0f : 3.0f, borderCol, 12);
                    }
                    if (isSel) {
                        char lbl[96];
                        std::snprintf(lbl, sizeof(lbl), "%s (Offset: %.2f m)", planeNode->GetName().c_str(), planeNode->GetOffset());
                        bg->AddText(ImVec2(q0.x + 6.0f, q0.y - 18.0f), IM_COL32(180, 240, 255, 240), lbl);
                    }
                }
            } else if (auto axisNode = std::dynamic_pointer_cast<DatumAxisNode>(node)) {
                Geom::Axis3D ax = axisNode->GetAxis();
                Geom::Vector3D dir = ax.direction.normalized();
                double halfLen = 2.4;
                Geom::Point3D p0 = ax.origin - dir * halfLen;
                Geom::Point3D p1 = ax.origin + dir * halfLen;

                float sx0 = 0.0f, sy0 = 0.0f, sx1 = 0.0f, sy1 = 0.0f, ox = 0.0f, oy = 0.0f;
                if (camera.WorldToScreenPoint(p0, vpW, vpH, sx0, sy0) &&
                    camera.WorldToScreenPoint(p1, vpW, vpH, sx1, sy1)) {
                    ImU32 lineCol = isSel ? IM_COL32(255, 200, 40, 245) : IM_COL32(235, 165, 45, 175);
                    float thick = isSel ? 2.6f : 1.6f;
                    bg->AddLine(ImVec2(sx0, sy0), ImVec2(sx1, sy1), lineCol, thick);
                    if (camera.WorldToScreenPoint(ax.origin, vpW, vpH, ox, oy)) {
                        bg->AddCircleFilled(ImVec2(ox, oy), isSel ? 4.5f : 3.2f, lineCol, 12);
                        if (isSel) {
                            char lbl[96];
                            std::snprintf(lbl, sizeof(lbl), "%s (Offset: %.2f m)", axisNode->GetName().c_str(), axisNode->GetOffset());
                            bg->AddText(ImVec2(ox + 8.0f, oy - 16.0f), IM_COL32(255, 230, 140, 245), lbl);
                        }
                    }
                }
            } else if (auto sketchNode = std::dynamic_pointer_cast<SketchNode>(node)) {
                if (isSel) {
                    // Render selected 2D sketch overlay in 3D viewport:
                    // 1. If sketch has geometry, draw its outline edges with crisp electric cyan line and vertex dots
                    const auto& verts = sketchNode->GetCachedVertices();
                    const auto& edges = sketchNode->GetCachedEdges();
                    if (!verts.empty() && !edges.empty()) {
                        for (size_t k = 0; k + 1 < edges.size(); k += 2) {
                            uint32_t i0 = edges[k];
                            uint32_t i1 = edges[k + 1];
                            if (i0 < verts.size() && i1 < verts.size()) {
                                Geom::Point3D p0(verts[i0].position[0], verts[i0].position[1], verts[i0].position[2]);
                                Geom::Point3D p1(verts[i1].position[0], verts[i1].position[1], verts[i1].position[2]);
                                float sx0 = 0.0f, sy0 = 0.0f, sx1 = 0.0f, sy1 = 0.0f;
                                if (camera.WorldToScreenPoint(p0, vpW, vpH, sx0, sy0) &&
                                    camera.WorldToScreenPoint(p1, vpW, vpH, sx1, sy1)) {
                                    bg->AddLine(ImVec2(sx0, sy0), ImVec2(sx1, sy1), IM_COL32(0, 230, 255, 255), 2.8f);
                                }
                            }
                        }
                        for (const auto& v : verts) {
                            Geom::Point3D p(v.position[0], v.position[1], v.position[2]);
                            float px = 0.0f, py = 0.0f;
                            if (camera.WorldToScreenPoint(p, vpW, vpH, px, py)) {
                                bg->AddCircleFilled(ImVec2(px, py), 4.5f, IM_COL32(0, 240, 255, 255), 12);
                                bg->AddCircle(ImVec2(px, py), 5.5f, IM_COL32(20, 30, 45, 220), 12, 1.2f);
                            }
                        }
                    } else {
                        // Empty sketch: draw reference boundary quad on its sketch plane
                        Geom::Plane pl = sketchNode->HasCustomPlane() ? sketchNode->GetCustomPlane() : Geom::Plane::XZ();
                        if (!sketchNode->HasCustomPlane()) {
                            Geom::Point3D o(sketchNode->GetPosition()[0], sketchNode->GetPosition()[1], sketchNode->GetPosition()[2]);
                            switch (sketchNode->GetPlane()) {
                                case SketchPlane::XZ_Ground: pl = Geom::Plane::XZ(o); break;
                                case SketchPlane::XY_Front:  pl = Geom::Plane::XY(o); break;
                                case SketchPlane::YZ_Right:  pl = Geom::Plane::YZ(o); break;
                            }
                        }
                        Geom::Vector3D n = pl.normal.normalized();
                        Geom::Vector3D ref = (std::abs(n.y) < 0.9) ? Geom::Vector3D(0.0, 1.0, 0.0) : Geom::Vector3D(1.0, 0.0, 0.0);
                        Geom::Vector3D u = ref.cross(n).normalized();
                        Geom::Vector3D v = n.cross(u).normalized();
                        double halfExtent = 1.5;
                        Geom::Point3D c0 = pl.origin + (-u - v) * halfExtent;
                        Geom::Point3D c1 = pl.origin + ( u - v) * halfExtent;
                        Geom::Point3D c2 = pl.origin + ( u + v) * halfExtent;
                        Geom::Point3D c3 = pl.origin + (-u + v) * halfExtent;
                        float sx[4], sy[4];
                        if (camera.WorldToScreenPoint(c0, vpW, vpH, sx[0], sy[0]) &&
                            camera.WorldToScreenPoint(c1, vpW, vpH, sx[1], sy[1]) &&
                            camera.WorldToScreenPoint(c2, vpW, vpH, sx[2], sy[2]) &&
                            camera.WorldToScreenPoint(c3, vpW, vpH, sx[3], sy[3])) {
                            ImVec2 q0(sx[0], sy[0]), q1(sx[1], sy[1]), q2(sx[2], sy[2]), q3(sx[3], sy[3]);
                            bg->AddQuadFilled(q0, q1, q2, q3, IM_COL32(0, 200, 255, 30));
                            bg->AddQuad(q0, q1, q2, q3, IM_COL32(0, 220, 255, 200), 1.8f);
                            char lbl[96];
                            std::snprintf(lbl, sizeof(lbl), "✎ Empty Sketch: %s (Click Edit to draw)", sketchNode->GetName().c_str());
                            bg->AddText(ImVec2(q0.x + 6.0f, q0.y - 18.0f), IM_COL32(160, 235, 255, 240), lbl);
                        }
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // 3D Viewport Subtle X/Y/Z Axes Reference & View Orientation Controls
    // -------------------------------------------------------------------------
    if (m_showAxesReference && !m_sketchModeActive) {
        ImDrawList* bg = ImGui::GetBackgroundDrawList();
        float vpW = static_cast<float>(renderer.GetWidth());
        float vpH = static_cast<float>(renderer.GetHeight());
        if (vpW <= 0.0f || vpH <= 0.0f) {
            ImGuiIO& io = ImGui::GetIO();
            vpW = io.DisplaySize.x;
            vpH = io.DisplaySize.y;
        }

        // 1. Subtle 3D World Axes Cross at World Origin (0, 0, 0)
        float ox = 0.0f, oy = 0.0f;
        if (camera.WorldToScreenPoint(Geom::Point3D(0.0, 0.0, 0.0), vpW, vpH, ox, oy)) {
            float axisLength = 2.0f; // 2 meters in world space
            float px = 0.0f, py = 0.0f;

            // Subtle center origin point
            bg->AddCircleFilled(ImVec2(ox, oy), 3.5f, IM_COL32(230, 235, 245, 210), 12);
            bg->AddCircle(ImVec2(ox, oy), 4.5f, IM_COL32(20, 25, 35, 180), 12, 1.2f);

            // +X Axis (Red)
            if (camera.WorldToScreenPoint(Geom::Point3D(axisLength, 0.0, 0.0), vpW, vpH, px, py)) {
                bg->AddLine(ImVec2(ox, oy), ImVec2(px, py), IM_COL32(235, 65, 65, 190), 2.0f);
                bg->AddText(ImVec2(px + 4.0f, py - 8.0f), IM_COL32(255, 110, 110, 240), "+X");
            }
            // +Y Axis (Green)
            if (camera.WorldToScreenPoint(Geom::Point3D(0.0, axisLength, 0.0), vpW, vpH, px, py)) {
                bg->AddLine(ImVec2(ox, oy), ImVec2(px, py), IM_COL32(65, 220, 85, 190), 2.0f);
                bg->AddText(ImVec2(px + 4.0f, py - 8.0f), IM_COL32(110, 255, 130, 240), "+Y");
            }
            // +Z Axis (Blue)
            if (camera.WorldToScreenPoint(Geom::Point3D(0.0, 0.0, axisLength), vpW, vpH, px, py)) {
                bg->AddLine(ImVec2(ox, oy), ImVec2(px, py), IM_COL32(65, 140, 255, 190), 2.0f);
                bg->AddText(ImVec2(px + 4.0f, py - 8.0f), IM_COL32(120, 180, 255, 240), "+Z");
            }
        }

        // 2. Subtle Viewport Coordinate Triad (Orientation Triad in corner)
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        ImGuiIO& io = ImGui::GetIO();

        float triadCenterX = 60.0f;
        float triadCenterY = vpH - 86.0f;
        float triadRadius = 32.0f;

        // Subtle dark background circle
        fg->AddCircleFilled(ImVec2(triadCenterX, triadCenterY), 38.0f, IM_COL32(15, 18, 24, 180), 32);
        fg->AddCircle(ImVec2(triadCenterX, triadCenterY), 38.0f, IM_COL32(60, 75, 95, 140), 32, 1.0f);

        float fwd[3], rgt[3], up[3];
        camera.GetBasisVectors(fwd, rgt, up);

        struct AxisProj {
            const char* name;
            float scrX, scrY, depth;
            ImU32 color;
            void (*onClick)(Camera&);
        };

        std::vector<AxisProj> axes = {
            { "X",  rgt[0], -up[0], fwd[0], IM_COL32(235, 65, 65, 240), [](Camera& c) { c.SetSideView(true); } },
            { "Y",  rgt[1], -up[1], fwd[1], IM_COL32(65, 220, 85, 240), [](Camera& c) { c.SetTopView(true); } },
            { "Z",  rgt[2], -up[2], fwd[2], IM_COL32(65, 140, 255, 240), [](Camera& c) { c.SetFrontView(true); } }
        };

        std::sort(axes.begin(), axes.end(), [](const AxisProj& a, const AxisProj& b) {
            return a.depth < b.depth;
        });

        for (const auto& ax : axes) {
            ImVec2 tip(triadCenterX + ax.scrX * triadRadius, triadCenterY + ax.scrY * triadRadius);
            fg->AddLine(ImVec2(triadCenterX, triadCenterY), tip, ax.color, 2.2f);

            float dMouse = std::hypot(io.MousePos.x - tip.x, io.MousePos.y - tip.y);
            bool hovered = (dMouse < 9.0f);
            if (hovered) {
                fg->AddCircleFilled(tip, 8.5f, IM_COL32(255, 255, 255, 240), 16);
                fg->AddCircleFilled(tip, 6.5f, ax.color, 16);
                char tipBuf[64];
                std::snprintf(tipBuf, sizeof(tipBuf), "%s-Axis View [Click to Recenter on Origin]", ax.name);
                ImGui::SetTooltip("%s", tipBuf);
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    ax.onClick(camera);
                }
            } else {
                fg->AddCircleFilled(tip, 6.0f, ax.color, 16);
            }
            fg->AddText(ImVec2(tip.x - 3.5f, tip.y - 6.0f), IM_COL32(255, 255, 255, 255), ax.name);
        }

        // Center origin pivot
        float dCenter = std::hypot(io.MousePos.x - triadCenterX, io.MousePos.y - triadCenterY);
        bool centerHovered = (dCenter < 7.0f);
        fg->AddCircleFilled(ImVec2(triadCenterX, triadCenterY), centerHovered ? 6.0f : 4.0f, centerHovered ? IM_COL32(255, 230, 80, 255) : IM_COL32(180, 195, 215, 220), 16);
        if (centerHovered) {
            ImGui::SetTooltip("Isometric View [Click to Recenter on Origin]");
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                camera.SetIsometricView(true);
            }
        }
    }

    // 3. Floating Viewport Recenter & Orientation Control Bar
    if (!m_sketchModeActive) {
        float vpH = static_cast<float>(renderer.GetHeight());
        if (vpH <= 0.0f) vpH = ImGui::GetIO().DisplaySize.y;

        ImGui::SetNextWindowPos(ImVec2(18.0f, vpH - 38.0f), ImGuiCond_Always);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.12f, 0.15f, 0.20f, 0.88f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.28f, 0.38f, 0.52f, 0.70f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 5.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.0f, 4.0f));
        if (ImGui::Begin("##ViewportViewControlsHUD", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav)) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(ImVec4(0.35f, 0.8f, 1.0f, 1.0f), "View:");
            ImGui::SameLine();
            if (ImGui::SmallButton("⌂ Origin")) {
                camera.RecenterOrigin();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter camera target on (0, 0, 0) origin");
            ImGui::SameLine();
            if (ImGui::SmallButton("Top")) {
                camera.SetTopView(true);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter view on Top origin (+Y looking down on XZ)");
            ImGui::SameLine();
            if (ImGui::SmallButton("Side")) {
                camera.SetSideView(true);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter view on Side origin (+X looking on YZ)");
            ImGui::SameLine();
            if (ImGui::SmallButton("Iso")) {
                camera.SetIsometricView(true);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter view on Isometric 3D origin");
            ImGui::SameLine();
            if (ImGui::SmallButton("Front")) {
                camera.SetFrontView(true);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Recenter view on Front origin (+Z looking on XY)");

            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            const char* presetLabel = "Medium (0.25m)";
            if (m_selectionResPreset == SelectionResolutionPreset::UltraFine) presetLabel = "UltraFine (0.05m)";
            else if (m_selectionResPreset == SelectionResolutionPreset::Fine) presetLabel = "Fine (0.12m)";
            else if (m_selectionResPreset == SelectionResolutionPreset::Medium) presetLabel = "Medium (0.25m)";
            else if (m_selectionResPreset == SelectionResolutionPreset::Coarse) presetLabel = "Coarse (0.45m)";
            else if (m_selectionResPreset == SelectionResolutionPreset::Custom) presetLabel = "Custom";

            char pickResBtn[64];
            std::snprintf(pickResBtn, sizeof(pickResBtn), "⚙ Pick Res: %s ▾", presetLabel);
            if (ImGui::SmallButton(pickResBtn)) {
                ImGui::OpenPopup("SelectionResolutionPopup");
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Selection resolution & filter for points, planes, and edges.\nHold Shift/Ctrl to multi-select points or planes.");
            }

            if (ImGui::BeginPopup("SelectionResolutionPopup")) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Selection Resolution (Hit Tolerance)");
                ImGui::Separator();
                if (ImGui::RadioButton("Ultra-Fine (0.05m - dense vertices/planes)", m_selectionResPreset == SelectionResolutionPreset::UltraFine)) {
                    SetSelectionResolutionPreset(SelectionResolutionPreset::UltraFine);
                }
                if (ImGui::RadioButton("Fine (0.12m)", m_selectionResPreset == SelectionResolutionPreset::Fine)) {
                    SetSelectionResolutionPreset(SelectionResolutionPreset::Fine);
                }
                if (ImGui::RadioButton("Medium (0.25m - standard CAD default)", m_selectionResPreset == SelectionResolutionPreset::Medium)) {
                    SetSelectionResolutionPreset(SelectionResolutionPreset::Medium);
                }
                if (ImGui::RadioButton("Coarse (0.45m - wide hit area)", m_selectionResPreset == SelectionResolutionPreset::Coarse)) {
                    SetSelectionResolutionPreset(SelectionResolutionPreset::Coarse);
                }
                if (ImGui::RadioButton("Custom Slider", m_selectionResPreset == SelectionResolutionPreset::Custom)) {
                    m_selectionResPreset = SelectionResolutionPreset::Custom;
                }

                if (m_selectionResPreset == SelectionResolutionPreset::Custom) {
                    ImGui::Indent();
                    float ptTol = static_cast<float>(m_pointPickTolerance);
                    if (ImGui::SliderFloat("Point Tol", &ptTol, 0.01f, 1.0f, "%.2f m")) {
                        m_pointPickTolerance = ptTol;
                    }
                    float edTol = static_cast<float>(m_edgePickTolerance);
                    if (ImGui::SliderFloat("Edge Tol", &edTol, 0.01f, 1.0f, "%.2f m")) {
                        m_edgePickTolerance = edTol;
                    }
                    ImGui::Unindent();
                }

                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Sub-Element Filter");
                ImGui::Separator();
                int curFilter = static_cast<int>(m_selectionFilter);
                if (ImGui::RadioButton("All Elements (Points, Edges, Planes)", curFilter == 0)) {
                    m_selectionFilter = SelectionFilter::All;
                }
                if (ImGui::RadioButton("Points Only", curFilter == 1)) {
                    m_selectionFilter = SelectionFilter::PointsOnly;
                }
                if (ImGui::RadioButton("Planes Only", curFilter == 2)) {
                    m_selectionFilter = SelectionFilter::PlanesOnly;
                }
                if (ImGui::RadioButton("Edges Only", curFilter == 3)) {
                    m_selectionFilter = SelectionFilter::EdgesOnly;
                }

                ImGui::Spacing();
                ImGui::TextDisabled("Multi-selection: Hold Shift or Ctrl while clicking\nto select multiple points or planes.");
                ImGui::EndPopup();
            }

            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            if (m_measureToolActive) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.55f, 0.75f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.26f, 0.34f, 1.0f));
            }
            if (ImGui::SmallButton(m_measureToolActive ? "📏 Measuring [ON]" : "📏 Measure")) {
                ToggleMeasureTool();
            }
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Toggle 3D Spatial Measure Tool (Shortcut: M)");
            }
        }
        ImGui::End();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(2);
    }
}

// -----------------------------------------------------------------------------
// 3D Spatial Measure Tool Dialog & Overlay Implementation
// -----------------------------------------------------------------------------
void CADGui::RenderMeasureToolDialog() {
    if (!m_measureToolActive && !m_showMeasureDialog) return;

    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        CloseMeasurement();
        return;
    }

    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_Appearing);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f, 0.13f, 0.18f, 0.94f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.20f, 0.65f, 0.90f, 0.85f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));

    bool open = true;
    if (ImGui::Begin("📏 3D Spatial Measure Tool###MeasureToolDialog", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "CAD 3D Spatial Measurement");
        ImGui::Separator();

        if (m_measureHasCircle && m_measuredCircle.isCircle) {
            // Circular Loop Geometry Display
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.06f, 0.20f, 0.26f, 0.92f));
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.15f, 0.85f, 0.95f, 1.0f));
            ImGui::BeginChild("CircleMeasurementBanner", ImVec2(0, 56), true);
            ImGui::TextColored(ImVec4(0.35f, 0.95f, 1.0f, 1.0f), "● Circular Loop Geometry Detected");
            if (m_measureDisplayRadius) {
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.25f, 1.0f), "Radius (R): %.4f m", m_measuredCircle.radius);
                ImGui::SameLine();
                ImGui::TextDisabled("(Diameter Ø: %.4f m)", m_measuredCircle.diameter);
            } else {
                ImGui::TextColored(ImVec4(0.35f, 0.95f, 1.0f, 1.0f), "Diameter (Ø): %.4f m", m_measuredCircle.diameter);
                ImGui::SameLine();
                ImGui::TextDisabled("(Radius R: %.4f m)", m_measuredCircle.radius);
            }
            ImGui::EndChild();
            ImGui::PopStyleColor(2);
            ImGui::Spacing();

            // Toggle Button between Diameter and Radius
            if (ImGui::Button(m_measureDisplayRadius ? "⇄ Switch to Diameter (Ø)" : "⇄ Switch to Radius (R)", ImVec2(-1, 26))) {
                ToggleMeasureCircleRadiusMode();
            }
            ImGui::Spacing();

            // Additional circular dimensions
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Circular Metrics:");
            ImGui::TextColored(ImVec4(0.40f, 0.95f, 0.65f, 1.0f), "  Circumference (2πR): %.4f m", m_measuredCircle.circumference);
            ImGui::TextColored(ImVec4(1.0f, 0.70f, 0.35f, 1.0f), "  Enclosed Area (πR²):  %.4f m²", m_measuredCircle.area);
            ImGui::Spacing();

            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Spatial Orientation:");
            ImGui::Text("  Center: (%.3f, %.3f, %.3f)", m_measuredCircle.center.x, m_measuredCircle.center.y, m_measuredCircle.center.z);
            ImGui::Text("  Normal: (%.3f, %.3f, %.3f)", m_measuredCircle.normal.x, m_measuredCircle.normal.y, m_measuredCircle.normal.z);
            ImGui::Text("  Fitted Segments: %zu edges (%zu loop points)", m_measuredCircle.segments.size(), m_measuredCircle.points.size());
            ImGui::Spacing();
        } else if (!m_measureHasP1 && !m_measureHasP2) {
            ImGui::TextDisabled("Status: Waiting for points...");
            ImGui::Spacing();
            ImGui::TextWrapped("Click any two points or vertices on 3D models in the viewport to measure distances and spatial angles.");
            ImGui::Spacing();
            ImGui::TextDisabled("(Tip: You can also Shift-click two vertices on any model)");
        } else if (m_measureHasP1 && !m_measureHasP2) {
            ImGui::TextColored(ImVec4(1.0f, 0.80f, 0.20f, 1.0f), "● Point 1 (P1): (%.3f, %.3f, %.3f)", m_measureP1.x, m_measureP1.y, m_measureP1.z);
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "Status: Click second point on 3D model...");
            ImGui::TextDisabled("(P2 will complete the measurement)");
        } else {
            auto m = Geom::ComputeMeasurement(m_measureP1, m_measureP2);

            ImGui::TextColored(ImVec4(1.0f, 0.80f, 0.20f, 1.0f), "● Point 1 (P1): (%.3f, %.3f, %.3f)", m_measureP1.x, m_measureP1.y, m_measureP1.z);
            ImGui::TextColored(ImVec4(0.20f, 0.85f, 1.0f, 1.0f), "● Point 2 (P2): (%.3f, %.3f, %.3f)", m_measureP2.x, m_measureP2.y, m_measureP2.z);
            ImGui::Spacing();

            // Direct Euclidean Distance in prominent banner
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.18f, 0.25f, 0.90f));
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.20f, 0.75f, 0.95f, 1.0f));
            ImGui::BeginChild("DirectDistanceBanner", ImVec2(0, 52), true);
            ImGui::TextColored(ImVec4(0.35f, 0.90f, 1.0f, 1.0f), "Direct 3D Distance:");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "%.4f m", m.directDistance);
            ImGui::TextDisabled("(Euclidean straight-line span between P1 and P2)");
            ImGui::EndChild();
            ImGui::PopStyleColor(2);
            ImGui::Spacing();

            // Orthogonal Axis Deltas (X, Y, Z)
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Orthogonal Axis Deltas:");
            ImGui::TextColored(ImVec4(1.0f, 0.40f, 0.40f, 1.0f), "  ΔX: %.4f m  (Signed: %+.4f m)", m.absDeltaX, m.deltaX);
            ImGui::TextColored(ImVec4(0.40f, 1.0f, 0.40f, 1.0f), "  ΔY: %.4f m  (Signed: %+.4f m)", m.absDeltaY, m.deltaY);
            ImGui::TextColored(ImVec4(0.40f, 0.65f, 1.0f, 1.0f), "  ΔZ: %.4f m  (Signed: %+.4f m)", m.absDeltaZ, m.deltaZ);
            ImGui::Spacing();

            // Planar Projected Distances
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Planar 2D Projections:");
            ImGui::Text("  Horizontal Ground (XZ): %.4f m", m.planarDistXZ);
            ImGui::Text("  Front Plane (XY):       %.4f m", m.planarDistXY);
            ImGui::Text("  Side Plane (YZ):        %.4f m", m.planarDistYZ);
            ImGui::Spacing();

            // Angles
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Spatial Angles:");
            ImGui::Text("  Incline from Horizontal:  %+.2f°", m.elevationDeg);
            ImGui::Text("  Ground Heading (XZ):      %+.2f°", m.azimuthDeg);
            ImGui::Text("  Angle to X-Axis:           %.2f°", m.angleDegX);
            ImGui::Text("  Angle to Y-Axis (Vertical):%.2f°", m.angleDegY);
            ImGui::Text("  Angle to Z-Axis:           %.2f°", m.angleDegZ);
            ImGui::Spacing();
        }

        ImGui::Separator();
        if (ImGui::Button("⇄ Swap P1/P2", ImVec2(100, 24))) {
            SwapMeasurePoints();
        }
        ImGui::SameLine();
        if (ImGui::Button("✕ Clear", ImVec2(70, 24))) {
            ClearMeasurePoints();
        }
        ImGui::SameLine();
        if (ImGui::Button("Done (Esc)", ImVec2(80, 24))) {
            CloseMeasurement();
        }
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);

    if (!open) {
        CloseMeasurement();
    }
}

void CADGui::RenderMeasureVisualOverlay(Camera& camera, ViewportRenderer& renderer) {
    if (m_sketchModeActive) return;
    if (!m_measureToolActive && !m_showMeasureDialog) return;
    if (!m_measureHasP1 && !m_measureHasCircle) return;

    float vpW = static_cast<float>(renderer.GetWidth());
    float vpH = static_cast<float>(renderer.GetHeight());
    if (vpW <= 0.0f || vpH <= 0.0f) {
        ImGuiIO& io = ImGui::GetIO();
        vpW = io.DisplaySize.x;
        vpH = io.DisplaySize.y;
    }

    ImDrawList* fg = ImGui::GetForegroundDrawList();

    // 0. Circular Edge Overlay
    if (m_measureHasCircle && m_measuredCircle.isCircle) {
        const auto& pts = m_measuredCircle.points;
        for (size_t i = 0; i < pts.size(); ++i) {
            size_t j = (i + 1) % pts.size();
            float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
            if (camera.WorldToScreenPoint(pts[i], vpW, vpH, x0, y0) &&
                camera.WorldToScreenPoint(pts[j], vpW, vpH, x1, y1)) {
                fg->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(0, 240, 255, 255), 3.0f);
            }
        }

        float cx = 0.0f, cy = 0.0f;
        if (camera.WorldToScreenPoint(m_measuredCircle.center, vpW, vpH, cx, cy)) {
            fg->AddCircleFilled(ImVec2(cx, cy), 5.0f, IM_COL32(255, 200, 50, 255), 16);
            fg->AddLine(ImVec2(cx - 10.0f, cy), ImVec2(cx + 10.0f, cy), IM_COL32(255, 200, 50, 240), 1.5f);
            fg->AddLine(ImVec2(cx, cy - 10.0f), ImVec2(cx, cy + 10.0f), IM_COL32(255, 200, 50, 240), 1.5f);

            char badge[128];
            if (m_measureDisplayRadius) {
                std::snprintf(badge, sizeof(badge), "R: %.3f m  |  Circ: %.3f m  |  Area: %.3f m²",
                    m_measuredCircle.radius, m_measuredCircle.circumference, m_measuredCircle.area);
            } else {
                std::snprintf(badge, sizeof(badge), "Ø: %.3f m  |  Circ: %.3f m  |  Area: %.3f m²",
                    m_measuredCircle.diameter, m_measuredCircle.circumference, m_measuredCircle.area);
            }
            ImVec2 bSz = ImGui::CalcTextSize(badge);
            ImVec2 b0(cx + 14.0f, cy - 12.0f);
            ImVec2 b1(b0.x + bSz.x + 16.0f, b0.y + bSz.y + 8.0f);
            fg->AddRectFilled(b0, b1, IM_COL32(10, 20, 28, 235), 4.0f);
            fg->AddRect(b0, b1, IM_COL32(0, 230, 255, 230), 4.0f, 0, 1.5f);
            fg->AddText(ImVec2(b0.x + 8.0f, b0.y + 4.0f), IM_COL32(180, 245, 255, 255), badge);
        }
    }

    if (!m_measureHasP1) return;

    // 1. Point 1 (P1)
    float sx1 = 0.0f, sy1 = 0.0f;
    bool vis1 = camera.WorldToScreenPoint(m_measureP1, vpW, vpH, sx1, sy1);

    if (vis1) {
        ImVec2 pt1(sx1, sy1);
        fg->AddCircleFilled(pt1, 7.0f, IM_COL32(255, 180, 0, 255), 24); // Amber
        fg->AddCircleFilled(pt1, 3.5f, IM_COL32(255, 255, 255, 255), 16);
        fg->AddCircle(pt1, 9.0f, IM_COL32(0, 0, 0, 200), 24, 2.0f);

        // Label P1
        fg->AddRectFilled(ImVec2(pt1.x - 14.0f, pt1.y - 24.0f), ImVec2(pt1.x + 14.0f, pt1.y - 8.0f), IM_COL32(20, 24, 32, 220), 4.0f);
        fg->AddRect(ImVec2(pt1.x - 14.0f, pt1.y - 24.0f), ImVec2(pt1.x + 14.0f, pt1.y - 8.0f), IM_COL32(255, 180, 0, 240), 4.0f, 0, 1.2f);
        fg->AddText(ImVec2(pt1.x - 8.0f, pt1.y - 23.0f), IM_COL32(255, 220, 100, 255), "P1");
    }

    // If only P1 is defined, display status indicator near cursor or P1
    if (!m_measureHasP2) {
        if (vis1) {
            char p1Buf[128];
            std::snprintf(p1Buf, sizeof(p1Buf), "P1: (%.2f, %.2f, %.2f)  |  Click 2nd point to measure", m_measureP1.x, m_measureP1.y, m_measureP1.z);
            ImVec2 sz = ImGui::CalcTextSize(p1Buf);
            ImVec2 b0(sx1 + 20.0f, sy1 - 12.0f);
            ImVec2 b1(b0.x + sz.x + 16.0f, b0.y + sz.y + 8.0f);
            fg->AddRectFilled(b0, b1, IM_COL32(14, 18, 26, 230), 4.0f);
            fg->AddRect(b0, b1, IM_COL32(255, 180, 0, 220), 4.0f, 0, 1.5f);
            fg->AddText(ImVec2(b0.x + 8.0f, b0.y + 4.0f), IM_COL32(255, 240, 180, 255), p1Buf);
        }
        return;
    }

    // 2. Point 2 (P2)
    float sx2 = 0.0f, sy2 = 0.0f;
    bool vis2 = camera.WorldToScreenPoint(m_measureP2, vpW, vpH, sx2, sy2);

    if (vis2) {
        ImVec2 pt2(sx2, sy2);
        fg->AddCircleFilled(pt2, 7.0f, IM_COL32(0, 210, 255, 255), 24); // Cyan
        fg->AddCircleFilled(pt2, 3.5f, IM_COL32(255, 255, 255, 255), 16);
        fg->AddCircle(pt2, 9.0f, IM_COL32(0, 0, 0, 200), 24, 2.0f);

        // Label P2
        fg->AddRectFilled(ImVec2(pt2.x - 14.0f, pt2.y - 24.0f), ImVec2(pt2.x + 14.0f, pt2.y - 8.0f), IM_COL32(20, 24, 32, 220), 4.0f);
        fg->AddRect(ImVec2(pt2.x - 14.0f, pt2.y - 24.0f), ImVec2(pt2.x + 14.0f, pt2.y - 8.0f), IM_COL32(0, 210, 255, 240), 4.0f, 0, 1.2f);
        fg->AddText(ImVec2(pt2.x - 8.0f, pt2.y - 23.0f), IM_COL32(180, 245, 255, 255), "P2");
    }

    auto m = Geom::ComputeMeasurement(m_measureP1, m_measureP2);

    // 3. Orthogonal component step lines (ΔX along Red, ΔY along Green, ΔZ along Blue)
    // Corner A: (P2.x, P1.y, P1.z)
    // Corner B: (P2.x, P2.y, P1.z)
    Geom::Point3D cornerA(m_measureP2.x, m_measureP1.y, m_measureP1.z);
    Geom::Point3D cornerB(m_measureP2.x, m_measureP2.y, m_measureP1.z);

    float sxA = 0.0f, syA = 0.0f;
    float sxB = 0.0f, syB = 0.0f;
    bool visA = camera.WorldToScreenPoint(cornerA, vpW, vpH, sxA, syA);
    bool visB = camera.WorldToScreenPoint(cornerB, vpW, vpH, sxB, syB);

    if (vis1 && visA && m.absDeltaX > 1e-4) {
        // ΔX component (Red line)
        fg->AddLine(ImVec2(sx1, sy1), ImVec2(sxA, syA), IM_COL32(255, 80, 80, 220), 2.0f);
        char dxText[32];
        std::snprintf(dxText, sizeof(dxText), "ΔX: %.2f", m.absDeltaX);
        ImVec2 midA((sx1 + sxA) * 0.5f, (sy1 + syA) * 0.5f);
        fg->AddText(ImVec2(midA.x + 4.0f, midA.y - 12.0f), IM_COL32(255, 130, 130, 240), dxText);
    }

    if (visA && visB && m.absDeltaY > 1e-4) {
        // ΔY component (Green line)
        fg->AddLine(ImVec2(sxA, syA), ImVec2(sxB, syB), IM_COL32(80, 240, 110, 220), 2.0f);
        char dyText[32];
        std::snprintf(dyText, sizeof(dyText), "ΔY: %.2f", m.absDeltaY);
        ImVec2 midB((sxA + sxB) * 0.5f, (syA + syB) * 0.5f);
        fg->AddText(ImVec2(midB.x + 4.0f, midB.y - 12.0f), IM_COL32(130, 255, 150, 240), dyText);
    }

    if (visB && vis2 && m.absDeltaZ > 1e-4) {
        // ΔZ component (Blue line)
        fg->AddLine(ImVec2(sxB, syB), ImVec2(sx2, sy2), IM_COL32(90, 160, 255, 220), 2.0f);
        char dzText[32];
        std::snprintf(dzText, sizeof(dzText), "ΔZ: %.2f", m.absDeltaZ);
        ImVec2 midC((sxB + sx2) * 0.5f, (syB + sy2) * 0.5f);
        fg->AddText(ImVec2(midC.x + 4.0f, midC.y - 12.0f), IM_COL32(150, 200, 255, 240), dzText);
    }

    // 4. Direct 3D Dimension Line between P1 and P2
    if (vis1 && vis2) {
        ImVec2 pt1(sx1, sy1);
        ImVec2 pt2(sx2, sy2);

        // Dark outer drop shadow line
        fg->AddLine(pt1, pt2, IM_COL32(0, 0, 0, 200), 5.0f);
        // Vibrant gold/amber line
        fg->AddLine(pt1, pt2, IM_COL32(255, 200, 30, 255), 3.0f);
        // Bright white/gold core
        fg->AddLine(pt1, pt2, IM_COL32(255, 248, 190, 255), 1.5f);

        // 5. Floating Dimension Callout Badge at screen midpoint
        ImVec2 midPt((pt1.x + pt2.x) * 0.5f, (pt1.y + pt2.y) * 0.5f);

        char l1[64], l2[64], l3[64];
        std::snprintf(l1, sizeof(l1), "📏 D = %.4f m", m.directDistance);
        std::snprintf(l2, sizeof(l2), "ΔX: %.3f  ΔY: %.3f  ΔZ: %.3f", m.absDeltaX, m.absDeltaY, m.absDeltaZ);
        std::snprintf(l3, sizeof(l3), "Incline: %+.1f°  Ground: %+.1f°  (∠X: %.0f°, ∠Y: %.0f°, ∠Z: %.0f°)", 
                      m.elevationDeg, m.azimuthDeg, m.angleDegX, m.angleDegY, m.angleDegZ);

        ImVec2 sz1 = ImGui::CalcTextSize(l1);
        ImVec2 sz2 = ImGui::CalcTextSize(l2);
        ImVec2 sz3 = ImGui::CalcTextSize(l3);
        float boxW = std::max({sz1.x, sz2.x, sz3.x}) + 16.0f;
        float boxH = sz1.y + sz2.y + sz3.y + 16.0f;

        float boxX = midPt.x - boxW * 0.5f;
        float boxY = midPt.y - boxH - 16.0f;
        if (boxX < 10.0f) boxX = 10.0f;
        if (boxX + boxW > vpW - 10.0f) boxX = vpW - boxW - 10.0f;
        if (boxY < 65.0f) boxY = midPt.y + 20.0f;

        ImVec2 bP0(boxX, boxY);
        ImVec2 bP1(boxX + boxW, boxY + boxH);

        // Leader line from midpoint to callout box
        fg->AddLine(midPt, ImVec2(midPt.x, (boxY < midPt.y) ? bP1.y : bP0.y), IM_COL32(255, 200, 30, 200), 1.5f);

        // Badge background & border
        fg->AddRectFilled(bP0, bP1, IM_COL32(12, 16, 24, 235), 6.0f);
        fg->AddRect(bP0, bP1, IM_COL32(255, 195, 45, 220), 6.0f, 0, 1.5f);

        // Text lines
        float curY = boxY + 4.0f;
        fg->AddText(ImVec2(boxX + 8.0f, curY), IM_COL32(255, 225, 90, 255), l1);
        curY += sz1.y + 2.0f;
        fg->AddText(ImVec2(boxX + 8.0f, curY), IM_COL32(200, 230, 255, 255), l2);
        curY += sz2.y + 2.0f;
        fg->AddText(ImVec2(boxX + 8.0f, curY), IM_COL32(160, 245, 180, 255), l3);
    }
}

void CADGui::RenderCutPreviewVisualOverlay(Camera& camera, ViewportRenderer& renderer) {
    if (m_sketchModeActive) return;
    if (!m_featurePreviewActive || !m_previewNode) return;

    bool isExtrudeCut = (m_previewFeatureType == PreviewFeatureType::CutExtrude);
    bool isRevolveCut = (m_previewFeatureType == PreviewFeatureType::CutRevolve);

    auto extrude = std::dynamic_pointer_cast<ExtrudeNode>(m_previewNode);
    if (extrude && extrude->IsCut()) isExtrudeCut = true;

    auto revolve = std::dynamic_pointer_cast<RevolveNode>(m_previewNode);
    if (revolve && revolve->IsCut()) isRevolveCut = true;

    if (!isExtrudeCut && !isRevolveCut) return;

    float vpW = static_cast<float>(renderer.GetWidth());
    float vpH = static_cast<float>(renderer.GetHeight());
    if (vpW <= 0.0f || vpH <= 0.0f) {
        ImGuiIO& io = ImGui::GetIO();
        vpW = io.DisplaySize.x;
        vpH = io.DisplaySize.y;
    }

    ImDrawList* fg = ImGui::GetForegroundDrawList();

    if (isExtrudeCut && extrude) {
        auto sketch = extrude->GetSketchReference();
        std::vector<std::vector<std::array<float, 2>>> loops;
        if (sketch && !sketch->GetShapes().empty()) {
            loops = sketch->GetShapes();
        } else if (sketch && !sketch->GetProfile().empty()) {
            loops.push_back(sketch->GetProfile());
        }

        if (loops.empty()) return;

        float h = extrude->GetHeight();
        bool useCustomPlane = sketch ? sketch->HasCustomPlane() : false;
        Geom::Plane customPlane = sketch && useCustomPlane ? sketch->GetCustomPlane() : Geom::Plane::XZ(Geom::Point3D(0, 0, 0));
        SketchPlane plane = sketch ? sketch->GetPlane() : SketchPlane::XZ_Ground;
        float px = sketch ? sketch->GetPosition()[0] : 0.0f;
        float py = sketch ? sketch->GetPosition()[1] : 0.0f;
        float pz = sketch ? sketch->GetPosition()[2] : 0.0f;

        auto Make3DPos = [&](float u, float v, float w) -> Geom::Point3D {
            if (useCustomPlane) {
                Geom::Point3D ptOnPlane = customPlane.to_world(Geom::Point2D(u, v));
                return ptOnPlane + customPlane.normal * static_cast<double>(w);
            }
            switch (plane) {
                case SketchPlane::XZ_Ground:
                    return Geom::Point3D(u + px, w + py, v + pz);
                case SketchPlane::XY_Front:
                    return Geom::Point3D(u + px, v + py, w + pz);
                case SketchPlane::YZ_Right:
                    return Geom::Point3D(w + px, v + py, u + pz);
            }
            return Geom::Point3D(u, w, v);
        };

        for (const auto& loop : loops) {
            if (loop.size() < 3) continue;

            // Project entrance loop (w = 0) and floor loop (w = h)
            std::vector<ImVec2> entranceScreenPts;
            std::vector<ImVec2> floorScreenPts;
            entranceScreenPts.reserve(loop.size());
            floorScreenPts.reserve(loop.size());

            double floorCx = 0.0, floorCy = 0.0, floorCz = 0.0;
            double entCx = 0.0, entCy = 0.0, entCz = 0.0;

            for (const auto& pt : loop) {
                Geom::Point3D pEnt = Make3DPos(pt[0], pt[1], 0.0f);
                Geom::Point3D pFlr = Make3DPos(pt[0], pt[1], h);

                entCx += pEnt.x; entCy += pEnt.y; entCz += pEnt.z;
                floorCx += pFlr.x; floorCy += pFlr.y; floorCz += pFlr.z;

                float sx0 = 0.0f, sy0 = 0.0f;
                float sx1 = 0.0f, sy1 = 0.0f;
                if (camera.WorldToScreenPoint(pEnt, vpW, vpH, sx0, sy0)) {
                    entranceScreenPts.emplace_back(sx0, sy0);
                }
                if (camera.WorldToScreenPoint(pFlr, vpW, vpH, sx1, sy1)) {
                    floorScreenPts.emplace_back(sx1, sy1);
                }
            }

            double invN = 1.0 / static_cast<double>(loop.size());
            Geom::Point3D floorCentroid(floorCx * invN, floorCy * invN, floorCz * invN);
            Geom::Point3D entranceCentroid(entCx * invN, entCy * invN, entCz * invN);

            // 1. Fill Floor polygon with translucent amber/coral tint
            if (floorScreenPts.size() == loop.size()) {
                fg->AddConvexPolyFilled(floorScreenPts.data(), static_cast<int>(floorScreenPts.size()), IM_COL32(255, 90, 20, 75));
                fg->AddPolyline(floorScreenPts.data(), static_cast<int>(floorScreenPts.size()), IM_COL32(255, 120, 20, 240), ImDrawFlags_Closed, 2.2f);
            }

            // 2. Entrance polygon outline
            if (entranceScreenPts.size() == loop.size()) {
                fg->AddPolyline(entranceScreenPts.data(), static_cast<int>(entranceScreenPts.size()), IM_COL32(255, 200, 50, 220), ImDrawFlags_Closed, 2.0f);
            }

            // 3. Side wall connecting edges
            if (entranceScreenPts.size() == loop.size() && floorScreenPts.size() == loop.size()) {
                for (size_t i = 0; i < loop.size(); ++i) {
                    fg->AddLine(entranceScreenPts[i], floorScreenPts[i], IM_COL32(255, 140, 30, 190), 1.6f);
                }
            }

            // 4. Direction line and Cut Depth callout badge at floor centroid
            float fsx = 0.0f, fsy = 0.0f;
            float esx = 0.0f, esy = 0.0f;
            bool flrVis = camera.WorldToScreenPoint(floorCentroid, vpW, vpH, fsx, fsy);
            bool entVis = camera.WorldToScreenPoint(entranceCentroid, vpW, vpH, esx, esy);

            if (entVis && flrVis) {
                // Leader line between centroids
                fg->AddLine(ImVec2(esx, esy), ImVec2(fsx, fsy), IM_COL32(255, 60, 20, 230), 2.2f);
                // Dot at entrance centroid
                fg->AddCircleFilled(ImVec2(esx, esy), 4.0f, IM_COL32(255, 200, 50, 255));
                // Dot at floor centroid
                fg->AddCircleFilled(ImVec2(fsx, fsy), 4.5f, IM_COL32(255, 60, 20, 255));
            }

            if (flrVis) {
                char depthBuf[64];
                std::snprintf(depthBuf, sizeof(depthBuf), "⤋ Cut Depth: %.2f m", std::abs(h));
                ImVec2 tSz = ImGui::CalcTextSize(depthBuf);
                ImVec2 b0(fsx + 12.0f, fsy - 10.0f);
                ImVec2 b1(b0.x + tSz.x + 16.0f, b0.y + tSz.y + 8.0f);

                fg->AddRectFilled(b0, b1, IM_COL32(18, 14, 20, 235), 4.0f);
                fg->AddRect(b0, b1, IM_COL32(255, 100, 30, 240), 4.0f, 0, 1.6f);
                fg->AddText(ImVec2(b0.x + 8.0f, b0.y + 4.0f), IM_COL32(255, 220, 150, 255), depthBuf);
            }
        }
    } else if (isRevolveCut && revolve) {
        auto sketch = revolve->GetSketchReference();
        if (sketch) {
            float ang = revolve->GetAngleDegrees();
            Geom::Plane sketchPlane;
            if (sketch->HasCustomPlane()) {
                sketchPlane = sketch->GetCustomPlane();
            } else {
                Geom::Point3D pos(sketch->GetPosition()[0], sketch->GetPosition()[1], sketch->GetPosition()[2]);
                switch (sketch->GetPlane()) {
                    case SketchPlane::XZ_Ground: sketchPlane = Geom::Plane::XZ(pos); break;
                    case SketchPlane::XY_Front:  sketchPlane = Geom::Plane::XY(pos); break;
                    case SketchPlane::YZ_Right:  sketchPlane = Geom::Plane::YZ(pos); break;
                }
            }
            Geom::Axis3D axis = revolve->ComputeEffectiveAxis(sketchPlane);

            float ax0 = 0.0f, ay0 = 0.0f, ax1 = 0.0f, ay1 = 0.0f;
            Geom::Point3D pStart = axis.origin - axis.direction * 2.0;
            Geom::Point3D pEnd = axis.origin + axis.direction * 2.0;
            if (camera.WorldToScreenPoint(pStart, vpW, vpH, ax0, ay0) &&
                camera.WorldToScreenPoint(pEnd, vpW, vpH, ax1, ay1)) {
                fg->AddLine(ImVec2(ax0, ay0), ImVec2(ax1, ay1), IM_COL32(255, 120, 20, 200), 2.0f);
            }

            float ox = 0.0f, oy = 0.0f;
            if (camera.WorldToScreenPoint(axis.origin, vpW, vpH, ox, oy)) {
                char revBuf[64];
                std::snprintf(revBuf, sizeof(revBuf), "↻ Cut Angle: %.1f°", ang);
                ImVec2 tSz = ImGui::CalcTextSize(revBuf);
                ImVec2 b0(ox + 12.0f, oy - 10.0f);
                ImVec2 b1(b0.x + tSz.x + 16.0f, b0.y + tSz.y + 8.0f);
                fg->AddRectFilled(b0, b1, IM_COL32(18, 14, 20, 235), 4.0f);
                fg->AddRect(b0, b1, IM_COL32(255, 100, 30, 240), 4.0f, 0, 1.6f);
                fg->AddText(ImVec2(b0.x + 8.0f, b0.y + 4.0f), IM_COL32(255, 220, 150, 255), revBuf);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// STEP ISO 10303-21 AP214 Exporter Modal
// -----------------------------------------------------------------------------
void CADGui::RenderExportStepModal(
    ParametricGraph& graph,
    std::shared_ptr<DAGNode> selectedNode
) {
    if (!m_showExportStepModal) return;

    ImGui::OpenPopup("Export Model to STEP (.stp)##Modal");

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(480, 0), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse;
    if (ImGui::BeginPopupModal("Export Model to STEP (.stp)##Modal", &m_showExportStepModal, flags)) {
        ImGui::TextColored(ImVec4(0.35f, 0.82f, 1.0f, 1.0f), "⬡ Save / Export Model as STEP (.stp)");
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::TextWrapped("Exports 3D solid model geometry to an industry-standard ISO 10303-21 AP214 clear text file (.stp / .step). Compatible with SolidWorks, Fusion 360, FreeCAD, and Siemens NX.");
        ImGui::Spacing();

        // Export Scope Selection
        ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Export Scope:");
        ImGui::RadioButton("Entire Model (All Active Solids in Scene)##ExportScope", &m_exportStepScope, 0);

        bool hasSelectedSolid = (selectedNode && selectedNode->IsSolidFeature() && !selectedNode->GetCachedVertices().empty());
        if (!hasSelectedSolid) {
            ImGui::BeginDisabled();
        }
        std::string selLabel = "Selected Solid Only";
        if (hasSelectedSolid) {
            selLabel += " (" + selectedNode->GetName() + ")";
        }
        selLabel += "##ExportScope";
        ImGui::RadioButton(selLabel.c_str(), &m_exportStepScope, 1);
        if (!hasSelectedSolid) {
            ImGui::EndDisabled();
        }

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Output File Destination (.stp / .step):");
        float browseBtnWidth = 92.0f;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - browseBtnWidth - 8.0f);
        ImGui::InputText("##ExportStepPath", m_exportStepPath, sizeof(m_exportStepPath));
        ImGui::SameLine();
        if (ImGui::Button("📂 Browse...", ImVec2(browseBtnWidth, 0))) {
#ifdef _WIN32
            OPENFILENAMEA ofn;
            char szFile[MAX_PATH] = "";
            if (m_exportStepPath[0] != '\0') {
                std::strncpy(szFile, m_exportStepPath, sizeof(szFile) - 1);
            } else {
                std::strncpy(szFile, "model.stp", sizeof(szFile) - 1);
            }

            ZeroMemory(&ofn, sizeof(ofn));
            ofn.lStructSize = sizeof(ofn);
            ofn.hwndOwner = NULL;
            ofn.lpstrFile = szFile;
            ofn.nMaxFile = sizeof(szFile);
            ofn.lpstrFilter = "STEP Files (*.stp;*.step)\0*.stp;*.step\0All Files (*.*)\0*.*\0";
            ofn.nFilterIndex = 1;
            ofn.lpstrDefExt = "stp";
            ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;

            if (GetSaveFileNameA(&ofn) == TRUE) {
                std::strncpy(m_exportStepPath, szFile, sizeof(m_exportStepPath) - 1);
                m_exportStepPath[sizeof(m_exportStepPath) - 1] = '\0';
                m_exportStepStatus.clear();
            }
#endif
        }

        // Geometry Summary Preview
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Model Geometry Summary:");
        if (m_exportStepScope == 0) {
            size_t totalSolids = 0;
            size_t totalFaces = 0;
            size_t totalVerts = 0;
            for (const auto& n : graph.GetNodes()) {
                if (n && n->IsSolidFeature() && !n->IsConsumed() && !n->HasConflict() && !n->GetCachedVertices().empty()) {
                    totalSolids++;
                    totalFaces += n->GetCachedIndices().size() / 3;
                    totalVerts += n->GetCachedVertices().size();
                }
            }
            ImGui::BulletText("Solids to Export: %zu", totalSolids);
            ImGui::BulletText("Total Facets:     %zu", totalFaces);
            ImGui::BulletText("Total Vertices:   %zu", totalVerts);
        } else if (hasSelectedSolid) {
            auto target = selectedNode->IsConsumed() ? selectedNode->GetActiveLeafFeature() : selectedNode;
            ImGui::BulletText("Solid:          %s", target->GetName().c_str());
            ImGui::BulletText("Total Facets:   %zu", target->GetCachedIndices().size() / 3);
            ImGui::BulletText("Total Vertices: %zu", target->GetCachedVertices().size());
        }

        // Status banner
        if (!m_exportStepStatus.empty()) {
            ImGui::Spacing();
            if (m_exportStepSuccess) {
                ImGui::TextColored(ImVec4(0.3f, 0.95f, 0.5f, 1.0f), "%s", m_exportStepStatus.c_str());
#ifdef _WIN32
                ImGui::Spacing();
                if (ImGui::Button("📁 Show in File Explorer", ImVec2(180, 26))) {
                    std::string selectArg = "/select,\"" + m_lastExportedStepFullPath + "\"";
                    ShellExecuteA(NULL, "open", "explorer.exe", selectArg.c_str(), NULL, SW_SHOWNORMAL);
                }
                ImGui::SameLine();
                if (ImGui::Button("📂 Open Folder", ImVec2(120, 26))) {
                    try {
                        std::filesystem::path p(m_lastExportedStepFullPath);
                        std::string parentDir = p.parent_path().string();
                        ShellExecuteA(NULL, "open", parentDir.c_str(), NULL, NULL, SW_SHOWNORMAL);
                    } catch (...) {}
                }
#endif
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", m_exportStepStatus.c_str());
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.55f, 0.35f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.68f, 0.42f, 1.0f));
        if (ImGui::Button("💾 Export to .STP", ImVec2(140, 32))) {
            std::string outPath = m_exportStepPath;
            if (outPath.empty()) {
                outPath = "model.stp";
            }
            if (outPath.length() < 4 || (outPath.substr(outPath.length() - 4) != ".stp" && outPath.substr(outPath.length() - 5) != ".step")) {
                outPath += ".stp";
                std::snprintf(m_exportStepPath, sizeof(m_exportStepPath), "%s", outPath.c_str());
            }

            Tessera::Export::StepExportResult res;
            if (m_exportStepScope == 1 && hasSelectedSolid) {
                res = Tessera::Export::StepExporter::ExportSolid(outPath, selectedNode);
            } else {
                res = Tessera::Export::StepExporter::ExportModel(outPath, graph);
            }

            if (res.success) {
                m_exportStepSuccess = true;
                m_lastExportedStepFullPath = res.filePath;
                char msg[512];
                std::snprintf(msg, sizeof(msg), "✓ Successfully exported %zu solid(s), %zu faces (%zu bytes)\nLocation: %s",
                              res.solidCount, res.faceCount, res.fileSizeBytes, res.filePath.c_str());
                m_exportStepStatus = msg;
            } else {
                m_exportStepSuccess = false;
                m_lastExportedStepFullPath.clear();
                m_exportStepStatus = "✕ Export failed: " + res.errorMessage;
            }
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(90, 32))) {
            m_showExportStepModal = false;
            m_exportStepStatus.clear();
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
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
        m_mouseTool = SketchMouseTool::SelectMove;

        const auto& shapes = activeSketch->GetShapes();
        const auto& savedConstraints = activeSketch->GetEdgeConstraints();
        if (!shapes.empty()) {
            for (size_t i = 0; i < shapes.size(); ++i) {
                CanvasShape cs;
                cs.name = "Shape " + std::to_string(i + 1);
                cs.points = shapes[i];
                cs.isClosed = true;
                auto it = savedConstraints.find(i);
                if (it != savedConstraints.end()) {
                    cs.edgeConstraints = it->second;
                }
                m_canvasShapes.push_back(std::move(cs));
            }
        } else if (!activeSketch->GetProfile().empty()) {
            CanvasShape cs;
            cs.name = "Shape 1";
            cs.points = activeSketch->GetProfile();
            cs.isClosed = true;
            auto it = savedConstraints.find(0);
            if (it != savedConstraints.end()) {
                cs.edgeConstraints = it->second;
            }
            m_canvasShapes.push_back(std::move(cs));
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

    ImGui::SetNextWindowPos(ImVec2(390.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(840.0f, 640.0f), ImGuiCond_FirstUseEver);

    std::string planeLabel = activeSketch->HasCustomPlane()
        ? (activeSketch->GetDatumPlaneReference() ? activeSketch->GetDatumPlaneReference()->GetName() : "Custom Reference Plane")
        : GetSketchPlaneName(activeSketch->GetPlane());
    std::string winTitle = "2D Sketch Canvas - " + activeSketch->GetName() + " (" + planeLabel + ")###SketchCanvas";
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
    // Top Tool Ribbon (Row 1: Confirm/Cancel + Drawing Tools + Loop Status)
    // -------------------------------------------------------------------------
    // Keep Confirm & Cancel pinned at the start of Row 1 so they are always on display and never cut off
    ImGui::BeginDisabled(!isSketchClosed);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.62f, 0.35f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.72f, 0.42f, 1.0f));
    if (ImGui::Button("✓ Confirm Changes", ImVec2(128, 28))) {
        std::vector<std::vector<std::array<float, 2>>> outShapes;
        std::map<size_t, std::map<size_t, float>> outConstraints;
        for (const auto& sh : m_canvasShapes) {
            if (sh.isClosed && sh.points.size() >= 3) {
                outShapes.push_back(sh.points);
                if (!sh.edgeConstraints.empty()) {
                    outConstraints[outShapes.size() - 1] = sh.edgeConstraints;
                }
            }
        }
        if (outShapes.empty()) {
            m_showEmptySketchExitPrompt = true;
            m_emptySketchToExitId = activeSketch->GetId();
        } else {
            activeSketch->SetShapes(std::move(outShapes));
            activeSketch->SetEdgeConstraints(std::move(outConstraints));
            graph.EvaluateDirtyNodes(device, exchange);
            m_sketchModeActive = false;
        }
    }
    ImGui::PopStyleColor(2);
    ImGui::EndDisabled();

    if (!isSketchClosed && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Cannot confirm sketch:\n%s\nAll shapes must be closed loops to apply.", unclosedReason.c_str());
    }
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.70f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("✕ Cancel", ImVec2(68, 28))) {
        if (m_canvasShapes.empty() && activeSketch->GetShapes().empty()) {
            m_showEmptySketchExitPrompt = true;
            m_emptySketchToExitId = activeSketch->GetId();
        } else {
            m_sketchModeActive = false;
        }
    }
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Discard changes and close sketch canvas.");
    }
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    auto ToolBtn = [&](SketchMouseTool tool, const char* label, float width, const char* tooltip) {
        bool isSel = (m_mouseTool == tool);
        if (isSel) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.58f, 0.95f, 1.0f));
        }
        if (ImGui::Button(label, ImVec2(width, 28))) {
            CloseMeasurement();
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

    ToolBtn(SketchMouseTool::SelectMove, "Select/Move", 78.0f, "Select & drag vertices to reshape (V)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Rectangle, "Rectangle", 72.0f, "Click and drag diagonally to draw rectangle (R)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Circle, "Circle", 58.0f, "Click center and drag outward to draw circle (C)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Line, "Line/Poly", 72.0f, "Click vertices to draw polyline, click first point / double click to close (L)");
    ImGui::SameLine();
    ToolBtn(SketchMouseTool::Spline, "Spline", 58.0f, "Click control points for smooth spline, click first / double click to close (S)");
    ImGui::SameLine();

    // Quick loop close button if in-progress has points
    if (!m_inProgressPoints.empty() && m_inProgressPoints.size() >= 3) {
        if (ImGui::Button("Close Loop", ImVec2(76, 28))) {
            m_canvasShapes.push_back({
                "Polyline " + std::to_string(m_canvasShapes.size() + 1),
                m_inProgressPoints,
                true
            });
            m_inProgressPoints.clear();
        }
        ImGui::SameLine();
    } else if (!m_inProgressSplineControls.empty() && m_inProgressSplineControls.size() >= 3) {
        if (ImGui::Button("Close Spline", ImVec2(80, 28))) {
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

    // Closure Status Badge
    if (isSketchClosed) {
        ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.4f, 1.0f), "● Closed Loop(s)");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.25f, 1.0f), "▲ Unclosed");
    }

    // -------------------------------------------------------------------------
    // Top Tool Ribbon (Row 2: Grid/Snap, View & Shape Utilities)
    // -------------------------------------------------------------------------
    if (ImGui::Button("⚡ Stitch", ImVec2(72, 24))) {
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
    ImGui::SetNextItemWidth(72.0f);
    ImGui::InputFloat("##Grid", &m_gridSnapSize, 0.1f, 0.5f, "%.1fm");
    if (m_gridSnapSize < 0.05f) m_gridSnapSize = 0.05f;
    ImGui::SameLine();

    if (ImGui::Button("Center", ImVec2(56, 24))) {
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

    if (!m_canvasShapes.empty()) {
        ImGui::SameLine();
        if (ImGui::Button("Clear All", ImVec2(68, 24))) {
            m_canvasShapes.clear();
            m_selectedShapeIdx = -1;
            m_selectedVertexIdx = -1;
            m_inProgressPoints.clear();
            m_inProgressSplineControls.clear();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Clear all shapes from sketch canvas");
    }

    // Contextual edit row for selected vertex or shape (Fillet / Chamfer / Delete)
    if (m_selectedShapeIdx >= 0 && m_selectedShapeIdx < static_cast<int>(m_canvasShapes.size())) {
        auto& selShape = m_canvasShapes[m_selectedShapeIdx];
        if (m_selectedVertexIdx >= 0 && m_selectedVertexIdx < static_cast<int>(selShape.points.size())) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.28f, 0.28f, 1.0f));
            if (ImGui::Button("🗑 Del Vertex", ImVec2(92, 24))) {
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

            if (ImGui::RadioButton("Dist##SkFilletChoice", m_filletChoice == FilletDimensionChoice::Distance)) {
                m_filletChoice = FilletDimensionChoice::Distance;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Specify Fillet by Distance / Radius");
            ImGui::SameLine();
            if (ImGui::RadioButton("Angle##SkFilletChoice", m_filletChoice == FilletDimensionChoice::Angle)) {
                m_filletChoice = FilletDimensionChoice::Angle;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Specify Fillet by Angle in degrees");
            ImGui::SameLine();

            if (m_filletChoice == FilletDimensionChoice::Distance) {
                ImGui::SetNextItemWidth(65.0f);
                ImGui::InputFloat("##SkFilletR", &m_sketchFilletRadius, 0.05f, 0.1f, "%.2fm");
                if (m_sketchFilletRadius < 0.01f) m_sketchFilletRadius = 0.01f;
            } else {
                ImGui::SetNextItemWidth(65.0f);
                ImGui::InputFloat("##SkFilletAngle", &m_sketchFilletAngle, 1.0f, 5.0f, "%.1f°");
                m_sketchFilletAngle = std::clamp(m_sketchFilletAngle, 5.0f, 175.0f);
            }
            ImGui::SameLine();
            if (ImGui::Button("⌒ Fillet", ImVec2(64, 24))) {
                bool isAngle = (m_filletChoice == FilletDimensionChoice::Angle);
                float val = isAngle ? m_sketchFilletAngle : m_sketchFilletRadius;
                if (Sketch::ApplyFillet2D(selShape.points, static_cast<size_t>(m_selectedVertexIdx), val, selShape.isClosed, 6, isAngle, m_sketchFilletRadius)) {
                    m_selectedVertexIdx = -1;
                }
            }
            if (ImGui::IsItemHovered()) {
                if (m_filletChoice == FilletDimensionChoice::Distance) {
                    ImGui::SetTooltip("Apply round fillet arc with distance/radius %.2fm to selected corner vertex", m_sketchFilletRadius);
                } else {
                    ImGui::SetTooltip("Apply round fillet arc with angle %.1f° to selected corner vertex", m_sketchFilletAngle);
                }
            }
            ImGui::SameLine();

            ImGui::SetNextItemWidth(68.0f);
            ImGui::InputFloat("##SkChamferD", &m_sketchChamferDist, 0.05f, 0.1f, "%.2fm");
            if (m_sketchChamferDist < 0.01f) m_sketchChamferDist = 0.01f;
            ImGui::SameLine();
            if (ImGui::Button("⟍ Chamfer", ImVec2(76, 24))) {
                if (Sketch::ApplyChamfer2D(selShape.points, static_cast<size_t>(m_selectedVertexIdx), m_sketchChamferDist, selShape.isClosed)) {
                    m_selectedVertexIdx = -1;
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply beveled chamfer of distance %.2fm to selected corner vertex", m_sketchChamferDist);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.28f, 0.28f, 1.0f));
            if (ImGui::Button("🗑 Del Shape", ImVec2(90, 24))) {
                m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                m_selectedShapeIdx = -1;
                m_selectedVertexIdx = -1;
            }
            ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Delete selected shape (Del)");
            ImGui::SameLine();

            if (m_filletChoice == FilletDimensionChoice::Distance) {
                ImGui::SetNextItemWidth(65.0f);
                ImGui::InputFloat("##AllFilletR", &m_sketchFilletRadius, 0.05f, 0.1f, "%.2fm");
                if (m_sketchFilletRadius < 0.01f) m_sketchFilletRadius = 0.01f;
            } else {
                ImGui::SetNextItemWidth(65.0f);
                ImGui::InputFloat("##AllFilletAngle", &m_sketchFilletAngle, 1.0f, 5.0f, "%.1f°");
                m_sketchFilletAngle = std::clamp(m_sketchFilletAngle, 5.0f, 175.0f);
            }
            ImGui::SameLine();
            if (ImGui::Button("⌒ Fillet All", ImVec2(78, 24))) {
                bool isAngle = (m_filletChoice == FilletDimensionChoice::Angle);
                float val = isAngle ? m_sketchFilletAngle : m_sketchFilletRadius;
                size_t origSize = selShape.points.size();
                for (int c = static_cast<int>(origSize) - 1; c >= 0; --c) {
                    Sketch::ApplyFillet2D(selShape.points, static_cast<size_t>(c), val, selShape.isClosed, 6, isAngle, m_sketchFilletRadius);
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply fillet arc to all corners of this shape");
            ImGui::SameLine();

            ImGui::SetNextItemWidth(68.0f);
            ImGui::InputFloat("##AllChamferD", &m_sketchChamferDist, 0.05f, 0.1f, "%.2fm");
            if (m_sketchChamferDist < 0.01f) m_sketchChamferDist = 0.01f;
            ImGui::SameLine();
            if (ImGui::Button("⟍ Chamfer All", ImVec2(90, 24))) {
                size_t origSize = selShape.points.size();
                for (int c = static_cast<int>(origSize) - 1; c >= 0; --c) {
                    Sketch::ApplyChamfer2D(selShape.points, static_cast<size_t>(c), m_sketchChamferDist, selShape.isClosed);
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply beveled chamfer to all corners of this shape");
        }
    }

    // -------------------------------------------------------------------------
    // Top Tool Ribbon (Row 3: Keyboard Dimensions & Precision CAD Inputs)
    // -------------------------------------------------------------------------
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.15f, 0.20f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.24f, 0.45f, 0.70f, 0.85f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
    if (ImGui::BeginChild("SketchKeyboardDimensionsBar", ImVec2(0, 36.0f), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
        ImGui::AlignTextToFramePadding();

        bool enterPressed = ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter);

        if (m_selectedShapeIdx >= 0 && m_selectedShapeIdx < static_cast<int>(m_canvasShapes.size())) {
            auto& selShape = m_canvasShapes[m_selectedShapeIdx];

            if (m_selectedVertexIdx >= 0 && m_selectedVertexIdx < static_cast<int>(selShape.points.size())) {
                ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "⌨ Vertex #%d:", m_selectedVertexIdx);
                ImGui::SameLine();
                float vertU = selShape.points[m_selectedVertexIdx][0];
                float vertV = selShape.points[m_selectedVertexIdx][1];
                ImGui::SetNextItemWidth(75.0f);
                if (ImGui::InputFloat("U##VertU", &vertU, 0.1f, 0.5f, "%.2f m")) {
                    selShape.points[m_selectedVertexIdx][0] = vertU;
                    EnforceShapeConstraints(selShape, m_selectedVertexIdx);
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(75.0f);
                if (ImGui::InputFloat("V##VertV", &vertV, 0.1f, 0.5f, "%.2f m")) {
                    selShape.points[m_selectedVertexIdx][1] = vertV;
                    EnforceShapeConstraints(selShape, m_selectedVertexIdx);
                }

                size_t nPts = selShape.points.size();
                if (nPts >= 2) {
                    size_t prevIdx = (m_selectedVertexIdx + nPts - 1) % nPts;
                    float prevD = std::hypot(vertU - selShape.points[prevIdx][0], vertV - selShape.points[prevIdx][1]);
                    ImGui::SameLine();
                    ImGui::TextDisabled("|");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(75.0f);
                    if (ImGui::InputFloat("Prev Len##VertPrevLen", &prevD, 0.1f, 0.5f, "%.2f m")) {
                        if (prevD > 0.001f) {
                            float dU = vertU - selShape.points[prevIdx][0];
                            float dV = vertV - selShape.points[prevIdx][1];
                            float curLen = std::hypot(dU, dV);
                            if (curLen > 1e-4f) {
                                selShape.points[m_selectedVertexIdx][0] = selShape.points[prevIdx][0] + (dU / curLen) * prevD;
                                selShape.points[m_selectedVertexIdx][1] = selShape.points[prevIdx][1] + (dV / curLen) * prevD;
                                EnforceShapeConstraints(selShape, m_selectedVertexIdx);
                            }
                        }
                    }
                    size_t nextIdx = (m_selectedVertexIdx + 1) % nPts;
                    float nextD = std::hypot(selShape.points[nextIdx][0] - vertU, selShape.points[nextIdx][1] - vertV);
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(75.0f);
                    if (ImGui::InputFloat("Next Len##VertNextLen", &nextD, 0.1f, 0.5f, "%.2f m")) {
                        if (nextD > 0.001f) {
                            float dU = selShape.points[nextIdx][0] - vertU;
                            float dV = selShape.points[nextIdx][1] - vertV;
                            float curLen = std::hypot(dU, dV);
                            if (curLen > 1e-4f) {
                                selShape.points[nextIdx][0] = vertU + (dU / curLen) * nextD;
                                selShape.points[nextIdx][1] = vertV + (dV / curLen) * nextD;
                                EnforceShapeConstraints(selShape, m_selectedVertexIdx);
                            }
                        }
                    }
                }
            } else if (m_selectedEdgeSegmentIdx >= 0 && m_selectedEdgeSegmentIdx < static_cast<int>(selShape.points.size())) {
                size_t nPts = selShape.points.size();
                size_t i0 = m_selectedEdgeSegmentIdx;
                size_t i1 = (i0 + 1) % nPts;
                float edgeLen = std::hypot(selShape.points[i1][0] - selShape.points[i0][0], selShape.points[i1][1] - selShape.points[i0][1]);
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "⌨ Edge #%zu:", i0);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0f);
                if (ImGui::InputFloat("Length##EdgeLen", &edgeLen, 0.1f, 0.5f, "%.2f m")) {
                    if (edgeLen > 0.001f) {
                        float dU = selShape.points[i1][0] - selShape.points[i0][0];
                        float dV = selShape.points[i1][1] - selShape.points[i0][1];
                        float curLen = std::hypot(dU, dV);
                        if (curLen > 1e-4f) {
                            float deltaL = edgeLen - curLen;
                            float ux = dU / curLen;
                            float uy = dV / curLen;
                            if (selShape.isClosed && selShape.points.size() == 4) {
                                size_t i2 = (i0 + 2) % 4;
                                selShape.points[i1][0] += ux * deltaL;
                                selShape.points[i1][1] += uy * deltaL;
                                selShape.points[i2][0] += ux * deltaL;
                                selShape.points[i2][1] += uy * deltaL;
                            } else {
                                selShape.points[i1][0] = selShape.points[i0][0] + ux * edgeLen;
                                selShape.points[i1][1] = selShape.points[i0][1] + uy * edgeLen;
                            }
                            if (selShape.IsEdgeConstrained(m_selectedEdgeSegmentIdx)) {
                                selShape.SetEdgeConstraint(m_selectedEdgeSegmentIdx, edgeLen);
                            }
                            EnforceShapeConstraints(selShape, static_cast<int>(i0));
                        }
                    }
                }
                ImGui::SameLine();
                bool isConstrained = selShape.IsEdgeConstrained(m_selectedEdgeSegmentIdx);
                if (isConstrained) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.55f, 0.10f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.65f, 0.15f, 1.0f));
                    if (ImGui::SmallButton("🔒 Constrained (Unlock)")) {
                        selShape.RemoveEdgeConstraint(m_selectedEdgeSegmentIdx);
                    }
                    ImGui::PopStyleColor(2);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("This dimension is CONSTRAINED/LOCKED.\nSubsequent edits to other dimensions will not alter this length.\nClick to toggle OFF constraint.");
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.30f, 0.40f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.40f, 0.55f, 1.0f));
                    if (ImGui::SmallButton("🔓 Lock / Constrain")) {
                        selShape.SetEdgeConstraint(m_selectedEdgeSegmentIdx, edgeLen);
                    }
                    ImGui::PopStyleColor(2);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Click to toggle ON constraint (locks this edge to %.2f m)", edgeLen);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("< Prev Edge")) {
                    m_selectedEdgeSegmentIdx = (m_selectedEdgeSegmentIdx + static_cast<int>(nPts) - 1) % static_cast<int>(nPts);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("Next Edge >")) {
                    m_selectedEdgeSegmentIdx = (m_selectedEdgeSegmentIdx + 1) % static_cast<int>(nPts);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("Deselect Edge")) {
                    m_selectedEdgeSegmentIdx = -1;
                }
            } else {
                // Calculate bounding box and center
                float minU = 1e9f, maxU = -1e9f, minV = 1e9f, maxV = -1e9f;
                for (const auto& pt : selShape.points) {
                    minU = std::min(minU, pt[0]); maxU = std::max(maxU, pt[0]);
                    minV = std::min(minV, pt[1]); maxV = std::max(maxV, pt[1]);
                }
                float shW = (maxU > minU) ? (maxU - minU) : 0.01f;
                float shH = (maxV > minV) ? (maxV - minV) : 0.01f;
                float cU = (minU + maxU) * 0.5f;
                float cV = (minV + maxV) * 0.5f;

                // Detect if circular
                bool isCircle = (selShape.points.size() >= 16);
                float avgR = 0.0f;
                if (isCircle) {
                    for (const auto& pt : selShape.points) avgR += std::hypot(pt[0] - cU, pt[1] - cV);
                    avgR /= selShape.points.size();
                    for (const auto& pt : selShape.points) {
                        if (std::abs(std::hypot(pt[0] - cU, pt[1] - cV) - avgR) > 0.06f * avgR) { isCircle = false; break; }
                    }
                }

                if (isCircle) {
                    ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "⌨ Circle '%s':", selShape.name.c_str());
                    ImGui::SameLine();
                    float r = avgR;
                    ImGui::SetNextItemWidth(75.0f);
                    if (ImGui::InputFloat("Radius##ShR", &r, 0.1f, 0.5f, "%.2f m")) {
                        if (r > 0.01f && avgR > 0.001f) {
                            float scale = r / avgR;
                            for (auto& pt : selShape.points) {
                                pt[0] = cU + (pt[0] - cU) * scale;
                                pt[1] = cV + (pt[1] - cV) * scale;
                            }
                        }
                    }
                    ImGui::SameLine();
                    float d = avgR * 2.0f;
                    ImGui::SetNextItemWidth(75.0f);
                    if (ImGui::InputFloat("Diameter##ShD", &d, 0.1f, 0.5f, "%.2f m")) {
                        if (d > 0.02f && avgR > 0.001f) {
                            float scale = (d * 0.5f) / avgR;
                            for (auto& pt : selShape.points) {
                                pt[0] = cU + (pt[0] - cU) * scale;
                                pt[1] = cV + (pt[1] - cV) * scale;
                            }
                        }
                    }
                } else {
                    ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "⌨ Shape '%s':", selShape.name.c_str());
                    ImGui::SameLine();
                    float w = shW;
                    ImGui::SetNextItemWidth(75.0f);
                    if (ImGui::InputFloat("Width##ShW", &w, 0.1f, 0.5f, "%.2f m")) {
                        if (w > 0.01f && shW > 0.001f) {
                            float scale = w / shW;
                            for (auto& pt : selShape.points) {
                                pt[0] = cU + (pt[0] - cU) * scale;
                            }
                            EnforceShapeConstraints(selShape);
                        }
                    }
                    ImGui::SameLine();
                    float h = shH;
                    ImGui::SetNextItemWidth(75.0f);
                    if (ImGui::InputFloat("Height##ShH", &h, 0.1f, 0.5f, "%.2f m")) {
                        if (h > 0.01f && shH > 0.001f) {
                            float scale = h / shH;
                            for (auto& pt : selShape.points) {
                                pt[1] = cV + (pt[1] - cV) * scale;
                            }
                            EnforceShapeConstraints(selShape);
                        }
                    }
                }
                ImGui::SameLine();
                float newCU = cU;
                ImGui::SetNextItemWidth(70.0f);
                if (ImGui::InputFloat("Center U##ShCU", &newCU, 0.1f, 0.5f, "%.2f m")) {
                    float dU = newCU - cU;
                    for (auto& pt : selShape.points) pt[0] += dU;
                }
                ImGui::SameLine();
                float newCV = cV;
                ImGui::SetNextItemWidth(70.0f);
                if (ImGui::InputFloat("Center V##ShCV", &newCV, 0.1f, 0.5f, "%.2f m")) {
                    float dV = newCV - cV;
                    for (auto& pt : selShape.points) pt[1] += dV;
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("Select First Edge")) {
                    m_selectedEdgeSegmentIdx = 0;
                }
            }
        } else {
            if (m_mouseTool == SketchMouseTool::Rectangle) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 0.45f, 1.0f), "⌨ New Rectangle:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(75.0f);
                bool enterW = ImGui::InputFloat("Width##KbRW", &m_kbRectWidth, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                if (m_kbRectWidth < 0.05f) m_kbRectWidth = 0.05f;
                ImGui::SameLine();
                ImGui::SetNextItemWidth(75.0f);
                bool enterH = ImGui::InputFloat("Height##KbRH", &m_kbRectHeight, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                if (m_kbRectHeight < 0.05f) m_kbRectHeight = 0.05f;
                ImGui::SameLine();
                ImGui::SetNextItemWidth(68.0f);
                bool enterCU = ImGui::InputFloat("Center U##KbRCU", &m_kbRectCenterU, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(68.0f);
                bool enterCV = ImGui::InputFloat("Center V##KbRCV", &m_kbRectCenterV, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                bool addRect = ImGui::Button("+ Add Rect (Enter)", ImVec2(130, 22)) || enterW || enterH || enterCU || enterCV || enterPressed;
                if (addRect) {
                    float hw = m_kbRectWidth * 0.5f;
                    float hh = m_kbRectHeight * 0.5f;
                    m_canvasShapes.push_back({
                        "Rectangle " + std::to_string(m_canvasShapes.size() + 1),
                        { {m_kbRectCenterU - hw, m_kbRectCenterV - hh},
                          {m_kbRectCenterU + hw, m_kbRectCenterV - hh},
                          {m_kbRectCenterU + hw, m_kbRectCenterV + hh},
                          {m_kbRectCenterU - hw, m_kbRectCenterV + hh} },
                        true
                    });
                    m_selectedShapeIdx = static_cast<int>(m_canvasShapes.size() - 1);
                }
            } else if (m_mouseTool == SketchMouseTool::Circle) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 0.45f, 1.0f), "⌨ New Circle:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(75.0f);
                bool enterR = ImGui::InputFloat("Radius##KbCR", &m_kbCircleRadius, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                if (m_kbCircleRadius < 0.05f) m_kbCircleRadius = 0.05f;
                ImGui::SameLine();
                ImGui::SetNextItemWidth(68.0f);
                bool enterCU = ImGui::InputFloat("Center U##KbCCU", &m_kbCircleCenterU, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(68.0f);
                bool enterCV = ImGui::InputFloat("Center V##KbCCV", &m_kbCircleCenterV, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                bool addCircle = ImGui::Button("+ Add Circle (Enter)", ImVec2(130, 22)) || enterR || enterCU || enterCV || enterPressed;
                if (addCircle) {
                    const int segs = 32;
                    std::vector<std::array<float, 2>> profile;
                    profile.reserve(segs);
                    for (int i = 0; i < segs; ++i) {
                        float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 6.2831853f;
                        profile.push_back({m_kbCircleCenterU + m_kbCircleRadius * std::cos(theta), m_kbCircleCenterV + m_kbCircleRadius * std::sin(theta)});
                    }
                    m_canvasShapes.push_back({
                        "Circle " + std::to_string(m_canvasShapes.size() + 1),
                        std::move(profile),
                        true
                    });
                    m_selectedShapeIdx = static_cast<int>(m_canvasShapes.size() - 1);
                }
            } else if (m_mouseTool == SketchMouseTool::Line) {
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 0.45f, 1.0f), "⌨ Next Line Segment:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(72.0f);
                bool enterLen = ImGui::InputFloat("Length##KbLLen", &m_kbLineLength, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                if (m_kbLineLength < 0.01f) m_kbLineLength = 0.01f;
                ImGui::SameLine();
                ImGui::SetNextItemWidth(65.0f);
                bool enterAng = ImGui::InputFloat("Angle##KbLAng", &m_kbLineAngle, 5.0f, 15.0f, "%.1f°", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(65.0f);
                bool enterTU = ImGui::InputFloat("Target U##KbLTU", &m_kbLineTargetU, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(65.0f);
                bool enterTV = ImGui::InputFloat("Target V##KbLTV", &m_kbLineTargetV, 0.1f, 0.5f, "%.2f m", ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                bool addPt = ImGui::Button("+ Add Segment (Enter)", ImVec2(140, 22)) || enterLen || enterAng || enterTU || enterTV || enterPressed;
                if (addPt) {
                    if (m_inProgressPoints.empty()) {
                        m_inProgressPoints.push_back({m_kbLineTargetU, m_kbLineTargetV});
                    } else {
                        float rad = m_kbLineAngle * 3.14159265f / 180.0f;
                        const auto& last = m_inProgressPoints.back();
                        float nextU = (enterTU || enterTV) ? m_kbLineTargetU : (last[0] + m_kbLineLength * std::cos(rad));
                        float nextV = (enterTU || enterTV) ? m_kbLineTargetV : (last[1] + m_kbLineLength * std::sin(rad));
                        m_inProgressPoints.push_back({nextU, nextV});
                        m_kbLineTargetU = nextU;
                        m_kbLineTargetV = nextV;
                    }
                }
                if (m_inProgressPoints.size() >= 3) {
                    ImGui::SameLine();
                    if (ImGui::Button("Close Loop", ImVec2(80, 22))) {
                        m_canvasShapes.push_back({
                            "Polyline " + std::to_string(m_canvasShapes.size() + 1),
                            m_inProgressPoints,
                            true
                        });
                        m_inProgressPoints.clear();
                    }
                }
            } else {
                ImGui::TextColored(ImVec4(0.7f, 0.8f, 0.9f, 1.0f), "⌨ Select a shape, edge, or vertex handle to edit dimensions directly via keyboard.");
            }
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);

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

    // Esc: cancel whatever is being drawn and go back to Select/Move; in Select/Move it clears the
    // selection. Works without hovering the canvas, but not while typing into a field.
    if (!io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        if (m_mouseTool != SketchMouseTool::SelectMove || m_isDrawing || !m_inProgressPoints.empty() ||
            !m_inProgressSplineControls.empty()) {
            m_mouseTool = SketchMouseTool::SelectMove;
        } else if (m_draggedShapeIdx < 0) {
            m_selectedShapeIdx = -1;
            m_selectedVertexIdx = -1;
            m_selectedEdgeSegmentIdx = -1;
        }
        m_isDrawing = false;
        m_inProgressPoints.clear();
        m_inProgressSplineControls.clear();
        if (m_draggedShapeIdx >= 0 && m_draggedVertexIdx < 0 &&
            m_draggedShapeIdx < static_cast<int>(m_canvasShapes.size()) &&
            m_canvasShapes[m_draggedShapeIdx].points.size() == m_dragShapeOrigPoints.size()) {
            m_canvasShapes[m_draggedShapeIdx].points = m_dragShapeOrigPoints; // Esc mid-drag puts the shape back
        }
        m_draggedShapeIdx = -1;
        m_draggedVertexIdx = -1;
        m_dragShapeOrigPoints.clear();
    }

    // Canvas navigation: Pan & Zoom
    if (isHovered) {
        if (ImGui::IsKeyPressed(ImGuiKey_V)) { CloseMeasurement(); m_mouseTool = SketchMouseTool::SelectMove; }
        if (ImGui::IsKeyPressed(ImGuiKey_R)) { CloseMeasurement(); m_mouseTool = SketchMouseTool::Rectangle; }
        if (ImGui::IsKeyPressed(ImGuiKey_C)) { CloseMeasurement(); m_mouseTool = SketchMouseTool::Circle; }
        if (ImGui::IsKeyPressed(ImGuiKey_L)) { CloseMeasurement(); m_mouseTool = SketchMouseTool::Line; }
        if (ImGui::IsKeyPressed(ImGuiKey_S)) { CloseMeasurement(); m_mouseTool = SketchMouseTool::Spline; }

        // Delete / Backspace key to delete selected vertex or selected shape
        if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
            if (m_selectedShapeIdx >= 0 && m_selectedShapeIdx < static_cast<int>(m_canvasShapes.size())) {
                auto& selShape = m_canvasShapes[m_selectedShapeIdx];
                if (m_selectedVertexIdx >= 0 && m_selectedVertexIdx < static_cast<int>(selShape.points.size())) {
                    selShape.points.erase(selShape.points.begin() + m_selectedVertexIdx);
                    m_selectedVertexIdx = -1;
                    m_selectedEdgeSegmentIdx = -1;
                    if (selShape.points.size() < 3) selShape.isClosed = false;
                    if (selShape.points.empty()) {
                        m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                        m_selectedShapeIdx = -1;
                    }
                } else {
                    m_canvasShapes.erase(m_canvasShapes.begin() + m_selectedShapeIdx);
                    m_selectedShapeIdx = -1;
                    m_selectedVertexIdx = -1;
                    m_selectedEdgeSegmentIdx = -1;
                }
            }
        }

        if (io.MouseWheel != 0.0f) {
            float oldZoom = m_canvasZoom;
            bool isSmooth = io.KeyShift;
            float step = isSmooth ? 1.04f : 1.15f;
            float factor = (io.MouseWheel > 0.0f) ? step : (1.0f / step);
            float newZoom = std::clamp(m_canvasZoom * factor, 5.0f, 600.0f);

            float mouseU = (io.MousePos.x - canvasP0.x - m_canvasPanX) / oldZoom;
            float mouseV = -(io.MousePos.y - canvasP0.y - m_canvasPanY) / oldZoom;

            m_canvasZoom = newZoom;
            m_canvasPanX = io.MousePos.x - canvasP0.x - mouseU * newZoom;
            m_canvasPanY = io.MousePos.y - canvasP0.y + mouseV * newZoom;
        }

        if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) || ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
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

                    // Subtle reference lines: thin, muted grey-blue, well below the sketch's own geometry.
                    // Edges lying on the sketch plane (the face being sketched on) stay readable; edges
                    // elsewhere on the model are only a faint hint.
                    const bool onPlane = std::abs(sketchPlane.distance_to(p0)) < 0.05 &&
                                         std::abs(sketchPlane.distance_to(p1)) < 0.05;
                    drawList->AddLine(s0, s1, onPlane ? IM_COL32(130, 150, 175, 110) : IM_COL32(120, 135, 155, 40), 1.0f);
                }
            }

            // Small, dim dots for vertices on the sketch plane (still usable as snap targets)
            for (const auto& v : rVerts) {
                Geom::Point3D pt(v.position[0], v.position[1], v.position[2]);
                if (std::abs(sketchPlane.distance_to(pt)) < 0.05) {
                    Geom::Point2D uv = sketchPlane.to_local(pt);
                    ImVec2 sPos = ToScreen(static_cast<float>(uv.u), static_cast<float>(uv.v));
                    drawList->AddCircleFilled(sPos, 2.0f, IM_COL32(140, 160, 185, 120));
                }
            }

            // Reference feature tag, kept quiet
            std::string refBadge = "Ref: " + refFeature->GetName();
            drawList->AddText(ImVec2(canvasP0.x + 12.0f, canvasP0.y + 10.0f), IM_COL32(140, 150, 165, 140), refBadge.c_str());
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

    // Calculate hover states across shapes, vertices, and edges
    if (m_draggedShapeIdx < 0) {
        m_hoveredShapeIdx = -1;
        m_hoveredVertexIdx = -1;
        m_hoveredEdgeIdx = -1;
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

        // If no vertex handle is hovered, check if cursor is over a shape edge or body
        if (m_hoveredVertexIdx < 0) {
            for (int s = static_cast<int>(m_canvasShapes.size()) - 1; s >= 0; --s) {
                const auto& sh = m_canvasShapes[s];
                size_t numEdges = sh.isClosed ? sh.points.size() : (sh.points.size() > 1 ? sh.points.size() - 1 : 0);
                for (size_t i = 0; i < numEdges; ++i) {
                    size_t next = (i + 1) % sh.points.size();
                    ImVec2 p0 = ToScreen(sh.points[i][0], sh.points[i][1]);
                    ImVec2 p1 = ToScreen(sh.points[next][0], sh.points[next][1]);
                    if (DistToSegment(io.MousePos, p0, p1) < 8.0f) {
                        m_hoveredShapeIdx = s;
                        m_hoveredEdgeIdx = static_cast<int>(i);
                        break;
                    }
                }
                if (m_hoveredShapeIdx >= 0) break;
                if (sh.points.size() >= 3 && Sketch::PointInPolygon2D({rawU, rawV}, sh.points)) {
                    m_hoveredShapeIdx = s;
                    break;
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // Double-Click: edit dimension and lock/constrain line (works across tools)
    // -------------------------------------------------------------------------
    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && isHovered) {
        int hitShape = -1;
        int hitEdge = -1;
        float bestDist = 20.0f; // pixel tolerance for line or dimension badge

        for (size_t s = 0; s < m_canvasShapes.size(); ++s) {
            const auto& sh = m_canvasShapes[s];
            if (sh.points.size() < 2) continue;
            size_t numEdges = sh.isClosed ? sh.points.size() : (sh.points.size() > 1 ? sh.points.size() - 1 : 0);
            for (size_t i = 0; i < numEdges; ++i) {
                size_t next = (i + 1) % sh.points.size();
                ImVec2 p0 = ToScreen(sh.points[i][0], sh.points[i][1]);
                ImVec2 p1 = ToScreen(sh.points[next][0], sh.points[next][1]);
                float dSeg = DistToSegment(io.MousePos, p0, p1);

                float dx = p1.x - p0.x;
                float dy = p1.y - p0.y;
                float segLen = std::hypot(dx, dy);
                float offX = 0.0f, offY = 0.0f;
                if (segLen > 1e-3f) {
                    offX = (-dy / segLen) * 14.0f;
                    offY = (dx / segLen) * 14.0f;
                }
                ImVec2 mid = ImVec2((p0.x + p1.x) * 0.5f + offX, (p0.y + p1.y) * 0.5f + offY);
                float dMid = std::hypot(io.MousePos.x - mid.x, io.MousePos.y - mid.y);
                float d = std::min(dSeg, dMid);
                if (d < bestDist) {
                    bestDist = d;
                    hitShape = static_cast<int>(s);
                    hitEdge = static_cast<int>(i);
                }
            }
        }

        if (hitShape >= 0 && hitEdge >= 0) {
            auto& sh = m_canvasShapes[hitShape];
            size_t next = (hitEdge + 1) % sh.points.size();
            float curLen = std::hypot(sh.points[next][0] - sh.points[hitEdge][0],
                                      sh.points[next][1] - sh.points[hitEdge][1]);
            if (sh.IsEdgeConstrained(hitEdge)) {
                m_dimEditValue = sh.GetEdgeConstrainedLength(hitEdge);
            } else {
                m_dimEditValue = curLen;
            }
            m_dimEditConstrained = true; // Auto-checked: "remember that this is 'constraining' the drawing"
            m_dimEditShapeIdx = hitShape;
            m_dimEditEdgeIdx = hitEdge;
            m_dimEditPopupPos = { io.MousePos.x, io.MousePos.y };
            m_showDimensionEditPopup = true;
            m_selectedShapeIdx = hitShape;
            m_selectedEdgeSegmentIdx = hitEdge;
            m_selectedVertexIdx = -1;
            m_draggedShapeIdx = -1;
            m_draggedVertexIdx = -1;
            m_dragShapeOrigPoints.clear();
            m_mouseTool = SketchMouseTool::SelectMove;
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
                // Crosshairs on selected vertex
                drawList->AddLine(ImVec2(ptScr.x - 12.0f, ptScr.y), ImVec2(ptScr.x + 12.0f, ptScr.y), IM_COL32(255, 230, 0, 220), 1.5f);
                drawList->AddLine(ImVec2(ptScr.x, ptScr.y - 12.0f), ImVec2(ptScr.x, ptScr.y + 12.0f), IM_COL32(255, 230, 0, 220), 1.5f);
                char vPosBuf[64];
                snprintf(vPosBuf, sizeof(vPosBuf), "Vertex #%zu (%.2f, %.2f) m", i, shape.points[i][0], shape.points[i][1]);
                drawList->AddText(ImVec2(ptScr.x + 10.0f, ptScr.y - 18.0f), IM_COL32(255, 230, 50, 255), vPosBuf);
            } else if (isHoveredPt || isDraggedPt) {
                char buf[64];
                snprintf(buf, sizeof(buf), "%s #%zu (%.2f, %.2f) [RMB: Del]", shape.name.c_str(), i, shape.points[i][0], shape.points[i][1]);
                drawList->AddText(ImVec2(ptScr.x + 8.0f, ptScr.y - 14.0f), IM_COL32(255, 255, 100, 255), buf);
            }
        }

        // ---------------------------------------------------------------------
        // CAD Dimension Annotations (Extension lines, Arrows & Numerical Labels)
        // ---------------------------------------------------------------------
        if (isShapeSelected) {
            float minU = 1e9f, maxU = -1e9f, minV = 1e9f, maxV = -1e9f;
            for (const auto& pt : shape.points) {
                minU = std::min(minU, pt[0]); maxU = std::max(maxU, pt[0]);
                minV = std::min(minV, pt[1]); maxV = std::max(maxV, pt[1]);
            }
            float cU = (minU + maxU) * 0.5f;
            float cV = (minV + maxV) * 0.5f;

            // Check if circle
            bool isCircle = (shape.points.size() >= 16);
            float avgR = 0.0f;
            if (isCircle) {
                for (const auto& pt : shape.points) avgR += std::hypot(pt[0] - cU, pt[1] - cV);
                avgR /= shape.points.size();
                for (const auto& pt : shape.points) {
                    if (std::abs(std::hypot(pt[0] - cU, pt[1] - cV) - avgR) > 0.06f * avgR) { isCircle = false; break; }
                }
            }

            if (isCircle) {
                // Circle Radius CAD callout
                ImVec2 scrCenter = ToScreen(cU, cV);
                float angleRad = 0.785398f; // 45 degrees
                ImVec2 scrRim = ToScreen(cU + avgR * std::cos(angleRad), cV + avgR * std::sin(angleRad));
                drawList->AddLine(scrCenter, scrRim, IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddCircleFilled(scrCenter, 3.5f, IM_COL32(255, 215, 0, 255));
                drawList->AddCircleFilled(scrRim, 3.5f, IM_COL32(255, 215, 0, 255));
                char rBuf[64];
                snprintf(rBuf, sizeof(rBuf), "R %.2f m (Ø %.2f m)", avgR, avgR * 2.0f);
                drawList->AddText(ImVec2(scrRim.x + 8.0f, scrRim.y - 14.0f), IM_COL32(255, 230, 80, 255), rBuf);
            } else if (maxU > minU + 0.01f && maxV > minV + 0.01f) {
                // Horizontal Width Dimension Line (below shape)
                float offsetV = (minV - 0.35f);
                ImVec2 extL0 = ToScreen(minU, minV);
                ImVec2 extL1 = ToScreen(minU, offsetV - 0.1f);
                ImVec2 extR0 = ToScreen(maxU, minV);
                ImVec2 extR1 = ToScreen(maxU, offsetV - 0.1f);
                drawList->AddLine(extL0, extL1, IM_COL32(180, 200, 220, 160), 1.0f);
                drawList->AddLine(extR0, extR1, IM_COL32(180, 200, 220, 160), 1.0f);

                ImVec2 dimW0 = ToScreen(minU, offsetV);
                ImVec2 dimW1 = ToScreen(maxU, offsetV);
                drawList->AddLine(dimW0, dimW1, IM_COL32(255, 215, 0, 220), 1.5f);
                // Arrow heads on horizontal dim line
                drawList->AddLine(dimW0, ImVec2(dimW0.x + 6.0f, dimW0.y - 4.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddLine(dimW0, ImVec2(dimW0.x + 6.0f, dimW0.y + 4.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddLine(dimW1, ImVec2(dimW1.x - 6.0f, dimW1.y - 4.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddLine(dimW1, ImVec2(dimW1.x - 6.0f, dimW1.y + 4.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                char wBuf[64];
                snprintf(wBuf, sizeof(wBuf), "W: %.2f m", maxU - minU);
                ImVec2 wTextSz = ImGui::CalcTextSize(wBuf);
                drawList->AddRectFilled(ImVec2((dimW0.x + dimW1.x)*0.5f - wTextSz.x*0.5f - 4.0f, dimW0.y - wTextSz.y*0.5f - 2.0f),
                                        ImVec2((dimW0.x + dimW1.x)*0.5f + wTextSz.x*0.5f + 4.0f, dimW0.y + wTextSz.y*0.5f + 2.0f),
                                        IM_COL32(20, 24, 30, 220), 3.0f);
                drawList->AddText(ImVec2((dimW0.x + dimW1.x)*0.5f - wTextSz.x*0.5f, dimW0.y - wTextSz.y*0.5f), IM_COL32(255, 230, 80, 255), wBuf);

                // Vertical Height Dimension Line (to right of shape)
                float offsetU = (maxU + 0.35f);
                ImVec2 extB0 = ToScreen(maxU, minV);
                ImVec2 extB1 = ToScreen(offsetU + 0.1f, minV);
                ImVec2 extT0 = ToScreen(maxU, maxV);
                ImVec2 extT1 = ToScreen(offsetU + 0.1f, maxV);
                drawList->AddLine(extB0, extB1, IM_COL32(180, 200, 220, 160), 1.0f);
                drawList->AddLine(extT0, extT1, IM_COL32(180, 200, 220, 160), 1.0f);

                ImVec2 dimH0 = ToScreen(offsetU, minV);
                ImVec2 dimH1 = ToScreen(offsetU, maxV);
                drawList->AddLine(dimH0, dimH1, IM_COL32(255, 215, 0, 220), 1.5f);
                // Arrow heads on vertical dim line
                drawList->AddLine(dimH0, ImVec2(dimH0.x - 4.0f, dimH0.y - 6.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddLine(dimH0, ImVec2(dimH0.x + 4.0f, dimH0.y - 6.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddLine(dimH1, ImVec2(dimH1.x - 4.0f, dimH1.y + 6.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                drawList->AddLine(dimH1, ImVec2(dimH1.x + 4.0f, dimH1.y + 6.0f), IM_COL32(255, 215, 0, 220), 1.5f);
                char hBuf[64];
                snprintf(hBuf, sizeof(hBuf), "H: %.2f m", maxV - minV);
                ImVec2 hTextSz = ImGui::CalcTextSize(hBuf);
                drawList->AddRectFilled(ImVec2(dimH0.x - hTextSz.x*0.5f - 4.0f, (dimH0.y + dimH1.y)*0.5f - hTextSz.y*0.5f - 2.0f),
                                        ImVec2(dimH0.x + hTextSz.x*0.5f + 4.0f, (dimH0.y + dimH1.y)*0.5f + hTextSz.y*0.5f + 2.0f),
                                        IM_COL32(20, 24, 30, 220), 3.0f);
                drawList->AddText(ImVec2(dimH0.x - hTextSz.x*0.5f, (dimH0.y + dimH1.y)*0.5f - hTextSz.y*0.5f), IM_COL32(255, 230, 80, 255), hBuf);
            }

        }

        // ---------------------------------------------------------------------
        // Edge Dimension Annotations & Constraint Badges
        // ---------------------------------------------------------------------
        size_t numEdges = shape.isClosed ? shape.points.size() : (shape.points.size() > 1 ? shape.points.size() - 1 : 0);
        for (size_t e = 0; e < numEdges; ++e) {
            size_t next = (e + 1) % shape.points.size();
            bool isConstrained = shape.IsEdgeConstrained(e);
            bool isEdgeSelected = (isShapeSelected && m_selectedEdgeSegmentIdx == static_cast<int>(e));
            bool isEdgeHovered = (m_hoveredShapeIdx == static_cast<int>(s) && m_hoveredEdgeIdx == static_cast<int>(e));
            // Show badge if constrained, selected, hovered, or if shape is selected and small polygon (<= 8 vertices)
            bool shouldDrawBadge = isConstrained || isEdgeSelected || isEdgeHovered || (isShapeSelected && shape.points.size() <= 8);

            if (!shouldDrawBadge) continue;

            ImVec2 p0Scr = screenPts[e];
            ImVec2 p1Scr = screenPts[next];
            float eLen = isConstrained ? shape.GetEdgeConstrainedLength(e)
                                       : std::hypot(shape.points[next][0] - shape.points[e][0],
                                                    shape.points[next][1] - shape.points[e][1]);

            // Draw line highlight
            if (isEdgeSelected) {
                drawList->AddLine(p0Scr, p1Scr, IM_COL32(255, 230, 0, 255), 4.5f);
            } else if (isConstrained) {
                drawList->AddLine(p0Scr, p1Scr, IM_COL32(255, 185, 30, 220), 2.5f);
            } else if (isEdgeHovered) {
                drawList->AddLine(p0Scr, p1Scr, IM_COL32(100, 210, 255, 200), 2.5f);
            }

            // Normal offset for midpoint badge
            float dx = p1Scr.x - p0Scr.x;
            float dy = p1Scr.y - p0Scr.y;
            float segLen = std::hypot(dx, dy);
            float offX = 0.0f, offY = 0.0f;
            if (segLen > 1e-3f) {
                offX = (-dy / segLen) * 14.0f;
                offY = (dx / segLen) * 14.0f;
            }
            ImVec2 midScr = ImVec2((p0Scr.x + p1Scr.x) * 0.5f + offX, (p0Scr.y + p1Scr.y) * 0.5f + offY);

            char badgeBuf[64];
            if (isConstrained) {
                snprintf(badgeBuf, sizeof(badgeBuf), "🔒 %.2f m", eLen);
            } else if (isEdgeSelected) {
                snprintf(badgeBuf, sizeof(badgeBuf), "Edge #%zu: %.2f m", e, eLen);
            } else {
                snprintf(badgeBuf, sizeof(badgeBuf), "%.2f m", eLen);
            }

            ImVec2 bSz = ImGui::CalcTextSize(badgeBuf);
            ImVec2 bMin = ImVec2(midScr.x - bSz.x * 0.5f - 4.0f, midScr.y - bSz.y * 0.5f - 2.5f);
            ImVec2 bMax = ImVec2(midScr.x + bSz.x * 0.5f + 4.0f, midScr.y + bSz.y * 0.5f + 2.5f);

            ImU32 bgCol = isConstrained ? IM_COL32(38, 30, 10, 235) :
                          isEdgeSelected ? IM_COL32(25, 30, 42, 240) :
                          isEdgeHovered ? IM_COL32(20, 28, 38, 230) :
                                          IM_COL32(18, 22, 28, 200);

            ImU32 borderCol = isConstrained ? IM_COL32(255, 185, 30, 255) :
                              isEdgeSelected ? IM_COL32(255, 230, 0, 255) :
                              isEdgeHovered ? IM_COL32(100, 200, 255, 220) :
                                              IM_COL32(75, 95, 120, 180);

            ImU32 txtCol = isConstrained ? IM_COL32(255, 215, 60, 255) :
                           isEdgeSelected ? IM_COL32(255, 240, 90, 255) :
                           isEdgeHovered ? IM_COL32(130, 225, 255, 255) :
                                           IM_COL32(190, 210, 230, 220);

            drawList->AddRectFilled(bMin, bMax, bgCol, 3.0f);
            drawList->AddRect(bMin, bMax, borderCol, 3.0f, 0, 1.2f);
            drawList->AddText(ImVec2(midScr.x - bSz.x * 0.5f, midScr.y - bSz.y * 0.5f), txtCol, badgeBuf);

            // Hover tooltip on dimension badge or edge
            if (isHovered && io.MousePos.x >= bMin.x && io.MousePos.x <= bMax.x &&
                io.MousePos.y >= bMin.y && io.MousePos.y <= bMax.y) {
                if (isConstrained) {
                    ImGui::SetTooltip("🔒 Constrained: %.2f m\n[Double-click to edit length or toggle constraint]\n(Locked: won't change when scaling or dragging)", eLen);
                } else {
                    ImGui::SetTooltip("Dimension: %.2f m (Edge #%zu)\n[Double-click to edit & constrain dimension]", eLen, e);
                }
            }
        }
    }

    // Snap cursor indicator
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

    // 1. Select / Move Tool
    if (m_mouseTool == SketchMouseTool::SelectMove) {
        if (m_draggedShapeIdx < 0) {
            // Left Click: select / start dragging
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered && !m_showDimensionEditPopup) {
                if (m_hoveredVertexIdx >= 0) {
                    m_selectedShapeIdx = m_hoveredShapeIdx;
                    m_selectedVertexIdx = m_hoveredVertexIdx;
                    m_selectedEdgeSegmentIdx = -1;
                    m_draggedShapeIdx = m_hoveredShapeIdx;
                    m_draggedVertexIdx = m_hoveredVertexIdx;
                    m_dragVertexOrigU = m_canvasShapes[m_hoveredShapeIdx].points[m_hoveredVertexIdx][0];
                    m_dragVertexOrigV = m_canvasShapes[m_hoveredShapeIdx].points[m_hoveredVertexIdx][1];
                } else if (m_hoveredEdgeIdx >= 0) {
                    m_selectedShapeIdx = m_hoveredShapeIdx;
                    m_selectedVertexIdx = -1;
                    m_selectedEdgeSegmentIdx = m_hoveredEdgeIdx;
                    m_draggedShapeIdx = m_hoveredShapeIdx;
                    m_draggedVertexIdx = -1;
                    m_dragShapeOrigPoints = m_canvasShapes[m_hoveredShapeIdx].points;
                    m_dragShapeStartU = rawU;
                    m_dragShapeStartV = rawV;
                } else if (m_hoveredShapeIdx >= 0) {
                    m_selectedShapeIdx = m_hoveredShapeIdx;
                    m_selectedVertexIdx = -1;
                    m_selectedEdgeSegmentIdx = -1;
                    m_draggedShapeIdx = m_hoveredShapeIdx;
                    m_draggedVertexIdx = -1;
                    m_dragShapeOrigPoints = m_canvasShapes[m_hoveredShapeIdx].points;
                    m_dragShapeStartU = rawU;
                    m_dragShapeStartV = rawV;
                } else {
                    m_selectedShapeIdx = -1;
                    m_selectedVertexIdx = -1;
                    m_selectedEdgeSegmentIdx = -1;
                }
            }

            // Right Click in SelectMove mode: instant delete of hovered vertex or hovered shape
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && isHovered) {
                if (m_hoveredShapeIdx >= 0 && m_hoveredShapeIdx < static_cast<int>(m_canvasShapes.size())) {
                    auto& shape = m_canvasShapes[m_hoveredShapeIdx];
                    if (m_hoveredVertexIdx >= 0 && m_hoveredVertexIdx < static_cast<int>(shape.points.size())) {
                        shape.points.erase(shape.points.begin() + m_hoveredVertexIdx);
                        if (m_selectedShapeIdx == m_hoveredShapeIdx) {
                            m_selectedVertexIdx = -1;
                            m_selectedEdgeSegmentIdx = -1;
                        }
                        if (shape.points.size() < 3) shape.isClosed = false;
                        if (shape.points.empty()) {
                            m_canvasShapes.erase(m_canvasShapes.begin() + m_hoveredShapeIdx);
                            m_selectedShapeIdx = -1;
                        }
                        m_hoveredShapeIdx = -1;
                        m_hoveredVertexIdx = -1;
                        m_hoveredEdgeIdx = -1;
                    } else {
                        m_canvasShapes.erase(m_canvasShapes.begin() + m_hoveredShapeIdx);
                        m_selectedShapeIdx = -1;
                        m_selectedVertexIdx = -1;
                        m_selectedEdgeSegmentIdx = -1;
                        m_hoveredShapeIdx = -1;
                        m_hoveredEdgeIdx = -1;
                    }
                }
            }
        } else {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && m_draggedShapeIdx < static_cast<int>(m_canvasShapes.size())) {
                bool isDragging = ImGui::IsMouseDragging(ImGuiMouseButton_Left, 5.0f);
                auto& dragShape = m_canvasShapes[m_draggedShapeIdx];
                if (isDragging) {
                    if (m_draggedVertexIdx >= 0) {
                        if (m_draggedVertexIdx < static_cast<int>(dragShape.points.size())) {
                            dragShape.points[m_draggedVertexIdx] = { snapU, snapV };
                            EnforceShapeConstraints(dragShape, m_draggedVertexIdx);
                        }
                    } else if (dragShape.points.size() == m_dragShapeOrigPoints.size()) {
                        // Whole-shape drag: translate the original outline by the cursor offset. The offset
                        // snaps to the grid, so a grid-aligned shape stays grid-aligned.
                        float du = rawU - m_dragShapeStartU;
                        float dv = rawV - m_dragShapeStartV;
                        if (m_gridSnapEnable && m_gridSnapSize > 0.0f) {
                            du = std::round(du / m_gridSnapSize) * m_gridSnapSize;
                            dv = std::round(dv / m_gridSnapSize) * m_gridSnapSize;
                        }
                        for (size_t i = 0; i < dragShape.points.size(); ++i) {
                            dragShape.points[i] = { m_dragShapeOrigPoints[i][0] + du, m_dragShapeOrigPoints[i][1] + dv };
                        }
                        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                        if (du != 0.0f || dv != 0.0f) {
                            char buf[64];
                            snprintf(buf, sizeof(buf), "Move (%+.2f, %+.2f) m", du, dv);
                            drawList->AddText(ImVec2(io.MousePos.x + 14.0f, io.MousePos.y + 10.0f), IM_COL32(255, 230, 120, 255), buf);
                        }
                    }
                }
            } else {
                m_draggedShapeIdx = -1;
                m_draggedVertexIdx = -1;
                m_dragShapeOrigPoints.clear();
            }
        }

        // Hint that a hovered shape body can be dragged as a whole
        if (m_draggedShapeIdx < 0 && isHovered && m_hoveredShapeIdx >= 0 && m_hoveredVertexIdx < 0) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
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
        case SketchMouseTool::SelectMove: toolTipText = "[Select/Move] Click vertex/shape to select. Drag a vertex or a whole shape to move. Del/RMB: Delete"; break;
        case SketchMouseTool::Rectangle:  toolTipText = "[Rectangle] Click & drag diagonally to draw rectangle (R). Esc: Select"; break;
        case SketchMouseTool::Circle:     toolTipText = "[Circle] Click center & drag outward to draw circle (C). Esc: Select"; break;
        case SketchMouseTool::Line:       toolTipText = "[Line/Poly] Click vertices. Click first / double-click to close (RMB: Cancel, Esc: Select)"; break;
        case SketchMouseTool::Spline:     toolTipText = "[Spline] Click control points. Click first / double-click to close (RMB: Cancel, Esc: Select)"; break;
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
