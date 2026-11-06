#include "CADGui.hpp"
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

void CADGui::RenderUI(
    ParametricGraph& graph,
    RHI::RHIDevice* device,
    AtomicMeshExchange& exchange,
    Camera& camera,
    ViewportRenderer& renderer
) {
    if (!m_initialized) return;

    // -------------------------------------------------------------------------
    // PANEL 1: CAD Tool Palette - Shape Creation & 2D Sketching
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360.0f, 430.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("CAD Tool Palette", nullptr)) {
        // Collect all SketchNodes in the DAG
        std::vector<std::shared_ptr<SketchNode>> sketchNodes;
        for (const auto& n : graph.GetNodes()) {
            if (auto s = std::dynamic_pointer_cast<SketchNode>(n)) {
                sketchNodes.push_back(s);
            }
        }

        if (ImGui::BeginTabBar("PaletteTabBar")) {
            // =================================================================
            // TAB 1: 3D Features & Solids
            // =================================================================
            if (ImGui::BeginTabItem("3D Features")) {
                const char* shapeTypes[] = { "Extrude Feature (from Sketch)", "Revolve Feature (from Sketch)", "Box Feature (from Sketch)", "Parametric Cylinder", "Parametric Sphere" };
                ImGui::Combo("Feature Type", &m_selectedShapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
                ImGui::Spacing();

                if (m_selectedShapeType == 0) {
                    // Extrude Feature (CAD workflow: extrude from reference sketch)
                    ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Extrude from Reference Sketch");
                    ImGui::Separator();

                    if (!sketchNodes.empty()) {
                        std::vector<std::string> sketchNames;
                        for (const auto& s : sketchNodes) {
                            sketchNames.push_back("[" + std::to_string(s->GetId()) + "] " + s->GetName() + 
                                                  " (" + std::to_string(s->GetProfile().size()) + " pts)");
                        }
                        if (m_selectedSketchIndex >= static_cast<int>(sketchNames.size())) {
                            m_selectedSketchIndex = 0;
                        }

                        if (ImGui::BeginCombo("Reference Sketch", sketchNames[m_selectedSketchIndex].c_str())) {
                            for (int i = 0; i < static_cast<int>(sketchNames.size()); ++i) {
                                bool isSelected = (m_selectedSketchIndex == i);
                                if (ImGui::Selectable(sketchNames[i].c_str(), isSelected)) {
                                    m_selectedSketchIndex = i;
                                }
                                if (isSelected) {
                                    ImGui::SetItemDefaultFocus();
                                }
                            }
                            ImGui::EndCombo();
                        }

                        ImGui::SliderFloat("Extrude Depth", &m_newExtrudeHeight, 0.2f, 15.0f, "%.2f m");

                        ImGui::Spacing();
                        if (ImGui::Button("+ Extrude from Selected Sketch", ImVec2(-1, 32))) {
                            auto refSketch = sketchNodes[m_selectedSketchIndex];
                            std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                            auto node = std::make_shared<ExtrudeNode>(name, refSketch, m_newExtrudeHeight);
                            graph.AddNode(node);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_selectedNodeId = node->GetId();
                        }
                    } else {
                        ImGui::TextDisabled("No 2D Sketches exist in the Model Tree yet.");
                        ImGui::Spacing();
                        if (ImGui::Button("+ Create 2D Sketch to Extrude", ImVec2(-1, 30))) {
                            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XZ_Ground);
                            sketch->SetPresetLBracket(3.5f, 3.5f, 1.0f);
                            graph.AddNode(sketch);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_selectedNodeId = sketch->GetId();
                            m_activeSketchId = sketch->GetId();
                            m_sketchModeActive = true;
                        }
                    }
                } else if (m_selectedShapeType == 1) {
                    // Revolve Feature (CAD workflow: rotational sweep around axis)
                    ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Revolve from Reference Sketch");
                    ImGui::Separator();

                    if (!sketchNodes.empty()) {
                        std::vector<std::string> sketchNames;
                        for (const auto& s : sketchNodes) {
                            sketchNames.push_back("[" + std::to_string(s->GetId()) + "] " + s->GetName() + 
                                                  " (" + std::to_string(s->GetProfile().size()) + " pts)");
                        }
                        if (m_selectedSketchIndex >= static_cast<int>(sketchNames.size())) {
                            m_selectedSketchIndex = 0;
                        }

                        if (ImGui::BeginCombo("Reference Sketch", sketchNames[m_selectedSketchIndex].c_str())) {
                            for (int i = 0; i < static_cast<int>(sketchNames.size()); ++i) {
                                bool isSelected = (m_selectedSketchIndex == i);
                                if (ImGui::Selectable(sketchNames[i].c_str(), isSelected)) {
                                    m_selectedSketchIndex = i;
                                }
                                if (isSelected) {
                                    ImGui::SetItemDefaultFocus();
                                }
                            }
                            ImGui::EndCombo();
                        }

                        ImGui::SliderFloat("Revolve Angle", &m_newRevolveAngle, 10.0f, 360.0f, "%.1f deg");
                        ImGui::SliderInt("Angular Slices", &m_newRevolveSegments, 8, 64);

                        const char* axes[] = { "X-Axis (Horizontal)", "Y-Axis (Up)", "Z-Axis (Depth)" };
                        ImGui::Combo("Revolution Axis", &m_newRevolveAxis, axes, IM_ARRAYSIZE(axes));
                        ImGui::Checkbox("Solid Manifold Body (Caps)", &m_newRevolveSolid);

                        ImGui::Spacing();
                        if (ImGui::Button("+ Revolve from Selected Sketch", ImVec2(-1, 32))) {
                            auto refSketch = sketchNodes[m_selectedSketchIndex];
                            std::string name = "Revolve_" + std::to_string(m_shapeCounter++);
                            Geom::Axis3D revAxis = (m_newRevolveAxis == 0) ? Geom::Axis3D::X() :
                                                   (m_newRevolveAxis == 2) ? Geom::Axis3D::Z() :
                                                                             Geom::Axis3D::Y();

                            auto node = std::make_shared<RevolveNode>(name, refSketch, m_newRevolveAngle, revAxis);
                            node->SetAngularSegments(m_newRevolveSegments);
                            node->SetSolid(m_newRevolveSolid);

                            graph.AddNode(node);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_selectedNodeId = node->GetId();
                        }
                    } else {
                        ImGui::TextDisabled("No 2D Sketches exist in the Model Tree yet.");
                        ImGui::Spacing();
                        if (ImGui::Button("+ Create 2D Sketch to Revolve", ImVec2(-1, 30))) {
                            std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                            auto sketch = std::make_shared<SketchNode>(name, SketchPlane::XY_Front);
                            sketch->SetPresetRectangle(2.5f, 2.5f);
                            sketch->SetPosition(4.0f, 0.0f, 0.0f);
                            graph.AddNode(sketch);
                            graph.EvaluateDirtyNodes(device, exchange);
                            m_selectedNodeId = sketch->GetId();
                            m_activeSketchId = sketch->GetId();
                            m_sketchModeActive = true;
                        }
                    }
                } else if (m_selectedShapeType == 2) {
                    // Parametric Box (Requires creating and referencing a 2D rectangular sketch)
                    ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Parametric Box (from 2D Reference Sketch)");
                    ImGui::Separator();

                    ImGui::SliderFloat("Width (X)",  &m_newBoxDims[0], 0.2f, 15.0f, "%.2f m");
                    ImGui::SliderFloat("Depth (Z)",  &m_newBoxDims[2], 0.2f, 15.0f, "%.2f m");
                    ImGui::SliderFloat("Height (Y)", &m_newBoxDims[1], 0.2f, 15.0f, "%.2f m");

                    const char* planes[] = { "XZ Ground Plane (Top)", "XY Front Plane", "YZ Right Plane" };
                    ImGui::Combo("Base Datum Plane", &m_newSketchPlane, planes, IM_ARRAYSIZE(planes));

                    ImGui::Spacing();
                    if (ImGui::Button("+ Create Box (Rectangle Sketch + Extrude)", ImVec2(-1, 32))) {
                        uint32_t counter = m_shapeCounter++;
                        std::string sketchName = "Sketch_Box_" + std::to_string(counter);
                        std::string extrudeName = "Box_" + std::to_string(counter);

                        auto sketch = std::make_shared<SketchNode>(sketchName, static_cast<SketchPlane>(m_newSketchPlane));
                        sketch->SetPresetRectangle(m_newBoxDims[0], m_newBoxDims[2]);

                        float posX = static_cast<float>((counter % 4) - 1.5f) * 4.0f;
                        float posZ = static_cast<float>(counter / 4) * 4.0f;
                        sketch->SetPosition(posX, 0.0f, posZ);
                        graph.AddNode(sketch);

                        auto extrude = std::make_shared<ExtrudeNode>(extrudeName, sketch, m_newBoxDims[1]);
                        extrude->SetPosition(posX, 0.0f, posZ);
                        graph.AddNode(extrude);

                        graph.EvaluateDirtyNodes(device, exchange);
                        m_selectedNodeId = extrude->GetId();
                    }
                } else if (m_selectedShapeType == 3) {
                    // Cylinder parameters
                    ImGui::SliderFloat("Radius", &m_newCylinderRadius, 0.1f, 10.0f, "%.2f m");
                    ImGui::SliderFloat("Height", &m_newCylinderHeight, 0.2f, 15.0f, "%.2f m");
                    ImGui::SliderInt("Segments", &m_newCylinderSegments, 8, 96);

                    ImGui::Spacing();
                    if (ImGui::Button("+ Create Parametric Cylinder", ImVec2(-1, 32))) {
                        std::string name = "Cylinder_" + std::to_string(m_shapeCounter++);
                        auto node = std::make_shared<CylinderNode>(name, m_newCylinderRadius, m_newCylinderHeight, m_newCylinderSegments);
                        float posX = static_cast<float>((m_shapeCounter % 4) - 1.5f) * 4.0f;
                        float posZ = static_cast<float>(m_shapeCounter / 4) * 4.0f;
                        node->SetPosition(posX, 0.0f, posZ);
                        graph.AddNode(node);
                        graph.EvaluateDirtyNodes(device, exchange);
                        m_selectedNodeId = node->GetId();
                    }
                } else if (m_selectedShapeType == 4) {
                    // Sphere parameters
                    ImGui::SliderFloat("Radius", &m_newSphereRadius, 0.2f, 10.0f, "%.2f m");
                    ImGui::SliderInt("Rings",   &m_newSphereRings, 6, 48);
                    ImGui::SliderInt("Sectors", &m_newSphereSectors, 8, 64);

                    ImGui::Spacing();
                    if (ImGui::Button("+ Create Parametric Sphere", ImVec2(-1, 32))) {
                        std::string name = "Sphere_" + std::to_string(m_shapeCounter++);
                        auto node = std::make_shared<SphereNode>(name, m_newSphereRadius, m_newSphereRings, m_newSphereSectors);
                        float posX = static_cast<float>((m_shapeCounter % 4) - 1.5f) * 4.0f;
                        float posZ = static_cast<float>(m_shapeCounter / 4) * 4.0f;
                        node->SetPosition(posX, 0.0f, posZ);
                        graph.AddNode(node);
                        graph.EvaluateDirtyNodes(device, exchange);
                        m_selectedNodeId = node->GetId();
                    }
                }

                ImGui::EndTabItem();
            }

            // =================================================================
            // TAB 2: 2D Sketch Mode
            // =================================================================
            if (ImGui::BeginTabItem("2D Sketch Mode")) {
                ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Datum Planes & Profile Presets");
                ImGui::Separator();

                const char* planes[] = { "XZ Ground Plane (Top View)", "XY Front Plane", "YZ Right Plane" };
                ImGui::Combo("Datum Plane", &m_newSketchPlane, planes, IM_ARRAYSIZE(planes));

                const char* presets[] = { "Rectangle", "Regular Polygon", "L-Bracket", "5-Point Star", "T-Beam" };
                ImGui::Combo("Initial Profile", &m_newSketchPreset, presets, IM_ARRAYSIZE(presets));

                if (m_newSketchPreset == 0) {
                    ImGui::SliderFloat("Width (U)",  &m_sketchRectDims[0], 0.5f, 15.0f, "%.2f m");
                    ImGui::SliderFloat("Height (V)", &m_sketchRectDims[1], 0.5f, 15.0f, "%.2f m");
                } else if (m_newSketchPreset == 1) {
                    ImGui::SliderInt("Sides", &m_sketchPolySides, 3, 24);
                    ImGui::SliderFloat("Radius", &m_sketchPolyRadius, 0.5f, 10.0f, "%.2f m");
                } else if (m_newSketchPreset == 2) {
                    ImGui::SliderFloat("Width (U)", &m_sketchLBracket[0], 1.0f, 12.0f, "%.2f m");
                    ImGui::SliderFloat("Height (V)", &m_sketchLBracket[1], 1.0f, 12.0f, "%.2f m");
                    ImGui::SliderFloat("Flange Thickness", &m_sketchLBracket[2], 0.2f, 4.0f, "%.2f m");
                } else if (m_newSketchPreset == 3) {
                    ImGui::SliderInt("Points", &m_sketchStarPoints, 3, 12);
                    ImGui::SliderFloat("Outer Radius", &m_sketchStarRadii[0], 0.5f, 10.0f, "%.2f m");
                    ImGui::SliderFloat("Inner Radius", &m_sketchStarRadii[1], 0.2f, 8.0f, "%.2f m");
                } else if (m_newSketchPreset == 4) {
                    ImGui::SliderFloat("Flange Width", &m_sketchTBeam[0], 1.0f, 12.0f, "%.2f m");
                    ImGui::SliderFloat("Web Height",   &m_sketchTBeam[1], 1.0f, 12.0f, "%.2f m");
                    ImGui::SliderFloat("Thickness",    &m_sketchTBeam[2], 0.2f, 3.0f, "%.2f m");
                }

                ImGui::Spacing();
                if (ImGui::Button("+ Create 2D Sketch in Model Tree", ImVec2(-1, 32))) {
                    std::string name = "Sketch_" + std::to_string(m_shapeCounter++);
                    auto sketch = std::make_shared<SketchNode>(name, static_cast<SketchPlane>(m_newSketchPlane));
                    
                    if (m_newSketchPreset == 0)      sketch->SetPresetRectangle(m_sketchRectDims[0], m_sketchRectDims[1]);
                    else if (m_newSketchPreset == 1) sketch->SetPresetRegularPolygon(m_sketchPolySides, m_sketchPolyRadius);
                    else if (m_newSketchPreset == 2) sketch->SetPresetLBracket(m_sketchLBracket[0], m_sketchLBracket[1], m_sketchLBracket[2]);
                    else if (m_newSketchPreset == 3) sketch->SetPresetStar(m_sketchStarPoints, m_sketchStarRadii[0], m_sketchStarRadii[1]);
                    else if (m_newSketchPreset == 4) sketch->SetPresetTBeam(m_sketchTBeam[0], m_sketchTBeam[1], m_sketchTBeam[2]);

                    float posX = static_cast<float>((m_shapeCounter % 4) - 1.5f) * 4.0f;
                    float posZ = static_cast<float>(m_shapeCounter / 4) * 4.0f;
                    sketch->SetPosition(posX, 0.0f, posZ);

                    graph.AddNode(sketch);
                    graph.EvaluateDirtyNodes(device, exchange);
                    m_selectedNodeId = sketch->GetId();
                    m_activeSketchId = sketch->GetId();
                    m_sketchModeActive = true;
                }

                if (!sketchNodes.empty()) {
                    ImGui::Spacing();
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Interactive Canvas:");
                    if (ImGui::Button("✏ Open 2D Mouse Sketch Canvas", ImVec2(-1, 32))) {
                        m_sketchModeActive = true;
                        if (m_activeSketchId == 0) {
                            m_activeSketchId = sketchNodes[0]->GetId();
                        }
                    }
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();

    // -------------------------------------------------------------------------
    // PANEL 2: Model Tree & Interactive Property Inspector
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(16.0f, 455.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360.0f, 410.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Model Tree & Property Inspector", nullptr)) {
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "CAD Model Tree (%zu Objects)", graph.GetNodeCount());
        ImGui::Separator();

        auto nodes = graph.GetNodes();
        std::shared_ptr<DAGNode> selectedNode = nullptr;

        ImGui::BeginChild("NodeList", ImVec2(0, 130), true);
        for (const auto& node : nodes) {
            bool isSelected = (node->GetId() == m_selectedNodeId);

            std::string prefix = "[Solid] ";
            if (auto s = std::dynamic_pointer_cast<SketchNode>(node)) {
                prefix = "[Sketch] ";
            } else if (auto e = std::dynamic_pointer_cast<ExtrudeNode>(node)) {
                prefix = e->GetSketchReference() ? "[Extrude (Ref)] " : "[Extrude] ";
            } else if (auto r = std::dynamic_pointer_cast<RevolveNode>(node)) {
                prefix = r->GetSketchReference() ? "[Revolve (Ref)] " : "[Revolve] ";
            } else if (std::dynamic_pointer_cast<DatumPlaneNode>(node)) {
                prefix = "[DatumPlane] ";
            }

            std::string label = prefix + node->GetName() + " (ID " + std::to_string(node->GetId()) + ")";
            if (ImGui::Selectable(label.c_str(), isSelected)) {
                m_selectedNodeId = node->GetId();
            }
            if (isSelected) {
                selectedNode = node;
                if (m_selectionScrollPending) {
                    ImGui::SetScrollHereY(0.5f);
                }
            }
        }
        m_selectionScrollPending = false;
        ImGui::EndChild();

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Live Feature / Sketch Inspector:");
        ImGui::Separator();

        if (selectedNode) {
            ImGui::Text("Selected: %s (Gen %llu)", selectedNode->GetName().c_str(), selectedNode->GetGeneration());

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
                        m_sketchModeActive = false;
                    }
                }
                ImGui::Spacing();

                int currentPlane = static_cast<int>(sketch->GetPlane());
                const char* planes[] = { "XZ Ground Plane (Top)", "XY Front Plane", "YZ Right Plane" };
                if (ImGui::Combo("Datum Plane", &currentPlane, planes, IM_ARRAYSIZE(planes))) {
                    sketch->SetPlane(static_cast<SketchPlane>(currentPlane));
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                bool visible = sketch->IsVisible();
                if (ImGui::Checkbox("Show 2D Sketch Wireframe in 3D", &visible)) {
                    sketch->SetVisible(visible);
                    graph.EvaluateDirtyNodes(device, exchange);
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

                // Interactive 2D Points Editor
                ImGui::Spacing();
                const auto& profile = sketch->GetProfile();
                ImGui::Text("Sketch Vertices (%zu points):", profile.size());

                ImGui::BeginChild("PointEditor", ImVec2(0, 95), true);
                for (size_t i = 0; i < profile.size(); ++i) {
                    float pt[2]{profile[i][0], profile[i][1]};
                    std::string label = "Pt #" + std::to_string(i);
                    if (ImGui::DragFloat2(label.c_str(), pt, 0.05f, -20.0f, 20.0f, "%.2f")) {
                        sketch->UpdatePoint(i, pt[0], pt[1]);
                        graph.EvaluateDirtyNodes(device, exchange);
                    }
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
                if (ImGui::SliderFloat("Extrude Depth", &h, 0.2f, 15.0f, "%.2f m")) {
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

                float ang = revolve->GetAngleDegrees();
                if (ImGui::SliderFloat("Revolve Angle", &ang, 10.0f, 360.0f, "%.1f deg")) {
                    revolve->SetAngleDegrees(ang);
                    graph.EvaluateDirtyNodes(device, exchange);
                }

                int segs = revolve->GetAngularSegments();
                if (ImGui::SliderInt("Angular Slices", &segs, 8, 64)) {
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
            // C. Primitive Nodes (Box, Cylinder, Sphere)
            // -----------------------------------------------------------------
            else if (auto box = std::dynamic_pointer_cast<BoxNode>(selectedNode)) {
                float w = box->GetWidth();
                float h = box->GetHeight();
                float d = box->GetDepth();
                bool changed = false;

                changed |= ImGui::SliderFloat("Width",  &w, 0.2f, 15.0f, "%.2f m");
                changed |= ImGui::SliderFloat("Height", &h, 0.2f, 15.0f, "%.2f m");
                changed |= ImGui::SliderFloat("Depth",  &d, 0.2f, 15.0f, "%.2f m");

                if (changed) {
                    box->SetDimensions(w, h, d);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            } else if (auto cyl = std::dynamic_pointer_cast<CylinderNode>(selectedNode)) {
                float r = cyl->GetRadius();
                float h = cyl->GetHeight();
                int segs = static_cast<int>(cyl->GetSegments());
                bool changed = false;

                changed |= ImGui::SliderFloat("Radius",   &r, 0.1f, 10.0f, "%.2f m");
                changed |= ImGui::SliderFloat("Height",   &h, 0.2f, 15.0f, "%.2f m");
                changed |= ImGui::SliderInt("Segments",   &segs, 8, 96);

                if (changed) {
                    cyl->SetParameters(r, h, segs);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            } else if (auto sphere = std::dynamic_pointer_cast<SphereNode>(selectedNode)) {
                float r = sphere->GetRadius();
                int rings = static_cast<int>(sphere->GetRings());
                int sectors = static_cast<int>(sphere->GetSectors());
                bool changed = false;

                changed |= ImGui::SliderFloat("Radius",  &r, 0.2f, 10.0f, "%.2f m");
                changed |= ImGui::SliderInt("Rings",    &rings, 6, 48);
                changed |= ImGui::SliderInt("Sectors",  &sectors, 8, 64);

                if (changed) {
                    sphere->SetParameters(r, rings, sectors);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }

            ImGui::Spacing();
            if (ImGui::Button("Delete Selected Object", ImVec2(-1, 26))) {
                graph.RemoveNode(selectedNode->GetId());
                graph.PublishCombinedSceneMesh(device, exchange);
                m_selectedNodeId = 0;
            }
        } else {
            ImGui::TextDisabled("Select an object or sketch from the Model Tree above to inspect.");
        }
    }
    ImGui::End();

    // -------------------------------------------------------------------------
    // PANEL 3: CAD Viewport Controls & HUD
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 360.0f, 16.0f), ImGuiCond_FirstUseEver);
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

    ImGui::SetNextWindowPos(ImVec2(390.0f, 16.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(820.0f, 650.0f), ImGuiCond_FirstUseEver);

    std::string winTitle = "2D Sketch Canvas - " + activeSketch->GetName() + " (" + GetSketchPlaneName(activeSketch->GetPlane()) + ")###SketchCanvas";
    bool isOpen = true;
    if (!ImGui::Begin(winTitle.c_str(), &isOpen)) {
        if (!isOpen) m_sketchModeActive = false;
        ImGui::End();
        return;
    }
    if (!isOpen) {
        m_sketchModeActive = false;
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
        if (ImGui::Button(label, ImVec2(84, 28))) {
            if (m_mouseTool != tool) {
                m_mouseTool = tool;
                m_isDrawing = false;
                m_inProgressPoints.clear();
                m_inProgressSplineControls.clear();
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

    ImGui::Spacing();
    ImGui::SameLine();
    ImGui::Checkbox("Snap", &m_gridSnapEnable);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::SliderFloat("Grid", &m_gridSnapSize, 0.1f, 2.0f, "%.1f m");
    ImGui::SameLine();

    if (ImGui::Button("Center View", ImVec2(90, 28))) {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        m_canvasPanX = avail.x * 0.5f;
        m_canvasPanY = avail.y * 0.5f;
        m_canvasZoom = 35.0f;
    }
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.60f, 0.35f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.70f, 0.42f, 1.0f));
    if (ImGui::Button("✓ Finish Sketch", ImVec2(105, 28))) {
        m_sketchModeActive = false;
    }
    ImGui::PopStyleColor(2);

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
        // Keyboard tool switching shortcuts
        if (ImGui::IsKeyPressed(ImGuiKey_V)) m_mouseTool = SketchMouseTool::SelectMove;
        if (ImGui::IsKeyPressed(ImGuiKey_R)) m_mouseTool = SketchMouseTool::Rectangle;
        if (ImGui::IsKeyPressed(ImGuiKey_C)) m_mouseTool = SketchMouseTool::Circle;
        if (ImGui::IsKeyPressed(ImGuiKey_L)) m_mouseTool = SketchMouseTool::Line;
        if (ImGui::IsKeyPressed(ImGuiKey_S)) m_mouseTool = SketchMouseTool::Spline;

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

    // Existing profile
    const auto& profile = activeSketch->GetProfile();
    if (profile.size() >= 3) {
        std::vector<ImVec2> screenPts;
        screenPts.reserve(profile.size());
        for (const auto& pt : profile) {
            screenPts.push_back(ToScreen(pt[0], pt[1]));
        }

        drawList->AddConvexPolyFilled(screenPts.data(), static_cast<int>(screenPts.size()), IM_COL32(20, 140, 255, 35));
        drawList->AddPolyline(screenPts.data(), static_cast<int>(screenPts.size()), IM_COL32(0, 210, 255, 255), ImDrawFlags_Closed, 2.5f);

        for (size_t i = 0; i < profile.size(); ++i) {
            ImVec2 ptScr = screenPts[i];
            bool isHoveredPt = (m_hoveredVertexIdx == static_cast<int>(i));
            bool isDraggedPt = (m_draggedVertexIdx == static_cast<int>(i));

            float radius = (isHoveredPt || isDraggedPt) ? 7.0f : 4.5f;
            ImU32 color = isDraggedPt ? IM_COL32(255, 80, 80, 255) : 
                          isHoveredPt ? IM_COL32(255, 215, 0, 255) : 
                                        IM_COL32(255, 255, 255, 230);

            drawList->AddCircleFilled(ptScr, radius, color);
            drawList->AddCircle(ptScr, radius + 1.0f, IM_COL32(10, 10, 10, 200), 12, 1.5f);

            if (isHoveredPt || isDraggedPt) {
                char buf[64];
                snprintf(buf, sizeof(buf), "#%zu (%.2f, %.2f)", i, profile[i][0], profile[i][1]);
                drawList->AddText(ImVec2(ptScr.x + 8.0f, ptScr.y - 14.0f), IM_COL32(255, 255, 100, 255), buf);
            }
        }
    }

    // Cursor coordinates & snapping
    float rawU = (io.MousePos.x - canvasP0.x - m_canvasPanX) / m_canvasZoom;
    float rawV = -(io.MousePos.y - canvasP0.y - m_canvasPanY) / m_canvasZoom;
    float snapU = m_gridSnapEnable ? (std::round(rawU / m_gridSnapSize) * m_gridSnapSize) : rawU;
    float snapV = m_gridSnapEnable ? (std::round(rawV / m_gridSnapSize) * m_gridSnapSize) : rawV;
    ImVec2 snapPosScr = ToScreen(snapU, snapV);

    if (isHovered) {
        drawList->AddCircle(snapPosScr, 4.0f, IM_COL32(0, 255, 255, 180), 8, 1.2f);
    }

    // -------------------------------------------------------------------------
    // Tool Interactions
    // -------------------------------------------------------------------------
    // 1. Select / Move Tool
    if (m_mouseTool == SketchMouseTool::SelectMove) {
        if (m_draggedVertexIdx < 0) {
            m_hoveredVertexIdx = -1;
            float bestDist = 14.0f;
            for (size_t i = 0; i < profile.size(); ++i) {
                ImVec2 ptScr = ToScreen(profile[i][0], profile[i][1]);
                float dist = std::hypot(io.MousePos.x - ptScr.x, io.MousePos.y - ptScr.y);
                if (dist < bestDist) {
                    bestDist = dist;
                    m_hoveredVertexIdx = static_cast<int>(i);
                }
            }
            if (m_hoveredVertexIdx >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
                m_draggedVertexIdx = m_hoveredVertexIdx;
            }
        } else {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                activeSketch->UpdatePoint(m_draggedVertexIdx, snapU, snapV);
                graph.EvaluateDirtyNodes(device, exchange);
            } else {
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

            drawList->AddRectFilled(minScr, maxScr, IM_COL32(30, 144, 255, 50));
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
                    std::vector<std::array<float, 2>> rectProfile = {
                        {minU, minV},
                        {maxU, minV},
                        {maxU, maxV},
                        {minU, maxV}
                    };
                    activeSketch->SetProfile(std::move(rectProfile));
                    graph.EvaluateDirtyNodes(device, exchange);
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

            drawList->AddCircleFilled(centerScr, radiusScr, IM_COL32(30, 144, 255, 50), 48);
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
                    activeSketch->SetProfile(std::move(circleProfile));
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }
        }
    }
    // 4. Line / Polyline Tool
    else if (m_mouseTool == SketchMouseTool::Line) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            m_inProgressPoints.clear();
        }

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
            if (m_inProgressPoints.empty()) {
                m_inProgressPoints.push_back({snapU, snapV});
            } else {
                ImVec2 firstPtScr = ToScreen(m_inProgressPoints[0][0], m_inProgressPoints[0][1]);
                float distToFirst = std::hypot(io.MousePos.x - firstPtScr.x, io.MousePos.y - firstPtScr.y);

                if ((distToFirst < 14.0f || ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) && m_inProgressPoints.size() >= 3) {
                    activeSketch->SetProfile(m_inProgressPoints);
                    graph.EvaluateDirtyNodes(device, exchange);
                    m_inProgressPoints.clear();
                } else if (distToFirst >= 14.0f) {
                    auto last = m_inProgressPoints.back();
                    if (std::hypot(snapU - last[0], snapV - last[1]) > 0.01f) {
                        m_inProgressPoints.push_back({snapU, snapV});
                    }
                }
            }
        }

        if (!m_inProgressPoints.empty()) {
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
            float distToFirst = std::hypot(io.MousePos.x - firstPtScr.x, io.MousePos.y - firstPtScr.y);
            if (m_inProgressPoints.size() >= 3) {
                drawList->AddCircle(firstPtScr, 8.0f, distToFirst < 14.0f ? IM_COL32(50, 255, 50, 255) : IM_COL32(255, 180, 0, 180), 16, 2.0f);
                if (distToFirst < 14.0f) {
                    drawList->AddText(ImVec2(firstPtScr.x + 12.0f, firstPtScr.y - 12.0f), IM_COL32(50, 255, 50, 255), "Click to Close Loop");
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

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && isHovered) {
            if (m_inProgressSplineControls.empty()) {
                m_inProgressSplineControls.push_back({snapU, snapV});
            } else {
                ImVec2 firstPtScr = ToScreen(m_inProgressSplineControls[0][0], m_inProgressSplineControls[0][1]);
                float distToFirst = std::hypot(io.MousePos.x - firstPtScr.x, io.MousePos.y - firstPtScr.y);

                if ((distToFirst < 14.0f || ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) && m_inProgressSplineControls.size() >= 3) {
                    auto denseSpline = EvaluateCatmullRomSpline(m_inProgressSplineControls, 12);
                    activeSketch->SetProfile(std::move(denseSpline));
                    graph.EvaluateDirtyNodes(device, exchange);
                    m_inProgressSplineControls.clear();
                } else if (distToFirst >= 14.0f) {
                    auto last = m_inProgressSplineControls.back();
                    if (std::hypot(snapU - last[0], snapV - last[1]) > 0.01f) {
                        m_inProgressSplineControls.push_back({snapU, snapV});
                    }
                }
            }
        }

        if (!m_inProgressSplineControls.empty()) {
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
                float distToFirst = std::hypot(io.MousePos.x - firstPtScr.x, io.MousePos.y - firstPtScr.y);
                drawList->AddCircle(firstPtScr, 9.0f, distToFirst < 14.0f ? IM_COL32(50, 255, 50, 255) : IM_COL32(255, 100, 200, 200), 16, 2.0f);
                if (distToFirst < 14.0f) {
                    drawList->AddText(ImVec2(firstPtScr.x + 12.0f, firstPtScr.y - 12.0f), IM_COL32(50, 255, 50, 255), "Click to Close Spline");
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
    // Coordinate Readout HUD
    char coordBuf[128];
    snprintf(coordBuf, sizeof(coordBuf), "Cursor: (U: %+.2f m, V: %+.2f m) | Grid Snap: %.2f m", snapU, snapV, m_gridSnapSize);
    drawList->AddRectFilled(ImVec2(canvasP0.x + 8.0f, canvasP1.y - 28.0f), ImVec2(canvasP0.x + 380.0f, canvasP1.y - 6.0f), IM_COL32(15, 17, 20, 200), 4.0f);
    drawList->AddText(ImVec2(canvasP0.x + 14.0f, canvasP1.y - 25.0f), IM_COL32(0, 230, 255, 255), coordBuf);

    // Tool quick tip
    const char* toolTipText = "";
    switch (m_mouseTool) {
        case SketchMouseTool::SelectMove: toolTipText = "[Select/Move] Click & drag vertex handles to reshape (updates 3D solids live!)"; break;
        case SketchMouseTool::Rectangle:  toolTipText = "[Rectangle] Click & drag diagonally to draw rectangle"; break;
        case SketchMouseTool::Circle:     toolTipText = "[Circle] Click center & drag outward to draw circle"; break;
        case SketchMouseTool::Line:       toolTipText = "[Line/Poly] Click vertices. Click first point / double-click to close (RMB: Cancel)"; break;
        case SketchMouseTool::Spline:     toolTipText = "[Spline] Click control points. Click first / double-click to close (RMB: Cancel)"; break;
    }
    ImVec2 tipSz = ImGui::CalcTextSize(toolTipText);
    drawList->AddRectFilled(ImVec2(canvasP1.x - tipSz.x - 20.0f, canvasP0.y + 8.0f), ImVec2(canvasP1.x - 8.0f, canvasP0.y + 28.0f), IM_COL32(15, 17, 20, 190), 4.0f);
    drawList->AddText(ImVec2(canvasP1.x - tipSz.x - 14.0f, canvasP0.y + 11.0f), IM_COL32(230, 230, 230, 230), toolTipText);

    drawList->PopClipRect();
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
