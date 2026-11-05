#include "CADGui.hpp"
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_vulkan.h"
#include "engine/hal/vulkan/VulkanDevice.hpp"
#include <GLFW/glfw3.h>
#include <iostream>
#include <algorithm>

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
    // PANEL 1: CAD Tool Palette - Shape Creation
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340.0f, 380.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("CAD Tool Palette - Create Shapes", nullptr)) {
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "Parametric Primitives");
        ImGui::Separator();

        const char* shapeTypes[] = { "Box", "Cylinder", "Sphere", "Extrude 2D" };
        ImGui::Combo("Primitive Type", &m_selectedShapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
        ImGui::Spacing();

        if (m_selectedShapeType == 0) {
            // Box parameters
            ImGui::SliderFloat("Width (X)",  &m_newBoxDims[0], 0.2f, 15.0f, "%.2f m");
            ImGui::SliderFloat("Height (Y)", &m_newBoxDims[1], 0.2f, 15.0f, "%.2f m");
            ImGui::SliderFloat("Depth (Z)",  &m_newBoxDims[2], 0.2f, 15.0f, "%.2f m");

            ImGui::Spacing();
            if (ImGui::Button("+ Create Parametric Box", ImVec2(-1, 32))) {
                std::string name = "Box_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<BoxNode>(name, m_newBoxDims[0], m_newBoxDims[1], m_newBoxDims[2]);
                float posX = static_cast<float>((m_shapeCounter % 4) - 1.5f) * 4.0f;
                float posZ = static_cast<float>(m_shapeCounter / 4) * 4.0f;
                node->SetPosition(posX, 0.0f, posZ);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                m_selectedNodeId = node->GetId();
            }
        } else if (m_selectedShapeType == 1) {
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
        } else if (m_selectedShapeType == 2) {
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
        } else if (m_selectedShapeType == 3) {
            // Extrude 2D Sketch
            const char* profiles[] = { "L-Bracket Profile", "Hexagon Profile", "Star Profile", "T-Beam Profile" };
            ImGui::Combo("2D Sketch", &m_selectedExtrudeProfile, profiles, IM_ARRAYSIZE(profiles));
            ImGui::SliderFloat("Extrude Depth", &m_newExtrudeHeight, 0.2f, 15.0f, "%.2f m");

            ImGui::Spacing();
            if (ImGui::Button("+ Create Extrusion", ImVec2(-1, 32))) {
                std::vector<std::array<float, 2>> profile;
                if (m_selectedExtrudeProfile == 0) {
                    // L-Bracket
                    profile = {{-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, -1.0f}, {-1.0f, -1.0f}, {-1.0f, 2.0f}, {-2.0f, 2.0f}};
                } else if (m_selectedExtrudeProfile == 1) {
                    // Regular Hexagon
                    for (int i = 0; i < 6; ++i) {
                        float angle = i * 1.04719755f;
                        profile.push_back({2.0f * std::cos(angle), 2.0f * std::sin(angle)});
                    }
                } else if (m_selectedExtrudeProfile == 2) {
                    // 5-Point Star
                    for (int i = 0; i < 10; ++i) {
                        float r = (i % 2 == 0) ? 2.5f : 1.2f;
                        float angle = i * 0.62831853f;
                        profile.push_back({r * std::cos(angle), r * std::sin(angle)});
                    }
                } else {
                    // T-Beam
                    profile = {{-2.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 0.0f}, {0.5f, 0.0f}, {0.5f, -2.0f}, {-0.5f, -2.0f}, {-0.5f, 0.0f}, {-2.0f, 0.0f}};
                }

                std::string name = "Extrude_" + std::to_string(m_shapeCounter++);
                auto node = std::make_shared<ExtrudeNode>(name, profile, m_newExtrudeHeight);
                float posX = static_cast<float>((m_shapeCounter % 4) - 1.5f) * 4.0f;
                float posZ = static_cast<float>(m_shapeCounter / 4) * 4.0f;
                node->SetPosition(posX, 0.0f, posZ);
                graph.AddNode(node);
                graph.EvaluateDirtyNodes(device, exchange);
                m_selectedNodeId = node->GetId();
            }
        }
    }
    ImGui::End();

    // -------------------------------------------------------------------------
    // PANEL 2: Model Tree & Interactive Property Inspector
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(16.0f, 410.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340.0f, 420.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Model Tree & Property Inspector", nullptr)) {
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "DAG Scene Objects (%zu)", graph.GetNodeCount());
        ImGui::Separator();

        auto nodes = graph.GetNodes();
        std::shared_ptr<DAGNode> selectedNode = nullptr;

        ImGui::BeginChild("NodeList", ImVec2(0, 140), true);
        for (const auto& node : nodes) {
            bool isSelected = (node->GetId() == m_selectedNodeId);
            std::string label = "[" + std::to_string(node->GetId()) + "] " + node->GetName();
            if (ImGui::Selectable(label.c_str(), isSelected)) {
                m_selectedNodeId = node->GetId();
            }
            if (isSelected) {
                selectedNode = node;
            }
        }
        ImGui::EndChild();

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), "Live Shape Inspector:");
        ImGui::Separator();

        if (selectedNode) {
            ImGui::Text("Editing: %s (Gen %llu)", selectedNode->GetName().c_str(), selectedNode->GetGeneration());

            float pos[3]{selectedNode->GetPosition()[0], selectedNode->GetPosition()[1], selectedNode->GetPosition()[2]};
            if (ImGui::DragFloat3("Position (X, Y, Z)", pos, 0.1f, -50.0f, 50.0f, "%.2f m")) {
                selectedNode->SetPosition(pos[0], pos[1], pos[2]);
                graph.EvaluateDirtyNodes(device, exchange);
            }
            ImGui::Spacing();

            if (auto box = std::dynamic_pointer_cast<BoxNode>(selectedNode)) {
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
            } else if (auto extrude = std::dynamic_pointer_cast<ExtrudeNode>(selectedNode)) {
                float h = extrude->GetHeight();
                if (ImGui::SliderFloat("Extrude Depth", &h, 0.2f, 15.0f, "%.2f m")) {
                    extrude->SetHeight(h);
                    graph.EvaluateDirtyNodes(device, exchange);
                }
            }

            ImGui::Spacing();
            if (ImGui::Button("Delete Selected Shape", ImVec2(-1, 26))) {
                graph.RemoveNode(selectedNode->GetId());
                graph.PublishCombinedSceneMesh(device, exchange);
                m_selectedNodeId = 0;
            }
        } else {
            ImGui::TextDisabled("Select a shape from the list above to modify.");
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
