#include "Application.hpp"
#include <GLFW/glfw3.h>
#include "backends/imgui_impl_glfw.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <windows.h>
#endif

#include <iostream>
#include <thread>
#include <chrono>

namespace Tessera {

static void GLFWErrorCallback(int error, const char* description) {
    std::cerr << "[GLFW Error " << error << "]: " << description << "\n";
}

Application::Application() {
    std::cout << "[Application] Bootstrapping TesseraCAD Engine Subsystems...\n";

    // 1. Initialize Worker Pool (std::jthread)
    m_threadPool = std::make_shared<ThreadPool>(4);

    // 2. Initialize Parametric Graph
    m_graph = std::make_unique<ParametricGraph>(m_threadPool);

    // 3. Create initial parametric 2D Sketches & linked 3D Features (CAD B-Rep workflow)
    auto initialSketch = std::make_shared<SketchNode>("Sketch_LBracket", SketchPlane::XZ_Ground);
    initialSketch->SetPresetLBracket(4.0f, 4.0f, 1.2f);
    initialSketch->SetPosition(-3.0f, 0.0f, 0.0f);
    m_graph->AddNode(initialSketch);

    auto initialExtrude = std::make_shared<ExtrudeNode>("Extrude_LBracket", initialSketch, 3.0f);
    initialExtrude->SetPosition(-3.0f, 0.0f, 0.0f);
    m_graph->AddNode(initialExtrude);

    // Revolve feature created by revolving a 2D sketch cross-section around the axis
    // Cylindrical wall thickness is dictated by the sketch drawing (u=1.0 to 2.2 -> thickness=1.2m, height=3.5m)
    auto cylSketch = std::make_shared<SketchNode>("Sketch_CylinderProfile", SketchPlane::XY_Front);
    cylSketch->SetProfile({ {1.0f, 0.0f}, {2.2f, 0.0f}, {2.2f, 3.5f}, {1.0f, 3.5f} });
    cylSketch->SetPosition(2.5f, 0.0f, 0.0f);
    m_graph->AddNode(cylSketch);

    auto cylRevolve = std::make_shared<RevolveNode>("Revolve_Cylinder", cylSketch, 360.0f, RevolveAxisMode::Sketch_V_Axis);
    cylRevolve->SetPosition(2.5f, 0.0f, 0.0f);
    m_graph->AddNode(cylRevolve);
    m_testRevolveNode = cylRevolve;

    // 4. Initialize Desktop Window & Native Graphics HAL
    InitWindow();
}

Application::~Application() {
    Stop();
}

void Application::InitWindow() {
    glfwSetErrorCallback(GLFWErrorCallback);
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW!");
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    m_window = glfwCreateWindow(m_width, m_height, "TesseraCAD - 3D Parametric CAD Workstation", nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW desktop window!");
    }

    RHI::NativeWindowHandle windowHandle{};
#if defined(_WIN32)
    HWND hwnd = glfwGetWin32Window(m_window);
    windowHandle.windowHandle = hwnd;
    windowHandle.displayHandle = GetModuleHandle(nullptr);
#endif

    try {
        m_rhiDevice = RHI::RHIDevice::Create(windowHandle, m_width, m_height);
    } catch (const std::exception& e) {
        std::cerr << "[Application] Native GPU device initialization failed: " << e.what() << "\n";
    }

    // 5. Initialize Viewport Renderer
    m_renderer = std::make_unique<ViewportRenderer>(m_rhiDevice.get(), m_meshExchange);

    // 6. Initialize Dear ImGui CAD Interface
    m_gui = std::make_unique<CADGui>();
    m_gui->Init(m_window, m_rhiDevice.get());
    m_renderer->SetGui(m_gui.get());

    SetupCallbacks();
}

void Application::SetupCallbacks() {
    glfwSetWindowUserPointer(m_window, this);

    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow* window, int width, int height) {
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (app && width > 0 && height > 0) {
            app->m_width = static_cast<uint32_t>(width);
            app->m_height = static_cast<uint32_t>(height);
            if (app->m_renderer) {
                app->m_renderer->OnResize(app->m_width, app->m_height);
            }
        }
    });

    glfwSetMouseButtonCallback(m_window, [](GLFWwindow* window, int button, int action, int mods) {
        // Forward directly to Dear ImGui
        ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) return;

        double curX = 0.0, curY = 0.0;
        glfwGetCursorPos(window, &curX, &curY);

        if (action == GLFW_PRESS) {
            bool overUI = (app->m_gui && app->m_gui->WantCaptureMouse());
            app->m_pressOverUI = overUI;

            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                app->m_pressMouseX = curX;
                app->m_pressMouseY = curY;
                if (!overUI) {
                    app->m_leftMouseDown = true;
                }
            } else if (button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE) {
                if (!overUI) {
                    app->m_rightMouseDown = true;
                }
            }
        } else if (action == GLFW_RELEASE) {
            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                app->m_leftMouseDown = false;

                // Check if this was a click (not an orbit drag) in the 3D viewport
                double dragDist = std::hypot(curX - app->m_pressMouseX, curY - app->m_pressMouseY);
                bool overUI = (app->m_gui && app->m_gui->WantCaptureMouse());
                bool inSketchCanvas = (app->m_gui && app->m_gui->IsSketchModeActive());

                if (dragDist < 4.0 && !app->m_pressOverUI && !overUI && !inSketchCanvas) {
                    if (app->m_renderer && app->m_graph && app->m_gui) {
                        Ray ray = app->m_renderer->ScreenPointToRay(
                            static_cast<float>(curX),
                            static_cast<float>(curY)
                        );
                        uint64_t hitNodeId = app->m_graph->PickNode(ray.origin, ray.direction);
                        app->m_gui->SetSelectedNodeId(hitNodeId);
                    }
                }
            } else if (button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE) {
                app->m_rightMouseDown = false;
            }
        }
    });

    glfwSetCursorPosCallback(m_window, [](GLFWwindow* window, double xpos, double ypos) {
        // Forward directly to Dear ImGui
        ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);

        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) return;

        double dx = xpos - app->m_lastMouseX;
        double dy = ypos - app->m_lastMouseY;
        app->m_lastMouseX = xpos;
        app->m_lastMouseY = ypos;

        // If neither button is down and GUI wants mouse, ignore for camera
        if (!app->m_leftMouseDown && !app->m_rightMouseDown && app->m_gui && app->m_gui->WantCaptureMouse()) {
            return;
        }

        if (app->m_leftMouseDown && app->m_renderer) {
            // Orbit camera
            app->m_renderer->OrbitCamera(static_cast<float>(dx) * 0.005f, static_cast<float>(dy) * 0.005f);
        } else if (app->m_rightMouseDown && app->m_renderer) {
            // Pan camera
            app->m_renderer->PanCamera(static_cast<float>(-dx) * 0.005f, static_cast<float>(dy) * 0.005f);
        }
    });

    glfwSetScrollCallback(m_window, [](GLFWwindow* window, double xoffset, double yoffset) {
        // Forward directly to Dear ImGui
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);

        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) return;

        if (app->m_gui && app->m_gui->WantCaptureMouse()) {
            return;
        }

        if (app->m_renderer) {
            app->m_renderer->ZoomCamera(static_cast<float>(yoffset) * 0.4f);
        }
    });

    glfwSetKeyCallback(m_window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    });

    glfwSetCharCallback(m_window, [](GLFWwindow* window, unsigned int c) {
        ImGui_ImplGlfw_CharCallback(window, c);
    });

    glfwSetWindowFocusCallback(m_window, [](GLFWwindow* window, int focused) {
        ImGui_ImplGlfw_WindowFocusCallback(window, focused);
    });

    glfwSetCursorEnterCallback(m_window, [](GLFWwindow* window, int entered) {
        ImGui_ImplGlfw_CursorEnterCallback(window, entered);
    });
}

void Application::Stop() {
    if (m_running.exchange(false)) {
        if (m_renderer) {
            m_renderer->Stop();
        }
        if (m_gui) {
            m_gui->Shutdown();
        }
        if (m_threadPool) {
            m_threadPool->Shutdown();
        }
        if (m_window) {
            glfwDestroyWindow(m_window);
            m_window = nullptr;
            glfwTerminate();
        }
    }
}

void Application::Run() {
    // Trigger initial DAG evaluation on background worker pool
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    std::cout << "\n[TesseraCAD] Interactive CAD GUI active. Waiting for user input in desktop window...\n";

    // Main interactive UI & Presentation Pump
    while (!glfwWindowShouldClose(m_window) && m_running.load()) {
        glfwPollEvents();

        // 1. Begin ImGui frame
        if (m_gui) {
            m_gui->BeginFrame();

            // 2. Render CAD Tool Palette, Shape Creation, and Live Model Tree
            m_gui->RenderUI(
                *m_graph, 
                m_rhiDevice.get(), 
                m_meshExchange, 
                m_renderer->GetCamera(), 
                *m_renderer
            );
        }

        // 3. Render 3D CAD viewport and GUI frame to swapchain
        if (m_renderer) {
            m_renderer->RenderFrame();
        }
    }

    std::cout << "\n[TesseraCAD] Window close requested. Initiating graceful shutdown...\n";
    Stop();
}

} // namespace Tessera
