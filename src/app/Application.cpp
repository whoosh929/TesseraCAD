#include "Application.hpp"
#include <GLFW/glfw3.h>

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

    // 3. Create initial parametric primitives
    m_testBoxNode = std::make_shared<BoxNode>("ParametricBox_01", 3.0f, 1.5f, 4.0f);
    m_graph->AddNode(m_testBoxNode);

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
        (void)mods;
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) return;

        if (app->m_gui && app->m_gui->WantCaptureMouse()) {
            return;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            app->m_leftMouseDown = (action == GLFW_PRESS);
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE) {
            app->m_rightMouseDown = (action == GLFW_PRESS);
        }
    });

    glfwSetCursorPosCallback(m_window, [](GLFWwindow* window, double xpos, double ypos) {
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) return;

        double dx = xpos - app->m_lastMouseX;
        double dy = ypos - app->m_lastMouseY;
        app->m_lastMouseX = xpos;
        app->m_lastMouseY = ypos;

        if (app->m_gui && app->m_gui->WantCaptureMouse()) {
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
        (void)xoffset;
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (!app) return;

        if (app->m_gui && app->m_gui->WantCaptureMouse()) {
            return;
        }

        if (app->m_renderer) {
            app->m_renderer->ZoomCamera(static_cast<float>(yoffset) * 0.4f);
        }
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

void Application::SetBoxDimensions(float width, float height, float depth) {
    if (m_testBoxNode) {
        m_testBoxNode->SetDimensions(width, height, depth);
        m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);
    }
}

void Application::Run() {
    // Start presentation loop on dedicated render thread (locked at unblocked 120 FPS)
    m_renderer->Start();

    // Trigger initial DAG evaluation of initial Box
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    std::cout << "\n[TesseraCAD] Interactive CAD GUI active. Waiting for user input in desktop window...\n";

    // Main interactive UI & Event Pump
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

        // Render loop runs concurrently on m_renderThread without blocking!
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }

    std::cout << "\n[TesseraCAD] Window close requested. Initiating graceful shutdown...\n";
    Stop();
}

} // namespace Tessera
