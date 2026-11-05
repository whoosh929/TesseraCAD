#include "Application.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace Tessera {

Application::Application() {
    std::cout << "[Application] Bootstrapping TesseraCAD Engine Subsystems...\n";

    // 1. Initialize Worker Pool (std::jthread)
    m_threadPool = std::make_shared<ThreadPool>(4);

    // 2. Initialize Parametric Graph
    m_graph = std::make_unique<ParametricGraph>(m_threadPool);

    // 3. Create initial parametric Box node (2.0 x 1.0 x 3.0)
    m_testBoxNode = std::make_shared<BoxNode>("ParametricBox_01", 2.0f, 1.0f, 3.0f);
    m_graph->AddNode(m_testBoxNode);

    // 4. Initialize Hardware RHI Device (or mock/headless if running in automated CI)
    RHI::NativeWindowHandle windowHandle{};
    try {
        m_rhiDevice = RHI::RHIDevice::Create(windowHandle, 1920, 1080);
    } catch (const std::exception& e) {
        std::cout << "[Application] Native GPU device unavailable (" << e.what() 
                  << "); running in headless simulation mode.\n";
    }

    // 5. Initialize Viewport Renderer
    m_renderer = std::make_unique<ViewportRenderer>(m_rhiDevice.get(), m_meshExchange);
}

Application::~Application() {
    Stop();
}

void Application::Stop() {
    if (m_running.exchange(false)) {
        if (m_renderer) {
            m_renderer->Stop();
        }
        if (m_threadPool) {
            m_threadPool->Shutdown();
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
    // Start presentation loop on dedicated render thread
    m_renderer->Start();

    // Trigger initial DAG evaluation
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    std::cout << "\n[Application] Simulation running. Dispatching interactive parametric edits...\n";

    // Simulate user editing CAD dimensions interactively
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "\n>>> [User Action] Changing box dimensions to (5.0, 3.0, 2.0)...\n";
    SetBoxDimensions(5.0f, 3.0f, 2.0f);

    // Camera continuously orbits on the render thread during this heavy worker job
    for (int i = 0; i < 5; ++i) {
        m_renderer->OrbitCamera(0.05f, 0.02f);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::cout << "\n>>> [User Action] Changing box dimensions to (1.5, 8.0, 1.5)...\n";
    SetBoxDimensions(1.5f, 8.0f, 1.5f);

    for (int i = 0; i < 5; ++i) {
        m_renderer->OrbitCamera(0.05f, -0.01f);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    std::cout << "\n[Application] Evaluation complete. Initiating graceful shutdown...\n";

    Stop();
}

} // namespace Tessera
