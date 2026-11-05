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

    // 3. Create initial parametric primitives
    m_testBoxNode = std::make_shared<BoxNode>("ParametricBox_01", 2.0f, 1.0f, 3.0f);
    m_graph->AddNode(m_testBoxNode);

    m_testCylinderNode = std::make_shared<CylinderNode>("ParametricCylinder_01", 1.5f, 3.0f, 32);
    m_graph->AddNode(m_testCylinderNode);

    // 2D L-bracket profile for ExtrudeNode
    std::vector<std::array<float, 2>> lBracketProfile = {
        {-2.0f, -2.0f},
        { 2.0f, -2.0f},
        { 2.0f, -1.0f},
        {-1.0f, -1.0f},
        {-1.0f,  2.0f},
        {-2.0f,  2.0f}
    };
    m_testExtrudeNode = std::make_shared<ExtrudeNode>("ParametricExtrude_01", lBracketProfile, 2.0f);
    m_graph->AddNode(m_testExtrudeNode);

    m_testSphereNode = std::make_shared<SphereNode>("ParametricSphere_01", 1.8f, 16, 32);
    m_graph->AddNode(m_testSphereNode);

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

    // Trigger initial DAG evaluation of the Box
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    std::cout << "\n[Application] Simulation running. Dispatching interactive parametric edits...\n";

    // 1. Interactive Box modification
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "\n>>> [User Action] Parametric Box edit -> (5.0, 3.0, 2.0)...\n";
    SetBoxDimensions(5.0f, 3.0f, 2.0f);

    for (int i = 0; i < 4; ++i) {
        m_renderer->OrbitCamera(0.04f, 0.01f);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // 2. Interactive Cylinder modification
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::cout << "\n>>> [User Action] Parametric Cylinder edit -> Radius: 2.0, Height: 4.0, Segments: 48...\n";
    m_testCylinderNode->SetParameters(2.0f, 4.0f, 48);
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    for (int i = 0; i < 4; ++i) {
        m_renderer->OrbitCamera(0.04f, -0.01f);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // 3. Interactive Extrude (L-Bracket) height adjustment
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::cout << "\n>>> [User Action] Parametric Extrude edit -> L-Bracket Height: 3.5...\n";
    m_testExtrudeNode->SetHeight(3.5f);
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    for (int i = 0; i < 4; ++i) {
        m_renderer->PanCamera(0.02f, -0.01f);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    // 4. Interactive Sphere modification
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::cout << "\n>>> [User Action] Parametric Sphere edit -> Radius: 2.5, Rings: 24, Sectors: 48...\n";
    m_testSphereNode->SetParameters(2.5f, 24, 48);
    m_graph->EvaluateDirtyNodes(m_rhiDevice.get(), m_meshExchange);

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    std::cout << "\n[Application] Evaluation complete. Initiating graceful shutdown...\n";

    Stop();
}

} // namespace Tessera
