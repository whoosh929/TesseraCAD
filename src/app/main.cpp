#include "Application.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::cout << "=========================================================\n";
    std::cout << "           TesseraCAD Parametric Engine (Phase 1)        \n";
    std::cout << "      Decoupled Dual-Thread Lock-Free Architecture       \n";
    std::cout << "=========================================================\n\n";

    try {
        Tessera::Application app;
        app.Run();
    } catch (const std::exception& ex) {
        std::cerr << "[Fatal Error] Exception caught: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[Fatal Error] Unknown exception caught.\n";
        return 1;
    }

    std::cout << "\n[TesseraCAD] Engine terminated cleanly.\n";
    return 0;
}
