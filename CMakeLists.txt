cmake_minimum_required(VERSION 3.20)
project(TesseraCAD
    VERSION 0.1.0
    LANGUAGES C CXX
    DESCRIPTION "High-Performance Cross-Platform 3D Parametric CAD Engine"
)

# -----------------------------------------------------------------------------
# Modular CMake Setup & Platform Configuration
# -----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(PlatformSetup)

# -----------------------------------------------------------------------------
# Engine Target (tessera_engine)
# -----------------------------------------------------------------------------
add_library(tessera_engine STATIC)

target_sources(tessera_engine
    PRIVATE
        # HAL Abstract Interfaces
        src/engine/hal/RHICommon.hpp
        src/engine/hal/RHI.hpp

        # Concurrency & Lock-Free Primitives
        src/engine/concurrency/AtomicMeshExchange.hpp
        src/engine/concurrency/ThreadPool.hpp
        src/engine/concurrency/SpinLock.hpp

        # Parametric DAG & Tessellation Kernel
        src/engine/graph/GeometryTypes.hpp
        src/engine/graph/Tessellation.hpp
        src/engine/graph/Node.hpp
        src/engine/graph/ParametricGraph.hpp
        src/engine/graph/BoxNode.hpp
        src/engine/graph/CylinderNode.hpp
        src/engine/graph/SphereNode.hpp
        src/engine/graph/ExtrudeNode.hpp

        # Render Loop & Viewport
        src/engine/render/Camera.hpp
        src/engine/render/MeshRenderProxy.hpp
        src/engine/render/CADShaders.hpp
        src/engine/render/ViewportRenderer.hpp
        src/engine/render/ViewportRenderer.cpp
)

# Configure platform-specific HAL sources
if(APPLE)
    set_source_files_properties(
        src/engine/hal/metal/MetalDevice.cpp
        src/engine/hal/metal/MetalBuffer.cpp
        PROPERTIES LANGUAGE CXX
    )
    target_sources(tessera_engine
        PRIVATE
            src/engine/hal/metal/MetalDevice.hpp
            src/engine/hal/metal/MetalDevice.cpp
            src/engine/hal/metal/MetalBuffer.hpp
            src/engine/hal/metal/MetalBuffer.cpp
    )
    target_link_libraries(tessera_engine PUBLIC MetalCpp::MetalCpp)
elseif(WIN32)
    target_sources(tessera_engine
        PRIVATE
            src/engine/hal/vulkan/VulkanBuffer.hpp
            src/engine/hal/vulkan/VulkanBuffer.cpp
            src/engine/hal/vulkan/VulkanDevice.hpp
            src/engine/hal/vulkan/VulkanDevice.cpp
    )
endif()

target_include_directories(tessera_engine
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tessera_engine
    PUBLIC
        ${PLATFORM_GRAPHICS_LIBS}
)

target_compile_features(tessera_engine PUBLIC cxx_std_20)

# -----------------------------------------------------------------------------
# Application Executable Target (tessera_cad)
# -----------------------------------------------------------------------------
add_executable(tessera_cad
    src/app/main.cpp
    src/app/Application.hpp
    src/app/Application.cpp
)

target_include_directories(tessera_cad
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(tessera_cad
    PRIVATE
        tessera_engine
        glfw
)

target_compile_features(tessera_cad PUBLIC cxx_std_20)

set_target_properties(tessera_cad PROPERTIES
    OUTPUT_NAME "TesseraCAD"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
