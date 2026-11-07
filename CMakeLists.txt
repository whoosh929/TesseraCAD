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
# Dear ImGui Library Target
# -----------------------------------------------------------------------------
add_library(imgui STATIC
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_vulkan.cpp
)

target_include_directories(imgui PUBLIC
    external/imgui
    external/imgui/backends
)

target_compile_definitions(imgui PUBLIC
    IMGUI_IMPL_VULKAN_USE_VOLK
    VK_USE_PLATFORM_WIN32_KHR
)

target_link_libraries(imgui PUBLIC glfw volk Vulkan-Headers)

# -----------------------------------------------------------------------------
# Engine Target (tessera_engine)
# -----------------------------------------------------------------------------
add_library(tessera_engine STATIC)

target_sources(tessera_engine
    PRIVATE
        # HAL Abstract Interfaces
        src/engine/hal/RHICommon.hpp
        src/engine/hal/RHI.hpp

        # Geometric & Analytical Kernel Foundations
        src/engine/geom/Transform3D.hpp
        src/engine/geom/Axis3D.hpp
        src/engine/geom/Plane.hpp
        src/engine/geom/Curves2D.hpp

        # 2D Parametric Sketch & Profile Extraction
        src/engine/sketch/SketchProfile.hpp
        src/engine/sketch/Sketch.hpp
        src/engine/sketch/ProfileTessellator.hpp

        # Modeling Operations & B-Rep Topology
        src/engine/modeling/Topology.hpp
        src/engine/modeling/ExtrudeOp.hpp
        src/engine/modeling/RevolveOp.hpp
        src/engine/modeling/Tessellator.hpp
        src/engine/modeling/BlendGeometry.hpp
        src/engine/modeling/BlendGeometry.cpp
        src/engine/modeling/FilletOp.hpp
        src/engine/modeling/FilletOp.cpp
        src/engine/modeling/CornerBlend.hpp
        src/engine/modeling/ssi/QuadricIntersection.hpp
        src/engine/modeling/ssi/QuadricIntersection.cpp
        src/engine/modeling/topology/HalfEdgeMesh.hpp
        src/engine/modeling/topology/EulerOperators.hpp
        src/engine/modeling/topology/EulerOperators.cpp
        src/engine/modeling/blend/FilletSpine.hpp
        src/engine/modeling/blend/VertexBlend3Corner.hpp
        src/engine/modeling/blend/VertexBlend3Corner.cpp

        # Concurrency & Lock-Free Primitives
        src/engine/concurrency/AtomicMeshExchange.hpp
        src/engine/concurrency/ThreadPool.hpp
        src/engine/concurrency/SpinLock.hpp

        # Parametric DAG & Tessellation Kernel
        src/engine/graph/GeometryTypes.hpp
        src/engine/graph/Tessellation.hpp
        src/engine/graph/Node.hpp
        src/engine/graph/ParametricGraph.hpp
        src/engine/graph/DatumPlaneNode.hpp
        src/engine/graph/DatumAxisNode.hpp
        src/engine/graph/CylinderNode.hpp
        src/engine/graph/SphereNode.hpp
        src/engine/graph/SketchNode.hpp
        src/engine/graph/ExtrudeNode.hpp
        src/engine/graph/RevolveNode.hpp

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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/imgui>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tessera_engine
    PUBLIC
        ${PLATFORM_GRAPHICS_LIBS}
        imgui
)

target_compile_features(tessera_engine PUBLIC cxx_std_20)

# -----------------------------------------------------------------------------
# Application Executable Target (tessera_cad)
# -----------------------------------------------------------------------------
add_executable(tessera_cad
    src/app/main.cpp
    src/app/Application.hpp
    src/app/Application.cpp
    src/app/CADGui.hpp
    src/app/CADGui.cpp
)

target_include_directories(tessera_cad
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends
)

target_link_libraries(tessera_cad
    PRIVATE
        tessera_engine
        imgui
        glfw
        volk
        Vulkan-Headers
)

target_compile_features(tessera_cad PUBLIC cxx_std_20)

set_target_properties(tessera_cad PROPERTIES
    OUTPUT_NAME "TesseraCAD"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# -----------------------------------------------------------------------------
# Test Suite Target (test_geom)
# -----------------------------------------------------------------------------
enable_testing()

add_executable(test_geom
    tests/test_geom.cpp
)

target_include_directories(test_geom
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(test_geom
    PRIVATE
        tessera_engine
)

target_compile_features(test_geom PUBLIC cxx_std_20)

set_target_properties(test_geom PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME GeomTests COMMAND test_geom)

add_executable(test_sketch
    tests/test_sketch.cpp
)

target_include_directories(test_sketch
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(test_sketch
    PRIVATE
        tessera_engine
)

target_compile_features(test_sketch PUBLIC cxx_std_20)

set_target_properties(test_sketch PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME SketchTests COMMAND test_sketch)

add_executable(test_modeling
    tests/test_modeling.cpp
)

target_include_directories(test_modeling
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(test_modeling
    PRIVATE
        tessera_engine
)

target_compile_features(test_modeling PUBLIC cxx_std_20)

set_target_properties(test_modeling PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME ModelingTests COMMAND test_modeling)

add_executable(test_picking
    tests/test_picking.cpp
)

target_include_directories(test_picking
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(test_picking
    PRIVATE
        tessera_engine
)

target_compile_features(test_picking PUBLIC cxx_std_20)

set_target_properties(test_picking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME PickingTests COMMAND test_picking)
