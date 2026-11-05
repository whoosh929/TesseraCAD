cmake_minimum_required(VERSION 3.25)
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
        src/engine/hal/RHICommon.hpp
        src/engine/hal/RHI.hpp
        src/engine/concurrency/AtomicMeshExchange.hpp
)

# Configure platform-specific HAL sources
if(APPLE)
    # Enable Objective-C++ compilation if using .mm, or C++20 for .cpp
    set_source_files_properties(
        src/engine/hal/metal/MetalDevice.cpp
        PROPERTIES LANGUAGE CXX
    )
    target_sources(tessera_engine
        PRIVATE
            # Backend translation units will be added as implemented
            # src/engine/hal/metal/MetalDevice.cpp
            # src/engine/hal/metal/MetalBuffer.cpp
    )
    target_link_libraries(tessera_engine PUBLIC MetalCpp::MetalCpp)
elseif(WIN32)
    target_sources(tessera_engine
        PRIVATE
            # src/engine/hal/vulkan/VulkanDevice.cpp
            # src/engine/hal/vulkan/VulkanBuffer.cpp
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

target_compile_features(tessera_cad PRIVATE cxx_std_20)

set_target_properties(tessera_cad PROPERTIES
    OUTPUT_NAME "TesseraCAD"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
