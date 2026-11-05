include(FetchContent)

# -----------------------------------------------------------------------------
# C++ Standard & Compiler Flags
# -----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(
        /W4
        /permissive-
        /volatile:iso
        /Zc:preprocessor
        /Zc:__cplusplus
        /EHsc
    )
    add_compile_definitions(
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
else()
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wno-unused-parameter
    )
endif()

# -----------------------------------------------------------------------------
# Windowing Subsystem: GLFW via FetchContent
# -----------------------------------------------------------------------------
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

# -----------------------------------------------------------------------------
# Platform-Specific Graphics Backends
# -----------------------------------------------------------------------------
if(APPLE)
    message(STATUS "[TesseraCAD] Configuring macOS platform backend: Apple Metal")

    # Link native Apple frameworks
    find_library(METAL_FRAMEWORK Metal REQUIRED)
    find_library(QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
    find_library(FOUNDATION_FRAMEWORK Foundation REQUIRED)
    find_library(APPKIT_FRAMEWORK AppKit REQUIRED)

    set(PLATFORM_GRAPHICS_LIBS
        ${METAL_FRAMEWORK}
        ${QUARTZCORE_FRAMEWORK}
        ${FOUNDATION_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
    )

    include(${CMAKE_CURRENT_LIST_DIR}/FindMetalCpp.cmake)

elseif(WIN32)
    message(STATUS "[TesseraCAD] Configuring Windows platform backend: Vulkan 1.3")

    find_package(Vulkan QUIET)

    if(Vulkan_FOUND)
        message(STATUS "[TesseraCAD] Found installed Vulkan SDK at: ${Vulkan_INCLUDE_DIRS}")
        set(PLATFORM_GRAPHICS_LIBS Vulkan::Vulkan)
    else()
        message(STATUS "[TesseraCAD] Vulkan SDK not found locally; fetching volk + Vulkan-Headers")
        
        FetchContent_Declare(
            vulkan_headers
            GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
            GIT_TAG v1.3.283
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(vulkan_headers)

        FetchContent_Declare(
            volk
            GIT_REPOSITORY https://github.com/zeux/volk.git
            GIT_TAG vulkan-sdk-1.3.283.0
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(volk)

        set(PLATFORM_GRAPHICS_LIBS volk Vulkan-Headers)
    endif()

    add_compile_definitions(VK_USE_PLATFORM_WIN32_KHR)

else()
    message(FATAL_ERROR "[TesseraCAD] Unsupported platform. Only macOS and Windows are supported.")
endif()
