# =============================================================================
# FindMetalCpp.cmake
# Discovers or fetches Apple metal-cpp (zero-overhead C++ bindings for Metal)
# =============================================================================

if(TARGET MetalCpp::MetalCpp)
    return()
endif()

set(METAL_CPP_SEARCH_PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/external/metal-cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external/metal-cpp"
    "/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/System/Library/Frameworks/Metal.framework"
    "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks/Metal.framework"
)

find_path(METAL_CPP_INCLUDE_DIR
    NAMES Metal/Metal.hpp
    PATHS ${METAL_CPP_SEARCH_PATHS}
    PATH_SUFFIXES metal-cpp
)

if(NOT METAL_CPP_INCLUDE_DIR)
    message(STATUS "[metal-cpp] Local headers not found; downloading official metal-cpp release via FetchContent...")
    include(FetchContent)
    
    # Official Apple metal-cpp release mirror or distribution
    FetchContent_Declare(
        metal_cpp_pkg
        URL https://developer.apple.com/metal/cpp/files/metal-cpp_macOS14_iOS17-beta.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(metal_cpp_pkg)
    set(METAL_CPP_INCLUDE_DIR ${metal_cpp_pkg_SOURCE_DIR})
endif()

message(STATUS "[metal-cpp] Include directory: ${METAL_CPP_INCLUDE_DIR}")

add_library(MetalCpp INTERFACE)
add_library(MetalCpp::MetalCpp ALIAS MetalCpp)
target_include_directories(MetalCpp INTERFACE ${METAL_CPP_INCLUDE_DIR})

