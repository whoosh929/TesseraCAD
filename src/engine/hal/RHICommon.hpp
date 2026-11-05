#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <array>
#include <type_traits>

namespace Tessera::RHI {

// -----------------------------------------------------------------------------
// Fundamental RHI Enumerations
// -----------------------------------------------------------------------------

enum class BufferUsage : uint32_t {
    None        = 0,
    Vertex      = 1 << 0,
    Index       = 1 << 1,
    Uniform     = 1 << 2,
    Storage     = 1 << 3,
    TransferSrc = 1 << 4,
    TransferDst = 1 << 5
};

constexpr BufferUsage operator|(BufferUsage a, BufferUsage b) noexcept {
    return static_cast<BufferUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

constexpr BufferUsage operator&(BufferUsage a, BufferUsage b) noexcept {
    return static_cast<BufferUsage>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

constexpr bool HasFlag(BufferUsage mask, BufferUsage flag) noexcept {
    return (mask & flag) == flag;
}

enum class MemoryUsage : uint8_t {
    GpuOnly,       // Fast VRAM (VK: DeviceLocal, Metal: Private)
    CpuToGpu,      // Host mapped / shared (VK: HostVisible|HostCoherent, Metal: Shared)
    GpuToCpu       // Readback staging
};

enum class IndexFormat : uint8_t {
    UInt16,
    UInt32
};

enum class PrimitiveTopology : uint8_t {
    TriangleList,
    TriangleStrip,
    LineList,
    LineStrip,
    PointList
};

enum class CullMode : uint8_t {
    None,
    Front,
    Back
};

enum class FrontFace : uint8_t {
    CounterClockwise,
    Clockwise
};

enum class CompareOp : uint8_t {
    Never,
    Less,
    Equal,
    LessOrEqual,
    Greater,
    NotEqual,
    GreaterOrEqual,
    Always
};

enum class ShaderStage : uint8_t {
    Vertex   = 1 << 0,
    Fragment = 1 << 1,
    Compute  = 1 << 2
};

constexpr ShaderStage operator|(ShaderStage a, ShaderStage b) noexcept {
    return static_cast<ShaderStage>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

enum class VertexFormat : uint8_t {
    Float,
    Float2,
    Float3,
    Float4,
    UByte4_Norm,
    UInt,
    UInt2,
    UInt3
};

constexpr size_t GetVertexFormatSize(VertexFormat format) noexcept {
    switch (format) {
        case VertexFormat::Float:       return 4;
        case VertexFormat::Float2:      return 8;
        case VertexFormat::Float3:      return 12;
        case VertexFormat::Float4:      return 16;
        case VertexFormat::UByte4_Norm: return 4;
        case VertexFormat::UInt:        return 4;
        case VertexFormat::UInt2:       return 8;
        case VertexFormat::UInt3:       return 12;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// Pipeline & Layout Descriptors
// -----------------------------------------------------------------------------

struct VertexAttribute {
    uint32_t location{0};
    VertexFormat format{VertexFormat::Float3};
    uint32_t offset{0};
};

struct VertexLayout {
    uint32_t stride{0};
    std::vector<VertexAttribute> attributes;
};

struct RasterizerState {
    CullMode cullMode{CullMode::Back};
    FrontFace frontFace{FrontFace::CounterClockwise};
    bool wireframe{false};
    bool depthClamp{false};
    float depthBiasConstant{0.0f};
    float depthBiasSlope{0.0f};
};

struct DepthStencilState {
    bool depthTestEnable{true};
    bool depthWriteEnable{true};
    CompareOp depthCompareOp{CompareOp::Less};
};

struct BlendState {
    bool blendEnable{false};
};

struct RHIBufferDesc {
    size_t sizeInBytes{0};
    BufferUsage usage{BufferUsage::Vertex};
    MemoryUsage memory{MemoryUsage::CpuToGpu};
    std::string debugName;
};

struct RHIPipelineDesc {
    std::string vertexShaderSource;
    std::string fragmentShaderSource;
    const void* spvVertexBytes{nullptr};
    size_t spvVertexSize{0};
    const void* spvFragmentBytes{nullptr};
    size_t spvFragmentSize{0};
    std::string vertexEntryPoint{"main"};
    std::string fragmentEntryPoint{"main"};
    VertexLayout vertexLayout;
    PrimitiveTopology topology{PrimitiveTopology::TriangleList};
    RasterizerState rasterizer;
    DepthStencilState depthStencil;
    BlendState blendState;
    std::string debugName;
};

struct ClearColor {
    float r{0.08f};
    float g{0.08f};
    float b{0.10f};
    float a{1.0f};
};

struct RenderPassDesc {
    ClearColor clearColor;
    float clearDepth{1.0f};
    bool clearColorBuffer{true};
    bool clearDepthBuffer{true};
};

struct Viewport {
    float x{0.0f};
    float y{0.0f};
    float width{0.0f};
    float height{0.0f};
    float minDepth{0.0f};
    float maxDepth{1.0f};
};

struct ScissorRect {
    int32_t x{0};
    int32_t y{0};
    uint32_t width{0};
    uint32_t height{0};
};

// -----------------------------------------------------------------------------
// Native Window Handles (for swapchain creation)
// -----------------------------------------------------------------------------
struct NativeWindowHandle {
    void* windowHandle{nullptr};      // HWND on Win32, NSWindow* / CAMetalLayer* on macOS
    void* displayHandle{nullptr};     // HINSTANCE on Win32, nullptr on macOS
};

} // namespace Tessera::RHI
