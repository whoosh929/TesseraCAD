#pragma once

#include "engine/hal/RHICommon.hpp"
#include <array>
#include <string>

namespace Tessera {

// -----------------------------------------------------------------------------
// Uniform Buffer Structs (16-byte aligned for STD140 / Metal)
// -----------------------------------------------------------------------------

struct alignas(16) CADModelUniforms {
    std::array<float, 16> modelViewProjection;
    std::array<float, 4> cameraPosition;
    std::array<float, 4> lightDirection; // Key light direction (normalized)
    std::array<float, 4> surfaceColor;   // CAD neutral aluminum / steel
    std::array<float, 4> lightingParams; // x: ambient, y: diffuse, z: specular, w: shininess
};

struct alignas(16) CADWireframeUniforms {
    std::array<float, 16> modelViewProjection;
    std::array<float, 4> edgeColor;      // Sharp feature edge color (dark charcoal)
};

struct alignas(16) CADGridUniforms {
    std::array<float, 16> viewProjection;
    std::array<float, 16> invViewProjection;
    std::array<float, 4> cameraPosition;
    float nearPlane{0.1f};
    float farPlane{1000.0f};
    float gridScale{1.0f}; // 1.0 unit major grid, 0.1 unit minor grid
    float padding{0.0f};
};

// -----------------------------------------------------------------------------
// Shaders: Apple Metal (MSL) & Vulkan (GLSL)
// -----------------------------------------------------------------------------
class CADShaders {
public:
    // -------------------------------------------------------------------------
    // 1. Shaded CAD Surface Shader (Metal MSL)
    // -------------------------------------------------------------------------
    static inline const std::string Metal_ShadedSurface = R"(
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
};

struct VertexOut {
    float4 position [[position]];
    float3 worldNormal;
    float3 worldPos;
};

struct ModelUniforms {
    float4x4 modelViewProjection;
    float4 cameraPosition;
    float4 lightDirection;
    float4 surfaceColor;
    float4 lightingParams; // x: ambient, y: diffuse, z: specular, w: shininess
};

vertex VertexOut vertex_main(
    VertexIn in [[stage_in]],
    constant ModelUniforms& uniforms [[buffer(1)]]
) {
    VertexOut out;
    out.position = uniforms.modelViewProjection * float4(in.position, 1.0);
    out.worldNormal = in.normal;
    out.worldPos = in.position;
    return out;
}

fragment float4 fragment_main(
    VertexOut in [[stage_in]],
    constant ModelUniforms& uniforms [[buffer(1)]]
) {
    float3 N = normalize(in.worldNormal);
    float3 V = normalize(uniforms.cameraPosition.xyz - in.worldPos);
    float3 L1 = normalize(uniforms.lightDirection.xyz);
    float3 L2 = normalize(float3(-uniforms.lightDirection.x, 0.3, -uniforms.lightDirection.z));

    // Two-tone CAD directional lighting
    float diff1 = max(dot(N, L1), 0.0);
    float diff2 = max(dot(N, L2), 0.0) * 0.4; // Soft fill light

    // Half-angle Blinn-Phong specular highlight
    float3 H = normalize(L1 + V);
    float spec = pow(max(dot(N, H), 0.0), uniforms.lightingParams.w) * uniforms.lightingParams.z;

    // Subtle Fresnel rim lighting for surface definition
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.15;

    float ambient = uniforms.lightingParams.x;
    float3 litColor = uniforms.surfaceColor.rgb * (ambient + diff1 * uniforms.lightingParams.y + diff2) + float3(spec + fresnel);

    return float4(litColor, uniforms.surfaceColor.a);
}
)";

    // -------------------------------------------------------------------------
    // 2. Crisp Feature-Edge / Wireframe Shader (Metal MSL)
    // -------------------------------------------------------------------------
    static inline const std::string Metal_Wireframe = R"(
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
};

struct VertexOut {
    float4 position [[position]];
};

struct WireframeUniforms {
    float4x4 modelViewProjection;
    float4 edgeColor;
};

vertex VertexOut vertex_main(
    VertexIn in [[stage_in]],
    constant WireframeUniforms& uniforms [[buffer(1)]]
) {
    VertexOut out;
    float4 pos = uniforms.modelViewProjection * float4(in.position, 1.0);
    // Slight depth bias in clip space to eliminate z-fighting
    pos.z -= 0.0001 * pos.w;
    out.position = pos;
    return out;
}

fragment float4 fragment_main(
    VertexOut in [[stage_in]],
    constant WireframeUniforms& uniforms [[buffer(1)]]
) {
    return uniforms.edgeColor;
}
)";

    // -------------------------------------------------------------------------
    // 3. Infinite Ground Grid / Datum Plane Shader (Metal MSL)
    // -------------------------------------------------------------------------
    static inline const std::string Metal_InfiniteGrid = R"(
#include <metal_stdlib>
using namespace metal;

struct GridUniforms {
    float4x4 viewProjection;
    float4x4 invViewProjection;
    float4 cameraPosition;
    float nearPlane;
    float farPlane;
    float gridScale;
    float padding;
};

struct VertexOut {
    float4 position [[position]];
    float3 nearPoint;
    float3 farPoint;
};

// Generates fullscreen triangle from vertex ID without vertex buffers
vertex VertexOut vertex_main(
    uint vertexId [[vertex_id]],
    constant GridUniforms& uniforms [[buffer(1)]]
) {
    float2 gridPlane[6] = {
        float2(-1, -1), float2( 1, -1), float2( 1,  1),
        float2( 1,  1), float2(-1,  1), float2(-1, -1)
    };

    float2 p = gridPlane[vertexId];
    VertexOut out;
    out.position = float4(p, 0.0, 1.0);

    // Unproject near and far points
    float4 nearNDC = float4(p, 0.0, 1.0);
    float4 farNDC  = float4(p, 1.0, 1.0);

    float4 nearWorld = uniforms.invViewProjection * nearNDC;
    float4 farWorld  = uniforms.invViewProjection * farNDC;

    out.nearPoint = nearWorld.xyz / nearWorld.w;
    out.farPoint  = farWorld.xyz  / farWorld.w;
    return out;
}

fragment float4 fragment_main(
    VertexOut in [[stage_in]],
    constant GridUniforms& uniforms [[buffer(1)]]
) {
    // Intersect view ray with ground plane Y = 0
    float t = -in.nearPoint.y / (in.farPoint.y - in.nearPoint.y);
    if (t <= 0.0) {
        discard_fragment();
    }

    float3 worldPos = in.nearPoint + t * (in.farPoint - in.nearPoint);

    // Compute depth for correct scene occlusion
    float4 clipPos = uniforms.viewProjection * float4(worldPos, 1.0);
    float depth = clipPos.z / clipPos.w;

    // Anti-aliased dual frequency grid lines
    float2 coord = worldPos.xz / uniforms.gridScale;
    float2 derivative = fwidth(coord);
    float2 grid = abs(fract(coord - 0.5) - 0.5) / derivative;
    float line = min(grid.x, grid.y);
    float c = 1.0 - min(line, 1.0);

    // Axis lines (X axis = Red, Z axis = Blue)
    float2 axisDerivative = fwidth(worldPos.xz);
    float2 axisGrid = abs(worldPos.xz) / axisDerivative;
    float zAxis = 1.0 - min(axisGrid.x, 1.0); // X = 0 (Z axis)
    float xAxis = 1.0 - min(axisGrid.y, 1.0); // Z = 0 (X axis)

    float4 color = float4(0.25, 0.25, 0.28, c * 0.4);

    if (xAxis > 0.1) color = mix(color, float4(0.85, 0.25, 0.25, 0.9), xAxis);
    if (zAxis > 0.1) color = mix(color, float4(0.25, 0.45, 0.85, 0.9), zAxis);

    // Distance fade towards horizon
    float dist = length(worldPos.xz - uniforms.cameraPosition.xz);
    float fade = exp(-dist * 0.025);
    color.a *= fade;

    if (color.a < 0.01) {
        discard_fragment();
    }

    return color;
}
)";

    // -------------------------------------------------------------------------
    // 4. Shaded CAD Surface Shader (Vulkan GLSL)
    // -------------------------------------------------------------------------
    static inline const std::string Vulkan_ShadedVertex = R"(
#version 450
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(push_constant) uniform PushConstants {
    mat4 modelViewProjection;
    vec4 cameraPosition;
    vec4 lightDirection;
    vec4 surfaceColor;
    vec4 lightingParams;
} pc;

layout(location = 0) out vec3 outWorldNormal;
layout(location = 1) out vec3 outWorldPos;

void main() {
    gl_Position = pc.modelViewProjection * vec4(inPosition, 1.0);
    outWorldNormal = inNormal;
    outWorldPos = inPosition;
}
)";

    static inline const std::string Vulkan_ShadedFragment = R"(
#version 450
layout(location = 0) in vec3 inWorldNormal;
layout(location = 1) in vec3 inWorldPos;

layout(push_constant) uniform PushConstants {
    mat4 modelViewProjection;
    vec4 cameraPosition;
    vec4 lightDirection;
    vec4 surfaceColor;
    vec4 lightingParams;
} pc;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 N = normalize(inWorldNormal);
    vec3 V = normalize(pc.cameraPosition.xyz - inWorldPos);
    vec3 L1 = normalize(pc.lightDirection.xyz);
    vec3 L2 = normalize(vec3(-pc.lightDirection.x, 0.3, -pc.lightDirection.z));

    float diff1 = max(dot(N, L1), 0.0);
    float diff2 = max(dot(N, L2), 0.0) * 0.4;

    vec3 H = normalize(L1 + V);
    float spec = pow(max(dot(N, H), 0.0), pc.lightingParams.w) * pc.lightingParams.z;
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.15;

    float ambient = pc.lightingParams.x;
    vec3 litColor = pc.surfaceColor.rgb * (ambient + diff1 * pc.lightingParams.y + diff2) + vec3(spec + fresnel);
    outColor = vec4(litColor, pc.surfaceColor.a);
}
)";

    // -------------------------------------------------------------------------
    // 5. Crisp Wireframe Shader (Vulkan GLSL)
    // -------------------------------------------------------------------------
    static inline const std::string Vulkan_WireframeVertex = R"(
#version 450
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(push_constant) uniform PushConstants {
    mat4 modelViewProjection;
    vec4 edgeColor;
} pc;

void main() {
    vec4 pos = pc.modelViewProjection * vec4(inPosition, 1.0);
    pos.z -= 0.0001 * pos.w; // Depth bias
    gl_Position = pos;
}
)";

    static inline const std::string Vulkan_WireframeFragment = R"(
#version 450
layout(push_constant) uniform PushConstants {
    mat4 modelViewProjection;
    vec4 edgeColor;
} pc;

layout(location = 0) out vec4 outColor;

void main() {
    outColor = pc.edgeColor;
}
)";

    // -------------------------------------------------------------------------
    // 6. Infinite Ground Grid Shader (Vulkan GLSL)
    // -------------------------------------------------------------------------
    static inline const std::string Vulkan_GridVertex = R"(
#version 450
layout(push_constant) uniform PushConstants {
    mat4 viewProjection;
    mat4 invViewProjection;
    vec4 cameraPosition;
    float nearPlane;
    float farPlane;
    float gridScale;
    float padding;
} pc;

layout(location = 0) out vec3 outNearPoint;
layout(location = 1) out vec3 outFarPoint;

const vec2 gridPlane[6] = vec2[](
    vec2(-1, -1), vec2( 1, -1), vec2( 1,  1),
    vec2( 1,  1), vec2(-1,  1), vec2(-1, -1)
);

void main() {
    vec2 p = gridPlane[gl_VertexIndex];
    gl_Position = vec4(p, 0.0, 1.0);

    vec4 nearWorld = pc.invViewProjection * vec4(p, 0.0, 1.0);
    vec4 farWorld  = pc.invViewProjection * vec4(p, 1.0, 1.0);

    outNearPoint = nearWorld.xyz / nearWorld.w;
    outFarPoint  = farWorld.xyz  / farWorld.w;
}
)";

    static inline const std::string Vulkan_GridFragment = R"(
#version 450
layout(location = 0) in vec3 inNearPoint;
layout(location = 1) in vec3 inFarPoint;

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;
    mat4 invViewProjection;
    vec4 cameraPosition;
    float nearPlane;
    float farPlane;
    float gridScale;
    float padding;
} pc;

layout(location = 0) out vec4 outColor;

void main() {
    float t = -inNearPoint.y / (inFarPoint.y - inNearPoint.y);
    if (t <= 0.0) discard;

    vec3 worldPos = inNearPoint + t * (inFarPoint - inNearPoint);

    vec2 coord = worldPos.xz / pc.gridScale;
    vec2 derivative = fwidth(coord);
    vec2 grid = abs(fract(coord - 0.5) - 0.5) / derivative;
    float line = min(grid.x, grid.y);
    float c = 1.0 - min(line, 1.0);

    vec2 axisDerivative = fwidth(worldPos.xz);
    vec2 axisGrid = abs(worldPos.xz) / axisDerivative;
    float zAxis = 1.0 - min(axisGrid.x, 1.0);
    float xAxis = 1.0 - min(axisGrid.y, 1.0);

    vec4 color = vec4(0.25, 0.25, 0.28, c * 0.4);
    if (xAxis > 0.1) color = mix(color, vec4(0.85, 0.25, 0.25, 0.9), xAxis);
    if (zAxis > 0.1) color = mix(color, vec4(0.25, 0.45, 0.85, 0.9), zAxis);

    float dist = length(worldPos.xz - pc.cameraPosition.xz);
    color.a *= exp(-dist * 0.025);

    if (color.a < 0.01) discard;
    outColor = color;
}
)";
};

} // namespace Tessera
