#version 460

layout (constant_id = 0) const uint numLights = 0;
layout (constant_id = 1) const bool tessellation = false;
layout (constant_id = 2) const bool hasDiffuseTexture = false;
layout (constant_id = 3) const bool hasNormalTexture = false;
layout (constant_id = 4) const bool hasSpecularTexture = false;
layout (constant_id = 5) const bool hasRoughnessTexture = false;
layout (constant_id = 6) const bool hasMetalnessTexture = false;
layout (constant_id = 7) const bool hasSubsurfaceScatteringTexture = false;

struct light
{
    vec4 position;
    vec4 color;
    vec4 direction;
    ivec4 state; // 1 type, shadow map index
};

layout (set = 1, binding = 0) uniform globalUniforms
{
    mat4 inverseViewMatrices[2];
    mat4 inverseProjectionMatrices[2];
    vec4 resolution;
    light lights[16];
    mat4 lightMatrices[16];
} globals;

layout (set = 1, binding = 1) uniform localUniforms
{
    mat4 transform;
    vec4 color;
    uint receivesShadows;
    float emissivity;
    float roughness;
    float metalness;
} locals;

layout (location = 0) in vertexData{
    vec3 position;
    vec3 normal;
    vec2 uv;
    mat3 TBN;
    vec3 cameraPosition;
    flat uint view;
}vertex;

void main(void)
{
}