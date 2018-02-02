#version 450

layout (location = 0) out vec4 outputColor;
layout (location = 1) out vec4 outputSpecular;

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
    vec3 position;
    int type;
    vec4 color;
    vec4 direction;
};

layout (set = 1, binding = 0) uniform globalUniforms
{
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
    vec4 resolution;
    light lights[16];
} globals;

layout (set = 1, binding = 1) uniform localUniforms
{
    mat4 inverse[128];
} locals;

layout (location = 0) in vertexData{
    vec3 position;
    vec3 normal;
    vec2 uv;
    mat3 TBN;
    vec3 cameraPosition;
    flat int index;
}vertex;

layout (set = 1, binding = 2) uniform sampler2D diffuseTexture;
layout (set = 1, binding = 3) uniform sampler2D normalTexture;
layout (set = 1, binding = 4) uniform sampler2D roughnessTexture;
layout (set = 1, binding = 5) uniform sampler2D metalnessTexture;
layout (set = 1, binding = 6) uniform sampler2D subsurfaceScatteringTexture;
layout (set = 1, binding = 7) uniform sampler2D depthAttachment;
layout (set = 1, binding = 8) uniform sampler2D normalAttachment;

// Constants
const float PI = 3.1415;

// Global variables
vec3 finalDiffuse = vec3(0);
vec3 finalSpecular = vec3(0);
vec3 diffuseColor = vec3(1, 1, 1);
vec3 normal = vec3(0, 0, 1);
float specularValue = 20;
vec3 cameraDirection = vec3(0, 0, 1);
float specularPow = 0;
vec3 specularColor = vec3(1, 1, 1);
float roughness = 1.0;
float metalness = 0.0;
float subsurfaceScattering = 0.0;
vec3 diffuseIndirect = vec3(0, 0, 0);
float opacity = 1.0;
vec2 uv = vec2(0);

// Functions
void readTextures();
void calculateIndirectLighting();
void calculatePBRLighting(vec3 lightDirection, vec3 lightColor, float lightIntensity);
float geometryTerm(vec3 vector);
float squared(float x);
vec3 reconstructPosition(float depth);

void main(void)
{
    vec4 position = vec4(reconstructPosition(texture(depthAttachment, gl_FragCoord.xy/globals.resolution.xy).x), 1);

    position = locals.inverse[vertex.index] * position;
    uv = position.xy + 0.5;

    if (abs(position.x) > 0.5) discard;
    if (abs(position.y) > 0.5) discard;
    if (abs(position.z) > 0.5) discard;

    readTextures();

    normal = texture(normalAttachment, gl_FragCoord.xy/globals.resolution.xy).rgb;

    // If it's 0, then there's a divide by zero error
    roughness = max(roughness * roughness, 0.0001);

    cameraDirection = normalize(vertex.cameraPosition - vertex.position);

    //calculateIndirectLighting();

    vec3 lightRay = vec3(0);
    float lightIntensity = 0;

    for (int i = 0; i < numLights; i++)
    {
        int type = globals.lights[i].type;
        if (type > 1)
        {
            lightRay = vertex.position.xyz - globals.lights[i].position.xyz;
            float distanceSquared = dot(lightRay, lightRay);
            lightIntensity = globals.lights[i].color.a / distanceSquared;

            if (type == 3)
            {
                if (globals.lights[i].direction.a > dot(normalize(lightRay), normalize(globals.lights[i].direction.xyz)))
                {
                    lightIntensity = 0;
                }
            }
        }
        else
        {
            lightIntensity = globals.lights[i].color.a;
            lightRay = globals.lights[i].direction.xyz;
        }

        calculatePBRLighting(normalize(lightRay), globals.lights[i].color.rgb, lightIntensity);
    }

    finalDiffuse *= diffuseColor;
    outputColor = vec4(finalDiffuse, opacity);
    outputSpecular = vec4(finalSpecular, opacity);
}

void readTextures()
{
    float mipLevel = textureQueryLod(diffuseTexture, uv).x;

    if (hasDiffuseTexture)
    {
        vec4 diffuseColorTemp = texture(diffuseTexture, uv, mipLevel);
        diffuseColor = diffuseColorTemp.rgb;
        opacity = diffuseColorTemp.a;
    }
    else
    {
        diffuseColor = vec3(1, 0, 0);//locals.color.rgb;
    }

    if (hasNormalTexture)
    {
        normal = vertex.TBN * normalize((2.0 * texture(normalTexture, uv, mipLevel).rgb) - 1.0);
    }
    else
    {
        normal = vertex.TBN * normal;
    }

    if (hasRoughnessTexture)
    {
        roughness = texture(roughnessTexture, uv, mipLevel).r;
    }

    if (hasMetalnessTexture)
    {
        metalness = texture(metalnessTexture, uv, mipLevel).r;
    }

    if (hasSubsurfaceScatteringTexture)
    {
        subsurfaceScattering = texture(subsurfaceScatteringTexture, uv, mipLevel).r;
    }
}

void calculateIndirectLighting()
{
    // Fresnel term: Schlick's approximation
    vec3 F_0 = mix(vec3(0.04), diffuseColor, metalness);
    vec3 F = (F_0) + (1.0 - F_0) * pow(1.0 - max(dot(cameraDirection, normal), 0), 5);

    // Energy conservation
    vec3 k_s = F;
    vec3 k_d = (1 - k_s) * (1.0 - metalness);

    finalDiffuse += diffuseIndirect * k_d;
}

void calculatePBRLighting(vec3 lightDirection, vec3 lightColor, float lightIntensity)
{
    // Lambert BRDF
    lightDirection *= -1;
    float diffusePow = 1.0 / PI;
    float l_dot_n = max(dot(normal, lightDirection), 0);

    vec3 halfway = normalize(normalize(lightDirection) + cameraDirection);

    // Cook-Torrance BRDF

    // Distribution term: Trowbridge-Reitz
    float roughness_squared = roughness * roughness;
    float D = roughness_squared / (PI * squared(squared(max(dot(halfway, normal), 0)) * (roughness_squared - 1) + 1));

    // Fresnel term: Schlick's approximation
    vec3 F_0 = mix(vec3(0.04), diffuseColor, metalness);
    vec3 F = max((F_0) + (1.0 - F_0) * pow(1.0 - max(dot(cameraDirection, halfway), 0), 5), 0);

    // Geometry term: Schlick's GGX
    float G = geometryTerm(cameraDirection) * geometryTerm(lightDirection);

    vec3 specularPow = (D * F * G) /
        (4 * max(dot(normal, cameraDirection), 0.01) * max(dot(normal, lightDirection), 0.01));

    // Energy conservation
    vec3 k_s = F;
    vec3 k_d = (1 - k_s) * (1.0 - metalness);

    finalDiffuse += k_d * diffusePow * lightColor * l_dot_n * lightIntensity;
    finalSpecular += specularPow * specularColor * lightColor * l_dot_n * lightIntensity;
}

float geometryTerm(vec3 vector)
{
    float k = roughness / 2;
    float vector_dot_n = max(dot(vector, normal), 0);
    return vector_dot_n / (vector_dot_n * (1 - k) + k);
}

float squared(float x)
{
    return x * x;
}

vec3 reconstructPosition(float depth)
{
    vec3 coords = vec3(gl_FragCoord.xy / globals.resolution.xy, 1);
    coords.x = 2 * coords.x - 1;
    coords.y = 2 * coords.y - 1;
    coords.z = depth;

    vec4 position = vec4(coords, 1);
    position = globals.inverseProjectionMatrix * position;
    position /= position.w;

    position = globals.inverseViewMatrix * position;

    return position.xyz;
}