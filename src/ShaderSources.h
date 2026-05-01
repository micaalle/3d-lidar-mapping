#pragma once

namespace Shaders
{

    static const char *PointVertex = R"GLSL(
#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in float aRadius;
layout(location = 2) in float aProgress;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uPointSize;
uniform float uRenderDistance;
uniform float uBuildProgress;

out float vVisible;

void main()
{
    bool visible = (aRadius <= uRenderDistance) && (aProgress <= uBuildProgress);
    vVisible = visible ? 1.0 : 0.0;

    gl_Position = uProj * uView * vec4(aPosition, 1.0);
    gl_PointSize = visible ? uPointSize : 0.0;
}
)GLSL";

    static const char *PointFragment = R"GLSL(
#version 330 core

in float vVisible;

uniform vec3 uPointColor;
uniform float uAlpha;

out vec4 FragColor;

void main()
{
    if (vVisible < 0.5)
        discard;

    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(coord, coord);

    if (r2 > 1.0)
        discard;

    // Soft circular splat edge.
    float edge = smoothstep(1.0, 0.70, r2);
    float centerBoost = 1.0 - 0.20 * r2;

    FragColor = vec4(uPointColor * centerBoost, uAlpha * edge);
}
)GLSL";

    static const char *MeshVertex = R"GLSL(
#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProj;
uniform mat4 uModel;

out vec3 vNormal;
out vec3 vWorldPos;

void main()
{
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorldPos = world.xyz;
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uProj * uView * world;
}
)GLSL";

    static const char *MeshFragment = R"GLSL(
#version 330 core

in vec3 vNormal;
in vec3 vWorldPos;

uniform vec3 uMeshColor;
uniform float uMeshAlpha;
uniform vec3 uLightDir;

out vec4 FragColor;

void main()
{
    vec3 n = normalize(vNormal);
    float diff = max(dot(n, normalize(-uLightDir)), 0.0);
    float light = 0.30 + 0.70 * diff;

    FragColor = vec4(uMeshColor * light, uMeshAlpha);
}
)GLSL";

}
