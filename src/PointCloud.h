#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

struct PointVertex
{
    glm::vec3 position;
    float radius;
    float progress;
};

struct PointCloudData
{
    std::vector<PointVertex> vertices;
    glm::vec3 center{0.0f};
    float sceneRadius = 1.0f;
    float maxRadius = 1.0f;
};

bool loadRoomScanCsv(const std::string &path, PointCloudData &out, int maxPoints = 0);
