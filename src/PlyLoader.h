#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

struct MeshData
{
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;
};

bool loadPlyMesh(const std::string &path, MeshData &out);
