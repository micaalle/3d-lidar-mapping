#pragma once
#include <string>
#include <glm/glm.hpp>

class Shader
{
public:
    unsigned int id = 0;

    Shader() = default;
    Shader(const char *vertexSrc, const char *fragmentSrc);

    void use() const;
    void destroy();

    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setMat4(const std::string &name, const glm::mat4 &value) const;

private:
    static unsigned int compile(unsigned int type, const char *src);
};
