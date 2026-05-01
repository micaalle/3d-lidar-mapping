#include "Shader.h"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

Shader::Shader(const char *vertexSrc, const char *fragmentSrc)
{
    unsigned int vs = compile(GL_VERTEX_SHADER, vertexSrc);
    unsigned int fs = compile(GL_FRAGMENT_SHADER, fragmentSrc);

    id = glCreateProgram();
    glAttachShader(id, vs);
    glAttachShader(id, fs);
    glLinkProgram(id);
    // ok
    int ok = 0;
    glGetProgramiv(id, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char info[2048];
        glGetProgramInfoLog(id, sizeof(info), nullptr, info);
        std::cerr << "Shader link error:\n"
                  << info << std::endl;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
}

unsigned int Shader::compile(unsigned int type, const char *src)
{
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    int ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char info[2048];
        glGetShaderInfoLog(shader, sizeof(info), nullptr, info);
        std::cerr << "Shader compile error:\n"
                  << info << std::endl;
    }

    return shader;
}

void Shader::use() const
{
    glUseProgram(id);
}

void Shader::destroy()
{
    if (id)
    {
        glDeleteProgram(id);
        id = 0;
    }
}

void Shader::setInt(const std::string &name, int value) const
{
    glUniform1i(glGetUniformLocation(id, name.c_str()), value);
}

void Shader::setFloat(const std::string &name, float value) const
{
    glUniform1f(glGetUniformLocation(id, name.c_str()), value);
}

void Shader::setVec3(const std::string &name, const glm::vec3 &value) const
{
    glUniform3fv(glGetUniformLocation(id, name.c_str()), 1, glm::value_ptr(value));
}

void Shader::setMat4(const std::string &name, const glm::mat4 &value) const
{
    glUniformMatrix4fv(glGetUniformLocation(id, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}
