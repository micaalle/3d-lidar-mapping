#include "PlyLoader.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace
{

    int typeSize(const std::string &t)
    {
        if (t == "char" || t == "uchar" || t == "int8" || t == "uint8")
            return 1;
        if (t == "short" || t == "ushort" || t == "int16" || t == "uint16")
            return 2;
        if (t == "int" || t == "uint" || t == "float" || t == "int32" || t == "uint32" || t == "float32")
            return 4;
        if (t == "double" || t == "float64")
            return 8;
        return 0;
    }

    float readScalarAsFloat(std::istream &in, const std::string &type)
    {
        if (type == "float" || type == "float32")
        {
            float v;
            in.read(reinterpret_cast<char *>(&v), 4);
            return v;
        }
        if (type == "double" || type == "float64")
        {
            double v;
            in.read(reinterpret_cast<char *>(&v), 8);
            return static_cast<float>(v);
        }
        if (type == "int" || type == "int32")
        {
            int32_t v;
            in.read(reinterpret_cast<char *>(&v), 4);
            return static_cast<float>(v);
        }
        if (type == "uint" || type == "uint32")
        {
            uint32_t v;
            in.read(reinterpret_cast<char *>(&v), 4);
            return static_cast<float>(v);
        }
        if (type == "short" || type == "int16")
        {
            int16_t v;
            in.read(reinterpret_cast<char *>(&v), 2);
            return static_cast<float>(v);
        }
        if (type == "ushort" || type == "uint16")
        {
            uint16_t v;
            in.read(reinterpret_cast<char *>(&v), 2);
            return static_cast<float>(v);
        }
        if (type == "char" || type == "int8")
        {
            int8_t v;
            in.read(reinterpret_cast<char *>(&v), 1);
            return static_cast<float>(v);
        }
        if (type == "uchar" || type == "uint8")
        {
            uint8_t v;
            in.read(reinterpret_cast<char *>(&v), 1);
            return static_cast<float>(v);
        }

        char dummy = 0;
        in.read(&dummy, 1);
        return 0.0f;
    }

    unsigned int readScalarAsUInt(std::istream &in, const std::string &type)
    {
        if (type == "uchar" || type == "uint8")
        {
            uint8_t v;
            in.read(reinterpret_cast<char *>(&v), 1);
            return v;
        }
        if (type == "char" || type == "int8")
        {
            int8_t v;
            in.read(reinterpret_cast<char *>(&v), 1);
            return static_cast<unsigned int>(v);
        }
        if (type == "ushort" || type == "uint16")
        {
            uint16_t v;
            in.read(reinterpret_cast<char *>(&v), 2);
            return v;
        }
        if (type == "short" || type == "int16")
        {
            int16_t v;
            in.read(reinterpret_cast<char *>(&v), 2);
            return static_cast<unsigned int>(v);
        }
        if (type == "uint" || type == "uint32")
        {
            uint32_t v;
            in.read(reinterpret_cast<char *>(&v), 4);
            return v;
        }
        if (type == "int" || type == "int32")
        {
            int32_t v;
            in.read(reinterpret_cast<char *>(&v), 4);
            return static_cast<unsigned int>(v);
        }

        return static_cast<unsigned int>(readScalarAsFloat(in, type));
    }

    void computeNormals(MeshData &m)
    {
        m.normals.assign(m.vertices.size(), glm::vec3(0.0f));

        for (size_t i = 0; i + 2 < m.indices.size(); i += 3)
        {
            unsigned int ia = m.indices[i];
            unsigned int ib = m.indices[i + 1];
            unsigned int ic = m.indices[i + 2];

            if (ia >= m.vertices.size() || ib >= m.vertices.size() || ic >= m.vertices.size())
                continue;

            glm::vec3 a = m.vertices[ia];
            glm::vec3 b = m.vertices[ib];
            glm::vec3 c = m.vertices[ic];
            glm::vec3 n = glm::cross(b - a, c - a);

            m.normals[ia] += n;
            m.normals[ib] += n;
            m.normals[ic] += n;
        }

        for (auto &n : m.normals)
        {
            float len = glm::length(n);
            if (len > 1e-8f)
                n /= len;
            else
                n = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }

}

bool loadPlyMesh(const std::string &path, MeshData &out)
{
    out = MeshData{};

    std::ifstream in(path, std::ios::binary);
    if (!in)
        return false;

    std::string line;
    std::getline(in, line);
    if (line.find("ply") == std::string::npos)
        return false;

    bool binary = false;
    bool ascii = false;
    size_t vertexCount = 0;
    size_t faceCount = 0;
    bool inVertex = false;
    bool inFace = false;

    struct Prop
    {
        std::string name;
        std::string type;
        bool isList = false;
        std::string countType;
        std::string itemType;
    };

    std::vector<Prop> vertexProps;
    std::vector<Prop> faceProps;

    while (std::getline(in, line))
    {
        if (line == "end_header")
            break;

        std::istringstream ss(line);
        std::string word;
        ss >> word;

        if (word == "format")
        {
            std::string fmt;
            ss >> fmt;
            binary = (fmt == "binary_little_endian");
            ascii = (fmt == "ascii");
        }
        else if (word == "element")
        {
            std::string elem;
            size_t count;
            ss >> elem >> count;
            inVertex = (elem == "vertex");
            inFace = (elem == "face");
            if (inVertex)
                vertexCount = count;
            if (inFace)
                faceCount = count;
        }
        else if (word == "property")
        {
            Prop p;
            std::string maybeList;
            ss >> maybeList;
            if (maybeList == "list")
            {
                p.isList = true;
                ss >> p.countType >> p.itemType >> p.name;
            }
            else
            {
                p.type = maybeList;
                ss >> p.name;
            }

            if (inVertex)
                vertexProps.push_back(p);
            else if (inFace)
                faceProps.push_back(p);
        }
    }

    if (!binary && !ascii)
    {
        std::cerr << "Unsupported PLY format in " << path << std::endl;
        return false;
    }

    out.vertices.reserve(vertexCount);

    int xProp = -1, yProp = -1, zProp = -1;
    for (size_t i = 0; i < vertexProps.size(); ++i)
    {
        if (vertexProps[i].name == "x")
            xProp = static_cast<int>(i);
        if (vertexProps[i].name == "y")
            yProp = static_cast<int>(i);
        if (vertexProps[i].name == "z")
            zProp = static_cast<int>(i);
    }

    if (xProp < 0 || yProp < 0 || zProp < 0)
        return false;

    if (ascii)
    {
        for (size_t i = 0; i < vertexCount; ++i)
        {
            std::getline(in, line);
            std::istringstream ss(line);

            std::vector<float> vals(vertexProps.size(), 0.0f);
            for (size_t p = 0; p < vertexProps.size(); ++p)
                ss >> vals[p];

            out.vertices.push_back(glm::vec3(-vals[xProp], -vals[yProp], vals[zProp]));
        }

        for (size_t i = 0; i < faceCount; ++i)
        {
            std::getline(in, line);
            std::istringstream ss(line);
            int n = 0;
            ss >> n;
            std::vector<unsigned int> face(n);
            for (int j = 0; j < n; ++j)
                ss >> face[j];

            if (n >= 3)
            {
                for (int j = 1; j + 1 < n; ++j)
                {
                    out.indices.push_back(face[0]);
                    out.indices.push_back(face[j]);
                    out.indices.push_back(face[j + 1]);
                }
            }
        }
    }
    else
    {
        for (size_t i = 0; i < vertexCount; ++i)
        {
            glm::vec3 v(0.0f);

            for (size_t p = 0; p < vertexProps.size(); ++p)
            {
                float value = readScalarAsFloat(in, vertexProps[p].type);
                if (static_cast<int>(p) == xProp)
                    v.x = value;
                if (static_cast<int>(p) == yProp)
                    v.y = value;
                if (static_cast<int>(p) == zProp)
                    v.z = value;
            }

            v.x = -v.x;
            v.y = -v.y;
            out.vertices.push_back(v);
        }

        for (size_t i = 0; i < faceCount; ++i)
        {
            if (faceProps.empty() || !faceProps[0].isList)
                return false;

            unsigned int n = readScalarAsUInt(in, faceProps[0].countType);
            std::vector<unsigned int> face(n);

            for (unsigned int j = 0; j < n; ++j)
            {
                face[j] = readScalarAsUInt(in, faceProps[0].itemType);
            }

            // skip extra face properties if prsent
            for (size_t p = 1; p < faceProps.size(); ++p)
            {
                if (faceProps[p].isList)
                {
                    unsigned int c = readScalarAsUInt(in, faceProps[p].countType);
                    int sz = typeSize(faceProps[p].itemType);
                    in.seekg(static_cast<std::streamoff>(c * sz), std::ios::cur);
                }
                else
                {
                    int sz = typeSize(faceProps[p].type);
                    in.seekg(static_cast<std::streamoff>(sz), std::ios::cur);
                }
            }

            if (n >= 3)
            {
                for (unsigned int j = 1; j + 1 < n; ++j)
                {
                    out.indices.push_back(face[0]);
                    out.indices.push_back(face[j]);
                    out.indices.push_back(face[j + 1]);
                }
            }
        }
    }

    computeNormals(out);
    return !out.vertices.empty() && !out.indices.empty();
}
