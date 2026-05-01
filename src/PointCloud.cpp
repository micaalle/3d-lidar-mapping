#include "PointCloud.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace
{

    std::vector<std::string> splitCsvLine(const std::string &line)
    {
        std::vector<std::string> out;
        std::string cell;
        std::stringstream ss(line);
        while (std::getline(ss, cell, ','))
            out.push_back(cell);
        return out;
    }

    std::string lower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    bool parseFloat(const std::string &s, float &v)
    {
        try
        {
            v = std::stof(s);
            return std::isfinite(v);
        }
        catch (...)
        {
            return false;
        }
    }

    bool looksLikeHeader(const std::vector<std::string> &cells)
    {
        for (auto c : cells)
        {
            c = lower(c);
            if (c == "x" || c == "y" || c == "z")
                return true;
        }
        return false;
    }

    int findCol(const std::vector<std::string> &cells, const std::string &name)
    {
        for (size_t i = 0; i < cells.size(); ++i)
        {
            if (lower(cells[i]) == name)
                return static_cast<int>(i);
        }
        return -1;
    }

}

bool loadRoomScanCsv(const std::string &path, PointCloudData &out, int maxPoints)
{
    out = PointCloudData{};

    std::ifstream in(path);
    if (!in)
    {
        std::cerr << "Could not open CSV: " << path << std::endl;
        return false;
    }

    std::string line;
    if (!std::getline(in, line))
        return false;

    auto first = splitCsvLine(line);
    bool hasHeader = looksLikeHeader(first);

    int xCol = -3, yCol = -2, zCol = -1;
    int timestampCol = 1;

    std::vector<std::vector<std::string>> rows;

    if (hasHeader)
    {
        xCol = findCol(first, "x");
        yCol = findCol(first, "y");
        zCol = findCol(first, "z");
        int ts = findCol(first, "host_timestamp_ns");
        if (ts >= 0)
            timestampCol = ts;
    }
    else
    {
        rows.push_back(first);
    }

    while (std::getline(in, line))
    {
        if (!line.empty())
            rows.push_back(splitCsvLine(line));
    }

    std::vector<glm::vec3> positions;
    positions.reserve(rows.size());

    for (const auto &row : rows)
    {
        int n = static_cast<int>(row.size());
        int xi = xCol < 0 ? n + xCol : xCol;
        int yi = yCol < 0 ? n + yCol : yCol;
        int zi = zCol < 0 ? n + zCol : zCol;

        if (xi < 0 || yi < 0 || zi < 0 || xi >= n || yi >= n || zi >= n)
            continue;

        float x, y, z;
        if (!parseFloat(row[xi], x))
            continue;
        if (!parseFloat(row[yi], y))
            continue;
        if (!parseFloat(row[zi], z))
            continue;

        // flip room because lidar orientation is defult flipped from how i scan
        positions.emplace_back(-x, -y, z);
    }

    if (positions.empty())
    {
        std::cerr << "No valid xyz points loaded from CSV." << std::endl;
        return false;
    }

    // decemate
    if (maxPoints > 0 && static_cast<int>(positions.size()) > maxPoints)
    {
        std::vector<glm::vec3> decimated;
        decimated.reserve(maxPoints);

        double step = static_cast<double>(positions.size() - 1) / static_cast<double>(maxPoints - 1);
        for (int i = 0; i < maxPoints; ++i)
        {
            size_t idx = static_cast<size_t>(std::round(i * step));
            idx = std::min(idx, positions.size() - 1);
            decimated.push_back(positions[idx]);
        }

        positions = std::move(decimated);
    }

    glm::vec3 minP = positions[0];
    glm::vec3 maxP = positions[0];
    glm::vec3 sum(0.0f);
    float maxRadius = 0.0f;

    for (const auto &p : positions)
    {
        minP = glm::min(minP, p);
        maxP = glm::max(maxP, p);
        sum += p;
        maxRadius = std::max(maxRadius, glm::length(p));
    }

    out.center = sum / static_cast<float>(positions.size());
    out.sceneRadius = glm::length(maxP - minP) * 0.5f;
    out.sceneRadius = std::max(out.sceneRadius, 1.0f);
    out.maxRadius = std::max(maxRadius, 1.0f);

    out.vertices.reserve(positions.size());

    size_t count = positions.size();
    for (size_t i = 0; i < count; ++i)
    {
        float progress = count > 1 ? static_cast<float>(i) / static_cast<float>(count - 1) : 1.0f;
        float radius = glm::length(positions[i]);
        out.vertices.push_back(PointVertex{positions[i], radius, progress});
    }

    std::cout << "Loaded " << out.vertices.size() << " points from " << path << std::endl;
    return true;
}
