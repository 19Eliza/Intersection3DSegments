#pragma once

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

#include "GeometryObjects/segment3d.h"
#include "GeometryObjects/vector3d.h"

struct ReturnValueIntersection
{
    Vector3D intersection_point;
    Segment3D intersection_segment;
};

void PrintIntersection(const std::vector<Vector3D>& points, std::ofstream& os)
{
    if (points.empty())
    {
        os << "\tDoes not intersection\n";
    }
    else if (points.size() == 1)
    {
        const auto& p = points[0];
        os << "\tPoint: " << p << std::endl;
    }
    else if (points.size() == 2)
    {
        const auto& p1 = points[0];
        const auto& p2 = points[1];
        os << "\tSegment: [" << p1 << "," << p2 << "]\n";
    }
}

struct TestCase
{
    Segment3D segment1;
    Segment3D segment2;
    std::string description;
};

std::vector<TestCase> ReadTestsFromFile(const std::string& filename)
{
    std::vector<TestCase> tests;
    std::ifstream file{filename};
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return tests;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        double x1, y1, z1, x2, y2, z2;
        double x3, y3, z3, x4, y4, z4;

        if (!(ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3 >> x4 >> y4 >> z4))
        {
            std::cerr << "Invalid line with coordinates: " << line << std::endl;
            continue;
        }

        TestCase test;
        test.segment1 = Segment3D(Vector3D(x1, y1, z1), Vector3D(x2, y2, z2));
        test.segment2 = Segment3D(Vector3D(x3, y3, z3), Vector3D(x4, y4, z4));
        test.description = "File input";

        tests.push_back(test);
    }

    return tests;
}
