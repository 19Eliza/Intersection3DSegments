
// #include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>

#include "GeometryObjects/segment3d.h"
#include "GeometryObjects/vector3d.h"
// #include "InputOutput.h"

/// @brief Finding intersection of two segments. There are available cases :
/// @param segment1 the first segment
/// @param segment2 the second segment
/// @return empty set or point or segmemt
std::vector<Vector3D> Intersect(const Segment3D&, const Segment3D&);

/// @brief Print results of Intersect in results.txt
void PrintIntersection(const std::vector<Vector3D>& points, std::ofstream& os);

/// @brief Test case
struct TestCase
{
    Segment3D segment1;
    Segment3D segment2;
    std::string description;
};

/// @brief Read tests from segments.txt file
std::vector<TestCase> ReadTestsFromFile(const std::string& filename);

int main()
{
    /// output file for results

    std::string file_result{"result.txt"};
    std::ofstream fout{file_result};

    /// Running tests cases
    auto RunTests = [](const std::vector<TestCase>& tests, std::ofstream& out)
    {
        for (const auto& test : tests)
        {
            out << test.description << ":\n";
            auto result = Intersect(test.segment1, test.segment2);

            out << "Intersection of " << test.segment1 << " and " << test.segment2 << std::endl;
            PrintIntersection(result, out);
            out << "\n";
        }
    };

    /// Tests
    std::vector<TestCase> tests1 = {
        {Segment3D(Vector3D(0, 0, 0), Vector3D(2, 0, 0)),
         Segment3D(Vector3D(1, 0, 0), Vector3D(4, 2, 10)), "Coplanar, intersection at a point"},
        {Segment3D(Vector3D(0, 0, 0), Vector3D(4, 0, 0)),
         Segment3D(Vector3D(2, 0, 0), Vector3D(6, 0, 0)), "Collinear, overlapping: "},
        {Segment3D(Vector3D(0, 0, 0), Vector3D(4, 0, 0)),
         Segment3D(Vector3D(1, 0, 0), Vector3D(3, 0, 0)), "Collinear, full overlapping: "},
        {Segment3D(Vector3D(0, 0, 0), Vector3D(2, 0, 0)),
         Segment3D(Vector3D(2, 0, 0), Vector3D(4, 0, 0)), "Collinear, intersecting at the ends: "},
        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
         Segment3D(Vector3D(2, 0, 0), Vector3D(3, 0, 0)), "Collinear, do not intersect: "},
        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
         Segment3D(Vector3D(0, 1, 1), Vector3D(0, 2, 2)), "Not coplanar: "},
        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
         Segment3D(Vector3D(0, 1, 0), Vector3D(1, 1, 0)), "Parallel, not collinear: "}};

    RunTests(tests1, fout);

    /// Input from file in format : x1 y1 z1 x2 y2 z2  x3 y3 z3 x4 y4 z4
    /// s1 = (x1,y1,z1), s2 = (x2, y2, z2), s3 = (x3, y3, z3), s4 = (x4, y4, z4)
    std::string filename = "segments.txt";

    auto tests2 = ReadTestsFromFile(filename);

    RunTests(tests2, fout);

    std::cout << "Results write to result.txt\n";

    return 0;
}

/// @brief Finding intersection of two segments
std::vector<Vector3D> Intersect(const Segment3D& segment1, const Segment3D& segment2)
{
    segment1.CheckSegmentValid();
    segment2.CheckSegmentValid();

    Vector3D segment1_start = segment1.GetStart();
    Vector3D segment1_end = segment1.GetEnd();

    Vector3D segment2_start = segment2.GetStart();
    Vector3D segment2_end = segment2.GetEnd();

    Vector3D vector_segment1 = segment1_end - segment1_start;
    Vector3D vector_segment2 = segment2_end - segment2_start;

    auto cross_segments12 = vector_segment1.Cross(vector_segment2);
    double length_cross_segments12 = cross_segments12.LengthVector();

    auto w = (segment1_start - segment2_start).Dot(cross_segments12);
    auto w1 = (segment1_start - segment2_start).Cross(vector_segment1);
    double length_w1 = w1.LengthVector();

    ///  Non colinear cases
    if (IsNonColinear(segment1, segment2))
    {
        if (std::abs(w))
            return {};  /// No coplanar
        else
        {
            return IntersectionOfCoplanarNonColinear(segment1, segment2);  /// coplanar non colinear
        }
    }
    /// Colinear cases
    else
    {
        return IntersectionOfColinear(segment1, segment2);
    }
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