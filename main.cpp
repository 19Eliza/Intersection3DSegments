
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>

#include "GeometryObjects/segment3d.h"
#include "GeometryObjects/vector3d.h"
#include "InputOutput.h"

/// @brief Finding intersection of two segments. There are available cases : 1 - No complanar,
/// 2 - Coplanar (intersection at a point or not intersect), 3 - Parallel (not intersection),
/// 4 - Collinear
/// @param segment1 the first segment
/// @param segment2 the second segment
/// @return empty set or point or segmemt
std::vector<Vector3D> Intersect(const Segment3D&, const Segment3D&);

/// @brief Solve system linear equations using Cramer formula
/// @param A matrix of linear system equations
/// @param b right part
/// @return solution of linear system equations
std::optional<Eigen::Vector2d> CramerFormula(const Eigen::Matrix2d& A, const Eigen::Vector2d& b);

int main()
{
    /// output file for results

    std::string file_result{"result.txt"};
    std::ofstream fout{file_result};

    /// Tests
    std::vector<TestCase> tests = {
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

    for (const auto& test : tests)
    {
        fout << test.description << ": ";
        auto res = Intersect(test.segment1, test.segment2);
        fout << "Intersection of " << test.segment1 << "and " << test.segment2;
        PrintIntersection(res, fout);
        fout << std::endl;
    }

    /// Input from file in format : x1 y1 z1 x2 y2 z2  x3 y3 z3 x4 y4 z4
    /// s1 = (x1 y1 z1), s2 = (x2 y2 z2), s3 = (x3 y3 z3), s4 = (x4 y4 z4)
    std::string filename = "segments.txt";

    auto tests_ = ReadTestsFromFile(filename);

    for (const auto& test : tests_)
    {
        fout << test.description << ": ";
        auto res = Intersect(test.segment1, test.segment2);
        fout << "Intersection of " << test.segment1 << "and " << test.segment2;
        PrintIntersection(res, fout);
        fout << std::endl;
    }

    return 0;
}

std::optional<double> FirstNonZero(double x, double y, double z)
{
    if (x != 0.0)
        return x;
    if (y != 0.0)
        return y;
    if (z != 0.0)
        return z;
    return std::nullopt;
}

/// @brief Solve system linear equations using Cramer formula
std::optional<Eigen::Vector2d> CramerFormula(const Eigen::Matrix2d& A, const Eigen::Vector2d& b)
{
    double detA = A.determinant();

    if (!std::abs(detA))
    {
        throw std::invalid_argument("Determinate is equal zero.");
    }

    Eigen::Matrix2d Ax = A;
    Eigen::Matrix2d Ay = A;

    Ax.col(0) = b;
    Ay.col(1) = b;

    double s = Ax.determinant() / detA;
    double t = Ay.determinant() / detA;

    return Eigen::Vector2d(s, t);
}

/// @brief Finding intersection of two segments
std::vector<Vector3D> Intersect(const Segment3D& segment1, const Segment3D& segment2)
{
    Vector3D segment1_start = segment1.GetStart();
    Vector3D segment1_end = segment1.GetEnd();
    segment1.CheckSegmentValid();

    Vector3D segment2_start = segment2.GetStart();
    Vector3D segment2_end = segment2.GetEnd();
    segment2.CheckSegmentValid();

    Vector3D vector_segment1 = segment1_end - segment1_start;
    Vector3D vector_segment2 = segment2_end - segment2_start;

    auto cross_segments12 = vector_segment1.Cross(vector_segment2);
    double length_cross_segments12 = cross_segments12.LengthVector();

    auto w = (segment1_start - segment2_start).Dot(cross_segments12);
    auto w1 = (segment1_start - segment2_start).Cross(vector_segment1);
    double length_w1 = w1.LengthVector();

    if (length_cross_segments12)
    {
        if (std::abs(w))
            return {};  /// No complanar
        else
        {
            double det_xy = vector_segment1.GetX() * (-vector_segment2.GetY()) -
                            vector_segment1.GetY() * (-vector_segment2.GetX());
            double det_xz = vector_segment1.GetX() * (-vector_segment2.GetZ()) -
                            vector_segment1.GetZ() * (-vector_segment2.GetX());
            double det_yz = vector_segment1.GetY() * (-vector_segment2.GetZ()) -
                            vector_segment1.GetY() * (-vector_segment2.GetZ());

            Eigen::MatrixXd A(2, 2);
            Eigen::Vector2d b(2);

            if (det_xy)
            {
                A.col(0) << vector_segment1.GetX(), vector_segment1.GetY();
                A.col(1) << -vector_segment2.GetX(), -vector_segment2.GetY();
                b << (segment2_start - segment1_start).GetX(),
                    (segment2_start - segment1_start).GetY();
            }
            else
            {
                if (det_xz)
                {
                    A.col(0) << vector_segment1.GetX(), vector_segment1.GetZ();
                    A.col(1) << -vector_segment2.GetX(), -vector_segment2.GetZ();
                    b << (segment2_start - segment1_start).GetX(),
                        (segment2_start - segment1_start).GetZ();
                }
                else
                {
                    A.col(0) << vector_segment1.GetY(), vector_segment1.GetZ();
                    A.col(1) << -vector_segment2.GetY(), -vector_segment2.GetZ();
                    b << (segment2_start - segment1_start).GetY(),
                        (segment2_start - segment1_start).GetZ();
                }
            }

            auto x = CramerFormula(A, b);

            double s = x.value()(0);
            double t = x.value()(1);

            Vector3D segment1_point = segment1_start + vector_segment1 * s;
            Vector3D segment2_point = segment2_start + vector_segment2 * t;

            if (s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0)  /// intersection point of segments
            {
                return {segment1_start + vector_segment1 * s};
            }

            return {};  /// lines are intersected, but segments are not
        }
    }
    /// Colinear cases
    else
    {
        if (length_w1)
            return {};  /// Parallel, not collinear

        double t0;
        double t1;

        if (vector_segment1.GetX() != 0.0)
        {
            t0 = (segment2_start.GetX() - segment1_start.GetX()) / vector_segment1.GetX();
            t1 = (segment2_end.GetX() - segment1_start.GetX()) / vector_segment1.GetX();
        }
        else
        {
            if (vector_segment1.GetY() != 0.0)
            {
                t0 = (segment2_start.GetY() - segment1_start.GetY()) / vector_segment1.GetY();
                t1 = (segment2_end.GetY() - segment1_start.GetY()) / vector_segment1.GetY();
            }
            else
            {
                t0 = (segment2_start.GetZ() - segment1_start.GetZ()) / vector_segment1.GetZ();
                t1 = (segment2_end.GetZ() - segment1_start.GetZ()) / vector_segment1.GetZ();
            }
        }

        double t_start = std::max(0.0, std::min(t0, t1));
        double t_end = std::min(1.0, std::max(t0, t1));

        if (t_start < t_end)
        {
            return {segment1_start + vector_segment1 * t_start,
                    segment1_start + vector_segment1 * t_end};
        }  /// intersection is segment
        if (t_start == t_end)
        {
            return {segment1_start + vector_segment1 * t_start};
        }  /// intersection is point, end of one of the segment
        return {};
    }
};
