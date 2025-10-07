
#include <iostream>
#include <optional>

#include "GeometryObjects/segment3d.h"
#include "GeometryObjects/vector3d.h"
#include "function.h"

struct ReturnValueIntersection
{
    std::optional<Vector3D> intersection_point;
    std::optional<Segment3D> intersection_segment;
};

/// @brief Finding intersection of two segments. There are available cases : 1 - No complanar,
/// 2 - Coplanar (intersection at a point or not intersect), 3 - Parallel (not intersection),
/// 4 - Collinear
/// @param segment1 the first segment
/// @param segment2 the second segment
/// @return empty set or point or segmemt
std::optional<ReturnValueIntersection> Intersection3DSegments(const Segment3D&, const Segment3D&);

std::optional<Eigen::Vector2d> CramerFormula(const Eigen::Matrix2d& A, const Eigen::Vector2d& b);

void PrintVector(const std::optional<Vector3D>& vecOpt)
{
    if (vecOpt)
    {
        const Vector3D& vector_segment2 = *vecOpt;
        std::cout << "(" << vector_segment2.GetX() << ", " << vector_segment2.GetY() << ", "
                  << vector_segment2.GetZ() << ")";
    }
    else
    {
        std::cout << "нет пересечения";
    }
}

int main()
{
    struct TestCase
    {
        Segment3D segment1;
        Segment3D segment2;
        std::string description;
    };

    std::vector<TestCase> tests = {
        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 1, 0)),
         Segment3D(Vector3D(0, 1, 0), Vector3D(1, 0, 0)), "Coplanar, intersection at a point"},

        {Segment3D(Vector3D(0, 0, 0), Vector3D(4, 0, 0)),
         Segment3D(Vector3D(2, 0, 0), Vector3D(6, 0, 0)), "Collinear, overlapping"},

        {Segment3D(Vector3D(0, 0, 0), Vector3D(4, 0, 0)),
         Segment3D(Vector3D(1, 0, 0), Vector3D(3, 0, 0)), "Collinear, full inclusion"},

        {Segment3D(Vector3D(0, 0, 0), Vector3D(2, 0, 0)),
         Segment3D(Vector3D(2, 0, 0), Vector3D(4, 0, 0)), "Collinear, intersecting at the ends"},

        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
         Segment3D(Vector3D(2, 0, 0), Vector3D(3, 0, 0)), "Collinear, do not intersect"},

        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
         Segment3D(Vector3D(0, 1, 1), Vector3D(0, 2, 2)), "Not coplanar"},

        {Segment3D(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
         Segment3D(Vector3D(0, 1, 0), Vector3D(1, 1, 0)), "Parallel, not collinear"}};

    for (const auto& test : tests)
    {
        std::cout << test.description << ": ";
        auto res = Intersection3DSegments(test.segment1, test.segment2);
        PrintVector(res.value().intersection_point);
        std::cout << std::endl;
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

/// @brief Calculate solutuin using Cramer formula
/// @param A matrix f
/// @param b right part
/// @return solution of linear system equations
std::optional<Eigen::Vector2d> CramerFormula(const Eigen::Matrix2d& A, const Eigen::Vector2d& b)
{
    double detA = A.determinant();

    if (!std::abs(detA))
        return std::nullopt;  // система вырожденная

    Eigen::Matrix2d Ax = A;
    Eigen::Matrix2d Ay = A;

    Ax.col(0) = b;
    Ay.col(1) = b;

    double s = Ax.determinant() / detA;
    double t = Ay.determinant() / detA;

    return Eigen::Vector2d(s, t);
}

/// @brief Finding intersection of two segments
std::optional<ReturnValueIntersection> Intersection3DSegments(const Segment3D& segment1,
                                                              const Segment3D& segment2)
{
    Vector3D segment1_start = segment1.GetStart();
    Vector3D segment1_end = segment1.GetEnd();
    if (segment1_start == segment1_end)
        throw std::invalid_argument("Start and end for first segment are equal.");

    Vector3D segment2_start = segment2.GetStart();
    Vector3D segment2_end = segment2.GetEnd();
    if (segment2_start == segment2_end)
        throw std::invalid_argument("Start and end for second segment are equal.");

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
            return std::nullopt;  /// No complanar
        else
        {
            // Eigen::MatrixXd A(3, 2);
            // A.col(0) << vector_segment1.GetX(), vector_segment1.GetY(), vector_segment1.GetZ();
            // A.col(1) << -vector_segment2.GetX(), -vector_segment2.GetY(),
            // -vector_segment2.GetZ();

            Eigen::MatrixXd A(2, 2);
            A.col(0) << vector_segment1.GetX(), vector_segment1.GetY();
            A.col(1) << -vector_segment2.GetX(), -vector_segment2.GetY();

            // Eigen::VectorXd b(3);
            // b << (segment2_start - segment1_start).GetX(), (segment2_start -
            // segment1_start).GetY(),
            //     (segment2_start - segment1_start).GetZ();

            Eigen::VectorXd b(2);
            b << (segment2_start - segment1_start).GetX(), (segment2_start - segment1_start).GetY();

            // Eigen::Matrix2d AtA = A.transpose() * A;
            // Eigen::Vector2d Atb = A.transpose() * b;
            // auto sol = CramerFormula(AtA, Atb);
            auto x = CramerFormula(A, b);

            double s = (*x)(0);
            double t = (*x)(1);

            Vector3D segment1_point = segment1_start + vector_segment1 * s;
            Vector3D segment2_point = segment2_start + vector_segment2 * t;
            // if ((segment1_point - segment2_point).LengthVector())
            //     return std::nullopt;  /// there is no intersection,
            //                           /// there is a point close to two lines

            if (s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0)
            {
                ReturnValueIntersection result;
                result.intersection_point = segment1_start + vector_segment1 * s;
                return result;
            }  /// intersection point
            return std::nullopt;  /// lines are intersected, but segments are not
        }
    }
    /// Colinear cases
    else
    {
        if (length_w1)
            return std::nullopt;  /// Parallel, not collinear

        auto non_zero_component_u =
            FirstNonZero(vector_segment1.GetX(), vector_segment1.GetY(), vector_segment1.GetZ());
        if (!non_zero_component_u)
            return std::nullopt;

        double t0 = (segment2_start.GetX() - segment1_start.GetX()) / non_zero_component_u.value();
        double t1 = (segment2_end.GetX() - segment1_start.GetX()) / non_zero_component_u.value();

        double t_start = std::max(0.0, std::min(t0, t1));
        double t_end = std::min(1.0, std::max(t0, t1));

        if (t_start < t_end)
        {
            ReturnValueIntersection result;
            result.intersection_segment = Segment3D(segment1_start + vector_segment1 * t_start,
                                                    segment1_start + vector_segment1 * t_end);
            return result;
        }  /// intersection is segment
        if (t_start == t_end)
        {
            ReturnValueIntersection result;
            result.intersection_point = segment1_start + vector_segment1 * t_start;
            return result;
        }  /// intersection is point, end of one of the segment
        return std::nullopt;
    }
};
