#include "segment3d.h"

Segment3D::Segment3D() : start(), end() {};

Segment3D::Segment3D(const Vector3D& start1, const Vector3D& end1) : start(start1), end(end1) {};

Segment3D::Segment3D(const Segment3D& new_segment3d)
{
    start = new_segment3d.start;
    end = new_segment3d.end;
}

Vector3D Segment3D::GetStart() const noexcept { return start; }
Vector3D Segment3D::GetEnd() const noexcept { return end; }

void Segment3D::CheckSegmentValid() const noexcept
{
    if (start == end)
        throw std::invalid_argument("Segment start and end cannot be equal");
}

bool IsNonColinear(const Segment3D& segment1, const Segment3D& segment2)
{
    Vector3D segment1_start = segment1.GetStart();
    Vector3D segment1_end = segment1.GetEnd();
    Vector3D segment2_start = segment2.GetStart();
    Vector3D segment2_end = segment2.GetEnd();
    Vector3D vector_segment1 = segment1_end - segment1_start;
    Vector3D vector_segment2 = segment2_end - segment2_start;

    auto cross_segments12 = vector_segment1.Cross(vector_segment2);
    double length_cross_segments12 = cross_segments12.LengthVector();

    return length_cross_segments12 != 0.0;
}

std::vector<Vector3D> IntersectionOfCoplanarNonColinear(const Segment3D& segment1,
                                                        const Segment3D& segment2)
{
    Vector3D segment1_start = segment1.GetStart();
    Vector3D segment1_end = segment1.GetEnd();

    Vector3D segment2_start = segment2.GetStart();
    Vector3D segment2_end = segment2.GetEnd();

    Vector3D vector_segment1 = segment1_end - segment1_start;
    Vector3D vector_segment2 = segment2_end - segment2_start;

    auto cross_segments12 = vector_segment1.Cross(vector_segment2);
    double length_cross_segments12 = cross_segments12.LengthVector();

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
        b << (segment2_start - segment1_start).GetX(), (segment2_start - segment1_start).GetY();
    }
    else
    {
        if (det_xz)
        {
            A.col(0) << vector_segment1.GetX(), vector_segment1.GetZ();
            A.col(1) << -vector_segment2.GetX(), -vector_segment2.GetZ();
            b << (segment2_start - segment1_start).GetX(), (segment2_start - segment1_start).GetZ();
        }
        else
        {
            A.col(0) << vector_segment1.GetY(), vector_segment1.GetZ();
            A.col(1) << -vector_segment2.GetY(), -vector_segment2.GetZ();
            b << (segment2_start - segment1_start).GetY(), (segment2_start - segment1_start).GetZ();
        }
    }

    /// @brief Solve system linear equations using Cramer formula
    /// @param A matrix of linear system equations
    /// @param b right part
    /// @return solution of linear system equations
    auto CramerFormula = [](const Eigen::Matrix2d& A,
                            const Eigen::Vector2d& b) -> std::optional<Eigen::Vector2d>
    {
        double detA = A.determinant();
        if (std::abs(detA) == 0.0)
            throw std::invalid_argument("Determinant is zero.");

        Eigen::Matrix2d Ax = A, Ay = A;
        Ax.col(0) = b;
        Ay.col(1) = b;

        return Eigen::Vector2d(Ax.determinant() / detA, Ay.determinant() / detA);
    };

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

std::vector<Vector3D> IntersectionOfColinear(const Segment3D& segment1, const Segment3D& segment2)
{
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

    if (length_w1)
        return {};  /// Parallel, not lies on one line

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

std::ostream& operator<<(std::ostream& os, const Segment3D& s)
{
    return os << "[" << s.start << "," << s.end << "]";
}

std::istream& operator>>(std::istream& is, Segment3D& p) { return is >> p.start >> p.end; }