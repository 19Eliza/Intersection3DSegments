#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "vector3d.h"

class Segment3D
{
   private:
    Vector3D start;  /// start point of segment
    Vector3D end;    /// end point of segment

   public:
    Segment3D();
    Segment3D(const Vector3D& start1, const Vector3D& end1);
    Segment3D(const Segment3D& new_segment3d);

    /// @brief get end and begin of segment
    Vector3D GetStart() const noexcept;
    Vector3D GetEnd() const noexcept;

    /// @brief check validation of segment
    void CheckSegmentValid() const noexcept;

    friend bool IsNonColinear(const Segment3D& segment1, const Segment3D& segment2);
    friend std::vector<Vector3D> IntersectionOfCoplanarNonColinear(const Segment3D& segment1,
                                                                   const Segment3D& segment2);

    friend std::vector<Vector3D> IntersectionOfColinear(const Segment3D& segment1,
                                                        const Segment3D& segment2);

    /// @brief input and output function
    friend std::ostream& operator<<(std::ostream& os, const Segment3D& s);

    friend std::istream& operator>>(std::istream& is, Segment3D& s);

    ~Segment3D() = default;
};