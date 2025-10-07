#pragma once

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
};