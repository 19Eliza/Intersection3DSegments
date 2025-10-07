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