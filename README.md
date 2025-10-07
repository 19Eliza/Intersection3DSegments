# Intersection3DSegments
Finding the intersection of two Segment3D.

This task have several cases.

1. Non coplanar segments 
   - Define segment vectors:
       u = end of segment 1 - start of segment 1
       v = end of segment 2 - start of segment 2
       w0 = start of segment 1 - start of segment 2
   - Check if segments lie in the same plane using the product:
       w0 · (u × v)
   - If the result is non zero, the segments are non coplanar — no intersection.

2. Coplanar segments
   - Segments lie in the same plane (w0 · (u × v) = 0).
   - To find intersection, solve the linear system (Cramer's method).
   - If the solution 0 ≤ s ≤ 1 and 0 ≤ t ≤ 1 , the segments intersect.
   - Possible cases:
       - Intersection at a point
       - Intersection is a segment
       - No intersection

3. Collinear segments
   - Cross product u × v = 0.
   - If additionally the start points are collinear, check for overlap: 
       - Fully overlapping
       - Partial overlap 
       - No overlap