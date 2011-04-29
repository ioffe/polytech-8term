#pragma once

#include "obb_fwd.h"
#include "obb_intersect.h"
#include "triangle_3_fast.h"

namespace cg
{

typedef ::OBB_INTERSECT_DETAIL::POINT_DETAIL Detail;

namespace sat
{

template<class Point>
inline bool separate_axis(OBB_t<Point> const& obbA, OBB_t<Point> const& obbB, Point const& axis, double &depth)
{
   typedef typename Point::scalar_type Scalar;
   Scalar const ARad = cg::abs(obbA.extents(0) * axis) + cg::abs(obbA.extents(1) * axis) + cg::abs(obbA.extents(2) * axis);
   Scalar const BRad = cg::abs(obbB.extents(0) * axis) + cg::abs(obbB.extents(1) * axis) + cg::abs(obbB.extents(2) * axis);
   Scalar const D    = cg::abs((obbB.origin - obbA.origin) * axis);

   if (D > ARad + BRad)
      return true;

   depth = ARad + BRad - D;

   return false;
}

template<class Point>
inline range_t<typename Point::scalar_type> obb_to_axis(OBB_t<Point> const &obb, Point const &axis)
{
   typedef typename Point::scalar_type Scalar;
   Scalar halfLength = cg::abs(obb.extents(0) * axis) + cg::abs(obb.extents(1) * axis) + cg::abs(obb.extents(2) * axis);
   Scalar center = obb.origin * axis;

   return range_t<Scalar>(center - halfLength, center + halfLength );
}

template<class Point>
inline range_t<typename Point::scalar_type> tri_to_axis(triangle_3_fast<Point>const &tri, Point const &axis)
{
   typedef typename Point::scalar_type Scalar ; 
   Scalar d1 = tri.v1() * axis;
   Scalar d2 = tri.v2() * axis;
   Scalar d3 = tri.v3() * axis;

   return range_t<Scalar>(d1, d2) | d3 ;
}

template<class Point>
inline bool axis_intersect(OBB_t< Point > const &obb, triangle_3_fast< Point > const &tri, Point const &axis, Point &mtd,
                           range_t<typename Point::scalar_type> &cross)
{
   typedef typename Point::scalar_type Scalar;
   typedef range_t<Scalar>             Range;

   if(eq_zero(axis, Scalar(1.e-4)))
      return true;

   Point sepAxis = (axis * tri.normal() < Scalar(0.)) ? -axis : axis;

   Range a(obb_to_axis(obb, sepAxis));
   Range b(tri_to_axis(tri, sepAxis));

   bool intersected = cg::has_intersection(a, b);

   if(intersected)
   {
      Scalar const halfSize = Scalar(0.5) * a.size();
      Scalar const ac       = a.lo() + halfSize;

      Range mtdRange(b);
      mtdRange.inflate(halfSize);
      mtdRange -= ac;

      sepAxis *= ((mtdRange.lo() > Scalar(0.)) ? mtdRange.lo() : mtdRange.hi()) / norm_sqr(axis);

      if(norm_sqr(sepAxis) < norm_sqr(mtd))
      {
         mtd = sepAxis;
         cross = (Scalar(1.) / norm(axis)) * (a & b);
      }
   }

   return intersected;
}

template<class Point>
inline bool separate_axes(OBB_t<Point> const &obb, triangle_3_fast<Point> const &tri, Point &mtd,
                          range_t<typename Point::scalar_type> &cross)
{
   // Check triangle normal
   if(!axis_intersect(obb, tri, tri.normal(), mtd, cross))
      return false;

   // Check OBB faces normals
   for(size_t i = 0; i < 3; ++i)
   {
      if(!axis_intersect(obb, tri, obb.extents(i)                        , mtd, cross) ||
         !axis_intersect(obb, tri, obb.extents(i) ^ (tri.v2() - tri.v1()), mtd, cross) ||
         !axis_intersect(obb, tri, obb.extents(i) ^ (tri.v3() - tri.v2()), mtd, cross) ||
         !axis_intersect(obb, tri, obb.extents(i) ^ (tri.v1() - tri.v3()), mtd, cross))
         return false;
   }

   return true;
}

template<class Point, size_t N>
inline bool get_support_vertices(OBB_t<Point> const &obb, Point const &axis, vector_fixed_capacity<Point, N> &points)
{
   typedef typename Point::scalar_type Scalar;

   Assert(eq(norm(axis), Scalar(1.), Scalar(1.e-5)));

   // Axes mask
   static size_t const allAxes = OBB_INTERSECT_DETAIL::AXIS_X | 
                                 OBB_INTERSECT_DETAIL::AXIS_Y | 
                                 OBB_INTERSECT_DETAIL::AXIS_Z;
   Scalar d[3];

   // Check for supporting faces
   static Scalar const faceE = Scalar(1.e-3);

   for(size_t i = 0; i < 3; ++i)
   {
      d[i] = axis * obb.extents(i) / norm(obb.extents(i));

      if(ge(cg::abs(d[i]), Scalar(1.), faceE))
      {
         size_t const knownAxis = (d[i] < 0. ? 1 : 0) << i;
         size_t const shift2 = (i + 1) % 3;
         size_t const shift3 = (i + 2) % 3;

         points.push_back(vertex(obb, allAxes & (knownAxis | (0 << shift2) | (0 << shift3))));
         points.push_back(vertex(obb, allAxes & (knownAxis | (1 << shift2) | (0 << shift3))));
         points.push_back(vertex(obb, allAxes & (knownAxis | (1 << shift2) | (1 << shift3)))); 
         points.push_back(vertex(obb, allAxes & (knownAxis | (0 << shift2) | (1 << shift3))));
         return true;
      }
   }

   // Check for supporting edges
   static Scalar const edgeE = Scalar(2.6e-3);

   for(size_t i = 0; i < 3; ++i)
   {
      if(le(cg::abs(d[i]), Scalar(0.), edgeE)) // 
      {
         size_t const knownAxes = (d[(i + 1) % 3] < 0. ? 1 : 0) << ((i + 1) % 3) |
            (d[(i + 2) % 3] < 0. ? 1 : 0) << ((i + 2) % 3);

         points.push_back(vertex(obb, allAxes & (knownAxes | (0 << i))));
         points.push_back(vertex(obb, allAxes & (knownAxes | (1 << i))));
         return true;
      }
   }

   // Check for supporting vertex
   size_t const knownAxes = (d[0] < 0. ? 1 : 0) << 0 |
                            (d[1] < 0. ? 1 : 0) << 1 | 
                            (d[2] < 0. ? 1 : 0) << 2;
   points.push_back(vertex(obb, allAxes & knownAxes));

   return true;
}

template<class Point, size_t N>
inline bool get_support_vertices(triangle_3_fast<Point> const &tri, Point const &axis, vector_fixed_capacity<Point, N> &points, 
                                 range_t<typename Point::scalar_type> const &cross)
{
   typedef typename Point::scalar_type Scalar;

   Assert(eq(norm(axis), Scalar(1.), Scalar(1.e-5)));

   // Check for supporting faces
   static Scalar const faceE = Scalar(2.e-3);

   Scalar n = axis * tri.normal();

   if(ge(cg::abs(n), Scalar(1.), faceE))
   {
      points.push_back(tri.v1());
      points.push_back(tri.v2());
      points.push_back(tri.v3());
      return true;
   }

   // Check for supporting edges
   static Scalar const edgeE  = Scalar(2.6e-3);
   static Scalar const rangeE = Scalar(1.e-3);

   Scalar d[3];
   
   Point e1 = tri.v2() - tri.v1();
   normalize(e1);
   d[0] = tri.v1() * axis;
   if(le(cg::abs(e1 * axis), Scalar(0.), edgeE) && cross.contains(d[0], rangeE)) // 
   {
      points.push_back(tri.v1());
      points.push_back(tri.v2());
      return true;
   }

   Point e2 = tri.v3() - tri.v2();
   normalize(e2);
   d[1] = tri.v2() * axis;
   if(le(cg::abs(e2 * axis), Scalar(0.), edgeE) && cross.contains(d[1], rangeE)) // 
   {
      points.push_back(tri.v2());
      points.push_back(tri.v3());
      return true;
   }

   Point e3 = tri.v1() - tri.v3();
   normalize(e3);
   d[2] = tri.v3() * axis;
   if(le(cg::abs(e3 * axis), Scalar(0.), edgeE) && cross.contains(d[2], rangeE)) // 
   {
      points.push_back(tri.v3());
      points.push_back(tri.v1());
      return true;
   }

   // Check for supporting vertices within overlapping range
   Scalar min = d[0];
   size_t idx = cross.contains(d[0], rangeE) ? 0 : 4;
   for(size_t i = 1; i < 3; ++i)
   {
      if((min < d[i]) && (cross.contains(d[i], rangeE)))
      {
         min = d[i];
         idx = i;
      }
   }

   if(idx < 3)
      points.push_back(tri[idx]);

   return idx < 3;
}

namespace clip
{

template<class Point, size_t N1, size_t N2>
bool byPlane(vector_fixed_capacity<Point, N1> const &in, vector_fixed_capacity<Point, N2> &out, Point const &n, typename Point::scalar_type d)
{
   typedef typename Point::scalar_type Scalar;

   bool isBack[N1];
   bool hasBackVertices  = false;
   bool hasFrontVertices = false;

   for(size_t i = 0, size = in.size(); i < size; ++i) 
   {
      isBack[i] = (in[i] * n - d < 0.) ? true : false;

      hasBackVertices  = hasBackVertices  || isBack[i];
      hasFrontVertices = hasFrontVertices || !isBack[i];
   }

   if(!hasBackVertices)
   {
      out.clear();
      return false;
   }

   if(!hasFrontVertices) 
   {
      out = in;
      return true;
   }

   // Process polygon edge vs clipper intersection
   out.clear();
   size_t maxSize = (in.size() > 2) ? in.size() : 1;

   for(size_t size = in.size(), i = size - 1, j = 0; j < size; i = j++) 
   {
      if(isBack[i]) 
      {
         if(out.full()) 
            return true;

         out.push_back(in[i]);
      }

      // When we clip edge with two vertices
      // First is front and second is back, we clip it
      // but do not add second vertex to clipped polygon.
      // Following should fix it.
      if(j >= maxSize) 
         break;

      if(isBack[j] ^ isBack[i]) 
      {
         if(out.full())
            return true;

         Point  edge = (in[j] - in[i]);

         Scalar den = edge * n;
         if(eq_zero(den)) // Numerical problem
            return true;

         Scalar t = (d - in[i] * n) / den;
         out.push_back(in[i] +  edge * t);
      }
   }

   return true;
}

template<class Point, size_t N1, size_t N2, size_t N3>
bool byPolygon(vector_fixed_capacity<Point, N1> const &clipper, vector_fixed_capacity<Point, N2> const &poly, vector_fixed_capacity<Point, N3> &patch)
{
   Assert(clipper.size() > 2);
   if(clipper.size() <= 2)
      return false;

   Point n = normalized_safe((clipper[1] - clipper[0]) ^ (clipper[2] - clipper[1]));

   vector_fixed_capacity<Point, N3> tmp(poly);
   for(size_t size = clipper.size(), i = size - 1, j = 0; j < size; i = j++) 
   {
      Point tmpN((clipper[j] - clipper[i]) ^ n);

      if(!byPlane(tmp, patch, tmpN, clipper[i] * tmpN)) 
         return false;

      tmp = patch;
   }

   return true;
}

} // end of namespace clip

namespace contacts
{

template<class Point>
Point nearest(Point const &a, Point const &b0, Point const &b1)
{
   typedef typename Point::scalar_type Scalar;

   Point bd  = b1 - b0;
   if(eq_zero(bd))
      return a;

   Point b0a = a - b0;
   Scalar t = cg::bound<Scalar>((b0a * bd) / (bd * bd), 0, 1);
   return b0 + t * bd;
}

template<class Point, size_t N>
void point_edge(Point const &a, Point const &b0, Point const &b1, vector_fixed_capacity<Detail, N> &points, Point const &axis, Point const &origin)
{
   if(points.full())
      return;

   Point const p = nearest(a, b0, b1);
   points.push_back(Detail(p - origin, axis, cg::distance(a, p)));
}

template<class Point, size_t N1, size_t N2>
void point_face(Point const &a, vector_fixed_capacity<Point, N1> const &face, vector_fixed_capacity<Detail, N2> &points, Point const &axis, bool pointOnFace, Point const &origin)
{
   if(points.full())
      return;

   typedef typename Point::scalar_type Scalar;

   Point const x(face[1] - face[0]);
   Point const y(face[2] - face[1]);

   Point const n = normalized_safe(x ^ y);
   
   Scalar dist = (a - face[0]) * n;

   double signDist = cg::sign(axis * n) * dist;
   if((pointOnFace && (signDist < 0.)) || (!pointOnFace && (signDist > 0.)))
      return;

   Scalar absDist = cg::abs(dist);

   Point pnt(a);
   if(pointOnFace)
   {
      pnt -= dist * n;
      
// There is possibility that pnt is in face's plane but outside face poly
// so commented code is trying to fix this problem but surely it's slower
// than uncommented one

//       // Build in plane cs (x, y)
//       y = n ^ x;
//       // Project points on (x, y)
//       size_t size = face.size();
// 
//       typedef cg::point_t<Scalar, 2> Point2;
// 
//       vector_fixed_capacity<Point2, N1> projectedFace;
//       for(size_t i = 0; i < size; ++i)
//       {
//          projectedFace.push_back(Point2(face[i] * x, face[i] * y));
//       }
// 
//       Point2 projectedA(a * x, a * y);
// 
//       if(!cg::is_inside_poly(projectedA, projectedFace.begin(), projectedFace.size()))
//       { // Point is not on face
//          pnt = Point();
//          for(size_t i = 0, size = face.size(); i < size; ++i)
//          {
//             pnt += face[i];
//          }
//          pnt /= Scalar(face.size());
//       }
//       for(size_t i = 0, size = face.size(); (i < size) && !points.full(); ++i)
//          points.push_back(Detail(face[i], axis, absDist));
// 
//       return;
   }

   points.push_back(Detail(pnt - origin, axis, absDist));
}

template<class Point, size_t N>
void edge_edge(Point const &a0, Point const &a1, Point const &b0, Point const &b1, vector_fixed_capacity<Detail, N> &points, Point const &axis, Point const &origin, OBB_INTERSECT_DETAIL *detail)
{
   if(points.full())
      return;

   typedef typename Point::scalar_type Scalar;

   Point ad = a1 - a0;
   Point bd = b1 - b0;
   Point n  = ad ^ bd;

   Point m = n ^ bd;
   Scalar md = m * b0;

   Scalar den = ad * m;
   if(eq_zero(den))
      return;

// Debug
// 
   detail->obbDetailType = OBB_INTERSECT_DETAIL::EDGE;
   detail->triDetailType = OBB_INTERSECT_DETAIL::EDGE;

   detail->triDetail[0] = a0 - origin;
   detail->triDetail[1] = a1 - origin;

   detail->obbDetail[0] = b0 - origin;
   detail->obbDetail[1] = b1 - origin;
// 
// end of Debug

   Scalar t = cg::bound<Scalar>((md - a0 * m) / den, 0, 1);

   Point const pa = a0 + t * ad;
   Point const pb = nearest(pa, b0, b1);

   points.push_back(Detail(pb - origin, axis, cg::distance(pa, pb)));
}

template<class Point, size_t N1, size_t N2, size_t N3>
bool polygon(vector_fixed_capacity<Point, N1> const &clipper, vector_fixed_capacity<Point, N2> const &poly, vector_fixed_capacity<Detail, N3> &points, Point const &axis, bool pointsOnClipper, Point const &origin)
{
   if(points.full())
      return false;

   typedef typename Point::scalar_type Scalar;
   vector_fixed_capacity<Point, OBB_INTERSECT_DETAIL::SIZE> patch;
   
   if(!clip::byPolygon(clipper, poly, patch))
      return false;

   for(size_t i = 0, size = patch.size(); i < size; i ++)
      point_face(patch[i], clipper, points, axis, pointsOnClipper, origin);

   return (0 != patch.size());
}

} // end of namespace contacts
} // end of namespace sat

template<class Point>
inline bool has_intersection(OBB_t<Point> const& obbA, OBB_t<Point> const& obbB, double &depth)
{
   // TODO : Make real depth.
   for( size_t i = 0; i != 3; ++i )
   {
      if(sat::separate_axis(obbA, obbB, obbA.extents( i ), depth)) 
         return false;
      
      if(sat::separate_axis(obbA, obbB, obbB.extents( i ), depth)) 
         return false;

      for( size_t j = 0; j != 3; ++j )
         if(sat::separate_axis(obbA, obbB, obbA.extents( i ) ^ obbB.extents( j ), depth)) 
            return false;
   }

   return true;
}

template<class Point>
inline bool detect_intersection(OBB_t<Point> const &obb, triangle_3_fast<Point> const &tri, OBB_INTERSECT_DETAIL *detail)
{
   // Check if detail is already filled
   if(detail && detail->points.full())
      return false;

   if(eq_zero(obb.extents(0)) || eq_zero(obb.extents(1)) || eq_zero(obb.extents(2)))
      return false;

   // Check if triangle is valid
   if(eq_zero((tri.v2() - tri.v1()) ^ (tri.v3() - tri.v1())))
      return false;

   typedef typename Point::scalar_type  Scalar;
   typedef range_t<Scalar>              Range;

   static Scalar const m = 1.e+10;
   Point mtd(m, m, m); // Minimal Translation Distance

   Range cross;
   if(!sat::separate_axes(obb, tri, mtd, cross))
      return false;

   if(!detail)
      return !eq_zero(mtd);

   // Deduce supporting vertices
   Scalar depth = cg::norm_sqr(mtd);
   if(eq_zero(depth))
      return false;

   depth = ::cg::sqrt(depth);
   Point axis = mtd / depth;

   vector_fixed_capacity<Point, 4> verticesOBB;
   if(!sat::get_support_vertices(obb, axis, verticesOBB))
      return false;

   vector_fixed_capacity<Point, 3> verticesTri;
   if(!sat::get_support_vertices(tri, axis, verticesTri, cross))
      return false;

   // Construct contact patch
   if(1 == verticesOBB.size())
      detail->points.push_back(Detail(verticesOBB[0] - obb.origin, axis, depth));
   else if(2 == verticesOBB.size())
   {
      switch(verticesTri.size())
      {
         case 1: 
         {
            sat::contacts::point_edge(verticesTri[0], verticesOBB[0], verticesOBB[1], detail->points, axis, obb.origin); 
            break;
         }
         case 2: 
         {
            sat::contacts::edge_edge(verticesTri[0], verticesTri[1], verticesOBB[0], verticesOBB[1], detail->points, axis, obb.origin, detail);
            break;
         }
         default: 
         {
            Assert(3 == verticesTri.size());
            return sat::contacts::polygon(verticesTri, verticesOBB, detail->points, axis, false, obb.origin); 
         }
      }
   }
   else // 4 == numVerticesOBB
   {
      Assert(4 == verticesOBB.size());
      if(1 == verticesTri.size()) 
      {
         sat::contacts::point_face(verticesTri[0], verticesOBB, detail->points, axis, true, obb.origin);
      }
      else
      {
         return sat::contacts::polygon(verticesOBB, verticesTri, detail->points, axis, true, obb.origin);
      }
   }

   return true;
}

} // end of namespace cg
