#pragma once

#include <Geometry/primitives/line.h>
#include <Geometry/cgal_predicates.h>

namespace cg
{
   inline bool isTripleConvex( cg::point_2 const &v1, cg::point_2 const &v2, cg::point_2 const &v3 )
   {
      return cg::le(((v3 - v2) ^ (v2 - v1)), 0, 1e-8);
   }

   inline point_2 calculateBisector( cg::point_2 const &prev, cg::point_2 const &cur, cg::point_2 const &next,
                                     bool adjustLength = true, bool convex = false, bool assureConvexity = false )
   {
      double nextNorm = cg::norm(next - cur);
      double prevNorm = cg::norm(prev - cur);
      cg::point_2 nextNormalized = (next - cur) / nextNorm;
      cg::point_2 prevNormalized = (prev - cur) / prevNorm;

      point_2 bisector = prevNormalized + nextNormalized;
      double bisectorNorm = cg::norm(bisector);

      if (cg::eq_zero(bisectorNorm, 1e-6))
      {
         bisector = cg::normalized(cg::normal(nextNormalized));
         adjustLength = false;
         assureConvexity = false;
         convex = true;
      }
      else
         bisector /= bisectorNorm;

      if (!convex)
      {
         if (!cg::le((nextNormalized ^ (-prevNormalized)), 0, 1e-8)) // if (!isTripleConvex(prev, cur, next))
            bisector = -bisector;
      }

      if (adjustLength)
      {
         // Adjust bisector length
         double divisor = nextNormalized ^ bisector;
         if (cg::ge(divisor, 0) && !cg::eq_zero(divisor, 1e-7))
            bisector /= cg::abs(divisor);
      }

      if (assureConvexity)
      {
         if (!cg::ge(nextNormalized ^ bisector, 0, 1e-4) || !cg::ge((-prevNormalized) ^ bisector, 0, 1e-4))
            bisector = -bisector;
      }

      Assert(!_isnan(bisector.x) && !_isnan(bisector.y));
      return bisector;
   }

   inline bool rayLineIntersection( cg::line_2 const &ray, cg::line_2 const &line, cg::point_2 &out )
   {
      double t ; 
      if (has_intersection(ray, line, &t) && cg::ge(t, 0))
      {
         out = ray(t) ; 
         return true;
      }

      return false;
   }

   inline bool robust_rayLineIntersection( cg::line_2 const &ray, cg::line_2 const &line, cg::point_2 &out )
   {
      if (has_intersection(ray, line, out))
         return cg::ge(ray(out), 0);

      return false;
   }

   inline bool robust_has_intersection( cg::line_2 const &lineA, cg::line_2 const &lineB, cg::point_2 &out )
   {
      typedef details::CGAL_Kernel::Exact_kernel ExactKernel;
      CGAL::Cartesian_converter<details::CGAL_Kernel, ExactKernel> c;
      CGAL::Cartesian_converter<ExactKernel, details::CGAL_Kernel> cc;

      CGAL::Object result = ExactKernel().intersect_2_object()(c(details::construct_cgal_line(lineA)),
         c(details::construct_cgal_line(lineB)));
      ExactKernel::Point_2 ipoint;
      if (CGAL::assign(ipoint, result))
      {
         details::CGAL_Kernel::Point_2 cipoint (cc(ipoint));
         out = cg::point_2 (cipoint.x(), cipoint.y());
         return true;
      }

      return false;
   }

} // End of 'cg' namespace
