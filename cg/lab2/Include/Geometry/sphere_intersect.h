#pragma once

#include "Geometry/vector_fixed_capacity.h"
#include "Geometry/primitive_intersect.h"

#pragma pack(push,1)
struct SPHERE_INTERSECT_DETAIL
{
   SPHERE_INTERSECT_DETAIL() {}

   static const size_t SIZE = 16;

   typedef PRIMITIVE_POINT_DETAIL   POINT_DETAIL;
   typedef POINT_DETAIL::point_type point_type;

   typedef cg::vector_fixed_capacity<POINT_DETAIL, SIZE> Points;

   Points points;
};

#pragma pack(pop)


namespace cg
{
   namespace details
   {
      // Решает квадратное ур-е
      //   при a > 0 t = наименьший корень из отрезка [0, 1]
      //   при a < 0 t = наибольший корень из отрезка [0, 1] (не используется т.к. a = dir^2)
      //   при a = 0 assertion
      template< class S >
         bool solve2Equation( S a, S b, S c, S &t, S* t2 )
      {
         S d = b * b - 4 * a * c;

         Assert(!eq_zero(a)) ;
         if (d < 0)
            return false;

         d = cg::sqrt(d);

         S t0 = (-b - d) / (2 * a);
         S t1 = (-b + d) / (2 * a);

         if (0 <= t0 && t0 <= 1)
         {
            t = t0;
            if (t2)
               *t2 = t1;
            return true;
         }
         else if (0 <= t1 && t1 <= 1)
         {
            t = t1;
            if (t2)
               *t2 = t0;
            return true;
         }

         return false;
      }

   }

   template< class S, size_t Dim >
      bool ray_intersection( point_t< S, Dim > const& p0, point_t< S, Dim > const& p1, sphere_t< S, Dim > const& sph, S & ratio, S * ratio_next = NULL )
   {
      typedef point_t< S, Dim > point_type; 

      point_type dir  = p1 - p0;
      point_type move = p0 - sph.center;

      S a = cg::norm_sqr(dir);
      S c = cg::norm_sqr(move) - sph.radius * sph.radius;

      if(cg::eq_zero(a))
      {
         ratio = 0.;
         return cg::eq_zero(c);
      }

      S b = 2 * (dir * move);

      return details::solve2Equation(a, b, c, ratio, ratio_next);
   }

   template< class S, size_t Dim >
      point_3 CalcNormal( point_t< S, Dim > const& p, sphere_t< S, Dim > const& sph )
   {
      return cg::normalized_safe( p - sph.center ) ;
   }
}
