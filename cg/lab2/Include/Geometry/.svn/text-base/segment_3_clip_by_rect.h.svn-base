#pragma once

#include "Primitives\segment.h"
#include "Primitives\rectangle.h"

namespace cg
{

template< class Scalar >
   range_t< Scalar > clip( segment_t< Scalar, 3 > const & seg,
                           rectangle_t< Scalar, 3 > const & rect, Scalar eps )
{
   typedef range_t< Scalar > range_type;
   typedef segment_t< Scalar, 3 > segment_type;
   typedef segment_type::point_type point_type;
   typedef rectangle_t< Scalar, 3 > rectangle_type;

   range_type result(0, 1);
   point_type p0 = seg.P0(), p1 = seg.P1();

   for (size_t i = 0; i < rectangle_type::dimension; i++)
   {
      Scalar di = p1[i] - p0[i];

      if (cg::eq_zero(di, eps))
      {
         if (p0[i] < rect[i].lo() && p1[i] < rect[i].lo() ||
             p0[i] > rect[i].hi() && p1[i] > rect[i].hi())
         {
            return range_type();
         }
         continue;
      }

      Scalar inv_di = 1 / di;
      range_type cur_range((rect[i].lo() - p0[i]) * inv_di,
                           (rect[i].hi() - p0[i]) * inv_di);

      result &= cur_range;
      if (result.empty())
         return result;
   }

   return result;
}

template< class Scalar >
   range_t< Scalar > clip( segment_t< Scalar, 3 > const & seg, rectangle_t< Scalar, 3 > const & rect )
{
   return clip(seg, rect, cg::epsilon< Scalar >());
}

//////////////////////////////////////////////////////////////////////////

template< class Scalar >
   bool clip( segment_t< Scalar, 3 > const & seg, rectangle_t< Scalar, 3 > const & rect,
              segment_t< Scalar, 3 > & result, Scalar eps )
{
   range_t< Scalar > t_res = clip(seg, rect, eps);
   if (t_res.empty())
      return false;
   result = segment_t< Scalar, 3 >(seg(t_res.lo()), seg(t_res.hi()));
   return true;
}

template< class Scalar >
   bool clip( segment_t< Scalar, 3 > const & seg, rectangle_t< Scalar, 3 > const & rect,
              segment_t< Scalar, 3 > & result )
{
   return clip(seg, rect, result, cg::epsilon< Scalar >());
}

} // End of 'cg' namespace
