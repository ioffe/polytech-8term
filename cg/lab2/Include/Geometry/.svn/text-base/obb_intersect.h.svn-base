#pragma once

#include "Geometry/primitives/rectangle.h"
#include "Geometry/obb.h"
#include "Geometry/vector_fixed_capacity.h"

#include "Geometry/primitive_intersect.h"

#pragma pack(push,1)
struct OBB_INTERSECT_DETAIL
{
   OBB_INTERSECT_DETAIL() {}

   OBB_INTERSECT_DETAIL( long type, bool fixed )
   {
      for ( size_t i = 0; i != SIZE; ++i )
      {
         points[i].type  = type;
         points[i].fixed = fixed;
      }
      Assert(0 == points.size());
   }

   enum 
   {
      AXIS_X = 1 << 0,
      AXIS_Y = 1 << 1,
      AXIS_Z = 1 << 2
   };

   static const size_t SIZE = /*26*/8 + 6;

   typedef PRIMITIVE_POINT_DETAIL   POINT_DETAIL;
   typedef POINT_DETAIL::point_type point_type;

   typedef cg::vector_fixed_capacity<POINT_DETAIL, SIZE> Points;

   Points points;
// 
// // Debug detail
   enum Detail
   {
      VERTEX = 0,
      EDGE   = 1,
      FACE   = 2
   };

   Detail obbDetailType;
   Detail triDetailType;

   cg::point_3 obbDetail[3];
   cg::point_3 triDetail[3];
// 
// // Debug detail
} ;

#pragma pack(pop)

// inline void add_detail( OBB_INTERSECT_DETAIL & to, OBB_INTERSECT_DETAIL const & from )
// {
//    for ( size_t i = 0, size = from.points.size(); (i < size) && !to.points.full(); ++i )
//    {
//       to.points.push_back(from.points[i]);
//    }
// }

inline cg::point_3 vertex( cg::rectangle_3 const & rc, size_t idx )
{
   cg::point_3 res = rc.xyz();
   if ( idx & OBB_INTERSECT_DETAIL::AXIS_X )
      res.x = rc.x.hi(); 

   if ( idx & OBB_INTERSECT_DETAIL::AXIS_Y )
      res.y = rc.y.hi(); 

   if ( idx & OBB_INTERSECT_DETAIL::AXIS_Z )
      res.z = rc.z.hi(); 

   return res;
}

template<class Point>
inline cg::point_3 vertex( cg::OBB_extents_t< Point > const & ext, size_t idx )
{
   static size_t const masks[3] = { OBB_INTERSECT_DETAIL::AXIS_X,
                                    OBB_INTERSECT_DETAIL::AXIS_Y,
                                    OBB_INTERSECT_DETAIL::AXIS_Z };

   Point res;

   if ( idx < 8 )
   {
      for ( size_t i = 0; i != 3; ++i )
      {
         if ( idx & masks[i] )
            res += ext(i);
         else
            res -= ext(i);
      }
   }
   else if ( idx < 8 + 6 )
   {
      if ( idx & 1 )
         res += ext((idx - 8) / 2);
      else
         res -= ext((idx - 8) / 2);
   }
   else
   {
      if ( idx == 14 )
      {
         res += ext(0);
         res += ext(1);
      }
      else if ( idx == 15 )
      {
         res -= ext(0);
         res += ext(1);
      }
      else if ( idx == 16 )
      {
         res += ext(0);
         res -= ext(1);
      }
      else if ( idx == 17 )
      {
         res -= ext(0);
         res -= ext(1);
      }
      else if ( idx == 18 )
      {
         res += ext(1);
         res += ext(2);
      }
      else if ( idx == 19 )
      {
         res -= ext(1);
         res += ext(2);
      }
      else if ( idx == 20 )
      {
         res += ext(1);
         res -= ext(2);
      }
      else if ( idx == 21 )
      {
         res -= ext(1);
         res -= ext(2);
      }
      else if ( idx == 22 )
      {
         res += ext(0);
         res += ext(2);
      }
      else if ( idx == 23 )
      {
         res -= ext(0);
         res += ext(2);
      }
      else if ( idx == 24 )
      {
         res += ext(0);
         res -= ext(2);
      }
      else if ( idx == 25 )
      {
         res -= ext(0);
         res -= ext(2);
      }
   }

   return res;
}

template< class Point >
inline Point vertex( cg::OBB_t< Point > const & obb, size_t idx )
{
   return obb.origin + vertex( obb.extents, idx );
}

