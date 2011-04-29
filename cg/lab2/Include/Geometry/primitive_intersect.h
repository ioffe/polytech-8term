#pragma once

#include "Geometry/DupPointsEliminator.h"

#pragma pack(push,1)

struct PRIMITIVE_POINT_DETAIL : cg::point_3
{
   PRIMITIVE_POINT_DETAIL( )
      : cg::point_3 ()
      , type        ( -1 )
      , depth       ( 0. )
      , fixed       ( false )
      , objectId    ( 0 )
      , cookie      ( 0 )
   {}

   PRIMITIVE_POINT_DETAIL( cg::point_3 const &pnt, cg::point_3 const &normal, double depth, long type = -1, bool fixed = false )
      : cg::point_3 ( pnt    )
      , type        ( type   )
      , normal      ( normal )
      , depth       ( depth  )
      , fixed       ( fixed  )
      , objectId    ( 0 )
      , cookie      ( 0 )
   {}

   PRIMITIVE_POINT_DETAIL(double x, double y)
      : cg::point_3 ( x, y, 0. )
      , type        ( -1 )
      , normal      ()
      , depth       ( 0. )
      , fixed       ( false )
      , objectId    ( 0 )
      , cookie      ( 0 )
   {}

   typedef cg::point_3 point_type;

   long           type;
   point_type     normal;
   double         depth;
   bool           fixed;
   long           objectId;
   long           cookie;
};

#pragma pack(pop)

namespace cg
{

static double const COLLISION_PNT_PROXIMITY_EPS = 1.e-8;

template<class Points>
   void filter_dup_collision_points(Points const &in, Points &out, double eps = COLLISION_PNT_PROXIMITY_EPS)
{
   if(out.full())
      return;

   typedef DuplicateEliminator3dTraits<PRIMITIVE_POINT_DETAIL> dpe_traits_type;

   long   type   = (in.begin())->type;
   bool   fixed  = (in.begin())->fixed;
   long   objId  = (in.begin())->objectId;
   long   cookie = (in.begin())->cookie;

   size_t const points_size = in.size() ;
   PRIMITIVE_POINT_DETAIL * points = reinterpret_cast<PRIMITIVE_POINT_DETAIL *>(_alloca(sizeof(PRIMITIVE_POINT_DETAIL) * points_size));
   size_t i = 0;
   for(Points::const_iterator it = in.begin(), end = in.end(); it != end; ++it, ++i)
   {
      Assert((type == it->type) && (fixed == it->fixed) && (objId == it->objectId) && (cookie == it->cookie));
      points[i] = *it;
   }

   std::sort(points, points + points_size, dpe_traits_type::Comparer());

   for(PRIMITIVE_POINT_DETAIL const * it = points, * end = points + points_size; it != end && !out.full();)
   {
      PRIMITIVE_POINT_DETAIL const * p1 = it;
      PRIMITIVE_POINT_DETAIL const * p2 = p1 + 1;

      std::vector<PRIMITIVE_POINT_DETAIL const *> heap;
      heap.reserve(points_size);
      heap.push_back(p1);

      dpe_traits_type traits;
      while((p2 != end) && traits.is_closer(*p1, *p2, eps))
         heap.push_back(p2++);

      // analyze heap
      PRIMITIVE_POINT_DETAIL pnt;
      for(size_t i = 0, size = heap.size(); i != size; ++i)
      {
         pnt        += *(heap[i]);
         pnt.normal += heap[i]->normal;
         pnt.depth  += heap[i]->depth;

         pnt.type     = type;
         pnt.fixed    = fixed;
         pnt.objectId = objId;
         pnt.cookie   = cookie;
      }

      // Note: resulting depth isn't correct!
      if(1 != heap.size()) 
      {
         static_cast<cg::point_3&>(pnt) /= heap.size();
         cg::normalize(pnt.normal);
         pnt.depth /= heap.size();
      }

      out.push_back(pnt);
      it = p2;
   }
}

} // end of namespace cg
