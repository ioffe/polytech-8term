#pragma once

#include "Geometry\cgal_predicates.h"

//
// Predicate to compare two segments by x coordinate
// Point-segment is mentioned to be greater if lies on not point-segment
//

namespace cg
{

template< class Segment >
struct SegmentXLess
{
   typedef typename Segment::PointT PointT;

   bool operator () ( Segment const &e1, Segment const &e2 ) const
   {
      Segment a = e1, b = e2;

      if (a.vertexDestination.y < a.vertexOrigin.y)
         std::swap(a.vertexOrigin, a.vertexDestination);
      if (b.vertexDestination.y < b.vertexOrigin.y)
         std::swap(b.vertexOrigin, b.vertexDestination);

      if (b.vertexOrigin == b.vertexDestination)
      {
         // 'b' - point-edge

         return robust_right_turn(b.vertexOrigin, a.vertexOrigin, a.vertexDestination) &&
                robust_right_turn(b.vertexDestination, a.vertexOrigin, a.vertexDestination) &&
               !(a.vertexOrigin == b.vertexOrigin && a.vertexDestination == b.vertexDestination);
      }

      if (a.vertexOrigin == a.vertexDestination)
      {
         // 'a' - point-edge

         VecOrientation orien = robust_orientation(a.vertexOrigin, b.vertexOrigin, b.vertexDestination);
         if (orien == VO_COLLINEAR)
            return false;

         return orien == VO_LEFT &&
            robust_left_turn(a.vertexDestination, b.vertexOrigin, b.vertexDestination) &&
            !(a.vertexOrigin == b.vertexOrigin && a.vertexDestination == b.vertexDestination);
      }

      if (a.vertexDestination.y == a.vertexOrigin.y)
      {
         // 'a' is horizontal

         if (b.vertexDestination.y == b.vertexOrigin.y)
         {
            // 'b' is horizontal
            return cg::max(a.vertexOrigin.x, a.vertexDestination.x) <=
                   cg::min(b.vertexOrigin.x, b.vertexDestination.x);
         }

         return robust_left_turn(a.vertexOrigin, b.vertexOrigin, b.vertexDestination) &&
                robust_left_turn(a.vertexDestination, b.vertexOrigin, b.vertexDestination);
      }

      if (b.vertexDestination.y == b.vertexOrigin.y)
      {
         // 'b' is horizontal

         return robust_right_turn(b.vertexOrigin, a.vertexOrigin, a.vertexDestination) &&
                robust_right_turn(b.vertexDestination, a.vertexOrigin, a.vertexDestination);
      }

      // Для этих случаев нужен какой-то строгий предикат, чтобы не выполнялось (a < b) && (b < a)
      if ( a.vertexOrigin == b.vertexDestination )
      {
         return robust_left_turn(a.vertexDestination, b.vertexOrigin, b.vertexDestination) && 
            a.vertexDestination.y > b.vertexOrigin.y;
      }
      if ( a.vertexDestination == b.vertexOrigin )
      {
         return robust_left_turn(a.vertexOrigin, b.vertexOrigin, b.vertexDestination) &&
            a.vertexOrigin.y > b.vertexDestination.y;
      }

      // General case

      return ((robust_left_turn(a.vertexOrigin, b.vertexOrigin, b.vertexDestination) &&
               robust_left_turn(a.vertexDestination, b.vertexOrigin, b.vertexDestination)) ||
              (robust_right_turn(b.vertexOrigin, a.vertexOrigin, a.vertexDestination) &&
               robust_right_turn(b.vertexDestination, a.vertexOrigin, a.vertexDestination))) &&
               !(a.vertexOrigin == b.vertexOrigin && a.vertexDestination == b.vertexDestination);
   }
};

} // End of 'cg' namespace
