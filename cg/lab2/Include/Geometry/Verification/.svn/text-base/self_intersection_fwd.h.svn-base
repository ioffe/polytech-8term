#pragma once

#include <geometry/primitives.h>

namespace cg {
namespace verification {

   struct intersection
   {
      intersection ( ) {}

      intersection ( size_t cA, size_t cB, cg::point_2 const &ref )
         : cntA (cA)
         , cntB (cB)
         , refPoint (ref)
      {
      }

      size_t cntA;
      size_t cntB;
      cg::point_2 refPoint;
   };

   struct contours_intersection
   {
      contours_intersection() {}

      // TODO: Currently order of contours is important for comparison.
      contours_intersection( size_t cA, size_t cB,
                             size_t sA, size_t sB,
                             cg::point_2 const &ref )
         : cntA(cA)
         , cntB(cB)
         , segA(sA)
         , segB(sB)
         , refPoint(ref)
      {}

      size_t cntA;
      size_t cntB;
      size_t segA;
      size_t segB;
      cg::point_2 refPoint;
   };

   inline bool operator < ( contours_intersection const &a, contours_intersection const &b )
   {
      if (a.cntA == b.cntA)
      {
         if (a.cntB == b.cntB)
         {
            if (a.segA == b.segA)
            {
               if (a.segB == b.segB)
                  return a.refPoint < b.refPoint;
               else
                  return a.segB < b.segB;
            }
            else
               return a.segA < b.segA;
         }
         else
            return a.cntB < b.cntB;
      }
      else
         return a.cntA < b.cntA;
   }

}}
