#pragma once

#include "common\assert.h"
#include "xmath.h"
#include "primitives\segment.h"
namespace cg 
{
   /*
   struct empty_intersection : std::logic_error 
   {
   empty_intersection() : std::logic_error("empty_intersection") {}
   };
   */
   namespace shit 
   {
      inline double y_by_x(point_2 const &s0, point_2 const &s1, double x)
      {
         return lerp(s0.x, s1.x, s0.y, s1.y)(x);
      }

      inline void y_by_x(point_3 const &s0, point_3 const &s1, double x, double * pY, double * pZ)
      {
         *pY = lerp(s0.x, s1.x, s0.y, s1.y)(x);
         *pZ = lerp(s0.x, s1.x, s0.z, s1.z)(x);
      }

      inline double y_by_x(segment_2 const & s, double x)
      {
         return y_by_x(s.P0(), s.P1(), x);
      }

      // Пересечение прямой s с прямой x = x
      inline point_2 intersection_x(segment_2 const & s, double x)
      {                                               
         return point_2(x, y_by_x(s, x));
      }

      inline double x_by_y(point_2 const &s0, point_2 const &s1, double y)
      {
         return lerp(s0.y, s1.y, s0.x, s1.x)(y);
      }

      inline void x_by_y(point_3 const &s0, point_3 const &s1, double y, double * pX, double * pZ)
      {
         *pX = lerp(s0.y, s1.y, s0.x, s1.x)(y);
         *pZ = lerp(s0.y, s1.y, s0.z, s1.z)(y);
      }

      inline double x_by_y(segment_2 const & s, double y)
      {
         return x_by_y(s.P0(), s.P1(), y);
      }

      // Пересечение прямой s с прямой y = y
      inline point_2 intersection_y(segment_2 const & s, double y)
      {
         return point_2(x_by_y(s,y), y);
      }

      inline bool cut_by_left (point_2 &s0, point_2 const &s1, double Left)
      {
         const double eps = epsilon<double>( ) ;

         if (s0.x < Left - eps) 
         {
            if (!ge(s1.x, Left))
               return true;
            //              throw empty_intersection();
            s0.y = y_by_x(s0, s1, Left), s0.x = Left + eps;
         }
         return false;
      }

      inline bool cut_by_left_ex (point_2 &s0, point_2 const &s1, double Left, bool &inside)
      {
         inside = true;
         if (s0.x < Left) 
         {
            inside = false;
            if (s1.x < Left)
               return true;
            s0.y = y_by_x(s0, s1, Left), s0.x = Left;
         }
         return false;
      }

      inline bool cut_by_right (point_2 &s0, point_2 const &s1, double Right)
      {
         const double eps = epsilon<double>( ) ;

         if (s0.x > Right + eps) 
         {
            if (!le(s1.x, Right))
               return true;
            //              throw empty_intersection();
            s0.y = y_by_x(s0, s1, Right), s0.x = Right  - eps;
         }
         return false;
      }

      inline bool cut_by_right_ex (point_2 &s0, point_2 const &s1, double Right, bool &inside )
      {
         inside = true;
         if (s0.x > Right)
         {
            inside = false;
            if (s1.x > Right)
               return true;
            s0.y = y_by_x(s0, s1, Right), s0.x = Right;
         }
         return false;
      }

      inline bool cut_by_top (point_2 &s0, point_2 const &s1, double Top)
      {
         const double eps = epsilon<double>( ) ;

         if (s0.y < Top - eps) 
         {
            if (!ge(s1.y, Top))
               return true;
            //              throw empty_intersection();
            s0.x = x_by_y(s0, s1, Top), s0.y = Top + eps;
         }
         return false;
      }

      inline bool cut_by_top_ex (point_2 &s0, point_2 const &s1, double Top, bool &inside)
      {
         inside = true;
         if (s0.y < Top)
         {
            inside = false;
            if (s1.y < Top)
               return true;
            s0.x = x_by_y(s0, s1, Top), s0.y = Top;
         }
         return false;
      }

      inline bool cut_by_bottom (point_2 &s0, point_2 const &s1, double Bottom)
      {
         const double eps = epsilon<double>( ) ;

         if (s0.y > Bottom + eps) 
         {
            if (!le(s1.y, Bottom))
               return true;
            //              throw empty_intersection();
            s0.x = x_by_y(s0, s1, Bottom), s0.y = Bottom - eps;
         }
         return false;
      }

      inline bool cut_by_bottom_ex (point_2 &s0, point_2 const &s1, double Bottom, bool &inside )
      {
         inside = true;
         if (s0.y > Bottom)
         {
            inside = false;
            if (s1.y > Bottom)
               return true;
            s0.x = x_by_y(s0, s1, Bottom), s0.y = Bottom;
         }
         return false;
      }

      inline bool cut_segment (point_2 &s0, point_2 const &s1, rectangle_2 const &r)
      {
         return cut_by_left  (s0, s1, r.x.lo())
            || cut_by_right (s0, s1, r.x.hi())
            || cut_by_top   (s0, s1, r.y.lo())
            || cut_by_bottom(s0, s1, r.y.hi());
      }

      inline bool cut_segment_ex (point_2 &s0, point_2 const &s1, rectangle_2 const &r, bool &inside)
      {
         bool insideLeft, insideRight, insideTop, insideBottom;

         bool res = cut_by_left_ex  (s0, s1, r.x.lo(), insideLeft)
            || cut_by_right_ex (s0, s1, r.x.hi(), insideRight)
            || cut_by_top_ex   (s0, s1, r.y.lo(), insideTop)
            || cut_by_bottom_ex(s0, s1, r.y.hi(), insideBottom);

         inside = insideLeft && insideRight && insideBottom && insideTop;
         return res;
      }
   }

   inline bool cull(segment_2 const &s, rectangle_2 const &r, segment_2 &clipped)
   {
      if (eq(s.P0(),s.P1()))
      {
         clipped = s;
         return r.contains(s.P0());
      }

      point_2 s0 = s.P0(), s1 = s.P1();

      if (  shit::cut_segment(s0, s1, r)
         || shit::cut_segment(s1, s0, r) )
         return false;

      clipped = segment_2(s0, s1);
      return true;
   }

   inline bool cull_ex(segment_2 const &s, rectangle_2 const &r, segment_2 &clipped, bool &inside)
   {
      if (s.P0() == s.P1())
      {
         clipped = s;
         return inside = r.contains(s.P0(), 0); // eps = 0
      }

      point_2 s0 = s.P0(), s1 = s.P1();

      bool inside01, inside10;
      if (  shit::cut_segment_ex(s0, s1, r, inside01)
         || shit::cut_segment_ex(s1, s0, r, inside10) )
         return false;

      inside = inside01 && inside10;
      clipped = segment_2(s0, s1);
      return true;
   }

   // определяет точку выхода прямой s из прямоугольника rc 
   // requires: has_intersection(s, rc)
   inline point_2 correct(segment_2 const & s, rectangle_2 const & rc) 
   {
      double test_x = s.P1().x > s.P0().x ? rc.x.hi() : rc.x.lo();
      double test_y = s.P1().y > s.P0().y ? rc.y.hi() : rc.y.lo();

      point_2 x,y;

      if ( cg::eq(s.P0().y, s.P1().y) ) {
         return shit::intersection_x(s, test_x);
      } else {
         x = shit::intersection_y(s, test_y);
      }

      if ( cg::eq(s.P0().x, s.P1().x) ) {
         return shit::intersection_y(s, test_y);
      } else {
         y = shit::intersection_x(s, test_x);
      }

      if ( rc.x.contains(x.x) ) 
         return x;

      if ( rc.y.contains(y.y))
         return y;

      Assert(0);

      return point_2();
   }

}