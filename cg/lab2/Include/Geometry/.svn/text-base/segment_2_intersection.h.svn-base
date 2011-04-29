#pragma once

#include "primitives\segment.h"
#include "primitives\rectangle.h"

namespace cg {

   enum intersection_type { disjoint=0, intersect, overlap };

   //////////////////////////////////////////////////////
   // TODO :: Если точки NULL, то их вычислять не надо!!!
   //////////////////////////////////////////////////////

   template < class Pt, class Scalar >
      inline intersection_type generic_intersection(
         segment_t< Scalar, 2 > const &A, 
         segment_t< Scalar, 2 > const &B, 
         Pt *out1, 
         Pt *out2,
         Scalar eps = epsilon<Scalar>() )
   {
      typedef point_t< Scalar, 2 > point_type;

      point_type u = direction(A);
      point_type v = direction(B);
      point_type w = A.P0() - B.P0();

      Scalar eps_prim = 2 * eps; // u, v, w

      Scalar D = u ^ v; 

      Scalar eps_D = eps_prim * ( abs(u.x) + abs(u.y) + abs(v.x) + abs(v.y) );
      Scalar eps_vw = eps_prim * ( abs(v.x) + abs(v.y) + abs(w.x) + abs(w.y) );
      Scalar eps_uw = eps_prim * ( abs(u.x) + abs(u.y) + abs(w.x) + abs(w.y) );

      if ( eq_zero( D, eps_D ) )     // если они параллельны
      {
         if ( !eq_zero( u ^ w, eps_uw ) || !eq_zero( v ^ w, eps_vw ) )  // и не лежат на одной прямой
            return disjoint;     // они не пересекаются

         // мы определили, что они лежат на одной прямой
         Scalar lu = adjust( u * u, 2 * eps_prim * ( abs(u.x) + abs(u.y) ) );
         Scalar lv = adjust( v * v, 2 * eps_prim * ( abs(v.x) + abs(v.y) ) );

         if (lu == 0 && lv == 0) // оба отрезка - точки
         {
            // они могут не совпадать
            if ( !eq_zero( norm_sqr(w), 2 * eps_prim * ( abs(w.x) + abs(w.y) ) ) )
               return disjoint;

            // а могут и совпадать в некоторой точке
            if ( out1 != NULL ) 
            {
               out1->x = A.P0().x;
               out1->y = A.P0().y;
            }

            return intersect;
         }

         if (lu == 0)   // если A - точка
         {
            Scalar eps_contains = ( eps_vw * ( v * v ) + 2 * eps_prim * ( abs(v.x) + abs(v.y) ) * abs(v * w) ) / sqr(v * v) * cg::max( abs(B.P0().x) + abs(B.P1().x), abs(B.P0().y) + abs(B.P1().y) ) ;

            if ( !B.contains( A.P0(), eps_contains ) ) // которая не принадлежит B
               return disjoint;

            // или принадлежит
            if ( out1 != NULL ) 
            {
               out1->x = A.P0().x;
               out1->y = A.P0().y;
            }

            return intersect;
         }

         if (lv == 0)  // если В - точка
         {
            Scalar eps_contains = ( eps_uw * ( u * u ) + 2 * eps_prim * ( abs(u.x) + abs(u.y) ) * abs(u * -w) ) / sqr(u * u) * cg::max( abs(A.P0().x) + abs(A.P1().x), abs(A.P0().y) + abs(A.P1().y) ) ;

            if ( !A.contains(B.P0(), eps_contains) )  // которая не принадлежит A
               return disjoint;

            if ( out1 != NULL ) 
            {
               out1->x = B.P0().x;
               out1->y = B.P0().y;
            }

            return intersect;
         }

         // мы определили, что A и B - невырожденные отрезки, лежащие на одной прямой

         // t0 - коэф-ты проекций концов B на A
         Scalar t0 = A(B.P0());
         Scalar t1 = A(B.P1());

         point_type w_prim = B.P1() - A.P0();

         Scalar eps_t0 = ( eps_uw * ( u * u ) + 2 * eps_prim * ( abs(u.x) + abs(u.y) ) * abs(u * -w) ) / sqr(u * u) * cg::max( abs(A.P0().x) + abs(A.P1().x), abs(A.P0().y) + abs(A.P1().y) ) ;
         Scalar eps_uw_prim = eps_prim * ( abs(u.x) + abs(u.y) + abs(w_prim.x) + abs(w_prim.y) );
         Scalar eps_t1 = ( eps_uw_prim * ( u * u ) + 2 * eps_prim * ( abs(u.x) + abs(u.y) ) * abs(u * w_prim) ) / sqr(u * u) * cg::max( abs(A.P0().x) + abs(A.P1().x), abs(A.P0().y) + abs(A.P1().y) ) ;

         sort2(t0,t1,t0,t1);  // make t0 < t1

         // (!) точка пересечения - A.P0()
         if (eq_zero(t1,eps_t1)) 
         {        
            if ( out1 != NULL ) 
            {
               out1->x = A.P0().x;
               out1->y = A.P0().y;
            }
            return intersect;
         }

         // (!) точка пересечения - A.P1()
         if (eq(t0, 1, eps_t0)) 
         {        
            if ( out1 != NULL ) 
            {
               out1->x = A.P1().x;
               out1->y = A.P1().y;
            }

            return intersect;
         }

         if (t1 < 0 || t0 > 1) // отрезки не пересекаются
            return disjoint;

         //*out1 = t0 < 0 ? A.P0() : A(t0);
         if ( t0 < 0 )
         {
            if ( out1 != NULL ) 
            {
               out1->x = A.P0().x;
               out1->y = A.P0().y;
            }
         }
         else
         {
            if ( out1 != NULL ) 
            {
               out1->x = A(t0).x;
               out1->y = A(t0).y;
            }
         }

         //*out2 = t1 > 1 ? A.P1() : A(t1);
         if ( t1 > 1 )
         {
            if ( out2 != NULL ) 
            {
               out2->x = A.P1().x;
               out2->y = A.P1().y;
            }
         }
         else
         {
            if ( out2 != NULL ) 
            {
               out2->x = A(t1).x;
               out2->y = A(t1).y;
            }
         }

         return overlap;
      }

      // intersection parameter for A
      Scalar tA = (v ^ w) / D;
      Scalar eps_tA = ( eps_vw * abs(D) + eps_D * abs(v^w) ) / sqr(D);

      if ( !between01eps ( tA, eps_tA ) )
         //      if (!between01(tA))
         return disjoint;

      Scalar tB = (u ^ w) / D;
      Scalar eps_tB = ( eps_uw * abs(D) + eps_D * abs(u^w) ) / sqr(D);

      if ( !between01eps ( tB, eps_tB ) )
         //      if (!between01(tB))
         return disjoint;

      //*out1 = A(tA);
      if ( out1 != NULL ) 
      {
         out1->x = A(tA).x;
         out1->y = A(tA).y;
      }

      return intersect;

   }

   template < class Scalar >
      inline intersection_type generic_intersection( segment_t< Scalar, 2 > const &A, 
      segment_t< Scalar, 2 > const &B, 
      Scalar eps = epsilon<Scalar>() * 1e-5 )
   {
      return generic_intersection<cg::point_t< Scalar, 2 >, Scalar> ( A, B, NULL, NULL, eps );
   }


   inline bool has_intersection(segment_2 const &s1, segment_2 const &s2)
   {
      point_2 r;
      return disjoint != generic_intersection(s1,s2,&r,&r);
   }

   inline bool is_crossing_exact ( point_2 const & A, point_2 const & B, 
      point_2 const & C, point_2 const & D )
   {
      return has_intersection(segment_2(A, B), segment_2(C, D));
   }

   inline bool has_intersection(segment_2 const &S, rectangle_2 const &R)
   {
      if (R.contains(S))
         return true;

      if (!has_intersection(R.x, range_2(S.P0().x, S.P1().x)))
         return false;

      if (!has_intersection(R.y, range_2(S.P0().y, S.P1().y)))
         return false;

      if (has_intersection(segment_2(R.xy(),R.xY()), S))  
         return true;

      if (has_intersection(segment_2(R.Xy(),R.XY()), S))  
         return true;

      if (has_intersection(segment_2(R.xy(),R.Xy()), S))  
         return true;

      if (has_intersection(segment_2(R.xY(),R.XY()), S))  
         return true;

      return false;
   }

   inline bool has_intersection(cg::rectangle_2 const & r, cg::segment_2 const & s)
   {
      return has_intersection(s, r);
   }

   //inline double distance(point_2 const &pt, rectangle_2 const &rc)
   //{
   // return min(
   //  distance( top_side      (rc), pt ),
   //  distance( left_side     (rc), pt ),
   //  distance( right_side    (rc), pt ),
   //  distance( bottom_side   (rc), pt ));
   //}

   // расстояние от отрезка до отрезка
   inline double distance(segment_2 const &S1, segment_2 const &S2)
   {
      cg::intersection_type ipt_type = cg::generic_intersection <cg::point_2> ( S1, S2, NULL, NULL );

      if ( ipt_type == cg::disjoint )
         return cg::min ( distance(S2, S1.P0()), distance(S2, S1.P1()), distance(S1, S2.P0()), distance(S1, S2.P1()) );

      return 0;
   }
}
