#pragma once

#include "geometry/convex_hull.h"

namespace cg {
namespace algos {

   namespace details
   {
      // TODO :: избавиться от этого адаптора
      template < class RnaIter, class OutIter > 
         inline void calculate_convex_hull( RnaIter p, RnaIter q, OutIter output )
      {
         std::vector<int> result_poly;
         cg::build_convex_hull( p, q, std::back_inserter(result_poly) );

         for ( std::vector<int>::const_iterator it = result_poly.begin( ); it != result_poly.end( ); ++it )
            *output++ = p[ *it ];
      }

      template < class FwdIter >
         struct next_fun
      {
         next_fun( FwdIter p, FwdIter q )
            : beg_( p )
            , end_( q )
         {}

         FwdIter operator( ) ( FwdIter p ) const
         {
            // тут есть риск, что мы уже вне диапазона. надо подумать

            Assert( p != end_ );

            if ( ++p == end_ )
               return beg_;

            return p;
         }

      private:

         FwdIter beg_;
         FwdIter end_;
      };

      template<class point>
         double angle(point const &a, point const &b)
      {
         return atan2(a ^ b, a * b);
      }
   }

   // алгоритм считает радиус множества
   // TODO :: вынести в глобальную библиотеку

   // алгоритм для двумерного случая
   template < class FwdIter >
      inline segment_2 calc_diameter( FwdIter p, FwdIter q )
   {
      typedef
         std::iterator_traits< FwdIter >::value_type
         point_type;

      size_t nPoints = std::distance( p, q );

      if ( nPoints == 0 )
      {
         Assert( false );
         return segment_2();
      }

      if ( nPoints == 1 )
         return segment_2(*p, *p);
      
      if ( nPoints == 2 )
      {
         FwdIter n = p;
         return segment_2(*p, *(++n));
      }

      // ищем выпуклую оболочку
      typedef std::vector< point_type > points_type;
      points_type convex_hull;

      details::calculate_convex_hull( p, q, std::back_inserter( convex_hull ) );

      // запустить калиперы

      // находим самую левую и самую правую точки
      typedef
         points_type::iterator
         pi_type;

      pi_type
         min_it, max_it,
         p_it = min_it = std::min_element( convex_hull.begin( ), convex_hull.end( ) ),
         q_it = max_it = std::max_element( convex_hull.begin( ), convex_hull.end( ) );

      point_type
         dir_p( 0,  1 ),
         dir_q( 0, -1 ); 

      double    max_dist = 0.;
      segment_2 diameter( *p_it, *q_it );

      details::next_fun< pi_type >
         next( convex_hull.begin( ), convex_hull.end( ) );

      // цикл, пока пара точек снова не станет исходной
      do
      {
         // итерация -- один поворот калипер

         make_max( max_dist, distance( *p_it, *q_it ), diameter, segment_2(*p_it, *q_it) );            

         pi_type p_next_it, q_next_it;
         point_type dir_p_next, dir_q_next;
         double p_ang, q_ang;

         do
         {
            p_next_it = next( p_it );
            dir_p_next = *p_next_it - *p_it;
            p_ang = details::angle( dir_p, dir_p_next );
         }
         while (cg::eq_zero(p_ang) && (p_it = p_next_it, true));

         do
         {
            q_next_it = next( q_it );
            dir_q_next = *q_next_it - *q_it;
            q_ang = details::angle( dir_q, dir_q_next );
         }
         while (cg::eq_zero(q_ang) && (q_it = q_next_it, true));

         // кто больше?
         if ( p_ang > q_ang )
         {
            dir_p = dir_p_next;
            dir_q = -dir_p;

            p_it = p_next_it;
         }
         else
         {
            dir_q = dir_q_next;
            dir_p = -dir_q;

            q_it = q_next_it;
         }
      } 
      while ( p_it != min_it || q_it != max_it );

      return diameter;
   }

}}