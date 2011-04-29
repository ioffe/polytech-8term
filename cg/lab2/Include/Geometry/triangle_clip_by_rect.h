#pragma once

#include "primitives\triangle.h"
#include "primitives\rectangle.h"

namespace cg 
{
   namespace details
   {
      template <typename S>
         point_t<S,3> y_by_x(point_t<S,3> const &s0, point_t<S,3> const &s1, S x)
         {
            return point_t<S,3> ( x, lerp(s0.x, s1.x, s0.y, s1.y)(x), lerp(s0.x, s1.x, s0.z, s1.z)(x) ) ;
         }

      template <typename S>
         point_t<S,2> y_by_x(point_t<S,2> const &s0, point_t<S,2> const &s1, S x)
         {
            return point_t<S,2> ( x, lerp(s0.x, s1.x, s0.y, s1.y)(x) ) ;
         }

      template <typename S>
         point_t<S,3> x_by_y(point_t<S,3> const &s0, point_t<S,3> const &s1, S y)
         {
            return point_t<S,3> ( lerp(s0.y, s1.y, s0.x, s1.x)(y), y, lerp(s0.y, s1.y, s0.z, s1.z)(y) ) ;
         }

      template <typename S>
         point_t<S,2> x_by_y(point_t<S,2> const &s0, point_t<S,2> const &s1, S y)
         {
            return point_t<S,2> ( lerp(s0.y, s1.y, s0.x, s1.x)(y), y ) ;
         }

      template <typename S, size_t D>
         bool cut_by_left ( std::vector< point_t<S, D> > const & pin, S Left, std::vector< point_t<S, D> > & pout )
         {
            byte b1, b2;

            b1 = pin[0].x > Left ? 1 : 0;
            for ( size_t i = 0; i < pin.size()-1; i ++ )
            {
               b2 = pin[i+1].x > Left ? 1 : 0;

               if ( b1 )
                  pout.push_back( pin[i] );
               if ( b1 ^ b2 )
                  pout.push_back( details::y_by_x ( pin[i], pin[i+1], Left ) ) ;

               b1 = b2;
            }

            if ( pout.size() )
            {
               pout.push_back( pout.front() );
               return true;
            }
            return false;
         }

      template <typename S, size_t D>
         bool cut_by_right ( std::vector< point_t<S, D> > const & pin, S Right, std::vector< point_t<S, D> > & pout )
         {
            byte b1, b2;

            b1 = pin[0].x < Right ? 1 : 0;
            for ( size_t i = 0; i < pin.size()-1; i ++ )
            {
               b2 = pin[i+1].x < Right ? 1 : 0;

               if ( b1 )
                  pout.push_back( pin[i] );
               if ( b1 ^ b2 )
                  pout.push_back( details::y_by_x ( pin[i], pin[i+1], Right ) ) ;

               b1 = b2;
            }

            if ( pout.size() )
            {
               pout.push_back( pout.front() );
               return true;
            }
            return false;
         }

      template <typename S, size_t D>
         bool cut_by_top ( std::vector< point_t<S, D> > const & pin, S Top, std::vector< point_t<S, D> > & pout )
         {
            byte b1, b2;

            b1 = pin[0].y > Top ? 1 : 0;
            for ( size_t i = 0; i < pin.size()-1; i ++ )
            {
               b2 = pin[i+1].y > Top ? 1 : 0;

               if ( b1 )
                  pout.push_back( pin[i] );
               if ( b1 ^ b2 )
                  pout.push_back( details::x_by_y ( pin[i], pin[i+1], Top ) );

               b1 = b2;
            }

            if ( pout.size() )
            {
               pout.push_back( pout.front() );
               return true;
            }
            return false;
         }

      template <typename S, size_t D>
         bool cut_by_bottom ( std::vector< point_t<S, D> > const & pin, S Bottom, std::vector< point_t<S, D> > & pout )
         {
            byte b1, b2;

            b1 = pin[0].y < Bottom ? 1 : 0;
            for ( size_t i = 0; i < pin.size()-1; i ++ )
            {
               b2 = pin[i+1].y < Bottom ? 1 : 0;

               if ( b1 )
                  pout.push_back( pin[i] );
               if ( b1 ^ b2 )
                  pout.push_back( details::x_by_y ( pin[i], pin[i+1], Bottom ) );

               b1 = b2;
            }

            if ( pout.size() )
            {
               pout.push_back( pout.front() );
               return true;
            }
            return false;
         }

      template <typename S, size_t D>
         bool has_intersection( triangle_t<S, D> const &tr, rectangle_t<S, 2> const &r )
         {
            if ( tr[0].x <= r.x.lo() && tr[1].x <= r.x.lo() && tr[2].x <= r.x.lo() )
               return false;
            if ( tr[0].x >= r.x.hi() && tr[1].x >= r.x.hi() && tr[2].x >= r.x.hi() )
               return false;
            if ( tr[0].y <= r.y.lo() && tr[1].y <= r.y.lo() && tr[2].y <= r.y.lo() )
               return false;
            if ( tr[0].y >= r.y.hi() && tr[1].y >= r.y.hi() && tr[2].y >= r.y.hi() )
               return false;
            return true;  
         }

      template <typename S, size_t D>
         bool contains( rectangle_t<S, 2> const &r, triangle_t<S, D> const &tr )
         {
            return r.contains( tr[0] ) && r.contains( tr[1] ) && r.contains( tr[2]) ;  
         }
   }

   template <typename S, size_t D>
      bool cull( triangle_t<S, D> const &tr, rectangle_t<S, 2> const &r, std::vector< point_t<S, D> > & p )
      {
         if ( !details::has_intersection( tr, r ) )
            return false;

         std::vector< point_t<S, D> > pin;  pin.reserve( 8 );
         std::vector< point_t<S, D> > pout; pout.reserve( 8 );
         
         pin.push_back( tr[0] );
         pin.push_back( tr[1] );
         pin.push_back( tr[2] );
         pin.push_back( tr[0] );

         if ( !details::cut_by_left( pin, r.x.lo(), pout ) )
            return false;
         pin.clear( ); pin.swap( pout );
         if ( !details::cut_by_right( pin, r.x.hi(), pout ) )
            return false;
         pin.clear( ); pin.swap( pout );
         if ( !details::cut_by_top( pin, r.y.lo(), pout ) )
            return false; 
         pin.clear( ); pin.swap( pout );
         if ( !details::cut_by_bottom( pin, r.y.hi(), pout ) )
            return false;

         Assert( pout.size() <= 8 );
         
         p.assign( pout.begin(), pout.end()-1 );
         return true;
      }
}
