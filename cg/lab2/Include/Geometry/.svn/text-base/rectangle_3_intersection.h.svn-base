#pragma once

#include "common\util.h"
#include "geometry\primitives\rectangle.h"
#include "geometry\cylinder_triangle_intersection.h"

namespace cg{
    namespace rectangle_3_details {

       enum qual_type { 
          INSIDE    = 0x00, 
          BOTTOM    = 0x01, 
          TOP       = 0x02, 
          RIGHT     = 0x04, 
          LEFT      = 0x08, 
          UP        = 0x10, 
          DOWN      = 0x20 
       };

       inline qual_type qualify( double x, range_2 const & r, qual_type lt, qual_type gt, double eps )
       {
          static double common_eps = cg::epsilon< double >( );

          if( x < r.lo( ) - eps - common_eps )
             return lt;
          else if( x > r.hi( ) + eps + common_eps )
             return gt;

          return INSIDE;
       }

       inline qual_type qualify( point_3 const & p, rectangle_3 const & rect, double eps )
       {
          return static_cast< qual_type > ( qualify( p.x, rect.x, LEFT,   RIGHT, eps ) 
                                          | qualify( p.y, rect.y, BOTTOM, TOP,   eps ) 
                                          | qualify( p.z, rect.z, DOWN,   UP,    eps ) );
       }

      struct ray_intersecter_type
      {
         ray_intersecter_type( point_3 const & p1, point_3 const & p2, rectangle_3 const & rect, double eps )
            : ratio_( 0 )
            , eps_  ( eps )
            , ok_   ( false )
            , p_    ( p1 )
            , dp_   ( p2 - p1 )
            , q1_   ( qualify( p1, rect, eps ) )
            , q2_   ( qualify( p2, rect, eps ) )
         {
            ok_ = intersect( p2, rect );
         }

         bool operator( )( double * ratio, point_3 * pos ) const 
         { 
            if( ok_ )
            {
               if( ratio )
                  *ratio = ratio_;

               if( pos )
                  *pos = p_;
               
               return true;
            }

            return false; 
         }

      private:
         
         bool intersect( point_3 const & p2, rectangle_3 const & rect )
         {
            if( !component_intersection( p_.x, dp_.x, rect.x, rect ) )
               return false;

            if ( q1_ == INSIDE )
               return true;

            dp_ = p2 - p_;
            if( !component_intersection( p_.y, dp_.y, rect.y, rect ) )
               return false;

            if ( q1_ == INSIDE )
               return true;

            dp_ = p2 - p_;
            if( !component_intersection( p_.z, dp_.z, rect.z, rect ) )
               return false;

            if ( q1_ == INSIDE )
               return true;

            return false;
         }

         bool component_intersection( double x, double dx, range_2 const & r, rectangle_3 const & rect )
         {
            if ( q1_ & q2_ )
               return false;

            double t = 0;

            if ( dx > 0 && x < r.lo( ) )
               t = ( r.lo( ) - x ) / dx;
            else if ( dx < 0 && x > r.hi( ) )
               t = ( r.hi( ) - x ) / dx;

            if( t > 0 )
            {
               p_ += dp_ * t;
               ratio_ += ( 1 - ratio_ ) * t;
               q1_ = qualify( p_, rect, eps_ );
            }
            return true;
         }

      private:
         double          ratio_;
         const double    eps_;
         bool            ok_;
         point_3         p_;
         point_3         dp_;
         qual_type       q1_;
         const qual_type q2_;
      };
    }

    // Пересечение с лучом
    inline bool ray_intersection( point_3 const & p1, point_3 const & p2, rectangle_3 const & rect, double * ratio, point_3 * pos, double eps = cg::epsilon< double >( ) )
    {
         return rectangle_3_details::ray_intersecter_type( p1, p2, rect, eps )( ratio, pos );
    }

    inline point_3 CalcNormal( point_3 const & p, rectangle_3 const & rect )
    {
       // TODO:: после рефакторинга переделать на for
       point_3 n( 0, 0, 0 );
       if( cg::eq( rect.x.lo( ), p.x ) )
          n.x = -1;
       else if( cg::eq( rect.x.hi( ), p.x ) )
          n.x = 1;

       if( cg::eq( rect.y.lo( ), p.y ) )
          n.y = -1;
       else if( cg::eq( rect.y.hi( ), p.y ) )
          n.y = 1;

       if( cg::eq( rect.z.lo( ), p.z ) )
          n.z = -1;
       else if( cg::eq( rect.z.hi( ), p.z ) )
          n.z = 1;

       return cg::normalized_safe( n );
    }

    // Пересечение с цилиндром
    inline bool cylinder_rectangle_intersection( point_3 const & p1, point_3 p2, double radius, 
                                                 rectangle_3 const & rect, double * pRatio )
    {
      point_3 const points[ ] = { rect.xyz(), //0
                                  rect.xyZ(), //1
                                  rect.xYz(), //2
                                  rect.xYZ(), //3
                                  rect.Xyz(), //4
                                  rect.XyZ(), //5
                                  rect.XYz(), //6
                                  rect.XYZ()  //7
                                };

      static const point_3i triangles[] = { point_3i( 0, 5, 1 ), point_3i( 0, 4, 5 ),
                                            point_3i( 2, 1, 3 ), point_3i( 2, 0, 1 ),
                                            point_3i( 6, 3, 7 ), point_3i( 6, 2, 3 ),
                                            point_3i( 4, 7, 5 ), point_3i( 4, 6, 7 ),
                                            point_3i( 1, 7, 3 ), point_3i( 1, 5, 7 ),
                                            point_3i( 0, 6, 4 ), point_3i( 0, 2, 6 )
                                          };
      bool res = false;
      double ratio = 1;

      cg::cylinder_intersection_details::std_traits< cg::triangle_3_fast< point_3 > > traits ; 

      // Перебираем все треугольники
      for ( DWORD i = 0, size = util::array_size( triangles ); i != size; i++ )
      {
         cg::triangle_3_fast< point_3 > tr ( points, triangles[ i ].x, triangles[ i ].y, triangles[ i ].z ) ;

         if( ratio <= 1e-5 )
            break;

         cg::point_3 resp;
         double r = 0;
         cg::segment_3 const length ( p1, p2 ) ;
         if ( cg::cylinder_triangle_intersection ( length, radius, tr, r, resp, traits ) )
         {
            res = TRUE ;

            if ( !pRatio  )
               return TRUE ;

            ratio *= r ;
            p2 = length( r );
         }
      }

      if ( res && pRatio ) *pRatio = ratio ;

      return res ;
    }
}