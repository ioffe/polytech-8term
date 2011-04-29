#pragma once

#include "geometry\triangle_raster_aux.h"

namespace cg
{
   inline bool calc_barycentric_coords_2d( triangle_2 const & tr, point_2 const & pt, barycentric_coords & bc )
   {
      // считаем бароцентрические координаты
      const double bu0 = pt.x - tr[ 0 ].x;
      const double bv0 = pt.y - tr[ 0 ].y;

      const double bu1 = tr[ 1 ].x - tr[ 0 ].x;
      const double bv1 = tr[ 1 ].y - tr[ 0 ].y;

      const double bu2 = tr[ 2 ].x - tr[ 0 ].x;
      const double bv2 = tr[ 2 ].y - tr[ 0 ].y;

      const double det = bv2 * bu1 - bu2 * bv1; 

      if ( eq( det, 0.0 ) ) // проверка треугольника на вырожденность
         return false;

      const double detb = bv0 * bu1 - bv1 * bu0;
      bc.beta  = detb / det;

      const double eps = epsilon<float>() ;

      if ( bc.beta  < -eps  ||  bc.beta > 1.0 + eps )
         return false;

      const double deta = bv2 * bu0 - bv0 * bu2;
      bc.alpha = deta / det;

      if ( bc.alpha < -eps  ||  ( bc.alpha + bc.beta ) > 1.0 + eps )
         return false;

      return true;
   }

   namespace details
   {
      template< int N > point_2 get_project( point_3 const &pt );

      template<> inline point_2 get_project< 0 >( point_3 const &pt ){ return point_2( pt.y, pt.z ); }
      template<> inline point_2 get_project< 1 >( point_3 const &pt ){ return point_2( pt.x, pt.z ); }
      template<> inline point_2 get_project< 2 >( point_3 const &pt ){ return point_2( pt.x, pt.y ); }

      template< int N > 
         bool calc_barycentric_coords_3d_impl( triangle_3 const & t, point_3 const &p, barycentric_coords & bc )
      {
         const triangle_2 tr( get_project<N>( t[0] ), get_project<N>( t[1] ), get_project<N>( t[2] ) );
         return calc_barycentric_coords_2d( tr, get_project<N>( p ), bc );
      }
   }

   // WARNING :: точка pt должна лежать в плоскости треугольника t
   inline bool calc_barycentric_coords_3d( triangle_3 const & t, point_3 const &p, barycentric_coords & bc )
   {
      // нормаль треугольника
      const point_3 n = normal( t );

      // находим максимальную составляющую нормали
      if ( !eq( n.z, 0. ) )
         return details::calc_barycentric_coords_3d_impl<2>( t, p, bc );
      else if( !eq( n.y, 0. ) )
         return details::calc_barycentric_coords_3d_impl<1>( t, p, bc );
      else
         return details::calc_barycentric_coords_3d_impl<0>( t, p, bc );

      Assert( !"Все плохо: нормаль вырожденная" );
      return false;
   }
}
