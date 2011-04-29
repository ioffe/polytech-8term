#pragma once

#include "quad.h"
#include "geometry\primitives\rectangle.h"

#include "Geometry\triangle_clip_by_rect.h"

namespace cg 
{

namespace details
{

template <typename S, size_t D>
   bool has_intersection( quad_t<S, D> const &q, rectangle_t<S, 2> const &r )
{
   if ( q[0].x <= r.x.lo() && q[1].x <= r.x.lo() && q[2].x <= r.x.lo() && q[3].x <= r.x.lo() )
      return false;
   if ( q[0].x >= r.x.hi() && q[1].x >= r.x.hi() && q[2].x >= r.x.hi() && q[3].x >= r.x.hi() )
      return false;
   if ( q[0].y <= r.y.lo() && q[1].y <= r.y.lo() && q[2].y <= r.y.lo() && q[3].x <= r.y.lo() )
      return false;
   if ( q[0].y >= r.y.hi() && q[1].y >= r.y.hi() && q[2].y >= r.y.hi() && q[3].y >= r.y.hi() )
      return false;
   return true;  
}

} // End of 'details' namespace

template <typename S, size_t D>
   bool cull( quad_t<S, D> const &q, rectangle_t<S, 2> const &r, std::vector< point_t<S, D> > & p )
{
   if ( !details::has_intersection( q, r ) )
      return false;

   std::vector< point_t<S, D> > pin;  pin.reserve( 8 );
   std::vector< point_t<S, D> > pout; pout.reserve( 8 );
   
   pin.push_back( q[0] );
   pin.push_back( q[1] );
   pin.push_back( q[2] );
   pin.push_back( q[3] );
   pin.push_back( q[0] );

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

   p.assign( pout.begin(), pout.end()-1 );
   return true;
}

} // End of 'cg' namespace
