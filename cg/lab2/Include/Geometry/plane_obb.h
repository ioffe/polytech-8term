#pragma once
#include "convex_hull.h"
#include "primitives\range.h"

namespace cg
{
#pragma pack ( push, 1 )
  struct plane_obb
  {
    static point_2 rotate(point_2 const &v, double angle) // in radians
    {
        double cosa = cos(angle),
               sina = sin(angle);
        return point_2( v.x * cosa - v.y * sina,
                        v.x * sina + v.y * cosa );
    }

    naa_rect_2 rect;
    range_2    zrange;

    point_3 getPoint( size_t i ) const
    {
      Assert( i < 8 );
      double a  = rect.getangle( );
      double x1 = rect.getrect( ).x.lo( );
      double x2 = rect.getrect( ).x.hi( );
      double y1 = rect.getrect( ).y.lo( );
      double y2 = rect.getrect( ).y.hi( );
      double z1 = zrange.lo( );
      double z2 = zrange.hi( );
      switch ( i )
      {
      case 0:  return point_3( rotate( point_2( x1, y1 ), a ), z1 );
      case 1:  return point_3( rotate( point_2( x2, y1 ), a ), z1 );
      case 2:  return point_3( rotate( point_2( x1, y2 ), a ), z1 );
      case 3:  return point_3( rotate( point_2( x2, y2 ), a ), z1 );
      case 4:  return point_3( rotate( point_2( x1, y1 ), a ), z2 );
      case 5:  return point_3( rotate( point_2( x2, y1 ), a ), z2 );
      case 6:  return point_3( rotate( point_2( x1, y2 ), a ), z2 );
      case 7:  return point_3( rotate( point_2( x2, y2 ), a ), z2 );
      default: Assert( false ); return point_3( 0, 0, 0 );
      }
    }
  };

#pragma pack ( pop )


  inline void scale( cg::plane_obb & obb, double scale )
  {
    obb.rect.rect *= scale ;
    obb.zrange    *= scale ;
  }

  inline bool eq( const plane_obb & a, const plane_obb & b )
  {
    return eq( a.rect, b.rect ) && eq( a.zrange, b.zrange );
  }

  template< class Point > 
    inline void build_plane_obb( const Point * points, size_t nPoints, plane_obb & obb )
  {
    if ( nPoints > 0 )
    {
      build_naa_rect( points, nPoints, obb.rect );
      obb.zrange = range_2();
      for ( size_t i = 0; i < nPoints; ++i )
        obb.zrange |= points[i].z ;
    }
    else
    {
      obb.zrange = range_2(0);
      obb.rect.angle = 0.0;
      obb.rect.rect = rectangle_2( range_2(0), range_2(0) );
    }
  }


  inline plane_obb unite_obbs( const plane_obb * obbs, size_t nObbs )
  {
    std::vector< point_3 > points;
    points.reserve( nObbs * 8 );
    for ( size_t i = 0; i < nObbs; ++i )
    {
      for ( size_t j = 0; j < 8; ++j )
        points.push_back( obbs[i].getPoint( j ) );
    }
    plane_obb res;
    build_plane_obb( !points.empty() ? &points[0] : NULL, points.size( ), res );
    return res;
  }
}