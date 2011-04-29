#pragma once 

#include "geo_point_fwd.h"

#include "range.h"
#include "point.h"

#include <geometry\xmath.h>

namespace cg
{
   struct geo_point_2 ;
   struct geo_point_3 ;
   struct geo_rect_2 ; 
   struct geo_rect_3 ; 

   struct geo_pos_2 ; 

   geo_point_2 blend (geo_point_2 const & a, geo_point_2 const & b, double t) ; 
   geo_point_3 blend (geo_point_3 const & a, geo_point_3 const & b, double t) ; 

   geo_point_2 move_ref (geo_point_2 const & from, geo_point_2 const & to, geo_point_2 const & ref) ; 

   bool eq( geo_point_2 const & a, geo_point_2 const & b, double eps = epsilon< double >( ) );
   bool eq( geo_point_3 const & a, geo_point_3 const & b, double eps = epsilon< double >( ) );

   bool eq( geo_rect_2 const & a, geo_rect_2 const & b, double eps = epsilon< double >( ) );
   bool eq( geo_rect_3 const & a, geo_rect_3 const & b, double eps = epsilon< double >( ) );

   bool has_intersection( geo_rect_2 const& a, geo_rect_2 const& b ) ; 
   bool has_intersection( geo_rect_3 const& a, geo_rect_3 const& b ) ; 

   geo_rect_2 bounds ( geo_point_2 const * points, size_t count ) ; 

   point_2 as_metric(geo_point_2 const & gp ) ;  
   point_3 as_metric(geo_point_3 const & gp ) ;  

   geo_point_2 as_geo(point_2 const & mp ) ;  
   geo_point_3 as_geo(point_3 const & mp ) ;  

   bool line_cross_rect ( geo_point_2 const& p0, geo_point_2 const& p1, geo_rect_2 const& rect ) ; 
   bool point_in_zone ( geo_point_2 const& p, geo_point_2 const * points, size_t count ) ; 
}


namespace cg
{
   #pragma pack (push,1)

   struct geo_point_2
   {
      double lat, lon ; 
         
      geo_point_2 ()                         : lat ( 0     ), lon ( 0     ) {}
      geo_point_2 ( double lat, double lon ) : lat ( lat   ), lon ( lon   ) {}

      geo_point_2 const& as_2d () const { return *this ; } 
      geo_point_2 &      as_2d ()       { return *this ; } 
   } ; 

   struct geo_point_3 : geo_point_2
   {
      double height ; 

      geo_point_3 ()                                        : height ( 0 ) {}
      geo_point_3 ( double lat, double lon, double height ) : geo_point_2 ( lat, lon ), height ( height ) {}
      geo_point_3 ( const geo_point_2& p, double height   ) : geo_point_2 ( p ), height ( height ) {}

      geo_point_3 const& as_3d () const { return *this ; } 
      geo_point_3 &      as_3d ()       { return *this ; } 
   } ; 

   struct geo_pos_2 : geo_point_2
   {
      double course ; 

      geo_pos_2 ()                                        : course ( 0 ) {}
      geo_pos_2 ( double lat, double lon, double course ) : geo_point_2 ( lat, lon ), course ( course ) {}
      geo_pos_2 ( const geo_point_2& p, double course   ) : geo_point_2 ( p ), course ( course ) {}
   } ; 

   struct geo_rect_2
   {
      geo_point_2 beg, end;
     
      geo_rect_2 () 
      {
      }
      geo_rect_2 ( double lat0, double lon0, double lat1, double lon1 )   
         : beg ( lat0, lon0 ) 
         , end ( lat1, lon1 )
      {
      }    

      geo_rect_2 ( const geo_point_2& p0, const geo_point_2& p1 )   
         : beg ( p0 ) 
         , end ( p1 )
      {
      }

      geo_point_2 center() const
      {
         return blend ( beg, end, 0.5 ) ; 
      }

      bool contains( geo_point_2 const& p ) const
      {
         if ( !cg::range_2( beg.lat, end.lat ).contains( p.lat ) ) 
            return false ; 
         if ( !cg::range_2( beg.lon, end.lon ).contains( p.lon ) ) 
            return false ; 

         return true ;
      }

      geo_rect_2& operator |= ( geo_point_2 const& p ) 
      {
         make_min(beg.lat, p.lat) ; 
         make_min(beg.lon, p.lon) ; 
         make_max(end.lat, p.lat) ; 
         make_max(end.lon, p.lon) ; 

         return *this ; 
      }
   };

   struct geo_rect_3
   {
      geo_point_3 beg, end;
     
      geo_rect_3 () 
      {
      }
      geo_rect_3 ( double lat0, double lon0, double height0, double lat1, double lon1, double height1 )   
         : beg ( lat0, lon0, height0 ) 
         , end ( lat1, lon1, height1 )
      {
      }    

      geo_rect_3 ( const geo_point_3& p0, const geo_point_3& p1 )   
         : beg ( p0 ) 
         , end ( p1 )
      {
      }

      geo_point_3 center() const
      {
         return blend ( beg, end, 0.5 ) ; 
      }

      bool contains( geo_point_2 const& p ) const
      {
         if ( !cg::range_2( beg.lat, end.lat ).contains( p.lat ) ) 
            return false ; 
         if ( !cg::range_2( beg.lon, end.lon ).contains( p.lon ) ) 
            return false ; 

         return true ;
      }

      bool contains( geo_point_3 const& p ) const
      {
         if ( !contains(p.as_2d()) ) 
            return false ; 
         if ( !cg::range_2( beg.height, end.height ).contains( p.height ) ) 
            return false ; 

         return true ;
      }

      geo_rect_3& operator |= ( geo_point_3 const& p ) 
      {
         make_min(beg.lat,    p.lat) ; 
         make_min(beg.lon,    p.lon) ; 
         make_min(beg.height, p.height) ; 
         make_max(end.lat,    p.lat) ; 
         make_max(end.lon,    p.lon) ; 
         make_max(end.height, p.height) ; 

         return *this ; 
      }
   };

   
   #pragma pack (pop)
}


//////////////////////////////////////////////////////////////////////////
// Implementation 
namespace cg 
{
   inline geo_point_2 blend(geo_point_2 const & a, geo_point_2 const & b, double t)
   {
      return geo_point_2 ( blend(a.lat, b.lat, t), blend(a.lon, b.lon, t) ) ;  
   }

   inline geo_point_3 blend(geo_point_3 const & a, geo_point_3 const & b, double t)
   {
      return geo_point_3 ( blend(a.as_2d(), b.as_2d(), t), blend(a.height, b.height, t) ) ;  
   }

   inline geo_point_2 move_ref (geo_point_2 const & from, geo_point_2 const & to, geo_point_2 const & ref) 
   {
      return geo_point_2 ( from.lat + to.lat - ref.lat, from.lon + to.lon - ref.lon ) ;  
   }

   inline bool eq( geo_point_2 const & a, geo_point_2 const & b, double eps )
   {
      return eq( a.lat, b.lat, eps ) 
          && eq( a.lon, b.lon, eps );
   }

   inline bool eq( geo_point_3 const & a, geo_point_3 const & b, double eps )
   {
      return eq( a.lat,    b.lat,    eps ) 
          && eq( a.lon,    b.lon,    eps ) 
          && eq( a.height, b.height, eps );
   }

   inline bool eq( geo_rect_2 const & a, geo_rect_2 const & b, double eps )
   {
      return eq ( a.beg, b.beg, eps ) && eq ( a.end, b.end, eps ) ; 
   }

   inline bool eq( geo_rect_3 const & a, geo_rect_3 const & b, double eps )
   {
      return eq ( a.beg, b.beg, eps ) && eq ( a.end, b.end, eps ) ; 
   }

   inline bool has_intersection( geo_rect_2 const& a, geo_rect_2 const& b ) 
   {
      return has_intersection( range_2(a.beg.lat, a.end.lat), range_2(b.beg.lat, b.end.lat) ) && 
             has_intersection( range_2(a.beg.lon, a.end.lon), range_2(b.beg.lon, b.end.lon) ) ; 
   }

   inline bool has_intersection( geo_rect_3 const& a, geo_rect_3 const& b ) 
   {
      return has_intersection( range_2(a.beg.lat, a.end.lat), range_2(b.beg.lat, b.end.lat) ) && 
             has_intersection( range_2(a.beg.lon, a.end.lon), range_2(b.beg.lon, b.end.lon) ) && 
             has_intersection( range_2(a.beg.height, a.end.height), range_2(b.beg.height, b.end.height) ) ; 
   }

   inline geo_rect_2 bounds ( geo_point_2 const * points, size_t count ) 
   {
      geo_rect_2 rect ( points[0], points[0] ) ; 
      for ( size_t i = 1 ; i < count ; i ++ ) 
      {
         rect.beg.lat = min(rect.beg.lat, points[i].lat) ; 
         rect.beg.lon = min(rect.beg.lon, points[i].lon) ; 
         rect.end.lat = max(rect.end.lat, points[i].lat) ; 
         rect.end.lon = max(rect.end.lon, points[i].lon) ; 
      }
      return rect ; 
   }

   inline geo_rect_3 bounds ( geo_point_3 const * points, size_t count ) 
   {
      geo_rect_3 rect ( points[0], points[0] ) ; 
      for ( size_t i = 1 ; i < count ; i ++ ) 
      {
         rect.beg.lat    = min(rect.beg.lat,    points[i].lat) ; 
         rect.beg.lon    = min(rect.beg.lon,    points[i].lon) ; 
         rect.beg.height = min(rect.beg.height, points[i].height) ; 
         rect.end.lat    = max(rect.end.lat,    points[i].lat) ; 
         rect.end.lon    = max(rect.end.lon,    points[i].lon) ; 
         rect.end.height = max(rect.end.height, points[i].height) ; 
      }
      return rect ; 
   }

   inline point_2 as_metric(geo_point_2 const & gp ) 
   {
      return point_2 ( gp.lon, gp.lat ) ; 
   }

   inline point_3 as_metric(geo_point_3 const & gp )  
   {
      return point_3 ( gp.lon, gp.lat, gp.height ) ; 
   }

   inline geo_point_2 as_geo(point_2 const & mp ) 
   {
      return geo_point_2 ( mp.y, mp.x ) ; 
   }

   inline geo_point_3 as_geo(point_3 const & mp )  
   {
      return geo_point_3 ( mp.y, mp.x, mp.z ) ; 
   }

   inline bool line_cross_rect ( geo_point_2 const& p0, geo_point_2 const& p1, geo_rect_2 const& rect )  
   {
      if (  p0.lat < rect.beg.lat && p1.lat < rect.beg.lat
         || p0.lat > rect.end.lat && p1.lat > rect.end.lat
         || p0.lon < rect.beg.lon && p1.lon < rect.beg.lon
         || p0.lon > rect.end.lon && p1.lon > rect.end.lon ) 
         return false ;

      double nlat = - (p1.lon - p0.lon) ; 
      double nlon = + (p1.lat - p0.lat) ; 

      double dlat0 = rect.beg.lat - p0.lat ;
      double dlon0 = rect.beg.lon - p0.lon ;
      double dlat1 = rect.end.lat - p0.lat ;
      double dlon1 = rect.end.lon - p0.lon ;

      double d0 = dlat0 * nlat + dlon0 * nlon ; 
      double d1 = dlat1 * nlat + dlon0 * nlon ; 
      double d2 = dlat1 * nlat + dlon1 * nlon ; 
      double d3 = dlat0 * nlat + dlon1 * nlon ; 

      if ( d0 > 0 && d1 > 0 && d2 > 0 && d3 > 0 ) 
         return false ;

      if ( d0 < 0 && d1 < 0 && d2 < 0 && d3 < 0 ) 
         return false ;

      return true ;
   }

   inline bool point_in_zone ( geo_point_2 const& p, geo_point_2 const * points, size_t count ) 
   {
      if ( !count ) 
         return false ; 

      double lat = p.lat ; 
      double lon = p.lon ; 

      int horCrossings = 0;
      int verCrossings = 0;

      // note. were don't care of result, if point is on the edge of polygon
      for( size_t i = 0; i < count; i++)
      {
         const geo_point_2 &A = points[i];
         const geo_point_2 &B = points[(i+1)%count];

         if ( (lon >= A.lon && lon < B.lon || lon <= A.lon && lon > B.lon) && (A.lat > lat || B.lat > lat) ) 
         {
            double yy = A.lat + (lon - A.lon)*(B.lat - A.lat)/(B.lon - A.lon) - lat; 
            if ( yy > 0 ) 
               verCrossings ++ ; 
         }

         if ( (lat >= A.lat && lat < B.lat || lat <= A.lat && lat > B.lat) && (A.lon > lon || B.lon > lon) ) 
         {
            double xx = A.lon + (lat - A.lat)*(B.lon - A.lon)/(B.lat - A.lat) - lon ; 
            if ( xx > 0 ) 
               horCrossings ++ ; 
         }
      }

      return ((horCrossings & 1) && (verCrossings & 1));  
   }

}   
