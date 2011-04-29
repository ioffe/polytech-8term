#pragma once 

#include "Geometry\primitives\segment.h"
#include "Geometry\segment_2_intersection.h"

namespace cg 
{
   // TODO : angle and like that to namespace 
   namespace CircleSmoothing 
   {
      typedef std::vector< point_2 >   Points ;
      typedef double                   AngleT ;
      typedef std::vector< AngleT >    Angles ;  
                         
      struct RefPoint
      {
         enum  RollType 
         {
            RT_CW    ,  // clock wise
            RT_CCW   ,  // counter clock wise
            RT_LINE  ,

            RT_NONE
         };

         int type ;
         
         point_2 p0 ;   // first point
         point_2 p1 ;   // center in case of CCW or CW / end of segment in case of LINE 
               
         RefPoint() : type( RT_NONE ) {}
         
         RefPoint( int type, point_2 const& p0, point_2 const&p1 )
            : type ( type )
            , p0 ( p0 )
            , p1 ( p1 )
         {
         }
      };

      typedef 
         std::vector< RefPoint >
         RefPoints ;

      inline double Angle( double R, point_2 const& p1, point_2 const& p2 )  
      {
         point_2 dp = ( p2 - p1 ) / 2 ;

         if( norm ( dp ) > R )     // circle couldn't lie on these points 
            return 180 + epsilon<double>() ;

         point_2 norm    = normalized_safe( point_2 ( - dp.y, dp.x )) ;
         double  normLen = cg::sqrt( sqr( R ) - norm_sqr( dp )) ;

         point_2 radVector = dp + norm * normLen ; // unusual, from point to center 
         return norm360( rad2grad( atan2( radVector.y, - radVector.x ))) ; 
      }

      inline double Angle( double R, point_2 const& p, segment_2 const& seg, double& ratio )  
      {
         double pointsAngle = 180 ;

         point_2 pProj = seg( seg( p )) ;
         double  dist  = norm( p - pProj ) ;
         // dist + A = R_ 
         // A^2 + B^2 = R_^2

         double A = R - dist ;
         if ( abs( A ) > abs( R )) // circle doesn't touch the line 
         {
            ratio = 0 ;
            return 180 + epsilon<double>() ;
         }

         double B = cg::sqrt( sqr( R ) - sqr( A )) / norm( seg.P1() - seg.P0()) ;
         int    s = sign(( p - pProj ) * normal( seg )) ;

         ratio = bound( seg( p ) + s * B, 0., 1. ) ;

         point_2 refPoint = clamp_d( 0., 1., seg.P0(), seg.P1())( ratio ) ;

         if ( norm( p - refPoint ) < 2 * R )
            pointsAngle = min( pointsAngle, Angle( R, p, refPoint )) ;

         return pointsAngle ;
      }

      inline segment_2 ShiftedSegment( point_2 const& p0, point_2 const& p1, double offset ) // along normal 
      {
         point_2 dir = normalized_safe( p1 - p0 ) ;
         point_2 norm( - dir.y, dir.x ) ;

         point_2 shift = norm * offset ; 

         return segment_2( p0 + shift, p1 + shift ) ;
      }


      inline void UpSmoothing( Points const& points, double R, RefPoints& refPoints ) 
      {
         Assert( points.size() > 1 ) ;

         refPoints.clear() ;
         
         double stopAt    = points.size() - 1 ;
         double startFrom = 0 ;

         while( !cg::eq( startFrom, stopAt ))
         {
            bool     couldRoll  = true ;
            unsigned iStartFrom = ( unsigned ) startFrom ; 
            
            if ( cg::eq( double( iStartFrom + 1 ), startFrom )) // solving problems with rounding
               iStartFrom = iStartFrom + 1 ;

            segment_2 seg ( points[ iStartFrom ], points[ iStartFrom + 1 ] ) ;

            point_2   startPoint = seg( startFrom - iStartFrom ) ;
            segment_2 firstSeg   ( startPoint, points[ iStartFrom + 1 ] ) ;

            point_2   segNorm    = normalized_safe( normal( seg )) ;
            segment_2 normDiam   ( startPoint, startPoint + 2 * R * segNorm ) ;

            double   nextRefPoint = iStartFrom + 1 ;

            unsigned cp = iStartFrom + 1 ; // current point 

            //-- searching the next reference point as we couldn't roll by segment starting from "startFrom"
            if ( iStartFrom == startFrom )
            {
               double angleLimit = norm360( rad2grad( atan2( segNorm.y, - segNorm.x ))) ; // limitation by first segment  

               double minAngle   = 180 ;
               while( cp < points.size() - 1 && abs( startPoint.x - points[ cp ].x ) <= 2 * R  )
               {
                  double ratio    = 0 ;
                  double curAngle = Angle( R, startPoint, segment_2( points[ cp ], points[ cp + 1 ] ), ratio ) ;

                  if ( minAngle >= curAngle )
                  {
                     minAngle     = curAngle   ;
                     nextRefPoint = cp + ratio ;
                  }

                  ++ cp ;
               }

               // couldn't roll
               if ( minAngle <= angleLimit )
               {
                  couldRoll = false ; 

                  double  radAngle = grad2rad( minAngle ) ;
                  point_2 radVect ( - cos( radAngle ), sin( radAngle )) ; // unusual, from point to center 
                  point_2 center = startPoint + R * radVect ;

                  refPoints.push_back( RefPoint( RefPoint::RT_CCW, startPoint, center )) ;
               }
            }


            if ( couldRoll )
            {
               // rolling by the first segment, divides into to steps:
               // 1. searching for the closest point to the circle ( ends of segment )
               // 2. searching for the next reference segment 

               // 1. closest point 
               double firstSegLen  = norm( firstSeg.P1() - firstSeg.P0()) ;
               nextRefPoint = iStartFrom + 1 ;

               double minDist = firstSegLen ;

               cp = iStartFrom + 2 ;
               while( cp < points.size() && abs( points[ iStartFrom + 1 ].x - points[ cp ].x ) <= 2 * R  )
               {
                  double proj = 2 * normDiam( points[ cp ] ) - 1 ; 
                  if ( abs( proj ) <= 1 )
                  {
                     double dist = distance( normDiam, points[ cp ] ) - R * cg::sqrt( 1 - sqr( proj ));
                     if ( dist <= minDist )
                     {
                        nextRefPoint = cp   ;
                        minDist      = dist ;
                     }
                  }

                  ++ cp ;
               }

               // 2. next reference segment
               cp = iStartFrom + 1 ;
               segment_2 firstShSeg  = ShiftedSegment( startPoint, points[ iStartFrom + 1 ], R ) ;

               while( cp < points.size() - 1 && abs( points[ iStartFrom + 1 ].x - points[ cp ].x ) <= 2 * R  )
               {
                  segment_2 shSeg = ShiftedSegment( points[ cp ], points[ cp + 1 ], R ) ;

                  point_2 pt ;
                  if ( intersect == generic_intersection<point_2>( firstShSeg, shSeg, &pt, 0, cg::epsilon< double >()))
                  {
                     double dist = firstShSeg( pt ) * firstSegLen ;
                     if ( dist <= minDist )
                     {
                        minDist      = dist ;
                        nextRefPoint = cp + shSeg( pt ) ;
                     }
                  }

                  ++ cp ;
               }

               if( minDist < firstSegLen ) // that means we didn't reach end of first segment 
               {
                  point_2 stopPoint = firstSeg( bound( minDist / firstSegLen, 0., 1. )) ;
                  point_2 center    = stopPoint + segNorm * R ;

                  refPoints.push_back( RefPoint( RefPoint::RT_LINE, firstSeg.P0(), stopPoint))  ;
                  refPoints.push_back( RefPoint( RefPoint::RT_CCW , stopPoint, center )) ;
               }
               else
                  refPoints.push_back( RefPoint( RefPoint::RT_LINE, firstSeg.P0(), firstSeg.P1())) ;

            }

            startFrom = nextRefPoint ;
         }
      
         refPoints.push_back( RefPoint( RefPoint::RT_LINE, points.back(), points.back())) ;
      }

     inline void DownSmoothing( RefPoints const& refPoints, double Rd, RefPoints& extrPoints )
     {
         Assert( refPoints.size() > 1 ) ;

         extrPoints.clear() ;
         for( unsigned i = 0 ; i < refPoints.size() - 1; ++ i )
         {
            point_2 startsWith, endsWith ;

            switch( refPoints[ i ].type ) 
            {
            case RefPoint::RT_LINE:
               {
                  point_2 dir  = normalized( refPoints[ i ].p1 - refPoints[ i ].p0 ) ; 
                  point_2 norm ( - dir.y, dir.x ) ;
                  extrPoints.push_back( RefPoint( RefPoint::RT_LINE, refPoints[ i ].p0 + Rd * norm, refPoints[ i ].p1 + Rd * norm )) ;

                  endsWith = extrPoints.back().p1 ; 
               }
               break ;
            case RefPoint::RT_CCW :
               {
                  point_2 radVector = normalized( refPoints[ i ].p1 - refPoints[ i ].p0 ) ; // unusual dir, from point to center 
                  extrPoints.push_back( RefPoint( RefPoint::RT_CCW, refPoints[ i ].p0 + radVector * Rd, refPoints[ i ].p1 )) ;  

                  point_2 secRadVector = normalized( refPoints[ i ].p1 - refPoints[ i + 1 ].p0 ) ;
                  endsWith = refPoints[ i + 1 ].p0 + secRadVector * Rd ;
               }
               break ;
            }

            if ( i < refPoints.size() - 2 )
            {
               switch( refPoints[ i + 1 ].type ) 
               {
               case RefPoint::RT_LINE:
                  {
                     point_2 dir  = normalized( refPoints[ i + 1 ].p1 - refPoints[ i + 1 ].p0 ) ; 
                     point_2 norm ( - dir.y, dir.x ) ;
                     startsWith = refPoints[ i + 1 ].p0 + Rd * norm ;
                  }
                  break ;
               case RefPoint::RT_CCW :
                  {
                     point_2 radVector = normalized( refPoints[ i + 1 ].p1 - refPoints[ i + 1 ].p0 ) ; // unusual dir, from point to center 
                     startsWith = refPoints[ i + 1 ].p0 + radVector * Rd ;
                  }
                  break ;
               }
            }
            
            // WARNING, epsilon 
            if( !cg::eq( startsWith, endsWith, .001 ) || i == refPoints.size() - 2 )
               extrPoints.push_back( RefPoint( RefPoint::RT_CW, endsWith, refPoints[ i + 1 ].p0 )) ;

         }

         for( unsigned i = 0; i < extrPoints.size(); ++ i )
         {
            extrPoints[ i ].p0.y -= Rd ;
            extrPoints[ i ].p1.y -= Rd ;
         }
      }

      inline void GetEndPointLine( RefPoint const& refPoint, double& a, double& b ) 
      {
         point_2 lineDir ;
         if( refPoint.type == RefPoint::RT_CW || refPoint.type == RefPoint::RT_CCW )
         {
            point_2 radVector = refPoint.p1 - refPoint.p0 ;
            lineDir   = point_2 ( - radVector.y, radVector.x ) ;
         }
         else if ( refPoint.type == RefPoint::RT_LINE )
            lineDir = refPoint.p1 - refPoint.p0 ; 

         a = lineDir.y / lineDir.x ;
         b = - a * refPoint.p0.x + refPoint.p0.y ;                  
      }

      inline void DragPoints( RefPoints const& refPoints, Points& points, Angles& angles )
      {
         unsigned currRefPoint = 0 ; 
         unsigned currPoint    = 0 ;
         
         while( currPoint < points.size() && points[ currPoint ].x < refPoints[ 0 ].p0.x )
            ++ currPoint ;

         // dragging first points
         double a, b ;
         GetEndPointLine( refPoints[ 0 ], a, b ) ;
         for ( unsigned i = 0; i < currPoint ; ++ i )
         {
            double newY = a * points[ i ].x + b ; 
            Assert( le( points[ i ].y, newY )) ;
            points[ i ].y = newY ;
            angles[ i ]   = cg::rad2grad( atan( a )) ;
         }
         
         while( currRefPoint < refPoints.size() - 1 ) 
         {
            RefPoint const& refPoint = refPoints[ currRefPoint ] ;  

            double   radius ;
            point_2  center ;

            double a, b   ;

            switch( refPoint.type )
            {
            case RefPoint::RT_CCW :
            case RefPoint::RT_CW:
               center = refPoint.p1 ;
               radius = cg::norm( refPoint.p0 - center ) ;  
               break ;
            case RefPoint::RT_LINE:
               a = ( refPoint.p1.y - refPoint.p0.y ) / ( refPoint.p1.x - refPoint.p0.x ) ;
               b = - a * refPoint.p0.x + refPoint.p0.y ;
               break ;
            default:
               Assert( 0 ) ;
            }
            
            while( currPoint < points.size() && points[ currPoint ].x < refPoints[ currRefPoint + 1 ].p0.x )
            {  
               point_2& pt    = points[ currPoint ] ;
               double newY    = -1 ;  
               double& angle  = angles[ currPoint ] ;

               // dragging point 
               switch( refPoint.type )
               {
               case RefPoint::RT_CCW :
                  {
                     double angleCos = cg::abs(( pt.x - center.x ) / radius ) ;
                     Assert( abs ( angleCos ) <= 1 ) ;
                     
                     newY  = center.y - radius * sin( acos( angleCos )) ;
                     Assert( le( pt.y, newY )) ;
                     pt.y = newY ;
                                          
                     point_2 dir = cg::normalized_safe( center - pt ) ;
                     dir = point_2( dir.y, -dir.x ) ; // turn left
                     
                     angle = cg::rad2grad( atan2( dir.y, dir.x )) ;
                  }
                  break ;
               case RefPoint::RT_CW:
                  {
                     double angleCos = cg::abs(( pt.x - center.x ) / radius ) ;
                     Assert( abs ( angleCos ) <= 1 ) ;
                     
                     newY  = center.y + radius * sin( acos( angleCos )) ;
                     Assert( le( pt.y, newY )) ;
                     pt.y = newY ;
                     
                     point_2 dir = cg::normalized_safe( center - pt ) ;
                     dir = point_2( -dir.y, dir.x ) ; // turn right
                     
                     angle = cg::rad2grad( atan2( dir.y, dir.x )) ;
                  }
                  break ;
               case RefPoint::RT_LINE:
                  newY  = a * pt.x + b ;
                  
                  Assert( le( pt.y, newY )) ;
                  pt.y = newY ;

                  angle = cg::rad2grad( atan( a )) ; 
                  break ;

               default:
                  Assert( 0 ) ;
               }

               ++ currPoint ;
            }
         
            ++ currRefPoint ;
         }

         // dragging last points 
         if( currRefPoint < refPoints.size() && currPoint < points.size())
         {
            double a, b ;
            GetEndPointLine( refPoints[ currRefPoint ], a, b ) ;
            for ( unsigned i = currPoint; i < points.size() ; ++ i )
            {
               double newY = a * points[ i ].x + b ; 
               Assert( le( points[ i ].y, newY )) ;
               points[ i ].y = newY ;
               angles[ i ]   = cg::rad2grad( atan( a )) ;
            }
         }
      }
   } // namespace CircleSmoothing 
} // namespace cg 