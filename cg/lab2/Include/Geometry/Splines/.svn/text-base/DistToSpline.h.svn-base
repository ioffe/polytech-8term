#pragma once

#include "HermiteSplines.h"
#include "NaturalSpline.h"


#include "geometry\convex_hull.h"
#include "geometry\grid2l.h"
#include "geometry\grid2l\subdiv.h"

#include "geometry\primitives\segment.h"

#include "contours\common.h"

#include "contours\dtc\dtc_standard.h"

#include "contours\holder\builder.h"
#include "contours\holder\accessors.h"

#include "contours\algorithm\buildDistToContours.h"
#include "contours\algorithm\getDistToContours.h"

#include "contours\misc\smallcell.h"

#include <boost\shared_ptr.hpp>
#include <boost\scoped_ptr.hpp>

namespace cg
{
   namespace algos
   {
      namespace details
      {
         namespace dtc
         {

            using cg::contours::contour_id;
            using cg::contours::point_id;
            using cg::contours::segment_id;

            typedef
               cg::contours::dtc::standard::SCell
               smallcell_type;

            typedef
               cg::contours::dtc::standard::Grid
               grid_type;

            template < class point_type >
               struct length_point_type
                  : point_type 
            {
               length_point_type( point_type const & point, double length )
                  : length_( length )
                  , point_type( point )
               {}

               double length( ) const { return length_; }

            private:
               double length_;
            };
         }

         template <class Derived, class Grid, class Point, class ContourAttr>
         struct CDistToContours
         {
            typedef cg::contours::algos::DistToContoursResult<Derived> DTCResult;

            void createDist2ContoursGrid ( rectangle_2 const &scene_bb )
            {
               grid2.reset( cg::contours::_algos::buildDistToCoast< Grid >( self( ), scene_bb ) );
            }

            typedef cg::contours::algos::DistToContoursResult<Derived> DTCResult;

            DTCResult getDist2Contours( point_2 const &p ) const
            {
               return cg::contours::algos::getDistToContours( p, self(), &grid() );
            }

            typedef Grid grid_type;

            Grid const & grid () const { return *grid2; }
            Grid       & grid ()       { return *grid2; }

            void setGrid (Grid * g) { grid2.reset( g ); }

         protected:
            boost::scoped_ptr< Grid > grid2;

         private:
            Derived const & self() const { return static_cast<Derived const &>(*this); }
         };

         template <class point_type>
            struct dist2spline
            :   cg::contours::holder::Accessors <cg::contours::holder::Builder <dtc::length_point_type< point_type >, cg::contours::contour<cg::Empty> > > 
            ,   CDistToContours <dist2spline<point_type>, dtc::grid_type, dtc::length_point_type< point_type >, cg::Empty>
         {
            template < class FwdIter >
               void assign( FwdIter p, FwdIter q )
            {
               addContourChecked( p, q, cg::Empty( ) );
               process( );
            }

         private:
            void process( )
            {
               rectangle_2 
                  domain( cg::bbox );

               calculate_domain( domain );

               cg::Grid2LSubdiv 
                  subdiv( domain, 30, 10, 12 );

               grid2.reset( new cg::Grid2LInitializer< grid_type >( segmentsBegin(), segmentsEnd(), *this, subdiv ) );

               createDist2ContoursGrid( domain );
            }

         private:
            void calculate_domain( rectangle_2 & rect )
            {
               for ( point_id pid = pointsBegin( ); pid != pointsEnd( ); ++pid )
                  rect.unite( getPoint( pid ) );

               if ( cg::valid( rect ) )
                  rect.inflate( 1. );
            }
         };

    }
      template < class NaturalSpline >
         struct DistToSpline2d
      {
         typedef
            typename NaturalSpline::point_type  
            point_type;

         typedef
            std::vector< details::dtc::length_point_type< point_type > >
            points_type;

         DistToSpline2d( NaturalSpline const & spline, double step )
            : spline_ ( spline )
         {
            points_type points;
            int steps = cg::ceil ( spline_.Length() / step ) ;
                step  = spline_.Length() / steps ;

            points.reserve( steps );

            for ( int i = 0; i <= steps; ++i )
            {
               points.push_back( details::dtc::length_point_type< point_type >( spline_.Interpolate( i * step ), i * step ) );
            }

            d2s_.assign( points.begin( ), points.end( ) );
         }

         double dist_to_spline( point_type const & point, double *closestPointLen = 0 ) 
         {
            details::dist2spline<point_type>::DTCResult res = d2s_.getDist2Contours( point );

            if ( res.SegId() == d2s_.defaultSegmentId())  // point is out of scene
               return 1e20;

            if ( closestPointLen )
            {
               double l = d2s_.getPoint ( d2s_.getSegmentStartPoint( res.SegId())).length() ;
               double r = d2s_.getPoint ( d2s_.getSegmentEndPoint  ( res.SegId())).length() ;

               *closestPointLen = ( l + r ) / 2 ;
               spline_.GetDistance2SplineArea( point, *closestPointLen, ( r - l )) ;
            }

            return res.Dist();
         }

      private:

         details::dist2spline<point_type> d2s_;
         NaturalSpline                    spline_ ; // should have a valid copeing constructor 
         

      };
   }

   template <class Point>
   struct DistToSpline
   {
      const static int MIN_EDGE_LEN   = 20;
      const static int MAX_EDGE_LEN   = 20;
      const static int MIN_ANGLE      = 10;

      DistToSpline( HermiteSplineManager<Point> const &s, double max_epsilon=1.0 ) // max_epsilon - максимальная погрешность в ответах (в метрах)
         : splineLen ( -1 )
      {
         if (max_epsilon < 1.0)
            max_epsilon = 1.0;


         double delta = MAX_EDGE_LEN*max_epsilon ; 
         NaturalSpline<Point> natspline ( s, 1 ) ; 

         DWORD segm_count = (DWORD)(natspline.Length() / delta)  ; 
         delta = natspline.Length() / segm_count ; 

         segments.resize(segm_count+1) ; 
         for ( size_t i = 0 ; i < segm_count+1 ; i ++ ) 
            segments[i] = natspline.Interpolate ( i * delta ) ; 

         //            s.SetDivisionParams( MIN_EDGE_LEN*max_epsilon, MAX_EDGE_LEN*max_epsilon, MIN_ANGLE*max_epsilon );
         //            s.ReCalcDivision( segments, true );

         Assert( segments.size() > 1 );
         origin = segments[0];

         // создаем новый (подробный) сплайн
         spline.Clear();
         for (size_t i=0; i<segments.size(); ++i)
            spline.PushBack(segments[i]);

         spline.SetRightDirection ( 0, (spline.GetPoint(1) - spline.GetPoint(0)) / 4 ) ; 
         CalcLength();
      }

      ~DistToSpline()
      {
         spline.Clear();
      }

      // по заданной точке находит ближайшее расстояние до сплайна, а также ближайшую точку на сплайне (определяется параметром t)
      double GetDist( Point const &p, double &t ) const
      {
         double mindist = 1e30;
         double ratio   = -1.0;
         for (size_t i=0; i<segments.size()-1; ++i)
         {
            point_3 p1( segments[i] );
            point_3 p2( segments[i+1] );
            segment_3 seg( p1, p2 );

            //                segment_2 seg(segments[i], segments[i+1]);
            double cur_t = cg::bound( seg(point_3(p)), 0., 1. ) ;   
            double dist = distance_sqr(seg(cur_t), point_3(p));
            if (mindist > dist)
            {
               mindist = dist;
               ratio   = i + cur_t;
            }
         }
         t = ratio;
         return cg::sqrt(mindist);
      }

      // возвращает НАПРАВЛЕННОЕ расстояние до сплайна (справа - больше нуля, слева - меньше)
      double GetDDist( Point const &p, double &t, bool &on_right ) const
      {
         double mindist = 1e20;
         on_right = true;
         double ratio   = -1.0;
         for (size_t i=0; i<segments.size()-1; ++i)
         {
            point_3 p1( segments[i] );
            point_3 p2( segments[i+1] );
            segment_3 seg( p1, p2 );

            double cur_t = cg::bound( seg(point_3(p)), 0., 1. ) ;   
            double dist = distance_sqr(seg(cur_t), point_3(p));
            if (mindist > dist)
            {
               mindist  = dist;
               // HACK: в нашем трехмерном пространстве понятия "лево" и "право"
               //       определяются БЕЗ учитывания pitch и roll...
               on_right = ((point_2(seg.P1()) - point_2(seg.P0())) ^ (point_2(p) - point_2(seg.P0()))) > 0.;
               ratio    = i + cur_t;
            }
         }
         t = ratio;
         return cg::sqrt(mindist);
      }

      Point GetDirection ( double t ) const
      {
         return spline.Direction( t );
      }


      // HACK: эта нормаль - опять-таки в горизонтальной плоскости!!
      Point GetNormal(double t) const
      {
         int & * a ; 

         Point dir = spline.SecondDerivative(t);//spline.Derivative(t);
         Point res = dir;

         //            *static_cast<point_2*>( &res ) = normal( static_cast<point_2 const &>( dir ) );

         res = normalized( res );

         return res;
      }

      Point Derivative(double t) const
      {
         Point dir = spline.Derivative(t);
         return normalized( dir );
      }

      Point Derivative_(double t) const
      {
         return spline.Derivative(t);
      }

      Point SecondDerivative_(double t) const
      {
         return spline.SecondDerivative(t);
      }

      Point Interpolate(double t) const
      {
         return spline.Interpolate(t);
      }

      /*        void GetPointByIdx( int idx, double &x, double &y )
      {
      x = segments[idx].x;
      y = segments[idx].y;
      }
      */
      double Length() const
      {
         return splineLen;
      }

      size_t size() const
      {
         return spline.GetNumPoints();
      }

   private:
      void CalcLength()
      {
         splineLen = 0.001;
         for (size_t i=0; i<segments.size()-1; ++i)
            splineLen += norm(segments[i+1] - segments[i]);
      }

   private:
      HermiteSplineManager<Point> spline;
      std::vector<Point>          segments;
      Point                       origin;

      double                      splineLen;
   };

   inline point_2 DistToSpline< point_2 > :: GetNormal (double t) const
   {
      return normal ( spline.Direction( t ) ) ;
   }      
     
}
