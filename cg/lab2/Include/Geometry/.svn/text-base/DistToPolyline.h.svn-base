#pragma once

#include "convex_hull.h"

#include "grid2l.h"
#include "grid2l\subdiv.h"

#include "contours\common.h"

#include "contours\dtc\dtc_standard.h"

#include "contours\holder\builder.h"
#include "contours\holder\accessors.h"

#include "contours\algorithm\buildDistToContours.h"
#include "contours\algorithm\getDistToContours.h"

#include "contours\misc\smallcell.h"

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
            struct dist2polyline
               :   cg::contours::holder::Accessors <cg::contours::holder::Builder <point_type, cg::contours::contour<cg::Empty> > > 
               ,   CDistToContours <dist2polyline, dtc::grid_type, point_type, cg::Empty>
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
                     domain;

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
                     rect |= getPoint( pid ) ;

                  if ( !rect.empty() )
                     rect.inflate( 1. );
               }
            };
         }
      }
      struct DistTo2DPolyline
      {
         typedef
            point_2 
            point_type ;

         typedef
            std::vector< point_type >
            polyline_type ;

         DistTo2DPolyline( polyline_type const & polyline )
         {
            d2p_.assign( polyline.begin( ), polyline.end( ) );
         }

         double dist_to_polyline ( point_type const &point, point_type &closest_point ) 
         {
            details::dtc::dist2polyline<point_type>::DTCResult res = d2p_.getDist2Contours( point );

            if ( res.SegId() == d2p_.defaultSegmentId())  // point is out of scene
               return 1e20;

            closest_point = res.Closest();
            return res.Dist();
         }

      private:
         details::dtc::dist2polyline<point_type> d2p_;
      };

   }
 }
