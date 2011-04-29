#pragma once

#include "geometry\grid2l.h"
#include "geometry\grid2l\subdiv.h"
#include "contours\misc\smallcell.h"

namespace cg {

   namespace details {

      template< typename segments_type, typename grid_type, typename segment_id_type >
      struct SegmentsTraits
      {
         typedef     typename segments_type :: value_type      segment_type;

         SegmentsTraits( segments_type const & segments )
            : segments_( segments )
         {}

         segment_id_type segmentsBegin( ) const { return 0; }
         segment_id_type segmentsEnd  ( ) const { return ( segment_id_type ) segments_.size( ); }

         segment_id_type distance( segment_id_type a, segment_id_type b ) const
         {
            return abs( a -  b );
         }

         cg::rectangle_2 getBbox( segment_id_type id ) const
         {
            return cg::rectangle_2( segments_.at( id ).P0( ), segments_.at( id ).P1( ) );
         }

         segment_type const & getItem( segment_id_type id  ) const
         {
            return segments_.at( id );
         }

      private:
         segments_type const & segments_;
      };

      template< typename segment_id_type, typename point_type >
         struct SegmentsIntersection
      {
         SegmentsIntersection( segment_id_type id1, segment_id_type id2, point_type const & p,
                               intersection_type type )
            : id1_( std::min( id1, id2 ) )   // IMPORTANT! Don't touch with your playful hands!
            , id2_( std::max( id1, id2 ) )   // IMPORTANT! Don't touch with your playful hands!
            , p_  ( p )
            , type_( type )
         {}

         segment_id_type      id1( )   const { return id1_; }
         segment_id_type      id2( )   const { return id2_; }
         point_type const &   p( )     const { return p_; }
         intersection_type    type()   const { return type_; }

      private:
         segment_id_type   id1_;
         segment_id_type   id2_;
         point_type        p_;
         intersection_type type_;
      };

      template < typename segments_type, typename intr_type, typename OutIter >
         struct IntersectionsProcessor
      {
         typedef typename segments_type::value_type   segment_type;
         typedef typename segment_type::point_type    point_type;
         typedef typename point_type::scalar_type     scalar_type;
         typedef IntersectionsProcessor               BigCellProcessor;
         typedef IntersectionsProcessor               SmallCellProcessor;

         IntersectionsProcessor( segments_type const & segments, OutIter output, scalar_type eps = 1e-10 )
            : segments_( segments )
            , output_  ( output )    
            , eps_( eps )
         {}

         template< typename State, typename SmallCell >
            bool operator( )( State const &, SmallCell const & cell )
         {
            typedef     typename SmallCell :: segment_id           segment_id_type; 

            segment_id_type size = ( segment_id_type ) cell.segments( ).size( );
            for( segment_id_type i = 0; i != size; ++i )
            {
               for( segment_id_type j = i + 1; j != size; ++j )
               {
                  const segment_id_type id1 = cell.segments( ).at( i );
                  const segment_id_type id2 = cell.segments( ).at( j );

                  const segment_type & seg1 = segments_.at( id1 );
                  const segment_type & seg2 = segments_.at( id2 );

                  // TODO: This is probably incorrect for segments with near epsilon-length.
                  const scalar_type eps2 = cg::sqr( eps_ );
                  const bool seg1P0CloseToSeg2 = ( distance_sqr( seg1.P0( ), seg2 ) <= eps2 );
                  const bool seg1P1CloseToSeg2 = ( distance_sqr( seg1.P1( ), seg2 ) <= eps2 );
                  const bool seg2P0CloseToSeg1 = ( distance_sqr( seg2.P0( ), seg1 ) <= eps2 );
                  const bool seg2P1CloseToSeg1 = ( distance_sqr( seg2.P1( ), seg1 ) <= eps2 );
                  
                  size_t const closePointsNum = 
                     size_t( seg1P0CloseToSeg2 ) + size_t( seg1P1CloseToSeg2 ) + 
                     size_t( seg2P0CloseToSeg1 ) + size_t( seg2P1CloseToSeg1 );

                  if ( closePointsNum > 0 )
                  {
                     if ( closePointsNum == 1 )
                     {
                        const intersection_type intr = cg::intersect;

                        if ( seg1P0CloseToSeg2 )
                           *output_++ = intr_type( id1, id2, seg1.P0( ), intr );
                        else if ( seg1P1CloseToSeg2 )
                           *output_++ = intr_type( id1, id2, seg1.P1( ), intr );
                        else if ( seg2P0CloseToSeg1 )
                           *output_++ = intr_type( id1, id2, seg2.P0( ), intr );
                        else if ( seg2P1CloseToSeg1 )
                           *output_++ = intr_type( id1, id2, seg2.P1( ), intr );
                     }
                     else
                     {
                        // NOTE: Cases like this:
                        //
                        //    *----
                        //   /
                        //  /
                        //
                        // are treated here as overlaps.

                        const intersection_type intr = cg::overlap;

                        if ( seg1P0CloseToSeg2 )
                           *output_++ = intr_type( id1, id2, seg1.P0( ), intr );
                        if ( seg1P1CloseToSeg2 )
                           *output_++ = intr_type( id1, id2, seg1.P1( ), intr );
                        if ( seg2P0CloseToSeg1 )
                           *output_++ = intr_type( id1, id2, seg2.P0( ), intr );
                        if ( seg2P1CloseToSeg1 )
                           *output_++ = intr_type( id1, id2, seg2.P1( ), intr );
                     }
                  }
                  else
                  {
                     point_type a, b;
                     const intersection_type intr = cg::robust_isect_segments( seg1, seg2, a, b );
                     if( intr == cg::intersect )
                        *output_++ = intr_type( id1, id2, a, intr );   
                     else if( intr == cg::overlap )
                     {
                       *output_++ = intr_type( id1, id2, a, intr );   
                       *output_++ = intr_type( id1, id2, b, intr );   
                     }
                  }
               }
            }
            return false;
         }

         BigCellProcessor & processgrid( ) { return *this; }

         template < class State, class bigcell_type  >
            SmallCellProcessor & processbigcell( State const &, bigcell_type const & )
         {
            return *this;
         }

      private:
         segments_type        const & segments_;
         OutIter                      output_;
         scalar_type                  eps_;
      };
   }

   template< typename segments_type >
      struct RasterizedSegments
   {
      typedef     int                                                                     segment_id_type;
      typedef     cg::contours::smallcellw::SegmentsHolder< segment_id_type >             small_cell_type;
      typedef     cg::Grid2L< small_cell_type >                                           grid_type;
      typedef     details::SegmentsTraits< segments_type, grid_type, segment_id_type >    segments_holder_type;

      RasterizedSegments( segments_type const & segments )
      {
         segments_holder_type traits( segments );

         cg::AABB  bb ( traits.segmentsBegin(), traits.segmentsEnd(), traits );
         if ( !bb.empty() )
            bb.inflate( 1. ) ;

         cg::Grid2LSubdiv subdiv( bb, 30, 10, 50 );
         grid_.reset( new cg::Grid2LInitializer< grid_type >( traits.segmentsBegin( ), traits.segmentsEnd( ), traits, subdiv ) );
      }

      grid_type const & grid( ) const { return *grid_; }
   private:
      std::auto_ptr< grid_type > grid_;
   };

   template< typename segments_type, typename proc_type >
      void rasterize_segments( segments_type const & segments, proc_type & proc )
   {
      cg::visit_every_cell( RasterizedSegments< segments_type >( segments ).grid( ), proc );
   }

   template< typename segment_type, typename segments_type = std::vector< segment_type > >
      struct SegmentsIntersections
   {
   private:
      typedef     typename segment_type::scalar_type                             scalar_type;
      typedef     typename segment_type::point_type                              point_type;

   public:
      typedef     segment_type                                                   segment_type;
      typedef     size_t                                                         segment_id_type;
      typedef     details::SegmentsIntersection< segment_id_type, point_type >   intr_type;
      typedef     intr_type                                                      value_type;

   private:
      typedef     std::vector< intr_type >                                       intersections_type;

   public:
      typedef     typename intersections_type :: const_iterator                  const_iterator;

   public:
      SegmentsIntersections( segments_type const & segments, scalar_type eps = 1e-5 )
         : segments_( &segments )
         , need_destruction_( false )
         , eps_( eps )
      {
         find_intersections( );
      }

      template< typename FwdIter >
         SegmentsIntersections( FwdIter p, FwdIter q, double eps = 1e-5 )
         : segments_( new segments_type( p, q ) )
         , need_destruction_( true )
         , eps_( eps )
      {
         find_intersections( );
      }

      ~SegmentsIntersections()
      {
         if( need_destruction_ )
            delete segments_;
      }

   private:
      struct intersections_less_type
      {
         bool operator( )( intr_type const & i1, intr_type const & i2 ) const
         {
            return i1.id1( ) <  i2.id1( ) 
              || ( i1.id1( ) == i2.id1( ) ) && ( i1.id2( ) < i2.id2( ) )
              || ( i1.id1( ) == i2.id1( ) ) && ( i1.id2( ) == i2.id2( ) ) && ( i1.p() < i2.p() );
         }
      };

      struct intersections_eq_type
      {
         bool operator( )( intr_type const & i1, intr_type const & i2 ) const
         {
            return ( i1.id1( ) == i2.id1( ) ) && ( i1.id2( ) == i2.id2( ) ) && ( i1.p( ) == i2.p( ) );
         }
      };

      void find_intersections( )
      {
         // FIXME: Segments must be rasterized with eps_ neighbourhood, otherwise
         // epsilon-intersections on cell edge will be missed.
         details::IntersectionsProcessor< segments_type, intr_type, std::back_insert_iterator< intersections_type > >
            proc( *segments_, std::back_inserter( intersections_ ), eps_ );
         rasterize_segments( *segments_, proc );  

         std::sort( intersections_.begin( ), intersections_.end( ), intersections_less_type( ) );
         
         intersections_.erase( std::unique( intersections_.begin( ), intersections_.end( ), intersections_eq_type( ) ), 
                               intersections_.end( ) );
      }

   public:
      size_t                     size( )           const { return intersections_.size( ); };
      intr_type         const &  at( size_t i )    const { return intersections_.at( i ); };
      const_iterator             begin( )          const { return intersections_.begin( ); }
      const_iterator             end( )            const { return intersections_.end( ); }

   private:
      bool                    need_destruction_;
      segments_type  const *  segments_;
      intersections_type      intersections_;
      scalar_type             eps_;
   };
}
