#pragma once
#include "common/safe_bool.h"

namespace cg {
namespace polyoffset 
{

   template < typename segments_type >
      struct DistToSegLessThan
   {
      typedef  DistToSegLessThan                                          BigCellProcessor;
      typedef  DistToSegLessThan                                          SmallCellProcessor;

      typedef  typename segments_type  :: value_type                      segment_type;
      typedef  typename segment_type   :: point_type                      point_type;
      typedef  typename point_type     :: scalar_type                     scalar_type;

      DistToSegLessThan( segments_type const & segments, point_type const & p, scalar_type d )
         : segments_( segments )
         , p_ ( p )
         , d_ ( d )
         , found_ ( false )
      {}

      template< typename State, typename SmallCell >
         bool operator( )( State const & st, SmallCell const & cell )
      {
         typedef     typename SmallCell :: segment_id           segment_id_type;
         
         const segment_id_type size = ( segment_id_type ) cell.segments( ).size( );
         for( segment_id_type i = 0; i != size; ++i )
         {
            const segment_id_type id = cell.segments( ).at( i );
            segment_type const & s = segments_.at( id );
            if( cg::distance( s, p_ ) < d_ )
               found_ = true;
         }
         return found_;
      }

      BigCellProcessor & processgrid( ) { return *this; }

      template< class T1, class T2 >
         bool  postprocessbigcell( T1 const &, T2 const & ){ return false; }

      template < class State, class bigcell_type  >
         SmallCellProcessor & processbigcell( State const & state, bigcell_type const & bcell )
      {
         return *this;
      }

      SAFE_BOOL_OPERATOR(found_)

   private:
      segments_type        const & segments_;
      point_type           const   p_;
      scalar_type          const   d_;
      bool                         found_;
   }; 

   template < typename segments_type, typename point_type >
   struct less_dist
   {
      typedef typename segments_type :: value_type                       segment_type;
      typedef typename point_type :: scalar_type                         scalar_type;
      
      less_dist( segments_type const & segments )
         : segments_( segments )
      {}
      
      bool operator( )( point_type const & p, scalar_type d ) const
      {
         size_t size = segments_.size( );
         for ( size_t i = 0; i < size; ++i )
         {
            cg::segment_2 const & seg = segments_.at( i );
            const double cur_dist = cg::distance( p, seg );
            if ( cur_dist < d )
               return true;
         }
         return false;
      }

   private:
      segments_type const & segments_;
   };
}}
