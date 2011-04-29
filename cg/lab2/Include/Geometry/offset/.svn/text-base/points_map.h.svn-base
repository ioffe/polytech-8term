#pragma once

#include "geometry\grid1L.h"
#include <boost/optional.hpp>

namespace cg
{
   template < class point_type, class index_type >
      struct PointsMap
   {
      typedef point_type                                                point_type;
      typedef typename point_type::scalar_type                          scalar_type;  

   private:
      typedef std::pair< point_type, index_type >                       point_data_pair_type;
      typedef std::vector< point_data_pair_type >                       cell_type;
      typedef cg::Grid1L< cell_type >                                   grid_type;

      struct cell_processor_type
      {
         cell_processor_type( point_type const & pt, scalar_type eps )
            : pt_( pt )
            , eps_ ( eps )
         {}

      private:
         struct point_equal_to_type
         {
            point_equal_to_type( point_type const & p, scalar_type eps )
               : p_( p )
               , eps_( eps )
            {}

            bool operator( )( point_data_pair_type const & pd ) const
            {
               return cg::eq( p_.x, pd.first.x, eps_ ) && cg::eq( p_.y, pd.first.y, eps_ );
            }

         private:
            point_type  p_;
            scalar_type eps_;
         };

      public:
         template< class State >
            bool operator( )( State const & st, cell_type const & cell )
         {
            cell_type::const_iterator const 
               it = std::find_if( cell.begin(), cell.end(), point_equal_to_type( pt_, eps_ ) );
            
            if( it == cell.end() )
               return false;

            idx_ = it->second;
            return true;
         }

         index_type index( ) const { return *idx_; };

      private:
         point_type pt_;
         scalar_type eps_;
         boost::optional< index_type > idx_;
      };

   public:
      PointsMap( cg::aa_transform const & tform, cg::point_2i const & ext, scalar_type eps )
         : grid_( tform, ext )
         , eps_( eps )
      {}

      std::pair< index_type, bool >
         insert( point_type const & pt, index_type idx )
      {         
         const cg::point_2 eps_p( eps_, eps_ );

         cell_processor_type proc( pt, eps_ );
         
         if( !cg::visit( grid_, cg::rectangle_2( pt - eps_p, pt + eps_p ), proc ) )
         {
            cell_type & cell = grid_.at( lookup_cell( pt ) );
            cell.push_back( point_data_pair_type( pt, idx ) );
            return std::make_pair( idx, true );
         }

         return std::make_pair( proc.index( ), false );
      }

   private:
      cg::point_2i lookup_cell( point_type const & pt ) const
      {
         cg::point_2i const local_pt = floor( grid_.world2local( pt ) );
         return cg::rectangle_by_extents(grid_.extents()).closest_point(local_pt);
      }

   private:
      grid_type   grid_;
      scalar_type eps_; 
   };
}
