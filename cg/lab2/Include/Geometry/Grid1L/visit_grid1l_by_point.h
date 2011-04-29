#pragma once
#include "Grid1L_Impl.h"

namespace cg
{
   template <class G>
   struct visit_grid1l_by_point
   {
      typedef G grid_type;

      template <class Processor>
         static bool process( grid_type & grid, cg::point_2 const & point, Processor & processor )
      {
         const point_2i p = cg::rectangle_by_extents(grid.extents()).closest_point( floor( grid.world2local( point ) ) );

         if( processor( p, grid.at( p ) ) )
            return true;

         return false;
      }
   };

   template <class T, class Processor>
      inline bool visit(Grid1L<T> & grid, cg::point_2 const & point, Processor & processor)
   {
      return visit_grid1l_by_point< Grid1L<T> >::process( grid, point, processor );
   }

   template <class T, class Processor>
      inline bool visit(Grid1L<T> const & grid, cg::point_2 const & point, Processor & processor)
   {
      return visit_grid1l_by_point< Grid1L<T> const >::process( grid, point, processor );
   }
}