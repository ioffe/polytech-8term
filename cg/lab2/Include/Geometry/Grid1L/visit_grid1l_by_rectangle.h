#pragma once
#include "Grid1L_Impl.h"

namespace cg
{

   template <class G>
   struct visit_grid1l_by_rectangle
   {
      typedef G                       grid_type;
      typedef point_2i                smallidx_type;

      template <class Processor>
         static bool process( grid_type & grid, cg::rectangle_2 const & rect, Processor & processor )
      {
         cg::rectangle_2i ind_rect( point_2i( 0, 0 ), grid.extents( ) - point_2i( 1, 1 ) );

         const point_2i beg = ind_rect.closest_point( floor( grid.world2local( rect.xy( ) ) ) );
         const point_2i end = ind_rect.closest_point( floor( grid.world2local( rect.XY( ) ) ) );

         for( point_2i p = beg; p.x <= end.x; ++p.x ){
         for( p.y = beg.y; p.y <= end.y; ++p.y )
         {
            if( processor( p, grid.at( p ) ) )
               return true;
         }}
         return false;
      }
   };

   template <class T, class Processor>
      inline bool visit(Grid1L<T> & grid, cg::rectangle_2 const & rect, Processor & processor)
   {
      return visit_grid1l_by_rectangle< Grid1L<T> >::process( grid, rect, processor );
   }

   template <class T, class Processor>
      inline bool visit(Grid1L<T> const & grid, cg::rectangle_2 const & rect, Processor & processor)
   {
      return visit_grid1l_by_rectangle< Grid1L<T> const >::process( grid, rect, processor );
   }
}
