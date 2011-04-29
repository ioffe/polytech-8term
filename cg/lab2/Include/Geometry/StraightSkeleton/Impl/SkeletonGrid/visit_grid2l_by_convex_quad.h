#pragma once

#include "Geometry\Grid2L\Grid2L_Impl.h"
#include "Geometry\Grid2L\visit_grid2l_by_segment.h"
#include "Geometry\triangle3_raster_iterator.h"

#include "Geometry\quad_clip_by_rect.h"

#undef min
#undef max

namespace cg
{

template< class Processor >
   inline void my_rasterize_segment( cg::segment_2 const &seg, Processor &proc )
{
   if (cg::abs(seg.P1().y - seg.P0().y) < cg::epsilon< double >())
      rasterize_segment(cg::segment_2 (seg.P0(), cg::point_2 (seg.P1().x, seg.P0().y)), proc);
   else if (cg::abs(seg.P1().x - seg.P0().x) < cg::epsilon< double >())
      rasterize_segment(cg::segment_2 (seg.P0(), cg::point_2 (seg.P0().x, seg.P1().y)), proc);
   else
      rasterize_segment(seg, proc);
}

template< class grid_type >
   struct visit_grid2l_by_convex_quad
{
   typedef typename grid_type::smallcell_type smallcell_type;
   typedef typename grid_type::bigcell_type   bigcell_type;
   typedef typename grid_type::index_type     index_type;

   typedef point_2i bigidx_type;
   typedef point_2i smallidx_type;

   typedef index_type state;

   typedef traster_details::Sides Sides;

   template< class ActualSideProcessor >
      struct SegmentContructionCellProcessor 
         : cg::grid2l_visitor_base< grid_type, SegmentContructionCellProcessor< ActualSideProcessor > >
   {
      SegmentContructionCellProcessor ( Sides &sides, ActualSideProcessor & act_processor )
         : sides_(sides), actual_processor_ (act_processor)
      {
      }

      template< class State >
         bool operator () ( State const &st, smallcell_type &data )
      {
         Assert(sides_.is_valid(st.big.y));
         Assert(sides_.is_valid(st.big.y));
         sides_.get(st.big.y).unite(st.big.x);
   
         return actual_processor_(st, data);
      }

   private:
      ActualSideProcessor & actual_processor_;
      Sides               & sides_;
   };

   template <class Processor>
      static bool process(grid_type & grid, quad_2 const &q, Processor & processor)
   {
      double const eps = cg::epsilon< double >( );

      bigidx_type v1 = floor(grid.world2local(q[0]) + cg::point_2 (eps, eps));
      bigidx_type v2 = floor(grid.world2local(q[1]) + cg::point_2 (eps, eps));
      bigidx_type v3 = floor(grid.world2local(q[2]) + cg::point_2 (eps, eps));
      bigidx_type v4 = floor(grid.world2local(q[3]) + cg::point_2 (eps, eps));

      int min_y = cg::min(v1.y, v2.y, v3.y, v4.y);
      int max_y = cg::max(v1.y, v2.y, v3.y, v4.y);

      Sides sides(min_y, max_y - min_y + 1);

      typedef SegmentContructionCellProcessor<Processor::SideProcessor> sproc;

      // process_quad_sides
      {
         rectangle_2 bb = bounding(grid);
         bb.inflate( - epsilon<double>( ) );

         std::vector<point_2>   clipped;

         cull(q, bb, clipped);

         for (size_t i = 0; i != clipped.size(); ++i)
         {
            size_t n = cg::next((int)i, (int)clipped.size());

            sproc proc (sides, processor.side_processor((int)i,(int)n));

            visit(grid, cg::segment_2(clipped[i], clipped[n]), proc);
         }
      }

      bigidx_type  idx_big;

      for (idx_big.y = min_y; idx_big.y <= max_y; ++idx_big.y) 
      {
            range_2i const & x_range = sides.get(idx_big.y);

            if ( x_range.empty( ) )
               continue;

            for (idx_big.x = x_range.lo(); idx_big.x <= x_range.hi(); ++idx_big.x)
            {
               raster_2 const & raster = grid.bigcellraster(idx_big);

               if (!processor.processBigCell(raster))
                  continue;

               point_2 v1 = raster.translate(q[0]);
               point_2 v2 = raster.translate(q[1]);
               point_2 v3 = raster.translate(q[2]);
               point_2 v4 = raster.translate(q[3]);

               point_2i const & ext = grid.at(idx_big).extents();
               range_2i const   ext_rng(0, ext.x - 1);

               int max_h = ext.y - 1;

               int min_y = max(0,     floor(min(v1.y, v2.y, v3.y, v4.y))); 
               int max_y = min(max_h, floor(max(v1.y, v2.y, v3.y, v4.y)));

               if (max_y < min_y)
                  continue;

               Sides       sides(min_y, max_y - min_y + 1);

               namespace tr = triangle_rasterization;

               typedef tr::SidesCreator sproc;

               sproc   side_processor(sides, grid.at(idx_big).extents());

               my_rasterize_segment(segment_2(v1, v2), side_processor);
               my_rasterize_segment(segment_2(v2, v3), side_processor);
               my_rasterize_segment(segment_2(v3, v4), side_processor);
               my_rasterize_segment(segment_2(v4, v1), side_processor);

               smallidx_type idx_small;

               for (idx_small.y = min_y; idx_small.y <= max_y; ++idx_small.y) 
               {
                  range_2i x_range = sides.get(idx_small.y);

                  for (idx_small.x  = max(x_range.lo(), ext_rng.lo()); 
                        idx_small.x <= min(x_range.hi(), ext_rng.hi()); 
                        ++idx_small.x)
                  {
                        {
                           state st(idx_big, idx_small);

                           if (processor(st, grid.at(idx_big).at(idx_small)))
                              return true;
                        }
                  }
               }
            }
      }

      return false;
   }    
};

template <class T, class B, class Processor>
   inline bool visit(Grid2L<T,B> & grid, quad_2 const &q, Processor &processor)
{
   return visit_grid2l_by_convex_quad<Grid2L<T,B> >::process(grid, q, processor);
}

} // End of 'cg' namespace
