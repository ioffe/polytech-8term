#pragma once
#include "Geometry\triangle_raster_iterator.h"
#include "geometry/triangle3_raster_iterator.h"
#include "Geometry\Grid1L\visit_grid1l_by_segment.h"
#include "Geometry\Grid1L\Grid1l_Impl.h"

#include "Geometry\quad.h"
// #include "Geometry\quad_clip_by_rect.h"

namespace cg 
{
    template <class T>
        struct visit_grid1l_by_convex_quad
    {
        typedef point_2i state;
        typedef T        cell_type;

        typedef traster_details::Sides  Sides;

        template <class ActualSideProcessor>
            struct SegmentContructionCellProcessor
        {
            SegmentContructionCellProcessor(
                Sides &sides, ActualSideProcessor & act_processor)
                :  sides_(sides), actual_processor_ (act_processor)
            {}

            typedef typename
                visit_grid1L_by_segment<T>::state 
                segment_visitor_state;

            bool operator () (segment_visitor_state const &st, cell_type & data)
            {
                Assert(sides_.is_valid(st.y));
                sides_.get(st.y).unite(st.x);

                return actual_processor_(st, data);
            }

        private:
            ActualSideProcessor & actual_processor_;
            Sides               & sides_;
        };

        template <class Processor>
            static bool process(Grid1L<T> & grid, 
               quad_2 const &q, Processor & processor)
        {
            double const eps = cg::epsilon< double >( );

            point_2i v1 = floor(grid.world2local(q[0]) + cg::point_2 (eps, eps));
            point_2i v2 = floor(grid.world2local(q[1]) + cg::point_2 (eps, eps));
            point_2i v3 = floor(grid.world2local(q[2]) + cg::point_2 (eps, eps));
            point_2i v4 = floor(grid.world2local(q[3]) + cg::point_2 (eps, eps));

            int min_y = cg::min(v1.y, v2.y, v3.y, v4.y);
            int max_y = cg::max(v1.y, v2.y, v3.y, v4.y);

            Sides sides(min_y, max_y - min_y + 1);

            typedef  
                SegmentContructionCellProcessor<typename Processor::SideProcessor> 
                sproc;

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

            state idx;

            for (idx.y = min_y; idx.y <= max_y; ++idx.y) 
            {
                range_2i const & x_range = sides.get(idx.y);
                if ( x_range.empty( ) )
                   continue;

                for (idx.x = x_range.lo(); idx.x <= x_range.hi(); ++idx.x)
                {
                    if (processor(idx, grid.at(idx)))
                        return true;
                }
            }
            return false;
        }
    };

    template <class T, class Processor>
        inline bool visit(Grid1L<T> & grid, quad_2 const &q, Processor & proc)
    {
        return visit_grid1l_by_convex_quad<T>::process(grid, q, proc);
    }
}
