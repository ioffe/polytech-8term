#pragma once
#include "Geometry\triangle_raster_iterator.h"
#include "geometry/triangle3_raster_iterator.h"
#include "Geometry\Grid1L\visit_grid1l_by_segment.h"
#include "grid1l_Impl.h"

namespace cg 
{
    template <class T>
        struct visit_grid1l_by_triangle
    {
        typedef point_2i state;

        template <class Processor>
            static bool process(Grid1L<T> & grid, 
                triangle_2 const &t, Processor & processor)
        {
            triangle_2 translated = cg::world2local(t, grid);

            for (triangle_rasterization_iterator iter(translated); iter; ++iter)
            {
                if (grid.contains(*iter))
                {
                    state st = *iter;

                    if (processor(st, grid.at(*iter)))
                        return true;
                }
            }

            return false;
        }    
    };

    template <class T>
        struct visit_grid1l_by_triangle_ex
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
               triangle_2 const &t, Processor & processor)
        {
            point_2i v1 = floor(grid.world2local(t[0]));
            point_2i v2 = floor(grid.world2local(t[1]));
            point_2i v3 = floor(grid.world2local(t[2]));

            int min_y = cg::min(v1.y, v2.y, v3.y);
            int max_y = cg::max(v1.y, v2.y, v3.y);

            Sides sides(min_y, max_y - min_y + 1);

            typedef  
                SegmentContructionCellProcessor<typename Processor::SideProcessor> 
                sproc;

            sproc   ab_processor (sides, processor.side_processor(0,1));
            visit(grid, segment_2(t[0], t[1]), ab_processor);

            sproc   bc_processor (sides, processor.side_processor(1,2));
            visit(grid, segment_2(t[1], t[2]), bc_processor);

            sproc   ca_processor (sides, processor.side_processor(2,0));
            visit(grid, segment_2(t[2], t[0]), ca_processor);

            state idx;

            for (idx.y = min_y; idx.y <= max_y; ++idx.y) 
            {
                range_2i const & x_range = sides.get(idx.y);

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
        inline bool visit(Grid1L<T> & grid, triangle_2 const &t, Processor & proc)
    {
        return visit_grid1l_by_triangle<T>::process(grid, t, proc);
    }

    template <class T, class Processor>
        inline bool visit_ex(Grid1L<T> & grid, triangle_2 const &t, Processor & proc)
    {
        return visit_grid1l_by_triangle_ex<T>::process(grid, t, proc);
    }
}