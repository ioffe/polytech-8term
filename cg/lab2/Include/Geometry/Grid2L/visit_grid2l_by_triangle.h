#pragma once

#include "Grid2l_Impl.h"
#include "visit_grid2l_by_segment.h"
#include "geometry/triangle3_raster_iterator.h"
#include "geometry/triangle_clip_by_rect.h"

#undef min
#undef max

namespace cg
{

    // Обход 2-х уровнего грида вдоль теугольника
    template <class G>
        struct visit_grid2l_by_triangle
    {
        typedef G                       grid_type;
        typedef typename G::smallcell_type       smallcell_type;
        typedef typename G::bigcell_type         bigcell_type;
        typedef typename G::index_type           index_type;
        typedef point_2i                bigidx_type;
        typedef point_2i                smallidx_type;
    
        // состояние обхода для процессора
        typedef index_type              state;

        typedef traster_details::Sides  Sides;

        template <class ActualSideProcessor>
            struct SegmentContructionCellProcessor 
                :  cg::grid2l_visitor_base<G, SegmentContructionCellProcessor<ActualSideProcessor> >
        {
            SegmentContructionCellProcessor(
                Sides &sides, ActualSideProcessor & act_processor)
                :  sides_(sides), actual_processor_ (act_processor)
            {}

            typedef typename
                visit_grid2l_by_segment<G>::state 
                segment_visitor_state;

            bool operator () (segment_visitor_state const &st, smallcell_type & data)
            {
                if ( sides_.is_valid(st.big.y) )
                   sides_.get(st.big.y).unite(st.big.x);
            
                return actual_processor_(st, data);
            }

        private:
            ActualSideProcessor & actual_processor_;
            Sides               & sides_;
        };

        template <class Processor>
            static bool process(grid_type & grid, triangle_2 const &t, Processor & processor)
        {
            bigidx_type v1 = floor(grid.world2local(t[0]));
            bigidx_type v2 = floor(grid.world2local(t[1]));
            bigidx_type v3 = floor(grid.world2local(t[2]));

            int min_y = cg::min(v1.y, v2.y, v3.y);
            int max_y = cg::max(v1.y, v2.y, v3.y);

            Sides       sides(min_y, max_y - min_y + 1);

            typedef 
                SegmentContructionCellProcessor<Processor::SideProcessor> sproc;
/*
            rectangle_2 bb = bounding(grid);
            bb.inflate(-cg::epsilon<double>( ));

            std::vector<point_2>   clipped;
            
            cull(t, bb, clipped);

            // TODO :: 2 Kovalev
            for (size_t i = 0; i != clipped.size(); ++i)
            {
                size_t n = cg::next((int)i, (int)clipped.size());

                sproc   proc (sides, processor.side_processor((int)i,(int)n));

                visit(grid, cg::segment_2(clipped[i], clipped[n]), proc);
            }*/

            processor.process_triangle_sides <sproc> (grid, sides, t);

            bigidx_type  idx_big;

            for (idx_big.y = min_y; idx_big.y <= max_y; ++idx_big.y) 
            {
                range_2i const & x_range = sides.get(idx_big.y);

                if ( x_range.empty( ) )
                   continue;

                for (idx_big.x = x_range.lo(); idx_big.x <= x_range.hi(); ++idx_big.x)
                {
                    raster_2 const & raster = grid.bigcellraster(idx_big);

                    point_2 v1 = raster.translate(t[0]);
                    point_2 v2 = raster.translate(t[1]);
                    point_2 v3 = raster.translate(t[2]);

                    point_2i const & ext = grid.at(idx_big).extents();
                    range_2i const   ext_rng(0, ext.x - 1);

                    int max_h = ext.y - 1;

                    int min_y = max(0,     floor(min(v1.y, v2.y, v3.y))); 
                    int max_y = min(max_h, floor(max(v1.y, v2.y, v3.y)));

                    Sides       sides(min_y, max_y - min_y + 1);

                    namespace tr = triangle_rasterization;

                    typedef tr::SidesCreator sproc;

                    sproc   side_processor(sides, grid.at(idx_big).extents());

                    rasterize_segment(segment_2(v1, v2), side_processor);
                    rasterize_segment(segment_2(v2, v3), side_processor);
                    rasterize_segment(segment_2(v3, v1), side_processor);

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
        inline bool visit(Grid2L<T,B> & grid, triangle_2 const &t, Processor &processor)
    {
        return visit_grid2l_by_triangle<Grid2L<T,B> >::process(grid, t, processor);
    }

}