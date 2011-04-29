#pragma once

#include "Grid1l_Impl.h"
#include "visit_grid1L_by_segment.h"

namespace cg
{
    // Обход 2-х уровнего грида вдоль выпуклого многоугольника
    template <class G>
    struct visit_grid1L_by_convex_poly
    {
        typedef G                                grid_type;
        typedef typename G::smallcell_type       smallcell_type;
        typedef typename G::bigcell_type         bigcell_type;
        typedef typename G::index_type           index_type;
        typedef point_2i                bigidx_type;
        typedef point_2i                smallidx_type;

        // состояние обхода для процессора
        typedef index_type              state;

        typedef traster_details::Sides  Sides;

        struct SegmentContructionCellProcessor 
        {
            SegmentContructionCellProcessor(Sides &sides)
                :  sides_(sides)
            {}

            typedef typename
                visit_grid1L_by_segment<G>::state 
                segment_visitor_state;

            template <class T>
                bool operator () (segment_visitor_state const &st, T const & )
            {
                Assert(sides_.is_valid(st.big.y));
                sides_.get(st.big.y).unite(st.big.x);

                return false;
            }

        private:
            Sides               & sides_;
        };

        template <class Processor, class FwdIter>
            static bool process(grid_type & grid, FwdIter first, FwdIter beyond, Processor & processor)
        {
            cg::bbox_1di y_rng;

            for (FwdIter p = first; p != beyond; ++p)
            {
                y_rng.unite(floor(grid.world2local(*p)));
            }

            int min_y = y_rng.lo();
            int max_y = y_rng.hi();

            Sides       sides(min_y, max_y - min_y + 1);
            SegmentContructionCellProcessor sproc(sides);

            for (FwdIter p = first; p != beyond;)
            {
                FwdIter current = p;
                FwdIter next    = ++p;

                visit(grid, cg::segment_2(*current, *next), sproc);
            };

            bigidx_type  idx_big;

            for (idx_big.y = min_y; idx_big.y <= max_y; ++idx_big.y) 
            {
                range_2i const & x_range = sides.get(idx_big.y);

                for (idx_big.x = x_range.lo(); idx_big.x <= x_range.hi(); ++idx_big.x)
                {
                    if (processor(idx_big, grid.at(idx_big)))
                        return true;
                }
            }

            return false;
        }    
    };

    template <class T, class TForm, class Storage, class FwdIter, class Processor>
        inline bool visitConvexHull(Grid1L<T,TForm,Storage> & grid, FwdIter first, FwdIter beyond, Processor &processor)
    {
        return visit_grid1L_by_convex_poly<Grid1L<T,TForm,Storage> >::process(grid, first, beyond, processor);
    }
}