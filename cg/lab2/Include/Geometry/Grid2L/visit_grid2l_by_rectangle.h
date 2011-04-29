#pragma once
#include "Grid2l_Impl.h"

namespace cg
{

    template <class G>
        struct visit_grid2l_by_rectangle
    {
        typedef G                       grid_type;
        typedef typename G::smallcell_type       smallcell_type;
        typedef typename G::bigcell_type         bigcell_type;
        typedef typename G::index_type           index_type;
        typedef point_2i                bigidx_type;
        typedef point_2i                smallidx_type;

        struct state : index_type
        {
            state(grid_type & g, point_2i const &idx_big, point_2i const &idx_small)
                :   index_type(idx_big, idx_small), grid_(g)
            {}

            rectangle_2 smallcellbound() const {
                return grid_.bigcellraster(big).domain(small);
            }

            grid_type       & grid()       { return grid_; }
            grid_type const & grid() const { return grid_; }

        private:
            grid_type   & grid_;
        };

        template <class Processor, class Grid, class BigCell>
            static bool processbigcell (BigCell & bigcell, Grid & grid, 
                point_2 const & beg, point_2 const & end, 
                point_2i const & big_start, point_2i const & big_stop,
                point_2i const & big, Processor & bigcell_proc)
        {

// This part must be tested !!!
#define NO_GRID2L_BUG

#ifndef NO_GRID2L_BUG
            if (bigcell)
            {
                typename Processor::SmallCellProcessor &
                    sproc = bigcell_proc.processbigcell(big, bigcell);
#else
            typename Processor::SmallCellProcessor &
                sproc = bigcell_proc.processbigcell(big, bigcell);

            if (bigcell)
            {
#endif

                point_2i small_start = point_2i(0,0);
                point_2i small_stop  = bigcell.extents();

                if (big.x == big_start.x)
                    make_max(small_start.x, 
                    floor((beg.x - big_start.x) * bigcell.extents().x));

                if (big.y == big_start.y)
                    make_max(small_start.y, 
                    floor((beg.y - big_start.y) * bigcell.extents().y));

                if (big.x == big_stop.x)
                    make_min(small_stop.x, 
                    ceil((end.x - big_stop.x) * bigcell.extents().x));

                if (big.y == big_stop.y)
                    make_min(small_stop.y, 
                    ceil((end.y - big_stop.y) * bigcell.extents().y));

                point_2i small;

                for (small.x = small_start.x; small.x < small_stop.x; ++small.x) {
                    for (small.y = small_start.y; small.y < small_stop.y; ++small.y) {

                        state st(grid, big, small);

                        if (sproc(st, bigcell.at(small)))
                            return true;
                    }
                }
            }

            return bigcell_proc.postprocessbigcell(big, bigcell);
        }

        template <class Processor>
            static bool process(grid_type & grid, rectangle_2 const &rc, Processor & processor)
        {
            typename Processor::BigCellProcessor &
                bigcell_proc = 
                    static_cast<typename Processor::BigCellProcessor &>
                        (processor.processgrid());

            // —ейчас рассматриваетс€ случай, когда grid.principal_raster().angle() == 0

            point_2 beg = grid.world2local(rc.lo());
            point_2 end = grid.world2local(rc.hi());

            // провер€ем, что пересечение есть
            if (beg.x >= grid.extents().x || 
                beg.y >= grid.extents().y || 
                end.x < 0 || end.y < 0)
            {
                return false;
            }

            // клипуем пр€моугольник
            make_max(beg.x, 0.);
            make_max(beg.y, 0.);
            make_min(end.x, grid.extents().x - epsilon<double>( ) );
            make_min(end.y, grid.extents().y - epsilon<double>( ) );

            bigidx_type big_start = floor(beg);
            bigidx_type big_stop  = floor(end);

            make_max(big_start.x, 0);
            make_max(big_start.y, 0);
            make_min(big_stop.x, grid.extents( ).x - 1 );
            make_min(big_stop.y, grid.extents( ).y - 1 );

			   bigidx_type big;

            for (big.x = big_start.x; big.x <= big_stop.x; ++big.x) 
            {
                for (big.y = big_start.y; big.y <= big_stop.y; ++big.y) 
                {
                    if (processbigcell(grid.at(big), grid, beg, end, big_start, big_stop, big, bigcell_proc))
                        return true;
                }
            }

            return false;
        }
    };

    template <class T, class B, class H, class Processor>
        inline bool visit(Grid2L<T,B,H> & grid, rectangle_2 const &r, Processor &processor)
    {
        return visit_grid2l_by_rectangle<Grid2L<T,B,H> >::process(grid, r, processor);
    }

    template <class T, class B, class H, class Processor>
        inline bool visit(Grid2L<T,B,H> const & grid, rectangle_2 const &r, Processor &processor)
    {
        return visit_grid2l_by_rectangle<Grid2L<T,B,H> >::process( const_cast< Grid2L<T,B,H> & > ( grid ), r, processor);
    }
}