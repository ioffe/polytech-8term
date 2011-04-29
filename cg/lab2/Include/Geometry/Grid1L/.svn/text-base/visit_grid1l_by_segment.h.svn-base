#pragma once
#include "Grid1L_Impl.h"

namespace cg 
{

    template <class T>
        struct visit_grid1L_by_segment
    {
        struct state : point_2i
        {
            state(point_2i const &idx, double in_ratio, double out_ratio)
                :   point_2i    (idx)
                ,   in_ratio    (in_ratio)
                ,   out_ratio   (out_ratio)
            {}

            double in_ratio;
            double out_ratio;
        };

        template <class Grid, class ActualProcessor>
            struct CellProcessor
        {
            CellProcessor(Grid & data, ActualProcessor & actual_processor) 
                : data(data), actual_processor(actual_processor) 
            {}

            bool operator () (point_2i const &idx, double in_ratio, double out_ratio)
            {
                state st(idx, in_ratio, out_ratio);

                return data.contains(idx) ? actual_processor(st, data[idx]) : false;
            }

        private:
            Grid            & data;
            ActualProcessor & actual_processor;
        };

        template <class Processor>
            static bool process(Grid1L<T> & grid, 
                segment_2 const &s, Processor & processor)
        {
            segment_2 const translated = world2local(s, grid);

    //        segment_2 clipped = cull(s, ::bounding(*this));

            CellProcessor<Grid1L<T>, Processor>    cell_processor(grid, processor);

            return rasterize_segment(translated, cell_processor);
        }

        template <class Processor>
            static bool process(Grid1L<T> const & grid, 
                segment_2 const &s, Processor & processor)
        {
            segment_2 const translated = world2local(s, grid);

    //        segment_2 clipped = cull(s, ::bounding(*this));

            CellProcessor<const Grid1L<T>, Processor>    cell_processor(grid, processor);

            return rasterize_segment(translated, cell_processor);
        }
    };

    template <class T, class Processor, class Scalar>
        inline bool visit(Grid1L<T> & grid, segment_t< Scalar, 2 > const & seg, Processor & proc)
    {
        return visit_grid1L_by_segment<T>::process(grid, cg::segment_2( seg.P0(), seg.P1() ), proc);
    }

    template <class T, class Processor, class Scalar>
        inline bool visit(Grid1L<T> const & grid, segment_t< Scalar, 2 > const & seg, Processor & proc)
    {
        return visit_grid1L_by_segment<T>::process(grid, cg::segment_2( seg.P0(), seg.P1() ), proc);
    }
}