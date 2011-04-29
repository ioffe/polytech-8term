#pragma once

#include "Grid2L\grid2l_Impl.h"
#include "Grid2L\Grid2L_Mapped.h"
#include "Grid2L\visit_grid2l_by_point.h"
#include "Grid2l\visit_grid2l_by_segment.h"
#include "Grid2L\visit_grid2l_by_triangle.h"
#include "Grid2L\visit_grid2l_by_rectangle.h"
#include "Grid2L\visit_grid2l_every_cell.h"
//#include "Grid2L\intersection_visitor.h"

namespace cg
{
    template <class T, class Processor>
        inline __forceinline bool visit(MappedGrid2L<T> & grid, point_2 const &pt, Processor &processor)
    {
        return visit_grid2l_by_point<MappedGrid2L<T> >::process(grid, pt, processor);
    }    

    template <class T, class Processor>
        inline __forceinline bool visit(MappedGrid2L<T> const & grid, point_2 const &pt, Processor &processor)
    {
        return visit_grid2l_by_point<MappedGrid2L<T> const>::process(grid, pt, processor);
    }    

    template <class T, class Processor>
       inline __forceinline bool visit(MappedGrid2L<T> const & grid, point_2 const &pt, Processor &processor, cg::Index2L & idx)
    {
        return visit_grid2l_by_point<MappedGrid2L<T> const>::process(grid, pt, processor, idx);
    }    

    template <class T, class Processor>
        inline bool visit(MappedGrid2L<T> & grid, rectangle_2 const &r, Processor &processor)
    {
        return visit_grid2l_by_rectangle<MappedGrid2L<T> >::process(grid, r, processor);
    }

    template <class T, class Processor>
        inline bool visit(MappedGrid2L<T> const & grid, rectangle_2 const &r, Processor &processor)
    {
        return visit_grid2l_by_rectangle<MappedGrid2L<T> const>::process(grid, r, processor);
    }

    template <class T, class Processor, class Scalar>
        inline bool visit(MappedGrid2L<T> & grid, segment_t< Scalar, 2 > const &s, Processor &processor)
    {
        return visit_grid2l_by_segment<MappedGrid2L<T> >::process(grid, s, processor);
    }

    template <class T, class Processor, class Scalar>
        inline bool visit(MappedGrid2L<T> const & grid, segment_t< Scalar, 2 > const &s, Processor &processor)
    {
        return visit_grid2l_by_segment<MappedGrid2L<T> >::process(const_cast<MappedGrid2L<T>&>(grid), s, processor);
    }

    template <class T, class Processor>
        inline bool visit(MappedGrid2L<T> & grid, triangle_2 const &t, Processor &processor)
    {
        return visit_grid2l_by_triangle<MappedGrid2L<T> >::process(grid, t, processor);
    }

    template <class T, class B, class H, class Processor>
        inline bool visit_every_cell(MappedGrid2L<T,B,H> & grid, Processor & processor)
    {
        return visit_grid2l_every_cell<MappedGrid2L<T,B,H> >::process(grid, processor);
    }

    template <class T, class B, class H, class Processor>
        inline bool visit_every_cell(MappedGrid2L<T,B,H> const & grid, Processor & processor)
    {
        return visit_grid2l_every_cell<MappedGrid2L<T,B,H> >::process(const_cast<MappedGrid2L<T,B,H>&>(grid), processor);
    }

    //----------------------------------------------------------------------------------

    template <class T, class B, class H, class Processor>
        inline bool visit_every_cell(Grid2L<T,B,H> & grid, Processor & processor)
    {
        return visit_grid2l_every_cell<Grid2L<T,B,H> >::process(grid, processor);
    }

    template <class T, class B, class H, class Processor>
        inline bool visit_every_cell(Grid2L<T,B,H> const & grid, Processor & processor)
    {
        return visit_grid2l_every_cell<Grid2L<T,B,H> >::process(const_cast<Grid2L<T,B,H>&>(grid), processor);
    }

}