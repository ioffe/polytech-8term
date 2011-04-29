#pragma once

#include "Grid1L_Impl.h"
#include "Grid1L_Mapped_fwd.h"

namespace cg
{
    template <class G>
        struct visit_grid1l_every_cell
    {
        typedef G                       grid_type;
        typedef point_2i                smallidx_type;

        template <class Processor>
            static bool process(grid_type & grid, Processor & processor) 
        {
            for (grid_type::iterator it = grid.begin(); it != grid.end(); ++it)
            {
                if (processor(it, *it))
                    return true;
            }

            return false;
        }
    };

    template <class T, class Processor>
        inline bool visit_every_cell(Grid1L<T> & grid, Processor & processor)
    {
        return visit_grid1l_every_cell<Grid1L<T> >::process(grid, processor);
    }

    template <class T, class Processor>
        inline bool visit_every_cell(MappedGrid1L<T> const & grid, Processor & processor)
    {
        return visit_grid1l_every_cell<MappedGrid1L<T> const >::process(grid, processor);
    }
}