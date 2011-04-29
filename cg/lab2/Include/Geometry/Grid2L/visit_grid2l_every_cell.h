#pragma once
#include "Grid2L_Impl.h"

namespace cg
{

    template <class G>
        struct visit_grid2l_every_cell
    {
        typedef G                       grid_type;
        typedef typename G::smallcell_type       smallcell_type;
        typedef typename G::bigcell_type         bigcell_type;
        typedef typename G::index_type           index_type;
        typedef point_2i                bigidx_type;
        typedef point_2i                smallidx_type;

        struct state : index_type
        {
            state(grid_type & grid, point_2i const &idx_big, point_2i const &idx_small)
                :   index_type(idx_big, idx_small), grid_(grid) 
            {}

            rectangle_2 smallcellbound() const {
                return grid_.bigcellraster(big).domain(small);
            }

            grid_type       & grid()       { return grid_; }
            grid_type const & grid() const { return grid_; }

        private:
            grid_type   & grid_;
        };

        struct state_bigcell : bigidx_type
        {
            state_bigcell (grid_type & grid, bigidx_type const &bigidx)
              : grid_(grid), bigidx_type(bigidx)
            {}

            grid_type       & grid()       { return grid_; }
            grid_type const & grid() const { return grid_; }

        private:
            grid_type & grid_;
        };

        template <class Processor, class Iterator, class BigCell>
            static bool process(grid_type & grid, Processor & sproc, 
                typename grid_type::iterator bit, BigCell & bigcell, Iterator p, Iterator q)
        {
            for (Iterator sit = p; sit != q; ++sit)
            {
               {
                  state st(grid, grid.index(bit), bigcell.index(sit));

                  if (sproc(st, *sit))
                     return true;
               }
            }    
            return false;
        }

        template <class Processor, class BigCell>
            static bool process(grid_type & grid, BigCell & bigcell, Processor & sproc, typename grid_type::iterator bit)
        {
            if (bigcell)
            {
                if ( process(grid, sproc, bit, bigcell, bigcell.begin(), bigcell.end()) )
                    return true;
            }
            return false;
        }

        template <class Processor>
            static bool process(grid_type & grid, Processor & processor)
        {

#pragma warning (push)
#pragma warning (disable : 4239)

            typename Processor::BigCellProcessor &
                bigcell_proc = 
                    static_cast<typename Processor::BigCellProcessor &>
                        (processor.processgrid());
            
            for (grid_type::iterator bit = grid.begin(); bit != grid.end(); ++bit)
            {
                state_bigcell st_big(grid, grid.index(bit));

                typename Processor::SmallCellProcessor &
                    sproc = bigcell_proc.processbigcell(st_big, *bit);

                if ( process (grid, *bit, sproc, bit) )
                    return true;
           }

#pragma warning (pop)

            return false;
        }
    };

}