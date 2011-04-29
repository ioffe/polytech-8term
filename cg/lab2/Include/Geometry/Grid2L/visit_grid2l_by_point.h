#pragma  once
#include "Grid2l_Impl.h"

namespace cg 
{

    template <class G>
        struct visit_grid2l_by_point
    {
        typedef G                       grid_type;
        typedef typename G::smallcell_type       smallcell_type;
        typedef typename G::bigcell_type         bigcell_type;
        typedef typename G::index_type           index_type;
        typedef point_2i                bigidx_type;
        typedef point_2i                smallidx_type;

        struct state : index_type
        {
            __forceinline state (index_type idx, grid_type & g) : index_type(idx), grid_(g) {}

            // TODO: Вынести в grid_holder
            __forceinline grid_type       & grid ()       { return grid_; }
            __forceinline grid_type const & grid () const { return grid_; }

            __forceinline rectangle_2 smallcellbound() const {
                return cg::smallcell_bound(grid_, *this);
            }

            __forceinline void set(index_type const & idx) { static_cast<index_type&>(*this) = idx; }

        private:
            grid_type & grid_;
        };

        struct state_bigcell : bigidx_type
        {
            __forceinline state_bigcell (grid_type & grid, bigidx_type const &bigidx)
              : grid_(grid), bigidx_type(bigidx)
            {}

            __forceinline grid_type       & grid ()       { return grid_; }
            __forceinline grid_type const & grid () const { return grid_; }

        private:
            grid_type & grid_;
        };

        __forceinline static index_type calc_idx( grid_type & grid, point_2 const & pt )
        {
            point_2  pt_in_grid = grid.world2local(pt);
            point_2i idx_big = floor(pt_in_grid);

            if (grid.contains(idx_big))
            {
                bigcell_type & bigcell = grid[idx_big];

                if ( bigcell )
                {
                  point_2i idx_small = 
                    floor((pt_in_grid - idx_big) & bigcell.extents());

                  make_max( idx_small.x, 0 );
                  make_max( idx_small.y, 0 );
                  make_min( idx_small.x, bigcell.extents( ).x );
                  make_min( idx_small.y, bigcell.extents( ).y );

                  return index_type( idx_big, idx_small );
                }
            }

            return index_type( bigidx_type( -1, -1 ), smallidx_type() );
        }

        template <class Processor>
            __forceinline static bool process(grid_type & grid, point_2 const &pt, Processor & processor)
        {
           return process_in_scell( grid, pt, processor, calc_idx( grid, pt ) );
        }

        template <class Processor>
            __forceinline static bool process(grid_type & grid, point_2 const &pt, Processor & processor,
                                              index_type & idx )
        {
           if ( !grid.contains( idx.big ) || !cg::smallcell_bound( grid, idx ).contains( pt ) )
              idx = calc_idx( grid, pt );
           return process_in_scell( grid, pt, processor, idx );
        }

        template <class Processor>
            __forceinline static bool process_in_scell(grid_type & grid, point_2 const &, Processor & processor,
                                                       index_type const & idx )
        {
           if (grid.contains(idx.big))
           {
              bigcell_type & bigcell = grid[idx.big];

              if ( bigcell )
              {
                 state_bigcell st_big(grid, idx.big);

                 typename Processor::SmallCellProcessor &
                    sproc = processor.processbigcell(st_big, bigcell);

                 state st(idx, grid);

                 if (sproc(st, bigcell.at(idx.small)))
                    return true;
              }

           }

           return false;
        }
    };

    template <class T, class B, class H, class Processor>
        __forceinline  bool visit(Grid2L<T,B,H> & grid, point_2 const &pt, Processor &processor)
    {
        return visit_grid2l_by_point<Grid2L<T,B,H> >::process(grid, pt, processor);
    }

    template <class T, class B, class H, class Processor>
        __forceinline  bool visit(Grid2L<T,B,H> const & grid, point_2 const &pt, Processor &processor)
    {
        return visit_grid2l_by_point<Grid2L<T,B,H> >::process(const_cast<Grid2L<T,B,H>&>(grid), pt, processor);
    }

    template <class T, class B, class H, class Processor>
       __forceinline  bool visit(Grid2L<T,B,H> & grid, point_2 const &pt, Processor &processor, cg::Index2L & idx)
    {
        return visit_grid2l_by_point<Grid2L<T,B,H> >::process(grid, pt, processor, idx);
    }

    template <class T, class B, class H, class Processor>
       __forceinline  bool visit(Grid2L<T,B,H> const & grid, point_2 const &pt, Processor &processor, cg::Index2L & idx )
    {
        return visit_grid2l_by_point<Grid2L<T,B,H> >::process(const_cast<Grid2L<T,B,H>&>(grid), pt, processor, idx);
    }

}