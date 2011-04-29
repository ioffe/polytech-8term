#pragma once
#include "Geometry\Grid1L.h"
#include "Geometry\Grid1L\visit_grid1l_every_cell.h"

namespace cg
{
    struct HitCounter : Grid1L<int>
    {
        HitCounter( raster_2 const &raster, size_t def_count=0 )
            : Grid1L<int>(raster, raster.extents(), 0)
        {
            if (def_count)
            {
               UpdateCell updater((int)def_count);
               visit_every_cell(*this, updater);
            }
        }

        template <class U>
            HitCounter(Grid1L<U> const &other)  
              : Grid1L<int>(other, 0)
        {}

        HitCounter(aa_transform const &tform, point_2i const &extents)
            : Grid1L<int>(tform, extents, 0)
        {}

        struct UpdateCell
        {
            UpdateCell(int count)
                : count(count)
            {}

            template <class State>
                bool operator () (State, int & hit_count)
            {
                hit_count += count;
                return false;
            }

            struct EmptyProcessor
            {
               template < class State >
                   bool operator ( ) ( State, int & )
               {
                   return false;
               }
            };

            EmptyProcessor side_processor( int, int )
            {
               return EmptyProcessor( );
            }


            int count;
        };

        cg::point_2i operator () (cg::point_2i const & idx) const
        {
            return cg::point_2i(at(idx), at(idx));
        }

        template <class Element>
            void add(Element const &element, int count=1)
        {
            UpdateCell updater(count);
            visit(*this, element, updater);
        }
    };

}