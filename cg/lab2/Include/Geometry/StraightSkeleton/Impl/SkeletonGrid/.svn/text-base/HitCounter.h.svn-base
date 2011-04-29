#pragma once
#include "Geometry\Grid1L.h"
#include "Geometry\Grid1L\visit_grid1l_every_cell.h"

#include "visit_grid1l_by_beam.h"

namespace cg {
namespace skeleton {

    template< class CdtGrid >
      struct SkeletonHitCounter : Grid1L<int>
    {
        SkeletonHitCounter( CdtGrid &cdtGrid, raster_2 const &raster, size_t def_count=0 )
            : Grid1L<int>(raster, raster.extents(), 0), cdtGrid_ (cdtGrid)
        {
            if (def_count)
                visit_every_cell(*this, UpdateCell(def_count));
        }

        template <class U>
            SkeletonHitCounter( CdtGrid &cdtGrid, Grid1L<U> const &other )  
              : Grid1L<int>(other, 0), cdtGrid_ (cdtGrid)
        {}

        SkeletonHitCounter( CdtGrid &cdtGrid, aa_transform const &tform, point_2i const &extents)
            : Grid1L<int>(tform, extents, 0), cdtGrid_ (cdtGrid)
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

            int count;
        };

        cg::point_2i operator () (cg::point_2i const & idx) const
        {
            return cg::point_2i(at(idx), at(idx));
        }

        template <class Element>
            void add(Element const &element, double maxShift, int count=1)
        {
            UpdateCell updater(count);
            visit(*this, cdtGrid_, element, updater, maxShift);
        }
    private:
       CdtGrid &cdtGrid_;
    };

} // End of 'skeleton' namespace
} // End of 'cg' namespace
