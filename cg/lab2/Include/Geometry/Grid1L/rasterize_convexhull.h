#pragma once

#include "Geometry\rasterization\segment_raster_iterator.h"

namespace cg
{
    namespace details
    {
        struct SegmentContructionCellProcessor 
        {
            typedef traster_details::Sides  Sides;

            SegmentContructionCellProcessor(Sides &sides)
                :  sides_(sides)
            {}

            bool operator () (point_2i const & idx, double, double)
            {
                Assert(sides_.is_valid(idx.y));
                sides_.get(idx.y).unite(idx.x);

                return false;
            }

        private:
            Sides               & sides_;
        };
    }

    template <class FwdIter, class Processor>
        bool rasterizeConvexHull(FwdIter first, FwdIter beyond, Processor & processor)
    {
        cg::range_2i y_rng;

        for (FwdIter p = first; p != beyond; ++p)
        {
           y_rng |= (int)floor(*p).y;
        }

        int min_y = y_rng.lo();
        int max_y = y_rng.hi();

        traster_details::Sides       sides(min_y, max_y - min_y + 1);
        details::SegmentContructionCellProcessor sproc(sides);

        for (FwdIter p = first; p != beyond-1;)
        {
            FwdIter current = p;
            FwdIter next    = ++p;

            rasterize_segment(cg::segment_2(*current, *next), sproc);
        }

        point_2i idx_big;

        for (idx_big.y = min_y; idx_big.y <= max_y; ++idx_big.y) 
        {
            range_2i const & x_range = sides.get(idx_big.y);

            for (idx_big.x = x_range.lo(); idx_big.x <= x_range.hi(); ++idx_big.x)
            {
                if (processor(idx_big))
                    return true;
            }
        }

        return false;
    }
}