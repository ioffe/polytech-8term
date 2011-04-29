#pragma once

#include <stdlib.h>

#include "grid_common.h"
#include "Geometry\beam.h"

#include "visit_grid1l_by_convex_quad.h"

namespace cg
{

// Smallcell must contain visit_index variable

template< class grid_type >
   struct visit_grid1l_by_beam
{
   typedef typename grid_type::index_type     index_type;

   typedef point_2i bigidx_type;
   typedef point_2i smallidx_type;

   typedef index_type state;

   template< class Processor >
      static bool process( grid_type &grid, beam const &b, Processor &processor, double maxShift )
   {
      cg::rectangle_2 gbox = bounding(grid);

      cg::point_2 corners[4] =
      {
         gbox.lo(),
         cg::point_2 (gbox.hi().x, gbox.lo().y),
         gbox.hi(),
         cg::point_2 (gbox.lo().x, gbox.hi().y)
      };

      point_2 beamDir;
      if (!cg::eq(b.edge.P0(), b.edge.P1()))
         beamDir = cg::normalized(cg::normal(b.edge));
      else
         beamDir = cg::normalized(cg::normal(b.dirEnd - b.dirStart));

      size_t maxCorner;
      double maxDist = std::numeric_limits< double >::min();
      for (size_t c = 0; c < 4; c++)
      {
         double curDist = (corners[c] - b.edge.P0()) * beamDir;
         if (curDist > maxDist)
         {
            maxDist = curDist;
            maxCorner = c;
         }
      }

      maxDist = cg::min(maxShift, maxDist);

      cg::point_2 dirStartEnd = b.edge.P0() + b.dirStart * maxDist/* / (b.dirStart * beamDir))*/;
      cg::point_2 dirEndEnd = b.edge.P1() + b.dirEnd * maxDist/* / (b.dirEnd * beamDir))*/;

      cg::point_2 intersectionPoint;
      if (!cg::eq(b.edge.P0(), b.edge.P1()) &&
           cg::generic_intersection(cg::segment_2 (b.edge.P0(), dirStartEnd),
                                    cg::segment_2 (b.edge.P1(), dirEndEnd),
                                    &intersectionPoint, (cg::point_2 *)NULL) != cg::disjoint)
      {
         cg::visit(grid, cg::quad_2 (b.edge.P0(), b.edge.P1(), intersectionPoint, intersectionPoint), processor);
      }
      else
      {
         cg::visit(grid, cg::quad_2 (b.edge.P0(), b.edge.P1(), dirEndEnd, dirStartEnd), processor);
      }

      return false;
   }    
};

template< class BaseProcessor, class Grid, class CdtGrid >
   struct ProcessInternal_1L
{
   ProcessInternal_1L ( BaseProcessor &base, Grid const &grid, CdtGrid &cdtGrid, beam const &beam )
      : base_ (base), grid_ (grid), cdtGrid_ (cdtGrid), beam_ (beam)
   {
      if (!cg::eq(beam.edge.P0(), beam.edge.P1()))
         beamDir_ = cg::normalized(cg::normal(beam.edge));
      else
         beamDir_ = cg::normalized(cg::normal(beam.dirEnd - beam.dirStart));

      beamEdgeDir_ = -cg::normal(beamDir_);

      beamContour_ = cdtGrid_.findContourPointBelongsTo(0.5 * (beam.edge.P0() + beam.edge.P1()) +
         beamDir_ * 1e2 * cg::epsilon< double >());
   }

   template <class State, class cell_type>
      bool operator () ( State const &st, cell_type &cell )
   {
      //if (beamContour_ == -1)
      //   return true;

      cg::point_2 corners[4];
      corners[0] = grid_.origin() + cg::point_2 (grid_.unit().x * st.x, grid_.unit().y * st.y);
      corners[1] = corners[0] + cg::point_2 (grid_.unit().x, 0);
      corners[2] = corners[0] + grid_.unit();
      corners[3] = corners[0] + cg::point_2 (0, grid_.unit().y);

      size_t nearestCorner;
      double minDist = std::numeric_limits< double >::max();
      for (size_t c = 0; c < 4; ++c)
      {
         double dist = (corners[c] - beam_.edge.P0()) * beamDir_;
         if (dist < 0)
            return base_(st, cell);

         if (dist < minDist)
         {
            minDist = dist;
            nearestCorner = c;
         }
      }

      cg::point_2 A = corners[nearestCorner] + beamEdgeDir_ *
         ((corners[(nearestCorner + 3) % 4] - corners[nearestCorner]) * beamEdgeDir_);
      cg::point_2 B = corners[nearestCorner] + beamEdgeDir_ *
         ((corners[(nearestCorner + 1) % 4] - corners[nearestCorner]) * beamEdgeDir_);

      cg::segment_2 checkSeg (A, B);

      if (cdtGrid_.findContourPointBelongsTo(corners[nearestCorner]) != beamContour_ &&
          !cdtGrid_.rayIntersectContour(checkSeg, beamContour_))
          // cdtGrid_.simpleShootRay(checkSeg, cg::contours::contour_id (-1)) != beamContour_)
      {
          return false;
      }

      return base_(st, cell);
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   BaseProcessor &base_;
   empty_processor dummy_;

   Grid const &grid_;
   CdtGrid const &cdtGrid_;
   beam const &beam_;

   cg::point_2 beamDir_;
   cg::point_2 beamEdgeDir_;

   cg::contours::contour_id beamContour_;
};

template< class T, class CdtGrid, class Processor >
   inline bool visit( Grid1L< T > &grid, CdtGrid &cdtGrid, beam const &b, Processor &processor, double maxShift )
{
   ProcessInternal_1L< Processor, Grid1L< T >, CdtGrid > proc(processor, grid, cdtGrid, b);
   return visit_grid1l_by_beam< Grid1L< T > >::process(grid, b, proc, maxShift);
}

} // End of 'cg' namespace
