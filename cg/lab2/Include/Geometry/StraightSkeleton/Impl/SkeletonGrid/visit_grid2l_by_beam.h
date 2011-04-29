#pragma once

#include <stdlib.h>

#include "Geometry\beam.h"
#include "grid_common.h"

#include "visit_grid2l_by_convex_quad.h"

namespace cg
{

// Smallcell must contain visit_index variable

template< class grid_type >
   struct visit_grid2l_by_beam
{
   typedef typename grid_type::smallcell_type smallcell_type;
   typedef typename grid_type::bigcell_type   bigcell_type;
   typedef typename grid_type::index_type     index_type;

   typedef point_2i bigidx_type;
   typedef point_2i smallidx_type;

   typedef index_type state;

   template< class Processor >
      static bool process( grid_type &grid, beam const &b, Processor &processor, double maxShift )
   {
      cg::rectangle_2 gbox = bounding(grid);
      double maxSize = cg::norm(gbox.size());

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

      cg::point_2 dirStartEnd;
      //double startProj = b.dirStart * beamDir;
      //if (cg::eq_zero(startProj))
      //   return false;

      double ratio = cg::min(maxDist/* / startProj*/, maxSize);
      dirStartEnd = b.edge.P0() + b.dirStart * ratio;

      cg::point_2 dirEndEnd;
      //double endProj = b.dirEnd * beamDir;
      //if (cg::eq_zero(endProj))
      //   return false;

      ratio = cg::min(maxDist/* / endProj*/, maxSize);
      dirEndEnd = b.edge.P1() + b.dirEnd * ratio;

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
   struct ProcessInternal
{
   ProcessInternal ( BaseProcessor &base, Grid const &grid, CdtGrid const &cdtGrid, beam const &beam )
      : base_ (base), grid_ (grid), cdtGrid_ (cdtGrid), beam_ (beam)
   {
      if (!cg::eq(beam.edge.P0(), beam.edge.P1()))
      {
         beamDir_ = cg::normalized(cg::normal(beam.edge));
         beamEdgeDir_ = cg::normalized(beam.edge.P1() - beam.edge.P0());
      }
      else
      {
         beamEdgeDir_ = cg::normalized(beam.dirEnd - beam.dirStart);
         beamDir_ = cg::normal(beamEdgeDir_);
      }

      cg::point_2 edgeCenter = cg::center(beam.edge);
      beamContour_ = cdtGrid_.findContourPointBelongsTo(edgeCenter +
         beamDir_ * point_offset(edgeCenter, 1e2));
   }

   double point_offset( cg::point_2 const &p, double offset )
   {
      return max(abs(p.x), abs(p.y)) * offset * cg::epsilon< double >();
   }

   bool processRect( cg::rectangle_2 const &rect )
   {
      cg::point_2 corners[4];
      corners[0] = rect.lo();
      corners[1] = cg::point_2 (rect.hi().x, rect.lo().y);
      corners[2] = rect.hi();
      corners[3] = cg::point_2 (rect.lo().x, rect.hi().y);

      size_t nearestCorner;
      double minDist = std::numeric_limits< double >::max();
      for (size_t c = 0; c < 4; ++c)
      {
         double dist = (corners[c] - beam_.edge.P0()) * beamDir_;
         if (dist < 0)
            return true;

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
         //cdtGrid_.simpleShootRay(checkSeg, cg::contours::contour_id (-1)) != beamContour_)
      {
         return false;
      }

      return true;
   }

   template <class State, class cell_type>
      bool operator () ( State const &st, cell_type &cell )
   {
      //if (beamContour_ == -1)
      //   return true;

      cg::raster_2 bc_raster = grid_.bigcellraster(st.big);

      cg::rectangle_2 testRect (bc_raster(st.small), bc_raster(st.small) + bc_raster.unit());
      if (processRect(testRect))
         return base_(st, cell);

      return false;
   }

   bool processBigCell( cg::raster_2 const &bc_raster )
      {
      //if (beamContour_ == -1)
      //   return false;

      if (processRect(bounding(bc_raster)))
         return true;

      return false;
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

template< class T, class B, class Processor >
   inline bool visit( Grid2L< T, B > &grid, beam const &b, Processor &processor )
{
   return visit_grid2l_by_beam< Grid2L< T, B > >::process(grid, b, processor);
}

template< class Grid, class CdtGrid, class Processor >
   inline bool visit_internal( Grid &grid, CdtGrid &cdtGrid, beam const &b, Processor &processor, double maxShift )
{
   ProcessInternal< Processor, Grid::grid_type, CdtGrid > proc(processor, grid.grid(), cdtGrid, b);
   return visit_grid2l_by_beam< Grid::grid_type >::process(grid.grid(), b, proc, maxShift);
}

} // End of 'cg' namespace
