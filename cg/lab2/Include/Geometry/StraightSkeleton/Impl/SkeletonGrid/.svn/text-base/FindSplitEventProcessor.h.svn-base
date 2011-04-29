#pragma once

#include "..\FindSplitIntersection.h"

namespace cg {
namespace skeleton {

template< class GridType, class Skeleton >
   struct FindSplitEventProcessor
      : cg::grid2l_visitor_base< GridType, FindSplitEventProcessor< GridType, Skeleton > >
{
   FindSplitEventProcessor ( GridType &grid, Skeleton &skeleton,
                             double curTime, size_t vertexIdx,
                             size_t &edgeIdx, double &resT, cg::point_2 &intersection,
                             bool &resFound, size_t excludeEdgeA, size_t excludeEdgeB,
                             bool preprocessing = false, bool exact = false )
      : grid_ (grid), skeleton_ (skeleton)
      , curTime_ (curTime), vertexIdx_ (vertexIdx)
      , edgeIdx_ (edgeIdx), resT_ (resT), intersection_ (intersection), resFound_ (resFound)
      , preprocessing_ (preprocessing)
      , exact_ (exact)
      , excludeEdgeA_ (excludeEdgeA), excludeEdgeB_ (excludeEdgeB)
      , localCenter_ (skeleton.dcel().vertex(vertexIdx).pos)
   {
      resT_ = std::numeric_limits< double >::max();
      resFound_ = false;

      //origin_ = skeleton.vertexPos(vertexIdx);
      //direction_ = skeleton.dcel().vertex(vertexIdx).data.bisector;
      //dirNormSqr_ = cg::norm_sqr(direction_);
   }

   template< class State, class CellType >
      bool operator () ( State const &st, CellType &cell )
   {
      cg::raster_2 bc_raster = grid_.bigcellraster(st.big);
      cg::rectangle_2 cellRect (bc_raster(st.small), bc_raster(st.small) + bc_raster.unit());

      //cg::range_2 cellRange = getRectRangeForBisector(origin_, direction_, dirNormSqr_, cellRect);
      //if (!cellRange.empty())
      //   cellRange = cg::range_2 (cellRange.lo() + skeleton_.getTime(), cellRange.hi() + skeleton_.getTime());
      //if (!has_intersection(cellRange, cell.low) && !has_intersection(cellRange, cell.high))
      //   return false;

      typedef typename CellType::EdgeContainer::iterator edge_iterator;
      for (edge_iterator testEdge = cell.edges.begin(); testEdge != cell.edges.end(); ++testEdge)
      {
         if (skeleton_.dcel().edge(*testEdge/*->idx*/).hole ||
             vertexEdgeAdjacency(skeleton_.dcel(), vertexIdx_, *testEdge/*->idx*/))
            continue;

         if (*testEdge == excludeEdgeA_ || *testEdge == excludeEdgeB_)
            continue;

         //if (!cellRange.empty() && !testEdge->cRange.empty() && !has_intersection(cellRange, testEdge->cRange))
         //   continue;

         if (skeleton_.dcel().vertex(skeleton_.dcel().edge(*testEdge).vertexOrigin).data.processed ||
             skeleton_.dcel().vertex(skeleton_.dcel().edge(*testEdge).vertexDestination).data.processed)
         {
            continue;
         }

         cg::point_2 intersection;
         double eventT;
         if (cg::findSplitIntersection(skeleton_, vertexIdx_, *testEdge, intersection, eventT, preprocessing_, exact_))
         {
            if (cellRect.contains(intersection + localCenter_))
            {
               resFound_ = true;
               eventT += curTime_;
               if (eventT < resT_)
               {
                  resT_ = eventT;
                  intersection_ = intersection + localCenter_;
                  edgeIdx_ = *testEdge;
               }
            }
         }
      }

      return resFound_;
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   GridType &grid_;
   Skeleton &skeleton_;
   double curTime_;
   size_t vertexIdx_;
   cg::point_2 localCenter_;

   bool &resFound_;
   size_t &edgeIdx_;
   double &resT_;
   cg::point_2 &intersection_;

   bool preprocessing_;
   bool exact_;

   size_t excludeEdgeA_, excludeEdgeB_;
   //cg::point_2 origin_;
   //cg::point_2 direction_;
   //double dirNormSqr_;

   SideProcessor dummy_;
};

} // End of 'skeleton' namespace
} // End of 'cg' namespace
