#pragma once

namespace cg {
namespace skeleton {

template< class GridType >
   struct AddEdgeProcessor
      : cg::grid2l_visitor_base< GridType, AddEdgeProcessor< GridType > >
{
   // template< class Skeleton >
      AddEdgeProcessor ( /*Skeleton &skeleton, GridType &grid, double curTime,*/ size_t edgeIdx )
      : /*grid_ (grid), curTime_ (curTime), */edgeIdx_ (edgeIdx)
   {
      /*
      cg::segment_2 edgeSeg = skeleton.edgeSegment(edgeIdx);
      origin_ = edgeSeg.P0();
      direction_ = cg::normalized(cg::normal(edgeSeg));
      */
   }

   template< class State, class CellType >
      bool operator () ( State const &, CellType &cell )
   {
      /*
      cg::raster_2 bc_raster = grid_.bigcellraster(st.big);
      cg::rectangle_2 testRect (bc_raster(st.small), bc_raster(st.small) + bc_raster.unit());

      cg::range_2 cellRange = getRectRange(origin_, direction_, testRect);
      cellRange = cg::range_2 (cellRange.lo() + curTime_, cellRange.hi() + curTime_);
      */

      cell.edges.insert(/*EdgeRep (*/edgeIdx_/*, cellRange)*/);

      // mergeRangePairRange(cell.low, cell.high, cellRange);

      return false;
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   size_t edgeIdx_;
   SideProcessor dummy_;

   //cg::point_2 origin_;
   //cg::point_2 direction_;
   //GridType &grid_;
   //double curTime_;
};

template< class GridType >
   struct AddVertexProcessor
      : cg::grid2l_visitor_base< GridType, AddVertexProcessor< GridType > >
{
   AddVertexProcessor ( size_t vertexIdx )
      : vertexIdx_ (vertexIdx)
   {
   }

   template< class State, class CellType >
      bool operator () ( State const &, CellType &cell )
   {
      cell.vertices.insert(vertexIdx_);
      return false;
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   size_t vertexIdx_;
   SideProcessor dummy_;
};

} // End of 'skeleton' namespace
} // End of 'cg' namespace
