#pragma once

namespace cg {
namespace skeleton {

template< class GridType >
   struct RemoveEdgeProcessor
      : cg::grid2l_visitor_base< GridType, RemoveEdgeProcessor< GridType > >
{
   RemoveEdgeProcessor ( size_t edgeIdx ) : edgeIdx_ (edgeIdx)
   {
   }

   template< class State, class CellType >
      bool operator () ( State const &st, CellType &cell )
   {
      typedef typename CellType::EdgeContainer::iterator edge_iterator;
      edge_iterator place = cell.edges.find(/*EdgeRep (*/edgeIdx_/*)*/);

      if (place != cell.edges.end())
         cell.edges.erase(place);

      return false;
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   size_t edgeIdx_;
   SideProcessor dummy_;
};

template< class GridType >
   struct RemoveVertexProcessor
      : cg::grid2l_visitor_base< GridType, RemoveVertexProcessor< GridType > >
{
   RemoveVertexProcessor ( size_t vertexIdx ) : vertexIdx_ (vertexIdx)
   {
   }

   template< class State, class CellType >
      bool operator () ( State const &, CellType &cell )
   {
      typedef typename CellType::VerticesContainer::iterator vertex_iterator;
      vertex_iterator place = cell.vertices.find(vertexIdx_);

      if (place != cell.vertices.end())
         cell.vertices.erase(place);

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
