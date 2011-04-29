#pragma once

#include "dcel.h"

namespace cg
{

namespace dcel
{

namespace details
{

/* After rearrangement of elements in dcel vertex array call this function to validate dcel edge info */
template< class DCEL >
   void restore_edges_vertex_indices( DCEL &dcel )
{
   for (DCEL::vertices_const_iterator it = dcel.verticesBegin(); it != dcel.verticesEnd(); ++it)
   {
      size_t twinInc = dcel.edge(it->incidentEdge).twinEdge;

      DCEL::exiting_edge_iterator exiting = dcel.exitingEdgeBegin(it->incidentEdge);
      DCEL::entering_edge_iterator entering = dcel.enteringEdgeBegin(twinInc);

      for (; exiting != dcel.exitingEdgeEnd(it->incidentEdge) &&
            entering != dcel.enteringEdgeEnd(twinInc); ++exiting, ++entering)
      {
         exiting->vertexOrigin = it - dcel.verticesBegin();
         entering->vertexDestination = it - dcel.verticesBegin();
      }
   }
}

} // End of 'details' namespace

/* Sort dcel vertices in order specified by less predicate */
template< class DCEL, class LessPredicate >
   void sort_vertices( DCEL &dcel, LessPredicate predicate )
{
   std::sort(dcel.verticesBegin(), dcel.verticesEnd(), predicate);
   details::restore_edges_vertex_indices(dcel);
}

/* Sample predicate for sort_vertices algorithm */
struct VertexYGreater
{
   template< class Vertex >
      bool operator () ( Vertex const &v1, Vertex const &v2 ) const
   {
      if (v1.pos.y == v2.pos.y)
         return v1.pos.x < v2.pos.x;

      return v1.pos.y > v2.pos.y;
   }
};

/* Subdivide dcel edges while they are longer than 'maxEdgeLength' */
template< class DCEL >
   void subdivideSeq( DCEL &dcel,
                      typename DCEL::cycle_edge_iterator begin,
                      typename DCEL::cycle_edge_iterator end, float maxEdgeLength,
                      typename DCEL::cycle_const_iterator cIt = typename DCEL::cycle_const_iterator () )
{
   typedef typename DCEL::Edge Edge;

   for (DCEL::cycle_edge_iterator eIt = begin; eIt != end; ++eIt)
   {
      cg::segment_2 edgeSeg = cg::dcel::edgeSegment(dcel, *eIt);
      double length = cg::norm(edgeSeg.P1() - edgeSeg.P0());

      if (length > maxEdgeLength)
      {
         double part = maxEdgeLength / length;
         cg::point_2 newPos = edgeSeg.P0() * (1 - part) + edgeSeg.P1() * part;
         if (!cg::eq(newPos, edgeSeg.P0(), 1e-4) && !cg::eq(newPos, edgeSeg.P1(), 1e-4))
         {
            size_t newVertex = dcel.addVertex(dcel.vertex(eIt->vertexOrigin));
            dcel.vertex(newVertex).pos = newPos;

            size_t twinNextEdge = dcel.edge(eIt->twinEdge).nextEdge;

            size_t newCurEdge = dcel.addEdge(Edge (newVertex, eIt->vertexDestination,
               eIt.index(), eIt->nextEdge, eIt->twinEdge, eIt->hole));
            size_t newTwinEdge = dcel.addEdge(Edge (newVertex, eIt->vertexOrigin,
               eIt->twinEdge, twinNextEdge, eIt.index(), dcel.edge(eIt->twinEdge).hole));

            eIt->vertexDestination = dcel.edge(eIt->twinEdge).vertexDestination = newVertex;

            dcel.edge(eIt->twinEdge).twinEdge = newCurEdge;
            dcel.edge(eIt->twinEdge).nextEdge = newTwinEdge;
            eIt->twinEdge = newTwinEdge;

            dcel.edge(eIt->nextEdge).prevEdge = newCurEdge;
            eIt->nextEdge = newCurEdge;

            dcel.edge(twinNextEdge).prevEdge = newTwinEdge;
            dcel.vertex(newVertex).incidentEdge = newCurEdge;

            cIt.skip(newCurEdge);
            cIt.skip(newTwinEdge);
         }
      }
   }
}

template< class DCEL >
   void subdivide( DCEL &dcel, float maxEdgeLength )
{
   for (DCEL::cycle_iterator cIt = dcel.cyclesBegin(false); cIt != dcel.cyclesEnd(false); ++cIt)
      subdivideSeq(dcel, cIt->begin, cIt->end, maxEdgeLength, cIt);
}

template< class DCEL >
   void combine( DCEL &a, DCEL const &b )
{
   size_t vBase = a.verticesSize();
   size_t eBase = a.edges_.containerSize();

   a.vertices_.insert(a.vertices_.end(), b.vertices_.begin(), b.vertices_.end());
   for (size_t v = vBase; v < a.vertices_.size(); ++v)
      a.vertex(v).incidentEdge += eBase;

   DCEL::EdgesArray::iterator eIt = a.edges_.push(b.edges_);
   for (; eIt != a.edges_.end(); ++eIt)
   {
      eIt->twinEdge += eBase;
      eIt->prevEdge += eBase;
      eIt->nextEdge += eBase;

      eIt->vertexOrigin += vBase;
      eIt->vertexDestination += vBase;
   }
}

} // End of 'dcel' namespace
} // End of 'cg' namespace
