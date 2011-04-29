#pragma once

#include <geometry/primitives/segment.h>

namespace cg
{

namespace dcel
{

//
// Geometry access helpers
//

template< class DCEL >
   typename DCEL::point_type edgeVector( DCEL const & dcel, size_t idx )
{
   return dcel.vertex(dcel.edge(idx).vertexDestination).pos - dcel.vertex(dcel.edge(idx).vertexOrigin).pos;
}

template< class DCEL >
   typename DCEL::point_type edgeVector( DCEL const & dcel, typename DCEL::Edge const &edge )
{
   return dcel.vertex(edge.vertexDestination).pos - dcel.vertex(edge.vertexOrigin).pos;
}

template< class DCEL >
   cg::segment_t< typename DCEL::scalar_type, 2 > edgeSegment( DCEL const & dcel, size_t idx )
{
   return cg::segment_t< typename DCEL::scalar_type, 2 >
      (dcel.vertex(dcel.edge(idx).vertexOrigin).pos, dcel.vertex(dcel.edge(idx).vertexDestination).pos);
}

template< class DCEL >
   cg::segment_t< typename DCEL::scalar_type, 2 > edgeSegment( DCEL const & dcel,
                                                               typename DCEL::Edge const &edge )
{
   return cg::segment_t< typename DCEL::scalar_type, 2 >
      (dcel.vertex(edge.vertexOrigin).pos, dcel.vertex(edge.vertexDestination).pos);
}

//
// Connectivity data
//

template< class DCEL >
   bool verticesAdjacency( DCEL const & dcel, size_t origin, size_t destination )
{
   size_t incident = dcel.vertex(origin).incidentEdge;
   for (exiting_edge_const_iterator it = dcel.exitingEdgeBegin(incident); it != dcel.exitingEdgeEnd(incident); ++it)
   {
      if (it->vertexDestination == destination)
         return true;
   }

   return false;
}

template< class DCEL >
   bool edgesAdjacency( DCEL const & dcel, size_t e1, size_t e2 )
{
   return dcel.edge(e1).vertexOrigin == dcel.edge(e2).vertexOrigin ||
      dcel.edge(e1).vertexOrigin == dcel.edge(e2).vertexDestination ||
      dcel.edge(e1).vertexDestination == dcel.edge(e2).vertexOrigin ||
      dcel.edge(e1).vertexDestination == dcel.edge(e2).vertexDestination;
}

template< class DCEL >
   bool vertexEdgeAdjacency( DCEL const & dcel, size_t v, size_t e )
{
   return dcel.edge(e).vertexOrigin == v || dcel.edge(e).vertexDestination == v;
}

//
// Components properties
//

// Check if edges are equal segments
template< class DCEL >
   bool areEdgesEqual( DCEL const &dcel, size_t e1, size_t e2 )
{
   size_t
      e1o = dcel.edge(e1).vertexOrigin,
      e2o = dcel.edge(e2).vertexOrigin,
      e1d = dcel.edge(e1).vertexDestination,
      e2d = dcel.edge(e2).vertexDestination;

   return (dcel.vertex(e1o).pos == dcel.vertex(e2o).pos &&
           dcel.vertex(e1d).pos == dcel.vertex(e2d).pos) ||
          (dcel.vertex(e1d).pos == dcel.vertex(e2o).pos &&
           dcel.vertex(e1o).pos == dcel.vertex(e2d).pos);
}

template< class DCEL >
   struct exiting_edge_const_itpair :
      public std::pair< typename DCEL::exiting_edge_const_iterator, typename DCEL::exiting_edge_const_iterator >
{
};

template< class DCEL >
   exiting_edge_const_itpair< DCEL > make_edges_pair( typename DCEL::exiting_edge_const_iterator edgeItA, typename DCEL::exiting_edge_const_iterator edgeItB )
{
   exiting_edge_const_itpair< DCEL > result;
   result.first = edgeItA;
   result.second = edgeItB;
   return result;
}

// Obtain equal exiting edges sequence
template< class DCEL >
   exiting_edge_const_itpair< DCEL > exitingEdgeEqualRange( DCEL const & dcel, typename DCEL::exiting_edge_const_iterator edgeIt )
{
   exiting_edge_const_itpair< DCEL > result = make_edges_pair< DCEL >(edgeIt, edgeIt);
   typename DCEL::exiting_edge_const_iterator eIt = result.first;
   do 
   {
      result.first = eIt;
      --eIt;
   } while (eIt.index() != edgeIt.index() && areEdgesEqual(dcel, eIt.index(), edgeIt.index()));
   eIt = result.second;
   do 
   {
      ++eIt;
   } while(eIt.index() != edgeIt.index() && areEdgesEqual(dcel, eIt.index(), edgeIt.index()));
   result.second = eIt;
   return result;
}

//
template< class DCEL >
   bool isVertexDangling( DCEL const &dcel, size_t vertexIdx )
{
   size_t incidentEdge = dcel.vertex(vertexIdx).incidentEdge;
   for (DCEL::exiting_edge_const_iterator eIt = util::next(dcel.exitingEdgeBegin(incidentEdge));
                                          eIt != dcel.exitingEdgeEnd(incidentEdge); ++eIt)
   {
      if (!areEdgesEqual(dcel, incidentEdge, eIt.index()))
         return false;
   }

   return true;
}

//
template< class DCEL >
   exiting_edge_const_itpair< DCEL > exitingEdgeEqualRangeDanglingSupport( DCEL const &dcel, typename DCEL::exiting_edge_const_iterator edgeIt )
{
   if (isVertexDangling(dcel, edgeIt->vertexOrigin))
      return make_edges_pair< DCEL >(edgeIt, dcel.exitingEdgeEnd(edgeIt.index()));

   return exitingEdgeEqualRange(dcel, edgeIt);
}
   
// Check if vertex in poly-dcel is reflex
template< class DCEL >
   bool isVertexReflex( DCEL const &dcel, size_t prevEdge, size_t nextEdge )
{
   DCEL::PointT const &v2 = vertexPos(edge(prevEdge).vertexDestination);

   return ((vertexPos(edge(nextEdge).vertexDestination) - v2) ^
      (v2 - vertexPos(edge(prevEdge).vertexOrigin))) >= 0;
}

// Check if cycle is ordered CCW
template< class DCEL >
   bool cycleIsOrderedCCW( DCEL const &dcel,
                           typename DCEL::cycle_edge_const_iterator cycleBegin,
                           typename DCEL::cycle_edge_const_iterator cycleEnd )
{
   std::vector< DCEL::PointT > cycle;

   cycle.push_back(dcel.vertexPos(cycleBegin->vertexOrigin));
   for (DCEL::cycle_edge_const_iterator it = cycleBegin; it != cycleEnd; ++it)
      cycle.push_back(dcel.vertexPos(it->vertexDestination));

   return !cg::clockwise_ordered(cycle.begin(), cycle.end());
}

//template< class DCEL >
//   bool edgeIsWire( DCEL const &dcel, size_t idx ) const
//{
//   return dcel.edge(idx).hole == dcel.edge(dcel.edge(idx).twinEdge).hole && dcel.edge(idx).hole;
//}
//
//template< class DCEL >
//   bool edgeIsBorder( DCEL const &dcel, size_t idx ) const
//{
//   return (dcel.edge(idx).hole != dcel.edge(dcel.edge(idx).twinEdge).hole) || dcel.edge(idx).hole;
//}

//
// Transformation utilities
//

// Replace oldVertex in dcel with newVertex
template< class DCEL >
   void substituteVertex( DCEL &dcel, size_t oldVertex, size_t newVertex )
{
   size_t incident = dcel.vertex(oldVertex).incidentEdge;
   size_t twinInc = dcel.edge(incident).twinEdge;

   DCEL::exiting_edge_iterator exiting = dcel.exitingEdgeBegin(incident);
   DCEL::entering_edge_iterator entering = dcel.enteringEdgeBegin(twinInc);

   for (; exiting != dcel.exitingEdgeEnd(incident) && entering != dcel.enteringEdgeEnd(twinInc); ++exiting, ++entering)
   {
      exiting->vertexOrigin = newVertex;
      entering->vertexDestination = newVertex;
   }
}

// Convert sequence of contour to dcel
template< class DCEL, class ContourType >
   bool convertContourToDCEL( DCEL &dcel, ContourType const &contour, typename DCEL::edge_data const & ed = DCEL::edge_data())
{
   bool non_empty = false; 

   for (ContourType::const_iterator ci = contour.begin(); ci != contour.end(); ++ci)
   {
      if (ci->size() < 3)
         continue;

      typedef typename ContourType::value_type SubcontourType;
      SubcontourType::const_iterator cend = ci->end();
      if (*ci->begin() == *util::prev(cend))
         --cend;

      dcel.addContour(ci->begin(), cend, cg::dcel::NoVertexData (), ed);

      non_empty = true;
   }

   return non_empty;
}

// Convert dcel to sequence of contours
template< class ContourType, class DCEL >
   void convertDCELToContour( ContourType &contour, DCEL const &dcel )
{
   for (DCEL::cycle_const_iterator cIt = dcel.cyclesBegin(); cIt != dcel.cyclesEnd(); ++cIt)
   {
      typedef ContourType::value_type SubcontourType;
      contour.push_back(SubcontourType ());
      SubcontourType &sc = contour.back();

      for (DCEL::cycle_edge_const_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
         sc.push_back(dcel.vertex(eIt->vertexOrigin).pos);
      //sc.updateType(); // TODO: Maybe this is needed.
   }
}

template <class Iterator, class ScType, class DCEL>
   void convertNthDcelToPolygon_impl(Iterator begin, Iterator beyond, ScType & sc, DCEL const &dcel)   
   {
      for (Iterator eIt = begin; eIt != beyond; ++eIt)
         sc.push_back(dcel.vertex(eIt->vertexOrigin).pos);
      sc.updateType();
   }

template <class Iterator, class ContourType, class DCEL>
   void convertNthDcelToPolygon(Iterator begin, Iterator beyond, int i, ContourType & contour, DCEL const &dcel)
{
   for (Iterator cIt = begin; cIt != beyond; ++cIt)
   {
      typedef ContourType::value_type SubcontourType;
      contour.push_back(SubcontourType ());
      SubcontourType &sc = contour.back();

      convertNthDcelToPolygon_impl(cIt->begin, cIt->end, sc, dcel);
   }
}

template <class ContourType, class DCEL, class Traits>
   void convertNthDcelToPolygon(int i, ContourType & contour, DCEL const &dcel, std::set<size_t> const & segs, Traits const & traits)
{
   convertNthDcelToPolygon(dcel.cyclesBegin(i,segs, traits), dcel.cyclesEnd(i,segs, traits), i, contour, dcel);
}

template< class ContourType, class DCEL >
   void convertDCELToContoursByData( ContourType &contour, DCEL const &dcel )
{
   //Profiler::CProfiler profiler("convertContourToDCEL");
   throw "not implemented";
   /*for (DCEL::filtered_cycle_const_iterator cIt = dcel.filteredCyclesBegin(); cIt != dcel.filteredCyclesEnd(); ++cIt)
   {
      int t = -1;
      for (DCEL::filtered_cycle_edge_const_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
         if (eIt->data != -1)
            if (t == -1) 
               t = eIt->data;

      if (t == -1)
         continue;

      typedef ContourType::underlying_type::value_type SubcontourType;
      contour(t).push_back(SubcontourType ());
      SubcontourType &sc = contour(t).back();

      for (DCEL::filtered_cycle_edge_const_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
         sc.push_back(dcel.vertex(eIt->vertexOrigin).pos);
   }*/
}

// Reverse DCEL edges
template< class DCEL >
   void reverse( DCEL & dcel )
{
   for (DCEL::edges_iterator eit = dcel.edgesBegin(); eit != dcel.edgesEnd(); ++eit)
   {
      if (dcel.vertex(eit->vertexOrigin).incidentEdge == eit.index())
         dcel.vertex(eit->vertexOrigin).incidentEdge = eit->twinEdge;
      std::swap(eit->vertexOrigin, eit->vertexDestination);
      std::swap(eit->nextEdge, eit->prevEdge);
   }
}

// Depth-First-Search for vertex in DCEL with predicate

namespace details
{
   template< class DCEL, class Predicate, class FlagsStorage >
      size_t vertexDFS_recursive( DCEL const &dcel, Predicate pred,
                                  size_t startVertex, FlagsStorage &storage )
   {
      if (pred(startVertex))
         return startVertex;

      storage[startVertex] = true;
      size_t incEdge = dcel.vertex(startVertex).incidentEdge;

      if (incEdge == -1)
         return static_cast< size_t >( -1 );

      for (DCEL::exiting_edge_const_iterator eIt = dcel.exitingEdgeBegin(incEdge);
                                             eIt != dcel.exitingEdgeEnd(incEdge); ++eIt)
      {
         size_t nextVertex = eIt->vertexDestination;
         if (storage[nextVertex])
            continue;

         size_t res = vertexDFS_recursive(dcel, pred, nextVertex, storage);
         if (res != -1)
            return res;
      }

      return static_cast< size_t >( -1 );
   }
}

template< class DCEL, class Predicate >
   size_t vertexDFS( DCEL const &dcel, Predicate pred, size_t startVertex = 0 )
{
   std::vector< bool > visited (dcel.verticesSize(), false);
   return details::vertexDFS_recursive(dcel, pred, startVertex, visited);
}

} // End of 'dcel' namespace
} // End of 'cg' namespace
