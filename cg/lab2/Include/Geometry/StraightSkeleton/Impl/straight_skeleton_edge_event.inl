#pragma once

#include "Geometry\cgal_predicates.h"

namespace cg
{

SKELETON_EXT_METHOD(void) stopCycleVertices( EdgeList const &edgeList, bool useFixedPos,
                                             cg::point_2 const &stopPos )
{
   if (edgeList.size() == 0)
      return;

   EdgeList::const_iterator it = edgeList.begin();
   size_t curVertex = dcel_.edge(*it).vertexOrigin;
   if (attr_.optimize)
      grid_->removeVertex(curVertex, maxShiftEstimation_);
   stopVertexUpdate(curVertex);
   if (useFixedPos)
      dcel_.vertex(curVertex).pos = stopPos;
   while (it != edgeList.end())
   {
      curVertex = dcel_.edge(*it).vertexDestination;
      if (attr_.optimize)
         grid_->removeVertex(curVertex, maxShiftEstimation_);
      stopVertexUpdate(curVertex);
      if (useFixedPos)
         dcel_.vertex(curVertex).pos = stopPos;
      ++it;
   }
}

SKELETON_EXT_METHOD(size_t) collapse( EdgeList const &edgeList,
                                      VertexList &incVertices,
                                      ss::VertexData::VerticesSet &parents )
{
   bool looped = edgeList.front() == edgeList.back();

   Assert(edgeList.size() >= 2);
   size_t stopVertex = dcel_.edge(edgeList.back()).vertexOrigin;
   if (looped)
      stopVertex = dcel_.edge(*util::prev(util::prev(edgeList.end()))).vertexOrigin;

   // Correspondence info and vertex elimination
   size_t incidentEdge = static_cast< size_t >( -1 );
   if (looped)
   {
      for (EdgeList::const_reverse_iterator it = edgeList.rbegin(); it != edgeList.rend(); ++it)
      {
         size_t prevEdgeTwin = dcel_.edge(dcel_.edge(*it).prevEdge).twinEdge;
         size_t twinEdgeNext = dcel_.edge(dcel_.edge(*it).twinEdge).nextEdge;
         if (prevEdgeTwin != twinEdgeNext)
         {
            incidentEdge = twinEdgeNext;
            break;
         }
      }
   }

   for (EdgeList::const_iterator it = edgeList.begin(); it != (looped ? util::prev(edgeList.end()): edgeList.end()); ++it)
   {
      size_t edgeTwin = dcel_.edge(*it).twinEdge;
      size_t edgeTwinNext = dcel_.edge(edgeTwin).nextEdge;

      size_t prevEdge = dcel_.edge(*it).prevEdge;
      size_t prevEdgeTwin = dcel_.edge(prevEdge).twinEdge;

      size_t curVertex = dcel_.edge(*it).vertexOrigin;
      if (prevEdgeTwin == edgeTwinNext)
      {
         updateCorrespondenceData(curVertex, stopVertex, incidentEdge, true, &parents);
         if (incidentEdge == -1)
            incVertices.push_back(curVertex);
      }
      else
         incidentEdge = edgeTwinNext;

      if (curVertex != stopVertex)
         cg::dcel::substituteVertex(dcel_, curVertex, stopVertex);
   }

   for (EdgeList::const_iterator it = edgeList.begin(); it != util::prev(edgeList.end()); ++it)
   {
      deleteHalfEdge(dcel_.edge(*it).twinEdge);
      deleteHalfEdge(*it);
   }
   if (incidentEdge != -1)
      dcel_.vertex(stopVertex).incidentEdge = incidentEdge;

   return stopVertex;
}

SKELETON_EXT_METHOD(bool)
#ifndef SS_DEBUG_LOG
   handle_edge_event_degeneracy( cg::point_2 const &localCenter, size_t prevEdge, size_t nextEdge )
#else /* SS_DEBUG_LOG */
   handle_edge_event_degeneracy( cg::point_2 const &localCenter, size_t prevEdge, size_t nextEdge, bool logCurEvent )
#endif /* SS_DEBUG_LOG */
{
   // Degenerate bisector case
   bool degenerate = false;

   point_2 eventVector = edgeVector(ed_.eventEdge, localCenter);
   double eventLength = cg::norm_sqr(eventVector);
   if (cg::eq_zero(dcel_.vertex(ed_.eventVertex).data.bisector ^ eventVector))
   {
      double prevEdgeLength = cg::norm_sqr(edgeVector(prevEdge, localCenter));
      if (prevEdgeLength < eventLength)
      {
         degenerate = true;
         ss::EventData newEvent (ss::ET_EDGE, ed_.t, dcel_.edge(prevEdge).vertexOrigin, prevEdge);
         newEvent.nextVertex = ed_.eventVertex;
         newEvent.intersection = vertexPosFixed(dcel_.edge(prevEdge).vertexOrigin);
         if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
               newEvent.t <= dcel_.vertex(newEvent.eventVertex).data.iter->t)
         {
            restoreCurrentEvent();
            addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
         }

#ifdef SS_DEBUG_LOG
            if (logCurEvent)
               *debugLog_ << "degeneracy_type_0 ";
#endif /* SS_DEBUG_LOG */
      }
   }

   if (cg::eq_zero(dcel_.vertex(dcel_.edge(ed_.eventEdge).vertexDestination).data.bisector ^ eventVector))
   {
      double nextEdgeLength = cg::norm_sqr(edgeVector(nextEdge, localCenter));
      if (nextEdgeLength < eventLength)
      {
         degenerate = true;
         ss::EventData newEvent (ss::ET_EDGE, ed_.t, dcel_.edge(ed_.eventEdge).vertexDestination, nextEdge);
         newEvent.nextVertex = dcel_.edge(nextEdge).vertexDestination;
         newEvent.intersection = vertexPosFixed(dcel_.edge(nextEdge).vertexDestination);
         if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
               newEvent.t <= dcel_.vertex(newEvent.eventVertex).data.iter->t)
         {
            restoreCurrentEvent();
            addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
         }

#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << "degeneracy_type_1 ";
#endif /* SS_DEBUG_LOG */
      }
   }

   return degenerate;
}

SKELETON_EXT_METHOD(void) handle_edge_event()
{
   ss::EdgeListEx edgesToUpdate;
   ss::VertexList verticesToUpdate;

   cg::point_2 localCenter (dcel_.vertex(ed_.eventVertex).pos);

   //
   // 1. Obtain collapsing edges chain
   //

   EdgeList collapsingEdges;
   collapsingEdges.push_back(ed_.eventEdge);

   size_t prevEdge = dcel_.edge(ed_.eventEdge).prevEdge;
   while (prevEdge != ed_.eventEdge &&
          cg::eq(vertexPos(dcel_.edge(prevEdge).vertexOrigin, localCenter), ed_.intersection - localCenter, 1e-7))
   {
      collapsingEdges.push_front(prevEdge);
      prevEdge = dcel_.edge(prevEdge).prevEdge;
   }

   size_t nextEdge = dcel_.edge(ed_.eventEdge).nextEdge;
   while (nextEdge != *collapsingEdges.begin() &&
          cg::eq(vertexPos(dcel_.edge(nextEdge).vertexDestination, localCenter), ed_.intersection - localCenter, 1e-7))
   {
      collapsingEdges.push_back(nextEdge);
      nextEdge = dcel_.edge(nextEdge).nextEdge;
   }

   if (nextEdge != *collapsingEdges.begin() && dcel_.edge(nextEdge).nextEdge == prevEdge)
   {
      // Degenerate triangle case
      collapsingEdges.push_back(nextEdge);
      collapsingEdges.push_back(prevEdge);
      nextEdge = collapsingEdges.front();
      prevEdge = collapsingEdges.back();
   }

#ifdef SS_DEBUG_LOG
   bool logCurEvent = false;
   for (EdgeList::const_iterator it = collapsingEdges.begin(); it != collapsingEdges.end(); ++it)
   {
      if (verticesOfInterest_.find(dcel_.edge(*it).vertexOrigin) != verticesOfInterest_.end())
      {
         logCurEvent = true;
         break;
      }
   }
   if (!logCurEvent)
   {
      if (verticesOfInterest_.find(dcel_.edge(*util::prev(it)).vertexDestination) != verticesOfInterest_.end())
         logCurEvent = true;
   }

   if (logCurEvent)
   {
      *debugLog_ << step_ << " t=" << t_ << " ET_EDGE: vertices: ";
      for (EdgeList::const_iterator it = collapsingEdges.begin(); it != collapsingEdges.end(); ++it)
         *debugLog_ << dcel_.edge(*it).vertexOrigin << ' ';
      *debugLog_ << dcel_.edge(*util::prev(it)).vertexDestination << ' ';
   }
#endif /* SS_DEBUG_LOG */

   //
   // 2. Check for degenerate case (bisector || edge -> switch to shorter neighbor)
   //

   if (collapsingEdges.size() == 1)
   {
#ifndef SS_DEBUG_LOG
      if (handle_edge_event_degeneracy(localCenter, prevEdge, nextEdge))
#else /* SS_DEBUG_LOG */
      if (handle_edge_event_degeneracy(localCenter, prevEdge, nextEdge, logCurEvent))
#endif /* SS_DEBUG_LOG */
      {
#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << std::endl;
#endif /* SS_DEBUG_LOG */
         return;
      }
   }

   //
   // 3. Process collapse
   //

   cg::point_2 prevEdgeBisector = dcel_.vertex(dcel_.edge(prevEdge).vertexDestination).data.bisector;
   cg::point_2 nextEdgeBisector = dcel_.vertex(dcel_.edge(nextEdge).vertexOrigin).data.bisector;

   // Add split-vertex events (v, _), v is from opposite vertices of collapsing edges
   for (EdgeList::const_iterator it = collapsingEdges.begin(); it != collapsingEdges.end(); ++it)
      edgesToUpdate.push_back(ss::EdgeListElem (*it, ss::EUT_REMOVED));

   ss::EdgeListElem prevEdgeElem (prevEdge, ss::EUT_CHANGED);
   ss::EdgeListElem nextEdgeElem (nextEdge, ss::EUT_CHANGED);

   // Remove collapsing edges from grid
   if (attr_.optimize)
   {
      prevEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, prevEdge, prevEdgeElem.beamT);
      nextEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, nextEdge, nextEdgeElem.beamT);

      for (ss::EdgeListEx::iterator eIt = edgesToUpdate.begin(); eIt != edgesToUpdate.end(); ++eIt)
         eIt->initialBeam = cg::skeleton::edge2beam_prev(*this, eIt->edge, eIt->beamT);
   }

   // Stop motion of event vertices
   stopCycleVertices(collapsingEdges, true, ed_.intersection);

   // Connect to stop vertex
   collapsingEdges.push_back(nextEdge);
   VertexList incVertices;
   ss::VertexData::VerticesSet stopParents;
   size_t stopVertex = collapse(collapsingEdges, incVertices, stopParents);

   if (nextEdge == *collapsingEdges.begin())
   {
      dcel_.vertex(stopVertex).data.parentVertices.insert(stopParents.begin(), stopParents.end());
      if (attr_.optimize)
         update_queue_optimized(edgesToUpdate, verticesToUpdate);
      else
         update_queue(edgesToUpdate, verticesToUpdate);
#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << std::endl;
#endif /* SS_DEBUG_LOG */
      return; // 'Circle' case
   }

   //
   // 4. Create new moving vertex and connect it to stop vertex
   //

   // Create result vertex (to avoid queue reconstruction due to invalid events)
   size_t newVertex = dcel_.addVertex(dcel_.vertex(stopVertex));

#ifdef SS_DEBUG_LOG
   if (logCurEvent)
   {
      *debugLog_ << "newVertex: " << newVertex << ' ';
      verticesOfInterest_.insert(newVertex);
   }
#endif /* SS_DEBUG_LOG */

   dcel_.vertex(newVertex).data.processed = false;
   dcel_.vertex(newVertex).data.t = t_;
   dcel_.vertex(newVertex).data.bisector = cg::point_2 ();
   dcel_.vertex(newVertex).data.parentVertices.clear();
   // dcel_.vertex(newVertex).data.parentVertices.insert(stopVertex);
   dcel_.vertex(newVertex).incidentEdge = nextEdge;
   dcel_.vertex(newVertex).data.iter = queue_.end();

   // Connect result vertex
   size_t prevEdgeTwin = dcel_.edge(prevEdge).twinEdge;
   size_t nextEdgeTwin = dcel_.edge(nextEdge).twinEdge;
   dcel_.edge(nextEdgeTwin).vertexDestination = newVertex;
   dcel_.edge(nextEdge).vertexOrigin = newVertex;
   dcel_.edge(prevEdge).vertexDestination = newVertex;
   dcel_.edge(prevEdgeTwin).vertexOrigin = newVertex;

   // Obtain connection edges
   size_t leftAttachment = dcel_.edge(prevEdgeTwin).prevEdge;
   size_t rightAttachment = dcel_.edge(nextEdgeTwin).nextEdge;

   // Create connection wire
   size_t leftConnector =
      dcel_.addEdge(DCEL::Edge (stopVertex, newVertex, leftAttachment, prevEdgeTwin, static_cast< size_t >( -1 ), true));
   dcel_.edge(prevEdgeTwin).prevEdge = leftConnector;

   size_t rightConnector =
      dcel_.addEdge(DCEL::Edge (newVertex, stopVertex, nextEdgeTwin, rightAttachment, leftConnector, true));
   dcel_.edge(nextEdgeTwin).nextEdge = rightConnector;

   dcel_.edge(leftConnector).twinEdge = rightConnector;

   if (dcel_.vertex(stopVertex).incidentEdge == nextEdge)
      dcel_.vertex(stopVertex).incidentEdge = leftConnector;

   // Update incidence info
   for (VertexList::const_iterator it = incVertices.begin(); it != incVertices.end(); ++it)
      assureNextEdgeIdSet(*it, stopVertex, leftConnector);
   dcel_.vertex(stopVertex).data.parentVertices.insert(stopParents.begin(), stopParents.end());

   if (leftAttachment == nextEdgeTwin)
   {
      dcel_.edge(leftConnector).prevEdge = rightConnector;
      dcel_.edge(rightConnector).nextEdge = leftConnector;
   }
   else
   {
      dcel_.edge(leftAttachment).nextEdge = leftConnector;
      dcel_.edge(rightAttachment).prevEdge = rightConnector;
   }

   //
   // 5. Set vertex movement and check for events
   //

   // Calculate bisector for result vertex
   actualizeVertexState(newVertex);
   dcel_.vertex(newVertex).data.bisector =
      calculateBisector(vertexPos(dcel_.edge(prevEdge).vertexOrigin, localCenter), ed_.intersection - localCenter,
      vertexPos(dcel_.edge(nextEdge).vertexDestination, localCenter), true, true, true);

   //double testDist0 = cg::distance(dcel_.vertex(newVertex).pos - localCenter + 200 * dcel_.vertex(newVertex).data.bisector,
   //   getEdgeLine(prevEdge, localCenter));

   //double checkAngleSin = cg::normalized(dcel_.vertex(newVertex).data.bisector - dcel_.vertex(dcel_.edge(prevEdge).vertexOrigin).data.bisector) ^
   //   getEdgeLine(prevEdge, localCenter).r();
   //if (!cg::eq_zero(checkAngleSin, 1e-6))
   //{
   //   maintainConsistency(localCenter, prevEdge, dcel_.edge(prevEdge).vertexOrigin, edgesToUpdate, verticesToUpdate);

   //   dcel_.vertex(newVertex).data.bisector =
   //      calculateBisector(vertexPos(dcel_.edge(prevEdge).vertexOrigin, localCenter), ed_.intersection - localCenter,
   //      vertexPos(dcel_.edge(nextEdge).vertexDestination, localCenter), true, true, true);
   //}
   //checkAngleSin = cg::normalized(dcel_.vertex(newVertex).data.bisector - dcel_.vertex(dcel_.edge(prevEdge).vertexOrigin).data.bisector) ^
   //   getEdgeLine(prevEdge, localCenter).r();
   //size_t prevV = dcel_.edge(prevEdge).vertexOrigin;
   //double testDist = cg::distance(dcel_.vertex(prevV).pos - localCenter + 200 * dcel_.vertex(prevV).data.bisector,
   //   getEdgeLine(prevEdge, localCenter));
   //double testDist2 = cg::distance(dcel_.vertex(newVertex).pos - localCenter + 200 * dcel_.vertex(newVertex).data.bisector,
   //   getEdgeLine(prevEdge, localCenter));

   maintainConsistency(localCenter, newVertex, prevEdge, nextEdge, edgesToUpdate, verticesToUpdate);

   if (!cg::eq_zero(dcel_.vertex(newVertex).data.bisector ^ edgeVector(nextEdge, localCenter)) &&
       !cg::eq_zero(dcel_.vertex(newVertex).data.bisector ^ edgeVector(prevEdge, localCenter)))
   {
      if (attr_.optimize)
         grid_->addVertex(newVertex, maxShiftEstimation_);

      // Check for new events
      verticesToUpdate.push_back(ss::VertexListElem (newVertex, true, false));

      if (cg::robust_orientation(dcel_.vertex(newVertex).data.bisector, prevEdgeBisector) == cg::VO_LEFT)
         prevEdgeElem.type = ss::EUT_CHANGED_ENLARGED;
      if (cg::robust_orientation(nextEdgeBisector, dcel_.vertex(newVertex).data.bisector) == cg::VO_LEFT)
         nextEdgeElem.type = ss::EUT_CHANGED_ENLARGED;

#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << "full processing";
#endif /* SS_DEBUG_LOG */
   }
   else
   {
      prevEdgeElem.type = nextEdgeElem.type = ss::EUT_REMOVED;
   }

   edgesToUpdate.push_back(prevEdgeElem);
   edgesToUpdate.push_back(nextEdgeElem);

   if (attr_.optimize)
      update_queue_optimized(edgesToUpdate, verticesToUpdate);
   else
      update_queue(edgesToUpdate, verticesToUpdate);

#ifdef SS_DEBUG_LOG
   if (logCurEvent)
      *debugLog_ << std::endl;
#endif /* SS_DEBUG_LOG */
}

} // End of 'cg' namespace
