#pragma once

namespace cg
{

SKELETON_EXT_METHOD(bool) collapse_split( cg::point_2 const &localCenter,
                                          size_t startEdge, size_t endEdge,
                                          bool &specialHandling )
{
   cg::point_2 startDir = edgeVector(startEdge, localCenter);
   cg::point_2 endDir = edgeVector(endEdge, localCenter);

   specialHandling = false;

   if (!cg::eq_zero(startDir ^ endDir))
      return false;

   if (!cg::le(startDir * endDir, 0))
      return false;

   if (dcel_.edge(startEdge).nextEdge != endEdge)
   {
      specialHandling = true;
      return false;
   }

   size_t startEdgeTwin = dcel_.edge(startEdge).twinEdge;
   size_t endEdgeTwin = dcel_.edge(endEdge).twinEdge;

   dcel_.edge(startEdgeTwin).twinEdge = endEdgeTwin;
   dcel_.edge(endEdgeTwin).twinEdge = startEdgeTwin;

   dcel_.vertex(dcel_.edge(startEdge).vertexOrigin).incidentEdge = endEdgeTwin;

   return true;
}

SKELETON_EXT_METHOD(bool)
#ifndef SS_DEBUG_LOG
   handle_split_zero_edge( cg::point_2 const &localCenter )
#else /* SS_DEBUG_LOG */
   handle_split_zero_edge( cg::point_2 const &localCenter, bool logCurEvent )
#endif /* SS_DEBUG_LOG */
{
   size_t stopVertex = ed_.eventVertex;

   size_t nextEdge = dcel_.vertex(ed_.eventVertex).incidentEdge;
   size_t prevEdge = dcel_.edge(nextEdge).prevEdge;

   bool quit = false;

   if (cg::eq(vertexPos(dcel_.edge(prevEdge).vertexOrigin, localCenter), vertexPos(stopVertex, localCenter), 1e-8))
   {
#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << "zero prevEdge";
#endif /* SS_DEBUG_LOG */

      ss::EventData newEvent (ss::ET_EDGE, ed_.t, dcel_.edge(prevEdge).vertexOrigin, prevEdge);
      newEvent.nextVertex = stopVertex;
      newEvent.intersection = ed_.intersection;
      if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
          newEvent.t < dcel_.vertex(newEvent.eventVertex).data.iter->t)
      {
         restoreCurrentEvent();
         addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
      }
      quit = true;
   }

   if (cg::eq(vertexPos(dcel_.edge(nextEdge).vertexDestination, localCenter), vertexPos(stopVertex, localCenter), 1e-8))
   {
#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << "zero nextEdge";
#endif /* SS_DEBUG_LOG */

      ss::EventData newEvent (ss::ET_EDGE, ed_.t, stopVertex, nextEdge);
      newEvent.nextVertex = dcel_.edge(nextEdge).vertexDestination;
      newEvent.intersection = ed_.intersection;
      if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
          newEvent.t < dcel_.vertex(newEvent.eventVertex).data.iter->t)
      {
         restoreCurrentEvent();
         addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
      }
      quit = true;
   }

#ifdef SS_DEBUG_LOG
   if (quit)
   {
      if (logCurEvent)
         *debugLog_ << std::endl;
   }
#endif /* SS_DEBUG_LOG */

   return quit;
}

SKELETON_EXT_METHOD(void) handle_split_event()
{
   ss::EdgeListEx edgesToUpdate;
   ss::VertexList verticesToUpdate;

   cg::point_2 localCenter (dcel_.vertex(ed_.eventVertex).pos);

   cg::segment_2 eventEdgeSeg (edgeSegment(ed_.eventEdge, localCenter));

   // Determine event neighborhood
   size_t twinEventEdge = dcel_.edge(ed_.eventEdge).twinEdge;

   size_t stopVertex = ed_.eventVertex;

   size_t nextEdge = dcel_.vertex(ed_.eventVertex).incidentEdge;
   size_t prevEdge = dcel_.edge(nextEdge).prevEdge;

#ifdef SS_DEBUG_LOG
   bool logCurEvent = false;
   if (verticesOfInterest_.find(ed_.eventVertex) != verticesOfInterest_.end() ||
       verticesOfInterest_.find(dcel_.edge(ed_.eventEdge).vertexOrigin) != verticesOfInterest_.end() ||
       verticesOfInterest_.find(dcel_.edge(ed_.eventEdge).vertexDestination) != verticesOfInterest_.end())
   {
      logCurEvent = true;
   }

   if (logCurEvent)
      *debugLog_ << step_ << " t=" << t_ << " ET_SPLIT: vertex: " << ed_.eventVertex <<
         " edge (" << dcel_.edge(ed_.eventEdge).vertexOrigin << ", " << dcel_.edge(ed_.eventEdge).vertexDestination << ") ";
#endif /* SS_DEBUG_LOG */

   if (cg::eq(eventEdgeSeg.P0(), eventEdgeSeg.P1()))
   {
      ss::EventData newEvent (ss::ET_EDGE, ed_.t, dcel_.edge(ed_.eventEdge).vertexOrigin, ed_.eventEdge);
      newEvent.nextVertex = dcel_.edge(ed_.eventEdge).vertexDestination;
      newEvent.intersection = ed_.intersection;
      restoreCurrentEvent();
      addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);

#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << "switch to EDGE event" << std::endl;
#endif /* SS_DEBUG_LOG */
      return;
   }

#ifndef SS_DEBUG_LOG
   if (handle_split_zero_edge(localCenter))
#else /* SS_DEBUG_LOG */
   if (handle_split_zero_edge(localCenter, logCurEvent))
#endif /* SS_DEBUG_LOG */
      return;

   bool leftPartSpecial = false;
   size_t nextEventEdge = dcel_.edge(ed_.eventEdge).nextEdge;
   size_t eventDestinationVertex = dcel_.edge(ed_.eventEdge).vertexDestination;
   ss::EdgeListElem *nextEdgeEventElemP = NULL;
   double tstop = eventEdgeSeg(vertexPos(stopVertex, localCenter));
   if (cg::eq(vertexPos(stopVertex, localCenter), vertexPos(eventDestinationVertex, localCenter), 1e-7) ||
       cg::ge(tstop, 1))
   {
      if (nextEventEdge == prevEdge)
      {
         ss::EventData newEvent (ss::ET_EDGE, ed_.t, eventDestinationVertex, nextEventEdge);
         newEvent.nextVertex = ed_.eventVertex;
         newEvent.intersection = ed_.intersection;
         if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
             newEvent.t < dcel_.vertex(newEvent.eventVertex).data.iter->t)
         {
            restoreCurrentEvent();
            addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
         }
#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << "degenaracy on left" << std::endl;
#endif /* SS_DEBUG_LOG */
         return;
      }

#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << "leftPartSpecial ";
#endif /* SS_DEBUG_LOG */

      leftPartSpecial = true;
      ss::EdgeListElem nextEventEdgeElem (nextEventEdge, ss::EUT_CHANGED);
      if (attr_.optimize)
         nextEventEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, nextEventEdge, nextEventEdgeElem.beamT);
      edgesToUpdate.push_back(nextEventEdgeElem);
      nextEdgeEventElemP = &edgesToUpdate.back();
   }

   bool rightPartSpecial = false;
   size_t prevEventEdge = dcel_.edge(ed_.eventEdge).prevEdge;
   size_t eventOriginVertex = dcel_.edge(ed_.eventEdge).vertexOrigin;
   ss::EdgeListElem *prevEdgeEventElemP = NULL;
   if (cg::eq(vertexPos(stopVertex, localCenter), vertexPos(eventOriginVertex, localCenter), 1e-7) ||
      cg::le(tstop, 0))
   {
      if (leftPartSpecial)
      {
         ss::EventData newEvent (ss::ET_EDGE, ed_.t, eventOriginVertex, ed_.eventEdge);
         newEvent.nextVertex = eventDestinationVertex;
         newEvent.intersection = ed_.intersection;
         if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
             newEvent.t < dcel_.vertex(newEvent.eventVertex).data.iter->t)
         {
            restoreCurrentEvent();
            addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
         }
#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << "degenaracy on right" << std::endl;
#endif /* SS_DEBUG_LOG */
         return;
      }

      if (prevEventEdge == nextEdge)
      {
         ss::EventData newEvent (ss::ET_EDGE, ed_.t, ed_.eventVertex, prevEventEdge);
         newEvent.nextVertex = eventOriginVertex;
         newEvent.intersection = ed_.intersection;
         if (dcel_.vertex(newEvent.eventVertex).data.iter == queue_.end() ||
             newEvent.t < dcel_.vertex(newEvent.eventVertex).data.iter->t)
         {
            restoreCurrentEvent();
            addEventToQueue(newEvent, newEvent.intersection, newEvent.eventVertex);
         }
#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << "degenaracy on right" << std::endl;
#endif /* SS_DEBUG_LOG */
         return;
      }

#ifdef SS_DEBUG_LOG
      if (logCurEvent)
         *debugLog_ << "rightPartSpecial ";
#endif /* SS_DEBUG_LOG */

      rightPartSpecial = true;
      ss::EdgeListElem prevEventEdgeElem (prevEventEdge, ss::EUT_CHANGED);
      if (attr_.optimize)
         prevEventEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, prevEventEdge, prevEventEdgeElem.beamT);
      edgesToUpdate.push_back(prevEventEdgeElem);
      prevEdgeEventElemP = &edgesToUpdate.back();
   }

   ss::EdgeListElem eventEdgeElem (ed_.eventEdge, ss::EUT_SPLIT);
   ss::EdgeListElem prevEdgeElem (prevEdge, ss::EUT_CHANGED);
   ss::EdgeListElem nextEdgeElem (nextEdge, ss::EUT_CHANGED);

   // Process splitting
   if (attr_.optimize)
   {
      eventEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, ed_.eventEdge, eventEdgeElem.beamT);
      prevEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, prevEdge, prevEdgeElem.beamT);
      nextEdgeElem.initialBeam = cg::skeleton::edge2beam_prev(*this, nextEdge, nextEdgeElem.beamT);
   }

   if (attr_.optimize)
      grid_->removeVertex(stopVertex, maxShiftEstimation_);
   stopVertexUpdate(stopVertex);
   dcel_.vertex(stopVertex).pos = ed_.intersection;

   size_t leftVertex = dcel_.addVertex(dcel_.vertex(stopVertex));
   dcel_.vertex(leftVertex).data.parentVertices.clear();
   // dcel_.vertex(leftVertex).data.parentVertices.insert(stopVertex);
   dcel_.vertex(leftVertex).data.processed = false;
   dcel_.vertex(leftVertex).data.t = t_;
   dcel_.vertex(leftVertex).data.bisector = cg::point_2 ();
   dcel_.vertex(leftVertex).data.iter = queue_.end();

   if (leftPartSpecial)
   {
      if (attr_.optimize)
         grid_->removeVertex(eventDestinationVertex, maxShiftEstimation_);
      stopVertexUpdate(eventDestinationVertex);
      cg::dcel::substituteVertex(dcel_, eventDestinationVertex, leftVertex);
   }

   size_t rightVertex = dcel_.addVertex(dcel_.vertex(stopVertex));
   dcel_.vertex(rightVertex).data.parentVertices.clear();
   // dcel_.vertex(rightVertex).data.parentVertices.insert(stopVertex);
   dcel_.vertex(rightVertex).data.processed = false;
   dcel_.vertex(rightVertex).data.t = t_;
   dcel_.vertex(rightVertex).data.bisector = cg::point_2 ();
   dcel_.vertex(rightVertex).data.iter = queue_.end();

#ifdef SS_DEBUG_LOG
   if (logCurEvent)
   {
      *debugLog_ << "leftVertex: " << leftVertex << " rightVertex: " << rightVertex << " ";
      verticesOfInterest_.insert(leftVertex);
      verticesOfInterest_.insert(rightVertex);
   }
#endif /* SS_DEBUG_LOG */

   if (rightPartSpecial)
   {
      if (attr_.optimize)
         grid_->removeVertex(eventOriginVertex, maxShiftEstimation_);
      stopVertexUpdate(eventOriginVertex);
      cg::dcel::substituteVertex(dcel_, eventOriginVertex, rightVertex);
   }

   size_t nextEdgeTwin = dcel_.edge(nextEdge).twinEdge;
   size_t prevEdgeTwin = dcel_.edge(prevEdge).twinEdge;

   size_t leftConnector = dcel_.edge(prevEdgeTwin).prevEdge;
   size_t rightConnector = dcel_.edge(nextEdgeTwin).nextEdge;

   size_t leftPart, leftPartTwin;
   if (!leftPartSpecial)
   {
      leftPart = ed_.eventEdge, leftPartTwin = twinEventEdge;
   }
   else
   {
      leftPart = nextEventEdge;
      leftPartTwin = dcel_.edge(twinEventEdge).prevEdge;
   }

   // Create double wire and second part of splitted edge
   size_t leftWire = dcel_.addEdge(DCEL::Edge (stopVertex, leftVertex, static_cast< size_t >( -1 ), leftPart, static_cast< size_t >( -1 ), true));
   size_t twinLeftWire = dcel_.addEdge(DCEL::Edge (leftVertex, stopVertex, leftPartTwin, static_cast< size_t >( -1 ), leftWire, true));
   dcel_.edge(leftWire).twinEdge = twinLeftWire;

   size_t rightWire = dcel_.addEdge(DCEL::Edge (rightVertex, stopVertex, static_cast< size_t >( -1 ), leftWire, static_cast< size_t >( -1 ), true));
   size_t twinRightWire = dcel_.addEdge(DCEL::Edge (stopVertex, rightVertex, twinLeftWire, static_cast< size_t >( -1 ), rightWire, true));
   dcel_.edge(rightWire).twinEdge = twinRightWire;

   dcel_.edge(leftWire).prevEdge = rightWire;
   dcel_.edge(twinLeftWire).nextEdge = twinRightWire;

   size_t rightPart, rightPartTwin;
   if (!leftPartSpecial && !rightPartSpecial)
   {
      rightPart = dcel_.addEdge(DCEL::Edge (dcel_.edge(ed_.eventEdge).vertexOrigin,
         rightVertex, dcel_.edge(ed_.eventEdge).prevEdge, rightWire, static_cast< size_t >( -1 ), false));
      rightPartTwin = dcel_.addEdge(DCEL::Edge (rightVertex,
         dcel_.edge(ed_.eventEdge).vertexOrigin, twinRightWire,
         dcel_.edge(twinEventEdge).nextEdge, rightPart, true));
      dcel_.edge(rightPart).twinEdge = rightPartTwin;
   }
   else if (!leftPartSpecial)
   {
      rightPart = prevEventEdge;
      rightPartTwin = dcel_.edge(twinEventEdge).nextEdge;
   }
   else
      rightPart = ed_.eventEdge, rightPartTwin = twinEventEdge;

   if (leftPartSpecial || rightPartSpecial)
   {
      dcel_.edge(rightPart).vertexDestination = rightVertex;
      dcel_.edge(rightPart).nextEdge = rightWire;
      dcel_.edge(rightPartTwin).vertexOrigin = rightVertex;
      dcel_.edge(rightPartTwin).prevEdge = twinRightWire;
   }

   dcel_.edge(twinRightWire).nextEdge = rightPartTwin;
   dcel_.edge(rightWire).prevEdge = rightPart;

   // Connect right part
   if (!rightPartSpecial)
   {
      dcel_.edge(prevEventEdge).nextEdge = rightPart;
      dcel_.edge(dcel_.edge(twinEventEdge).nextEdge).prevEdge = rightPartTwin;
   }
   dcel_.edge(rightPart).nextEdge = nextEdge;
   dcel_.edge(nextEdge).prevEdge = rightPart;
   dcel_.edge(nextEdgeTwin).nextEdge = rightWire;
   dcel_.edge(rightWire).prevEdge = nextEdgeTwin;
   dcel_.vertex(rightVertex).incidentEdge = nextEdge;

   dcel_.edge(nextEdge).vertexOrigin = rightVertex;
   dcel_.edge(nextEdgeTwin).vertexDestination = rightVertex;

   dcel_.vertex(eventOriginVertex).incidentEdge = rightPart;

   // Connect left part
   dcel_.edge(leftPart).vertexOrigin = leftVertex;
   dcel_.edge(leftPartTwin).vertexDestination = leftVertex;

   dcel_.edge(leftPart).prevEdge = prevEdge;
   dcel_.edge(prevEdge).nextEdge = leftPart;
   dcel_.edge(prevEdgeTwin).prevEdge = leftWire;
   dcel_.edge(leftWire).nextEdge = prevEdgeTwin;

   dcel_.vertex(leftVertex).incidentEdge = leftPart;
   dcel_.edge(leftPartTwin).nextEdge = twinLeftWire;

   dcel_.edge(prevEdge).vertexDestination = leftVertex;
   dcel_.edge(prevEdgeTwin).vertexOrigin = leftVertex;

   dcel_.vertex(stopVertex).incidentEdge = leftWire;
   updateCorrespondenceData(stopVertex, stopVertex, leftWire);

   if (leftConnector != nextEdgeTwin)
   {
      // Connect outer cycles
      dcel_.edge(leftConnector).nextEdge = leftWire;
      dcel_.edge(leftWire).prevEdge = leftConnector;
      dcel_.edge(rightConnector).prevEdge = rightWire;
      dcel_.edge(rightWire).nextEdge = rightConnector;
   }

   // Connect parts to stop vertex
   if (leftPartSpecial)
   {
      DCEL::exiting_edge_iterator eIt = util::prev(dcel_.exitingEdgeBegin(leftPart));
      if (eIt.index() != twinLeftWire)
      {
         dcel_.edge(dcel_.edge(leftPart).twinEdge).nextEdge = twinLeftWire;
         dcel_.edge(twinLeftWire).prevEdge = dcel_.edge(leftPart).twinEdge;

         eIt->prevEdge = twinLeftWire;
         dcel_.edge(twinLeftWire).nextEdge = eIt.index();

         DCEL::exiting_edge_iterator prevEIt = eIt;
         for (; eIt.index() != twinLeftWire; --eIt)
         {
            eIt->vertexOrigin = stopVertex;
            dcel_.edge(eIt->twinEdge).vertexDestination = stopVertex;
            prevEIt = eIt;
         }

         dcel_.edge(twinRightWire).prevEdge = prevEIt->twinEdge;
         dcel_.edge(prevEIt->twinEdge).nextEdge = twinRightWire;
      }
   }
   if (rightPartSpecial)
   {
      DCEL::entering_edge_iterator eIt = util::next(dcel_.enteringEdgeBegin(rightPart));
      if (eIt.index() != twinRightWire)
      {
         dcel_.edge(dcel_.edge(rightPart).twinEdge).prevEdge = twinRightWire;
         dcel_.edge(twinRightWire).nextEdge = dcel_.edge(rightPart).twinEdge;

         eIt->nextEdge = twinRightWire;
         dcel_.edge(twinRightWire).prevEdge = eIt.index();

         DCEL::entering_edge_iterator prevEIt = eIt;
         for (; eIt.index() != twinRightWire; ++eIt)
         {
            eIt->vertexDestination = stopVertex;
            dcel_.edge(eIt->twinEdge).vertexOrigin = stopVertex;
            prevEIt = eIt;
         }

         dcel_.edge(twinLeftWire).nextEdge = prevEIt->twinEdge;
         dcel_.edge(prevEIt->twinEdge).prevEdge = twinLeftWire;
      }
   }

   // Calculate bisectors
   if (leftPartSpecial)
   {
      if (!queue_.empty())
      {
         ss::EventData nextEd = queue_.top();
         if (nextEd.eventVertex == eventDestinationVertex)
         {
            if (nextEd.type == ss::ET_SPLIT && cg::eq(nextEd.t, t_, 1e-8))
            {
               dcel_.vertex(nextEd.eventVertex).data.iter = queue_.end();
               queue_.pop();
            }
         }
      }
      updateCorrespondenceData(eventDestinationVertex, stopVertex, twinRightWire);
   }
   bool specialHandlingLeft;
   bool collapseLeft = collapse_split(localCenter, leftPart, prevEdge, specialHandlingLeft);

#ifdef SS_DEBUG_LOG
   if (logCurEvent)
   {
      if (collapseLeft)
         *debugLog_ << "collapseLeft ";
      if (specialHandlingLeft)
         *debugLog_ << "specialHandlingLeft ";
   }
#endif /* SS_DEBUG_LOG */

   if (rightPartSpecial)
   {
      if (!queue_.empty())
      {
         ss::EventData nextEd = queue_.top();
         if (nextEd.eventVertex == eventOriginVertex)
         {
            if (nextEd.type == ss::ET_SPLIT && cg::eq(nextEd.t, t_, 1e-8))
            {
               dcel_.vertex(nextEd.eventVertex).data.iter = queue_.end();
               queue_.pop();
            }
         }
      }
      updateCorrespondenceData(eventOriginVertex, stopVertex, twinRightWire);
   }
   bool specialHandlingRight;
   bool collapseRight = collapse_split(localCenter, nextEdge, rightPart, specialHandlingRight);

#ifdef SS_DEBUG_LOG
   if (logCurEvent)
   {
      if (collapseRight)
         *debugLog_ << "collapseRight ";
      if (specialHandlingRight)
         *debugLog_ << "specialHandlingRight ";
   }
#endif /* SS_DEBUG_LOG */

   // Check for new events
   bool processLeft = false;
   if (!collapseLeft)
   {
      actualizeVertexState(leftVertex);
      dcel_.vertex(leftVertex).data.bisector =
         calculateBisector(vertexPos(dcel_.edge(prevEdge).vertexOrigin, localCenter),
         vertexPos(leftVertex, localCenter),
         vertexPos(dcel_.edge(leftPart).vertexDestination, localCenter), true, !leftPartSpecial);

      processLeft = /*!specialHandlingLeft && */!cg::eq_zero(dcel_.vertex(leftVertex).data.bisector ^ edgeVector(prevEdge, localCenter)) &&
         !cg::eq_zero(dcel_.vertex(leftVertex).data.bisector ^ edgeVector(leftPart, localCenter));

      if (!specialHandlingLeft && processLeft)
         maintainConsistency(localCenter, leftVertex, prevEdge, leftPart, edgesToUpdate, verticesToUpdate);
   }

   std::list< ss::EventData > deferredEvents;

   ss::VertexListElem leftVertexElem (leftVertex, false, !leftPartSpecial);
   leftVertexElem.splitProhibitions.push_back(rightPart);
   leftVertexElem.splitProhibitions.push_back(nextEdge);
   bool definiteSplitEventLeft = false;
   bool leftEnlarged = false;
   if (!collapseLeft)
   {
      if (attr_.optimize)
         grid_->addVertex(leftVertex, maxShiftEstimation_);

      if (processLeft)
      {
         leftVertexElem.process = true;
         leftEnlarged = cg::robust_orientation(dcel_.vertex(eventOriginVertex).data.bisector,
            dcel_.vertex(leftVertex).data.bisector) != cg::VO_RIGHT;
      }
      else
      {
         if (attr_.optimize)
            grid_->removeVertex(leftVertex, maxShiftEstimation_);
         stopVertexUpdate(leftVertex);
         dcel_.vertex(leftVertex).data.processed = false;
         definiteSplitEventLeft = true;
         size_t evVert = addDefiniteSplitEvent(dcel_.edge(prevEdge).vertexOrigin,
            leftVertex, dcel_.edge(leftPart).vertexDestination, prevEdge, leftPart, deferredEvents);
         verticesToUpdate.push_back(ss::VertexListElem (evVert, false, true));
         prevEdgeElem.type = ss::EUT_REMOVED;

#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << "definiteSplit on left with " << evVert << " ";
#endif /* SS_DEBUG_LOG */
      }
   }
   else
   {
      prevEdgeElem.type = ss::EUT_REMOVED;

      if (attr_.optimize)
      {
         grid_->removeVertex(leftVertex, maxShiftEstimation_);
         grid_->removeVertex(dcel_.edge(leftPart).vertexDestination, maxShiftEstimation_);
      }

      stopVertexUpdate(leftVertex);
      stopVertexUpdate(dcel_.edge(leftPart).vertexDestination);
      dcel_.vertex(dcel_.edge(leftPart).vertexDestination).pos = dcel_.vertex(leftVertex).pos;
   }

   bool processRight = false;
   if (!collapseRight)
   {
      actualizeVertexState(rightVertex);
      dcel_.vertex(rightVertex).data.bisector =
         calculateBisector(vertexPos(dcel_.edge(rightPart).vertexOrigin, localCenter),
         vertexPos(rightVertex, localCenter),
         vertexPos(dcel_.edge(nextEdge).vertexDestination, localCenter), true, !rightPartSpecial);

      processRight = /*!specialHandlingRight && */!cg::eq_zero(dcel_.vertex(rightVertex).data.bisector ^ edgeVector(nextEdge, localCenter)) &&
         !cg::eq_zero(dcel_.vertex(rightVertex).data.bisector ^ edgeVector(rightPart, localCenter));

      if (!specialHandlingRight && processRight)
         maintainConsistency(localCenter, rightVertex, rightPart, nextEdge, edgesToUpdate, verticesToUpdate);
   }
   edgesToUpdate.push_back(prevEdgeElem);
   verticesToUpdate.push_back(leftVertexElem);

   ss::VertexListElem rightVertexElem (rightVertex, false, !rightPartSpecial);
   rightVertexElem.splitProhibitions.push_back(prevEdge);
   rightVertexElem.splitProhibitions.push_back(leftPart);
   bool definiteSplitEventRight = false;
   bool rightEnlarged = false;
   if (!collapseRight)
   {
      if (attr_.optimize)
         grid_->addVertex(rightVertex, maxShiftEstimation_);

      if (processRight)
      {
         rightVertexElem.process = true;
         rightEnlarged = cg::robust_orientation(dcel_.vertex(eventDestinationVertex).data.bisector,
            dcel_.vertex(rightVertex).data.bisector) != cg::VO_LEFT;
      }
      else
      {
         if (attr_.optimize)
            grid_->removeVertex(rightVertex, maxShiftEstimation_);
         stopVertexUpdate(rightVertex);
         dcel_.vertex(rightVertex).data.processed = false;
         definiteSplitEventRight = true;
         size_t evVert = addDefiniteSplitEvent(dcel_.edge(rightPart).vertexOrigin,
            rightVertex, dcel_.edge(nextEdge).vertexDestination, rightPart, nextEdge, deferredEvents);
         verticesToUpdate.push_back(ss::VertexListElem (evVert, false, true));
         nextEdgeElem.type = ss::EUT_REMOVED;

#ifdef SS_DEBUG_LOG
         if (logCurEvent)
            *debugLog_ << "definiteSplit on right with " << evVert << " ";
#endif /* SS_DEBUG_LOG */
      }
   }
   else
   {
      nextEdgeElem.type = ss::EUT_REMOVED;

      if (attr_.optimize)
      {
         grid_->removeVertex(rightVertex, maxShiftEstimation_);
         grid_->removeVertex(dcel_.edge(rightPart).vertexOrigin, maxShiftEstimation_);
      }

      stopVertexUpdate(rightVertex);
      stopVertexUpdate(dcel_.edge(rightPart).vertexOrigin);
      dcel_.vertex(dcel_.edge(rightPart).vertexOrigin).pos = dcel_.vertex(rightVertex).pos;
   }
   edgesToUpdate.push_back(nextEdgeElem);
   verticesToUpdate.push_back(rightVertexElem);

   if (!collapseRight || !collapseLeft)
   {
      eventEdgeElem.splitRes = std::make_pair(leftPart, rightPart);
      eventEdgeElem.splitProc = std::make_pair(!collapseLeft && !definiteSplitEventLeft, !collapseRight && !definiteSplitEventRight);
      eventEdgeElem.splitEnlarged = std::make_pair(leftEnlarged, rightEnlarged);
      edgesToUpdate.push_back(eventEdgeElem);
   }
   if (collapseLeft)
   {
      if (nextEdgeEventElemP != NULL)
         nextEdgeEventElemP->type = ss::EUT_REMOVED;
   }
   if (collapseRight)
   {
      if (prevEdgeEventElemP != NULL)
         prevEdgeEventElemP->type = ss::EUT_REMOVED;
   }

   if (attr_.optimize)
      update_queue_optimized(edgesToUpdate, verticesToUpdate);
   else
      update_queue(edgesToUpdate, verticesToUpdate);

   for (std::list< ss::EventData >::iterator lIt = deferredEvents.begin(); lIt != deferredEvents.end(); ++lIt)
      addEventToQueue(*lIt, lIt->intersection, lIt->eventHost);

   if (collapseLeft)
   {
      dcel_.deleteHalfEdge(leftPart);
      dcel_.deleteHalfEdge(prevEdge);
   }
   if (collapseRight)
   {
      dcel_.deleteHalfEdge(rightPart);
      dcel_.deleteHalfEdge(nextEdge);
   }

#ifdef SS_DEBUG_LOG
   if (logCurEvent)
      *debugLog_ << std::endl;
#endif /* SS_DEBUG_LOG */
}

} // End of 'cg' namespace
