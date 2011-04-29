#pragma once

namespace cg
{

SKELETON_EXT_METHOD(void) update_queue_optimized( ss::EdgeListEx const &edges,
                                                  ss::VertexList const &vertices )
{
   // Handle new vertices
   for (ss::VertexList::const_iterator lIt = vertices.begin(); lIt != vertices.end(); ++lIt)
   {
      if (!lIt->process)
         continue;

      DCEL::Vertex &curV = dcel_.vertex(lIt->vertex);

      size_t incEdge = curV.incidentEdge;
      size_t prevEdge = dcel_.edge(incEdge).prevEdge;

      Assert(lIt->splitProhibitions.size() <= 2);

      size_t excludeEdgeA = static_cast< size_t >( -1 ), excludeEdgeB = static_cast< size_t >( -1 );
      ss::EdgeList::const_iterator eIt = lIt->splitProhibitions.begin();
      if (eIt != lIt->splitProhibitions.end())
      {
         excludeEdgeA = *eIt;
         ++eIt;
         if (eIt != lIt->splitProhibitions.end())
            excludeEdgeB = *eIt;
      }

      ss::EventData newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin,
         lIt->vertex, dcel_.edge(incEdge).vertexDestination, 0,
         lIt->convex, false, false, excludeEdgeA, excludeEdgeB);

      addEventToQueue(newEvent, newEvent.intersection, lIt->vertex);
   }

   std::set< size_t > handledVertices;
   std::list< size_t > verticesToObtainEvents;
   ss::EdgeListEx::const_iterator splitIt = edges.end();
   for (ss::EdgeListEx::const_iterator eIt = edges.begin(); eIt != edges.end(); ++eIt)
   {
      if (eIt->type == ss::EUT_SPLIT)
         splitIt = eIt;

      grid_->updateQueueByEdgeRemove(queue_, *eIt, t_,
         verticesToObtainEvents, handledVertices, edges, vertices, maxShiftEstimation_ - (eIt->beamT - iterationStart_));
   }

   for (ss::EdgeListEx::const_iterator eIt = edges.begin(); eIt != edges.end(); ++eIt)
   {
      if (eIt->type == ss::EUT_REMOVED)
         continue;

      if (eIt->type != ss::EUT_SPLIT)
      {
         grid_->updateQueueByEdgeAdd(queue_, eIt->edge, eIt->type == ss::EUT_CHANGED_ENLARGED,
            handledVertices, edges, vertices, maxShiftEstimation_ - (t_ - iterationStart_));
      }
      else
      {
         if (eIt->splitProc.first)
            grid_->updateQueueByEdgeAdd(queue_, eIt->splitRes.first, eIt->splitEnlarged.first || !eIt->splitProc.second, handledVertices, edges, vertices, maxShiftEstimation_ - (t_ - iterationStart_));
         if (eIt->splitProc.second)
            grid_->updateQueueByEdgeAdd(queue_, eIt->splitRes.second, eIt->splitEnlarged.second || !eIt->splitProc.first, handledVertices, edges, vertices, maxShiftEstimation_ - (t_ - iterationStart_));
      }
   }

   for (std::list< size_t >::iterator vIt = verticesToObtainEvents.begin(); vIt != verticesToObtainEvents.end(); ++vIt)
   {
      size_t incEdge = dcel_.vertex(*vIt).incidentEdge;
      size_t prevEdge = dcel_.edge(incEdge).prevEdge;

      ss::EventData newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin,
         *vIt, dcel_.edge(incEdge).vertexDestination);

      //if (splitIt != edges.end())
      //{
      //   if (newEvent.eventEdge == splitIt->splitRes.first || newEvent.eventEdge == splitIt->splitRes.second)
      //   {
      //      // Additional consistency check
      //      if (!(splitIt->splitProc.first && getSplitEventByCheck(*vIt, splitIt->splitRes.first, true, newEvent.intersection, newEvent) ||
      //            splitIt->splitProc.second && getSplitEventByCheck(*vIt, splitIt->splitRes.second, false, newEvent.intersection, newEvent)))
      //      {
      //         continue;
      //      }
      //   }
      //}

      if (newEvent.t == std::numeric_limits< double >::max())
      {
         if (dcel_.vertex(*vIt).data.iter != queue_.end())
         {
            queue_.erase(dcel_.vertex(*vIt).data.iter);
            dcel_.vertex(*vIt).data.iter = queue_.end();
         }
      }

      addEventToQueue(newEvent, newEvent.intersection, *vIt);      
   }
}

SKELETON_EXT_METHOD(void) update_queue( ss::EdgeListEx const &edges, ss::VertexList const &vertices )
{
   for (size_t v = 0; v < dcel_.verticesSize(); ++v)
   {
      DCEL::Vertex &curV = dcel_.vertex(v);
      if (curV.data.processed || dcel_.edge(curV.incidentEdge).hole ||
          curV.data.bisector.x == 0 && curV.data.bisector.y == 0)
      {
         continue;
      }

      size_t incEdge = curV.incidentEdge;
      size_t prevEdge = dcel_.edge(incEdge).prevEdge;

      bool processed = false;

      // Handle new vertices
      for (ss::VertexList::const_iterator lIt = vertices.begin(); lIt != vertices.end(); ++lIt)
      {
         Assert(lIt->splitProhibitions.size() <= 2);

         if (lIt->vertex == v)
         {
            if (lIt->process)
            {               
               size_t excludeEdgeA = static_cast< size_t >( -1 ), excludeEdgeB = static_cast< size_t >( -1 );
               ss::EdgeList::const_iterator eIt = lIt->splitProhibitions.begin();
               if (eIt != lIt->splitProhibitions.end())
               {
                  excludeEdgeA = *eIt;
                  ++eIt;
                  if (eIt != lIt->splitProhibitions.end())
                     excludeEdgeB = *eIt;
               }
                 
               ss::EventData newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin,
                  v, dcel_.edge(incEdge).vertexDestination, 0,
                  lIt->convex, false, false, excludeEdgeA, excludeEdgeB);

               addEventToQueue(newEvent, newEvent.intersection, v);
            }
            processed = true;
            break;
         }
      }
      if (processed)
         continue;

      // Vertex had split event with one of changed edges
      if (curV.data.iter != queue_.end() && curV.data.iter->type == ss::ET_SPLIT)
      {
         for (ss::EdgeListEx::const_iterator eIt = edges.begin(); eIt != edges.end(); ++eIt)
         {
            if (eIt->edge == curV.data.iter->eventEdge)
            {
               cg::point_2 prevEventIntersection = curV.data.iter->intersection;

               if (eIt->type != ss::EUT_REMOVED)
                  Verify(!vertexEdgeAdjacency(dcel_, v, eIt->edge));

               ss::EventData newEvent;
               switch (eIt->type)
               {
               case ss::EUT_REMOVED:
                  {
                     //if (vertexEdgeAdjacency(dcel_, dcel_.edge(prevEdge).vertexOrigin, eIt->edge) ||
                     //    vertexEdgeAdjacency(dcel_, dcel_.edge(incEdge).vertexDestination, eIt->edge))
                     //{
                     //   break;
                     //}
                     newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin, v,
                        dcel_.edge(incEdge).vertexDestination, 0, false, false, false, eIt->edge);
                     break;
                  }

               case ss::EUT_SPLIT:
                  if (!eIt->splitProc.first || !getSplitEventByCheck(v, eIt->splitRes.first, true, prevEventIntersection, newEvent))
                  {
                     if (!eIt->splitProc.second || !getSplitEventByCheck(v, eIt->splitRes.second, false, prevEventIntersection, newEvent))
                     {
                        newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin, v,
                                      dcel_.edge(incEdge).vertexDestination);
                     }
                  }
                  break;

               case ss::EUT_CHANGED:
               case ss::EUT_CHANGED_ENLARGED:
                  if (!(getSplitEvent(v, eIt->edge, newEvent) && newEvent.type == ss::ET_SPLIT && newEvent.t != t_))
                  {
                     newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin, v,
                                   dcel_.edge(incEdge).vertexDestination);

                  }
                  break;

               default:
                  Verify(false);
               }

               addEventToQueue(newEvent, newEvent.intersection, v, true);

               processed = true;
               break;
            }
         }
      }
      if (processed)
         continue;

      // Vertex had edge event with one of changed edges
      if (curV.data.iter != queue_.end() && curV.data.iter->type == ss::ET_EDGE)
      {
         for (ss::EdgeListEx::const_iterator eIt = edges.begin(); eIt != edges.end(); ++eIt)
         {
            if (eIt->edge == curV.data.iter->eventEdge)
            {
               ss::EventData newEvent = obtainVertexEvent(dcel_.edge(prevEdge).vertexOrigin,
                                           v, dcel_.edge(incEdge).vertexDestination);

               addEventToQueue(newEvent, newEvent.intersection, v);
               processed = true;
               break;
            }
         }
      }
      if (processed)
         continue;

      // Other vertices possibly collide with changed edges
      ss::EventData newEvent, checkEvent;
      for (ss::EdgeListEx::const_iterator eIt = edges.begin(); eIt != edges.end(); ++eIt)
      {
         if ((eIt->type != ss::EUT_CHANGED_ENLARGED && eIt->type != ss::EUT_SPLIT) || vertexEdgeAdjacency(dcel_, v, eIt->edge))
            continue;
         
         size_t checkEdge = eIt->edge;
         bool checked = false;
         if (eIt->type != ss::EUT_CHANGED_ENLARGED)
         {
            if (eIt->splitEnlarged.first || !eIt->splitProc.second)
            {
               checked = true;
               checkEdge = eIt->splitRes.first;
               if (getSplitEvent(v, checkEdge, checkEvent))
               {
                   if (checkEvent.t < newEvent.t)
                       newEvent = checkEvent;
               }
            }

            if (eIt->splitEnlarged.second || !eIt->splitProc.first)
            {
               checked = true;
               checkEdge = eIt->splitRes.second;
               if (getSplitEvent(v, checkEdge, checkEvent))
               {
                   if (checkEvent.t < newEvent.t)
                       newEvent = checkEvent;
               }
            }
         }

         if (!checked)
         {
            if (getSplitEvent(v, checkEdge, checkEvent))
            {
                if (checkEvent.t < newEvent.t)
                newEvent = checkEvent;
            }
         }
      }
      if ((curV.data.iter != queue_.end() && newEvent.t < curV.data.iter->t) ||
         (curV.data.iter == queue_.end() && newEvent.t < std::numeric_limits< double >::max()))
      {
         if (newEvent.type == ss::ET_SPLIT)
         {
            cg::point_2 const &localCenter (dcel_.vertex(v).pos);

            if (cg::eq(vertexPos(v, localCenter), vertexPos(dcel_.edge(newEvent.eventEdge).vertexOrigin, localCenter)) ||
               cg::eq(vertexPos(v, localCenter), vertexPos(dcel_.edge(newEvent.eventEdge).vertexDestination, localCenter)))
            {
               continue;
            }
         }

         addEventToQueue(newEvent, newEvent.intersection, v);
      }
   }
}

} // End of 'cg' namespace
