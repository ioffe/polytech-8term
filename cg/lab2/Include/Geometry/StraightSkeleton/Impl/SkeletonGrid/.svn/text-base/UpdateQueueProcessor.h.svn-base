#pragma once

namespace cg
{
namespace skeleton
{

template< class GridType, class SkeletonType >
struct AddUpdateQueueProcessor
   : cg::grid2l_visitor_base< GridType, AddUpdateQueueProcessor< GridType, SkeletonType > >
{
   AddUpdateQueueProcessor ( SkeletonType &skeleton, ss::EventQueue &queue,
                             size_t edgeIdx, bool checkForNewEvents,
                             std::set< size_t > const &handledVertices,
                             ss::EdgeListEx const &edges, ss::VertexList const &vertices )
      : skeleton_ (skeleton)
      , queue_ (queue)
      , edgeIdx_ (edgeIdx)
      , checkForNewEvents_ (checkForNewEvents)
      , handledVertices_ (handledVertices)
      , edges_ (edges)
      , vertices_ (vertices)
   {
   }

   template< class State, class CellType >
      bool operator () ( State const &, CellType &cell )
   {
      typedef typename CellType::VerticesContainer::iterator vertex_iterator;
      if (checkForNewEvents_)
      {
         for (vertex_iterator vIt = cell.vertices.begin(); vIt != cell.vertices.end(); ++vIt)
         {
            if (vertexEdgeAdjacency(skeleton_.dcel(), *vIt, edgeIdx_))
               continue;

            if (handledVertices_.find(*vIt) != handledVertices_.end())
               continue;

            //for (ss::VertexList::const_iterator lIt = vertices_.begin(); lIt != vertices_.end(); ++lIt)
            //{
            //   if (lIt->vertex == *vIt)
            //      break;
            //}
            //if (lIt != vertices_.end())
            //   continue;

            //if (skeleton_.dcel().vertex(*vIt).data.iter != queue_.end())
            //{
            //   for (ss::EdgeListEx::const_iterator eIt = edges_.begin(); eIt != edges_.end(); ++eIt)
            //   {
            //      if (eIt->edge == skeleton_.dcel().vertex(*vIt).data.iter->eventEdge)
            //         break;
            //   }
            //   if (eIt != edges_.end())
            //      continue;
            //}

            ss::EventData newEvent;
            if (skeleton_.getSplitEvent(*vIt, edgeIdx_, newEvent))
            {
               if ((skeleton_.dcel().vertex(*vIt).data.iter != queue_.end() && newEvent.t < skeleton_.dcel().vertex(*vIt).data.iter->t) ||
                  (skeleton_.dcel().vertex(*vIt).data.iter == queue_.end() && newEvent.t < std::numeric_limits< double >::max()))
               {
                  cg::point_2 const &localCenter (skeleton_.dcel().vertex(*vIt).pos);
                  if (cg::eq(skeleton_.vertexPos(*vIt, localCenter), skeleton_.vertexPos(skeleton_.dcel().edge(newEvent.eventEdge).vertexOrigin, localCenter)) ||
                     cg::eq(skeleton_.vertexPos(*vIt, localCenter), skeleton_.vertexPos(skeleton_.dcel().edge(newEvent.eventEdge).vertexDestination, localCenter)))
                  {
                     continue;
                  }

                  skeleton_.addEventToQueue(newEvent, newEvent.intersection, *vIt);
               }
            }
         }
      }
      cell.edges.insert(edgeIdx_);
      return false;
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   SkeletonType &skeleton_;
   ss::EventQueue &queue_;
   size_t edgeIdx_;
   bool checkForNewEvents_;
   std::set< size_t > const &handledVertices_;
   ss::EdgeListEx const &edges_;
   ss::VertexList const &vertices_;

   SideProcessor dummy_;
};

template< class GridType, class SkeletonType >
struct RemoveUpdateQueueProcessor
   : cg::grid2l_visitor_base< GridType, RemoveUpdateQueueProcessor< GridType, SkeletonType > >
{
   RemoveUpdateQueueProcessor ( SkeletonType &skeleton, ss::EventQueue &queue,
                                ss::EdgeListElem const &elem, double t,
                                std::list< size_t > &verticesToObtainEvents,
                                std::set< size_t > &handledVertices,
                                ss::EdgeListEx const &edges, ss::VertexList const &vertices )
      : skeleton_ (skeleton)
      , queue_ (queue)
      , elem_ (elem)
      , t_ (t)
      , verticesToObtainEvents_ (verticesToObtainEvents)
      , handledVertices_ (handledVertices)
      , edges_ (edges)
      , vertices_ (vertices)
   {
   }

   template< class State, class CellType >
      bool operator () ( State const &, CellType &cell )
   {
      typedef typename CellType::VerticesContainer::iterator vertex_iterator;
      for (vertex_iterator vIt = cell.vertices.begin(); vIt != cell.vertices.end(); ++vIt)
      {
         if (skeleton_.dcel().vertex(*vIt).data.iter != queue_.end())
         {
            ss::VertexList::const_iterator lIt = vertices_.begin();
            for (; lIt != vertices_.end(); ++lIt)
            {
               if (lIt->vertex == *vIt)
                  break;
            }
            if (lIt != vertices_.end())
               continue;

            ss::EventQueue::iterator evIt = skeleton_.dcel().vertex(*vIt).data.iter;
            if (elem_.edge == evIt->eventEdge)
            {
               ss::EventData newEvent;
               bool found = false;
               if (evIt->type == ss::ET_SPLIT && elem_.type != ss::EUT_REMOVED)
               {
                  if (elem_.type == ss::EUT_SPLIT)
                  {
                     if (elem_.splitProc.first && skeleton_.getSplitEventByCheck(*vIt, elem_.splitRes.first, true, evIt->intersection, newEvent) ||
                         elem_.splitProc.second && skeleton_.getSplitEventByCheck(*vIt, elem_.splitRes.second, false, evIt->intersection, newEvent))
                     {
                        found = true;
                     }
                  }
                  else
                  {
                     if (skeleton_.getSplitEvent(*vIt, elem_.edge, newEvent) && newEvent.t != t_)
                        found = true;
                  }
               }

               if (found && newEvent.t < std::numeric_limits< double >::max())
               {
                  skeleton_.addEventToQueue(newEvent, newEvent.intersection, *vIt);
                  handledVertices_.insert(*vIt);
               }

               if (!found)
               {
                  //size_t incEdge = skeleton_.dcel().vertex(*vIt).incidentEdge;
                  //size_t prevEdge = skeleton_.dcel().edge(incEdge).prevEdge;

                  verticesToObtainEvents_.push_back(*vIt);
                  handledVertices_.insert(*vIt);
                  //newEvent = skeleton_.obtainVertexEvent(skeleton_.dcel().edge(prevEdge).vertexOrigin, *vIt,
                  //   skeleton_.dcel().edge(incEdge).vertexDestination);
                  //found = true;
               }
            }
         }
      }
      cell.edges.erase(elem_.edge);
      return false;
   }

   typedef empty_processor SideProcessor;
   SideProcessor & side_processor(int, int) { return dummy_; }

private:
   SkeletonType &skeleton_;
   ss::EventQueue &queue_;
   ss::EdgeListElem const &elem_;
   double t_;
   std::list< size_t > &verticesToObtainEvents_;
   std::set< size_t > &handledVertices_;
   ss::EdgeListEx const &edges_;
   ss::VertexList const &vertices_;

   SideProcessor dummy_;
};

} // End of 'skeleton' namespace
} // End of 'cg' namespace
