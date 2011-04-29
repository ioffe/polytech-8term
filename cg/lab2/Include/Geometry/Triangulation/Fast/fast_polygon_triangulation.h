#pragma once

#include <stack>

#include "Geometry/dcel/dcel.h"
#include "Geometry/dcel/dcel_algos.h"
#include "Geometry/polygon_2.h"

#include "segment_xless_predicate.h"
#include "monotone_polygon_triangulation.h"
#include "common/indexer.h"

namespace cg
{

///////////////////////////////////////////////////////////////////////////////

template< class VerticesArrayT, class IndexRandomIter, class OutputIterator >
struct PolyFastTriangulator
{
   typedef typename VerticesArrayT::value_type PointT;
   typedef typename PointT::scalar_type scalar_type;

   typedef typename std::iterator_traits<IndexRandomIter>::value_type ContourT;

   enum VertexType
   {
      VT_REGULAR = 0,
      VT_START,
      VT_END,
      VT_SPLIT,
      VT_MERGE
   };

   struct AddVertexData
   {
      AddVertexData ()
         : vertexIdx (static_cast< size_t >( -1 )), type (VT_REGULAR)
      {
      }

      AddVertexData ( size_t vIdx, VertexType t = VT_REGULAR )
         : vertexIdx (vIdx), type (t)
      {
      }

      VertexType type;
      size_t vertexIdx;

      template< class Stream >
      void dump( Stream & out )
      {
         out << "type = " << type << ", id = " << vertexIdx;
      }
   };

   typedef dcel::DCEL< scalar_type, AddVertexData > SimplifiedDCEL;
   typedef typename SimplifiedDCEL::cycle_iterator cycle_iterator;

   // Instead of explicit construction use cg::triangulation::fast()
   PolyFastTriangulator (  VerticesArrayT const & vertices, IndexRandomIter contoursBegin, IndexRandomIter contoursEnd, 
                           OutputIterator triangles, bool step_by_step = false ) :
      vertices_ (vertices),
      out_      (triangles),      
      event_    (0)
   {
      init_dcel( vertices, contoursBegin, contoursEnd );

      cycleIt_ = dcel_.cyclesEnd();

      // Prepare DCEL for sweep line processing
      sort_vertices(dcel_, cg::dcel::VertexYGreater ());

      if ( step_by_step )
         return;

      // Process plane sweeping to construct monotone polygons
      planeSweep();

      // ... and triangulate them
      triangulateMonoPolys();
   }

private:

   void init_dcel(   VerticesArrayT const & vertices, 
                     IndexRandomIter contoursBegin, IndexRandomIter contoursEnd )
   {
      util::indexer< PointT, size_t > vertex_indexer;
      add_vertices_to_dcel( vertices, contoursBegin, contoursEnd, vertex_indexer );
      add_edges_to_dcel( vertices, contoursBegin, contoursEnd, vertex_indexer );
   }

   void add_vertices_to_dcel( VerticesArrayT const & vertices, 
                              IndexRandomIter contoursBegin, IndexRandomIter contoursEnd,
                              util::indexer< PointT, size_t > & vertex_indexer )
   {
      for (IndexRandomIter cIt = contoursBegin; cIt != contoursEnd; ++cIt)
      {
         for (ContourT::const_iterator vIt = cIt->begin(); vIt != cIt->end(); ++vIt)
         {
            PointT const & curPoint = vertices[*vIt];
            size_t old_size = vertex_indexer.size();
            if ( vertex_indexer(curPoint) == old_size )
            {
               size_t v = dcel_.addVertex( SimplifiedDCEL::Vertex(  vertices[*vIt] ) );

               PointT
                  prevPoint = vertices[*(vIt == cIt->begin() ? cIt->end() - 1 : vIt - 1)],
                  nextPoint = vertices[*(vIt == cIt->end() - 1 ? cIt->begin() : vIt + 1)];

               bool concave = robust_left_turn(nextPoint - curPoint, curPoint - prevPoint);

               VertexType type;
               if (prevPoint.y >= curPoint.y && nextPoint.y >= curPoint.y)
               {
                  type = concave ? VT_MERGE : VT_END;
               }
               else if (prevPoint.y <= curPoint.y && nextPoint.y <= curPoint.y)
               {
                  type = concave ? VT_SPLIT : VT_START;
               }
               else
                  type = VT_REGULAR;

               dcel_.vertex( v ).data.vertexIdx = *vIt;
               dcel_.vertex( v ).data.type = type;
            }
         }
      }
   }

   void add_edges_to_dcel( VerticesArrayT const & vertices, 
                           IndexRandomIter contoursBegin, IndexRandomIter contoursEnd,
                           util::indexer< PointT, size_t > & vertex_indexer )
   {
      size_t edges_num = 0;

      for (IndexRandomIter cIt = contoursBegin; cIt != contoursEnd; ++cIt)
      {
         for (ContourT::const_iterator vIt = cIt->begin(); vIt != cIt->end(); ++vIt, edges_num += 2)
         {
            PointT const & curr_pt = vertices[*vIt];
            PointT const & next_pt = vertices[*(vIt == cIt->end() - 1 ? cIt->begin() : vIt + 1)];
            size_t curr = vertex_indexer(curr_pt);
            size_t next = vertex_indexer(next_pt);
            dcel_.addEdgePair( curr, next );

            dcel_.vertex( curr ).incidentEdge = edges_num;
         }
      }

      for ( size_t e = 1; e < dcel_.edgesSize(); e += 2 )
         dcel_.edge( e ).hole = true;
   }

   struct StateEdge
   {
      typedef PointT PointT;
      typedef typename SimplifiedDCEL::Edge EdgeT;

      StateEdge ( EdgeT const &e, PointT const &vOrig, PointT const &vDest ) :
         base (e), vertexOrigin (vOrig), vertexDestination (vDest), helperVertex (-1)
      {
      }

      EdgeT base;

      PointT vertexOrigin;
      PointT vertexDestination;

      int helperVertex;
   };

   StateEdge stateEdge( size_t edgeId )
   {
      return StateEdge (dcel_.edge(edgeId),
         dcel_.vertex(dcel_.edge(edgeId).vertexOrigin).pos,
         dcel_.vertex(dcel_.edge(edgeId).vertexDestination).pos);
   }

   bool isEdgeEnding( StateEdge &edge, size_t event )
   {
      if (event == edge.base.vertexOrigin)
         return edge.base.vertexDestination < edge.base.vertexOrigin;

      return edge.base.vertexOrigin < edge.base.vertexDestination;
   }

   void handleEdgeEvent( StateEdge &edge, size_t event, bool doNotAdd = false )
   {
      if (!isEdgeEnding(edge, event))
      {
         // 1. Edge starts -> insert in current state

         if (doNotAdd)
            return;

         edge.helperVertex = (int)event;
         sweepLineState_.insert(edge);
      }
      else
      {
         // 2. Edge ends

         SweepState::iterator curEdgeIt = sweepLineState_.find(edge);
         if (curEdgeIt != sweepLineState_.end())
         {
            if (curEdgeIt->helperVertex != -1 &&
               dcel_.vertex(curEdgeIt->helperVertex).data.type == VT_MERGE)
            {
               // -> Add diagonal from helper to 'edge' helper
               dcel_.addEdgePair(event, curEdgeIt->helperVertex, true);
            }
            
            // -> remove from current state
            sweepLineState_.erase(curEdgeIt);
         }
      }

   }

   void handleVertexEvent( size_t event )
   {
      StateEdge next = stateEdge(dcel_.vertex(event).incidentEdge);
      StateEdge prev = stateEdge(dcel_.edge(dcel_.vertex(event).incidentEdge).prevEdge);

      typename SimplifiedDCEL::Vertex const &eventVertex = dcel_.vertex(event);

      StateEdge fakeEdge (SimplifiedDCEL::Edge (), eventVertex.pos, eventVertex.pos);
      if (eventVertex.data.type == VT_SPLIT)
      {
         // Find edge closest to current vertex (on the left of it)
         SweepState::iterator rightEdgeIter = sweepLineState_.lower_bound(fakeEdge);
         if (rightEdgeIter != sweepLineState_.begin())
         {
            SweepState::iterator leftEdgeIt = --rightEdgeIter;

            // -> Add diagonal from event vertex to the nearest left edge helper
            Assert(leftEdgeIt->helperVertex != -1);
            if (dcel_.addEdgePair(event, leftEdgeIt->helperVertex, true))
               leftEdgeIt->helperVertex = (int)event;
         }
      }

      if (isEdgeEnding(next, event))
      {
         // Do not add to state in case of 'right' edges
         handleEdgeEvent(next, event, true);
         handleEdgeEvent(prev, event, true);
      }
      else
      {
         // Do not add prev in case of 'split' or 'start' vertex
         handleEdgeEvent(prev, event, true);
         handleEdgeEvent(next, event);
      }
                                                    
      // Check if event vertex is new helper for some state-edge
      if (eventVertex.data.type == VT_MERGE || eventVertex.data.type == VT_REGULAR)
      {
         SweepState::iterator rightEdgeIter = sweepLineState_.lower_bound(fakeEdge);
         if (rightEdgeIter != sweepLineState_.begin())
         {
            SweepState::iterator leftEdgeIt = --rightEdgeIter;

            if (leftEdgeIt->base.vertexOrigin != event && leftEdgeIt->base.vertexDestination != event)
            {
               if (leftEdgeIt->helperVertex != -1)
               {
                  if (dcel_.vertex(leftEdgeIt->helperVertex).data.type == VT_MERGE &&
                     eventVertex.data.type != VT_SPLIT)
                  {
                     // -> Add diagonal from old helper to the new helper
                     if (!dcel_.addEdgePair(leftEdgeIt->helperVertex, event, true))
                        return;
                  }
               }

               leftEdgeIt->helperVertex = (int)event;
            }
         }
      }
   }

public:
   void planeSweep()
   {
      for (size_t event = 0; event < dcel_.verticesSize(); ++event)
      {
         // Process new event
         handleVertexEvent(event);
      }
   }

   bool triangulateMonoPolys()
   {
      for (cycle_iterator cIt = dcel_.cyclesBegin(); cIt != dcel_.cyclesEnd(); ++cIt)
      {
         if (!triangulation::mono_poly(&dcel_, cIt.index(), out_))
            return false;
      }
      return true;
   }

public:
   //
   // For debug purpose only
   //

   bool stepByStep()
   {
      if (cycleIt_ == dcel_.cyclesEnd() && event_ >= dcel_.verticesSize())
         return false;

      if (event_ < dcel_.verticesSize())
      {
         handleVertexEvent(event_);
         event_++;
         if (event_ == dcel_.verticesSize())
            cycleIt_ = dcel_.cyclesBegin();
         return true; 
      }

      //if (event_ < dcel_.verticesSize())
      //{
      //   for (event_ = 0; event_ < dcel_.verticesSize(); ++event_)
      //   {
      //      // Process new event
      //      handleVertexEvent(event_);
      //   }

      //   if (event_ == dcel_.verticesSize())
      //      cycleIt_ = dcel_.cyclesBegin();

      //   return true;
      //}

      if (event_ == dcel_.verticesSize())
      {
         if (cycleIt_ != dcel_.cyclesEnd())
         {
            if (!triangulation::mono_poly(&dcel_, cycleIt_.index(), out_))
               return false;

            ++cycleIt_;
         }
      }

      return event_ != dcel_.verticesSize() || cycleIt_ != dcel_.cyclesEnd();
   }

   typedef std::set< StateEdge, SegmentXLess< StateEdge > > SweepState;

   SweepState const &getState()
   {
      return sweepLineState_;
   }

   SimplifiedDCEL const &getDCEL()
   {
      return dcel_;
   }

   PointT getLastEvent()
   {
      if (event_ == dcel_.verticesSize())
         return PointT ();

      return dcel_.vertex(event_).pos;
   }

   size_t getNextEdge()
   {
      if (cycleIt_ == dcel_.cyclesEnd())
         return -1;

      return cycleIt_.index();
   }

   PointT getDCELVertex( size_t vIdx )
   {
      return dcel_.vertex(vIdx).pos;
   }

private:
   VerticesArrayT const &vertices_;

   SimplifiedDCEL dcel_;
   
   // typedef std::set< StateEdge, typename SimplifiedDCEL::EdgeXLess > SweepState;
   SweepState sweepLineState_;

   // Debug
   size_t event_;
   cycle_iterator cycleIt_;

   OutputIterator out_;
};

///////////////////////////////////////////////////////////////////////////////

namespace triangulation
{

template< class Vertices, class IndexRandomIter, class OutputIterator >
void fast( Vertices const &vertices, IndexRandomIter contoursBegin, IndexRandomIter contoursEnd, OutputIterator out )
{
   PolyFastTriangulator< Vertices, IndexRandomIter, OutputIterator > (vertices, contoursBegin, contoursEnd, out);
}

template< class Vertices, class OutputIterator >
void fast( Vertices const & poly, OutputIterator out )
{
   typedef std::vector< size_t > Contour;
   typedef std::vector< Contour > ContoursArray;

   bool ccw = !cg::clockwise_ordered(poly.begin(), poly.end());

   ContoursArray indices (1);

   indices[0] = Contour (poly.size());
   for (size_t i = 0; i != indices[0].size(); ++i)
      indices[0][i] = ccw ? i : indices[0].size() - 1 - i;

   fast(poly, indices.begin(), indices.end(), out);
}

inline void fast( cg::polygon_2 const & poly, std::vector< cg::point_2 > & vertices, std::vector< cg::point_3i > & faces )
{
   std::vector< std::vector< size_t > > contours;
   indexate_polygon( poly, vertices, contours );

   std::vector< size_t > indices;
   fast( vertices, contours.begin(), contours.end(), std::back_inserter( indices ) );

   for ( size_t i = 0; i < indices.size(); i += 3 )
      faces.push_back( cg::point_3i( indices[i], indices[i+1], indices[i+2] ) );
}

}
}