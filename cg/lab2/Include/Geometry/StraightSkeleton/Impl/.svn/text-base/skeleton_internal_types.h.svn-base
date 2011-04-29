#pragma once

#include <queue>
#include <list>

#include "Impl\SkeletonGrid\Geometry\beam.h"

#pragma pack (push, 1)

namespace cg
{

namespace ss
{
   struct ConstructionParams
   {
      double offsetDistance;
      bool createCorrespondenceData;

      bool optimize;
      bool noEstimation;
      bool outside;

      bool subdivideReflexAngles;
      double subdivisionMinAngle;

      ConstructionParams ()
         : offsetDistance (std::numeric_limits< double >::max())
         , createCorrespondenceData (true)
         , optimize (false)
         , noEstimation (false)
         , outside (false)
         , subdivideReflexAngles (false)
         , subdivisionMinAngle (30)
      {
      }

      ConstructionParams ( double dist, bool outside = false,
                           bool useGrid = false, bool noEstimation = false,
                           bool subdivideReflex = true, double subdivisionMinAngle = 30 )
         : offsetDistance (dist)
         , createCorrespondenceData (true)
         , optimize (useGrid)
         , noEstimation (noEstimation)
         , outside (outside)
         , subdivideReflexAngles (subdivideReflex)
         , subdivisionMinAngle (subdivisionMinAngle)
      {
      }

      template< class Stream >
         void dump( Stream &stream ) const
      {
         stream << offsetDistance << ' ';
         stream << createCorrespondenceData << ' ';
         stream << optimize << ' ';
         stream << noEstimation << ' ';
         stream << outside << ' ';
         stream << subdivideReflexAngles << ' ';
         stream << subdivisionMinAngle;
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         stream >> offsetDistance;
         stream >> createCorrespondenceData;
         stream >> optimize;
         stream >> noEstimation;
         stream >> outside;
         stream >> subdivideReflexAngles;
         stream >> subdivisionMinAngle;
      }
   };

   //
   // Event data section
   //

   enum EventType
   {
      ET_EDGE = 0,
      ET_SPLIT
   };

   struct EventData
   {
      EventType type;
      double t; // time from offset start

      size_t eventVertex;

      size_t eventEdge;
      size_t nextVertex; // valid only if type == ET_EDGE

      size_t eventHost;

      cg::point_2 intersection;
      
      EventData ()
         : t (std::numeric_limits< double >::max())
         , eventVertex (static_cast< size_t >( -1 ))
         , eventEdge (static_cast< size_t >( -1 ))
         , nextVertex (static_cast< size_t >( -1 ))
         , eventHost (static_cast< size_t >( -1 ))
      {
      }

      EventData ( EventType typeEv, double tEv, size_t vertEv, size_t edgeEv )
         : type (typeEv)
         , t (tEv)
         , eventVertex (vertEv)
         , eventEdge (edgeEv)
         , nextVertex (static_cast< size_t >( -1 ))
         , eventHost (static_cast< size_t >( -1 ))
      {
      }

      template< class Stream >
         void dump( Stream &stream ) const
      {
         stream << type << ' ';
         stream << t << ' ';
         stream << eventVertex << ' ';
         stream << eventEdge << ' ';
         stream << nextVertex << ' ';
         stream << eventHost << ' ';
         stream << std::setprecision(32) << intersection.x  << ' ' << intersection.y;
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         size_t temp;
         stream >> temp;
         type = EventType (temp);
         stream >> t;
         stream >> eventVertex;
         stream >> eventEdge;
         stream >> nextVertex;
         stream >> eventHost;
         stream >> intersection.x;
         stream >> intersection.y;
      }
   };

   inline bool operator == ( EventData const &a, EventData const &b )
   {
      return a.type == b.type &&
             a.t == b.t &&
             a.eventVertex == b.eventVertex &&
             a.eventEdge == b.eventEdge &&
             a.nextVertex == b.nextVertex &&
             a.intersection == b.intersection;
   }

   
   struct EventPriorityLess
   {
      bool operator () ( EventData const &a, EventData const &b ) const
      {
         return a.t < b.t || (a.t == b.t && a.type < b.type);
//          return !cg::ge(a.t, b.t, 1e-9) || (cg::eq(a.t, b.t, 1e-9) && (a.type < b.type));
      }
   };

   typedef std::multiset< EventData, EventPriorityLess > EventQueueBase;

   struct EventQueue : public EventQueueBase
   {
      iterator push( EventData const &event )
      {
         return insert(begin(), event);
      }
      
      void pop()
      {
         erase(begin());
      }

      EventData const &top()
      {
         return *begin();
      }
   };

   typedef EventQueue::iterator queue_iterator;

   enum EdgeUpdateType
   {
      EUT_CHANGED = 0,
      EUT_CHANGED_ENLARGED,
      EUT_REMOVED,
      EUT_SPLIT      
   };

   struct EdgeListElem
   {
      size_t edge;

      EdgeUpdateType type;

      std::pair< size_t, size_t > splitRes; // For split event
      std::pair< bool, bool > splitProc;
      std::pair< bool, bool > splitEnlarged;

      beam initialBeam; // For optimized processing
      double beamT;

      EdgeListElem ( size_t edge, EdgeUpdateType type,
                     std::pair< size_t, size_t > splitRes = std::make_pair(-1, -1),
                     std::pair< bool, bool > splitProc = std::make_pair(false, false),
                     std::pair< bool, bool > splitEnlarged = std::make_pair(false, false) )
         : edge (edge)
         , type (type)
         , splitRes (splitRes)
         , splitProc (splitProc)
         , splitEnlarged (splitEnlarged)
      {
      }
   };
   typedef std::list< EdgeListElem > EdgeListEx;
   typedef std::list< size_t > EdgeList;

   struct VertexListElem
   {
      size_t vertex;
      EdgeList splitProhibitions;

      bool process;
      bool convex;

      VertexListElem ( size_t vertex, bool process = true, bool convex = true )
         : vertex (vertex)
         , process (process)
         , convex (convex)
      {
      }
   };
   typedef std::list< VertexListElem > VertexList;

   //
   // Vertex data section
   //

   struct VertexData
   {
      cg::point_2 bisector;
      double t;
      bool processed;

      typedef std::set< size_t > VerticesSet;
      VerticesSet parentVertices;

      queue_iterator iter;

      VertexData ()
         : t (-1)
         , processed (false)
      {}

      VertexData ( cg::Empty const & )
         : t (-1)
         , processed (false)
      {}

      VertexData ( cg::point_2 const &b, double tCreated, size_t parentVertex )
         : bisector (b)
         , t (tCreated)
         , processed (false)
      {
         parentVertices.insert(parentVertex);
      }

      template< class Stream >
         void dump( Stream &stream )
      {
         stream << std::setprecision(32) << bisector.x << ' ' << bisector.y << ' ';
         stream << std::setprecision(32) << t << ' ';
         stream << processed << ' ';
         stream << parentVertices.size() << ' ';
         for (VerticesSet::iterator vIt = parentVertices.begin(); vIt != parentVertices.end(); ++vIt)
            stream << *vIt << ' ';
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         stream >> bisector.x >> bisector.y;
         stream >> t;
         stream >> processed;
         size_t parentVerticesSize;
         stream >> parentVerticesSize;
         for (size_t i = 0; i < parentVerticesSize; ++i)
         {
            size_t v;
            stream >> v;
            parentVertices.insert(v);
         }
      }
   };

   //
   // Edge data section
   //

} // End of 'ss' namespace

} // End of 'cg' namespace

#pragma pack(pop)

