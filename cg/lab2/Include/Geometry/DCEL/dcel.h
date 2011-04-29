#pragma once

#include <set>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include "Common\util.h"

#include "Geometry\empty.h"
#include "Geometry\clockwise.h"

#include "Geometry\cgal_predicates.h"

#include "list_on_vector.h"

#include "dcel_iterator.h"
#include "dcel_utility.h"

//
// Doubly Connected Edge List data type representation
//


namespace cg
{
   typedef char DCEL_SetIn;


   inline bool contains(std::vector<size_t> const & cont, size_t x)
   {
      return std::binary_search(cont.begin(), cont.end(), x);
   }

   typedef std::vector<DCEL_SetIn> ContourFlags;

/*
   inline bool has_0(ContourFlags const & cf)
   {
      return cf[0];
   }

   inline bool has_N(ContourFlags const & cf, size_t N)
   {
      return cf[N];
   }

   template <class F>
      void for_all_set_greater_0(ContourFlags const & cf, F f)
      {
         for (size_t i = 1; i != cf.size(); ++i)
         {
            if(cf[i])
               f(i);
         }
      }
*/

   template <class T>
   void SetIn(T & data, ContourFlags const &)
   {}

   template <class T>
   void SetIn(T & data, std::vector<size_t> const *)
   {}

namespace dcel
{

struct corrupted_exception
{
};


template<
           class Scalar,
           class AddVertexData = cg::Empty,
           class AddEdgeData   = cg::Empty
        >
struct DCEL
{
   //
   // Components data types
   //

   typedef Scalar                          scalar_type;   
   typedef cg::point_t< scalar_type, 2 >   point_type;
   typedef cg::segment_t< scalar_type, 2 > segment_type;

   typedef AddVertexData   vertex_data;
   typedef AddEdgeData     edge_data;

   // Vertices

   struct Vertex
   {
      Vertex ()
      {
      }

      Vertex ( point_type const &pos, size_t incidentEdge = -1 )
         : pos (pos), incidentEdge (incidentEdge)
      {
      }

      point_type     pos;      
      size_t         incidentEdge;

      AddVertexData  data;

      template< class Stream >
         void dump( Stream &stream )
      {
         stream << std::setprecision(32) << pos.x << ' ' << pos.y << ' ';
         stream << incidentEdge << ' ';
         data.dump(stream);
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         stream >> pos.x;
         stream >> pos.y;
         stream >> incidentEdge;
         data.restore(stream);
      }
   };

   typedef std::vector< Vertex > VertexArray;

   typedef typename VertexArray::iterator vertices_iterator;
   typedef typename VertexArray::const_iterator vertices_const_iterator;

   // Edges

   struct Edge
   {
      Edge () : hole (false)
      {
      }

      Edge ( size_t vOrig, size_t vDest, size_t prevE, size_t nextE, size_t twinE, bool h, AddEdgeData const & data = AddEdgeData() )
         : vertexOrigin (vOrig), vertexDestination (vDest)
         , prevEdge (prevE), nextEdge (nextE)
         , hole (h), twinEdge (twinE), data(data)
      {
      }

      bool hole;
      size_t twinEdge;

      size_t vertexOrigin;
      size_t vertexDestination;

      size_t prevEdge;
      size_t nextEdge;

      AddEdgeData data;

      typedef AddEdgeData data_type;

      template< class Stream >
         void dump( Stream &stream )
      {
         stream << hole << ' ';
         stream << twinEdge << ' ';
         stream << vertexOrigin << ' ';
         stream << vertexDestination << ' ';
         stream << prevEdge << ' ';
         stream << nextEdge;
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         stream >> hole;
         stream >> twinEdge;
         stream >> vertexOrigin;
         stream >> vertexDestination;
         stream >> prevEdge;
         stream >> nextEdge;
      }
   };

   typedef list_on_vector< Edge > EdgesArray;

   typedef typename EdgesArray::iterator edges_iterator;
   typedef typename EdgesArray::const_iterator edges_const_iterator;

   typedef
      details::base_edge_const_iterator< EdgesArray, details::ExitingEdgesIterationTraits >
      exiting_edge_const_iterator;
   typedef
      details::base_edge_iterator< EdgesArray, details::ExitingEdgesIterationTraits >
      exiting_edge_iterator;

   typedef
      details::base_edge_const_iterator< EdgesArray, details::EnteringEdgesIterationTraits >
      entering_edge_const_iterator;
   typedef
      details::base_edge_iterator< EdgesArray, details::EnteringEdgesIterationTraits >
      entering_edge_iterator;

   typedef
      details::base_edge_const_iterator< EdgesArray, details::CycleEdgesIterationTraits >
      cycle_edge_const_iterator;
   typedef
      details::base_edge_iterator< EdgesArray, details::CycleEdgesIterationTraits >
      cycle_edge_iterator;

   typedef details::FilteredCycleEdgesIterationTraits<details::CutManyByOneTraits>  CutManyByOneTraits;

   typedef 
      details::base_edge_const_iterator<EdgesArray, CutManyByOneTraits>
      cut_many_cycle_edge_const_iterator;

   // Cycles

   typedef
      details::cycle_const_iterator< EdgesArray, cycle_edge_const_iterator >
      cycle_const_iterator;
   typedef
      details::cycle_iterator< EdgesArray, cycle_edge_iterator, cycle_edge_const_iterator >
      cycle_iterator;

   typedef
      details::cycle_const_iterator_ex< EdgesArray, cut_many_cycle_edge_const_iterator >
      cut_many_cycle_const_iterator;

public:
   void reserve( size_t numVertices )
   {
      edges_.reserve(2 * numVertices);
      vertices_.reserve(numVertices);
   }

   // TODO: replace by std::vector<std::vector<>>
   typedef std::map<int, std::set<size_t> >  ContoursToSegments;

   template <class Traits>
      void buildContoursToSegments(ContoursToSegments & contours2segments, Traits const & traits) const
   {
      for (size_t idx = edges_.head(); idx != -1; idx = edges_.next(idx))
      {
         traits.qualify(edges_, idx, contours2segments);
      }
   }

   template < class VertBiIter, class DataFwdIter >
      void addContour( VertBiIter begin, VertBiIter end, DataFwdIter addVertDataBegin, AddEdgeData const & edgeData = AddEdgeData() )
   {
      size_t curVertexId = vertices_.size();
      
      // Only not looped polylines are valid!
      size_t const size = std::distance(begin, end);
      Assert(size >= 2);
      
      // In robust clipping end points can be on epsilon-distance.
      //Assert(!cg::eq(*begin, *util::prev(end)));
      Assert(*begin != *util::prev(end));

      //edges_.reserve(edges_.size() + 2 * size);
      //vertices_.reserve(vertices_.size() + size);

      size_t lastEdge = static_cast< size_t >( -1 ), lastEdgeTwin = static_cast< size_t >( -1 );
      size_t firstEdge = static_cast< size_t >( -1 ), firstEdgeTwin = static_cast< size_t >( -1 );
      size_t contourStartVertexId = curVertexId;

      DataFwdIter addData = addVertDataBegin;
      for (VertBiIter p = begin; p != end; ++p, ++curVertexId, ++addData)
      {
         VertBiIter nextIndexIter = p != util::prev(end) ? util::next(p) : begin;
         size_t nextVertexId =
            curVertexId - contourStartVertexId + 1 < size ? curVertexId + 1 : contourStartVertexId;

         size_t edge = edges_.push(Edge (curVertexId, nextVertexId, lastEdge, static_cast< size_t >( -1 ), static_cast< size_t >( -1 ), false, edgeData));
         size_t edgeTwin = edges_.push(Edge (nextVertexId, curVertexId, static_cast< size_t >( -1 ), lastEdgeTwin, edge, true, edgeData));

         edges_[edge].twinEdge = edgeTwin;

         vertices_.push_back(Vertex (*p, edge));
         vertices_.back().data = *addData;

         if (lastEdge != -1 && lastEdgeTwin != -1)
         {
            edges_[lastEdge].nextEdge = edge;
            edges_[lastEdgeTwin].prevEdge = edgeTwin;
         }
         else
            firstEdge = edge, firstEdgeTwin = edgeTwin;

         lastEdge = edge, lastEdgeTwin = edgeTwin;
      }

      edges_[lastEdge].nextEdge = firstEdge;
      edges_[firstEdge].prevEdge = lastEdge;
      edges_[lastEdgeTwin].prevEdge = firstEdgeTwin;
      edges_[firstEdgeTwin].nextEdge = lastEdgeTwin;      
   }

   void clear()
   {
      vertices_.clear();
      edges_.clear();
   }
   
public:
   //
   // Components data
   //

   ////////////////////////////////////////////////////////////////////////////

   // STL style vertices iteration

   vertices_iterator       verticesBegin()       { return vertices_.begin(); }
   vertices_const_iterator verticesBegin() const { return vertices_.begin(); }

   vertices_iterator       verticesEnd()         { return vertices_.end(); }
   vertices_const_iterator verticesEnd()   const { return vertices_.end(); }

   // C style vertices iteration

   size_t        verticesSize()       const { return vertices_.size(); }
   Vertex       &vertex( size_t idx )       { return vertices_[idx]; }
   Vertex const &vertex( size_t idx ) const { return vertices_[idx]; }

   ////////////////////////////////////////////////////////////////////////////

   // STL style edges iteration

   edges_iterator       edgesBegin()       { return edges_.begin(); }
   edges_const_iterator edgesBegin() const { return edges_.begin(); }

   edges_iterator       edgesEnd()         { return edges_.end(); }
   edges_const_iterator edgesEnd()   const { return edges_.end(); }

   // C style edges iteration

   size_t        edgesSize()        const { return edges_.size(); }
   Edge         &edge( size_t idx )       { return edges_[idx]; }
   Edge   const &edge( size_t idx ) const { return edges_[idx]; }     

public:
   //
   // Additional iteration. (CCW ordering)
   //

   //////////////////////////////////////////////////////////////////////////

   // Exiting edges iteration

   exiting_edge_const_iterator exitingEdgeBegin( size_t edgeIdx ) const
   {
      return exiting_edge_const_iterator (&edges_, edgeIdx);
   }

   exiting_edge_iterator exitingEdgeBegin( size_t edgeIdx )
   {
      return exiting_edge_iterator (&edges_, edgeIdx);
   }

   exiting_edge_const_iterator exitingEdgeEnd( size_t edgeIdx ) const
   {
      return exiting_edge_const_iterator (&edges_, edgeIdx, true);
   }

   exiting_edge_iterator exitingEdgeEnd( size_t edgeIdx )
   {
      return exiting_edge_iterator (&edges_, edgeIdx, true);
   }

   // Entering edges iteration

   entering_edge_const_iterator enteringEdgeBegin( size_t edgeIdx ) const
   {
      return entering_edge_const_iterator (&edges_, edgeIdx);
   }

   entering_edge_iterator enteringEdgeBegin( size_t edgeIdx )
   {
      return entering_edge_iterator (&edges_, edgeIdx);
   }

   entering_edge_const_iterator enteringEdgeEnd( size_t edgeIdx ) const
   {
      return entering_edge_const_iterator (&edges_, edgeIdx, true);
   }

   entering_edge_iterator enteringEdgeEnd( size_t edgeIdx )
   {
      return entering_edge_iterator (&edges_, edgeIdx, true);
   }

   // Cycle edges iteration

   cycle_edge_const_iterator cycleEdgeBegin( size_t edgeIdx ) const
   {
      return cycle_edge_const_iterator (&edges_, edgeIdx);
   }

   cycle_edge_iterator cycleEdgeBegin( size_t edgeIdx )
   {
      return cycle_edge_iterator (&edges_, edgeIdx);
   }

   cycle_edge_const_iterator cycleEdgeEnd( size_t edgeIdx ) const
   {
      return cycle_edge_const_iterator (&edges_, edgeIdx, true);
   }

   cycle_edge_iterator cycleEdgeEnd( size_t edgeIdx )
   {
      return cycle_edge_iterator (&edges_, edgeIdx, true);
   }

   //////////////////////////////////////////////////////////////////////////

   // Cycles iteration

   cycle_const_iterator cyclesBegin( bool ignoreHoles = true ) const
   {
      return cycle_const_iterator (&edges_, ignoreHoles);
   }

   cycle_iterator cyclesBegin( bool ignoreHoles = true )
   {
      return cycle_iterator (&edges_, ignoreHoles);
   }

   cycle_const_iterator cyclesEnd( bool ignoreHoles = true ) const
   {
      return cycle_const_iterator (&edges_, ignoreHoles, true);
   }

   cycle_iterator cyclesEnd( bool ignoreHoles = true )
   {
      return cycle_iterator (&edges_, ignoreHoles, true);
   }

   template <class Traits>
      static details::base_edge_const_iterator<EdgesArray, Traits>
         filteredFactory(int N, EdgesArray const * edges, size_t idx, bool end, Traits const & traits)
   {
      return details::base_edge_const_iterator<EdgesArray, Traits>(edges, idx, end, traits);
   }

   template <class Traits>
   details::cycle_const_iterator_ex< EdgesArray, details::base_edge_const_iterator<EdgesArray, details::FilteredCycleEdgesIterationTraits< Traits > > >  
      cyclesBegin( int N, std::set<size_t> const & segs, Traits const & traits, bool ignoreHoles = true ) const
   {
      return    details::cycle_const_iterator_ex< EdgesArray, details::base_edge_const_iterator<EdgesArray, details::FilteredCycleEdgesIterationTraits< Traits > > >  
         (&edges_, ignoreHoles, false, 
         segs, boost::bind(&DCEL::filteredFactory<details::FilteredCycleEdgesIterationTraits< Traits > >, N, _1, _2, _3, traits));
   }

   template <class Traits>
   details::cycle_const_iterator_ex< EdgesArray, details::base_edge_const_iterator<EdgesArray, details::FilteredCycleEdgesIterationTraits< Traits > > >  
      cyclesEnd( int N, std::set<size_t> const & segs, Traits const & traits, bool ignoreHoles = true ) const
   {
      return    details::cycle_const_iterator_ex< EdgesArray, details::base_edge_const_iterator<EdgesArray, details::FilteredCycleEdgesIterationTraits< Traits > > >  
         (&edges_, ignoreHoles, true, 
         segs, boost::bind(&DCEL::filteredFactory<details::FilteredCycleEdgesIterationTraits< Traits > >, N, _1, _2, _3, traits));
   }

public:
   //
   // Transformation
   //

   template< class DCEL >
      friend void combine( DCEL &a, DCEL const &b );

   size_t addVertex( Vertex const &v )
   {
      vertices_.push_back(v);
      return vertices_.size() - 1;
   }

   // Simply inserts edge in container all interconnection should be updated manually
   size_t addEdge( Edge const &newEdge )
   {
      return edges_.push(newEdge);
   }

   // Inserts diagonal (pair of edges) and correctly updates edges list
   bool addEdgePair( size_t origin, size_t destination, bool ignoreHoles = false, AddEdgeData const & edgeData = AddEdgeData() )
   {
      int placeDest = -1;
      if (vertex(destination).incidentEdge != -1)
      {
         placeDest = findEdgeNextToLine(origin, destination, RobustOrientationPredicate (), ignoreHoles);
         if (placeDest == -1)
            return false;
      }

      int placeOrig = -1;
      if (vertex(origin).incidentEdge != -1)
      {
         placeOrig = findEdgeNextToLine(destination, origin, RobustOrientationPredicate (), ignoreHoles);
         if (placeOrig == -1)
            return false;
      }

      size_t edgeId = edges_.push(Edge (origin, destination,
         placeOrig != static_cast< size_t >( -1 ) ? edges_[placeOrig].prevEdge : static_cast< size_t >( -1 ),
         placeDest, static_cast< size_t >( -1 ), false, edgeData));

      size_t twinEdgeId = edges_.push(Edge (destination, origin,
         placeDest != -1 ? edges_[placeDest].prevEdge : edgeId, 
         placeOrig != -1 ? placeOrig : edgeId, 
         edgeId, false, edgeData));

      edges_[edgeId].twinEdge = twinEdgeId;
      if (placeOrig == -1)
         edges_[edgeId].prevEdge = twinEdgeId;
      if (placeDest == -1)
         edges_[edgeId].nextEdge = twinEdgeId;

      edges_[edges_[edgeId].prevEdge].nextEdge = edgeId;
      edges_[edges_[edgeId].nextEdge].prevEdge = edgeId;
      edges_[edges_[twinEdgeId].nextEdge].prevEdge = twinEdgeId;
      edges_[edges_[twinEdgeId].prevEdge].nextEdge = twinEdgeId;

      if (placeOrig == -1)
         vertices_[origin].incidentEdge = edgeId;
      if (placeDest == -1)
         vertices_[destination].incidentEdge = twinEdgeId;

      return true;
   }

   // Removes edge from container and tries to correct interconnections
   void deleteHalfEdge( size_t idx )
   {
      if (vertices_[edges_[idx].vertexOrigin].incidentEdge == idx)
         vertices_[edges_[idx].vertexOrigin].incidentEdge = (++exitingEdgeBegin(idx)).index();

      size_t delPrev = edges_[idx].prevEdge;
      size_t delNext = edges_[idx].nextEdge;
      size_t delTwin = edges_[idx].twinEdge;

      if (delTwin != -1)
         edges_[delTwin].twinEdge = static_cast< size_t >( -1 );
      edges_[delNext].prevEdge = delPrev;
      edges_[delPrev].nextEdge = delNext;

      edges_.remove(idx);
   }

   void removeHalfEdge( size_t idx )
   {
      edges_.remove(idx);
   }

   void eraseVertices( vertices_iterator first, vertices_iterator last )
   {
      vertices_.erase(first, last);
      details::restore_edges_vertex_indices(*this);
   }

public:
   enum EdgesPairOrientation
   {
      EPO_LEFT = 0,
      EPO_RIGHT,
      EPO_CODIRECTIONAL,
      EPO_OPPOSITELY_DIRECTED
   };

   struct StdOrientationPredicate
   {
      EdgesPairOrientation operator () ( cg::point_2 const &origin, cg::point_2 const &a, cg::point_2 const &b ) const
      {
         double check = (a - origin) ^ (b - origin);
         
         if (check > 0)
            return EPO_LEFT;
         else if (check < 0)
            return EPO_RIGHT;
         else
            return a * b > 0 ? EPO_CODIRECTIONAL : EPO_OPPOSITELY_DIRECTED;
      }
   };

   struct RobustOrientationPredicate
   {
      EdgesPairOrientation operator ()( cg::point_2 const &origin, cg::point_2 const &a, cg::point_2 const &b ) const
      {
         VecOrientation orien = robust_orientation(b, origin, a);
         switch (orien)
         {
         case VO_LEFT: return DCEL::EPO_LEFT;
         case VO_RIGHT: return DCEL::EPO_RIGHT;
         }

         // VO_COLLINEAR
         if (robust_collinear_are_ordered_along_line(a, origin, b))
            return DCEL::EPO_OPPOSITELY_DIRECTED;

         return DCEL::EPO_CODIRECTIONAL;
      }
   };

   // Finds next edge in destination vertex for line (origin, destination)
   template< class OrientationPredicate >
      int findEdgeNextToLine( size_t origin, size_t destination, OrientationPredicate pred, bool ignoreHoles = true ) const
   {
      segment_type newEdgeSeg (vertex(destination).pos, vertex(origin).pos);

      size_t incident = vertex(destination).incidentEdge;
      exiting_edge_const_iterator it = exitingEdgeBegin(incident);
      while (ignoreHoles && it != exitingEdgeEnd(incident) && it->hole)
      {
         if (it->vertexDestination == origin)
            return -1;
         ++it;
      }

      if (it == exitingEdgeEnd(incident))
         return -1;

      exiting_edge_const_iterator nearest = it;
      segment_type prevEdgeSeg = cg::dcel::edgeSegment(*this, *nearest);
      ++it;
      for (; it != exitingEdgeEnd(incident); ++it)
      {
         //if (it->vertexDestination == origin)
         //   return -1;

         if (ignoreHoles && it->hole)
            continue;

         segment_type curEdgeSeg = cg::dcel::edgeSegment(*this, *it);

         EdgesPairOrientation prevCur = pred(newEdgeSeg.P0(), prevEdgeSeg.P1(), curEdgeSeg.P1());
         if (prevCur == EPO_LEFT || prevCur == EPO_OPPOSITELY_DIRECTED)
         {
            EdgesPairOrientation prevInput = pred(newEdgeSeg.P0(), prevEdgeSeg.P1(), newEdgeSeg.P1());
            if (prevInput == EPO_LEFT || prevInput == EPO_CODIRECTIONAL)
            {
               EdgesPairOrientation inputCur = pred(newEdgeSeg.P0(), newEdgeSeg.P1(), curEdgeSeg.P1());
               if (inputCur == EPO_LEFT || inputCur == EPO_OPPOSITELY_DIRECTED)
                  break;
            }
         }
         else if (prevCur == EPO_RIGHT)
         {
            EdgesPairOrientation prevInput = pred(newEdgeSeg.P0(), prevEdgeSeg.P1(), newEdgeSeg.P1());
            if (prevInput != EPO_RIGHT)
               break;
            else
            {
               EdgesPairOrientation inputCur = pred(newEdgeSeg.P0(), newEdgeSeg.P1(), curEdgeSeg.P1());
               if (inputCur == EPO_LEFT)
                  break;
            }
         }

         nearest = it;
         prevEdgeSeg = curEdgeSeg;
      }

      return nearest.index();
   }

   template< class Stream >
      void dump( Stream &stream )
   {
      stream << vertices_.size() << ' ';
      for (size_t i = 0; i < vertices_.size(); ++i)
      {
         vertices_[i].dump(stream);
         stream << ' ';
      }

      edges_.dump(stream);
   }

   template< class Stream >
      void restore( Stream &stream )
   {
      size_t verticesSize;
      stream >> verticesSize;
      vertices_.resize(verticesSize);
      for (size_t i = 0; i < verticesSize; ++i)
      {
         vertices_[i].restore(stream);
      }

      edges_.restore(stream);
   }
      
   size_t nonHoleEdgesSize() const
   {
      size_t amount = 0;
      for (cycle_const_iterator cIt = cyclesBegin(); cIt != cyclesEnd(); ++cIt)
      {
         for (cycle_edge_const_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
            amount++;
      }

      return amount;
   }

// Additional temporary functions
public:
   size_t edgesContainerSize() const
   {
      return edges_.containerSize();
   }

   std::vector<size_t> const * putContourFlags(std::vector<size_t> const & cf)
   {
      return &*contour_flags_.insert(cf).first;
   }

private:
   EdgesArray edges_;
   VertexArray vertices_;
   std::set<std::vector<size_t> >   contour_flags_;
};

} // End of 'dcel' namespace
} // End of 'cg' namespace
