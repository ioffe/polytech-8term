#pragma once

#include <queue>
#include <stack>
#include <list>

#include <boost\scoped_ptr.hpp>

#include <boost\math\special_functions\next.hpp>

#pragma warning (push)
#pragma warning (disable : 4706)

#include <boost\exception.hpp>

#pragma warning (pop)

#include "Geometry\point_io.h"
#include "Geometry\Verification\verification.h"

#include "Geometry\robust_segments_intersections.h"

#include "Geometry\dcel\dcel.h"
#include "Geometry\dcel\dcel_algos.h"

#include "segment_xless_predicate.h"
#include "common\PerfCounter.h"
#include <fstream>

namespace cg
{

enum BooleanOp
{
   BO_UNION = 0,
   BO_INTERSECTION,
   BO_DIFFERENCE,
   BO_IGNORE,
   BO_XOR
};

namespace pslg
{
   struct invalid_input_or_failure_exception {};
   struct algorithm_hung : std::exception, virtual boost::exception
   {
      explicit algorithm_hung ( cg::point_2 const &ref_point )
         : ref_point (ref_point)
      {
      }

      const char * what () const 
      {
         return util::make_str() << *this;
      }

      cg::point_2 ref_point;

      template< class Stream >
         friend Stream &operator << ( Stream &stream, algorithm_hung const &exc )
      {
         stream << "PSLG overlay hung near point: " << exc.ref_point << std::endl;
         return stream;
      }
   };

   typedef boost::error_info<struct PSLGOverlayProcessor_tag, std::string> PSLGOverlayProcessor_data;
} // End of 'pslg' namespace

///////////////////////////////////////////////////////////////////////////////


struct UIntSet
{
   UIntSet() : sorted_(false) {}

   void clear() 
   {
      for (std::vector<size_t>::const_iterator it = v_.begin(); it != v_.end(); ++it)
      {
         data_[*it] = false;
      }

      v_.clear();
   }

   typedef size_t value_type;

   void insert(value_type x)
   {
      data_[x] = true;
      v_.push_back(x);
      sorted_ = false;
   }

   void remove(value_type x)
   {
      data_[x] = false;
      v_.erase(std::remove(v_.begin(), v_.end(), x), v_.end());
   }

   void set(value_type x, bool in)
   {
      if (in) insert(x); else remove(x);
   }

   void sort() const
   {
      if (!sorted_)
      {
         std::sort(v_.begin(), v_.end());
         v_.erase(std::unique(v_.begin(), v_.end()), v_.end());
         sorted_ = true;
      }
   }

   size_t size() const 
   {
      sort();
      return v_.size();
   }

   bool contains(value_type const & x) const 
   {
      return data_[x] != 0;
   }

   void resize(size_t sz)
   {
      data_.resize(sz);
   }

   template <class F>
      void for_each(F f, value_type upper_limit)
      {
         sort();
         value_type x = -1;
         for (std::vector<size_t>::const_iterator it = v_.begin(); it != v_.end(); ++it)
         {
            for (value_type v = x+1; v != *it; ++v)
            {
               f(v, false);
            }

            x = *it;

            f(x, true);
         }

         for (value_type v = x+1; v != upper_limit; ++v)
         {
            f(v, false);
         }
      }

   mutable std::vector<size_t>  v_;
   mutable bool sorted_;
   std::vector<char>    data_;
};

template< class DCEL >
   struct PSLGOverlayProcessor
{
   ALGORITHM("PSLG Overlay")

   INPUT_VERIFICATION(cg::verification::VF_RECURRING_POINTS |
                      cg::verification::VF_SELF_INTERSECTION |
                      cg::verification::VF_NESTED_ORIENTATION) // separate check for each input

   typedef typename DCEL::point_type point_type;
   typedef typename DCEL::Vertex vertex_type; 

   typedef typename DCEL::edges_iterator edges_iterator;
   typedef typename DCEL::edges_const_iterator edges_const_iterator;

   typedef typename DCEL::exiting_edge_iterator exiting_edge_iterator;
   typedef typename DCEL::exiting_edge_const_iterator exiting_edge_const_iterator;

   typedef typename DCEL::cycle_iterator cycle_iterator;
   typedef typename DCEL::cycle_const_iterator cycle_const_iterator;

   typedef typename DCEL::cycle_edge_iterator cycle_edge_iterator;
   typedef typename DCEL::cycle_edge_const_iterator cycle_edge_const_iterator;

   //void deleteEdge( size_t idx )
   //{
   //   size_t twinEdge = dcelRes_.edge(idx).twinEdge;

   //   size_t prevEdge = dcelRes_.edge(idx).prevEdge;
   //   size_t nextEdge = dcelRes_.edge(idx).nextEdge;

   //   size_t twinNextEdge = dcelRes_.edge(twinEdge).nextEdge;
   //   size_t twinPrevEdge = dcelRes_.edge(twinEdge).prevEdge;

   //   if (dcelRes_.edge(twinEdge).nextEdge == idx)
   //      dcelRes_.vertex(dcelRes_.edge(idx).vertexOrigin).incidentEdge = -1;
   //   else
   //   {
   //      if (dcelRes_.vertex(dcelRes_.edge(idx).vertexOrigin).incidentEdge == idx)
   //         dcelRes_.vertex(dcelRes_.edge(idx).vertexOrigin).incidentEdge = twinNextEdge;
   //      dcelRes_.edge(prevEdge).nextEdge = twinNextEdge;
   //      dcelRes_.edge(twinNextEdge).prevEdge = prevEdge;
   //   }

   //   if (dcelRes_.edge(idx).nextEdge == twinEdge)
   //      dcelRes_.vertex(dcelRes_.edge(idx).vertexDestination).incidentEdge = -1;
   //   else
   //   {
   //      if (dcelRes_.vertex(dcelRes_.edge(idx).vertexDestination).incidentEdge == twinEdge)
   //         dcelRes_.vertex(dcelRes_.edge(idx).vertexDestination).incidentEdge = nextEdge;
   //      dcelRes_.edge(nextEdge).prevEdge = twinPrevEdge;
   //      dcelRes_.edge(twinPrevEdge).nextEdge = nextEdge;
   //   }         

   //   dcelRes_.removeHalfEdge(idx);
   //   dcelRes_.removeHalfEdge(twinEdge);
   //}

   // Instead of explicit construction use cg::pslg::overlay()
   template< class FwdDCELIter >
      PSLGOverlayProcessor ( DCEL &res, FwdDCELIter begin, FwdDCELIter end, BooleanOp bo_op,
                             bool subdivide_input = false, bool debug = false )
         : dcelRes_ (res)
         , boOp_ (bo_op)
         , dcelAmount_ (0)
         , sweepFinished_ (false)
         , curFormationVertex_ (0)
         , checkCount_ (0)
   {
      try
      {
         PerfCounter pf;

         if (begin != end)
            combineDCELs(begin, end, subdivide_input);

         static double t_comb = 0;
         t_comb += pf.time(); pf.restart();

         subStatus_.resize(dcelAmount_);
         setStart_.resize(dcelAmount_);
         setIn_.resize(dcelAmount_);
         setInInitialized_.resize(dcelAmount_);

         // Sweep line for self intersection handling
         initEventQueue();
         planeSweepPreparatory();
         //for (size_t i = 0; i < edgesToDelete_.size(); ++i)
         //   deleteEdge(edgesToDelete_[i]);
         destroyIsolatedVertices();

         static double t_prep = 0;
         t_prep += pf.time(); pf.restart();
         // fixDanglingVertices();

         // Prepare DCEL for sweep line processing
         initEventQueue();

         if (!debug)
         {
            // Process plane sweeping to form result
            planeSweep();

            // Form result DCEL by deleting superfluous edges
            formCycles();

            destroyIsolatedVertices();

            static double t_sweep = 0;
            t_sweep += pf.time(); 

#ifdef DEBUG_POLYOPS
            std::ofstream("d://pslg.log") 
               << "combine = " << t_comb 
               << " prep = " << t_prep
               << " sweep = " << t_sweep;
#endif
         }
      }
      catch (boost::exception & e)
      {
         std::stringstream s;

         s << "PSLGOverlayProcessor encountered some exception during ";
         switch (bo_op)
         {
         case BO_UNION:
            s << "UNION operation";
            break;
         case BO_INTERSECTION:
            s << "INTERSECTION operation";
            break;
         case BO_DIFFERENCE:
            s << "DIFFERENCE operation";
            break;
         case BO_XOR:
            s << "XOR operation";
            break;
         }
         size_t i = 0;
         for (FwdDCELIter dcelIter = begin; dcelIter != end; ++dcelIter, ++i)
            s << std::endl << "DCEL " << i << " contains " << dcelIter->verticesSize() << " vertices and " << dcelIter->edgesSize() << "edges" << std::endl;

         e << pslg::PSLGOverlayProcessor_data(s.str());

         // TODO: make exception info typed
         // e << PSLGOverlayProcessor_data(bo_op, begin, end);

         throw;
      }
   }

   struct EdgeFilter
   {
      EdgeFilter() : dcel_(0) {}
      EdgeFilter(BooleanOp op, DCEL const & dcel) : op_(op), dcel_(&dcel) {}

      bool operator () (int i, int j) const
      {
         return true;
         //return op_ == BO_IGNORE
         //   ?  process(dcel_->edge(i).data, dcel_->edge(j).data)
         //   :  true;
      }

   private:
      bool process(int tA, int tB) const 
      {
         return tA == -1 || tB == -1 || tA == tB;
      }

      template <class T> bool process(T const & tA, T const & tB) const
      {
         return true;
      }

   private:
      BooleanOp      op_;
      DCEL const *   dcel_;
   };

   typedef
      cg::robust_segments_intersector< cg::segment_2, std::vector<cg::segment_2>, EdgeFilter >
      intersector_type;

   boost::scoped_ptr< intersector_type > intersector;

private:
   template< class FwdDCELIter, class intersector_type, class correspondence_map_type >
   void subdivide_edge( FwdDCELIter begin, FwdDCELIter end, bool subdivide_input,
                        std::pair< size_t, bool > edgeHolderIdx, size_t edge2subdiv,
                        intersector_type const &intersector, correspondence_map_type &corr )
   {
      size_t leftIdx = intersector.holder()[edgeHolderIdx.first].left;
      size_t rightIdx = intersector.holder()[edgeHolderIdx.first].right;

      if (leftIdx == -1 || rightIdx == -1)
         return;

      cg::point_2 subdivPoint = intersector.holder()[leftIdx].seg.P1();

      if (!edgeHolderIdx.second)
         std::swap(leftIdx, rightIdx);

      size_t nextEdge =
         subdivideExitingEdge(dcelRes_, edge2subdiv, vertex_type (subdivPoint));

      if (subdivide_input)
      {
         typename correspondence_map_type::iterator place = corr.find(edge2subdiv);
         Verify(place != corr.end());
         FwdDCELIter it = begin; std::advance(it, place->second.first);
         size_t inpNextEdge = subdivideExitingEdge(*it,
            place->second.second, vertex_type (subdivPoint), false);
         corr[nextEdge] = std::make_pair(place->second.first, inpNextEdge);
      }

      subdivide_edge(begin, end, subdivide_input, std::make_pair(leftIdx, edgeHolderIdx.second),
         edge2subdiv, intersector, corr);

      subdivide_edge(begin, end, subdivide_input, std::make_pair(rightIdx, edgeHolderIdx.second),
         nextEdge, intersector, corr);
   }

   template< class FwdDCELIter >
      void combineDCELs( FwdDCELIter begin, FwdDCELIter end, bool subdivide_input )
   {
      std::vector< size_t > edgesContainerSize;

      // Combine result DCEL
      FwdDCELIter curDCEL = begin;
      dcelRes_ = *curDCEL;
      edgesContainerSize.push_back(curDCEL->edgesContainerSize());
      for (++curDCEL; curDCEL != end; ++curDCEL)
      {
         combine(dcelRes_, *curDCEL);
         edgesContainerSize.push_back(curDCEL->edgesContainerSize());
      }
      for (size_t i = 1; i < edgesContainerSize.size(); ++i)
         edgesContainerSize[i] += edgesContainerSize[i - 1];

      // Store edges types
      typedef
         std::map< size_t, std::pair< size_t, size_t > >
         correspondence_map;

      correspondence_map corr;
      dcelAmount_ = edgesContainerSize.size();
      for (DCEL::edges_const_iterator eIt = dcelRes_.edgesBegin(); eIt != dcelRes_.edgesEnd(); ++eIt)
      {
         typedef std::vector< size_t >::const_iterator VIt;
         std::pair< VIt, VIt > edgeEqRange =
            std::equal_range(edgesContainerSize.begin(), edgesContainerSize.end(), eIt.index());

         size_t dcelIdx = edgeEqRange.first - edgesContainerSize.begin();
         if (*edgeEqRange.first == eIt.index())
            dcelIdx++;

         size_t input_idx = eIt.index();
         if (dcelIdx != 0)
            input_idx -= edgesContainerSize[dcelIdx - 1];
         corr[eIt.index()] = std::make_pair(dcelIdx, input_idx);

         edgeInfo(eIt.index()) = EdgeInfo (!eIt->hole ? ET_DCEL : ET_UNDEF, dcelIdx);
      }

      // Process intersections
      std::vector< cg::segment_2 > edge_segments;
      edge_segments.reserve(dcelRes_.edgesSize());
      std::map< size_t, size_t > idx_converter;
      for (DCEL::edges_iterator eIt = dcelRes_.edgesBegin(); eIt != dcelRes_.edgesEnd(); ++eIt)
      {
         if (eIt->hole)
            continue;

         idx_converter[edge_segments.size()] = eIt.index();
         edge_segments.push_back(cg::segment_2 (dcelRes_.vertex(eIt->vertexOrigin).pos,
            dcelRes_.vertex(eIt->vertexDestination).pos));
      }

      EdgeFilter edge_filter(boOp_, dcelRes_);

      intersector.reset(new intersector_type (edge_segments, false, 1e-10, edge_filter));

      typedef
         std::set< size_t >
         intersection_map;
         
      intersection_map subdivision;
      for (size_t i = 0; i < intersector->size(); ++i)
      {
         subdivision.insert((*intersector)[i].idA());
         subdivision.insert((*intersector)[i].idB());
      }

      for (intersection_map::iterator it = subdivision.begin(); it != subdivision.end(); ++it)
      {
         subdivide_edge(begin, end, subdivide_input,
            intersector->input2holder(*it), idx_converter[*it], *intersector, corr);
      }
   }

public:
   struct EventVertex
   {
      EventVertex ()
      {
      }

      EventVertex ( point_type const &position, size_t vIdx )
         : pos (position), idx (vIdx)
      {
      }

      point_type pos;
      size_t idx;
   };

   struct EventVertexYLess
   {
      bool operator () ( EventVertex const &v1, EventVertex const &v2 ) const
      {
         return (v1.pos.y <= v2.pos.y) && (v1.pos.y != v2.pos.y || v1.pos.x > v2.pos.x);
      }
   };

   typedef std::priority_queue< EventVertex, std::vector< EventVertex >, EventVertexYLess > EventQueue;

public:
   struct StateEdge
   {
      typedef point_type point_type;

      StateEdge ( DCEL const &dcel, size_t idx, size_t dcelIdx )
         : dcel (&dcel)
         , edgeIdx (idx)
         , dcelIdx (dcelIdx)
      {
      }

      StateEdge ( point_type const &vOrig, point_type const &vDest )
         : edgeIdx (static_cast< size_t >( -1 )), dcelIdx(static_cast<size_t>(-1)), dcel (NULL)
         , vertexOrigin (vOrig)
         , vertexDestination (vDest)
      {
      }

      operator cg::segment_2 () const
      {
         if (edgeIdx == -1)
            return cg::segment_2 (vertexOrigin, vertexDestination);

         return cg::segment_2 (dcel->vertex(dcel->edge(edgeIdx).vertexOrigin).pos,
                               dcel->vertex(dcel->edge(edgeIdx).vertexDestination).pos);
      }

      DCEL const *dcel;
      size_t edgeIdx;
      size_t dcelIdx;

      point_type vertexOrigin, vertexDestination;

      enum Membership
      {
         SET_EXTERNAL = 0,
         SET_INTERNAL
      };
   };

   typedef std::multiset< StateEdge, pslg::SegmentXLess< StateEdge > > SweepState;

   enum EdgeType
   {
      ET_UNDEF = 0,
      ET_HOLE,
      ET_DCEL,
      ET_DCEL_RES
   };

   struct EdgeInfo
   {
      EdgeInfo () : initialType (ET_UNDEF), type (ET_UNDEF), dcelIdx (static_cast< size_t >(-1))
      {
      }

      EdgeInfo ( EdgeType edgeType, size_t initialDCELIdx )
         : type (edgeType)
         , initialType (edgeType)
         , dcelIdx (initialDCELIdx)
      {
      }

      EdgeType initialType;
      EdgeType type;
      size_t dcelIdx;
   };

private:
   void initEventQueue()
   {
      for (DCEL::vertices_const_iterator vIt = dcelRes_.verticesBegin(); vIt != dcelRes_.verticesEnd(); ++vIt)
         queue_.push(EventVertex (vIt->pos, vIt - dcelRes_.verticesBegin()));
   }

   size_t subdivideExitingEdge( DCEL &dcel, size_t edge2Subdiv,
                                vertex_type const &vertex2Dup, bool handleEdgeInfo = true )
   {
      size_t vDestination = dcel.edge(edge2Subdiv).vertexDestination;

      size_t newVertex = dcel.addVertex(vertex2Dup);
      size_t twinEdge = dcel.edge(edge2Subdiv).twinEdge;
      dcel.vertex(newVertex).incidentEdge = twinEdge;

      size_t nextEdge = dcel.edge(edge2Subdiv).nextEdge;
      size_t prevTwinEdge = dcel.edge(twinEdge).prevEdge;

      size_t newNextEdge = dcel.addEdge(DCEL::Edge (newVertex, vDestination,
         edge2Subdiv, nextEdge, static_cast< size_t >(-1), dcel.edge(edge2Subdiv).hole, dcel.edge(edge2Subdiv).data));
      if (handleEdgeInfo)
      {
         copyEdgeInfo(newNextEdge, edge2Subdiv);
         edgeInfo(newNextEdge).type = edgeInfo(newNextEdge).initialType;
      }

      size_t newPrevTwinEdge = dcel.addEdge(DCEL::Edge (vDestination, newVertex,
         prevTwinEdge, twinEdge, newNextEdge, dcel.edge(twinEdge).hole, dcel.edge(edge2Subdiv).data));
      if (handleEdgeInfo)
      {
         copyEdgeInfo(newPrevTwinEdge, twinEdge);
         edgeInfo(newPrevTwinEdge).type = edgeInfo(newPrevTwinEdge).initialType;
      }

      dcel.edge(newNextEdge).twinEdge = newPrevTwinEdge;

      dcel.edge(edge2Subdiv).nextEdge = newNextEdge;
      dcel.edge(edge2Subdiv).vertexDestination = newVertex;
      dcel.edge(twinEdge).vertexOrigin = newVertex;
      dcel.edge(twinEdge).prevEdge = newPrevTwinEdge;
      dcel.edge(nextEdge).prevEdge = newNextEdge;
      dcel.edge(prevTwinEdge).nextEdge = newPrevTwinEdge;

      if (dcel.vertex(vDestination).incidentEdge == twinEdge)
         dcel.vertex(vDestination).incidentEdge = newPrevTwinEdge;

      return newNextEdge;
   }

   bool isPartOfResult( std::vector< DCEL_SetIn > const &setIn )
   {
      switch (boOp_)
      {
      case BO_UNION:
         // exist x : setIn(x) = true
         return std::find(setIn.begin(), setIn.end(), static_cast<DCEL_SetIn>(true)) != setIn.end();
      case BO_INTERSECTION:
         // any x: setIn(x) = true
         return std::find(setIn.begin(), setIn.end(), static_cast<DCEL_SetIn>(false)) == setIn.end();
      case BO_DIFFERENCE:
         // setIn(0) and any x>0: setIn(x) = false
         return setIn[0] && std::find(util::next(setIn.begin()), setIn.end(), static_cast<DCEL_SetIn>(true)) == setIn.end();
      case BO_IGNORE:
         return false;
      case BO_XOR:
         {
            bool result = false;
            for (size_t i = 0; i < setIn.size(); ++i)
               result = setIn[i] == !result;
            return result;
         }
      }

      Assert(false);
      return false;
   }

   std::vector<size_t>  setInH_, setInG_;

   void handleVertexEvent( std::vector< size_t > const &eventVertices )
   {
      Assert(eventVertices.size() != 0);
      size_t eventIdx = eventVertices[0];

      if (eventVertices.size() > 1) // 'match' event
      {
         std::queue< std::vector< size_t > > secVExitingEdges;

         for (size_t sV = 1; sV < eventVertices.size(); ++sV)
         {
            // size_t secVIncEdge = dcelRes_.vertex(eventVertices[sV]).incidentEdge;
            //size_t startEdge =
            //   cg::dcel::exitingEdgeEqualRange(dcelRes_, dcelRes_.exitingEdgeBegin(secVIncEdge)).first.index();
            size_t startEdge = dcelRes_.vertex(eventVertices[sV]).incidentEdge;
            startEdge = cg::dcel::exitingEdgeEqualRangeDanglingSupport(dcelRes_,
               dcelRes_.exitingEdgeBegin(startEdge)).first.index();

            for (exiting_edge_const_iterator eIt = dcelRes_.exitingEdgeBegin(startEdge); eIt != dcelRes_.exitingEdgeEnd(startEdge);)
            {
               cg::dcel::exiting_edge_const_itpair< DCEL > eqRange =
                  cg::dcel::exitingEdgeEqualRangeDanglingSupport(dcelRes_, eIt);

               std::vector< size_t > eqEdges;
               for (eIt = eqRange.first; eIt != eqRange.second; ++eIt)
                  eqEdges.push_back(eIt.index());

               secVExitingEdges.push(eqEdges);
            }
         }

         while (!secVExitingEdges.empty())
         {
            std::vector< size_t > curEdges = secVExitingEdges.front();
            secVExitingEdges.pop();

            size_t curEdge = curEdges.front();

            size_t prevEdge = dcelRes_.findEdgeNextToLine(dcelRes_.edge(curEdge).vertexDestination,
               eventIdx, DCEL::RobustOrientationPredicate (), false);
            Verify(prevEdge != -1);

            // Correct place in case of intersection by border
            if (cg::dcel::areEdgesEqual(dcelRes_, prevEdge, curEdge))
            {
               cg::dcel::exiting_edge_const_itpair< DCEL > prevEdgeEqRange =
                  cg::dcel::exitingEdgeEqualRange(dcelRes_, dcelRes_.exitingEdgeBegin(prevEdge));
               prevEdge = util::prev(prevEdgeEqRange.first).index();

               size_t searchEdge = dcelRes_.edge(dcelRes_.edge(curEdge).twinEdge).prevEdge;
               for (exiting_edge_const_iterator eIt = prevEdgeEqRange.first; eIt != prevEdgeEqRange.second; ++eIt)
               {
                  if (eIt.index() == searchEdge)
                  {
                     prevEdge = eIt.index();
                     break;
                  }
               }
            }

            for (size_t i = 0; i < curEdges.size(); ++i)
            {
               size_t curEdge = curEdges[i];

               prevEdge = dcelRes_.edge(prevEdge).prevEdge;
               size_t nextEdge = dcelRes_.edge(prevEdge).nextEdge;

               // Insert edge
               dcelRes_.edge(prevEdge).nextEdge = curEdge;
               dcelRes_.edge(curEdge).prevEdge = prevEdge;
               dcelRes_.edge(nextEdge).prevEdge = dcelRes_.edge(curEdge).twinEdge;
               dcelRes_.edge(dcelRes_.edge(curEdge).twinEdge).nextEdge = nextEdge;

               dcelRes_.edge(curEdge).vertexOrigin = eventIdx;
               dcelRes_.edge(dcelRes_.edge(curEdge).twinEdge).vertexDestination = eventIdx;

               prevEdge = curEdge;
            }
         }
      }

      size_t incEdge = dcelRes_.vertex(eventIdx).incidentEdge;
      size_t upEdgeBegin = getUpEdgeBegin(incEdge);

      // L(event)
      exiting_edge_const_iterator eIt = dcelRes_.exitingEdgeBegin(upEdgeBegin);
      
      std::vector<std::pair<size_t, size_t> > edges_up;

      SweepState::iterator it = status_.end();  

      for (; eIt != dcelRes_.exitingEdgeEnd(upEdgeBegin) && isExitingEdgeUp(eIt.index()); ++eIt)
      {
         if (it == status_.end())
            it = status_.find(StateEdge(dcelRes_, eIt.index(), static_cast<size_t>(-1)));
         else
         {
            typename DCEL::Edge const & e = dcelRes_.edge(eIt.index());
            std::pair<size_t, size_t> p (e.vertexOrigin, e.vertexDestination);
            cg::sort2(p.first, p.second);

            edges_up.push_back(p);
         }
         
         //StateEdge toErase (dcelRes_, eIt.index(), static_cast<size_t>(-1));

         /// !
         //status_.erase(toErase);
         //for (size_t dcelIdx = 0; dcelIdx < dcelAmount_; ++dcelIdx)
         //   subStatus_[dcelIdx].erase(toErase);
      }

      if (it != status_.end())
      {
         SweepState::iterator l_it = it, r_it = it;
         
         for (++r_it; r_it != status_.end(); ++r_it)
         {
            typename DCEL::Edge const & e = dcelRes_.edge(r_it->edgeIdx);
            std::pair<size_t, size_t> p (e.vertexOrigin, e.vertexDestination);
            cg::sort2(p.first, p.second);

            std::vector<std::pair<size_t, size_t> >::iterator 
               it = std::find(edges_up.begin(), edges_up.end(), p);

            if (it != edges_up.end())
               edges_up.erase(it);
            else
               break;
         }

         do 
         {
            if (l_it != status_.begin())
            {
               --l_it;

               typename DCEL::Edge const & e = dcelRes_.edge(l_it->edgeIdx);
               std::pair<size_t, size_t> p (e.vertexOrigin, e.vertexDestination);
               cg::sort2(p.first, p.second);

               std::vector<std::pair<size_t, size_t> >::iterator 
                  it = std::find(edges_up.begin(), edges_up.end(), p);

               if (it == edges_up.end())
               {
                  ++l_it;
                  break;
               }
               else
                  edges_up.erase(it);
            } 
            else
               break;

         } while (true);

         Verify(edges_up.empty());

         it = status_.erase(l_it, r_it);
      }
      // U(event), sl - the most left segment of U, sr - the most right segment of U
      typedef typename SweepState::const_iterator StateIt;
      StateIt sl = status_.end(), sr = status_.end();

      //for (size_t dcelIdx = 0; dcelIdx < dcelAmount_; ++dcelIdx)
      //   setStart_[dcelIdx] = statusLocalization(subStatus_[dcelIdx], dcelRes_.vertex(eventIdx).pos);

      //std::vector<typename StateEdge::Membership>  setStart(setStart_.size());
      
      if (it == status_.end()) {
         cg::point_2 queryPt = dcelRes_.vertex(eventIdx).pos;
         it = status_.equal_range(StateEdge(queryPt, queryPt)).first;
      }

      statusLocalization(it);
      //Assert(setStart_ == setStart);

      for (; eIt != dcelRes_.exitingEdgeEnd(upEdgeBegin); ++eIt)
      {
         StateEdge edgeToInsert (dcelRes_, eIt.index(), edgeInfo(eIt.index()).dcelIdx);
         SweepState::iterator insertionPlace = status_.insert(it, edgeToInsert);
         if (sl == status_.end())
            sl = insertionPlace;
         sr = it = insertionPlace;

         Assert(edgeInfo(eIt.index()).dcelIdx < dcelAmount_);
         //subStatus_[edgeInfo(eIt.index()).dcelIdx].insert(edgeToInsert);
      }

      // Process BOp

      std::fill(setIn_.begin(), setIn_.end(), false);
      setInH_.clear();
      //std::fill(setInInitialized_.begin(), setInInitialized_.end(), false);
      setInInitialized_.clear();

      // Init our position in 'pie'
      incEdge = dcelRes_.vertex(eventIdx).incidentEdge;
      incEdge = cg::dcel::exitingEdgeEqualRange(dcelRes_, dcelRes_.exitingEdgeBegin(incEdge)).first.index();
      for (eIt = dcelRes_.exitingEdgeBegin(incEdge); eIt != dcelRes_.exitingEdgeEnd(incEdge);)
      {
         cg::dcel::exiting_edge_const_itpair< DCEL > itPair = cg::dcel::exitingEdgeEqualRange(dcelRes_, eIt);
         for (; eIt != itPair.second; ++eIt)
         {
            setInInitialized_.insert(edgeInfo(eIt.index()).dcelIdx);
            //setInInitialized_[edgeInfo(eIt.index()).dcelIdx] = true;
            setIn_[edgeInfo(eIt.index()).dcelIdx] = !isTwinSignificant(eIt.index());
            setInH_.push_back(edgeInfo(eIt.index()).dcelIdx);
         }

         /// TODO: something to handle it correctly
         //if (std::find(setInInitialized_.begin(), setInInitialized_.end(), static_cast<DCEL_SetIn>(false)) == setInInitialized_.end())
         //   break;
      }

      if (setInInitialized_.size() < dcelAmount_)
      //if (std::find(setInInitialized_.begin(), setInInitialized_.end(), static_cast<DCEL_SetIn>(false)) != setInInitialized_.end())
      {
         for (std::vector<size_t>::const_iterator it = setStartH_.begin(); it != setStartH_.end(); ++it)
         {
            if (setStart_[*it])
            {
               if (!setInInitialized_.contains(*it))
               {
                  setIn_[*it] = true;
                  setInH_.push_back(*it);
               }
            }
         }

         //setInInitialized_.for_each(F(*this), dcelAmount_);
/*
         for (size_t dcelIdx = 0; dcelIdx < dcelAmount_; ++dcelIdx)
         {
            if (!setInInitialized_[dcelIdx])
            {
               setInInitialized_[dcelIdx] = true;
               setIn_[dcelIdx] = setStart_[dcelIdx] == StateEdge::SET_INTERNAL;
            }
         }
*/
         eIt = dcelRes_.exitingEdgeBegin(incEdge);
      }
      size_t toStart = eIt.index();

      eIt = dcelRes_.exitingEdgeBegin(toStart);
      bool forward =  !isPartOfResult(setIn_);

      if (!forward)
      {
         toStart = util::prev(cg::dcel::exitingEdgeEqualRange(dcelRes_, util::prev(eIt)).second).index();
         eIt = dcelRes_.exitingEdgeBegin(toStart);
      }
      while (forward == !isPartOfResult(setIn_) && eIt != dcelRes_.exitingEdgeEnd(toStart))
      {
         cg::dcel::exiting_edge_const_itpair< DCEL > itPair = cg::dcel::exitingEdgeEqualRange(dcelRes_, eIt);
         if (!forward)
            eIt = util::prev(itPair.second);

         for (; eIt != (forward ? itPair.second : util::prev(itPair.first)); forward ? ++eIt : --eIt)
         {
            setIn_[edgeInfo(eIt.index()).dcelIdx] = isTwinSignificant(eIt.index()) != forward;
            setInH_.push_back(edgeInfo(eIt.index()).dcelIdx);
         }

         setInG_.clear();
         for (std::vector<size_t>::const_iterator it = setInH_.begin(); it != setInH_.end(); ++it)
         {
            if (setIn_[*it])
               setInG_.push_back(*it);
         }

         std::sort(setInG_.begin(), setInG_.end());
         setInG_.erase(std::unique(setInG_.begin(), setInG_.end()), setInG_.end());

         SetIn(const_cast<typename DCEL::edge_data&>(eIt->data), dcelRes_.putContourFlags(setInG_));
         if (forward == isPartOfResult(setIn_))
         {
            eIt = itPair.first;
            break;
         }
      }

      // Apply operation to 'pie'
      size_t startEdge = cg::dcel::exitingEdgeEqualRange(dcelRes_, eIt).first.index();
      eIt = dcelRes_.exitingEdgeBegin(startEdge);
      size_t resSectorStart = static_cast< size_t >( -1 );
      for (; eIt != dcelRes_.exitingEdgeEnd(startEdge); ++eIt)
      {
         cg::dcel::exiting_edge_const_itpair< DCEL > itPair = cg::dcel::exitingEdgeEqualRange(dcelRes_, eIt);
         size_t resEdge = static_cast< size_t >( -1 );
         for (; eIt != itPair.second; ++eIt)
         {
            size_t twinEdge = eIt->twinEdge;

            if (edgeInfo(eIt.index()).type == ET_DCEL_RES || edgeInfo(twinEdge).type == ET_DCEL_RES)
               resEdge = eIt.index();

            setIn_[edgeInfo(eIt.index()).dcelIdx] = !isTwinSignificant(eIt.index());
            setInH_.push_back(edgeInfo(eIt.index()).dcelIdx);

            setInG_.clear();
            for (std::vector<size_t>::const_iterator it = setInH_.begin(); it != setInH_.end(); ++it)
            {
               if (setIn_[*it])
                  setInG_.push_back(*it);
            }

            std::sort(setInG_.begin(), setInG_.end());
            setInG_.erase(std::unique(setInG_.begin(), setInG_.end()), setInG_.end());

            SetIn(const_cast<typename DCEL::edge_data&>(eIt->data), dcelRes_.putContourFlags(setInG_));
         }
         --eIt;


         if (resSectorStart == -1 && isPartOfResult(setIn_))
         {
            resSectorStart = resEdge == -1 ? eIt.index() : resEdge;
         }
         else if (resSectorStart != -1 && !isPartOfResult(setIn_))
         {
            if (!cg::dcel::areEdgesEqual(dcelRes_, resSectorStart, eIt.index()))
            {
               size_t curEdge = resEdge == -1 ? eIt.index() : resEdge;
               size_t curTwinEdge = dcelRes_.edge(curEdge).twinEdge;

               // Add 'pie'-sector to result
               edgeInfo(resSectorStart).type = ET_DCEL_RES;
               edgeInfo(dcelRes_.edge(resSectorStart).twinEdge).type = ET_HOLE;
               edgeInfo(curTwinEdge).type = ET_DCEL_RES;
               edgeInfo(curEdge).type = ET_HOLE;
            }
            else
               restoreInitialType(resSectorStart, false);

            resSectorStart = static_cast< size_t >( -1 );
         }
         else
            restoreInitialType(eIt.index(), false);
      }
      if (resSectorStart != -1)
         restoreInitialType(resSectorStart, false);
   }

   void handleVertexPreparatoryEvent( std::vector< size_t > const &eventVertices )
   {
      typedef std::map< size_t, size_t > dcel_knot_type;
      dcel_knot_type knot;

      std::vector< cg::dcel::exiting_edge_const_itpair< DCEL > > eItVec (eventVertices.size());

      for (size_t sV = 0; sV < eventVertices.size(); ++sV)
      {
         size_t incEdge = dcelRes_.vertex(eventVertices[sV]).incidentEdge;
         size_t upEdgeBegin = getUpEdgeBegin(incEdge);

         // Determine dcel
         exiting_edge_iterator searchIt = dcelRes_.exitingEdgeBegin(incEdge);
         size_t dcelIdx = static_cast< size_t >( -1 );
         for (; searchIt != dcelRes_.exitingEdgeEnd(incEdge); ++searchIt)
         {
            if (edgeInfo(searchIt.index()).type != ET_UNDEF)
               dcelIdx = edgeInfo(searchIt.index()).dcelIdx;
            else if (edgeInfo(searchIt->twinEdge).type != ET_UNDEF)
               dcelIdx = edgeInfo(searchIt->twinEdge).dcelIdx;
         }
         Assert(dcelIdx != -1);

         // L(event)
         exiting_edge_const_iterator eIt = dcelRes_.exitingEdgeBegin(upEdgeBegin);

         for (; eIt != dcelRes_.exitingEdgeEnd(upEdgeBegin) && isExitingEdgeUp(eIt.index()); ++eIt)
         {
            StateEdge toErase (dcelRes_, eIt.index(), static_cast<size_t>(-1));
            status_.erase(toErase);
            //for (size_t dcelIdx = 0; dcelIdx < dcelAmount_; ++dcelIdx)
            //   subStatus_[dcelIdx].erase(toErase);
         }

         // Check dcel knot
         dcel_knot_type::iterator place = knot.find(dcelIdx);
         if (place == knot.end())
         {
            knot[dcelIdx] = sV;
            eItVec[sV].first = eIt;
            eItVec[sV].second = dcelRes_.exitingEdgeEnd(upEdgeBegin);
         }
         else
         {
            // Add edges to dcel knot
            std::vector< size_t > edgesToAdd;
            for (eIt = dcelRes_.exitingEdgeBegin(incEdge); eIt != dcelRes_.exitingEdgeEnd(incEdge); ++eIt)
               edgesToAdd.push_back(eIt.index());

            for (size_t i = 0; i < edgesToAdd.size(); ++i)
            {
               size_t curEdge = edgesToAdd[i];

               size_t prevEdge = dcelRes_.findEdgeNextToLine(dcelRes_.edge(curEdge).vertexDestination,
                  eventVertices[place->second], DCEL::RobustOrientationPredicate (), false);
               Verify(prevEdge != -1);

               if (cg::dcel::areEdgesEqual(dcelRes_, prevEdge, curEdge))
               {
                  cg::dcel::exiting_edge_const_itpair< DCEL > prevEdgeEqRange =
                     cg::dcel::exitingEdgeEqualRange(dcelRes_, dcelRes_.exitingEdgeBegin(prevEdge));
                  prevEdge = util::prev(prevEdgeEqRange.first).index();

                  size_t searchEdge = dcelRes_.edge(dcelRes_.edge(curEdge).twinEdge).prevEdge;
                  for (eIt = prevEdgeEqRange.first; eIt != prevEdgeEqRange.second; ++eIt)
                  {
                     if (eIt.index() == searchEdge)
                     {
                        size_t checkEdge = dcelRes_.edge(eIt->twinEdge).prevEdge;
                        if (checkEdge == curEdge)
                        {
                           size_t twinEdge = eIt->twinEdge;
                           if (twinEdge == dcelRes_.vertex(eIt->vertexDestination).incidentEdge)
                              continue;
                        }
                        prevEdge = eIt.index();
                        break;
                     }
                  }
               }

               prevEdge = dcelRes_.edge(prevEdge).prevEdge;
               size_t nextEdge = dcelRes_.edge(prevEdge).nextEdge;

               // Insert edge
               dcelRes_.edge(prevEdge).nextEdge = curEdge;
               dcelRes_.edge(curEdge).prevEdge = prevEdge;
               dcelRes_.edge(nextEdge).prevEdge = dcelRes_.edge(curEdge).twinEdge;
               dcelRes_.edge(dcelRes_.edge(curEdge).twinEdge).nextEdge = nextEdge;

               dcelRes_.edge(curEdge).vertexOrigin = eventVertices[place->second];
               dcelRes_.edge(dcelRes_.edge(curEdge).twinEdge).vertexDestination = eventVertices[place->second];
            }

            // Restore iteration pair for knot
            incEdge = eItVec[place->second].first.index();
            upEdgeBegin = getUpEdgeBegin(incEdge);
            for (eIt = dcelRes_.exitingEdgeBegin(upEdgeBegin); eIt != dcelRes_.exitingEdgeEnd(upEdgeBegin) && isExitingEdgeUp(eIt.index()); ++eIt)
               ;

            eItVec[place->second].first = eIt;
            eItVec[place->second].second = dcelRes_.exitingEdgeEnd(upEdgeBegin);
         }
      }

      // Obtain set status
      typedef typename SweepState::const_iterator StateIt;
      //for (size_t dcelIdx = 0; dcelIdx < dcelAmount_; ++dcelIdx)
      //   setStart_[dcelIdx] = statusLocalization(subStatus_[dcelIdx], dcelRes_.vertex(eventVertices[0]).pos);

      //std::vector<typename StateEdge::Membership>  setStart(setStart_.size());
      statusLocalization(status_, dcelRes_.vertex(eventVertices[0]).pos, setStart_);
      //Assert(setStart_ == setStart);

      // Check knots (U)
      for (dcel_knot_type::iterator it = knot.begin(); it != knot.end(); ++it)
      {
         bool checkComplited;
         do
         {
            StateEdge::Membership cur = setStart_[it->first];
            checkComplited = true;
            for (exiting_edge_const_iterator eIt = eItVec[it->second].first; eIt != eItVec[it->second].second;)
            {
               cg::dcel::exiting_edge_const_itpair< DCEL > itPair = cg::dcel::exitingEdgeEqualRange(dcelRes_, eIt);
               for (; eIt != itPair.second; ++eIt)
               {
                  bool out = isTwinSignificant(eIt.index());
                  if ((out && cur != StateEdge::SET_INTERNAL) ||
                      (!out && cur != StateEdge::SET_EXTERNAL))
                  {
                     checkComplited = false;
                     std::swap(edgeInfo(eIt.index()), edgeInfo(dcelRes_.edge(eIt.index()).twinEdge));
                     std::swap(dcelRes_.edge(eIt.index()).hole, dcelRes_.edge(dcelRes_.edge(eIt.index()).twinEdge).hole);
                     // edgesToDelete_.push_back(eIt.index());
                     break;
                  }

                  cur = out ? StateEdge::SET_EXTERNAL : StateEdge::SET_INTERNAL;
               }
               if (!checkComplited)
                  break;
            }
         } while (!checkComplited);

         for (exiting_edge_const_iterator eIt = eItVec[it->second].first; eIt != eItVec[it->second].second; ++eIt)
         {
            StateEdge edgeToInsert (dcelRes_, eIt.index(), edgeInfo(eIt.index()).dcelIdx);
            status_.insert(edgeToInsert);
            Assert(edgeInfo(eIt.index()).dcelIdx < dcelAmount_);
            //subStatus_[edgeInfo(eIt.index()).dcelIdx].insert(edgeToInsert);
         }
      }
   }

   void formEdgeConnectionsForVertex( size_t v )
   {
      size_t incEdge = dcelRes_.vertex(v).incidentEdge;
      size_t resSectorStart = static_cast< size_t >( -1 );
      exiting_edge_const_iterator eIt;
      for (eIt = dcelRes_.exitingEdgeBegin(incEdge); eIt != dcelRes_.exitingEdgeEnd(incEdge); ++eIt)
         if (edgeInfo(eIt.index()).type == ET_DCEL_RES)
            break;

      if (eIt == dcelRes_.exitingEdgeEnd(incEdge))
         return;

      size_t startEdge = eIt.index();
      size_t holeStart = static_cast< size_t >( -1 );
      for (eIt = dcelRes_.exitingEdgeBegin(startEdge); eIt != dcelRes_.exitingEdgeEnd(startEdge); ++eIt)
      {
         if (resSectorStart == -1 && edgeInfo(eIt.index()).type == ET_DCEL_RES)
         {
            resSectorStart = eIt.index();
            if (holeStart != -1)
            {
               dcelRes_.edge(holeStart).prevEdge = eIt->twinEdge;
               dcelRes_.edge(eIt->twinEdge).nextEdge = holeStart;
               holeStart = static_cast< size_t >( -1 );
            }
            continue;
         }

         if (resSectorStart != -1 && edgeInfo(eIt->twinEdge).type == ET_DCEL_RES)
         {
            dcelRes_.edge(resSectorStart).prevEdge = eIt->twinEdge;
            dcelRes_.edge(eIt->twinEdge).nextEdge = resSectorStart;

            dcelRes_.vertex(dcelRes_.edge(resSectorStart).vertexOrigin).incidentEdge = resSectorStart;
            resSectorStart = static_cast< size_t >( -1 );
            holeStart = eIt.index();
         }
      }
      if (holeStart != -1)
      {
         dcelRes_.edge(holeStart).prevEdge = dcelRes_.edge(startEdge).twinEdge;
         dcelRes_.edge(dcelRes_.edge(startEdge).twinEdge).nextEdge = holeStart;
      }

      for (eIt = dcelRes_.exitingEdgeBegin(startEdge); eIt != dcelRes_.exitingEdgeEnd(startEdge); ++eIt)
      {
         if (edgeInfo(eIt.index()).type == ET_DCEL_RES && edgeInfo(eIt->prevEdge).type != ET_DCEL_RES)
         {
            restoreInitialType(eIt.index());
            eIt = cg::dcel::exitingEdgeEqualRange(dcelRes_, eIt).second;
         }
      }
   }

   void formEdgeConnections()
   {
      for (size_t v = 0; v < dcelRes_.verticesSize(); ++v)
         formEdgeConnectionsForVertex(v);
   }

   void handleMultipleVertexEvent()
   {
      if (queue_.empty())
         return;

      typedef std::vector< size_t > EvtVertices;
      EvtVertices evtVertices;

      EventVertex queueHead = queue_.top();
      do
      {
         EvtVertices::iterator place = std::lower_bound(evtVertices.begin(), evtVertices.end(), queue_.top().idx);
         if (place == evtVertices.end() || *place != queue_.top().idx)
            evtVertices.insert(place, queue_.top().idx);

         queue_.pop();
      } while (!queue_.empty() && !EventVertexYLess ()(queueHead, queue_.top()) &&
                                  !EventVertexYLess ()(queue_.top(), queueHead));

      handleVertexEvent(evtVertices);
   }

   void handleMultipleVertexPreparatoryEvent()
   {
      if (queue_.empty())
         return;

      typedef std::vector< size_t > EvtVertices;
      EvtVertices evtVertices;

      EventVertex queueHead = queue_.top();
      do
      {
         EvtVertices::iterator place = std::lower_bound(evtVertices.begin(), evtVertices.end(), queue_.top().idx);
         if (place == evtVertices.end() || *place != queue_.top().idx)
            evtVertices.insert(place, queue_.top().idx);

         queue_.pop();
      } while (!queue_.empty() && !EventVertexYLess ()(queueHead, queue_.top()) &&
                                  !EventVertexYLess ()(queue_.top(), queueHead));

      handleVertexPreparatoryEvent(evtVertices);
   }

   void planeSweep()
   {
      status_.clear();
      while (!queue_.empty())
         handleMultipleVertexEvent();
      formEdgeConnections();
   }

   void planeSweepPreparatory()
   {
      while (!queue_.empty())
         handleMultipleVertexPreparatoryEvent();
   }

   void formNextCycle()
   {
      while (curEdgeIt_ != dcelRes_.edgesEnd() && (iteratedEdges_.find(curEdgeIt_.index()) != iteratedEdges_.end() ||
             edgeInfo(curEdgeIt_.index()).type == ET_HOLE))
      {
         ++curEdgeIt_;
      }

      size_t startEdge = curEdgeIt_.index();
      nextCycleToResult_ = false;
      for (size_t eIt = startEdge; iteratedEdges_.find(eIt) == iteratedEdges_.end() && edgeInfo(eIt).type != ET_HOLE;
         eIt = dcelRes_.edge(eIt).nextEdge)
      {
         if (edgeInfo(eIt).type == ET_DCEL_RES)
         {
            if (!nextCycleToResult_ && !nextCycle_.empty())
               break;
            nextCycleToResult_ = true;
         }

         nextCycle_.push_back(eIt);
         iteratedEdges_.insert(eIt);
      }

      if (nextCycleToResult_)
      {
         // Add 'hole' edges
         for (size_t i = 0; i < nextCycle_.size(); ++i)
            iteratedEdges_.insert(dcelRes_.edge(nextCycle_[i]).twinEdge);
      }
      else
      {
         for (size_t eIt = dcelRes_.edge(startEdge).prevEdge; iteratedEdges_.find(eIt) == iteratedEdges_.end() &&
                 edgeInfo(eIt).type != ET_HOLE && edgeInfo(eIt).type != ET_DCEL_RES; eIt = dcelRes_.edge(eIt).prevEdge)
         {
            nextCycle_.push_back(eIt);
            iteratedEdges_.insert(eIt);
         }
      }

      while (curEdgeIt_ != dcelRes_.edgesEnd() && (iteratedEdges_.find(curEdgeIt_.index()) != iteratedEdges_.end() ||
             edgeInfo(curEdgeIt_.index()).type == ET_HOLE))
      {
         ++curEdgeIt_;
      }
   }

   struct TestTraits
   {
      TestTraits ( DCEL const &dcel ) : dcel_ (dcel)
      {
      }

      cg::point_2 const &at( size_t edge ) const
      {
         return dcel_.vertex(dcel_.edge(edge).vertexOrigin).pos;
      }

   private:
      DCEL dcel_;
   };

   void handleCycle()
   {
      if (nextCycleToResult_)
      {
         // Do we really need this cycle?
         for (size_t i = 0; i < nextCycle_.size() - 1; ++i)
         {
            size_t eIt = nextCycle_[i];
            size_t nextIt = nextCycle_[i + 1];

            edgeInfo(eIt).type = ET_DCEL_RES;
            dcelRes_.edge(eIt).hole = false;
            edgeInfo(nextIt).type = ET_DCEL_RES;
            dcelRes_.edge(nextIt).hole = false;
            edgeInfo(dcelRes_.edge(eIt).twinEdge).type = ET_HOLE;
            dcelRes_.edge(dcelRes_.edge(eIt).twinEdge).hole = true;
            edgeInfo(dcelRes_.edge(nextIt).twinEdge).type = ET_HOLE;
            dcelRes_.edge(dcelRes_.edge(nextIt).twinEdge).hole = true;
         }
      }
      else
      {
         if (boOp_ != BO_IGNORE)
            for (size_t i = 0; i < nextCycle_.size(); ++i)
               dcelRes_.removeHalfEdge(nextCycle_[i]);
      }

      util::clear(nextCycle_);
   }
   
   void formCycles()
   {
      curEdgeIt_ = dcelRes_.edgesBegin();
      while (curEdgeIt_ != dcelRes_.edgesEnd())
      {
         formNextCycle();
         handleCycle();
      }
   }
   
   void destroyIsolatedVertices()
   {
      // Determine isolated vertices
      std::vector< bool > vertexIsolated (dcelRes_.verticesSize(), true);

      for (cycle_iterator cIt = dcelRes_.cyclesBegin(); cIt != dcelRes_.cyclesEnd(); ++cIt)
      {
         for (cycle_edge_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
         {
            vertexIsolated[eIt->vertexOrigin] = false;
            vertexIsolated[eIt->vertexDestination] = false;
         }
      }

      // Destroy obtained vertices
      if (dcelRes_.verticesSize() > 0)
      {
         size_t lastVertex = dcelRes_.verticesSize() - 1;
         for (int v = vertexIsolated.size() - 1; v >= 0; --v)
         {
            if (vertexIsolated[v])
            {
               if (lastVertex != static_cast< size_t >(v) )
               {
                  size_t incEdge = dcelRes_.vertex(lastVertex).incidentEdge;
                  for (exiting_edge_iterator eIt = dcelRes_.exitingEdgeBegin(incEdge); eIt != dcelRes_.exitingEdgeEnd(incEdge); ++eIt)
                  {
                     eIt->vertexOrigin = v;
                     dcelRes_.edge(eIt->twinEdge).vertexDestination = v;
                  }

                  std::swap(dcelRes_.vertex(v), dcelRes_.vertex(lastVertex));
               }
               --lastVertex;
            }
         }
         dcelRes_.eraseVertices(dcelRes_.verticesBegin() + (lastVertex + 1), dcelRes_.verticesEnd());
      }
   }

   void fixDanglingVertices()
   {
      for (size_t v = 0; v < dcelRes_.verticesSize(); ++v)
      {
         size_t incident = dcelRes_.vertex(v).incidentEdge;
         bool dangling = true;
         size_t adjacentV = static_cast< size_t >( -1 );

         for (DCEL::exiting_edge_const_iterator eIt = dcelRes_.exitingEdgeBegin(incident);
                                                eIt != dcelRes_.exitingEdgeEnd(incident); ++eIt)
         {
            if (adjacentV == -1)
               adjacentV = eIt->vertexDestination;
            else
            {
               if (eIt->vertexDestination != adjacentV)
               {
                  dangling = false;
                  break;
               }
            }
         }

         if (dangling)
         {
            size_t twinEdge = dcelRes_.edge(incident).twinEdge;
            cg::dcel::exiting_edge_const_itpair< DCEL > edgeEqRange =
               cg::dcel::exitingEdgeEqualRange(dcelRes_, dcelRes_.exitingEdgeBegin(twinEdge));
                        
            size_t newStart = util::prev(edgeEqRange.second)->twinEdge;
            dcelRes_.vertex(v).incidentEdge = newStart;
         }
      }
   }

private:
   struct magic_iterator
   {
      magic_iterator ()
         : size (static_cast< size_t >(-1))
         , step (0)
         , inc (1, 1)
      {
      }

      cg::point_2i operator * ()
      {
         return inc;
      }

      magic_iterator &operator ++ ()
      {
         if (size == -1)
         {
            size = 0;
            return *this;
         }

         if (size == 0)
         {
            size = 1;
            inc = cg::point_2i (-1, 0);
            return *this;
         }

         if (step == 2 * size - 1)
            inc = cg::point_2i (0, -1);
         else if (step == 4 * size - 1)
            inc = cg::point_2i (1, 0);
         else if (step == 6 * size - 1)
            inc = cg::point_2i (0, 1);            
         else if (step == 8 * size - 2)
         {
            inc = cg::point_2i (1, 2);
            ++size;
            step = 0;
            return *this;
         }

         ++step;
         return *this;
      }

      size_t overall_step()
      {
         if (size == 0 || size == -1)
            return step;

         return 2 * (size + 1) * size - 3 + step;
      }

      cg::point_2i inc;
      size_t step, size;
   };

private:
   bool isExitingEdgeUp( size_t e )
   {
      point_type posV = dcelRes_.vertex(dcelRes_.edge(e).vertexOrigin).pos;
      point_type posDest = dcelRes_.vertex(dcelRes_.edge(e).vertexDestination).pos;
      return posDest.y > posV.y || (posDest.y == posV.y && posDest.x < posV.x);
   }

   size_t getUpEdgeBegin( size_t edgeIdx )
   {
      exiting_edge_const_iterator edgeIt = dcelRes_.exitingEdgeBegin(edgeIdx);
      if (isExitingEdgeUp(edgeIt.index()))
      {
         while (edgeIt != dcelRes_.exitingEdgeEnd(edgeIdx) && isExitingEdgeUp(edgeIt.index()))
            --edgeIt;
         return edgeIt != dcelRes_.exitingEdgeEnd(edgeIdx) ? (++edgeIt).index() : edgeIt.index();
      }
      else
      {
         exiting_edge_const_iterator leapIt = dcelRes_.exitingEdgeEnd(edgeIdx);
         while (edgeIt != dcelRes_.exitingEdgeEnd(edgeIdx) && !isExitingEdgeUp(edgeIt.index()))
         {
            cg::segment_2 prevEs = cg::dcel::edgeSegment(dcelRes_, *edgeIt);

            ++edgeIt;
            
            cg::segment_2 es = cg::dcel::edgeSegment(dcelRes_, *edgeIt);
            VecOrientation orien = robust_orientation(es.P1(), es.P0(), prevEs.P1());
            if (orien == VO_RIGHT)
               leapIt = edgeIt;
         }
         return edgeIt != dcelRes_.exitingEdgeEnd(edgeIdx) ? edgeIt.index() : leapIt.index();
      }
   }

public:
   EdgeInfo &edgeInfo( size_t e )
   {
      if (e >= edgeInfo_.size())
         edgeInfo_.resize(e + 1);

      return edgeInfo_[e];
   }

   void copyEdgeInfo( size_t dest, size_t source )
   {
      EdgeInfo sourceInfo = edgeInfo(source);
      edgeInfo(dest) = sourceInfo;
   }

   void restoreInitialType( size_t edgeIdx, bool reorientation = true )
   {
      size_t startEdge = static_cast< size_t >( -1 );
      cg::dcel::exiting_edge_const_itpair< DCEL > itPair = cg::dcel::exitingEdgeEqualRange(dcelRes_, dcelRes_.exitingEdgeBegin(edgeIdx));
      for (exiting_edge_const_iterator eIt = itPair.first; eIt != itPair.second; ++eIt)
      {
         size_t twinEdge = eIt->twinEdge;

         if (reorientation)
         {
            if (startEdge == -1 && edgeInfo(twinEdge).type == ET_DCEL_RES)
               startEdge = dcelRes_.edge(twinEdge).prevEdge;

            if (edgeInfo(eIt.index()).type == ET_DCEL_RES && isExitingEdgeUp(eIt.index()))
            {
               if (startEdge == -1)
                  throw boost::enable_error_info(pslg::invalid_input_or_failure_exception ());

               dcelRes_.edge(eIt->nextEdge).prevEdge = startEdge;
               dcelRes_.edge(startEdge).nextEdge = eIt->nextEdge;
            }
         }

         edgeInfo(eIt.index()).type = edgeInfo(eIt.index()).initialType;
         edgeInfo(twinEdge).type = edgeInfo(twinEdge).initialType;
      }
   }

   bool isTwinSignificant( size_t e )
   {
      if (edgeInfo(e).initialType == ET_DCEL)
         return false;
      Assert(edgeInfo(dcelRes_.edge(e).twinEdge).initialType == ET_DCEL);
      return true;
   }
   void statusLocalization( SweepState const &status, cg::point_2 const & query, std::vector<typename StateEdge::Membership> & res)
   {
      std::fill(res.begin(), res.end(), StateEdge::SET_EXTERNAL);

      typedef typename SweepState::const_iterator StateIt;
      std::pair< StateIt, StateIt > eqRange = status.equal_range(StateEdge (query, query));

      for (StateIt it = status.begin(); it != eqRange.first; ++it)
      {
         res[it->dcelIdx] = isTwinSignificant(it->edgeIdx) ? StateEdge::SET_EXTERNAL : StateEdge::SET_INTERNAL;
      }
   }

   std::vector<size_t>  setStartH_;

   void statusLocalization(typename SweepState::iterator next)
   {
      //std::fill(setStart_.begin(), setStart_.end(), StateEdge::SET_EXTERNAL);

      for (std::vector<size_t>::const_iterator it = setStartH_.begin(); it != setStartH_.end(); ++it)
      {
         //AssertRelease(setStart_[*it] == StateEdge::SET_INTERNAL);

         setStart_[*it] = StateEdge::SET_EXTERNAL;
      }
/*

      for (size_t i = 0; i != setStart_.size(); ++i)
      {
         AssertRelease(setStart_[i] == StateEdge::SET_EXTERNAL);
      }
*/

      setStartH_.clear();

      typedef typename SweepState::const_iterator StateIt;
      //std::pair< StateIt, StateIt > eqRange = status.equal_range(StateEdge (query, query));

      for (StateIt it = status_.begin(); it != next; ++it)
      {
         setStart_[it->dcelIdx] = isTwinSignificant(it->edgeIdx) ? StateEdge::SET_EXTERNAL : StateEdge::SET_INTERNAL;
         setStartH_.push_back(it->dcelIdx);
/*
         if (!isTwinSignificant(it->edgeIdx))
         {
            setStart_[it->dcelIdx] = StateEdge::SET_INTERNAL;
            setStartEx_.insert(it->dcelIdx);
         }
         else
         {
            if (setStart_[it->dcelIdx] == StateEdge::SET_INTERNAL)
            {
               setStart_[it->dcelIdx] = StateEdge::SET_EXTERNAL;
               setStartEx_.erase(it->dcelIdx);
            }
         }*/
         //setStart_[it->dcelIdx] = isTwinSignificant(it->edgeIdx) ? StateEdge::SET_EXTERNAL : StateEdge::SET_INTERNAL;
      }

      /*
      std::stable_sort(setStartH_.begin(), setStartH_.end(), StatusCmp());

      for (std::vector<size_t>::const_iterator it = setStartH_.begin(); it != setStartH_.end(); ++it)
      {
         if (setStartG_.empty())
            setStartG_.push_back(*it);
         else {
            if ((*it & 0x7fffffff) == (setStartG_.back() & 0x7fffffff))
               setStartG_.back() = *it;
            else
            {
               if (setStartG_.back() & 0x80000000)
                  setStartG_.pop_back();

               setStartG_.push_back(*it);
            }
         }
      }

      if (!setStartG_.empty() && (setStartG_.back() & 0x80000000))
         setStartG_.pop_back();

      setStartG_.swap(setStartH_);
      setStartG_.clear();

      for (std::vector<size_t>::const_iterator it = setStartH_.begin(); it != setStartH_.end(); ++it)
      {
         setStart_[*it] = StateEdge::SET_INTERNAL;
      }
      */
   }

   typename StateEdge::Membership statusLocalization( SweepState const &status, cg::point_2 const &query )
   {
      typedef typename SweepState::const_iterator StateIt;
      std::pair< StateIt, StateIt > eqRange = status.equal_range(StateEdge (query, query));
      if (eqRange.first != status.begin())
      {
         --eqRange.first;
         if (isTwinSignificant(eqRange.first->edgeIdx))
            return StateEdge::SET_EXTERNAL;

         return StateEdge::SET_INTERNAL;
      }

      return StateEdge::SET_EXTERNAL;
   }

   //
   // For debug purpose only
   //

   SweepState const &getState()
   {
      return status_;
   }

   DCEL const &getDCEL()
   {
      return dcelRes_;
   }

   bool stepByStep()
   {
      if (curEdgeIt_ == dcelRes_.edgesEnd() && nextCycle_.empty() && queue_.empty())
         return false;

      while (!queue_.empty())
      {
         handleMultipleVertexEvent();

         if (queue_.empty())
         {
            if (!sweepFinished_)
            {
               if (dcelRes_.verticesSize() > 0)
                  formEdgeConnectionsForVertex(curFormationVertex_++);
               sweepFinished_ = true;
            }
         }
         return true; 
      }

      if (curFormationVertex_ < dcelRes_.verticesSize())
      {
         formEdgeConnectionsForVertex(curFormationVertex_++);
         if (curFormationVertex_ == dcelRes_.verticesSize())
         {
            curEdgeIt_ = dcelRes_.edgesBegin();
            formNextCycle();
         }
         return true;
      }

      if (curEdgeIt_ != dcelRes_.edgesEnd() || !nextCycle_.empty())
      {
         handleCycle();
         if (curEdgeIt_ != dcelRes_.edgesEnd())
            formNextCycle();
         else
            destroyIsolatedVertices();
      }

      return !queue_.empty() || (curEdgeIt_ != dcelRes_.edgesEnd() || !nextCycle_.empty());
   }

   point_type getNextEvent()
   {
      if (queue_.empty())
         return point_type ();

      return dcelRes_.vertex(queue_.top().idx).pos;
   }

   std::vector< size_t > const &nextCycle() const
   {
      return nextCycle_;
   }

   size_t getCurFormationVertex() const
   {
      return curFormationVertex_;
   }

private:
   // Input data
   BooleanOp boOp_;
   size_t dcelAmount_;

   // Result
   DCEL &dcelRes_;   

   // Queue
   EventQueue queue_;

   // State
   SweepState status_;
   std::vector< SweepState > subStatus_;

   typedef std::queue< size_t > EdgesQueue;
   EdgesQueue edges2Handle_;
   bool nextCycleToResult_;

   std::set< size_t > iteratedEdges_;
   std::vector< size_t > nextCycle_;
   
   // Debug
   edges_iterator curEdgeIt_;
   bool sweepFinished_;
   size_t curFormationVertex_;

   cg::point_2 checkEventPos_;
   size_t checkCount_;
   static size_t const MaxCheckCount = 16;

   // Algorithm data
   std::vector< EdgeInfo > edgeInfo_;
   std::vector< typename StateEdge::Membership > setStart_;
   std::set<size_t> setStartEx_;
   std::vector< DCEL_SetIn > setIn_;
   //std::vector< DCEL_SetIn > setInInitialized_;
   UIntSet  setInInitialized_;

   // std::vector< size_t > edgesToDelete_;
};

///////////////////////////////////////////////////////////////////////////////

namespace pslg
{

template< class FwdDCELIter >
   void overlay( typename FwdDCELIter::value_type & res, FwdDCELIter begin, FwdDCELIter end, BooleanOp bo_op )
{
   PSLGOverlayProcessor<typename FwdDCELIter::value_type> (res, begin, end, bo_op, false);
}

template< class DCEL >
   void overlay( DCEL &res, DCEL const &a, DCEL const &b, BooleanOp bo_op )
{
   std::vector< DCEL > input (2);
   input[0] = a, input[1] = b;
   overlay(res, input.begin(), input.end(), bo_op);
}

template< class DCEL, class FwdDCELIter >
   void overlay_subd( DCEL &res, FwdDCELIter begin, FwdDCELIter end, BooleanOp bo_op )
{
   PSLGOverlayProcessor< DCEL > (res, begin, end, bo_op, true);
}

template< class DCEL >
   void overlay_subd( DCEL &res, DCEL &a, DCEL &b, BooleanOp bo_op )
{
   std::vector< DCEL > input (2);
   input[0] = a, input[1] = b;
   overlay_subd(res, input.begin(), input.end(), bo_op);
   a = input[0], b = input[1];
}

// TODO: functions for all operations

} // End of 'pslg' namespace
} // End of 'cg' namespace
