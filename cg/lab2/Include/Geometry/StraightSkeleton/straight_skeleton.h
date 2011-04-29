#pragma once

#include <sstream>

#include "Common\util.h"
#include "Common\linked_ptr.h"
#include "Common\pointers.h"

#include "Geometry/primitives/rotation.h"
#include "Geometry\polygon_2_io.h"
#include "Geometry\Verification\verification.h"
#include "Geometry\dcel\dcel.h"

#include "straight_skeleton_exceptions.h"

#include "Impl\skeleton_utility.h"
#include "Impl\skeleton_internal_types.h"

#include "Impl\FindSplitIntersection.h"

#include "Impl\SkeletonGrid\SkeletonGrid.h"

// #define SS_DEBUG_LOG

#ifdef SS_DEBUG_LOG
#include <boost/assign/list_inserter.hpp>
#endif /* SS_DEBUG_LOG */

//
// Straight skeleton generation
// Input: connected multicontour
// Output: DCEL of straight skeleton with additional data
// Requirements: input contours do not intersect each other
// Note: To create inner offset pass CCW contours (holes - CW)
//       To create outer offset pass CW contours (holes - CCW)
//

namespace cg
{

template< class VertexBuffer, class MultiContourRandomIterator >
   struct StraightSkeletonGenerator
{
   ALGORITHM("Straight Skeleton Generation")

   typedef ss::ConstructionParams Parameters;

   INPUT_VERIFICATION(cg::verification::VF_RECURRING_POINTS |
                      cg::verification::VF_SELF_INTERSECTION |
                      cg::verification::VF_NESTED_ORIENTATION)

   struct invalid_input_or_failure_exception {};

   typedef StraightSkeletonGenerator< VertexBuffer, MultiContourRandomIterator > SelfT;

   typedef typename VertexBuffer::value_type VertexT;
   typedef std::vector< size_t > ContourT;
   typedef std::vector< ContourT > MultiContourT;

   typedef dcel::DCEL< double, ss::VertexData > DCEL;

   typedef cg::skeleton::skeleton_grid< SelfT > SkeletonGrid;

   typedef std::list< size_t > EdgeList;
   typedef EdgeList VertexList;

   struct CorrespondenceElement
   {                                  
      size_t vertexId;
      size_t nextEdgeId;

      CorrespondenceElement ()
         : vertexId (-1)
         , nextEdgeId (-1)
      {
      }

      CorrespondenceElement ( size_t vertex, size_t edge )
         : vertexId (vertex)
         , nextEdgeId (edge)
      {
      }

      CorrespondenceElement ( size_t vertex )
         : vertexId (vertex)
         , nextEdgeId (static_cast< size_t >( -1 ))
      {
      }

      template< class Stream >
         void dump( Stream &stream )
      {
         stream << vertexId << ' ';
         stream << nextEdgeId;
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         stream >> vertexId;
         stream >> nextEdgeId;
      }
   };
   struct CorrespondenceLessPredicate
   {
      bool operator () ( CorrespondenceElement const &a, CorrespondenceElement const &b ) const
      {
         return a.vertexId < b.vertexId;
      }
   };

   struct VertexCorrespondenceData
   {
      typedef std::set< CorrespondenceElement, CorrespondenceLessPredicate > CorrSet;
      CorrSet corr;
      CorrespondenceElement first, last;
      size_t contactVertex;

      VertexCorrespondenceData ()
         : first (static_cast< size_t >( -1 ))
         , last (static_cast< size_t >( -1 ))
         , contactVertex (static_cast< size_t >( -1 ))
      {
      }

      template< class Stream >
         void dump( Stream &stream )
      {
         first.dump(stream);
         stream << ' ';
         last.dump(stream);
         stream << ' ';
         stream << contactVertex << ' ';
         stream << corr.size() << ' ';
         for (CorrSet::iterator it = corr.begin(); it != corr.end(); ++it)
         {
            it->dump(stream);
            if (it != util::prev(corr.end()))
               stream << ' ';
         }
      }

      template< class Stream >
         void restore( Stream &stream )
      {
         first.restore(stream);
         last.restore(stream);
         stream >> contactVertex;
         size_t setSize;
         stream >> setSize;
         for (size_t i = 0; i < setSize; ++i)
         {
            CorrespondenceElement curElem;
            curElem.restore(stream);
            corr.insert(corr.end(), curElem);
         }
      }
   };
   typedef std::vector< VertexCorrespondenceData > ContourCorrespondenceData;
   typedef std::vector< ContourCorrespondenceData > MultiContourCorrespondenceData;


public:
   StraightSkeletonGenerator ( StraightSkeletonGenerator const &other )
      : multiContour_ (other.multiContour_)
      , attr_ (other.attr_)
      , intermediateOffset_ (other.intermediateOffset_)
      , queue_ (other.queue_)
      , cntSizes_ (other.cntSizes_)
      , t_ (other.t_)
      , iterationStart_ (other.iterationStart_)
      , maxShiftEstimation_ (other.maxShiftEstimation_)
      , ed_ (other.ed_)
      , cantContinue_ (other.cantContinue_)
      , grid_ (other.grid_ ? new SkeletonGrid (*other.grid_.get()) : NULL)
      , basePoint_ (other.basePoint_)
      , dcel_ (other.dcel_)
      , correspondence_ (other.correspondence_)
      , newEvents_ (other.newEvents_)
      , step_ (other.step_)
      , numIterations_ (other.numIterations_)
#ifdef SS_DEBUG_LOG
      , verticesOfInterest_ (other.verticesOfInterest_)
      , debugLog_ (other.debugLog_)
#endif /* SS_DEBUG_LOG */
   {
   }

   // int - hack, compiler thinks this constructor is better for copying
   template< class Stream >
      StraightSkeletonGenerator ( Stream &stream, int )
   {
      restore(stream);
#ifdef SS_DEBUG_LOG
      initDebugLogging();
#endif /* SS_DEBUG_LOG */
   }

   StraightSkeletonGenerator ( VertexBuffer const &vertexBuffer,
                               MultiContourRandomIterator cntBegin, MultiContourRandomIterator cntEnd,
                               ss::ConstructionParams const &attr, bool delayProcess = false, bool debug = false )
      : multiContour_ (cntBegin, cntEnd)
      , attr_ (attr)
      , intermediateOffset_ (delayProcess ? std::numeric_limits< double >::max() : attr.offsetDistance)
      , cntSizes_ (multiContour_.size() + 1)
      , cantContinue_ (false)
      , step_ (0)
      , maxShiftEstimation_ (std::numeric_limits< double >::max())
      , numIterations_ (0)
      , t_(0)
   {
      Verify(multiContour_.size() > 0);
      try
      {
         {
            build_status::step_hiding prep (L"constructing bounding rect");

            // Obtain contour area origin
            cntSizes_[0] = 0;
            cg::rectangle_2 frame (vertexBuffer[multiContour_[0][0]]);
            for (size_t c = 0; c != multiContour_.size(); ++c)
            {
               for (size_t v = 0; v != multiContour_[c].size(); ++v)
                  frame |= vertexBuffer[multiContour_[c][v]];
               cntSizes_[c + 1] = multiContour_[c].size();
            }
            for (size_t c = 1; c != cntSizes_.size(); ++c)
               cntSizes_[c] += cntSizes_[c - 1];

            basePoint_ = cg::point_2 (0.5 * (frame.lo().x + frame.hi().x), 0.5 * (frame.lo().y + frame.hi().y));
         }

         {
            build_status::step_hiding dcel (L"constructing dcel");
            construct_initial_dcel(vertexBuffer);
         }

         if (attr_.createCorrespondenceData)
         {
            build_status::step_hiding corr (L"prepare correspondence data");
            create_correspondence_data();
         }

         {
            build_status::step_hiding bisectors (L"calculating bisectors and fixing input");
            fix_dcel_calculate_bisectors();
         }

         for (DCEL::vertices_iterator vIt = dcel_.verticesBegin(); vIt != dcel_.verticesEnd(); ++vIt)
            vIt->data.iter = queue_.end();

         if (attr_.subdivideReflexAngles)
         {
            build_status::step_hiding subd (L"subdividing reflex angles");
            subdivideReflexAngles();
         }

#ifdef SS_DEBUG_LOG
         initDebugLogging();
#endif /* SS_DEBUG_LOG */

         if (!debug)
         {
            if (!delayProcess)
               processTo(attr_.offsetDistance);
         }
         else
         {
            // Debug code
            if (!initShiftStep(attr_.offsetDistance))
               cantContinue_ = true;
         }
      }
      catch ( cg::verification::invalid_input_contours const & )
      {
         std::ostringstream errorStream;

         // Dump input contour.
         std::string const fileName = cg::dump_polygon(cg::polygon_2(vertexBuffer, cntBegin, cntEnd));

         errorStream << "StraightSkeletonGenerator failed with cg::verification::invalid_input_contours exception\n";

         if (!fileName.empty())
            errorStream << "Polygon dump saved to '" << fileName << "'\n";
         else
            errorStream << "Failed to save polygon dump\n";

         errorStream << "Construction parameters: ";
         attr.dump(errorStream);

         build_status::error() << errorStream.str();

         throw;
      }
      catch ( cg::straight_skeleton_infinite_loop_exception const & )
      {
         std::ostringstream errorStream;

         // Dump input contour.
         std::string const fileName = cg::dump_polygon(cg::polygon_2(vertexBuffer, cntBegin, cntEnd));

         errorStream << "StraightSkeletonGenerator failed with cg::straight_skeleton_infinite_loop_exception exception\n";

         if (!fileName.empty())
            errorStream << "Polygon dump saved to '" << fileName << "'\n";
         else
            errorStream << "Failed to save polygon dump\n";
         errorStream << "Construction parameters: ";
         attr.dump(errorStream);

         build_status::debug() << errorStream.str();

         throw;
      }
      catch (...)
      {
         std::ostringstream errorStream;

         std::string const fileName = cg::dump_polygon(cg::polygon_2(vertexBuffer, cntBegin, cntEnd));
         
         errorStream << "StraightSkeletonGenerator failed with unknown exception\n";

         if (!fileName.empty())
            errorStream << "Polygon dump saved to '" << fileName << "'\n";
         else
            errorStream << "Failed to save polygon dump\n";
         errorStream << "Construction parameters: ";
         attr.dump(errorStream);

         build_status::error() << errorStream.str();

         cg::verification::check_input< SelfT >(vertexBuffer, cntBegin, cntEnd);
         throw;
      }
   }

#ifdef SS_DEBUG_LOG
   void initDebugLogging()
   {
      boost::assign::insert(verticesOfInterest_)();
      debugLog_ = new std::ofstream ("C:\\ss_debug_log");
   }
#endif /* SS_DEBUG_LOG */

// Process control
public:
   bool initShiftStep( double offset )
   {
      numIterations_++;

      iterationStart_ = t_;
      maxShiftEstimation_ = attr_.offsetDistance - t_;
      if (attr_.optimize)
      {
         grid_.reset(new SkeletonGrid (*this));
         if (!attr_.noEstimation && !attr_.outside)
         {
            //if (!grid_->isOptimizationValid())
            //{
            //   attr_.optimize = false;
            //   if (numIterations_ != 1)
            //   {
            //      // Debug code
            //      //std::ofstream dump ("C:\\bug_dump.map");
            //      //std::ofstream dump2 ("C:\\bug_dump.txt");
            //      //for (DCEL::cycle_iterator contourIt = dcel_.cyclesBegin(); contourIt != dcel_.cyclesEnd(); ++contourIt)
            //      //{
            //      //   cg::point_2 lastP;
            //      //   bool first = true;
            //      //   for (DCEL::cycle_edge_iterator polyIt = contourIt->begin; polyIt != contourIt->end; ++polyIt)
            //      //   {
            //      //      cg::point_2 curP = vertexPosFixed(polyIt->vertexOrigin) + basePoint();
            //      //      if (first)
            //      //         lastP = curP;
            //      //      dump2 << polyIt->vertexOrigin << ' ';
            //      //      dump << std::setprecision(32) << '(' << curP.x << ',' << curP.y << ')';
            //      //   }                     
            //      //   dump << std::setprecision(32) << '(' << lastP.x << ',' << lastP.y << ')';
            //      //   dump << std::endl;
            //      //   dump2 << std::endl;
            //      //}
            //      // End of 'debug' code

            //      build_status::error() << "SkeletonGenerator: invalid input or failure!";
            //      throw invalid_input_or_failure_exception ();
            //   }
            //}
            //else
            {
               size_t const maxIteration = 5;
               if (numIterations_ < maxIteration)
                  maxShiftEstimation_ = grid_->makeShiftEstimation();
               else
                  build_status::warning() << "SkeletonGenerator: max iterations exceeded";
            }
         }
      }

      intermediateOffset_ = cg::min(offset, attr_.offsetDistance, t_ + maxShiftEstimation_);

      if (attr_.optimize)
         grid_->processSkeletonGrid(maxShiftEstimation_);

      queue_.empty();
      prepareProcess();

      if (!preprocessOffsetStep())
      {
         iterationStart_ = t_;
         intermediateOffset_ = cg::min(offset, attr_.offsetDistance);
         maxShiftEstimation_ = intermediateOffset_ - t_;
         if (attr_.optimize)
            grid_->processSkeletonGrid(maxShiftEstimation_);

         queue_.empty();
         prepareProcess();

         return preprocessOffsetStep();
      }

      return true;
   }

   bool areAllVerticesProcessed()
   {
      bool fullyProcessed = true;
      for (size_t v = 0; v < dcel_.verticesSize(); ++v)
      {
         if (!dcel_.vertex(v).data.processed)
         {
            // actualizeVertexState(v);
            fullyProcessed = false;
         }
      }

      return fullyProcessed;
   }

   bool processTo( double offset )
   {
      if (offset <= t_)
         return offset < attr_.offsetDistance;

      //build_status::progress_hiding progress (L"processing skeleton offset", attr_.offsetDistance, L"");
//      progress.SetProgress(offset);

      //build_status::counter_hiding offsetStep (L"offset step");

      do 
      {
//         ++offsetStep;
         //build_status::show_all();
         if (!initShiftStep(offset))
         {
            cantContinue_ = true;
            t_ = offset;
            ed_ = ss::EventData ();
            return false;
         }

         //build_status::counter_hiding_ef iterationStep (L"processing event", queue_.size(), L"events");

         size_t iteration(0);
         size_t const maxIteration = 10 * cntSizes_.back(); // TODO: Possible integer overflow.
         Verify(maxIteration > 0);

         for (; subStep() && iteration != maxIteration; ++iteration)
            ;//++iterationStep;

         if (iteration == maxIteration)
            throw straight_skeleton_infinite_loop_exception();

         cantContinue_ = areAllVerticesProcessed();

      } while (attr_.optimize && t_ < offset && !cantContinue_);

      if (cantContinue_)
      {
         t_ = offset;
      }

      ed_ = ss::EventData ();
      return !cantContinue_;
   }

   bool step()
   {
      if (!subStep())
      {
         cantContinue_ = areAllVerticesProcessed();
         if (attr_.optimize && t_ < attr_.offsetDistance && !cantContinue_)
         {
            if (!initShiftStep(attr_.offsetDistance))
            {
               ed_ = ss::EventData ();
               return false;
            }

            return true;
         }
         else if (cantContinue_)
         {
            t_ = attr_.offsetDistance;
         }

         ed_ = ss::EventData ();
         return false;
      }

      return true;
   }

   bool subStep()
   {
      processOffsetStep();
      return preprocessOffsetStep();
   }

// Generated data access functions
public:
   bool completed() const
   {
      return cantContinue_;
   }

   cg::point_2 const & basePoint() const
   {
      return basePoint_;
   }

   DCEL &dcel()
   {
      return dcel_;
   }

   DCEL const & dcel() const
   {
      return dcel_;
   }

   SkeletonGrid &grid()
   {
      Assert(grid_);
      return *grid_;
   }

   ss::ConstructionParams const &constructionParams() const
   {
      return attr_;
   }

   double getTime() const
   {
      return t_;
   }

   double getIterationStartTime() const
   {
      return iterationStart_;
   }

   cg::point_2 vertexPosFixed( size_t v ) const
   {
      return dcel_.vertex(v).pos + dcel_.vertex(v).data.bisector * (t_ - dcel_.vertex(v).data.t);
   }

   cg::point_2 realVertexPos( size_t v ) const
   {
      return vertexPosFixed( v ) + basePoint();
   }

   void set_base_point( point_2 const & new_base_point )
   {
      point_2 dp = basePoint_ - new_base_point;
      for (size_t v = 0; v < dcel_.verticesSize(); v++)
         dcel_.vertex(v).pos += dp;
      basePoint_ = new_base_point;
   }

   cg::point_2 vertexPos( size_t v, cg::point_2 const &localCenter ) const
   {
      return (dcel_.vertex(v).pos - localCenter) + dcel_.vertex(v).data.bisector * (t_ - dcel_.vertex(v).data.t);
   }

   cg::point_2 edgeVector( size_t e, cg::point_2 const &localCenter ) const
   {
      return vertexPos(dcel_.edge(e).vertexDestination, localCenter) - vertexPos(dcel_.edge(e).vertexOrigin, localCenter);
   }

   cg::segment_2 edgeSegment( size_t e, cg::point_2 const &localCenter ) const
   {
      return cg::segment_2 (vertexPos(dcel_.edge(e).vertexOrigin, localCenter),
         vertexPos(dcel_.edge(e).vertexDestination, localCenter));
   }

   VertexCorrespondenceData const &correspondingVertices( size_t contourIdx, size_t vertexIdx ) const
   {
      Assert(attr_.createCorrespondenceData);
      return correspondence_[contourIdx][vertexIdx];
   }

   cg::line_2 getEdgeLine( size_t e, cg::point_2 const &localCenter ) const
   {
      cg::line_2 edgeLine (cg::point_2 (), cg::point_2 (0, 1), line::by_direction);
      size_t aV = dcel_.edge(e).vertexOrigin;
      size_t bV = dcel_.edge(e).vertexDestination;
      cg::point_2 aVP = vertexPos(aV, localCenter);
      cg::point_2 bVP = vertexPos(bV, localCenter);
      if (cg::eq(aVP, bVP))
      {
         cg::point_2 edgeLineDir = dcel_.vertex(bV).data.bisector - dcel_.vertex(aV).data.bisector;
         edgeLine = cg::line_2 (aVP, edgeLineDir, line::by_direction);
      }
      else
         edgeLine = cg::line_2 (aVP, bVP, line::by_points);

      return edgeLine;
   }

// Generator functions
public: //private:
   bool checkSplitCandidate( size_t v, size_t e, bool left, cg::point_2 const &intersection, double &outT )
   {
      if (dcel_.edge(e).hole || vertexEdgeAdjacency(dcel_, v, e))
         return false;

      cg::point_2 const &localCenter (dcel_.vertex(v).pos);

      size_t aV = dcel_.edge(e).vertexOrigin;
      size_t bV = dcel_.edge(e).vertexDestination;

      cg::segment_2 seg = edgeSegment(e, localCenter);

      cg::line_2 edgeLine (vertexPos(aV, localCenter), vertexPos(bV, localCenter), line::by_points);
      cg::line_2 curBLine (vertexPos(v, localCenter), dcel_.vertex(v).data.bisector, line::by_direction);

      if (dcel_.vertex(v).data.bisector * cg::normal(edgeLine) > 0)
         return false;

      if (cg::eq(seg.P0(), vertexPos(v, localCenter), 1e-7) || cg::eq(seg.P1(), vertexPos(v, localCenter), 1e-7))
      {
         cg::point_2 checkVec;
         if (left)
            checkVec = edgeVector(dcel_.edge(dcel_.edge(e).prevEdge).twinEdge, localCenter);
         else
            checkVec = edgeVector(dcel_.edge(e).nextEdge, localCenter);

         if (cg::eq(vertexPos(v, localCenter), dcel_.vertex(v).pos - localCenter, 1e-7))
            return false;

         cg::VecOrientation orient = cg::robust_orientation((dcel_.vertex(v).pos - localCenter) - vertexPos(v, localCenter), checkVec);
         if (orient == (left ? cg::VO_LEFT : cg::VO_RIGHT))
         {
            outT = t_;
            return true;
         }

         return false;
      }

      if (cg::eq_zero((vertexPos(v, localCenter) - edgeLine.p()) * cg::normal(edgeLine), 1e-9))
      {
         double t = seg(vertexPos(v, localCenter));
         outT = t_;
         if (cg::ge(t, 0) && cg::le(t, 1))
            return true;

         return false;
      }

      point_2 edgeIntersection;
      if (!rayLineIntersection(curBLine, edgeLine, edgeIntersection))
         return false;

      cg::point_2 localIntersection = intersection - localCenter;
      if (!cg::ge((localIntersection - vertexPos(aV, localCenter)) ^ dcel_.vertex(aV).data.bisector, 0, 1e-7) ||
          !cg::ge(dcel_.vertex(bV).data.bisector ^ (localIntersection - vertexPos(bV, localCenter)), 0, 1e-7))
      {
         return false;
      }

      outT = cg::distance(edgeLine, localIntersection) + t_;
      return true;
   }

   bool getSplitEventByCheck( size_t v, size_t e, bool left, cg::point_2 const &intersection, ss::EventData &ed )
   {
      double curT = 0;
      if (checkSplitCandidate(v, e, left, intersection, curT))
      {
         ed = ss::EventData (ss::ET_SPLIT, curT, v, e);
         ed.intersection = intersection;
         return true;
      }

      return false;
   }

   bool vertexEdgeAdjacencySubd( size_t v, size_t e )
   {
      if (vertexEdgeAdjacency(dcel_, v, e))
         return true;

      size_t otherV = dcel_.edge(dcel_.vertex(v).incidentEdge).vertexDestination;
      while (v != otherV && cg::eq(vertexPos(v), vertexPos(otherV)))
      {
         if (vertexEdgeAdjacency(dcel_, otherV, e))
            return true;
         otherV = dcel_.edge(dcel_.vertex(otherV).incidentEdge).vertexDestination;
      }
      otherV = dcel_.edge(dcel_.edge(dcel_.vertex(v).incidentEdge).prevEdge).vertexOrigin;
      while (v != otherV && cg::eq(vertexPos(v), vertexPos(otherV)))
      {
         if (vertexEdgeAdjacency(dcel_, otherV, e))
            return true;
         otherV = dcel_.edge(dcel_.edge(dcel_.vertex(otherV).incidentEdge).prevEdge).vertexOrigin;
      }

      return false;
   }

   void addEventToQueue( ss::EventData &event, cg::point_2 const &intersection, size_t eventHost, bool addAnyway = false )
   {
      if (event.t < std::numeric_limits< double >::max())
      {
         event.intersection = intersection;
         event.eventHost = eventHost;
         ss::queue_iterator qIt = queue_.push(event);
         newEvents_.push_back(event);
         if (dcel_.vertex(event.eventHost).data.iter != queue_.end())
            queue_.erase(dcel_.vertex(event.eventHost).data.iter);
         dcel_.vertex(event.eventHost).data.iter = qIt;
      }
      else if (addAnyway)
      {
         if (dcel_.vertex(eventHost).data.iter != queue_.end())
         {
             queue_.erase(dcel_.vertex(eventHost).data.iter);
             dcel_.vertex(eventHost).data.iter = queue_.end();
         }
      }
   }

   void restoreCurrentEvent()
   {
      addEventToQueue(ed_, ed_.intersection, ed_.eventHost);
   }

   cg::point_2 getLocalCenter( size_t v, size_t e )
   {
      return (dcel_.vertex(v).pos + dcel_.vertex(dcel_.edge(e).vertexOrigin).pos + dcel_.vertex(dcel_.edge(e).vertexDestination).pos) / 3;
   }

   bool getSplitEvent( size_t v, size_t e, ss::EventData &splitEvent, bool preprocessing = false, bool exact = false )
   {
      if (dcel_.edge(e).hole /*|| vertexEdgeAdjacencySubd(v, e)*/ || vertexEdgeAdjacency(dcel_, v, e))
         return false;

      size_t aV = dcel_.edge(e).vertexOrigin;
      size_t bV = dcel_.edge(e).vertexDestination;

      if (dcel_.vertex(aV).data.processed || dcel_.vertex(bV).data.processed)
         return false;

      cg::point_2 const &localCenter (dcel_.vertex(v).pos);
      // cg::point_2 localCenter (0, 0);
      // cg::point_2 localCenter (getLocalCenter(v, e));
      // cg::point_2 localCenter (vertexPosFixed(v));

      cg::point_2 BPoint;
      double eventT;
      if (findSplitIntersection(*this, v, e, BPoint, eventT, preprocessing, exact))
      {
         /*
         double checkDist = cg::distance(cg::segment_2 (vertexPos(aV) + eventT * dcel_.vertex(aV).data.bisector,
            vertexPos(bV) + eventT * dcel_.vertex(bV).data.bisector),
            vertexPos(v) + eventT * dcel_.vertex(v).data.bisector);

         double aVBis = cg::norm(dcel_.vertex(aV).data.bisector);
         double bVBis = cg::norm(dcel_.vertex(bV).data.bisector);
         double vBis = cg::norm(dcel_.vertex(v).data.bisector);
         */

         splitEvent = ss::EventData (ss::ET_SPLIT, eventT + t_, v, e);
         splitEvent.intersection = BPoint + localCenter;

         return true;
      }

      return false;
   }

   size_t addDefiniteSplitEvent( size_t prev, size_t cur, size_t next, size_t prevEdge, size_t nextEdge,
                                 std::list< ss::EventData > &deferredEvents )
   {
      cg::point_2 const &localCenter (dcel_.vertex(cur).pos);
      cg::point_2 prevPoint = vertexPos(prev, localCenter), curPoint = vertexPos(cur, localCenter), nextPoint = vertexPos(next, localCenter);
      if (cg::norm_sqr(prevPoint - curPoint) < cg::norm_sqr(nextPoint - curPoint))
      {
         ss::EventData newEvent (ss::ET_SPLIT, t_, prev, nextEdge);
         newEvent.intersection = vertexPosFixed(prev);
         newEvent.eventHost = prev;
         deferredEvents.push_back(newEvent);
         return prev;
      }

      ss::EventData newEvent (ss::ET_SPLIT, t_, next, prevEdge);
      newEvent.intersection = vertexPosFixed(next);
      newEvent.eventHost = next;
      deferredEvents.push_back(newEvent);
      return next;
   }

   ss::EventData obtainVertexEvent( size_t prev, size_t cur, size_t next,
                                    ss::EventData *prevEdgeEvent = 0,
                                    bool convex = false, bool preprocessing = false,
                                    bool exact = false,
                                    size_t excludeEdgeA = -1, size_t excludeEdgeB = -1 )
   {
      cg::point_2 const &localCenter (dcel_.vertex(cur).pos);

      cg::point_2
         prevPoint = vertexPos(prev, localCenter),
         curPoint = vertexPos(cur, localCenter),
         nextPoint = vertexPos(next, localCenter);

      cg::point_2
         prevBisector = dcel_.vertex(prev).data.bisector,
         curBisector = dcel_.vertex(cur).data.bisector,
         nextBisector = dcel_.vertex(next).data.bisector;

      cg::line_2 prevBLine (prevPoint, prevBisector, cg::line::by_direction);
      cg::line_2 curBLine (curPoint, curBisector, cg::line::by_direction);
      cg::line_2 nextBLine (nextPoint, nextBisector, cg::line::by_direction);

      size_t incidentEdge = dcel_.vertex(cur).incidentEdge;

      // Detect possible edge event
      point_2 intersection;
      ss::EventData nearestEvent;
      if ( prevEdgeEvent )
         nearestEvent = *prevEdgeEvent;
      if (nearestEvent.t == std::numeric_limits< double >::max())
      {
         if (!cg::eq(prevPoint, curPoint) &&
             rayLineIntersection(curBLine, prevBLine, intersection)/* && prevBLine(intersection) >= 0*/)
         {
            size_t eventEdge = dcel_.edge(incidentEdge).prevEdge;

            point_2 edgeNormal =
               cg::normalized(cg::normal(cg::segment_2 (point_2 (), edgeVector(eventEdge, localCenter))));

            double relTime = (intersection - curPoint) * edgeNormal;
            if (cg::ge(relTime, 0, 1e-4))
            {
               nearestEvent = ss::EventData (ss::ET_EDGE, cg::max(0., relTime) + t_, prev, eventEdge);
               nearestEvent.intersection = intersection + localCenter;
               nearestEvent.nextVertex = cur;
            }
         }
      }

      ss::EventData prevEvent;

      if (!cg::eq(curPoint, nextPoint) &&
          rayLineIntersection(curBLine, nextBLine, intersection)/* && nextBLine(intersection) >= 0*/)
      {
         size_t eventEdge = incidentEdge;

         point_2 edgeNormal =
            cg::normalized(cg::normal(cg::segment_2 (point_2 (), edgeVector(eventEdge, localCenter))));

         double relTime = (intersection - curPoint) * edgeNormal;
         if (cg::ge(relTime, 0, 1e-4))
         {
            prevEvent = ss::EventData (ss::ET_EDGE, cg::max(relTime, 0.) + t_, cur, eventEdge);
            prevEvent.intersection = intersection + localCenter;
            prevEvent.nextVertex = next;
            if (ss::EventPriorityLess ()(prevEvent, nearestEvent))
               nearestEvent = prevEvent;
         }
      }

      if ( prevEdgeEvent )
         *prevEdgeEvent = prevEvent;

      // Detect possible split event
      bool singular = isVertexSingular(dcel_.edge(incidentEdge).prevEdge, incidentEdge);
      //bool singular = false;
      bool equal_points = cg::eq(prevPoint, curPoint) || cg::eq(curPoint, nextPoint);
      if (!convex && ((!singular && 
         isVertexReflex(dcel_.edge(incidentEdge).prevEdge, incidentEdge)) || equal_points))
      {
         if (attr_.optimize)
         {
            size_t eventEdge;
            double eventT;
            cg::point_2 BPoint;
            if (grid_->findSplitEvent(t_, cur, eventEdge, eventT, BPoint, excludeEdgeA, excludeEdgeB, maxShiftEstimation_, preprocessing, exact))
            {
               ss::EventData splitEvent (ss::ET_SPLIT, eventT, cur, eventEdge);
               splitEvent.intersection = BPoint;
               if (ss::EventPriorityLess ()(splitEvent, nearestEvent))
                  nearestEvent = splitEvent;
            }
         }
         else
         {
            for (DCEL::edges_const_iterator eIt = dcel_.edgesBegin(); eIt != dcel_.edgesEnd(); ++eIt)
            {
               size_t eIdx = eIt.index();
               if (eIdx == excludeEdgeA || eIdx == excludeEdgeB)
                  continue;

               ss::EventData newEvent;
               if (getSplitEvent(cur, eIdx, newEvent, preprocessing, exact))
               {
                  if (ss::EventPriorityLess ()(newEvent, nearestEvent))
                     nearestEvent = newEvent;
               }
            }
         }
      }

      return nearestEvent;
   }

   void prepareProcess()
   {
      for (DCEL::cycle_iterator contourIt = dcel_.cyclesBegin(); contourIt != dcel_.cyclesEnd(); ++contourIt)
      {
         ss::EventData prevEvent;
         for (DCEL::cycle_edge_iterator polyIt = contourIt->begin; polyIt != contourIt->end; ++polyIt)
         {
            size_t cur = polyIt->vertexOrigin;
            size_t next = polyIt->vertexDestination;
            size_t prev = dcel_.edge(polyIt->prevEdge).vertexOrigin;

            obtain_event_and_push(prev, cur, next, &prevEvent, false, true, true);
         }
      }
   }

   void maintainConsistency( cg::point_2 const &localCenter, size_t vertexIdx,
                             size_t prevEdge, size_t nextEdge,
                             ss::EdgeListEx &/* edges */, ss::VertexList &vertices )
   {
      size_t vPrev = dcel_.edge(prevEdge).vertexOrigin;
      size_t vNext = dcel_.edge(nextEdge).vertexDestination;

      double checkAnglePrev = cg::normalized(dcel_.vertex(vertexIdx).data.bisector -
         dcel_.vertex(vPrev).data.bisector) ^ getEdgeLine(prevEdge, localCenter).r();

      bool updateBaseVertex = false;

      cg::point_2 vertexLocPos (vertexPos(vertexIdx, localCenter));

      if (!cg::eq_zero(checkAnglePrev, 1e-8))
      {
         if (attr_.optimize)
         {
            grid_->removeVertex(vPrev, maxShiftEstimation_);
         }

         size_t prevPrevEdge = dcel_.edge(prevEdge).prevEdge;

         actualizeVertexState(vPrev);

         cg::point_2 prevPos (vertexPos(dcel_.edge(prevPrevEdge).vertexOrigin, localCenter));
         cg::point_2 curPos (vertexPos(vPrev, localCenter));

         if (!cg::eq(prevPos, curPos))
         {
            dcel_.vertex(vPrev).data.bisector = calculateBisector(prevPos, curPos, vertexLocPos, true);
            updateBaseVertex = true;
            vertices.push_back(ss::VertexListElem (vPrev, true, false));
         }

         if (attr_.optimize)
            grid_->addVertex(vPrev, maxShiftEstimation_);
      }

      double checkAngleNext = cg::normalized(dcel_.vertex(vNext).data.bisector -
         dcel_.vertex(vertexIdx).data.bisector) ^ getEdgeLine(nextEdge, localCenter).r();

      if (!cg::eq_zero(checkAngleNext, 1e-8))
      {
         if (attr_.optimize)
         {
            grid_->removeVertex(vNext, maxShiftEstimation_);
         }

         size_t nextNextEdge = dcel_.edge(nextEdge).nextEdge;

         actualizeVertexState(vNext);

         cg::point_2 curPos (vertexPos(vNext, localCenter));
         cg::point_2 nextPos (vertexPos(dcel_.edge(nextNextEdge).vertexDestination, localCenter));

         if (!cg::eq(curPos, nextPos))
         {
            dcel_.vertex(vNext).data.bisector = calculateBisector(vertexLocPos, curPos, nextPos, true);
            vertices.push_back(ss::VertexListElem (vNext, true, false));
            updateBaseVertex = true;
         }

         if (attr_.optimize)
            grid_->addVertex(vNext, maxShiftEstimation_);
      }

      if (updateBaseVertex)
      {
         //static size_t numShitEvents = 0;
         dcel_.vertex(vertexIdx).data.bisector = calculateBisector(vertexPos(vPrev, localCenter),
            vertexPos(vertexIdx, localCenter), vertexPos(vNext, localCenter), true);
         //numShitEvents++;
      }

      //// Debug code
      //double checkAngleNext2 = cg::normalized(dcel_.vertex(vNext).data.bisector -
      //   dcel_.vertex(vertexIdx).data.bisector) ^ getEdgeLine(nextEdge, localCenter).r();
      //double checkAnglePrev2 = cg::normalized(dcel_.vertex(vertexIdx).data.bisector -
      //   dcel_.vertex(vPrev).data.bisector) ^ getEdgeLine(prevEdge, localCenter).r();
      //// End of debug code
   }

   void deleteHalfEdge( size_t edge )
   {
      dcel_.deleteHalfEdge(edge);
   }

   // Prepare data for main processing
   bool preprocessOffsetStep()
   {
      if (cantContinue_)
         return false;

      if (queue_.empty() || !cg::le(t_, intermediateOffset_))
      {
         t_ = intermediateOffset_;
         return false;
      }

      bool continueAnyway = false;
      bool invalidEvent = false;
      do
      {
         continueAnyway = false;
         ed_ = queue_.top();

         size_t eventHost = ed_.eventVertex;
         if (dcel_.vertex(eventHost).data.iter == queue_.end() || dcel_.vertex(eventHost).data.iter != queue_.begin())
         {
            Assert(ed_.type == ss::ET_EDGE);
            eventHost = dcel_.edge(ed_.eventEdge).vertexDestination;
            Assert(dcel_.vertex(eventHost).data.iter != queue_.end());
            Assert(dcel_.vertex(eventHost).data.iter == queue_.begin());
         }

         queue_.pop();
         dcel_.vertex(eventHost).data.iter = queue_.end();

         invalidEvent = dcel_.vertex(ed_.eventVertex).data.processed ||
            (ed_.type == ss::ET_SPLIT && (dcel_.vertex(dcel_.edge(ed_.eventEdge).vertexDestination).data.processed ||
            dcel_.vertex(dcel_.edge(ed_.eventEdge).vertexOrigin).data.processed)) ||
            (ed_.type == ss::ET_EDGE && dcel_.vertex(ed_.nextVertex).data.processed);

      } while (!queue_.empty() && (continueAnyway || invalidEvent));

      if (invalidEvent)
      {
         t_ = intermediateOffset_;
         return false;
      }

      if (ed_.t > intermediateOffset_)
      {
         t_ = intermediateOffset_;
         addEventToQueue(ed_, ed_.intersection, ed_.eventHost);
         return false;
      }

      return true;
   }

   void addCorrespondenceData( size_t vertex, size_t correspondentVertex, size_t nextEdgeId )
   {
      if (!attr_.createCorrespondenceData)
         return;

      MultiContourIdx idx = cntIdx(vertex);
      Assert(isMultiContourVertex(idx));
      correspondence_[idx.first][idx.second].corr.insert(CorrespondenceElement (correspondentVertex, nextEdgeId));

      dcel_.vertex(correspondentVertex).data.parentVertices.insert(vertex);
   }

   void updateCorrespondenceData( size_t correspondentVertex, size_t newCorrespondentVertex, size_t nextEdgeId,
                                  bool fetchParents = false, ss::VertexData::VerticesSet *parents = 0)
   {
      if (!attr_.createCorrespondenceData)
         return;

      typedef ss::VertexData::VerticesSet::const_iterator vsci;
      for (vsci it = dcel_.vertex(correspondentVertex).data.parentVertices.begin();
                it != dcel_.vertex(correspondentVertex).data.parentVertices.end(); ++it)
      {
         size_t parentVertex = *it;
         if (parentVertex == -1)
            continue;

         MultiContourIdx idx = cntIdx(parentVertex);
         VertexCorrespondenceData::CorrSet::iterator place =
            correspondence_[idx.first][idx.second].corr.find(correspondentVertex);
         Assert(place != correspondence_[idx.first][idx.second].corr.end());

         if (!fetchParents)
            dcel_.vertex(newCorrespondentVertex).data.parentVertices.insert(parentVertex);
         else
            if ( parents )
               parents->insert(parentVertex);

         correspondence_[idx.first][idx.second].corr.erase(place);
         VertexCorrespondenceData::CorrSet::iterator newPlace =
            correspondence_[idx.first][idx.second].corr.insert(CorrespondenceElement (newCorrespondentVertex, nextEdgeId)).first;

         if (correspondence_[idx.first][idx.second].first.vertexId == correspondentVertex)
            correspondence_[idx.first][idx.second].first = *newPlace;
         if (correspondence_[idx.first][idx.second].last.vertexId == correspondentVertex)
            correspondence_[idx.first][idx.second].last = *newPlace;
      }
   }

   void assureNextEdgeIdSet( size_t correspondentVertex, size_t newCorrespondentVertex, size_t nextEdgeId )
   {
      if (!attr_.createCorrespondenceData)
         return;

      typedef ss::VertexData::VerticesSet::const_iterator vsci;
      for (vsci it = dcel_.vertex(correspondentVertex).data.parentVertices.begin();
         it != dcel_.vertex(correspondentVertex).data.parentVertices.end(); ++it)
      {
         size_t parentVertex = *it;
         if (parentVertex == -1)
            continue;

         MultiContourIdx idx = cntIdx(parentVertex);
         VertexCorrespondenceData::CorrSet::iterator place =
            correspondence_[idx.first][idx.second].corr.find(newCorrespondentVertex);
         Assert(place != correspondence_[idx.first][idx.second].corr.end());

         if (place->nextEdgeId == -1)
         {
            place->nextEdgeId = nextEdgeId;
            if (correspondence_[idx.first][idx.second].first.vertexId == newCorrespondentVertex)
               correspondence_[idx.first][idx.second].first = *place;
            if (correspondence_[idx.first][idx.second].last.vertexId == newCorrespondentVertex)
               correspondence_[idx.first][idx.second].last = *place;
         }
      }
   }

   void obtain_event_and_push( size_t prev, size_t cur, size_t next,
                               ss::EventData *prevEdgeEvent = 0, bool convex = false,
                               bool additionalCondition = true, bool preprocessing = false )
   {
      ss::EventData newEvent = obtainVertexEvent(prev, cur, next, prevEdgeEvent, convex, preprocessing);
      if (additionalCondition)
      {
         addEventToQueue(newEvent, newEvent.intersection, cur);
      }
   }

   void construct_initial_dcel( VertexBuffer const &vertexBuffer );
   void create_correspondence_data();
   void fix_dcel_calculate_bisectors();

   void subdivideReflexAngles();

   void update_queue( ss::EdgeListEx const &edges, ss::VertexList const &vertices );
   void update_queue_optimized( ss::EdgeListEx const &edges, ss::VertexList const &vertices );

   void handle_edge_event();

#ifndef SS_DEBUG_LOG
   bool handle_edge_event_degeneracy( cg::point_2 const &localCenter, size_t prevEdge, size_t nextEdge );
#else /* SS_DEBUG_LOG */
   bool handle_edge_event_degeneracy( cg::point_2 const &localCenter, size_t prevEdge, size_t nextEdge, bool logCurEvent );
#endif /* SS_DEBUG_LOG */

   void stopCycleVertices( EdgeList const &edgeList,
                           bool useFixedPos = false, cg::point_2 const &stopPos = cg::point_2 () );
   size_t collapse( EdgeList const &edgeList, VertexList &incVertices, ss::VertexData::VerticesSet &parents );

   void handle_split_event();

#ifndef SS_DEBUG_LOG
   bool handle_split_zero_edge( cg::point_2 const &localCenter );
#else /* SS_DEBUG_LOG */
   bool handle_split_zero_edge( cg::point_2 const &localCenter, bool logCurEvent );
#endif /* SS_DEBUG_LOG */

   bool collapse_split( cg::point_2 const &localCenter, size_t startEdge, size_t endEdge, bool &specialHandling );
   
   // Main processing function
   void processOffsetStep()
   {
      // !Note: Can cause unexpected behaviour if called without preprocessOffsetStep()

      if (cantContinue_ || !cg::le(t_, attr_.offsetDistance) || !cg::le(t_, intermediateOffset_))
         return;

      step_++;
      newEvents_.clear();

      double prevT = t_;
      t_ = cg::min(ed_.t, attr_.offsetDistance, intermediateOffset_);

      // Update connectivity info in DCEL
      if ((t_ < attr_.offsetDistance && t_ < intermediateOffset_) ||
          (cg::eq(t_, attr_.offsetDistance) && cg::eq(ed_.t, attr_.offsetDistance)) ||
          (cg::eq(t_, intermediateOffset_) && cg::eq(ed_.t, intermediateOffset_)))
      {                           
         if (ed_.type == ss::ET_SPLIT)
         {
            cg::point_2 const &localCenter (dcel_.vertex(ed_.eventVertex).pos);

            // Check that split event is valid (edge could changed its movements some steps before)
            double dist = cg::distance(edgeSegment(ed_.eventEdge, localCenter), vertexPos(ed_.eventVertex, localCenter));
            if (!cg::eq_zero(dist, 1e-5))
            {
               // !Now this code shouldn't be called

               //Verify(false);

               size_t cur = ed_.eventVertex;

               ss::EventData newEvent = ed_;
               if (cg::robust_orientation(vertexPos(ed_.eventVertex, localCenter) -
                  vertexPos(dcel_.edge(ed_.eventEdge).vertexOrigin, localCenter), edgeVector(ed_.eventEdge, localCenter)) != VO_LEFT)
               {
                  newEvent.t += dist / (1 + cg::abs(cg::normal(getEdgeLine(ed_.eventEdge, localCenter)) * dcel_.vertex(ed_.eventVertex).data.bisector));
               }
               else
                  newEvent.t -= dist / (1 + cg::abs(cg::normal(getEdgeLine(ed_.eventEdge, localCenter)) * dcel_.vertex(ed_.eventVertex).data.bisector));

               t_ = prevT;
               addEventToQueue(newEvent, newEvent.intersection, cur);

               //t_ = prevT;
               //ss::EventData newEvent = obtainVertexEvent(prev, cur, next, ss::EventData (), false, false, false);
               //if (newEvent.t < std::numeric_limits< double >::max())
               //{
               //   if (newEvent.eventVertex == ed_.eventVertex && newEvent.eventEdge == ed_.eventEdge &&
               //       cg::eq(newEvent.t, ed_.t))
               //   {
               //      t_ = cg::min(ed_.t, attr_.offsetDistance, intermediateOffset_);
               //      return;
               //   }

               //   addEventToQueue(newEvent, newEvent.intersection, cur);
               //}

               return;
            }

            // Check that it isn't a rare split-collapse event
            size_t nextEdge = dcel_.vertex(ed_.eventVertex).incidentEdge;
            size_t prevEdge = dcel_.edge(nextEdge).prevEdge;

            if (dcel_.edge(nextEdge).nextEdge == ed_.eventEdge &&
                dcel_.edge(prevEdge).prevEdge == ed_.eventEdge)
            {
               ss::EventData newEvent (ss::ET_EDGE, ed_.t, dcel_.edge(ed_.eventEdge).vertexOrigin, ed_.eventEdge);
               newEvent.nextVertex = dcel_.edge(ed_.eventEdge).vertexDestination;
               addEventToQueue(newEvent, ed_.intersection, newEvent.eventVertex);
               return;
            }
         }

         switch (ed_.type)
         {         
         case ss::ET_EDGE:
            handle_edge_event();
            break;

         case ss::ET_SPLIT:
            handle_split_event();
            break;
         }
      }
   }

   typedef std::pair< size_t, size_t > MultiContourIdx;
   size_t dcelIdx( MultiContourIdx cIdx )
   {
      return cntSizes_[cIdx.first] + (!attr_.outside ? cIdx.second :
         (multiContour_[cIdx.first].size() - 1 - cIdx.second));
   }

   bool isMultiContourVertex( MultiContourIdx cIdx ) const
   {
      if (cIdx.first >= multiContour_.size() || cIdx.second >= multiContour_[cIdx.first].size())
         return false;

      return true;
   }

   MultiContourIdx cntIdx( size_t dIdx ) const
   {
      size_t c = 1;
      for (; c < cntSizes_.size(); ++c)
      {
         if (dIdx < cntSizes_[c])
            break;
      }

      return std::make_pair(c - 1, !attr_.outside ?
         (dIdx - cntSizes_[c - 1]) : (cntSizes_[c] - cntSizes_[c - 1] - 1 - (dIdx - cntSizes_[c - 1])));
   }

   void actualizeVertexState( size_t v )
   {
      if (dcel_.vertex(v).data.processed)
         return;

      dcel_.vertex(v).pos = vertexPosFixed(v);
      dcel_.vertex(v).data.t = t_;
   }

   template< class Stream >
      void dumpState( Stream &stream )
   {
      stream << step_ << std::endl;
      attr_.dump(stream);
      stream << std::endl;
      stream << std::setprecision(32) << intermediateOffset_ << ' ' << maxShiftEstimation_ << ' ' <<
         iterationStart_ << ' ' << t_ << ' ' << cantContinue_ << ' ' << numIterations_ << std::endl;
      stream << std::setprecision(32) << basePoint_.x << ' ' << basePoint_.y << std::endl;
      ed_.dump(stream);
      stream << std::endl;

      stream << cntSizes_.size() << ' ';
      for (size_t i = 0; i < cntSizes_.size(); ++i)
         stream << cntSizes_[i] << ' ';
      stream << std::endl << std::endl;

      stream << correspondence_.size() << ' ';
      for (size_t i = 0; i < correspondence_.size(); ++i)
      {
         stream << correspondence_[i].size() << ' ';
         for (size_t j = 0; j < correspondence_[i].size(); ++j)
         {
            correspondence_[i][j].dump(stream);
            stream << ' ';
         }
      }
      stream << std::endl << std::endl;

      stream << queue_.size() << ' ';
      for (ss::EventQueue::iterator qIt = queue_.begin(); qIt != queue_.end(); ++qIt)
      {
         qIt->dump(stream);
         stream << ' ';
      }
      stream << std::endl << std::endl;

      dcel_.dump(stream);
      stream << std::endl;

      if (grid_)
         grid_->dump(stream);
   }

   template< class Stream >
      void restore( Stream &stream )
   {
      stream >> step_;
      attr_.restore(stream);
      stream >> intermediateOffset_;
      stream >> maxShiftEstimation_;
      stream >> iterationStart_;
      stream >> t_;
      stream >> cantContinue_;
      stream >> numIterations_;
      stream >> basePoint_.x;
      stream >> basePoint_.y;
      ed_.restore(stream);

      size_t cntSizesSize;
      stream >> cntSizesSize;
      cntSizes_.reserve(cntSizesSize);
      for (size_t i = 0; i < cntSizesSize; ++i)
      {
         size_t val;
         stream >> val;
         cntSizes_.push_back(val);
      }

      size_t corrSize;
      stream >> corrSize;
      correspondence_.resize(corrSize);
      for (size_t i = 0; i < corrSize; ++i)
      {
         size_t corrSubSize;
         stream >> corrSubSize;
         correspondence_[i].resize(corrSubSize);
         for (size_t j = 0; j < corrSubSize; ++j)
            correspondence_[i][j].restore(stream);
      }

      size_t queueSize;
      stream >> queueSize;
      for (size_t i = 0; i < queueSize; ++i)
      {
         ss::EventData ev;
         ev.restore(stream);
         queue_.insert(queue_.end(), ev);
      }

      dcel_.restore(stream);
      for (size_t v = 0; v < dcel_.verticesSize(); ++v)
         dcel_.vertex(v).data.iter = queue_.end();

      for (ss::queue_iterator qIt = queue_.begin(); qIt != queue_.end(); ++qIt)
      {
         Assert(dcel_.vertex(qIt->eventHost).data.iter == queue_.end());
         dcel_.vertex(qIt->eventHost).data.iter = qIt;
      }

      if (attr_.optimize)
         grid_ = new SkeletonGrid (stream, *this);
   }

private:
   void stopVertexUpdate( size_t v )
   {
      dcel_.vertex(v).pos = vertexPosFixed(v);
      dcel_.vertex(v).data.t = t_;
      dcel_.vertex(v).data.bisector = cg::point_2 ();
      if (!dcel_.vertex(v).data.processed && !queue_.empty() && dcel_.vertex(v).data.iter != queue_.end())
         queue_.erase(dcel_.vertex(v).data.iter);
      dcel_.vertex(v).data.processed = true;
      dcel_.vertex(v).data.iter = queue_.end();
   }

   bool isVertexReflex( size_t prevE, size_t nextE )
   {
      cg::point_2 const &localCenter (dcel_.vertex(dcel_.edge(prevE).vertexDestination).pos);
      cg::point_2 const &v2 = vertexPos(dcel_.edge(prevE).vertexDestination, localCenter);
      return ((vertexPos(dcel_.edge(nextE).vertexDestination, localCenter) - v2) ^
         (v2 - vertexPos(dcel_.edge(prevE).vertexOrigin, localCenter))) >= 0;
   }

   bool isVertexSingular( size_t prevE, size_t nextE )
   {
      return cg::robust_collinear(vertexPosFixed(dcel_.edge(prevE).vertexOrigin),
         vertexPosFixed(dcel_.edge(prevE).vertexDestination),
         vertexPosFixed(dcel_.edge(nextE).vertexDestination));
   }

// For debug purposes
public:
   ss::EventData const &nextStepEventData()
   {
      return ed_;
   }

   std::vector< ss::EventData > const &prevStepAddedEvents()
   {
      return newEvents_;
   }

   ss::EventQueue const &queue()
   {
      return queue_;
   }

   size_t stepInfo()
   {
      return step_;
   }

   size_t numIterations()
   {
      return numIterations_;
   }

// Data section
private:
   MultiContourT multiContour_;
   ss::ConstructionParams attr_;
   double intermediateOffset_;

   ss::EventQueue queue_;

   std::vector< size_t > cntSizes_;

   double t_, iterationStart_, maxShiftEstimation_;
   ss::EventData ed_;
   bool cantContinue_;

   // Data for optimized mode
   shared_ptr< SkeletonGrid > grid_;

   // Output data
   cg::point_2 basePoint_;
   DCEL dcel_;
   MultiContourCorrespondenceData correspondence_;

   // for debug purposes only
   std::vector< ss::EventData > newEvents_;
   size_t step_;
   size_t numIterations_;

#ifdef SS_DEBUG_LOG
   std::set< size_t > verticesOfInterest_;
   m_ptr< std::ofstream > debugLog_;
#endif /* SS_DEBUG_LOG */
};
   
} // End of 'cg' namespace

#define SKELETON_EXT_METHOD(ret_type) \
   template< class VertexBuffer, class MultiContourRandomIterator > \
   ret_type StraightSkeletonGenerator< VertexBuffer, MultiContourRandomIterator >::

#include "Impl\update_queue.inl"
#include "Impl\straight_skeleton_edge_event.inl"
#include "Impl\straight_skeleton_split_event.inl"
#include "Impl\straight_skeleton_dcel.inl"

#undef SKELETON_EXT_METHOD
#undef SS_DEBUG_LOG
