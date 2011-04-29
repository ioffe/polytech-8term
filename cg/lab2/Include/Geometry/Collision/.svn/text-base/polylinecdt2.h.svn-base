#pragma once

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <vector>
#include <set>
#include <functional>

#include "Geometry/primitives.h"
#include "Geometry/grid2L.h"
#include "Geometry/Grid2L/subdiv.h"
#include "Geometry/clockwise.h"
#include "Geometry/Empty.h"
#include "Geometry/point_ops.h"

namespace cg 
{
namespace cdt         
{

#pragma pack(push, 1)

template< class P, class T >
struct PolylineCDT2StdTraits
{
   // Set of pairs (polyline identifier, segment index).
   typedef std::set< std::pair< size_t, size_t > > small_cell_type;

   typedef cg::Grid2L< small_cell_type >           grid_type;
   typedef boost::shared_ptr< grid_type >          grid_ptr_type;

   // Map: polyline identifier -> polyline.
   typedef std::map< size_t, P >                   map_type;

   // Vector of vertices for polyline.
   typedef std::vector< T >                        vector_type;
};

namespace polyline_accept_filter
{
   typedef boost::function<bool (size_t, size_t)> filter_type;

   struct AllFilter
   {
      bool operator () ( size_t polylineId, size_t segmentIdx ) const
      {
         return true;
      }
   };
   
   filter_type const all = AllFilter();

   struct IgnorePolylineFilter
   {
      IgnorePolylineFilter( size_t polylineId )
         : polylineId_( polylineId )
      {}

      bool operator () ( size_t polylineId, size_t segmentIdx ) const
      {
         return polylineId_ != polylineId;
      }

   private:
      size_t const polylineId_;
   };

   // Logic operators for polyline filters.
   namespace operators
   {
      inline filter_type logical_not( filter_type const &a )
      {
         return boost::bind<bool>(std::logical_not<bool>(), boost::bind<bool>(a, _1, _2));
      }

      inline filter_type logical_and( filter_type const &a, filter_type const &b )
      {
         return boost::bind<bool>(std::logical_and<bool>(), boost::bind<bool>(a, _1, _2), boost::bind<bool>(b, _1, _2));
      }

      inline filter_type logical_or( filter_type const &a, filter_type const &b )
      {
         return boost::bind<bool>(std::logical_or<bool>(), boost::bind<bool>(a, _1, _2), boost::bind<bool>(b, _1, _2));
      }
   } // End of 'operators' namespace
} // End of 'polyline_accept_filter' namespace

namespace
{
   // Identifies is ray intersected obstacle as ray start point is outside obstacle or as it inside obstacle.
   bool isPointsInside( segment_2 const &ray, segment_2 const &intersectingSegment, bool isPolylineCW )
   {
      point_2 const &start = ray.P0();

      if (intersectingSegment.contains(start))
         return true;
      else if (eq(intersectingSegment.P0(), intersectingSegment.P1()))
      {
         // Shooting ray intersects singular segment.
         if (eq(start, intersectingSegment.P0()) || eq(start, intersectingSegment.P1()))
            return true;
      }
      else
      {
         Assert(!eq_zero(normal(intersectingSegment)));
         Assert(!eq_zero(direction(ray)));

         double const scalarProduct = normal(intersectingSegment) * direction(ray);

         if (cg::eq_zero(scalarProduct))
         {
            // Shoot ray and intersected segment lies on the same line.
            if (intersectingSegment.contains(start))
               return true;
         }
         else if (isPolylineCW ^ (scalarProduct < 0))
            return true;
      }

      return false;
   }
} // End of anonymous namespace

// Simple class for shoot-raying in dynamic polylines space.
template< class PolylineAttr, class VertexAttr = Empty, template<class, class> class Traits = PolylineCDT2StdTraits >
class PolylineCDT2
{
   struct Polyline;

   struct Vertex : public point_2
   {
      Vertex() {}
      Vertex( point_2 const &point )
         : point_2(point)
      {}

      VertexAttr  attr;

      // Serialization
      template< class Stream >
      void serialize( Stream &stream ) const
      {
         write(stream, *static_cast<point_2 const *>(this));
         write(stream, attr);
      }
   };

public:  
   typedef Traits<Polyline, Vertex>              traits_type;

   typedef typename traits_type::small_cell_type small_cell_type;
   typedef typename traits_type::grid_type       grid_type;

   typedef PolylineAttr polyline_attr_type;
   typedef VertexAttr   vertex_attr_type;

private:
   struct Polyline
   {
      typedef typename traits_type::vector_type points_type;

      // Polyline vertices.
      points_type  vertices;
      // Is polyline closed (contour).
      bool         closed;
      // Polyline additional attributes.
      PolylineAttr attr;

      Polyline( PolylineAttr const &newAttr, points_type const &newVertices, bool newClosed = false ) 
         : vertices(newVertices)
         , closed  (newClosed)
         , attr    (newAttr)
      {
      }

      // Serialization
      template<class Stream>
         void serialize( Stream &stream ) const
      {
         write(stream, vertices);
         write(stream, closed);
         write(stream, attr);
      }
   };

private:
   void registerPolyline( size_t polylineId )
   {
      for (size_t i = 0, count = getSegmentsCount(polylineId); i < count; i++)
         cg::visit(*grid_, getSegment(polylineId, i), RegisterSegmentVisitor(polylineId, i));
   }

   void unregisterPolyline( size_t polylineId )
   {
      for (size_t i = 0, count = getSegmentsCount(polylineId); i < count; i++)
         cg::visit(*grid_, getSegment(polylineId, i), UnregisterSegmentVisitor(polylineId, i));
   }

public:
   PolylineCDT2() 
      : numSegments_   (0)
      , nextPolylineId_(0)
   {
   }

   // Create polyline CDT with fixed grid.
   PolylineCDT2( point_2 const &org, point_2 const &unit, point_2i const &ext ) 
      : numSegments_   (0)
      , nextPolylineId_(0)
   {
      grid_.reset(new grid_type(org, unit, ext));
   }

   void buildGrid( cg::rectangle_2 const &bbox, int avgItemsInBCell, int avgItemsInSCell, int maxBCellSubdiv )
   {
      cg::Grid2LSubdiv subdiv(bbox, avgItemsInBCell, avgItemsInSCell, maxBCellSubdiv);

      grid_.reset(new grid_type(subdiv.getMainSubdiv(numSegments_)));
      cg::HitCounter hitcounter(*grid_);

      for (polyline_map_type::const_iterator polylineIt = polylines_.begin(); polylineIt != polylines_.end(); ++polylineIt)
      {
         Polyline const &polyline = polylineIt->second;
         for (size_t i = 0; i < polyline.vertices.size() - 1; ++i)
            hitcounter.add(getSegment(polylineIt->first, i));
      }

      subdiv.makeSubdivision(hitcounter, *grid_);

      for (polyline_map_type::const_iterator polylineIt = polylines_.begin(); polylineIt != polylines_.end(); ++polylineIt)
         registerPolyline(polylineIt->first);
   }

private:
   template< class VertexIterator >
   size_t addPolylineToStorage( VertexIterator vertexBegin, VertexIterator vertexEnd,
                                bool closed, PolylineAttr const &attr )
   {
      Polyline::points_type vertices;
      point_2 prevPoint;
      for (VertexIterator it = vertexBegin; it != vertexEnd; ++it)
      {
         point_2 const &pnt = *it;

         if (it != vertexBegin && cg::eq(prevPoint, pnt))
            continue;
         prevPoint = pnt;

         vertices.push_back(Vertex(pnt));
         bounding_ |= pnt;
      }

      Assert((int)vertices.size() >= 2 + (closed ? 1 : 0));

      if (closed)
         numSegments_ += vertices.size();
      else
         numSegments_ += vertices.size() - 1;

      size_t polylineId = nextPolylineId_++;
      polylines_.insert(std::make_pair(polylineId, Polyline(attr, vertices, closed)));

      return polylineId;
   }

   void removePolylineFromStorage( size_t polylineId )
   {
      polyline_map_type::iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      polylines_.erase(it);
   }

public:
   // Returns added polyline identifier
   template< class VertexIterator >
   size_t addPolyline( VertexIterator vertexBegin, VertexIterator vertexEnd, bool closed, PolylineAttr const &attr, bool shouldRegister = true )
   {
      size_t polylineId = addPolylineToStorage(vertexBegin, vertexEnd, closed, attr);
      
      if (shouldRegister)
         registerPolyline(polylineId);

      return polylineId;
   }

   // Removes polyline by identifier.
   void removePolyline( size_t polylineId )
   {
      unregisterPolyline(polylineId);
      removePolylineFromStorage(polylineId);
   }

   bool shootRayFirst( segment_2 const &segment, polyline_accept_filter::filter_type const &filter, 
                       double *ratio = NULL, size_t *polylineId = NULL, size_t *segmentIdx = NULL,
                       bool isPointsOnPolylineBorderCounts = true, bool cw = false ) const
   {
      ShootRayFirstProcessor<PolylineCDT2> proc(*this, segment, filter, NULL, isPointsOnPolylineBorderCounts, cw);
      cg::visit(*grid_, segment, proc);
      if (proc.hasIntersection())
      {
         if (ratio != NULL)
            *ratio = proc.ratio();
         if (polylineId != NULL)
            *polylineId = proc.polylineId();
         if (segmentIdx != NULL)
            *segmentIdx = proc.segmentIdx();

         return true;
      }
      else
         return false;
   }

   // Shoots ray up till first unfiltered polyline. Then checks clockwise order.
   // Obstacles must be oriented 'cw', holes - counter 'cw'. 
   // Obstacles must not intersect each other and be not self intersecting.
   // Filter should filter full polylines.
   // Example:
   //   All houses, lakes, fences and world working bounding rectangle ordered CCW.
   //   All coastlines, holes in houses ordered CW.
   //   All polylines not intersecting with each other and not self intersecting.
   //   Then findPolylinePointBelongsTo() will find identifier of house, lake, fence or world working bounding rectangle, 
   //     if point inside one of them, or -1, if point is in obstacle free space.
   size_t findPolylinePointBelongsTo( point_2 const &point, polyline_accept_filter::filter_type const &filter, 
                                      bool cw = false, bool isPointOnBorderCounts = false ) const
   {
      if (!cg::bounding(getGrid()).contains(point))
         return -1;

      segment_2 const ray(point, point_2(point.x, cg::bounding(getGrid()).y.hi()));
      PolylinePointBelongsToProcessor<PolylineCDT2> proc(*this, ray, filter, isPointOnBorderCounts, cw);
      cg::visit(*grid_, ray, proc);
      return proc.resultPolylineId();
   }

private:
   double verticalDistToPolylineCorner( point_2 const &point, polyline_accept_filter::filter_type const &filter, size_t polylineId ) const
   {
      if (polylineId == -1)
         return false;
      if (!bounding().contains(point))
         return false;
      if (!getPolylineBounding(polylineId).contains(point))
         return false;

      polyline_accept_filter::filter_type actualFilter;
      // Filter that accepts polyline from argument.
      actualFilter = polyline_accept_filter::operators::logical_not(polyline_accept_filter::IgnorePolylineFilter(polylineId));

      segment_2 const ray(point, point_2(point.x, bounding().y.hi()));

      double ratio;
      size_t isectPolylineId, isectSegmentId;
      if (shootRayFirst(ray, actualFilter, &ratio, &isectPolylineId, &isectSegmentId))
      {
         Assert(isectPolylineId == polylineId);

         point_2   const isectPoint    = ray(ratio);
         segment_2 const &isectSegment = getSegment(isectPolylineId, isectSegmentId);
         bool      const cw = isPolylineCWOrdered(isectPolylineId);

         if (isPointsInside(ray, isectSegment, cw))
            return distance(point, isectPoint);
      }
      
      return -1;
   }

public:
   bool isPointInPolyline( point_2 const &point, polyline_accept_filter::filter_type const &filter, size_t polylineId, bool canPointLieOnPolyline = false ) const
   {
      double const dist = verticalDistToPolylineCorner(point, filter, polylineId);

      if (dist >= 0)
      {
         if (!canPointLieOnPolyline && eq_zero(dist))
            return false;
         else
            return true;
      }
      else
         return false;
   }

   bool isPointOnPolylineBorder( point_2 const &point, polyline_accept_filter::filter_type const &filter, size_t polylineId ) const
   {
      double const dist = verticalDistToPolylineCorner(point, filter, polylineId);

      return eq_zero(dist);
   }

public:
   PolylineAttr const& getPolylineAttr( size_t polylineId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      return it->second.attr;
   }

   VertexAttr const& getVertexAttr( size_t polylineId, size_t vertexIdx ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      Assert(vertexIdx < it->second.vertices.size());
      return it->second.vertices[vertexIdx].attr;
   }

   void setVertexAttr( size_t polylineId, size_t vertexIdx, VertexAttr const &attr )
   {
      polyline_map_type::iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      Assert(vertexIdx < it->second.vertices.size());
      it->second.vertices[vertexIdx].attr = attr;
   }

   bool isPolylineClosed( size_t polylineId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      return it->second.closed;
   }

   bool isPolylineCWOrdered( size_t polylineId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      return cg::clockwise_ordered(it->second.vertices.begin(), it->second.vertices.end());
   }

   rectangle_2 getPolylineBounding( size_t polylineId ) const
   {
      rectangle_2 bbox;
      // TODO: Can be preprocessed and stored.
      for (size_t vertexIdx = 0, verticesCount = getVerticesCount(polylineId); vertexIdx < verticesCount; ++vertexIdx)
         bbox |= getVertex(polylineId, vertexIdx);

      return bbox;
   }

   size_t getVerticesCount( size_t polylineId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      return it->second.vertices.size();
   }

   point_2 getVertex( size_t polylineId, size_t vertexIdx ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      Assert(vertexIdx < it->second.vertices.size());
      return it->second.vertices[vertexIdx];
   }

   size_t getSegmentsCount( size_t polylineId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      return it->second.closed ? it->second.vertices.size() : it->second.vertices.size() - 1;
   }

   segment_2 getSegment( size_t polylineId, size_t segmentIdx ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());

      if (it->second.closed)
      {
         Assert(segmentIdx < it->second.vertices.size());
         return segment_2(it->second.vertices[segmentIdx], it->second.vertices[polylineId, (segmentIdx + 1) % it->second.vertices.size()]);
      }
      else
      {
         Assert(segmentIdx + 1 < it->second.vertices.size());
         return segment_2(it->second.vertices[segmentIdx], it->second.vertices[segmentIdx + 1]);
      }
   }

   // Retrieves next segment in polyline identifier.
   // Closed polylines works correctly.
   size_t getAdjacentSegment( size_t polylineId, size_t segmentIdx, bool next ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      Polyline const &polyline = it->second;

      size_t limit = polyline.vertices.size() - (polyline.closed ? 1 : 2);

      if (next)
         return segmentIdx < limit ? segmentIdx + 1 : 0;
      else
         return segmentIdx > 0 ? segmentIdx - 1 : limit;
   }

   // Polylines iteration
public:   
   size_t getPolylinesCount() const
   {
      return polylines_.size();
   }

   size_t getPolylineId( size_t idx ) const
   {
      polyline_map_type::const_iterator it = polylines_.begin();
      std::advance(it, idx);
      return it->first;
   }

private:
   struct RegisterSegmentVisitor
   {
      RegisterSegmentVisitor( size_t polylineId, size_t segmentIdx ) 
         : polylineId_( polylineId )
         , segmentId_ ( segmentIdx )
      {}

      // TODO: Use cg::grid2l_visitor_base
      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type &bcell )
      {
         Assert(bcell);
         return false;
      }

      template< class State >
      bool operator()( State &state, small_cell_type &cell )
      {
         cell.insert(std::make_pair(polylineId_, segmentId_));
         return false;
      }

   private:
      size_t const segmentId_;
      size_t const polylineId_;
   };

   struct UnregisterSegmentVisitor
   {
      UnregisterSegmentVisitor( size_t polylineId, size_t segmentIdx ) 
         : polylineId_( polylineId )
         , segmentId_ ( segmentIdx )
      {
      }

      // TODO: Use cg::grid2l_visitor_base
      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type &bcell )
      {
         Assert(bcell);
         return false;
      }

      template< class State >
      bool operator()( State &state, small_cell_type &cell )
      {
         cell.erase(std::make_pair(polylineId_, segmentId_));
         return false;
      }

   private:
      size_t const segmentId_;
      size_t const polylineId_;
   };

   template< class CDT >
   struct ShootRayFirstProcessor
   {
      ShootRayFirstProcessor( CDT const &cdt, segment_2 const &ray, polyline_accept_filter::filter_type const &filter, point_2 const* byNormal = NULL,
                              bool isPointsOnPolylineBorderCounts = true, bool cw = false ) 
         : cdt_            ( cdt )
         , filter_         ( filter )
         , ray_            ( ray )
         , byNormal_       ( byNormal ) 
      
         , isPointsOnPolylineBorderCounts_( isPointsOnPolylineBorderCounts )
         , cw_                            ( cw )

         , hasIntersection_( false )
         , ratio_          ( std::numeric_limits<double>::max() )
         , polylineId_     ( -1 )
         , segmentId_      ( -1 )
      {
         cg::cull(ray, cg::bounding(cdt.getGrid()), clippedRayByGrid_);
      }

      bool        hasIntersection      () const { return hasIntersection_; }
      double      ratio                () const { return ratio_;           }
      size_t      segmentIdx           () const { return segmentId_;       }
      size_t      polylineId           () const { return polylineId_;      }

      // TODO: Use cg::grid2l_visitor_base
      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type & )
      {
         return false;
      }

      template< class State >
      bool operator()( State &state, small_cell_type const &cell )
      {
         // Clipping intersecting ray by current cell;
         segment_2 clippedRayByCell(clippedRayByGrid_(state.in_ratio), clippedRayByGrid_(state.out_ratio));

         if (eq(ray_.P0(), ray_.P1()) && !isPointsOnPolylineBorderCounts_)
         {
            // Singular ray can't intersect any obstacle not by laying on it.
            return true;
         }

         // Intersecting clipped ray.
         for (small_cell_type::const_iterator it = cell.begin(); it != cell.end(); ++it)
         {
            size_t const &polylineId = it->first;
            size_t const &segmentIdx = it->second;

            point_2 const &start = ray_.P0();

            if (!filter_(polylineId, segmentIdx))
               continue;

            if (byNormal_)
            {
               segment_2 const &seg = cdt_.getSegment(polylineId, segmentIdx);
               if (!cg::ge(normal(seg) * (*byNormal_), 0))
                  continue;
            }

            point_2 isectPoint;
            segment_2 const segment(cdt_.getSegment(polylineId, segmentIdx));
            size_t intersectedSegmentIdx = segmentIdx;
            if (cg::generic_intersection(clippedRayByCell, segment, &isectPoint, (point_2 *)NULL) != cg::disjoint)
            {
               // FIXME! If (isPointsOnPolylineBorderCounts_ == true) need to handle case:
               // E *--* D
               //     /
               //    /
               // C *--* B
               //     /
               //    /
               // A *
               // In this sample ray [AE] must not intersect point C.

               if (!isPointsOnPolylineBorderCounts_ && eq(isectPoint, clippedRayByCell.P0()) && segment.contains(start))
               {
                  // 'isPointsOnPolylineBorderCounts_' implies that obstacles are closed.
                  Assert(cdt_.isPolylineClosed(polylineId));
                  // If this case will rise up, this should be implemented.
                  Assert(!eq(segment.P0(), segment.P1()));

                  bool rayLiesInVertex = false;
                  segment_2 s0,    s1;
                  size_t    s0Idx, s1Idx;
                  if (eq(clippedRayByCell.P0(), segment.P0()))
                  {
                     rayLiesInVertex = true;
                     s0Idx = cdt_.getAdjacentSegment(polylineId, segmentIdx, false);
                     s0    = cdt_.getSegment(polylineId, s0Idx);
                     s1Idx = segmentIdx;
                     s1    = segment;
                  }
                  if (eq(clippedRayByCell.P0(), segment.P1()))
                  {
                     rayLiesInVertex = true;
                     s0Idx = segmentIdx;
                     s0    = segment;
                     s1Idx = cdt_.getAdjacentSegment(polylineId, segmentIdx, true);
                     s1    = cdt_.getSegment(polylineId, s1Idx);
                  }

                  if (rayLiesInVertex)
                  {
                     // Ray start point lies in common point of two adjacent polylines segments.

                     Assert(!eq(s0.P0(), s0.P1())); // TODO: This cases can be implemented.
                     Assert(!eq(s1.P0(), s1.P1()));
                     Assert(!(cg::eq_zero(direction(s0) ^ direction(s1)) && direction(s0) * direction(s1) > 0));

                     if ((cg::eq_zero(direction(ray_) ^ direction(s0)) && ((direction(ray_) * -direction(s0) > 0) ^ cw_)) ||
                         (cg::eq_zero(direction(ray_) ^ direction(s1)) && ((direction(ray_) *  direction(s1) > 0) ^ cw_)))
                     {
                        // Ray lies on polyline border.
                        continue;
                     }

                     bool inFreeSpace; // Deduced for cw == false
                     if (cg::eq_zero(direction(s0) ^ direction(s1)))
                     {
                        // |
                        // * free
                        // |

                        // Implies, that polyline angle is not singular.
                        inFreeSpace = cg::right_turn_strict(start, s1.P1(), ray_.P1());
                     }
                     else if (cg::right_turn_strict(start, s1.P1(), s0.P0()))
                     {
                        //   /
                        //  * free
                        //  |
                        inFreeSpace = (cg::right_turn_strict(start, s1.P1(), ray_.P1()) && cg::left_turn_strict(start, s0.P0(), ray_.P1()));
                     }
                     else
                     {
                        // \
                        //  * free
                        //  |
                        inFreeSpace = !(cg::left_turn_strict(start, s1.P1(), ray_.P1()) && cg::right_turn_strict(start, s0.P0(), ray_.P1()));
                     }

                     if (cw_ ^ inFreeSpace)
                        continue;
                     else
                     {
                        // Choosing more looks like intersecting segment.
                        double const angle0 = cg::norm_pi(cg::angle(normal(s0) * (cw_ ? -1 : 1), direction(ray_)));
                        double const angle1 = cg::norm_pi(cg::angle(normal(s1) * (cw_ ? -1 : 1), direction(ray_)));
                        if (abs(angle0) < abs(angle1))
                           intersectedSegmentIdx = s0Idx;
                        else
                           intersectedSegmentIdx = s1Idx;
                     }
                  }
                  else
                  {
                     if (cg::eq_zero(direction(ray_) ^ direction(segment)))
                     {
                        // Ray lies on polyline border.
                        continue;
                     }

                     if (!isPointsInside(opposite(ray_), segment, cw_))
                     {
                        // Ray start lies on some segment, but ray end is directed outside obstacle.
                        continue;
                     }
                  }
               }

               double const ratio = ray_(isectPoint);
               if (ratio < ratio_)
               {
                  hasIntersection_ = true;
                  ratio_           = ratio;
                  polylineId_      = polylineId;
                  segmentId_       = intersectedSegmentIdx;
               }
            }
         }

         return hasIntersection_;
      }

   private:
      CDT       const &cdt_;
      segment_2 const ray_;
      segment_2       clippedRayByGrid_;
      point_2   const *byNormal_;

      bool      const isPointsOnPolylineBorderCounts_;
      bool      const cw_;

      polyline_accept_filter::filter_type const &filter_;

      bool            hasIntersection_;
      double          ratio_;
      size_t          polylineId_;
      size_t          segmentId_;
   };

   template< class CDT >
   struct PolylinePointBelongsToProcessor
   {
      // Ray should begin in interesting point and ends in eternity.
      // Note: ray should be clipped to CDT bounding.
      // Obstacles ordered 'cw'.
      PolylinePointBelongsToProcessor( CDT const &cdt, segment_2 const &ray, polyline_accept_filter::filter_type const &filter, bool canLieOnPolyline, bool cw ) 
         : cdt_              ( cdt )
         , canLieOnPolyline_ ( canLieOnPolyline )
         , cw_               ( cw )
         , filter_           ( filter )
         , ray_              ( ray )
         , resultPolylineId_ ( -1 )
         , foundIntersection_( false )
      {
         Assert(cg::bounding(cdt.getGrid()).contains(ray.P0()) && cg::bounding(cdt.getGrid()).contains(ray.P1()));
      }

      size_t resultPolylineId() const { return resultPolylineId_; }

      // TODO: Use cg::grid2l_visitor_base
      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type & )
      {
         return false;
      }

      template< class State >
      bool operator()( State &state, small_cell_type const& cell )
      {
         // Clipping intersecting ray by current cell.
         segment_2 const clippedRay(ray_(state.in_ratio), ray_(state.out_ratio));
         point_2   const start = ray_.P0();

         if (!foundIntersection_)
         {
            // Still no intersection.

            double closestDist = std::numeric_limits<double>::max();
            size_t closestPolylineId(-1), closestSegmentIdx(-1);
            segment_2 closestSegment;

            // Searching segment that intersects clipped ray closest to shooting ray start point.
            for (small_cell_type::const_iterator it = cell.begin(); it != cell.end(); ++it)
            {
               size_t const polylineId = it->first;
               size_t const segmentIdx = it->second;

               if (!filter_(polylineId, segmentIdx))
                  continue;

               Assert(cdt_.isPolylineClosed(polylineId));

               segment_2 const segment = cdt_.getSegment(polylineId, segmentIdx);

               point_2 isectPoint;
               if (cg::generic_intersection(clippedRay, segment, &isectPoint, (point_2 *)NULL) != cg::disjoint)
               {
                  double const dist = distance(start, isectPoint);

                  bool isSegmentCloser = false;
                  if (cg::eq(dist, closestDist))
                  {
                     // Intersection in common point of two adjacent segments.
                     Assert(eq(closestSegment.P0(), segment.P0()) || eq(closestSegment.P0(), segment.P1()) ||
                            eq(closestSegment.P1(), segment.P0()) || eq(closestSegment.P1(), segment.P1()));

                     double const d1 = distance_sqr(start, (closestSegment.P0() + closestSegment.P1()) / 2.0);
                     double const d2 = distance_sqr(start, (segment.P0()        + segment.P1()       ) / 2.0);
                     if (d2 < d1)
                        isSegmentCloser = true;
                  }
                  else if (dist < closestDist)
                     isSegmentCloser = true;

                  if (isSegmentCloser)
                  {
                     closestDist       = dist;
                     closestPolylineId = it->first;
                     closestSegmentIdx = it->second;
                     closestSegment    = segment;
                  }
               }
            }

            if (closestPolylineId != -1)
            {
               foundIntersection_ = true;

               if (isPointsInside(ray_, closestSegment, cw_))
               {
                  // Ray start inside some polyline.

                  if (closestSegment.contains(start) && !canLieOnPolyline_)
                  {
                     // Don't count that point lies inside polyline, if it lies on polyline corner.
                     return true;
                  }

                  if (cdt_.isPolylineCWOrdered(closestPolylineId) == cw_)
                  {
                     // Intersected with not hole-like polyline.
                     resultPolylineId_ = closestPolylineId;
                     return true;
                  }
                  else
                  {
                     // Intersected with hole polyline.
                     rayInsideHoleWithId_ = closestPolylineId;
                     intersectionDist_    = closestDist;
                  }
               }
               else
               {
                  // Ray start is not inside some polyline.
                  return true;
               }
            }
            else if (eq(ray_.P0(), ray_.P1()))
            {
               // Shooting ray is singular and not intersected any segment in first cell.
               return true;
            }
         }

         if (foundIntersection_)
         {
            // There was intersection with polyline hole, searching polyline containing this hole.

            // Reflection: distance from point to intersecting segment -> intersecting segment identifier.
            typedef std::map<double, std::pair< size_t, size_t > > SegmentsReflectionT;
            SegmentsReflectionT segments;

            // Storing all intersecting segments in map.
            for (small_cell_type::const_iterator it = cell.begin(); it != cell.end(); ++it)
            {
               size_t const polylineId = it->first;
               size_t const segmentIdx = it->second;

               if (!filter_(polylineId, segmentIdx))
                  continue;

               Assert(cdt_.isPolylineClosed(polylineId));

               segment_2 const segment = cdt_.getSegment(polylineId, segmentIdx);

               point_2 isectPoint;
               if (cg::generic_intersection(clippedRay, segment, &isectPoint, (point_2 *)NULL) != cg::disjoint)
               {
                  double const dist = ray_(isectPoint);
                  if (dist > intersectionDist_)
                     segments.insert(std::make_pair(dist, *it));
               }
            }

            // Processing all intersected segments in order of intersecting.
            for (SegmentsReflectionT::const_iterator it = segments.begin(); it != segments.end(); ++it)
            {
               size_t segmentIdx  = it->second.second;
               size_t polylineId = it->second.first;

               if (rayInsideHoleWithId_ != -1 && polylineId != rayInsideHoleWithId_)
               {
                  // Intersection inside hole.
                  continue;
               }

               if (rayInsideHoleWithId_ == polylineId)
               {
                  // Ray goes outside from hole.
                  rayInsideHoleWithId_ = -1;
                  continue;
               }

               if (cdt_.isPolylineCWOrdered(polylineId) == cw_)
               {
                  // Found intersection with not hole-like polyline.
                  resultPolylineId_ = polylineId;
                  return true;
               }
               else
               {
                  // Ray goes inside hole.
                  rayInsideHoleWithId_ = polylineId;
               }
            }
         }

         return false;
      }

   private:
      CDT             const &cdt_;
      segment_2       const ray_;
      bool            const cw_;
      bool            const canLieOnPolyline_;
      polyline_accept_filter::filter_type const &filter_;

      bool                  foundIntersection_;
      size_t                rayInsideHoleWithId_;
      double                intersectionDist_;
      
      size_t                resultPolylineId_;
   };

public:
   grid_type const& getGrid() const
   {
      return *grid_;
   }

private:
   template< class CDT, class PolylinesIdsOutputIterator >
   struct GetAllPolylinesInRectangleVisitor 
      : cg::grid2l_visitor_base<typename CDT::grid_type, GetAllPolylinesInRectangleVisitor<CDT, PolylinesIdsOutputIterator> >
   {
      typedef PolylinesIdsOutputIterator polylines_ids_output_iterator_type;

      GetAllPolylinesInRectangleVisitor( CDT const &cdt, cg::rectangle_2 const &rect, polylines_ids_output_iterator_type idsOut )
         : cdt_   (cdt)
         , rect_  (rect)
         , idsOut_(idsOut)
      {}

      polylines_ids_output_iterator_type getOuputIterator() const { return idsOut_; }

      template< class State, class Cell >
      bool operator () ( State const &state, Cell &cell )
      {
         // Cell is a set of pairs (polyline identifier, segment index).
         for (Cell::const_iterator it = cell.begin(); it != cell.end(); ++it)
         {
            size_t          const polylineId = it->first;

            if (outputtedPolylinesIds_.find(polylineId) != outputtedPolylinesIds_.end())
               continue;

            cg::rectangle_2 const polylineBB = cdt_.getPolylineBounding(polylineId);

            if (!(polylineBB & rect_).empty())
            {
               *idsOut_++ = polylineId;
               outputtedPolylinesIds_.insert(polylineId);
            }
         }
         
         return false;
      }

   private:
      CDT                          const &cdt_;
      cg::rectangle_2              const  rect_;
      polylines_ids_output_iterator_type  idsOut_;

      std::set<size_t>                    outputtedPolylinesIds_;
   };

public:
   template< class PolylineIdsOutputIterator >
   PolylineIdsOutputIterator getPolylinesInRect( cg::rectangle_2 const &rect, PolylineIdsOutputIterator idsOut ) const
   {
      GetAllPolylinesInRectangleVisitor<PolylineCDT2, PolylineIdsOutputIterator> proc(*this, rect, idsOut);
      cg::visit(getGrid(), rect, proc);
      return proc.getOuputIterator();
   }

public:
   // Returns all added polylines bounding.
   // Note: if some of polylines were removed this rectangle can be not equal to real bounding.
   cg::rectangle_2 bounding() const
   {
      return bounding_;
   }

   // Serialization.
public:
   template<class Stream>
      void serialize(Stream &stream) const
   {
      write(stream, polylines_);
      write(stream, numSegments_);
      write(stream, grid_.get());
      write(stream, bounding_);
      write(stream, nextPolylineId_);
   }

private:
   typedef typename traits_type::map_type polyline_map_type;

   // Polylines.
   polyline_map_type polylines_;
   size_t            numSegments_;

   // Shoot-ray grid.
   typedef typename traits_type::grid_ptr_type grid_ptr_type;
   grid_ptr_type grid_;

   // All added polylines bounding.
   cg::rectangle_2   bounding_;

private:
   // Next polyline id.
   size_t nextPolylineId_;

private:
   template<class Stream>
   friend void write( Stream &stream, Polyline const &p )
   {
      p.serialize(stream);
   }

   template<class Stream>
   friend void write( Stream &stream, Vertex const &v )
   {
      v.serialize(stream);
   }
};

#pragma pack(pop)

// Serialization.
template< class Stream, class PolylineAttr, class VertexAttr, template<class, class> class Traits >
void write( Stream &stream, PolylineCDT2< PolylineAttr, VertexAttr, Traits > const &cdt )
{
   cdt.serialize(stream);
}

} // end of namespace cdt
} // end of namespace cg
