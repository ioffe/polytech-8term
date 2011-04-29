#pragma once

#include <vector>
#include <set>
#include <queue>

#include "Geometry/primitives.h"
#include "Geometry/grid2L.h"
#include "Geometry/Grid2L/subdiv.h"
#include "Geometry/clockwise.h"
#include "Geometry/Empty.h"

namespace cg 
{
namespace cdt         
{

#pragma pack(push, 1)

template<class P, class T>
struct StdTraits
{
   typedef std::set< std::pair< size_t, size_t > >  small_cell_type;

   typedef cg::Grid2L< small_cell_type >            grid_type;
   typedef boost::shared_ptr< grid_type >           grid_ptr_type;

   typedef std::map< size_t, P >                    map_type;
   typedef std::vector< T >                         vector_type;
};

// Simple class for shoot-raying in dynamic polylines space.
template< class PolylineAttr, class VertexAttr = Empty, template<class, class> class Traits = StdTraits >
   class PolylineCDT
{
   struct Polyline;

   struct Vertex : public point_2
   {
      Vertex() {}
      Vertex( point_2 const &point )
         : point_2( point )
      {}

      VertexAttr  attr;

      // Serialization
      template<class Stream>
         void serialize(Stream &stream) const
      {
         write(stream, *static_cast<point_2 const *>(this));
         write(stream, attr);
      }
   };

public:  
   typedef Traits<Polyline, Vertex>              traits_type;

   typedef typename traits_type::small_cell_type small_cell_type;
   typedef typename traits_type::grid_type       grid_type;

private:
   struct Polyline
   {
      typedef typename traits_type::vector_type points_type;

      // Polyline vertices
      points_type  vertices;
      // Is polyline closed (contour) 
      bool         closed;
      // Polyline additional attributes
      PolylineAttr attr;

      Polyline( PolylineAttr newAttr, points_type const &newVertices, bool newClosed = false ) 
         : vertices(newVertices)
         , closed  (newClosed)
         , attr    (newAttr)
      {
      }

      // Serialization
      template<class Stream>
         void serialize(Stream &stream) const
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
   PolylineCDT() 
      : numSegments_   ( 0 )
      , nextPolylineId_( 0 )
   {
   }

   PolylineCDT( cg::rectangle_2 const &bbox ) 
      : numSegments_   ( 0 )
      , nextPolylineId_( 0 )
   {
      // Building grid
      buildEmptyGrid(bbox);
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
   void buildEmptyGrid( cg::rectangle_2 const &bbox )
   {
      grid_.reset(new grid_type(bbox.xy(), bbox.size() / point_2i(4, 4), point_2i(4, 4)));
   }

private:
   template< class VertexIterator >
      size_t addPolylineToStorage( VertexIterator vertexBegin, VertexIterator vertexEnd,
                                   bool closed, PolylineAttr const &attr)
   {
      Assert(std::distance(vertexBegin, vertexEnd) >= 2);

      Polyline::points_type vertices;
      for (; vertexBegin != vertexEnd; ++vertexBegin)
         vertices.push_back(Vertex(*vertexBegin));

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
      polylines_.erase(polylineId);
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

   bool shootRayFirst( segment_2 const &segment, size_t accept, double *ratio = NULL, size_t *polylineId = NULL, size_t *segmentId = NULL ) const
   {
      ShootRayFirstProcessor< PolylineCDT > proc(*this, segment, accept);
      if (cg::visit(*grid_, segment, proc))
      {
         if (ratio != NULL)
            *ratio = proc.ratio();
         if (polylineId != NULL)
            *polylineId = proc.polylineId();
         if (segmentId != NULL)
            *segmentId = proc.segmentId();

         return true;
      }
      else
         return false;
   }

   bool shootRayFirstByNormal( segment_2 const &segment, size_t accept, point_2 const& normal, 
                               double *ratio = NULL, size_t *polylineId = NULL, size_t *segmentId = NULL ) const
   {
      ShootRayFirstProcessor< PolylineCDT > proc(*this, segment, accept, &normal);
      if (cg::visit(*grid_, segment, proc))
      {
         if (ratio != NULL)
            *ratio = proc.ratio();
         if (polylineId != NULL)
            *polylineId = proc.polylineId();
         if (segmentId != NULL)
            *segmentId = proc.segmentId();

         return true;
      }
      else
         return false;
   }

   // Shoots ray up till first unfiltered polyline. Then checks clockwise order.
   // If collided polyline ordered as point is outside this polyline, than this polyline will be ignored.
   // Filter should filter full polylines!
   size_t findPolylinePointBelongsTo( point_2 const &point, size_t accept, bool cw = false ) const
   {
      rectangle_2 bb(cg::bounding(this->getGrid()));

      if (!bb.contains(point))
         return -1;

      segment_2 ray(point, point_2(point.x, bb.y.hi()));
      PolylinePointBelongsToProcessor< PolylineCDT > proc(*this, ray, accept, cw);
      cg::visit(*grid_, ray, proc);
      return proc.resultPolylineId();
   }

public:
   PolylineAttr const& getPolylineAttr( size_t polylineId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      return it->second.attr;
   }

   VertexAttr const& getVertexAttr( size_t polylineId, size_t vertexId ) const
   {
      polyline_map_type::const_iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      Assert(vertexId < it->second.vertices.size());
      return it->second.vertices[vertexId].attr;
   }

   void setVertexAttr( size_t polylineId, size_t vertexId, VertexAttr const &attr )
   {
      polyline_map_type::iterator it = polylines_.find(polylineId);
      Assert(it != polylines_.end());
      Assert(vertexId < it->second.vertices.size());
      it->second.vertices[vertexId].attr = attr;
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

   // Retrieves next segment in polyline identifier
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

   size_t getPolylineId(size_t idx) const
   {
      polyline_map_type::const_iterator it = polylines_.begin();
      std::advance(it, idx);
      return it->first;
   }

private:
   struct RegisterSegmentVisitor
   {
      RegisterSegmentVisitor( size_t polylineId, size_t segmentId ) 
         : polylineId_( polylineId )
         , segmentId_ ( segmentId )
      {}

      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type &bcell )
      {
         if (!bcell)
            bcell.subdivide(point_2i(20, 20));
         
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
      UnregisterSegmentVisitor( size_t polylineId, size_t segmentId ) 
         : polylineId_( polylineId )
         , segmentId_ ( segmentId )
      {
      }

      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type & )
      {
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
      ShootRayFirstProcessor( CDT const &cdt, segment_2 const &ray, size_t accept, point_2 const* byNormal = NULL ) 
         : cdt_            ( cdt )
         , hasIntersection_( false )
         , accept_         ( accept )
         , ray_            ( ray )
         , ratio_          ( 1e100 )
         , byNormal_       ( byNormal ) 
      {
         cg::cull(ray_, cg::bounding(cdt_.getGrid()), clippedRay_);
      }

      bool        hasIntersection      () const { return hasIntersection_; }
      double      ratio                () const { return ratio_; }
      size_t      segmentId            () const { return segmentId_; }
      size_t      polylineId           () const { return polylineId_; }

      template< class State, class bigcell_type >
         bool processbigcell( State const &, bigcell_type & )
      {
         return false;
      }

      template< class State >
         bool operator()( State &state, small_cell_type const &cell )
      {
         // Clipping intersecting ray by current cell;
         segment_2 clippedRay(clippedRay_(state.in_ratio), clippedRay_(state.out_ratio));

         // Intersecting clipped ray
         for (small_cell_type::const_iterator it = cell.begin(); it != cell.end(); ++it)
         {
            if (!(cdt_.getPolylineAttr(it->first).type & accept_))
               continue;

            if (byNormal_)
            {
               segment_2 seg = cdt_.getSegment(it->first, it->second);
               if (!cg::ge(normal(seg) * (*byNormal_), 0))
                  continue;
            }

            point_2 isect;
            if (cg::generic_intersection(clippedRay, cdt_.getSegment(it->first, it->second), &isect, &isect) != cg::disjoint)
            {
               double ratio = ray_(isect);
               if (!hasIntersection_ || ratio < ratio_)
               {
                  ratio_           = ratio;
                  polylineId_      = it->first;
                  segmentId_       = it->second;
                  hasIntersection_ = true;
               }
            }
         }

         return hasIntersection_;
      }

   private:
      CDT const      &cdt_;
      segment_2      ray_;
      segment_2      clippedRay_;
      point_2 const  *byNormal_;
      size_t         accept_;

      double        ratio_;
      size_t        polylineId_;
      size_t        segmentId_;
      bool          hasIntersection_;
   };

   template< class CDT >
      struct PolylinePointBelongsToProcessor
   {
      // Ray should begin in interesting point and ends in eternity
      // Note: ray should be clipped to CDT bounding.
      PolylinePointBelongsToProcessor( CDT const &cdt, segment_2 const &ray, size_t accept, bool cw ) 
         : cdt_             ( cdt )
         , cw_              ( cw )
         , accept_          ( accept )
         , resultPolylineId_( -1 )
         , ray_             ( ray )
      {}

      size_t resultPolylineId() const { return resultPolylineId_; }

      template< class State, class bigcell_type >
      bool processbigcell( State const &, bigcell_type & )
      {
         return false;
      }

      template< class State >
      bool operator()( State &state, small_cell_type const& cell )
      {
         if (ray_.P0() == ray_.P1())
            return false;

         // Clipping intersecting ray by current cell;
         segment_2 clippedRay(ray_(state.in_ratio), ray_(state.out_ratio));

         // Reflection: distance from point to intersecting segment -> intersecting segment identifier
         typedef std::map<double, std::pair< size_t, size_t > > SegmentsReflectionT;
         SegmentsReflectionT segments;

         // Storing all intersecting not ignoring segments in map
         for (small_cell_type::const_iterator it = cell.begin(); it != cell.end(); ++it)
         {
            if (!(cdt_.getPolylineAttr(it->first).type & accept_))
               continue;

            point_2 isect;
            if (cg::generic_intersection(clippedRay, cdt_.getSegment(it->first, it->second), &isect, &isect) != cg::disjoint &&
               outerPolylines_.find(it->first) == outerPolylines_.end())
            {
               segments.insert(std::make_pair(ray_(isect), *it));
            }
         }

         // Processing all intersected segments in order of intersecting
         for (SegmentsReflectionT::const_iterator it = segments.begin(); it != segments.end(); ++it)
         {
            size_t segmentId  = it->second.second;
            size_t polylineId = it->second.first;

            // Ignoring already ignoring polylines
            if (outerPolylines_.find(polylineId) != outerPolylines_.end())
               continue;

            segment_2 segment(cdt_.getSegment(polylineId, segmentId));
            double scalarProduct = normal(segment) * direction(ray_);

            if (cg::eq_zero(scalarProduct))
            {
               if (segment.contains(ray_.P0()))
               {
                  resultPolylineId_ = polylineId;
                  return true;
               }
               else
                  outerPolylines_.insert(polylineId);
            }
            else if (cw_ ^ cg::ge(scalarProduct, 0))
               outerPolylines_.insert(polylineId);
            else
            {
               resultPolylineId_ = polylineId;
               return true;
            }
         }

         return false;
      }

   private:
      CDT const             &cdt_;
      segment_2             ray_;
      bool                  cw_;
      size_t                accept_;

      std::set<size_t>      outerPolylines_;
      size_t                resultPolylineId_;
   };

public:
   grid_type const& getGrid() const
   {
      return *grid_;
   }

   cg::rectangle_2 bounding() const
   {
      cg::rectangle_2 rect;

      for(polyline_map_type::const_iterator it = polylines_.begin(), end = polylines_.end(); it != end; ++it)
         for(Polyline::points_type::const_iterator p = it->second.vertices.begin(), pend = it->second.vertices.end(); p != pend; ++p)
            rect |= *p;

      return rect;
   }

   // Serialization
public:
   template<class Stream>
      void serialize(Stream &stream) const
   {
      write(stream, polylines_);
      write(stream, numSegments_);
      write(stream, grid_.get());
   }

private:
   typedef typename traits_type::map_type polyline_map_type;

   // Polylines
   polyline_map_type polylines_;
   size_t            numSegments_;

   // Shoot-ray grid
   typedef typename traits_type::grid_ptr_type grid_ptr_type;
   grid_ptr_type grid_;

private:
   // Next polyline id
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

// Serialization
template< class Stream, class PolylineAttr, class VertexAttr, template<class, class> class Traits >
   void write(Stream &stream, PolylineCDT< PolylineAttr, VertexAttr, Traits > const &cdt)
{
   cdt.serialize(stream);
}

} // end of namespace cdt
} // end of namespace cg
