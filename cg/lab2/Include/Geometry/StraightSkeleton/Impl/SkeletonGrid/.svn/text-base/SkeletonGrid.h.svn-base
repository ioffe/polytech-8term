#pragma once

#include "contours\common.h"
#include "contours\holder\accessors.h"
#include "contours\holder\builder.h"
#include "contours\holder\mapped.h"
#include "contours\operations.h"
#include "contours\misc\smallcell.h"
#include "contours\misc\gridholder.h"
#include "contours\algorithm\buildContourNest.h"
#include "contours\algorithm\buildCdtInside.h"
#include "contours\algorithm\buildDistToContours.h"
#include "contours\algorithm\getDistToContours.h"
#include "contours\dtc\dtc_standard.h"

#include "Geometry\grid2L.h"
#include "Geometry\Grid2L\subdiv.h"

#include "visit_grid2l_by_beam.h"

#include "SimpleShootRay.h"
#include "SkeletonGridInitializer.h"

#include "AddProcessor.h"
#include "RemoveProcessor.h"
#include "UpdateQueueProcessor.h"

#include "FindSplitEventProcessor.h"
#include "findAllIntersections.h"

namespace cg {
namespace skeleton {

using cg::contours::contour_id;
using cg::contours::point_id;
using cg::contours::segment_id;

typedef cg::contours::smallcellw::Standard< segment_id, contour_id > igrid_smallcell_type;

//struct EdgeRep
//{
//   EdgeRep ( size_t edgeIdx, cg::range_2 const &r = cg::range_2 () )
//      : idx (edgeIdx), cRange (r)
//   {
//   }
//
//   size_t idx;
//   cg::range_2 cRange;
//};
//
//struct EdgeRepLess
//{
//   bool operator ()( EdgeRep const &a, EdgeRep const &b ) const
//   {
//      return a.idx < b.idx;
//   }
//};

struct SkeletonGridSmallCell
{
   // typedef std::set< EdgeRep, EdgeRepLess > EdgeContainer;
   typedef std::set< size_t > EdgeContainer;
   typedef std::set< size_t > VerticesContainer;

   // cg::range_2 low, high;
   EdgeContainer edges;
   VerticesContainer vertices;
};

typedef SkeletonGridSmallCell smallcell_type;

typedef
   cg::Grid2L< igrid_smallcell_type >
   igrid_type;

typedef
   cg::Grid2L< smallcell_type >
   skeleton_grid_type;

template < class contour_attributes >
   struct accessor_fun
{
private:
   typedef
      cg::contours::holder::Builder< cg::point_2, cg::contours::contour_nested< contour_attributes > > 
      builder_type;

public:
   typedef
      cg::contours::holder::Accessors< builder_type >
      type;
};

template < class derived, class contour_attributes >
   struct cdt
   : accessor_fun< contour_attributes >::type
   , cg::contours::Operations< derived, cdt< derived, contour_attributes >, contour_id >
   , cg::contours::misc::GridHolder< igrid_type >
   , cg::skeleton::shootraying::FindAllIntersections< derived, igrid_type >
{
   typedef igrid_type grid_type;
   typedef cg::skeleton::cdt< derived, contour_attributes > cdt_type;

   typedef contour_attributes contour_attributes_type;

   typedef
      typename accessor_fun< contour_attributes_type >::type
      contours_type;

   cdt_type const & getCdt() const { return static_cast< cdt_type const & >(*this); }
   cdt_type       & getCdt()       { return static_cast< cdt_type       & >(*this); }
};

template< class skeleton_type >
struct skeleton_grid : cdt< skeleton_grid< skeleton_type >, Empty >
{
   typedef cg::skeleton::cdt< skeleton_grid< skeleton_type >, Empty > cdt_type;
   typedef cg::contours::misc::GridHolder< skeleton_grid_type > skg_type;

   typedef typename skeleton_type::DCEL DCEL;

   template< class Stream >
      skeleton_grid ( Stream &stream, skeleton_type &skeleton )
         : skeleton_ (skeleton)
   {
      size_t contoursSize;
      stream >> contoursSize;
      for (size_t c = 0; c < contoursSize; ++c)
      {
         size_t pointsCount;
         stream >> pointsCount;
         std::vector< point_type > curContour;
         curContour.reserve(pointsCount);         
         for (size_t p = 0; p < pointsCount; ++p)
         {
            point_type val;
            stream >> val.x >> val.y;
            curContour.push_back(val);
         }
         add_contour(curContour.begin(), curContour.end(), -1);
      }
      process();

      // Skeleton grid
      cg::point_2 origin, unit;
      stream >> origin.x >> origin.y >> unit.x >> unit.y;
      cg::point_2i extents;
      stream >> extents.x >> extents.y;
      skGrid_.setGrid(new skeleton_grid_type (aa_transform (origin, unit), extents));

      cg::point_2i bigIdx;
      for (bigIdx.y = 0; bigIdx.y < extents.y; ++bigIdx.y)
         for (bigIdx.x = 0; bigIdx.x < extents.x; ++bigIdx.x)
         {
            cg::point_2i curCellExt;
            stream >> curCellExt.x >> curCellExt.y;

            skGrid_.grid().at(bigIdx).subdivide(curCellExt);

            cg::point_2i smallIdx;
            for (smallIdx.y = 0; smallIdx.y < curCellExt.y; ++smallIdx.y)
               for (smallIdx.x = 0; smallIdx.x < curCellExt.x; ++smallIdx.x)
               {
                  size_t numEdges;
                  stream >> numEdges;
                  for (size_t e = 0; e < numEdges; ++e)
                  {
                     size_t val;
                     stream >> val;
                     skGrid_.grid().at(bigIdx).at(smallIdx).edges.insert(val);
                  }

                  size_t numVertices;
                  stream >> numVertices;
                  for (size_t v = 0; v < numVertices; ++v)
                  {
                     size_t val;
                     stream >> val;
                     skGrid_.grid().at(bigIdx).at(smallIdx).vertices.insert(val);
                  }
               }
         }
   }

   struct time_meter
   {
      time_meter()
      {
         QueryPerformanceFrequency( ( LARGE_INTEGER * ) &freq_ ) ; 
         QueryPerformanceCounter ( ( LARGE_INTEGER * )&start_ ) ; 
      }

      double operator ()() const
      {
         long long end;
         QueryPerformanceCounter ( ( LARGE_INTEGER * ) &end ) ; 
         return (double)( end - start_ ) / freq_;
      }

   private:
      long long freq_;
      long long start_;
   };

   skeleton_grid ( skeleton_type &skeleton )
      : skeleton_ (skeleton)
   {
      typedef typename skeleton_type::DCEL DCEL;

      // Add contours
      for (DCEL::cycle_iterator contourIt = skeleton.dcel().cyclesBegin(); contourIt != skeleton.dcel().cyclesEnd(); ++contourIt)
      {
         std::vector< point_type > curContour;
         for (DCEL::cycle_edge_iterator polyIt = contourIt->begin; polyIt != contourIt->end; ++polyIt)
            curContour.push_back(skeleton.vertexPosFixed(polyIt->vertexOrigin));

         // Only not looped polylines are valid!
         Assert(curContour.size() > 2);
         Assert(curContour.front() != curContour.back());
         // bool ccw = !cg::clockwise_ordered(curContour.begin(), curContour.end());

         add_contour(curContour.rbegin(), curContour.rend(), static_cast< size_t >( -1 ));
      }

      process();

      // processSkeletonGrid();
   }

   typedef 
      cg::contours::dtc::standard::Grid
      dtc_grid_type;

   dtc_grid_type const & dtc() const { return *dtc_grid_; }

   template< class Abstract2dPoint >
      cg::point_2 convertPoint( Abstract2dPoint const &p )
   {
      return cg::point_2 (p.x(), p.y());
   }

   skg_type &skeletonGrid()
   {
      return skGrid_;
   }

   contour_id simpleShootRay(segment_2 const & s, contour_id defaultId) const
   {
      double const bugEpsilon = 100 * cg::epsilon< double >();

      segment_2 s_res;
      if (cg::abs(s.P1().y - s.P0().y) < bugEpsilon)
         s_res = cg::segment_2 (s.P0(), cg::point_2 (s.P1().x, s.P0().y));
      else if (cg::abs(s.P1().x - s.P0().x) < bugEpsilon)
         s_res = cg::segment_2 (s.P0(), cg::point_2 (s.P0().x, s.P1().y));
      else
         s_res = s;

      return cg::contours::algos::simpleShootRay(*this,
         static_cast< cdt_type const & >(*this), s_res, defaultId);
   }

   bool rayIntersectContour( cg::segment_2 const &s, contour_id contourId ) const
   {
      typedef std::vector< contours::contour_id > contours_vector;
      contours_vector contoursVec;
      size_t res = intersectSegment(s, std::back_inserter(contoursVec));
      if (res == 0)
         return false;

      for (contours_vector::const_iterator cIt = contoursVec.begin(); cIt != contoursVec.end(); ++cIt)
      {
         if (*cIt == contourId || contours().getContourContaining(*cIt) == contourId)
            return true;
      }

      return true;
   }

   template < class Stream >
      friend void write( Stream & out, skeleton_grid const & grid ) 
   {
      grid.contours  ( ).serialize( out );
      grid.gridHolder( ).serialize( out );
   }

   template < class FwdIter >
      void add_contour( FwdIter p, FwdIter q, size_t /*gid*/ )
   {
      contours( ).addContourChecked( p, q, Empty () );
   }

   void process( )
   {
      // Domain
      rectangle_2 domain;
      calculate_domain( domain );

      // Inside grid
      cg::Grid2LSubdiv subdiv( domain, 5, 10, 12 );

      setGrid ( new cg::Grid2LInitializer< grid_type >( 
         contours( ).segmentsBegin(), contours( ).segmentsEnd(), *this, subdiv ) );

      cg::contours::algos::buildContourNest( *this, *this );
      cg::contours::algos::buildCdtInside( *this, *this );

      // Temporary code
      // dtc_grid_.reset( cg::contours::_algos::buildDistToCoast< dtc_grid_type >( contours(), domain ) );
      // End of Temporary code
   }

   bool isOptimizationValid()
   {
      if (skeleton_.constructionParams().outside)
         return true;

      cg::contours::contour_id cIt = contoursBegin();
      for (DCEL::cycle_iterator contourIt = skeleton_.dcel().cyclesBegin(); contourIt != skeleton_.dcel().cyclesEnd(); ++contourIt, ++cIt)
      {
         std::vector< point_type > curContour;
         for (DCEL::cycle_edge_iterator polyIt = contourIt->begin; polyIt != contourIt->end; ++polyIt)
            curContour.push_back(skeleton_.vertexPosFixed(polyIt->vertexOrigin));

         bool cw = cg::clockwise_ordered(curContour.begin(), curContour.end());
         if (cw && getContourContaining(cIt) == -1)
            return false;
      }

      return true;
   }

   void processSkeletonGrid( double maxShift )
  {
      rectangle_2 domain;
      calculate_domain( domain );

      if (skeleton_.constructionParams().outside)
         domain.inflate(2 * maxShift);

      cg::Grid2LSubdiv subdiv( domain, 5, 1, 1 /* 10, 12 */ );

      skGrid_.setGrid(new cg::skeleton::SkeletonGridInitializer< skeleton_grid_type > (skeleton_, *this, subdiv, maxShift));
      insertEdges(maxShift);
      insertVertices(maxShift);
   }

   double makeShiftEstimation()
   {
      typedef cg::triangulation::cgal_triangulation<> Tri;

      std::vector< bool > holes;
      Tri triangulation;
      for (DCEL::cycle_iterator contourIt = skeleton_.dcel().cyclesBegin(); contourIt != skeleton_.dcel().cyclesEnd(); ++contourIt)
      {
         std::vector< point_type > curContour;
         for (DCEL::cycle_edge_iterator polyIt = contourIt->begin; polyIt != contourIt->end; ++polyIt)
            curContour.push_back(skeleton_.vertexPosFixed(polyIt->vertexOrigin));

         holes.push_back(cg::clockwise_ordered(curContour.begin(), curContour.end()));
         for (size_t v = 0; v < curContour.size(); ++v)
            triangulation.insert(curContour[v], curContour[(v != curContour.size() - 1) ? (v + 1) : 0]);
      }

      double maxShift = std::numeric_limits< double >::min();
      size_t triIdx = 0;
      for (Tri::faces_iterator fIt = triangulation.faces_begin(); fIt != triangulation.faces_end(); ++fIt, ++triIdx)
      {
         cg::point_2 vertex[3];
         vertex[0] = convertPoint(fIt->vertex(0)->point());
         vertex[1] = convertPoint(fIt->vertex(1)->point());
         vertex[2] = convertPoint(fIt->vertex(2)->point());

         cg::point_2 center = (vertex[0] + vertex[1] + vertex[2]) / 3;
         size_t cnt = (size_t)findContourPointBelongsTo(center);

         bool processTriangle = false;
         processTriangle = (cnt != -1 && !holes[cnt]);
         if (skeleton_.constructionParams().outside)
            processTriangle = !processTriangle;

         if (processTriangle)
         {
            size_t numConstrained = 0;
            size_t constrainedIdx = static_cast< size_t >( -1 ), nonConstrainedIdx = static_cast< size_t >( -1 );
            for (size_t e = 0; e < 3; ++e)
            {
               if (fIt->is_constrained(e))
               {
                  numConstrained++;
                  if (constrainedIdx == -1)
                     constrainedIdx = e;
               }
               else if (nonConstrainedIdx == -1)
                  nonConstrainedIdx = e;
            }

            double curShift = std::numeric_limits< double >::min();
            switch (numConstrained)
            {
            case 0:
               // Skip that type
               break;

            case 1:
               curShift = 0.5 * cg::distance(vertex[constrainedIdx],
                  cg::line_2 (vertex[(constrainedIdx + 1) % 3], vertex[(constrainedIdx + 2) % 3], cg::line::by_points));
               break;

            case 2:
               {
                  cg::point_2 bisector = cg::calculateBisector(vertex[(nonConstrainedIdx + 2) % 3],
                     vertex[nonConstrainedIdx], vertex[(nonConstrainedIdx + 1) % 3], false, true);

                  cg::line_2 bLine (vertex[nonConstrainedIdx], bisector, cg::line::by_direction);
                  cg::line_2 seg (vertex[(nonConstrainedIdx + 1) % 3], vertex[(nonConstrainedIdx + 2) % 3], cg::line::by_points);

                  cg::point_2 center;
                  if ( !rayLineIntersection(bLine, seg, center) )
                     curShift = 1;
                  else
                     curShift = cg::distance(center,
                        cg::segment_2 (vertex[nonConstrainedIdx], vertex[(nonConstrainedIdx + 1) % 3]));
               }
               break;

            case 3:               
               {
                  cg::point_2 bisector[2];
                  bisector[0] = cg::calculateBisector(vertex[2], vertex[0], vertex[1], false, true);
                  bisector[1] = cg::calculateBisector(vertex[0], vertex[1], vertex[2], false, true);

                  cg::line_2
                     bLine0 (vertex[0], bisector[0], cg::line::by_direction),
                     bLine1 (vertex[1], bisector[1], cg::line::by_direction);

                  cg::point_2 center;
                  if (!rayLineIntersection(bLine0, bLine1, center))
                     center = (vertex[0] + vertex[1] + vertex[2]) / 3; // In case of tiny triangles
                  curShift = cg::distance(cg::segment_2 (vertex[0], vertex[1]), center);
               }
               break;

            default:
               Verify(false);
               break;
            }

            maxShift = cg::max(curShift, maxShift);
         }
      }

      return maxShift;
   }

   void addEdge( size_t edgeIdx, double maxShift )
   {
      size_t
         vOrig = skeleton_.dcel().edge(edgeIdx).vertexOrigin,
         vDest = skeleton_.dcel().edge(edgeIdx).vertexDestination;

      if (skeleton_.dcel().vertex(vOrig).data.processed &&
          skeleton_.dcel().vertex(vDest).data.processed)
      {
         return;
      }

      AddEdgeProcessor< skeleton_grid_type > proc (edgeIdx);
      visit_internal(skGrid_, *this, edge2beam(skeleton_, edgeIdx), proc, maxShift);
   }

   void addVertex( size_t vertexIdx, double maxShift )
   {
      if (skeleton_.dcel().vertex(vertexIdx).data.processed)
         return;

      cg::segment_2 bisectorRay = prepareBisectorRay(vertexIdx, maxShift);
      AddVertexProcessor< skeleton_grid_type > proc(vertexIdx);
      visit(skGrid_.grid(), bisectorRay, proc);
   }

   //void removeEdge( size_t edgeIdx, double maxShift )
   //{
   //   visit_internal(skGrid_, *this, edge2beam_prev(skeleton_, edgeIdx),
   //      RemoveEdgeProcessor< skeleton_grid_type > (edgeIdx), maxShift);
   //}

   void removeVertex( size_t vertexIdx, double maxShift )
   {
      cg::segment_2 bisectorRay = prepareBisectorRay(vertexIdx, maxShift, true);
      RemoveVertexProcessor< skeleton_grid_type > proc(vertexIdx);
      visit(skGrid_.grid(), bisectorRay, proc);
   }

   cg::segment_2 prepareBisectorRay( size_t vertexIdx, double maxShift, bool prev = false )
   {
      cg::segment_2 bisectorRay = getRay(skGrid_.grid(), skeleton_, vertexIdx, maxShift, prev);

      FindContourNearestIntersectionProcessor searcher (bisectorRay.P0());
      genericShootRay(bisectorRay, searcher);
      if (searcher.was_intersection())
      {
         cg::point_2 candidatePoint (searcher.intersectionPoint());
         if (cg::norm_sqr(candidatePoint - bisectorRay.P0()) < cg::norm_sqr(bisectorRay.P1() - bisectorRay.P0()))
            bisectorRay = cg::segment_2 (bisectorRay.P0(), candidatePoint);
      }

      return bisectorRay;
   }

   bool findSplitEvent( double curTime, size_t vertexIdx,
                        size_t &edgeIdx, double &resT, cg::point_2 &BPoint,
                        size_t excludeEdgeA, size_t excludeEdgeB, double maxShift,
                        bool preprocessing = false, bool exact = false )
   {
      cg::segment_2 bisectorRay = prepareBisectorRay(vertexIdx, maxShift);
      
      bool eventFound;
      FindSplitEventProcessor< skeleton_grid_type, skeleton_type > proc(skGrid_.grid(),
                  skeleton_, curTime, vertexIdx, edgeIdx, resT, BPoint,
                  eventFound, excludeEdgeA, excludeEdgeB, preprocessing, exact);
      visit(skGrid_.grid(), bisectorRay, proc );

      return eventFound;
   }

   void updateQueueByEdgeAdd( ss::EventQueue &queue, size_t edgeIdx, bool checkForNewEvents,
                              std::set< size_t > const &handledVertices,
                              ss::EdgeListEx const &edges, ss::VertexList const &vertices, double maxShift )
   {
      AddUpdateQueueProcessor< skeleton_grid_type, skeleton_type > proc(skeleton_, queue,
            edgeIdx, checkForNewEvents, handledVertices, edges, vertices);
      visit_internal(skGrid_, *this, edge2beam(skeleton_, edgeIdx), proc, maxShift);
   }

   void updateQueueByEdgeRemove( ss::EventQueue &queue, ss::EdgeListElem elem, double t,
                                 std::list< size_t > &verticesToObtainEvents,
                                 std::set< size_t > &handledVertices,
                                 ss::EdgeListEx const &edges, ss::VertexList const &vertices, double maxShift )
   {
      RemoveUpdateQueueProcessor< skeleton_grid_type, skeleton_type > proc(skeleton_, queue,
                  elem, t, verticesToObtainEvents, handledVertices, edges, vertices);
      visit_internal(skGrid_, *this, elem.initialBeam, proc, maxShift);
   }

   template< class Stream >
      void dump( Stream &stream )
   {
      // Inside/outside grid input data
      stream << contours().contours().size() << std::endl;
      for (cg::contours::contour_id c = contoursBegin(); c != contoursEnd(); ++c)
      {
         stream << pointsCnt(c) << ' ';
         for (cg::contours::point_id p = pointsBegin(c); p != pointsEnd(c); ++p)
            stream << std::setprecision(32) << getPoint(p).x << ' ' << getPoint(p).y << ' ';
         stream << std::endl;
      }
      stream << std::endl;

      // Skeleton grid
      stream << std::setprecision(32) << skGrid_.grid().origin().x << ' ' << skGrid_.grid().origin().y << ' ';
      stream << std::setprecision(32) << skGrid_.grid().unit().x << ' ' << skGrid_.grid().unit().y << ' ';
      stream << skGrid_.grid().extents().x << ' ' << skGrid_.grid().extents().y << std::endl;

      cg::point_2i bigIdx;
      for (bigIdx.y = 0; bigIdx.y < skGrid_.grid().extents().y; ++bigIdx.y)
         for (bigIdx.x = 0; bigIdx.x < skGrid_.grid().extents().x; ++bigIdx.x)
         {
            stream << skGrid_.grid().at(bigIdx).extents().x << ' ' << skGrid_.grid().at(bigIdx).extents().y << ' ';

            cg::point_2i smallIdx;
            for (smallIdx.y = 0; smallIdx.y < skGrid_.grid().at(bigIdx).extents().y; ++smallIdx.y)
               for (smallIdx.x = 0; smallIdx.x < skGrid_.grid().at(bigIdx).extents().x; ++smallIdx.x)
               {
                  stream << skGrid_.grid().at(bigIdx).at(smallIdx).edges.size() << ' ';
                  for (SkeletonGridSmallCell::EdgeContainer::const_iterator eIt = skGrid_.grid().at(bigIdx).at(smallIdx).edges.begin();
                          eIt != skGrid_.grid().at(bigIdx).at(smallIdx).edges.end(); ++eIt)
                  {
                     stream << *eIt << ' ';
                  }

                  stream << skGrid_.grid().at(bigIdx).at(smallIdx).vertices.size() << ' ';
                  for (SkeletonGridSmallCell::VerticesContainer::const_iterator vIt = skGrid_.grid().at(bigIdx).at(smallIdx).vertices.begin();
                       vIt != skGrid_.grid().at(bigIdx).at(smallIdx).vertices.end(); ++vIt)
                  {
                     stream << *vIt << ' ';
                  }
               }
         }
   }

private:   
   contours_type         & contours( )       { return *this; }
   contours_type   const & contours( ) const { return *this; }

   void calculate_domain( rectangle_2 & rect )
   {
      for ( point_id pid = pointsBegin( ); pid != pointsEnd( ); ++pid )
         rect |= contours( ).getPoint( pid );

      if ( !rect.empty( ) )
         rect.inflate( 1. );
   }

   void insertEdges( double maxShift )
   {
      for (DCEL::edges_const_iterator eIt = skeleton_.dcel().edgesBegin(); eIt != skeleton_.dcel().edgesEnd(); ++eIt)
      {
         if (!eIt->hole)
            addEdge(eIt.index(), maxShift);
      }
   }

   void insertVertices( double maxShift )
   {
      for (DCEL::vertices_const_iterator vIt = skeleton_.dcel().verticesBegin(); vIt != skeleton_.dcel().verticesEnd(); ++vIt)
      {
         if (!vIt->data.processed)
            addVertex(vIt - skeleton_.dcel().verticesBegin(), maxShift);
      }
   }

private:
   skeleton_type &skeleton_;
   cg::contours::misc::GridHolder< skeleton_grid_type > skGrid_;

   // std::auto_ptr< dtc_grid_type > dtc_grid_;
};

} // End of 'skeleton' namespace
} // End of 'cg' namespace
