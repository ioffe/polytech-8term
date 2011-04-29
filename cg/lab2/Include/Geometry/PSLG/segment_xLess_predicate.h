#pragma once

//
// Predicate to compare two segments by x coordinate
// Point-segment is mentioned to be greater if lies on not point-segment
//

namespace cg
{
namespace pslg
{

template< class Segment >
   struct SegmentXLess
{
   typedef typename Segment::point_type point_type;

   struct CmpSegment
   {
      point_type vertexOrigin;
      point_type vertexDestination;

      CmpSegment ( Segment const &seg )
         : vertexOrigin (seg.edgeIdx == -1 ? seg.vertexOrigin : seg.dcel->vertex(seg.dcel->edge(seg.edgeIdx).vertexOrigin).pos)
         , vertexDestination (seg.edgeIdx == -1 ? seg.vertexDestination : seg.dcel->vertex(seg.dcel->edge(seg.edgeIdx).vertexDestination).pos)
      {
      }

      CmpSegment ( cg::segment_2 const &seg )
         : vertexOrigin (seg.P0()), vertexDestination (seg.P1())
      {
      }
   };

   bool lessF ( CmpSegment const &e1, CmpSegment const &e2 ) const
   {
      CmpSegment a = e1, b = e2;

      if (a.vertexDestination.y < a.vertexOrigin.y)
         std::swap(a.vertexOrigin, a.vertexDestination);
      if (b.vertexDestination.y < b.vertexOrigin.y)
         std::swap(b.vertexOrigin, b.vertexDestination);

      bool isAHorizontal = false;
      if (a.vertexDestination.y == a.vertexOrigin.y)
      {
         isAHorizontal = true;
         if (a.vertexOrigin.x < a.vertexDestination.x)
            std::swap(a.vertexOrigin, a.vertexDestination);
      }
      bool isBHorizontal = false;
      if (b.vertexDestination.y == b.vertexOrigin.y)
      {
         isBHorizontal = true;
         if (b.vertexOrigin.x < b.vertexDestination.x)
            std::swap(b.vertexOrigin, b.vertexDestination);
      }

      bool isAPointEdge = (a.vertexDestination == a.vertexOrigin);
      bool isBPointEdge = (b.vertexDestination == b.vertexOrigin);

      if (a.vertexDestination == b.vertexDestination)
      {
         if (isBPointEdge || isAPointEdge || (isAHorizontal && isBHorizontal))
            return false;

         return robust_left_turn_strict(b.vertexOrigin, b.vertexDestination, a.vertexOrigin);
      }

      if (a.vertexDestination.y >= b.vertexDestination.y)
      {
         if (isAPointEdge || isAHorizontal)
         {
            Assert(a.vertexDestination.y == b.vertexDestination.y);
            if (isBHorizontal)
               return a.vertexOrigin.x <= b.vertexDestination.x;
            return a.vertexOrigin.x <= b.vertexDestination.x;
         }

         VecOrientation orien = b.vertexDestination != a.vertexOrigin ?
            robust_orientation(b.vertexDestination, a.vertexOrigin, a.vertexDestination) :
            robust_orientation(a.vertexDestination, a.vertexOrigin, b.vertexOrigin);

         if (orien == VO_COLLINEAR)
            return robust_orientation(b.vertexOrigin, a.vertexOrigin, a.vertexDestination) == VO_RIGHT;

         return orien == VO_RIGHT;
      }

      if (b.vertexDestination.y >= a.vertexDestination.y)
      {
         if (isBPointEdge || isBHorizontal)
         {
            Assert(a.vertexDestination.y == b.vertexDestination.y);
            if (isAHorizontal || isAPointEdge)
               return a.vertexOrigin.x < b.vertexDestination.x;
            return a.vertexDestination.x < b.vertexOrigin.x;
         }

         VecOrientation orien = a.vertexDestination != b.vertexOrigin ?
            robust_orientation(a.vertexDestination, b.vertexOrigin, b.vertexDestination) :
            robust_orientation(b.vertexDestination, b.vertexOrigin, a.vertexOrigin);

         if (orien == VO_COLLINEAR)
            return robust_orientation(a.vertexOrigin, b.vertexOrigin, b.vertexDestination) == VO_LEFT;

         return orien == VO_LEFT;
      }

      return false;
   }

   bool operator () ( Segment const &e1, Segment const &e2 ) const
   {
      return lessF(CmpSegment (e1), CmpSegment (e2));
   }

   bool operator () ( cg::segment_2 const &e1, cg::segment_2 const &e2 ) const
   {
      return lessF(CmpSegment (e1), CmpSegment (e2));
   }
};

} // End of 'pslg' namespace
} // End of 'cg' namespace
