#pragma once

#include "Geometry\cgal_predicates.h"

namespace cg
{

template< class Skeleton >
   bool findSplitIntersection( Skeleton &skeleton, size_t v, size_t e,
                               cg::point_2 &out, double &outT,
                               bool preprocessing = false, bool exact = false )
{
   cg::point_2 const &localCenter (skeleton.dcel().vertex(v).pos);
   // cg::point_2 localCenter (0, 0);
   //cg::point_2 localCenter (skeleton.getLocalCenter(v, e));
   // cg::point_2 localCenter (skeleton.vertexPosFixed(v));

   size_t aV = skeleton.dcel().edge(e).vertexOrigin;
   size_t bV = skeleton.dcel().edge(e).vertexDestination;

   cg::line_2 edgeLine = skeleton.getEdgeLine(e, localCenter);
   cg::segment_2 seg = skeleton.edgeSegment(e, localCenter);

   cg::point_2 localVPos (skeleton.vertexPos(v, localCenter));

   cg::line_2 curBLine (localVPos, skeleton.dcel().vertex(v).data.bisector, line::by_direction);

   if (skeleton.dcel().vertex(v).data.bisector * cg::normal(edgeLine) > 0)
      return false;

   if (cg::eq(seg.P0(), localVPos, 1e-7) || cg::eq(seg.P1(), localVPos, 1e-7))
   {
      if (!cg::eq(localVPos, skeleton.dcel().vertex(v).pos - localCenter, 1e-7))
      {
         out = localVPos;
         outT = 0;
         return true;
      }
      // return false;
   }

   point_2 edgeIntersection;
   if (exact)
   {
      if (!robust_rayLineIntersection(curBLine, edgeLine, edgeIntersection))
         return false;
   }
   else
   {
      if (!rayLineIntersection(curBLine, edgeLine, edgeIntersection))
         return false;
   }

   bool vertexOnEdge = false;
   if (cg::eq_zero((localVPos - edgeLine.p()) * cg::normal(edgeLine)))
   {
      if (preprocessing)
         return false;

      vertexOnEdge = true;

      if (!cg::eq(seg.P0(), seg.P1()))
      {
         double t = seg(localVPos);
         if (t < 0 || t > 1)// if (!cg::ge(t, 0) || !cg::le(t, 1))
            return false;
      }
   }
   else if (cg::eq(skeleton.dcel().vertex(aV).data.bisector, skeleton.dcel().vertex(v).pos - localCenter) ||
            cg::eq(skeleton.dcel().vertex(bV).data.bisector, skeleton.dcel().vertex(v).pos - localCenter))
   {
      return false;
   }

   size_t vIncidentEdge = skeleton.dcel().vertex(v).incidentEdge;
   cg::line_2 incEdge = skeleton.getEdgeLine(vIncidentEdge, localCenter);
   cg::line_2 prevEdge = skeleton.getEdgeLine(skeleton.dcel().edge(vIncidentEdge).prevEdge, localCenter);

   cg::line_2 vEdgeLine = incEdge;
   if (abs(prevEdge.r() * edgeLine.r()) < abs(incEdge.r() * edgeLine.r()))
      vEdgeLine = prevEdge;

   if (vertexOnEdge)
      out = localVPos;
   else
   {
      cg::point_2 edgeWithEdgeIntersection;
      bool has_inters = false;
      if (!exact)
         has_inters = cg::has_intersection(vEdgeLine, edgeLine, edgeWithEdgeIntersection);
      else
         has_inters = cg::robust_has_intersection(vEdgeLine, edgeLine, edgeWithEdgeIntersection);

      if (has_inters)
      {
         cg::point_2 angleBisector = calculateBisector(edgeIntersection,
            edgeWithEdgeIntersection, localVPos, false, true);

         if (!exact)
         {
            cg::has_intersection(curBLine,
               cg::line_2 (edgeWithEdgeIntersection, angleBisector, line::by_direction), out);
         }
         else
         {
            cg::robust_has_intersection(curBLine,
               cg::line_2 (edgeWithEdgeIntersection, angleBisector, line::by_direction), out);
         }
      }         
      else
         out = localVPos + 0.5 * (edgeIntersection - localVPos);
   }

   // Check if result point is really an event point
   cg::point_2 aVPos (skeleton.vertexPos(aV, localCenter));
   cg::point_2 bVPos (skeleton.vertexPos(bV, localCenter));
   
   outT = cg::norm((out - aVPos) - edgeLine.r() * ((out - aVPos) * edgeLine.r()));

   double prod1 = (out - aVPos) ^ skeleton.dcel().vertex(aV).data.bisector;
   double prod2 = skeleton.dcel().vertex(bV).data.bisector ^ (out - bVPos);   
   if (prod1 > 0 && prod2 > 0)
   {      
      //cg::point_2 aVEventPos = aVPos + skeleton.dcel().vertex(aV).data.bisector * outT;
      //cg::point_2 bVEventPos = bVPos + skeleton.dcel().vertex(bV).data.bisector * outT;

      //cg::point_2 vEventPos = localVPos + skeleton.dcel().vertex(v).data.bisector * outT;

      //double checkDist = cg::distance(vEventPos, cg::segment_2 (aVEventPos, bVEventPos));
      //Assert(cg::eq_zero(checkDist, 1e-5));

      return true;
   }
   else if (   ( cg::eq_zero( prod1, 1e-6 ) && ( prod2 > 0 ) ) || 
               ( cg::eq_zero( prod2, 1e-6 ) && ( prod1 > 0 ) ) ||
               ( cg::eq_zero( prod1, 1e-6 ) && ( cg::eq_zero( prod2, 1e-6 ) ) ) )
   {
      cg::point_2 aVEventPos = aVPos + skeleton.dcel().vertex(aV).data.bisector * outT;
      cg::point_2 bVEventPos = bVPos + skeleton.dcel().vertex(bV).data.bisector * outT;
      cg::segment_2 seg( aVEventPos, bVEventPos );

      cg::point_2 vEventPos = localVPos + skeleton.dcel().vertex(v).data.bisector * outT;
      double inside = seg(vEventPos);
      return (inside >= 0) && (inside <= 1);
   }

   return false;
}

} // End of 'cg' namespace
