#pragma once

#include "Geometry/DCEL/dcel.h"
#include "Geometry/segments_intersections.h"
#include "self_intersection_fwd.h"
#include "self_intersection_overlaps.h"

namespace cg
{
namespace verification
{

namespace detail
{

template< class point_type >
   bool ordered( point_type const &a, point_type const &b, point_type const &c )
{
   typedef cg::dcel::DCEL< point_type::scalar_type > DCEL;
   DCEL::RobustOrientationPredicate pred;

   DCEL::EdgesPairOrientation ab = pred(point_type (), a, b);
   if (ab == DCEL::EPO_CODIRECTIONAL)
      return true;
   else if (ab == DCEL::EPO_LEFT)
   {
      DCEL::EdgesPairOrientation bc = pred(point_type (), b, c);
      if (bc == DCEL::EPO_RIGHT)
      {
         DCEL::EdgesPairOrientation ac = pred(point_type (), a, c);
         return ac != DCEL::EPO_LEFT;
      }
      else
         return true;
   }
   else
   {
      DCEL::EdgesPairOrientation bc = pred(point_type (), b, c);
      if (bc == DCEL::EPO_RIGHT)
         return false;
      if (bc == DCEL::EPO_OPPOSITELY_DIRECTED && ab == DCEL::EPO_OPPOSITELY_DIRECTED)
         return true;
      if (bc == DCEL::EPO_CODIRECTIONAL)
         return true;

      if (bc == DCEL::EPO_LEFT)
      {
         DCEL::EdgesPairOrientation ac = pred(point_type (), a, c);
         return ac != DCEL::EPO_LEFT;
      }
   }

   return false;
}

} // End of 'detail' namespace

template< class VertexBuffer, class ContoursIterator, class OutIter >
   void check_self_intersection( VertexBuffer const &vertices,
                                 ContoursIterator begin, ContoursIterator end,
                                 OutIter out, int check_overlap = OT_NONE )
{
   check_self_intersection( vertices, begin, end, out, check_overlap,
                            cg::epsilon< VertexBuffer::value_type::scalar_type >() );
}

template< class VertexBuffer, class ContoursIterator, class OutIter >
   void check_self_intersection( VertexBuffer const &vertices,
                                 ContoursIterator begin, ContoursIterator end,
                                 OutIter out, int check_overlap,
                                 typename VertexBuffer::value_type::scalar_type eps )
{
   check_self_intersection_n_overlap( vertices, begin, end, out, util::null_iterator(), check_overlap, eps );
}

template< class VertexBuffer, class ContoursIterator, class OutIter, class OutOverlapIter >
void check_self_intersection_n_overlap(  VertexBuffer const &vertices,
                                          ContoursIterator begin, ContoursIterator end,
                                          OutIter out, OutOverlapIter outOverlap, int check_overlap )
{
   check_self_intersection_n_overlap(    vertices, begin, end, out, outOverlap, check_overlap,
                                          cg::epsilon< VertexBuffer::value_type::scalar_type >() );
}

// TODO: Understand what this function should do.
// Currently it does something like searching of intersections and overlappings.
template< class VertexBuffer, class ContoursIterator, class OutIter, class OutOverlapIter >
   void check_self_intersection_n_overlap( VertexBuffer const &vertices,
                                 ContoursIterator begin, ContoursIterator end,
                                 OutIter out, OutOverlapIter outOverlap, int check_overlap,
                                 typename VertexBuffer::value_type::scalar_type eps )
{
   typedef VertexBuffer::value_type::scalar_type scalar_type;
   typedef cg::point_t< scalar_type, 2 > point_type;
   typedef cg::segment_t< double, 2 > segment_type;

   std::vector< size_t > cntStart;
   cntStart.reserve(std::distance(begin, end));

   // Calculate total number of contours segments.
   size_t num_segs = 0;
   for (ContoursIterator cIt = begin; cIt != end; ++cIt)
   {
      cntStart.push_back(num_segs);
      num_segs += std::distance(cIt->begin(), cIt->end());
   }

   std::vector< segment_type > segs;
   segs.reserve(num_segs);
   std::vector< size_t > segs2cnt;
   segs2cnt.reserve(num_segs);
   std::vector< size_t > segs2idxInContour; // TODO: Optimize. This value can be evaluated faster.
   segs2idxInContour.reserve(num_segs);

   // Construct list of all contours segments.
   size_t cntIdx(0);
   for (ContoursIterator cIt = begin; cIt != end; ++cIt,  ++cntIdx)
   {
      size_t idxInContour(0);
      for (ContoursIterator::value_type::const_iterator vIt = cIt->begin(); vIt != cIt->end(); ++vIt, ++idxInContour)
      {
         segs.push_back(
            segment_type(
               vertices[*vIt],
               vertices[*(util::next(vIt) == cIt->end() ? cIt->begin() : util::next(vIt))]));
         segs2cnt.push_back(cntIdx);
         segs2idxInContour.push_back(idxInContour);
      }
   }

   typedef cg::SegmentsIntersections< segment_type > segs_intersector;
   segs_intersector intersector(segs, eps);

   typedef
      std::set< contours_intersection >
      intersections_set;

   typedef
      std::map< std::pair< size_t, size_t >, intersections_set >
      inter_map;

   inter_map result;

   typedef
      std::map< std::pair< size_t, size_t >, OverlapType >
      contour_overlap_type;

   contour_overlap_type overlap;

   for (segs_intersector::const_iterator iIt = intersector.begin(); iIt != intersector.end(); ++iIt)
   {
      size_t const cntA = segs2cnt[iIt->id1()];
      size_t const cntB = segs2cnt[iIt->id2()];

      size_t const segIdxA = segs2idxInContour[iIt->id1()];
      size_t const segIdxB = segs2idxInContour[iIt->id2()];

      size_t const cntASize = (begin + cntA)->size();
      size_t const cntBSize = (begin + cntB)->size();

      size_t const dist = abs((int)iIt->id1() - (int)iIt->id2());
      if (cntA == cntB && (dist == 1 || dist == cntASize - 1))
      {
         // Ignore intersections between adjacent edges.
         continue;
      }

      if (check_overlap != OT_FULL)
      {
         // At least one of internal or external contacts types treated as NOT intersection.

         if (iIt->type() == cg::overlap)
         {
            // Ignore overlaps.
            continue;
         }

         if (iIt->type() == cg::intersect)
         {
            bool
               aVertexBeg = cg::eq(segs[iIt->id1()].P0(), iIt->p()), // TODO: Maybe use input 'eps'?
               aVertexEnd = cg::eq(segs[iIt->id1()].P1(), iIt->p()),
               bVertexBeg = cg::eq(segs[iIt->id2()].P0(), iIt->p()),
               bVertexEnd = cg::eq(segs[iIt->id2()].P1(), iIt->p());

            bool
               aVertex = aVertexBeg || aVertexEnd,
               bVertex = bVertexBeg || bVertexEnd;

            if (aVertex || bVertex)
            {
               // Two non-adjacent edges intersects in at least one of its ends.

               size_t idA = (aVertexEnd || !aVertex) ? iIt->id1() :
                  (cntStart[cntA] + (iIt->id1() - cntStart[cntA] + cntASize - 1) % cntASize);
               size_t idNextA = cntStart[cntA] + (idA - cntStart[cntA] + 1) % cntASize;

               size_t idB = (bVertexEnd || !bVertex) ? iIt->id2() :
                  (cntStart[cntB] + (iIt->id2() - cntStart[cntB] + cntBSize - 1) % cntBSize);
               size_t idNextB = cntStart[cntB] + (idB - cntStart[cntB] + 1) % cntBSize;

               point_type a, nextA;
               if (aVertex)
               {
                  a = -cg::direction(segs[idA]);
                  nextA = cg::direction(segs[idNextA]);
               }
               else
               {
                  a = segs[idA].P0() - iIt->p();
                  nextA = segs[idA].P1() - iIt->p();
               }

               point_type b, nextB;
               if (bVertex)
               {
                  b = -cg::direction(segs[idB]);
                  nextB = cg::direction(segs[idNextB]);
               }
               else
               {
                  b = segs[idB].P0() - iIt->p();
                  nextB = segs[idB].P1() - iIt->p();
               }

               bool external_overlap = detail::ordered(a, b, nextA) && detail::ordered(a, nextB, nextA);
               bool internal_overlap = detail::ordered(a, nextA, b) && detail::ordered(a, nextA, nextB);

               if (check_overlap == OT_EXTERNAL && internal_overlap)
                  continue;
               if (check_overlap == OT_INTERNAL && external_overlap)
                  continue;

               if (check_overlap == OT_NONE)
               {
                  if (internal_overlap && external_overlap)
                     continue;

                  contour_overlap_type::iterator place = overlap.find(std::make_pair(cntA, cntB));
                  if (place != overlap.end())
                  {
                     if (internal_overlap && place->second == OT_INTERNAL)
                        continue;
                     if (external_overlap && place->second == OT_EXTERNAL)
                        continue;
                  }
                  else
                  {
                     if (internal_overlap)
                        overlap[std::make_pair(cntA, cntB)] = OT_INTERNAL;
                     else
                        overlap[std::make_pair(cntA, cntB)] = OT_EXTERNAL;
                     continue;
                  }
               }
            }
         }
      }

      {
         size_t outCntA(cntA), outCntB(cntB);
         size_t outSegIdxA(segIdxA), outSegIdxB(segIdxB);
         if (outCntA > outCntB)
         {
            std::swap(outCntA, outCntB);
            std::swap(outSegIdxB, outSegIdxB);
         }
         else if (outCntA == outCntB && segIdxA > segIdxB)
         {
            std::swap(outSegIdxA, outSegIdxB);
         }

         result[std::make_pair(cntA, cntB)].insert(contours_intersection(outCntA, outCntB, outSegIdxA, outSegIdxB, iIt->p()));
      }
   }

   for (inter_map::const_iterator rIt = result.begin(); rIt != result.end(); ++rIt)
   {
      for (intersections_set::const_iterator sIt = rIt->second.begin(); sIt != rIt->second.end(); ++sIt)
         *out++ = *sIt;
   }

   for (contour_overlap_type::const_iterator oIt = overlap.begin(); oIt != overlap.end(); ++oIt)
   {
      *outOverlap++ = oIt->first;
   }
}

} // End of 'verification' namespace
} // End of 'cg' namespace
