#pragma once

namespace cg
{

   struct empty_processor
   {
      template< class State, class cell_type >
         bool operator () ( State const &, cell_type & )
      {
         return false;
      }
   };

namespace skeleton
{

   template< class Skeleton >
      inline beam edge2beam( Skeleton &skeleton, size_t edgeIdx )
   {
      cg::segment_2 edgeSeg = skeleton.edgeSegment(edgeIdx, cg::point_2 ());

      edgeSeg =
         cg::segment_2 (edgeSeg.P0(), edgeSeg.P1());

      return beam (edgeSeg,
         skeleton.dcel().vertex(skeleton.dcel().edge(edgeIdx).vertexOrigin).data.bisector,
         skeleton.dcel().vertex(skeleton.dcel().edge(edgeIdx).vertexDestination).data.bisector);
   }

   template< class Skeleton >
      inline beam edge2beam_prev( Skeleton &skeleton, size_t edgeIdx, double &beamT )
   {
      Skeleton::DCEL::Vertex const &vOrig = skeleton.dcel().vertex(skeleton.dcel().edge(edgeIdx).vertexOrigin);
      Skeleton::DCEL::Vertex const &vDest = skeleton.dcel().vertex(skeleton.dcel().edge(edgeIdx).vertexDestination);

      beamT = cg::max(skeleton.getIterationStartTime(), vOrig.data.t, vDest.data.t);

      cg::segment_2 edgeSeg (vOrig.pos + (beamT - vOrig.data.t) * vOrig.data.bisector,
                             vDest.pos + (beamT - vDest.data.t) * vDest.data.bisector);

      edgeSeg =
         cg::segment_2 (edgeSeg.P0(), edgeSeg.P1());

      return beam (edgeSeg,
         skeleton.dcel().vertex(skeleton.dcel().edge(edgeIdx).vertexOrigin).data.bisector,
         skeleton.dcel().vertex(skeleton.dcel().edge(edgeIdx).vertexDestination).data.bisector);
   }

   template< class GridType >
      inline void maximizeBeam( GridType &grid, beam &b )
   {
      cg::rectangle_2 gbox = bounding(grid);

      cg::point_2 corners[4] =
      {
         gbox.lo(),
         cg::point_2 (gbox.hi().x, gbox.lo().y),
         gbox.hi(),
         cg::point_2 (gbox.lo().x, gbox.hi().y)
      };

      point_2 beamDir;
      if (!cg::eq(b.edge.P0(), b.edge.P1()))
         beamDir = cg::normalized(cg::normal(b.edge));
      else
         beamDir = cg::normalized(cg::normal(b.dirEnd - b.dirStart));

      size_t maxCorner;
      double maxDist = std::numeric_limits< double >::min();
      for (size_t c = 0; c < 4; c++)
      {
         double curDist = (corners[c] - b.edge.P0()) * beamDir;
         if (curDist > maxDist)
         {
            maxDist = curDist;
            maxCorner = c;
         }
      }

      b.dirStart = b.dirStart * maxDist/* / (b.dirStart * beamDir))*/;
      b.dirEnd = b.dirEnd * maxDist/* / (b.dirEnd * beamDir))*/;
   }

   template< class GridType >
      inline cg::segment_2 maximizeRay( GridType &grid, cg::segment_2 const &seg, double maxShift )
   {
      cg::rectangle_2 gbox = bounding(grid);

      cg::point_2 corners[4] =
      {
         gbox.lo(),
         cg::point_2 (gbox.hi().x, gbox.lo().y),
         gbox.hi(),
         cg::point_2 (gbox.lo().x, gbox.hi().y)
      };

      cg::point_2 direction = seg.P1() - seg.P0();
      cg::point_2 vertexPos = seg.P0();

      double maxDist = std::numeric_limits< double >::min();
      size_t maxCorner;
      for (size_t c = 0; c < 4; ++c)
      {
         double curDist = (corners[c] - vertexPos) * direction;
         if (curDist > maxDist)
         {
            maxCorner = c;
            maxDist = curDist;
         }
      }

      return cg::segment_2 (vertexPos, vertexPos + direction * cg::min(maxShift, (maxDist / cg::norm_sqr(direction))));
   }

   template< class GridType, class Skeleton >
      inline cg::segment_2 getRay( GridType &grid, Skeleton &skeleton, size_t vertexIdx, double maxShift, bool prev = false )
   {
      cg::point_2 direction = skeleton.dcel().vertex(vertexIdx).data.bisector;
      cg::point_2 vertexPos;
      double t;
      if (!prev)
      {
         vertexPos = skeleton.vertexPosFixed(vertexIdx);
         t = skeleton.getTime();
      }
      else
      {
         t = cg::max(skeleton.getIterationStartTime(), skeleton.dcel().vertex(vertexIdx).data.t);
         vertexPos = skeleton.dcel().vertex(vertexIdx).pos + (t - skeleton.dcel().vertex(vertexIdx).data.t) *
            skeleton.dcel().vertex(vertexIdx).data.bisector;
      }

      return maximizeRay(grid, cg::segment_2 (vertexPos, vertexPos + direction),
                         maxShift - (t - skeleton.getIterationStartTime()));
   }

   inline cg::range_2 getRectRange( cg::point_2 const &origin,
                                    cg::point_2 const &direction,
                                    cg::rectangle_2 const &rect )
   {
      cg::point_2 corners[4] =
      {
         rect.lo(),
         cg::point_2 (rect.hi().x, rect.lo().y),
         rect.hi(),
         cg::point_2 (rect.lo().x, rect.hi().y)
      };

      double
         minProj = std::numeric_limits< double >::max(),
         maxProj = std::numeric_limits< double >::min();
      for (size_t c = 0; c < 4; c++)
      {
         double curProj = (corners[c] - origin) * direction;
         if (curProj < minProj)
            minProj = curProj;
         if (curProj > maxProj)
            maxProj = curProj;
      }

      return cg::range_2 (minProj, maxProj);
   }

   inline void mergeRangePairRange( cg::range_2 &low, cg::range_2 &high, cg::range_2 const &rangeToAdd )
   {
      if (distance(low | rangeToAdd, high) > distance(high | rangeToAdd, low))
         low |= rangeToAdd;
      else
         high |= rangeToAdd;
   }

   inline bool has_intersection( cg::line_2 const &l, cg::rectangle_2 const &r, cg::segment_2 *out )
   {
      if (cg::eq_zero(l.r().x))
      {
         // Vertical line
         if (r[0].contains(l.p().x))
         {
            if (out != NULL)
               *out = cg::segment_2 (cg::point_2 (l.p().x, r.lo().y), cg::point_2 (l.p().x, r.hi().y));

            return true;
         }

         return false;
      }

      if (cg::eq_zero(l.r().y))
      {
         // Vertical line
         if (r[1].contains(l.p().y))
         {
            if (out != NULL)
               *out = cg::segment_2 (cg::point_2 (r.lo().x, l.p().y), cg::point_2 (r.hi().x, l.p().y));

            return true;
         }

         return false;
      }

      cg::range_2 t;

      double invRx = 1.0 / l.r().x;
      double invRy = 1.0 / l.r().y;

      size_t numIntersections = 0;

      // Left
      double curT = (r.lo().x - l.p().x) * invRx;
      if (r[1].contains(l(curT).y))
      {
         t |= curT;
         numIntersections++;
      }

      // Right
      curT = (r.hi().x - l.p().x) * invRx;
      if (r[1].contains(l(curT).y))
      {
         t |= curT;
         numIntersections++;
      }

      if (numIntersections == 2)
      {
         *out = cg::segment_2 (l(t.lo()), l(t.hi()));
         return true;
      }

      // Bottom
      curT = (r.lo().y - l.p().y) * invRy;
      if (r[0].contains(l(curT).x))
      {
         t |= curT;
         numIntersections++;
      }

      if (numIntersections == 2)
      {
         *out = cg::segment_2 (l(t.lo()), l(t.hi()));
         return true;
      }

      // Top
      curT = (r.hi().y - l.p().y) * invRy;
      if (r[0].contains(l(curT).x))
      {
         t |= curT;
         numIntersections++;
      }

      if (t.empty())
         return false;

      *out = cg::segment_2 (l(t.lo()), l(t.hi()));
      return true;
   }

   inline cg::range_2 getRectRangeForBisector( cg::point_2 const &origin,
                                               cg::point_2 const &direction,
                                               double dirNormSqr,
                                               cg::rectangle_2 const &rect )
   {
      cg::segment_2 res;
      if (!has_intersection(cg::line_2 (origin, direction, cg::line::by_direction), rect, &res))
         return cg::range_2 ();
      
      return cg::range_2 ((res.P0() - origin) * direction / dirNormSqr,
                          (res.P1() - origin) * direction / dirNormSqr);
   }

   struct FindContourNearestIntersectionProcessor
   {
      FindContourNearestIntersectionProcessor ( cg::point_2 const &start )
         : start_ (start)
         , intersectionFound_ (false)
      {
      }

      template< class SmallCell >
         void onProcessingCell( SmallCell const & )
      {
      }

      template< class SegmentId >
         bool onBetterHit ( point_2 const &ipt, SegmentId /* sid */ )
      {
         if (cg::eq(ipt, start_, 1e-8))
            return false;

         intersectionFound_ = true;
         intersectionPoint_ = ipt;
         return true;
      }

      template< class SegmentId >
         bool onEqualHit (SegmentId /* sid */)
      {
         return false;
      }

      bool was_intersection() const
      {
         return intersectionFound_;
      }

      point_2 const &intersectionPoint() const
      {
         return intersectionPoint_;
      }

   private:
      cg::point_2 start_;

      bool intersectionFound_;
      cg::point_2 intersectionPoint_;
   };

} // End of 'skeleton' namespace
} // End of 'cg' namespace
