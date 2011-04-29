#pragma once

#include "Geometry\Grid1L\HitCounter.h"
#include "Geometry\beam.h"

#include "HitCounter.h"
#include "visit_grid1l_by_beam.h"

namespace cg {
namespace skeleton {

   template< class Grid >
   struct SkeletonGridInitializer : Grid
   {
      template< class Skeleton, class CdtGrid, class SubdivisionParams >
         SkeletonGridInitializer ( Skeleton &skeleton, CdtGrid &grid, SubdivisionParams const &params, double maxShift )
            : Grid (params.getMainSubdiv((int)(skeleton.dcel().nonHoleEdgesSize())))
      {
         SkeletonHitCounter< CdtGrid > hitcounter (grid, *this);

         typedef Skeleton::DCEL::edges_const_iterator edges_iterator;
         for (edges_iterator eIt = skeleton.dcel().edgesBegin(); eIt != skeleton.dcel().edgesEnd(); ++eIt)
         {
            if (!eIt->hole)
               hitcounter.add(edge2beam(skeleton, eIt.index()), maxShift);
         }

         params.makeSubdivision(hitcounter, *this);

         /*
         for (size_t e = skeleton.dcel().firstEdgeElem(); e != -1; e = skeleton.dcel().nextEdgeElem())
         {
            if (!skeleton.dcel().edge(e).hole)
            {
               visit(*this, beam (skeleton.edgeSegment(e),
                  skeleton.dcel().vertex(skeleton.dcel().edge(e).vertexOrigin).data.bisector,
                  skeleton.dcel().vertex(skeleton.dcel().edge(e).vertexDestination).data.bisector),
                  .../* traits.makeInserter(t) *);
            }
         }
         */
      }
   };

} // End of 'skeleton' namespace
} // End of 'cg' namespace
