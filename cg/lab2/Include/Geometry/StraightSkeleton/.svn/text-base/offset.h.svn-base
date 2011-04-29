#pragma once

#include "straight_skeleton.h"
#include "helpers.h"

namespace cg
{

typedef ss::ConstructionParams offset_params;

template< class Skeleton, class VertexBufferOut, class OutContourIterator >
void skeleton_result( Skeleton const & skel, VertexBufferOut &vOut,
                      OutContourIterator contoursOut, bool ignoreHoles = true )
{
   // Move result points to initial region
   size_t vOutBase = vOut.size();
   vOut.reserve(vOutBase + skel.dcel().verticesSize());
   for (size_t v = 0; v != skel.dcel().verticesSize(); ++v)
      vOut.push_back(VertexBufferOut::value_type (skel.vertexPosFixed(v) + skel.basePoint()));

   // Return skeleton segments
   for (typename Skeleton::DCEL::cycle_const_iterator cIt = skel.dcel().cyclesBegin( ignoreHoles );
        cIt != skel.dcel().cyclesEnd( ignoreHoles ); ++cIt)
   {
      std::vector< size_t > offsetContour;

      for (typename Skeleton::DCEL::cycle_edge_const_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
         offsetContour.push_back(eIt->vertexOrigin + vOutBase);

      if (offsetContour.size() >= 3)
         *contoursOut++ = offsetContour;
   }
}

template< class VertexBuffer, class VertexBufferOut, class MultiContourRandomIterator, class OutContourIterator >
void skeleton_offset( VertexBuffer const &vBuffer,         // Initial vertices buffer
                      VertexBufferOut &vOut,               // Vertex Buffer for skeleton
                      MultiContourRandomIterator cntBegin, // Container of containers of indices
                      MultiContourRandomIterator cntEnd,
                      offset_params params,
                      OutContourIterator contoursOut,
                      bool ignoreHoles = true )     // Iterator to receive result contours
{
   typedef StraightSkeletonGenerator< VertexBuffer, MultiContourRandomIterator > Generator;

   // Construct skeleton
   Generator generator (vBuffer, cntBegin, cntEnd, params);
   skeleton_result( generator, vOut, contoursOut, ignoreHoles );
}

template< class ContoursBuffer, class VertexBufferOut, class OutContourIterator >
void skeleton_offset( ContoursBuffer const &contours,      // Initial vertices buffer
                      offset_params params,
                      VertexBufferOut &vOut,               // Vertex Buffer for skeleton
                      OutContourIterator contoursOut,
                      bool ignoreHoles = true )     // Iterator to receive result contours
{
   std::vector< typename ContoursBuffer::value_type::value_type > points;
   std::vector< std::vector< size_t > > indices;

   for ( size_t i = 0, offset = 0; i < contours.size(); offset += contours[i].size(), ++i )
   {
      points.insert( points.end(), contours[i].begin(), contours[i].end() );
      indices.push_back( std::vector< size_t >() );

      size_t const n = contours[i].size();
      indices.back().reserve( n );
      for ( size_t j = 0; j < n; ++j )
         indices.back().push_back( offset + j );
   }

   skeleton_offset( points, vOut, indices.begin(), indices.end(), params, contoursOut, ignoreHoles );
}

// ! Constructs skeleton for each line-contour separately
template< class VertexBuffer, class MultiContourRandomIterator, class OutContourIterator >
void skeleton_side_offset( VertexBuffer const &vBuffer,         // Initial vertices buffer
                           VertexBuffer &vOut,                  // Vertex Buffer for skeleton
                           MultiContourRandomIterator cntBegin, // Container of containers of indices
                           MultiContourRandomIterator cntEnd,   //   each contour - is line
                           offset_params params,
                           OutContourIterator contoursOut,      // Iterator to receive result contours
                           bool leftSide )     
{
   Assert(params.subdivideReflexAngles); // As all contours are lines subdivision must be enabled

   for (MultiContourRandomIterator cont = cntBegin; cont != cntEnd; ++cont)
   {
      // Construct skeleton
      StraightSkeletonGenerator< VertexBuffer, MultiContourRandomIterator >
         generator (vBuffer, cont, util::next(cont), params);

      // Move result points to initial region
      size_t vOutBase = vOut.size();
      vOut.reserve(vOutBase + generator.dcel().verticesSize());
      for (size_t v = 0; v != generator.dcel().verticesSize(); ++v)
         vOut.push_back(generator.vertexPosFixed(v) + generator.basePoint());

      std::vector< size_t > offsetContour;

      if (leftSide)
      {
         for (int vI = cont->size() / 2 - 1; vI >= 0; --vI)
         {
            size_t nextVI = (vI + 1) % cont->size();

            cg::obtain_skeleton_section(generator, 0, (size_t)vI, nextVI,
               std::back_inserter(offsetContour), true, vI != cont->size() / 2 - 1);

            if (offsetContour.size() > 0 && vI != 0)
               offsetContour.pop_back();
         }
         std::reverse(offsetContour.begin(), offsetContour.end());
      }
      else
      {
         for (int vI = cont->size() - 1; vI >= (int)cont->size() / 2; --vI)
         {
            size_t nextVI = (vI + 1) % cont->size();

            cg::obtain_skeleton_section(generator, 0, (size_t)vI, nextVI,
               std::back_inserter(offsetContour), true, vI != cont->size() - 1);

            if (offsetContour.size() > 0 && vI != cont->size() / 2)
               offsetContour.pop_back();
         }
      }

      if (offsetContour.size() >= 2)
      {
         for (size_t v = 0; v < offsetContour.size(); ++v)
            offsetContour[v] += vOutBase;

         *contoursOut++ = offsetContour;
      }
   }
}

// note: finds only first (~random) intersection
template< class Skeleton >
bool has_intersection( Skeleton const & skel, line_2 const & ray, bool ignoreHoles = true,
                       point_2 * out = NULL, size_t * edge_idx = NULL )
{
   line_2 ray_local(ray.p() - skel.basePoint(), ray.r());

   for (typename Skeleton::DCEL::cycle_const_iterator cIt = skel.dcel().cyclesBegin(ignoreHoles);
           cIt != skel.dcel().cyclesEnd(ignoreHoles); ++cIt)
   {
      for (typename Skeleton::DCEL::cycle_edge_const_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
      {
         segment_2 cur_edge_seg(skel.vertexPosFixed(eIt->vertexOrigin), skel.vertexPosFixed(eIt->vertexDestination));
         point_2 isection;
         if (has_intersection(ray_local, line_2(cur_edge_seg.P0(), cur_edge_seg.P1(), line::by_points), isection) &&
             ray_local(isection) > 0 && !(range_2(cur_edge_seg(isection)) & range_2(0, 1)).empty())
         {
            if (out)
               *out = isection + skel.basePoint();
            if (edge_idx)
               *edge_idx = eIt.index();
            return true;
         }
      }
   }

   return false;
}

} // End of 'cg' namespace
