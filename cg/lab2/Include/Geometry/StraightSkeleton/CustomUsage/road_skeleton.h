#pragma once

#include "..\straight_skeleton.h"

namespace cg
{

template< class VertexBuffer, class MultiContourRandomIterator, class OutIndexIterator, class MultiContour >
void constructRoadSkeleton( VertexBuffer const &vBuffer,         // Initial vertices buffer
                            VertexBuffer &vOut,                  // Vertex Buffer for skeleton
                            MultiContourRandomIterator cntBegin, // Container of containers of indices
                            MultiContourRandomIterator cntEnd,
                            OutIndexIterator segmentsOut,        // Iterator to receive result indices
                            MultiContour &correspondence )       // Container of containers of indices of
                                                                 // corresponding vertices
{
   typedef StraightSkeletonGenerator< VertexBuffer, MultiContour::iterator > Generator;

   // Construct skeleton
   Generator generator (vBuffer, cntBegin, cntEnd, ss::ConstructionParams ());

   // Move result points to initial region
   size_t vOutBase = vOut.size();
   vOut.reserve(vOutBase + generator.dcel().verticesSize());
   for (size_t v = 0; v != generator.dcel().verticesSize(); ++v)
      vOut.push_back(generator.vertexPosFixed(v) + generator.basePoint());

   // Return skeleton segments
   for (Generator::DCEL::edges_const_iterator eIt = generator.dcel().edgesBegin(); eIt != generator.dcel().edgesEnd(); ++eIt)
   {
      if (eIt->vertexDestination > eIt->vertexOrigin)
      {
         *segmentsOut++ = eIt->vertexOrigin + vOutBase;
         *segmentsOut++ = eIt->vertexDestination + vOutBase;
      }
   }

   // Return correspondence
   correspondence.resize(cntEnd - cntBegin);
   MultiContourRandomIterator curContour = cntBegin;
   for (MultiContour::iterator mCIt = correspondence.begin(); mCIt != correspondence.end(); ++mCIt, ++curContour)
   {
      mCIt->resize(curContour->end() - curContour->begin());
      for (MultiContour::value_type::iterator cIt = mCIt->begin(); cIt != mCIt->end(); ++cIt)
         *cIt = generator.correspondingVertices(mCIt - correspondence.begin(), cIt - mCIt->begin()).first.vertexId + vOutBase;
   }
}

} // End of 'cg' namespace
