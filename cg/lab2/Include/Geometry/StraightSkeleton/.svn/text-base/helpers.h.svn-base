#pragma once

#include "straight_skeleton.h"

namespace cg
{

template < class T >
   std::vector< size_t > create_line_indices( std::vector< T > const & v )
{
   size_t const n = v.size();
   if ( n < 2 )
   {
      build_status::error() << "cg::create_line_indices: invalid vertex buffer";
      return std::vector< size_t >();
   }

   std::vector< size_t > ind;
   for ( size_t i = 0; i < n; ++i ) ind.push_back( i );
   for ( size_t i = n - 2; i > 0; --i ) ind.push_back( i );

   return ind;
}

// Returns chunk of skeleton vertices corresponding to the input edge.
// Order: from 'B' vertex to 'A' vertex.

template< class SkeletonType, class OutIterator >
   void obtain_skeleton_section( SkeletonType &skeleton,
                                 size_t contour, size_t vertexA, size_t vertexB, OutIterator out,
                                 bool aFirst = true, bool bFirst = true )
{
   if (skeleton.constructionParams().outside)
   {
      std::swap(vertexA, vertexB);
      std::swap(aFirst, bFirst);
   }

   typedef SkeletonType::VertexCorrespondenceData CorrData;
   CorrData const *endData = &skeleton.correspondingVertices(contour, vertexA);
   CorrData const *startData = &skeleton.correspondingVertices(contour, vertexB);
   Assert(endData->corr.size() > 0 && startData->corr.size() > 0);
   if (startData->contactVertex != -1)
   {
      SkeletonType::MultiContourIdx idx = skeleton.cntIdx(startData->contactVertex);
      Verify(skeleton.isMultiContourVertex(idx));
      startData = &skeleton.correspondingVertices(idx.first, idx.second);
   }

   size_t end = aFirst ? endData->first.vertexId : endData->last.vertexId;
   size_t start = bFirst ? startData->first.vertexId : startData->last.vertexId;

   size_t prev = static_cast< size_t >( -1 );
   if (start == end)
   {
      *out++ = start;
   }
   else
   {
      size_t curEdge = bFirst ? startData->first.nextEdgeId : startData->last.nextEdgeId;
      if (skeleton.dcel().edge(curEdge).hole)
         curEdge = skeleton.dcel().edge(curEdge).prevEdge;
      else
         curEdge = skeleton.dcel().edge(curEdge).twinEdge;

      do
      {
         curEdge = skeleton.dcel().edge(curEdge).nextEdge;
         size_t vOrig = skeleton.dcel().edge(curEdge).vertexOrigin;
         if (prev == -1 || !cg::eq(skeleton.vertexPosFixed(vOrig), skeleton.vertexPosFixed(prev)))
         {
            skeleton.actualizeVertexState(vOrig);
            *out++ = vOrig;
            prev = vOrig;
         }
      } while (skeleton.dcel().edge(curEdge).vertexOrigin != end);
   }
}

template< class SkeletonType, class OutIterator >
   void obtain_skeleton_section( SkeletonType &skeleton,
      typename SkeletonType::CorrespondenceElement const & vertexA,
      typename SkeletonType::CorrespondenceElement const & vertexB, OutIterator out )
{
   size_t end = vertexB.vertexId;
   size_t start = vertexA.vertexId;

   size_t prev = static_cast< size_t >(-1);
   if (start == end)
   {
      *out++ = start;
   }
   else
   {
      size_t curEdge = vertexA.nextEdgeId;
      curEdge = skeleton.dcel().edge(curEdge).prevEdge;
      do
      {
         curEdge = skeleton.dcel().edge(curEdge).nextEdge;
         size_t vOrig = skeleton.dcel().edge(curEdge).vertexOrigin;
         if (prev == -1 || !cg::eq(skeleton.vertexPosFixed(vOrig), skeleton.vertexPosFixed(prev)))
         {
            skeleton.actualizeVertexState(vOrig);
            *out++ = vOrig;
            prev = vOrig;
         }
      } while (skeleton.dcel().edge(curEdge).vertexOrigin != end);
   }
}

template< class SkeletonType, class OutIterator >
   void obtain_skeleton_section_edges( SkeletonType &skeleton, size_t contour,
                                       size_t vertexA, size_t vertexB, OutIterator out,
                                       bool aFirst = true, bool bFirst = true )
{
   if (skeleton.constructionParams().outside)
   {
      std::swap(vertexA, vertexB);
      std::swap(aFirst, bFirst);
   }

   typedef SkeletonType::VertexCorrespondenceData CorrData;
   CorrData const *endData = &skeleton.correspondingVertices(contour, vertexA);
   CorrData const *startData = &skeleton.correspondingVertices(contour, vertexB);
   Assert(endData->corr.size() > 0 && startData->corr.size() > 0);

   if (startData->contactVertex != -1)
   {
      SkeletonType::MultiContourIdx idx = skeleton.cntIdx(startData->contactVertex);
      Verify(skeleton.isMultiContourVertex(idx));
      startData = &skeleton.correspondingVertices(idx.first, idx.second);
   }

   size_t end = aFirst ? endData->first.vertexId : endData->last.vertexId;
   size_t start = bFirst ? startData->first.vertexId : startData->last.vertexId;

   if (start != end)
   {
      size_t curEdge = bFirst ? startData->first.nextEdgeId : startData->last.nextEdgeId;
      if (skeleton.dcel().edge(curEdge).hole)
         curEdge = skeleton.dcel().edge(curEdge).prevEdge;
      else
         curEdge = skeleton.dcel().edge(curEdge).twinEdge;

      do
      {
         curEdge = skeleton.dcel().edge(curEdge).nextEdge;
         size_t vOrig = skeleton.dcel().edge(curEdge).vertexOrigin;
         size_t vDest = skeleton.dcel().edge(curEdge).vertexDestination;
         if (!cg::eq(skeleton.vertexPosFixed(vOrig), skeleton.vertexPosFixed(vDest)))
         {
            skeleton.actualizeVertexState(vOrig);
            skeleton.actualizeVertexState(vDest);
            *out++ = curEdge;
         }
      } while (skeleton.dcel().edge(curEdge).vertexOrigin != end);
   }
}

template < class Skeleton >
   cg::point_2 vertex_to_point( Skeleton const & skeleton, size_t vertex )
{
   return skeleton.vertexPosFixed( vertex ) + skeleton.basePoint();
}

template < class Skeleton >
   size_t edge_origin( Skeleton const & skeleton, size_t e )
{
   return skeleton.dcel().edge( e ).vertexOrigin;
}

template < class Skeleton >
   size_t edge_destination( Skeleton const & skeleton, size_t e )
{
   return skeleton.dcel().edge( e ).vertexDestination;
}

template < class Skeleton >
   cg::point_2 edge_origin_point( Skeleton const & skeleton, size_t edge )
{
   return vertex_to_point( skeleton, edge_origin( skeleton, edge ) );
}

template < class Skeleton >
   cg::point_2 edge_destination_point( Skeleton const & skeleton, size_t edge )
{
   return vertex_to_point( skeleton, edge_destination( skeleton, edge ) );
}

template < class Skeleton, class Processor >
   Processor for_each_edge_points( Skeleton & skeleton, Processor proc )
{
   struct helper
   {
      helper( Skeleton & skeleton, Processor proc )
         : proc_( proc )
         , skel_( skeleton )
      {}

      void operator ()( size_t e )
      {
         cg::point_2 const & p = vertex_to_point( skel_, edge_origin( skel_, e ) );
         cg::point_2 const & q = vertex_to_point( skel_, edge_destination( skel_, e ) );
         proc_( p, q );
      }

      Processor proc() { return proc_; }

   private:
      Skeleton &  skel_;
      Processor   proc_;
   };

   return for_each_edge( skeleton, helper( skeleton, proc ) ).proc();
}

template < class Skeleton, class Processor >
   Processor for_each_edge( Skeleton & skeleton, Processor proc )
{
   typedef Skeleton::DCEL::edges_const_iterator EIter;
   for (EIter eIt = skeleton.dcel().edgesBegin(); eIt != skeleton.dcel().edgesEnd(); ++eIt)
   {
      if (eIt->vertexDestination > eIt->vertexOrigin)
         proc( *eIt );
   }

   return proc;
}

template< class Skeleton >
   struct SearchParentPredicate
{
   SearchParentPredicate ( Skeleton const &skeleton )
      : skel_ (skeleton)
   {
   }

   bool operator () ( size_t idx )
   {
      return skel_.isMultiContourVertex(skel_.cntIdx(idx));
   }

private:
   Skeleton const &skel_;
};

template< class Skeleton >
   size_t get_skeleton_vertex_parent_cnt( Skeleton const &skel, size_t vIdx )
{
   size_t resVertex = cg::dcel::vertexDFS(skel.dcel(),
      SearchParentPredicate< Skeleton > (skel), vIdx);

   if (resVertex == -1)
      return static_cast< size_t >( -1 );

   return skel.cntIdx(resVertex).first;
}

}