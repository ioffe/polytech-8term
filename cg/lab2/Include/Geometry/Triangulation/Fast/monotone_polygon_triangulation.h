#pragma once

#include <stack>

#include "Geometry/dcel/dcel.h"

namespace cg
{

template< class EdgeList, class OutputIterator >
struct MonoPolyTriangulator
{
   typedef typename EdgeList::point_type PointT;
   typedef typename OutputIterator::container_type::value_type OutputIndexT;

   MonoPolyTriangulator ( EdgeList const *dcel, OutputIterator output ) : dcel_ (dcel), out_ (output)
   {
   }

   bool operator () ( size_t startEdge )
   {
      if (dcel_ == NULL)
         return false;

      std::stack< ChainVertex > chain;

      // Find top vertex
      size_t curEdge = startEdge, incidentEdge = startEdge;
      size_t curVertex = dcel_->edge(curEdge).vertexOrigin;
      do
      {
         if (dcel_->edge(curEdge).vertexDestination < curVertex)
         {
            curVertex = dcel_->edge(curEdge).vertexDestination;
            incidentEdge = dcel_->edge(curEdge).nextEdge;
         }

         curEdge = dcel_->edge(curEdge).nextEdge;
      } while (dcel_->edge(curEdge).nextEdge != startEdge);

      // Iterate all vertices from top to bottom
      size_t leftEdge = incidentEdge;
      size_t rightEdge = dcel_->edge(leftEdge).prevEdge;

      chain.push(ChainVertex (curVertex));
      curVertex = nextVertex(leftEdge, rightEdge);
      chain.push(ChainVertex (curVertex, vertexEdgeAdjacency(*dcel_, curVertex, leftEdge)));

      while (!edgesAdjacency(*dcel_, leftEdge, rightEdge))
      {
         size_t prevCurVertex = curVertex;
         curVertex = nextVertex(leftEdge, rightEdge);

         if (curVertex == prevCurVertex)
         {
            // Polygon isn't monotone!
            return false;
         }

         ChainVertex chainVertex (curVertex, vertexEdgeAdjacency(*dcel_, curVertex, leftEdge));

         ChainVertex topInChain = chain.top();
         ChainVertex prevChainVertex = topInChain;

         chain.pop();

         if (chainVertex.leftChain != topInChain.leftChain)
         {
            while (!chain.empty())
            {
               ChainVertex curChainVertex = chain.top();
               addTriangle(chainVertex, prevChainVertex, curChainVertex);
               prevChainVertex = curChainVertex;
               chain.pop();
            }

            chain.push(topInChain);
         }
         else
         {
            while (!chain.empty())
            {
               ChainVertex curChainVertex = chain.top();
               if (!isTripleConvex(chainVertex, prevChainVertex, curChainVertex))
                  break;
               addTriangle(chainVertex, curChainVertex, prevChainVertex);
               prevChainVertex = curChainVertex;
               chain.pop();
            }
            chain.push(prevChainVertex);
         }

         chain.push(chainVertex);
      }

      curVertex = nextVertex(leftEdge, rightEdge);
      ChainVertex chainVertex (curVertex, vertexEdgeAdjacency(*dcel_, curVertex, leftEdge));

      ChainVertex prevChainVertex = chain.top();
      chain.pop();
      while (!chain.empty())
      {
         ChainVertex curChainVertex = chain.top();
         addTriangle(prevChainVertex, chainVertex, curChainVertex);
         prevChainVertex = curChainVertex;
         chain.pop();
      }

      return true;
   }

private:
   struct ChainVertex
   {
      ChainVertex ( size_t vertexId, bool isOnLeftChain = true ) : id (vertexId), leftChain (isOnLeftChain)
      {
      }

      size_t id;
      bool leftChain;
   };
   
   void addTriangle( ChainVertex const &a, ChainVertex const &b, ChainVertex const &c )
   {
      if (a.leftChain)
      {
         *out_++ = (OutputIndexT)dcel_->vertex(a.id).data.vertexIdx;
         *out_++ = (OutputIndexT)dcel_->vertex(b.id).data.vertexIdx;
         *out_++ = (OutputIndexT)dcel_->vertex(c.id).data.vertexIdx;
      }
      else
      {
         *out_++ = (OutputIndexT)dcel_->vertex(a.id).data.vertexIdx;
         *out_++ = (OutputIndexT)dcel_->vertex(c.id).data.vertexIdx;
         *out_++ = (OutputIndexT)dcel_->vertex(b.id).data.vertexIdx;
      }
   }

   bool isTripleConvex( ChainVertex const &a, ChainVertex const &b, ChainVertex const &c )
   {
      PointT aV = dcel_->vertex(a.id).pos;
      PointT bV = dcel_->vertex(b.id).pos;
      PointT cV = dcel_->vertex(c.id).pos;

      if (b.leftChain)
         return robust_right_turn(bV - aV, cV - bV);
      else
         return robust_left_turn(bV - aV, cV - bV);
   }

private:
   size_t nextEdgeDown( size_t curEdge, size_t nextVertex )
   {
      if (nextVertex == dcel_->edge(curEdge).vertexDestination)
         return dcel_->edge(curEdge).nextEdge;
      else
         return dcel_->edge(curEdge).prevEdge;
   }

   size_t nextVertex( size_t &leftEdge, size_t &rightEdge )
   {
      size_t nextLeftVertex = cg::max((unsigned)dcel_->edge(leftEdge).vertexOrigin, (unsigned)dcel_->edge(leftEdge).vertexDestination);
      if (cg::dcel::edgeVector(*dcel_, leftEdge).y == 0)
         nextLeftVertex = dcel_->edge(leftEdge).vertexDestination;

      size_t nextRightVertex = cg::max((unsigned)dcel_->edge(rightEdge).vertexOrigin, (unsigned)dcel_->edge(rightEdge).vertexDestination);
      if (cg::dcel::edgeVector(*dcel_, rightEdge).y == 0)
         nextRightVertex = dcel_->edge(rightEdge).vertexOrigin;

      if (nextLeftVertex < nextRightVertex)
      {
         leftEdge = nextEdgeDown(leftEdge, nextLeftVertex);
         return nextLeftVertex;
      }
      else
      {
         rightEdge = nextEdgeDown(rightEdge, nextRightVertex);
         return nextRightVertex;
      }
   }

private:
   EdgeList const *dcel_;
   OutputIterator out_;
};

///////////////////////////////////////////////////////////////////////////////

namespace triangulation
{

template< class EdgeList, class OutputIterator >
bool mono_poly( EdgeList const *dcel, size_t startEdge, OutputIterator out )
{
   return MonoPolyTriangulator< EdgeList, OutputIterator > (dcel, out)(startEdge);
}

} // End of 'triangulation' namespace

} // End of 'cg' namespace
