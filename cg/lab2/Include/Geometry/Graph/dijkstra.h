#pragma   once

#include "graph.h"

#include <vector>
#include <map>
#include <algorithm>

#define USE_NLOGN_DIJKSTRA

namespace cg
{
// ���������� false, ���� ���� �� ����� ���� ������ (���� ���������)
template < class Graph, class OutputIterator, class WeightFunc >
   bool dijkstra( Graph const & G, size_t from, size_t to, OutputIterator out, WeightFunc weightfunc )
{
   if (to==from)
   {
      *out = to;
      ++out;
      return true;
   }

   // ����� ������ � �����
   int const N = G.count_vertices();

   const double max_double = std::numeric_limits<double>::max();

   // ������ ������� �����
   std::vector<double>  w(N, max_double);
   // ������ ���������� ������
   std::vector<size_t>  p(N, Graph::INVALID_IDX);
   // ������� �� �������
   std::vector<int   >  f(N, false);

#ifdef USE_NLOGN_DIJKSTRA
   typedef std::multimap<double, int>  Weights;

   // ������������� �� ���� �������
   Weights     weightsOfNodes; 

   // ��� ������ ������� �� ��������� � ����������� ����-->�������
   std::vector<Weights::iterator>  nodeInWeights(N);

   for (size_t i = 0; i != N; ++i)
   {
      nodeInWeights[i] = 
         weightsOfNodes.insert(std::make_pair(w[i], i));
   }
#endif

   size_t node = to;

   f[to] = true;

   double w_node = 0;

   do {
      // ��������� �� ������� � �������� ������
      for (typename Graph::adj_iterator e_it = G.adj_edges(node); e_it; ++e_it)
      {
         // ��� ������ �������� �����
         size_t  y = e_it.to();

         // ��������� ��� ���
         double  weight = weightfunc(e_it);

         if (!f[y] && w[y] > w_node + weight)
         {
            w[y] = w_node + weight;

#ifdef USE_NLOGN_DIJKSTRA
            weightsOfNodes.erase(nodeInWeights[y]);

            nodeInWeights[y] = 
               weightsOfNodes.insert(std::make_pair(w[y], y));
#endif

            p[y] = node;
         }
      }

      // ���������� ����� �������� ������ ������� � ����������� �����

#ifdef USE_NLOGN_DIJKSTRA

      if (weightsOfNodes.empty())
         break;

      size_t bestnode = weightsOfNodes.begin()->second;
      double    wb       = weightsOfNodes.begin()->first;

      weightsOfNodes.erase(weightsOfNodes.begin());

#else

      double wb = std::numeric_limits<double>::max();
      size_t bestnode = node;

      for (size_t i = 0; i < N; ++i)
      {
         if (!f[i] && w[i] < wb) {
            bestnode = i; wb = w[i];
         }
      }
#endif

      f[bestnode] = true;

      if (bestnode == node)
         break;

      node    = bestnode;
      w_node  = wb;


   } while (true);

   node = from;

   do 
   {
      *out = node;
      ++out;
      node = p[node];
      if (node == Graph::INVALID_IDX) // ���� ���
         return false;

      //            Assert( node != -1 );
   } while (node != to);

   *out = to;
   ++out;
   return true;
}

} // namespace cg 