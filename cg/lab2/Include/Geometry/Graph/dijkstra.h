#pragma   once

#include "graph.h"

#include <vector>
#include <map>
#include <algorithm>

#define USE_NLOGN_DIJKSTRA

namespace cg
{
// возвращает false, если путь не может быть найден (граф несвязный)
template < class Graph, class OutputIterator, class WeightFunc >
   bool dijkstra( Graph const & G, size_t from, size_t to, OutputIterator out, WeightFunc weightfunc )
{
   if (to==from)
   {
      *out = to;
      ++out;
      return true;
   }

   // число вершин в графе
   int const N = G.count_vertices();

   const double max_double = std::numeric_limits<double>::max();

   // массив текущих весов
   std::vector<double>  w(N, max_double);
   // массив предыдущих вершин
   std::vector<size_t>  p(N, Graph::INVALID_IDX);
   // закрыта ли вершина
   std::vector<int   >  f(N, false);

#ifdef USE_NLOGN_DIJKSTRA
   typedef std::multimap<double, int>  Weights;

   // упорядоченные по весу вершины
   Weights     weightsOfNodes; 

   // Для каждой вершины ее положение в отображении веса-->вершины
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
      // пробегаем по смежным с вершиной ребрам
      for (typename Graph::adj_iterator e_it = G.adj_edges(node); e_it; ++e_it)
      {
         // для кажого смежного ребра
         size_t  y = e_it.to();

         // вычисляем его вес
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

      // определяем среди открытых вершин вершину с минимальным весом

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
      if (node == Graph::INVALID_IDX) // пути нет
         return false;

      //            Assert( node != -1 );
   } while (node != to);

   *out = to;
   ++out;
   return true;
}

} // namespace cg 