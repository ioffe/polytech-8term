#pragma once

#include <iterator>

#include "common/util.h"

#include "primitives/point.h"
#include "primitives/rectangle.h"
#include "point_ops.h"

namespace cg {

   namespace details
   {
      template <typename PointsRnaIter,
                typename IndexesFwdIter,
                typename IndexesOutIter,
                typename RightTurnStrictFunction>
      void build_convexhull_chain(PointsRnaIter points_first,
                                  IndexesFwdIter indices_begin,
                                  IndexesFwdIter indices_end,
                                  IndexesOutIter out,
                                  RightTurnStrictFunction rts_function)
      {
         IndexesFwdIter idx = indices_begin;
         IndexesFwdIter last = util::prev(indices_end);

         std::vector<size_t> r;
         r.push_back(*idx++);

         for (; idx != last; ++idx)
         {
            if (!rts_function(points_first[r.back()],
                              points_first[*idx],
                              points_first[*last]))
               continue;

            r.push_back(*idx);

            while (r.size() >= 3
               && !rts_function(points_first[r.end()[-3]],
                                points_first[r.end()[-2]],
                                points_first[r.end()[-1]]))
            {
               r.erase(r.end() - 2);
            }
         }

         std::copy(r.begin(), r.end(), out);
      }
   }

   template <typename PointsRnaIter,
             typename IndexesOutIter,
             typename RightTurnStrictFunction>
   void build_convex_hull(PointsRnaIter points_first,
                          PointsRnaIter points_last,
                          IndexesOutIter out,
                          RightTurnStrictFunction rts_function)
   {
      if (points_first == points_last)
         return;

      std::vector<size_t> indexset(points_last - points_first);
      for (size_t i = 0; i != indexset.size(); ++i )
         indexset[i] = i;

      struct index_compare
      {
         index_compare(PointsRnaIter points_first)
            : points_first(points_first)
         {}

         bool operator() (size_t index1, size_t index2) const
         {
            return points_first[index1] < points_first[index2];
         }

      private:
         PointsRnaIter points_first;
      };

      std::sort(indexset.begin(), indexset.end(), index_compare(points_first));

      if ((points_last - points_first) < 3)
      {
         // create convex hull as degenerate rectangle
         std::copy(indexset.begin(), indexset.end(), out);
         return;
      }

      details::build_convexhull_chain(points_first, indexset.begin(),  indexset.end(), out, rts_function);
      details::build_convexhull_chain(points_first, indexset.rbegin(), indexset.rend(), out, rts_function);
   }

   template <typename PointsRnaIter, typename IndexesOutIter>
   void build_convex_hull(PointsRnaIter points_first,
                          PointsRnaIter points_last,
                          IndexesOutIter out)
   {
      typedef typename std::iterator_traits<PointsRnaIter>::value_type::scalar_type scalar_type;
      build_convex_hull(points_first,
                        points_last,
                        out,
                        &cg::right_turn_strict<scalar_type, scalar_type, scalar_type>);
   }


#pragma pack ( push, 1 )
   struct naa_rect_2
   {
      naa_rect_2()
         : angle ( 0 )
      {}

      double getangle() const
      {
         return -angle;
      }
      rectangle_2 getrect() const
      {
         return rectangle_2( rect.x, range_2(-rect.y.hi(), -rect.y.lo()) );
      }

      rectangle_2 rect;
      double angle;
   };
#pragma pack ( pop )


   inline bool eq( const naa_rect_2 & a, const naa_rect_2 & b )
   {
      return eq( a.angle, b.angle ) && eq( a.rect, b.rect );
   }


   // угол по часовой стрелке, от горизонтальной направленной вправо оси
   template <class Point>
      inline double vector_angle( Point const &p1, Point const &p2 )
   {
      const double my_eps = 1e-20;

      double dy = p2.y - p1.y;
      double dx = p2.x - p1.x;

      if (eq(dx, 0.0, my_eps))
         dx = my_eps;//epsilon/2;//eps10;

      double alpha_tan = cg::abs(dy / dx);
      double alpha = atan( alpha_tan );

      if (dx>=0 && dy>=0) // первая четверть
         alpha = -alpha;
      else
         if (dx>=0 && dy<=0) // вторая четверть
            ;
         else
            if (dx<=0 && dy<=0) // третья четверть
               alpha = pi - alpha;
            else
               //    if (dx<=0 && dy>=0) // четвертая четверть
               alpha = -(pi - alpha);

      return alpha;
   }


   

   // находит not-axis-alined bounding box
   template <class Point>
      //inline void build_naa_rect( std::vector<Point> const &pointset, naa_rect_2 &naa_rect )
      inline void build_naa_rect( Point const *pointset, size_t p_num, naa_rect_2 &naa_rect )
   {
      //Assert( p_num >= 3 );

      // минимальный прямоугольник
      naa_rect.rect = rectangle_2();
      double min_perimetr = FLOAT_ETERNITY;

      // строим выпуклую оболочку
      std::vector<size_t> hull;
      build_convex_hull( pointset, pointset + p_num, std::back_inserter(hull) );

      int N = hull.size();
      if (N<3)
      {
         // находим горизонтальный bounding box и выходим
         for (size_t i=0; i<p_num; i++)
            naa_rect.rect |= point_2( pointset[i].x, -pointset[i].y ) ;
         naa_rect.angle = 0.0;
         return;
      }

      // берем первую сторону выпуклой оболочки, как основу,
      // и находим для нее самую левую, самую правую и самую дальнюю точки
      Point const &p1 = pointset[ hull[0] ];
      Point const &p2 = pointset[ hull[1] ];

      double alpha = vector_angle( p1, p2 );

      double sina = sin(alpha);
      double cosa = cos(alpha);

      double minx =  FLOAT_ETERNITY;
      double maxx = -FLOAT_ETERNITY;
      double miny =  FLOAT_ETERNITY;
      double maxy = -FLOAT_ETERNITY;

      int left    = -1;
      int right   = -1;
      int top     = -1;
      int bottom  = -1;

      for (int i=0; i<N; i++)
      {
         Point const &p = pointset[ hull[i] ];
         double x =   p.x * cosa  +  (-p.y) * sina;
         double y = - p.x * sina  +  (-p.y) * cosa;

         if (minx > x) { minx = x; left   = i; }
         if (maxx < x) { maxx = x; right  = i; }
         if (miny > y) { miny = y; bottom = i; }
         if (maxy < y) { maxy = y; top    = i; }
      }

      // проверяем периметр
      double perimetr = (maxx - minx) + (maxy - miny);
      if (perimetr < min_perimetr)
      {
         min_perimetr = perimetr;
         naa_rect.rect = rectangle_2( range_2(minx, maxx), range_2(miny, maxy) );
         naa_rect.angle = alpha;
      }

      // перебираем все точки выпуклой оболочки по часовой стрелке
      for (int cur = 1; cur<N; cur++ )
      {
         int next = (cur + 1) % N;

         // берем за основу новый отрезок
         Point const &p1 = pointset[ hull[cur ] ];
         Point const &p2 = pointset[ hull[next] ];

         // вычисляем новый угол поворота
         double alpha = vector_angle( p1, p2 );

         double sina = sin(alpha);
         double cosa = cos(alpha);

         // обновляем левую точку
         {
            Point const &p = pointset[ hull[left] ];
            minx =   p.x * cosa  +  (-p.y) * sina;

            do
            {
               int left1 = (left + 1) % N;
               Point const &p = pointset[ hull[left1] ];
               double x =   p.x * cosa  +  (-p.y) * sina;

               if (x < minx) // + eps10)
               {
                  minx = x;
                  left = left1;
               } else
                  break;

            } while (true);
         }

         // обновляем правую точку
         {
            Point const &p = pointset[ hull[right] ];
            maxx =   p.x * cosa  +  (-p.y) * sina;

            do
            {
               int right1 = (right + 1) % N;
               Point const &p = pointset[ hull[right1] ];
               double x =   p.x * cosa  +  (-p.y) * sina;

               if (x > maxx)// - eps10)
               {
                  maxx = x;
                  right = right1;
               } else
                  break;

            } while (true);
         }

         // обновляем нижнюю точку
         {
            Point const &p = pointset[ hull[bottom] ];
            miny = - p.x * sina  +  (-p.y) * cosa;

            do
            {
               int bottom1 = (bottom + 1) % N;
               Point const &p = pointset[ hull[bottom1] ];
               double y = - p.x * sina  +  (-p.y) * cosa;

               if (y < miny)// + eps10)
               {
                  miny = y;
                  bottom = bottom1;
               } else
                  break;

            } while (true);
         }

         // обновляем верхнюю точку
         {
            Point const &p = pointset[ hull[top] ];
            maxy = - p.x * sina  +  (-p.y) * cosa;

            do
            {
               int top1 = (top + 1) % N;
               Point const &p = pointset[ hull[top1] ];
               double y = - p.x * sina  +  (-p.y) * cosa;

               if (y > maxy)// - eps10)
               {
                  maxy = y;
                  top = top1;
               } else
                  break;

            } while (true);
         }

         // проверяем периметр
         double perimetr = (maxx - minx) + (maxy - miny);
         if (perimetr < min_perimetr)
         {
            min_perimetr = perimetr;
            naa_rect.rect = rectangle_2( range_2(minx, maxx), range_2(miny, maxy) );
            naa_rect.angle = alpha;
         }

      } // for (cur)
   }

}