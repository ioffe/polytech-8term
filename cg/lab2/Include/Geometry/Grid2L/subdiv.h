#pragma once

#include "Geometry\Grid1L\HitCounter.h"
#include "Geometry\grid_params.h"
#include "Geometry\Grid2L\Grid2L_Impl.h"
#include "contours\common.h"

namespace cg
{
   // Класс, который определяет политику подразбиения Grid2L
   // передается аргументом в Grid2L::MakeSubdivision
   // Пусть в некоторый bigcell попало N элементов (определяем с помощью HitCounter)
   // Тогда эта клетка будет подразбита на point_2i ( func(N), func(N) ) ; маленьких ячеек
   template <class Func, class HitCounterT >
   struct SubdivideByHitCount
   {
      SubdivideByHitCount (HitCounterT const &hit_counter, Func const &func)
         :   hit_counter(hit_counter), func(func)
      {}

      point_2i operator() (point_2i const &idx) const
      {
         int div = func(hit_counter[idx]);
         return point_2i(div, div);
      }

      HitCounterT const &hit_counter;
      Func       const &func;
   };

   // Просто выводим тип func
   template <class Func, class HitCounterT >
      SubdivideByHitCount<Func, HitCounterT>  subdivideByHitCount(HitCounterT const &hit_counter, Func const & func)
   {
      return SubdivideByHitCount<Func, HitCounterT> (hit_counter, func);
   }

   // определяет, насколько подразбить большую ячейку, которая содержит n_actual элементов,
   // чтобы в каждом small cell'e было в среднем n_desired элементов
   inline int desired_avg_subdiv (int n_desired, int n_actual)
   {
      return ceil( cg::sqrt( (double)n_actual / n_desired ) );
   }

   struct desired_avg_subdiv_func
   {
      desired_avg_subdiv_func(unsigned desired_avg) 
         :   desired_avg(desired_avg) 
      {}

      int operator () (int actual_elements) const
      {
         return desired_avg_subdiv(desired_avg, actual_elements);
      }

   private:
      unsigned const desired_avg;
   };

   struct desired_avg_subdiv_func_with_lim
   {
      desired_avg_subdiv_func_with_lim (unsigned desired_avg, unsigned limit)
         :   desired_avg_ (desired_avg)
         ,   limit_       (limit)
      {}

      int operator () (int n_actual_elements) const
      {
         return max(1, min(desired_avg_subdiv_func(desired_avg_)(n_actual_elements),limit_));            
      }

   private:
      unsigned const desired_avg_;
      int      const limit_;
   };

   struct FixedSubdivider 
   {
      FixedSubdivider(point_2i const &subdiv) : subdiv(subdiv) {}

      point_2i operator () (point_2i) {
         return subdiv;
      }

      point_2i    subdiv;
   };

   // определяет подразбиение для первого уровня сетки
   // причем старается сделать клетки как можно более квадратными
   // domain - прямоугольник, в котором должна жить сетка
   // n_desired - желаемое среднее элементов в каждой клетке
   // n_actual - общее количество элементов в сетке
   inline point_2i getSubdivisionParam(
      rectangle_2 const &domain, unsigned n_desired, unsigned n_actual)
   {
      if ( n_actual == 0 )
         return point_2i( 1, 1 );
      double domain_width  = domain.x.size();
      double domain_height = domain.y.size();

      const double eps = epsilon<double>( ) ;

      double a_div_b = domain_width / domain_height  +  eps;
      unsigned N = n_actual;
      double   n = n_desired;

      int divx = ceil( cg::sqrt( (double)N / n * a_div_b ) );
      int divy = ceil( divx / a_div_b );

      return point_2i(divx, divy);
   }
   /*
   template <class Point>
   struct AABB_StdTraits
   {
   };
   */
   struct AABB : rectangle_2
   {
      // От traits требуется getBbox(FwdIter) const
      template <class FwdIter, class Traits>
         AABB (FwdIter p, FwdIter q, Traits const & traits)
      {
         while (p != q)
            *this |= traits.getBbox(p++) ;
      }
   };

   // подтягивает src к сетке с началом в org и шагом unit.
   // возвращает закрытый прямоугольник, содержащий все клетки, которые заметаются src
   inline rectangle_2i adjust_to_fixed_grid(
      point_2 const & org, point_2 const & unit, rectangle_2 const & src)
   {
      point_2i xy = floor((src.xy() - org) / unit);
      point_2i XY = ceil((src.XY() - org) / unit);

      return rectangle_2i(xy, XY);
   }

   struct Grid2LSubdiv
   {
      Grid2LSubdiv (rectangle_2 const & aabb, int avg_items_in_bcell, 
         int avg_items_in_scell, int max_bcell_subdivision)
         :   aabb_ (aabb)
         ,   avg_items_in_bcell_ (avg_items_in_bcell)
         ,   avg_items_in_scell_ (avg_items_in_scell)
         ,   max_bcell_subdivision_ (max_bcell_subdivision)
      {}

      virtual grid_params getMainSubdiv (int n_actual) const
      {
         if (n_actual > 0 && !aabb_.empty())
         {
            point_2i ext = getSubdivisionParam(aabb_, avg_items_in_bcell_, n_actual);

            return grid_params (aabb_.xy(), aabb_.size() / ext, ext);
         }
         else
            return grid_params (point_2(0,0), point_2(1,1), point_2i(1,1));
      }

      template <class Grid, class HitCounterT >
         void makeSubdivision (HitCounterT const & hitcounter, Grid & grid) const
      {
         desired_avg_subdiv_func_with_lim func(avg_items_in_scell_, max_bcell_subdivision_);
         grid.MakeSubdivision(subdivideByHitCount(hitcounter, func));
      }

   protected:
      rectangle_2 aabb_;
      int const  avg_items_in_bcell_;
      int const  avg_items_in_scell_;
      int const  max_bcell_subdivision_;
   };

   struct Grid2LSubdiv200m 
   {
      Grid2LSubdiv200m (rectangle_2 const & aabb, int avg_items_in_scell, point_2 cellSize) 
         : aabb_ (aabb)
         , avg_items_in_scell_ (avg_items_in_scell)
         , cellSize_(cellSize)
      {}

      grid_params getMainSubdiv (int) const 
      {
         if ( !aabb_.empty() ) {
            point_2i ext = ceil(aabb_.size() / cellSize_);

            return grid_params (aabb_.lo(), cellSize_, ext);
         }

         return grid_params ( point_2(0, 0), point_2(1, 1), point_2i(1, 1) );
      }

      template <class Grid>
         void makeSubdivision (HitCounter const & hitcounter, Grid & grid) const
      {
         subdivfunc func(avg_items_in_scell_);
         grid.MakeSubdivision(subdivideByHitCount(hitcounter, func));
      }

   private:

      struct subdivfunc
      {
         int operator () (int n_actual) const 
         {
            return max(1, desired_avg_subdiv_func(desired_avg)(n_actual));
         }

         subdivfunc(unsigned desired_avg) : desired_avg (desired_avg) {}

      private:
         unsigned    desired_avg;
      };

      point_2 cellSize_;
      unsigned    avg_items_in_scell_;
      rectangle_2 aabb_;
   };

   template < class Grid, class SegmentId >
      struct SegmentInserter
         : grid2l_visitor_base<Grid, SegmentInserter< Grid, SegmentId >>
   {
      SegmentInserter( contours::segment_id seg_idx)
         : seg_idx( seg_idx )
      {}

      template < class T > 
         SegmentInserter( T t )
            : seg_idx( t )
      {}

      template <class State>
         bool operator () (State const &, smallcell_type & scell) const
      {
         scell.add_segment( seg_idx );
         return false;
      }

   private:
      SegmentId seg_idx;
   };

   template < class Grid, class T >
      SegmentInserter< Grid, T > create_segment_inserter( T t )
   {
      return SegmentInserter< Grid, T >( t );
   }

   template <class Grid>
   struct Grid2LInitializer : Grid
   {
      // От traits требуется T getItem (FwdIter p);
      //                size_t distance(FwdIter p, FwdIter q)
      //                     X makeInserter(FwdIter p)
      // От SubdivisionParams
      //          Y getMainSubdiv   (unsigned n_actual)
      //       void makeSubdivision (HitCounter const & hitcounter, Grid & grid)
      template <class FwdIter, class Traits, class SubdivisionParams>
         Grid2LInitializer (FwdIter p, FwdIter q, Traits const & traits, SubdivisionParams const &params)
         :   Grid (params.getMainSubdiv(traits.distance(p,q)))
      {
         HitCounter hitcounter (*this);

         for (FwdIter t = p; t != q; ++t)
            hitcounter.add(traits.getItem(t));

         params.makeSubdivision(hitcounter, *this);

         for (FwdIter t = p; t != q; ++t)
         {
            SegmentInserter< Grid, FwdIter > inserter = create_segment_inserter< Grid >( t );
            visit(*this, traits.getItem(t), inserter);
         }
      }
   };

}