#pragma once

#include "Grid2L_Impl.h"
#include <geometry\xmath.h>
#include "common\assert.h"
#include "segment_clip_by_rect.h"

namespace cg
{
   inline void correct_left(point_2 &pt, point_2 const &, point_2 const & epsilon)
   {
      Assert( epsilon.x > 0 );
      if (eq(pt.x, 0.))  { 
         pt.x = epsilon.x;
         pt.y += epsilon.y;
      }
   }
   inline void correct_right(point_2 &pt, point_2 const &bound, point_2 const & epsilon)
   {
      Assert( epsilon.x > 0 );
      if (eq(pt.x, bound.x)) {  
         pt.x -= epsilon.x;
         pt.y -= epsilon.y;
      }
   }

   inline void correct_top(point_2 &pt, point_2 const &, point_2 const & epsilon)
   {
      Assert( epsilon.y > 0 );
      if (eq(pt.y, 0.))    {
         pt.y = epsilon.y;
         pt.x += epsilon.x;
      }
   }
   inline void correct_bottom(point_2 &pt, point_2 const &bound, point_2 const & epsilon)
   {
      Assert( epsilon.y > 0 );
      if (eq(pt.y, bound.y)) {  
         pt.y -= epsilon.y;
         pt.x -= epsilon.x;
      }
   }

   inline void correct(point_2 &pt, point_2 const &bound, double epsilon)
   {
      if (le(pt.x, 0.)) 
         pt.x = epsilon;

      if (le(pt.y, 0.))    
         pt.y = epsilon;

      if (ge(pt.x, bound.x))   
         pt.x = bound.x - epsilon;

      if (ge(pt.y, bound.y))  
         pt.y = bound.y - epsilon;
   }

   inline void correct (point_2 &pt0, point_2 &pt1, point_2 const & bound)
   {
      const double eps = epsilon<double>( ) ;

      if ( eq( pt0, pt1 ) )
      {
         correct( pt0, bound, eps*1.e4 );
         pt1 = pt0;
         return;
      }

      point_2 epsilon ( (pt1 - pt0) / 100 );
      if ( pt0.x < pt1.x )
      {
         correct_left ( pt0, bound, epsilon );
         correct_right( pt1, bound, epsilon );
      }
      else if ( pt1.x < pt0.x )
      {
         correct_left ( pt1, bound, -epsilon );
         correct_right( pt0, bound, -epsilon );
      }
      if ( pt0.y < pt1.y )
      {
         correct_top   ( pt0, bound, epsilon );
         correct_bottom( pt1, bound, epsilon );
      }
      else if ( pt1.y < pt0.y )
      {
         correct_top   ( pt1, bound, -epsilon );
         correct_bottom( pt0, bound, -epsilon );
      }
   }

   // ќбход 2-х уровнего грида вдоль отрезка
   // T - тип элемента в сетке
   template <class G>
   struct visit_grid2l_by_segment
   {
      typedef G                       grid_type;
      typedef typename G::smallcell_type       smallcell_type;
      typedef typename G::bigcell_type         bigcell_type;
      typedef typename G::index_type           index_type;
      typedef point_2i                bigidx_type;
      typedef point_2i                smallidx_type;

      struct big_state : bigidx_type
      {
         big_state(bigidx_type const & idx, grid_type const & g)
            :   bigidx_type (idx), grid_ (g)
         {}

         grid_type const & grid() const { return grid_; }

      private:
         grid_type const & grid_;
      };

      //  ѕредставл€ет состо€ние обхода
      struct state : index_type
      {
         state(bigidx_type   const &idx_big, 
            smallidx_type const &idx_small, 
            double in_ratio, double out_ratio, 
            grid_type const & grid)

            :   index_type(idx_big, idx_small)
            ,   in_ratio  (in_ratio)
            ,   out_ratio (out_ratio)
            ,   grid_     (grid)
         {}

         double in_ratio;
         double out_ratio;

         grid_type const & grid() const { return grid_; }

      private:
         grid_type const & grid_;
      };

      // Ётот функтор вызываетс€ каждый раз,
      //  когда rasterize_segment заходит в очередную маленькую клетку
      template <class ActualProcessor>
      struct SmallCellProcessor
      {
         // вызываетс€ rasterize_segment
         bool operator () (point_2i const &idx_small, double in_ratio, double out_ratio)
         {
            // rasterize_segment работает на отклипованном отрезке, 
            // а actual_processor ожидает ratio на оригинальном отрезке
            if (bigcell.contains(idx_small))
            {
               in_ratio  = scaler(in_ratio);
               out_ratio = scaler(out_ratio);

               cg::make_min(in_ratio, out_ratio);

               state st(idx_big, idx_small, in_ratio, out_ratio, grid_);

               return actual_processor(st, bigcell.at(idx_small));
            }
            return false;
         }

         SmallCellProcessor(
            ActualProcessor    & actual_processor, 
            bigidx_type const  & idx_big,
            bigcell_type       & bigcell,
            range_2     const  & ratio_range,
            grid_type   const  & grid)

            :   actual_processor (actual_processor)
            ,   idx_big          (idx_big)
            ,   bigcell          (bigcell)
            ,   scaler           (0,1 , ratio_range.lo(), ratio_range.hi())
            ,   grid_            (grid)
         {}

         ActualProcessor    & actual_processor;
         bigidx_type const  & idx_big;
         bigcell_type       & bigcell;
         grid_type    const & grid_;
         Lerp<double> const   scaler;
      };

      // Ётот функтор вызываетс€ каждый раз,
      //  когда rasterize_segment заходит в очередную большую клетку
      template <class ActualProcessor>
      struct BigCellProcessor
      {
         bool operator () (point_2i const &idx_big, double in_ratio, double out_ratio)
         {
            if (!grid.is_valid(idx_big))
               return false;

            bigcell_type & bigcell = grid[idx_big];

            actual_processor.processbigcell(big_state(idx_big, grid), bigcell);

            //                NTRACE("idx_big(%d, %d), grid.ext(%d, %d)", idx_big.x, idx_big.y, grid.extents().x, grid.extents().y );

            if (bigcell_type& bigcell = grid[idx_big])
            {
               SmallCellProcessor<ActualProcessor>  
                  processor(actual_processor, idx_big, bigcell, range_2(scaler(in_ratio), scaler(out_ratio)), grid);

               cg::make_min(out_ratio, 1.);

               // обрезаем отрезок и переводим в с-му координат bigcell'a

               // правильно надо сначала переводить, а потом клиповать
               // так просто решаетс€ вопрос с naabb
               //                segment_2   clipped(cull(segment, grid.principal_raster().domain(idx_big)));

               //                NTRACE("clipped = (%f,%f,%f,%f)", clipped.P0().x, clipped.P0().y, clipped.P1().x, clipped.P1().y);

               raster_2   bc_raster = grid.bigcellraster(idx_big);

               segment_2 translated = segment_2( bc_raster.translate(segment.P0()), bc_raster.translate(segment.P1()) ); 
               segment_2 clipped;

               if (cg::cull(translated, 
                  rectangle_2(point_2(), bc_raster.extents()), clipped))
               {
                  point_2 pt0 = clipped.P0();
                  point_2 pt1 = clipped.P1();

                  //                NTRACE("translated = (%f,%f,%f,%f)", pt0.x, pt0.y, pt1.x, pt1.y);

                  //                        // делаем так, чтобы концы отрезка лежали строго внутри большой клетки
                  //                        correct(pt0, pt1, bc_raster.extents());
                  //
                  //                        correct(pt0, bc_raster.extents(), epsilon);
                  //                        correct(pt1, bc_raster.extents(), epsilon);
                  //                        //correct(pt1, bc_raster.extents());
                  //
                  //                NTRACE("corrected = (%f,%f,%f,%f)", pt0.x, pt0.y, pt1.x, pt1.y);

                  //                segment_2 translated(pt0, pt1);

                  return rasterize_segment(segment_2(pt0, pt1), processor);
               }
            }

            return false;
         }

         BigCellProcessor(
            ActualProcessor & actual_processor,
            grid_type       & grid,
            segment_2 const & segment,
            cg::range_2 const & ratio_range)

            :   actual_processor (actual_processor)
            ,   grid             (grid) 
            ,   segment          (segment)
            ,   scaler           (0, 1, ratio_range.lo(), ratio_range.hi())
         {}

         ActualProcessor  & actual_processor;
         grid_type        & grid;
         segment_2 const  & segment;
         cg::Lerp<double>   const scaler;
      };

      template <typename Processor>
         static bool process(grid_type & grid, segment_2 const &s, Processor & processor)
      {
         // ToDo: сделать поддержку naabb
         segment_2 clipped;
         cull(s, bounding(grid), clipped);

         BigCellProcessor<Processor>  
            bigcell_processor (processor, grid, clipped, cg::range_2(s(clipped.P0()), s(clipped.P1())));

         segment_2 translated (
            grid.world2local(clipped.P0()), 
            grid.world2local(clipped.P1())
            );

         return rasterize_segment(translated, bigcell_processor);
      }
   };

   template <class T, class B, class Processor, class Scalar>
      inline bool visit(Grid2L<T,B> & grid, segment_t< Scalar, 2 > const &s, Processor &processor)
   {
      return visit_grid2l_by_segment<Grid2L<T,B> >::process(grid, cg::segment_2( s.P0(), s.P1() ), processor);
   }

   template <class T, class B, class Processor, class Scalar>
      inline bool visit(Grid2L<T,B> const & grid, segment_t< Scalar, 2 > const &s, Processor &processor)
   {
      return visit_grid2l_by_segment<Grid2L<T,B> >::process(const_cast<Grid2L<T,B>&>(grid), cg::segment_2( s.P0(), s.P1() ), processor);
   }
}