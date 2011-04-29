#pragma once
#include "Common/m_ptr.h"
#include "Geometry/primitives/range.h"
#include "Geometry/Grid2L.h"
#include "Geometry/Grid1L/HitCounter.h"
#include "Geometry/Grid2L/visit_grid2l_by_circle.h"
#include "Geometry/Grid2L/subdiv.h"
#include "Geometry/triangle_clip_by_rect.h"

/*
Collision3D:

Индексирует множество треугольников для быстрого выполенения запросов 
на пересечение.

От параметров требует:
1. Итератор, который может перебрать входные треугольники
1.1  Индекс треугольника
1.2  triangle_3
2. По id треугольника
2.1  вычисление пересечения треугольника с вертикальным лучом
2.2  то же с произвольным

Сейчас в Column'e хранится вектор индексов
Для оптимизации можно хранить только начальный и конечный индекс в массиве индексов

К Collision3D не относится:
- загрузка точек и треугольников
- вычисление ограничивающего прямоугольника
- придание треугольникам правильной ориентации
- вычисление нормалей и прочих данных для ускорения shootRay
*/


#include "algos.h"

namespace cg
{

   template <class Traits>
      // Колонка, которая хранится в маленькой ячейке
   struct Column 
   {
   private:
      typedef typename Traits::face_id        face_id;

   public:
      typedef typename std::vector<face_id>   Faces;

      range_2 const & zrange() const { return zrange_; }
      range_2       & zrange()       { return zrange_; }

      Faces   const & faces () const { return faces_;  }
      Faces         & faces ()       { return faces_;  }

#ifdef _OPENMP
      Column() { omp_init_lock( &lock ); }
      ~Column() { omp_destroy_lock( &lock ); }
      omp_lock_t lock;
#endif

   private:
      // диапазон высот в данной клетке
      range_2  zrange_;

      // индексы граней, попавших в данную ячейку
      Faces    faces_;
   };

   template <class Traits>
   struct cdt_grid
   {
      typedef Grid2L< m_ptr<Column<Traits> > >  value;
   };

   template <class Traits>
   struct Collision3D 
      :   cdt::algos  < typename cdt_grid<Traits>::value, Traits,Collision3D<Traits> >
   {
      typedef typename Traits::face_id     face_id;


      typedef typename cdt_grid<Traits>::value  Grid_;

   private:

      // ToDo: triangle_raster_aux брать из Traits
      // Поскольку запросы к нему массовые, то
      // просим traits создать объект triangle_raster_aux
      struct ColumnCreator 
      { 
         // функционал, который возвращает для заданного small cell'a 
         // возвращает диапазон высот, заметаемый несущей плоскостью треугольника
         struct height_range
         {
            height_range (Grid_ const &grid, triangle_raster_aux const &face)
            {
               // получаем в глобальных координатах углы большой ячейки
               point_2  xy = grid.local2world(point_2(0,0));
               point_2  xY = grid.local2world(point_2(0,1));
               point_2  Xy = grid.local2world(point_2(1,0));
               point_2  XY = grid.local2world(point_2(1,1));

               range_2  zrange;

               double  h_xy = face.GenerateHeightForPoint(xy);
               double  h_xY = face.GenerateHeightForPoint(xY);
               double  h_Xy = face.GenerateHeightForPoint(Xy);
               double  h_XY = face.GenerateHeightForPoint(XY);

               // засекаем диапазон заметаемой плоскостью высот
               zrange.unite(h_xy);
               zrange.unite(h_xY);
               zrange.unite(h_Xy);
               zrange.unite(h_XY);

               z_range_ = zrange.size();

               delta_ = point_2(h_Xy - h_xy, h_xY - h_xy);

               z_base_ = h_xy;
            }

            range_2 operator () (point_2 const &idx) const
            {
               // note #1
               double z = z_base_ + delta_ * idx;
               return range_2(z, z + z_range_);
            }

         private:
            // перепад высот на одном big cell'e
            double  z_range_;

            // перепад высот по каждой из осей 
            // при смещении на одну большую ячейку 
            point_2  delta_;

            // высота в точке (0,0)
            double   z_base_;
         };

         ColumnCreator(Grid_ &grid, face_id faceid, triangle_3 const &face)
            :   grid_    (grid)
            ,   face_id_ (faceid)
            ,   face_    (face)
            ,   face_aux_(face_)
            ,   zrange_  (grid, face_aux_)
         {}

         struct SideProcessor
         {
            SideProcessor(double h0, double h1)
               :   h0_(h0), h1_(h1)
            {}

            template <class State>
               bool operator () (State const &state, m_ptr<Column<Traits> > &column)
            {
               if (!column)
#ifdef _OPENMP
#pragma omp critical (create_column)
#endif
                  if ( !column )
                     column = new Column<Traits>();

#ifdef _OPENMP
               omp_set_lock( &column->lock );
#endif

               double z0 = clamp(0., 1., h0_, h1_)(state.in_ratio);
               column->zrange().unite(z0);

               double z1 = clamp(0., 1., h0_, h1_)(state.out_ratio);
               column->zrange().unite(z1);               

#ifdef _OPENMP
               omp_unset_lock( &column->lock );
#endif

               return false;
            }

         private:
            double const  h0_, h1_;
         };

         SideProcessor side_processor(int v0, int v1) {
            return SideProcessor(face_[v0].z, face_[v1].z);
         }

         template <class SegmentContructionCellProcessor, class Sides>
            void process_triangle_sides(Grid_ & grid, Sides & sides, triangle_2 const & )
         {
            rectangle_2 bb = bounding(grid);
            bb.inflate( - epsilon<double>( ) );

            std::vector<point_3>  clipped;

            cull(face_, bb, clipped);

            for (size_t i = 0; i != clipped.size(); ++i)
            {
               size_t n = cg::next((int)i, (int)clipped.size());

               SideProcessor side_proc(clipped[i].z, clipped[n].z);
               SegmentContructionCellProcessor proc (sides, side_proc);

               visit(grid, cg::segment_2(clipped[i], clipped[n]), proc);
            }
         }

         template <class State>
            bool operator () (State const &state, m_ptr<Column<Traits> > & column)
         {
            raster_2 const & raster = grid_.bigcellraster(state.big);
            rectangle_2 const cell_bound = raster.domain(state.small);

            if (!column)
#ifdef _OPENMP
#pragma omp critical (create_column)
#endif
               if ( !column )
                  column = new Column<Traits>();

#ifdef _OPENMP
            omp_set_lock( &column->lock );
#endif

            column->faces().push_back(face_id_);

            range_2 & zrange = column->zrange();

            update_zrange(face_aux_, cell_bound.xy(), zrange);
            update_zrange(face_aux_, cell_bound.xY(), zrange);
            update_zrange(face_aux_, cell_bound.Xy(), zrange);
            update_zrange(face_aux_, cell_bound.XY(), zrange);

#ifdef _OPENMP
            omp_unset_lock( &column->lock );
#endif

            return false;
         }

      private:
         Grid_                 &   grid_;
         face_id             const face_id_;
         triangle_3          const face_;    
         triangle_raster_aux const face_aux_;
         height_range        const zrange_;
      };

      struct VColumnCreator
         : grid2l_visitor_base<Grid_, VColumnCreator>
      {
         VColumnCreator(face_id faceid, triangle_3 const &face)
            :   face_id_ (faceid)
            ,   face_    (face)
         {}

         template <class State>
            bool operator () (State const &state, m_ptr<Column<Traits> > & column)
         {
            if (!column)
#ifdef _OPENMP
#pragma omp critical (create_column)
#endif
               if ( !column )
                  column = new Column<Traits>();

#ifdef _OPENMP
            omp_set_lock( &column->lock );
#endif

            column->faces().push_back(face_id_);

            range_2 & zrange = column->zrange();

            // TODO :: написать более интересный zrange, если будет необходимо
            zrange.unite( face_[0].z );
            zrange.unite( face_[1].z );
            zrange.unite( face_[2].z );

#ifdef _OPENMP
            omp_unset_lock( &column->lock );
#endif

            return false;
         }

      private:
         face_id             const face_id_;
         triangle_3          const face_;    
      };

      struct subdivfunc {
         int operator () (int n) const {
            return traits_.subdivision(n);
         }
         subdivfunc(Traits const &traits) : traits_ (traits) {}

      private:
         Traits const & traits_;
      };

   public:

      // Требует от traits: 
      // aa_transform + extents
      // triangle_3 operator [] (face_id);
      // face_id    getFaceId(Iterator)
      // triangle_3 getTriangle(Itertator)
      // int subdivision(int actual_n);
      template <class Iterator>
         Collision3D(Traits const &traits, Iterator p, Iterator q)
         :   traits_ (traits)
         ,   grid_   (traits_.transform(), traits_.extents())
      {
         // Спрашиваем у traits параметры основной сетки
         HitCounter hitcounter (grid_);

         // Подсчитываем число попаданий треугольниов в bigcell'ы
         for ( Iterator t = p; t != q; ++t )
         {
            if ( traits_.is_vertical_triangle( t ) )
               hitcounter.add( traits_.getSegment2( t ) );
            else
               hitcounter.add( traits_.getTriangle2( t ) );
         }

         // Используя функцию из traits
         SubdivideByHitCount<subdivfunc, HitCounter> 
            subdiv(hitcounter, subdivfunc(traits_));

         // разбиваем bigcell'ы
         grid_.MakeSubdivision(subdiv);

         // заносим треугольники в колонки
#ifdef _OPENMP
#pragma omp parallel
#endif
         for (Iterator t = p; t != q; ++t) 
         {
#ifdef _OPENMP
#pragma omp single nowait
#endif
            {
               if ( traits_.is_vertical_triangle( t ) )
               {
                  VColumnCreator creator( traits_.getFaceId(t), traits_.getTriangle(t) );
                  visit(grid_, traits_.getSegment2(t), creator);
               }
               else
               {
                  ColumnCreator creator (grid_, traits_.getFaceId(t), traits_.getTriangle(t));
                  visit(grid_, traits_.getTriangle2(t), creator);
               }
            }
         }
      }

      Collision3D(Traits const & traits) : traits_(traits) {}

      Grid_ const & grid() const { return grid_; }
      Grid_       & grid()       { return grid_; }

      Traits const & traits() const { return traits_; }

      typedef Grid_ grid_type;

   private:
      Traits const & traits_;
      Grid_           grid_;
   };

   template <class Stream, class Traits>
      void write(Stream & stream, typename Column<Traits> const & column)
   {
      write(stream, column.zrange());
      write(stream, column.faces());
   }
}

/*
Построение Collision3d:
1. Спрашиваем, насколько подразбить HitCounter.
Предоставляем пользователю средства для вычисления bbox и подразбиения

2. Запись в HitCounter. Используется только triangle_3

3. Запись в Grid_. Используется triangle_3 и id треугольников
Формирование Column. ColumnBuilder
*/
