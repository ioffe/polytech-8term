#pragma once

/*  Алгоритм определения, какой подобласти принадлежит точка, используя CDT_Inside
 *  Подобласти не пересекаются и их объединение покрывает весь грид
 *  Алгоритм определяет, какой клетке грида принадлежит точка запроса,
 *  и пускает луч в центр данной клетки.
 *  Если находится пересечение, то за результат берется из него
 *  Если пересечения нет, то за результат берется остров, которому принадлежит центр ячейки.
 *
 *  Derived - класс - наследник
 *   Предоставляет функцию 
 *   Contour_id shootRayInCell(Grid::smallcell_type const & scell, 
 *      point_2 const & from, point_2 const & to, ContourId default_id) const;
 *
 *  Маленькая ячейка предоставляет функцию ContourId centerBelogsTo() const;
 *
 *  NB! На данный момент алгоритм заточен под то, что для каждой точки, попадающей в грид,
 *  будет вызван processor.
 */

namespace cg
{
    namespace cdt
    {
        template <class Derived, class Grid, class ContourId>
            struct cdt_inside_contains2
        {
            typedef Grid grid_type;

            ContourId findContourPointBelongsTo (point_2 const & pt, ContourId default_id) const
            {
                Processor processor(self(), pt);

                cg::visit(self().grid(), pt, processor);

                return processor.result(default_id);
            }

        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }

            struct Processor : grid2l_visitor_base < Grid, Processor >
            {
                template < class State >
                    bool operator () ( State & state, smallcell_type const & scell )
                {
                    point_2 scell_center = smallcell_center(self_.grid(), state);
                    cg::segment_2 ray ( query_pt_, scell_center );

                    result_ = self_.shootRayInCell ( scell, ray, scell.centerBelongsTo() );

                    initialized_ = true;

                    return false;
                }

                Processor (Derived const & sself, point_2 const & query_pt)
                    :   self_ (sself), query_pt_ (query_pt), initialized_ (false)
                {}

                ContourId result (ContourId default_id) const 
                {
                    return initialized_ ? result_ : default_id;
                }

            private:
                Derived const & self_;
                point_2 const & query_pt_;

                ContourId       result_;
                bool            initialized_;
            };
        };
 

        template <class Derived, class Grid, class ContourId>
            struct cdt_inside_contains
        {
            typedef Grid grid_type;

            ContourId findContourPointBelongsTo (point_2 const & pt, ContourId default_id) const
            {
                typedef cg::State < cg::contours::segment_id > CState;
                typedef cg::ShootRay2DContourProcessor < Derived, CState , cg::RIN > CProcessor;
                return findContourPointBelongsTo <CProcessor> (pt, default_id);
            }

            template < class ShootRayProcessor >
                ContourId findContourPointBelongsTo (point_2 const & pt, ContourId default_id) const
            {
                Processor < ShootRayProcessor > processor(self(), pt);

                cg::visit(self().grid(), pt, processor);

                return processor.result(default_id);
            }

            template< class Stream >
               void serialize( Stream & stream ) const
            {
               write( stream, char(123) ); // _dummy_
            }

        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }

            template < class ShootRayProcessor >
                struct Processor : grid2l_visitor_base < Grid, Processor<ShootRayProcessor> >
            {
                template < class State >
                    bool operator () ( State & state, smallcell_type const & scell )
                {
                    point_2 scell_center = smallcell_center(self_.grid(), state);
                    cg::segment_2 ray ( query_pt_, scell_center );

                    ShootRayProcessor shoot_ray_processor ( self_, ray, scell.centerBelongsTo(), self_.defaultContourId() );

                    self_.shootRayInCell ( scell, ray, shoot_ray_processor );

                    result_ = shoot_ray_processor.intersection();
                    initialized_ = true;

                    return false;
                }

                Processor (Derived const & sself, point_2 const & query_pt)
                    :   self_ (sself), query_pt_ (query_pt), initialized_ (false)
                {}

                ContourId result (ContourId default_id) const 
                {
                    return initialized_ ? result_ : default_id;
                }

            private:
                Derived const & self_;
                point_2 const & query_pt_;

                ContourId       result_;
                bool            initialized_;
            };

            // for serialization
            char _dummy_;
        };
    }
}