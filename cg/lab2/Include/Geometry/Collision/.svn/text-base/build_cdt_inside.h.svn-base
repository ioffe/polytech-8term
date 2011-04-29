#pragma once

/*
алгоритм, записывающий в каждую €чейку растра, какой подобласти (контуру)
принадлежит центр €чейки. ѕредполагаетс€, что подобласти не пересекаютс€ 
и их объединение покрывает весь грид.

јлгоритм при посещении очередной клетки пускает луч в центр предыдущей €чейки
если пересечени€ не было, беретс€ ее значение, если было - то определ€ем по нему тип подобласти

ContourId - тип, значени€ которого идентифицируют контур

ShootRayObject - объект, умеющий бросать лучи
ContourId ShootRayObject::operator () (point_2, point_2, ContourId default_id) const;
default_id - id контура, который будет возвращен в случае отсутстви€ пересечени€

Grid - сетка, у которой дл€ маленькой €чейки определен метод void belongsTo(ContourId)

Ёффективность алгоритма сильно зависит от пор€дка обхода грида.
„ем меньше рассто€ние между центрами последовательных €чеек, тем алгоритм более эффективен
ѕо умолчанию можно использовать cg::visit_every_cell
*/

namespace cg
{
    namespace cdt
    {
        template <class Grid, class ShootRayObject, class ContourId>
            struct CDT_Inside_Builder : grid2l_visitor_base<Grid, CDT_Inside_Builder<Grid, ShootRayObject, ContourId> >
        {
            // ¬ конструкторе задаетс€ точка, дл€ которой известно, какому контуру она принадлежит
            CDT_Inside_Builder (Grid & grid, ShootRayObject const & shootray, 
                point_2 const & prev_pt, ContourId const & prev_id)
                :   prev_      (prev_pt, prev_id)
                ,   grid_      (grid)
                ,   shootray_  (shootray)
            {}

            template <class Location>
                bool operator () (Location const & loc, smallcell_type & scell)
            {
                point_2 scell_center = smallcell_center(grid_, loc);

                scell.setBelongsTo( prev_.id = shootray_(scell_center, prev_.pt, prev_.id) );

                prev_.pt = scell_center;

                return false;
            }

        private:

            // состо€ние в предыдущем запросе
            struct State 
            {
                point_2     pt;
                ContourId   id;

                State (point_2 const & pt, ContourId id) : pt (pt), id(id) {}
            } prev_;

            Grid          & grid_;
            ShootRayObject  shootray_;
        };
    }
}
