#pragma once

#include "Geometry\rasterization\line_equation.h"

/***********************************************************************
    –астеризаци€ отрезка.                                                                     
    ќпредел€ютс€ различи€ в алгоритме растеризации различи€ 
    в зависимости от направлени€ отрезка.
 ***********************************************************************/

namespace cg
{
    namespace segment_rasterization_2
    {
        struct direction 
        {
            // зависимость от направлени€: по x или по y

            // »змен€етс€ координата x
            struct x  
            {
                // выбор координаты, подлежащей изменению
                int       & select(point_2i & pt)             { return pt.x; }
                int const & select(point_2i const & pt) const { return pt.x; }

                // ќпределение  ординаты пересечени€ пр€мой L с пр€мой x = t
                double      L_by(line_eq::line_equation_2 const &L, int t) const { return L.y_by_x(t); }

                // »зменение y при изменении x на единицу
                double      L_step(line_eq::line_equation_2 const &L)      const { return L.y_step();  }
            };

            // »змен€етс€ координата y
            struct y  
            {
                int       & select(point_2i & pt)             { return pt.y; }
                int const & select(point_2i const & pt) const { return pt.y; }

                // ќпределение  абциссы пересечени€ пр€мой L с пр€мой y = t
                double      L_by(line_eq::line_equation_2 const &L, int t) const { return L.x_by_y(t); }

                // »зменение x при изменении y на единицу
                double      L_step(line_eq::line_equation_2 const &L)      const { return L.x_step();  }
            };

            // зависимость от убывани€ или возврастани€ координаты

            //  оордината возврастает
            struct inc 
            {
                // "сторона", которую будем пересекать
                int    side(int    t) const { return t + 1; }

                // измен€ть ли знак шага
                double sign(double t) const { return t; }

                // приращение координаты
                void advance(int & t)       { ++t; }
            };

            //  оордината убывает
            struct dec 
            {
                int     side    (int    t) const { return  t; }
                double  sign    (double t) const { return -t; }
                void    advance (int &  t)       {       --t; }
            };

            // значени€ типов дл€ отладки
            enum Dir { inc_x, inc_y, dec_x, dec_y };

            struct x_inc : x, inc { enum { value = inc_x }; };
            struct x_dec : x, dec { enum { value = dec_x }; };

            struct y_inc : y, inc { enum { value = inc_y }; };
            struct y_dec : y, dec { enum { value = dec_y }; };

            struct none {};
        };

        // ќпределение координаты выхода пр€мой L из клетки A
        template <class Direction>
            double Ipt_Start(Direction dir, 
                line_eq::line_equation_2 const &L, point_2i const & A)
        {
            return dir.L_by(L, dir.side(dir.select(A)));
        }

        // ќпределение шага пересечений 
        template <class Direction>
            double Ipt_Step(Direction dir, line_eq::line_equation_2 const &L)
        {
            return dir.sign(dir.L_step(L));
        }

    }
}