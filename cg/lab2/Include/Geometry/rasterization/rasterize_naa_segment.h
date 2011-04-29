#pragma once

/*
 *	Растеризация косого отрезка (когда происходит переход по главному направлению)
 */

#include "direction_policies.h"
#include "rasterize_aa_segment.h"

namespace cg
{
    namespace segment_rasterization_2
    {        
        // Требования к Policy:
        // bool slave_cond() - стоит ли прекратить приращение второго направления
        // bool slave_cond_B() - то же, только до клетки B
        // 
        // bool master_cond() достигли ли мы последней клетки по главному направлению
        // 
        // bool process(inside, slave) - переход из начальной клетки по второму направлению
        // bool process(inside, master) - переход из начальной клетки по главному направлению
        //
        // bool process(slave, slave) - переход внутри по второму направлению
        // bool process(slave, master) - в этой клетке выход по главному направлению
        // bool process(master, slave) - в этой клетке вход из главного направления
        //
        // bool process(master, inside) - в этой клетке конец, причем вход в него с главного направления
        // bool process(slave , inside) - в этой клетке конец, причем вход в него с второго направления
        //
        // Следующие переходы невозможны в данном алгоритме:
        // bool process(master, master)
        // bool process(inside, inside)


        // TODO: можно оставить только функцию rasterize_segment
        //       В зависимости от свойств master и slave будет выбираться
        //       либо rasterize_naa_segment, либо rasterize_aa_segment, либо rasterize_single_cell

        template <class Policy>
            bool rasterize_naa_segment(Policy t)
        {
            // Для упрощения записи process(..., ...)
            state::master   master;
            state::slave    slave;
            state::inside   inside;

            if (t.slave_cond())
            {
                // inside - x_side
                if (t.process(inside, slave))
                    return true;

                // x_side - x_side
                while (t.slave_cond())
                    if (t.process(slave, slave))
                        return true;

                // x_side - y_side
                if (t.process(slave, master))
                    return true;
            }
            else
            {
                if (t.process(inside, master))
                    return true;
            }

            while (t.master_cond())
            {
                // y_side - x_side
                if (t.process(master,slave))
                    return true;

                // x_side - x_side
                while (t.slave_cond())
                    if (t.process(slave, slave))
                        return true;

                // x_side - y_side
                if (t.process(slave, master))
                    return true;
            }

            // y_side - inside
            if (! t.slave_cond_B())
                return t.process(master,inside);

            // y_side - x_side
            if (t.process(master,slave))
                return true;

            while (t.slave_cond_B())
                if (t.process(slave, slave))
                    return true;

            return t.process(slave, inside);
        }

        template <class Policy, class Master, class Slave>
            bool rasterize_segment(Policy & policy, Master, Slave)
        {
            return rasterize_naa_segment(policy);
        }

        template <class Policy, class Slave>
            bool rasterize_segment(Policy & policy, direction::none, Slave)
        {
            return rasterize_aa_segment(policy);
        }

        template <class Policy>
            bool rasterize_segment(Policy & policy, direction::none, direction::none)
        {
            return rasterize_cell_segment(policy);
        }

        template <class Policy>
            bool rasterize_segment(Policy policy)
        {
            return rasterize_segment(policy, policy.master, policy.slave);
        }

    }
}