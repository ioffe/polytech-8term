#pragma   once

#include "direction_policies.h"

/*
 *	Растеризация отрезка. 
 *  Определяется базовый класс политик обработки направления растеризации
 */

namespace cg
{
    namespace segment_rasterization_2
    {
        template <class Master, class Slave, class State>
            struct policy_base;

        // Базовый класс свойств обработки растеризации
        // для обработки отрезка axis aligned отрезка
        template <class Slave, class State>
            struct policy_base<direction::none, Slave, State> : State 
        {
            typedef direction::none     master_type;
            typedef Slave               slave_type;

            direction::none master;
            Slave           slave;

            // template <class State>
            policy_base(State & state) : State(state) {}

            // Проверка не пора ли остановиться по второму направлению,
            // когда уже достигли конца по главному направлению
            bool   slave_cond_B() const{ 
                return slave.select(A()) != slave.select(B());     
            }

            // приращение по второму направлению
            void   slave_advance()     { 
                slave.advance(slave.select(A())); 
            }
        };

        // TODO: рассмотреть возможность включения ipt, ipt_int, ipt_delta в State
        // TODO: рассмотреть возможность введения методов доступа в State
        // TODO: рассмотреть возможность наследования от State

        // Базовый класс свойств обработки растеризации
        // для обработки косого отрезка
        template <class Master, class Slave, class State>
            struct policy_base : policy_base<direction::none, Slave, State>
        {
            // Координата медленно изменяющаяся
            Master  master;
            // Быстро изменяющаяся
            Slave   slave;

            policy_base(State & state)
                :   policy_base<direction::none, Slave, State> (state)
            {
                // Определяем, когда пересечем сторону клетки по главному направлению
                ipt()       = Ipt_Start(master, L(), A());
                // В какой клетке произойдет выход из главного направления
                ipt_int()   = floor(ipt());

                ipt_delta() = Ipt_Step(master, L());  
                Assert(cg::abs(ipt_delta()) >= 1);
            }

            // Проверка достижения по главному направлению конца растеризации
            bool   master_cond() const { 
                return master.select(A()) != master.select(B()); 
            }

            // Приращение по главному направлению
            void   master_advance()    {
                // Изменяем номер клетки
                master.advance(master.select(A()));  
                // Обновляем условие выхода по главному направлению
                ipt_int() = floor(ipt() += ipt_delta());  
            }

            // Проверка, не пора ли сделать переход по главному направлению
            // в случае, если бежим до выхода из клетки
            bool   slave_cond() const  { 
                return slave.select(A()) != ipt_int(); 
            }

//        protected:
//            // координата следуюшего перехода по главному направлению
//            double  ipt;  
//            // в какой клетке он достигается
//            int     ipt_int;
//            // приращение координаты большого перехода
//            double  ipt_delta; 
        }; 
    }
}