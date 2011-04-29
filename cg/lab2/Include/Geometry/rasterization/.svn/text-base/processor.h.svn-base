#pragma   once

#include "direction_proc_base.h"

/*
 *	Растеризация отрезка.
 *  Определяется базовый процессор. 
 *  Своему приемнику он говорит только номер клетки.
 */

namespace cg
{
    // TODO: рассмотреть возможность включения в State  Processor

    // TODO: Попробовать отказаться от nne и т.д. т.е. таскать за собой Master и Slave

    namespace segment_rasterization_2
    {
        struct state 
        {
            struct inside {};
            struct master {};
            struct slave  {};
        };

        template <class Master, class Slave, class State>
            struct  default_processor 
                :   policy_base <Master, Slave, State>
        {

            default_processor(State & state)
                :   policy_base <Master, Slave, State>(state)
            {}

            template <class In>
                bool process(In, state::slave)
            {
                if (processor()(*this))
                    return true;

                slave_advance();

                return false;
            }

            template <class In>
                bool process(In, state::master)
            {
                if (processor()(*this))
                    return true;

                master_advance();

                return false;
            }

            template <class In>
                bool process(In, state::inside)
            {
                return processor()(*this);
            }
        };
    }
}