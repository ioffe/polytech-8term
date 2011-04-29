#pragma   once

#include "select_direction.h"
#include "state.h"
#include "processor.h"

namespace cg
{
    namespace segment_rasterization_2
    {
        template <class Processor>
            bool rasterize_segment_ex(point_2 const & from, point_2 const &to, Processor & proc)
        {
            states::proc_mix<states::basic, Processor> st0(states::basic(from, to), proc);

            if (st0.A().x == st0.B().x)
                return select_x_axis<default_processor>(st0);

            if (st0.A().y == st0.B().y)
                return select_y_axis<default_processor>(st0);

            states::proc_mix<states::basic_naa, Processor> st1(st0, proc);

            return select_naa<default_processor>(st1);
        }
    }
}