#pragma   once

#include "rasterize_naa_segment.h"

namespace cg
{
    namespace segment_rasterization_2
    {
        template <template <class, class, class> class Policy, class State>
            bool select_naa(State & state)
        {
            point_2i        & A = state.A();
            point_2i const  & B = state.B();

            point_2  const  & from = state.from();
            point_2  const  & to   = state.to();

            Assert(A.x != B.x);
            Assert(A.y != B.y);

            return
                A.x < B.x 
                ?   A.y < B.y
                    ?   to.x - from.x > to.y - from.y
                        ?   rasterize_segment(Policy<direction::y_inc, direction::x_inc, State>(state))
                        :   rasterize_segment(Policy<direction::x_inc, direction::y_inc, State>(state))
                    :   to.x - from.x > from.y - to.y
                        ?   rasterize_segment(Policy<direction::y_dec, direction::x_inc, State>(state))
                        :   rasterize_segment(Policy<direction::x_inc, direction::y_dec, State>(state))
                :   A.y < B.y
                    ?   from.x - to.x > to.y - from.y
                        ?   rasterize_segment(Policy<direction::y_inc, direction::x_dec, State>(state))
                        :   rasterize_segment(Policy<direction::x_dec, direction::y_inc, State>(state))
                    :   from.x - to.x > from.y - to.y
                        ?   rasterize_segment(Policy<direction::y_dec, direction::x_dec, State>(state))
                        :   rasterize_segment(Policy<direction::x_dec, direction::y_dec, State>(state));
        }

        // Отрезок вертикальный. 
        // Выбор: вверх, вниз или на месте
        template <template <class, class, class> class Policy, class State>
            bool select_x_axis(State & state)
        {
            point_2i        & A = state.A();
            point_2i const  & B = state.B();

            Assert(A.x == B.x);

            return 
                A.y == B.y 
                    ?   rasterize_segment(Policy<direction::none, direction::none, State>(state))
                    :   A.y < B.y
                        ?   rasterize_segment(Policy<direction::none, direction::y_inc, State>(state))   
                        :   rasterize_segment(Policy<direction::none, direction::y_dec, State>(state));   
        }

        // Отрезок горизонтальный
        // Выбор: налево, направо
        template <template <class, class, class> class Policy, class State>
            bool select_y_axis(State & state)
        {
            point_2i        & A = state.A();
            point_2i const  & B = state.B();

            Assert(A.y == B.y);
            Assert(A.x != B.x);

            return 
                A.x < B.x
                ?   rasterize_segment(Policy<direction::none, direction::x_inc, State>(state))   
                :   rasterize_segment(Policy<direction::none, direction::x_dec, State>(state));   
        }
    }
}