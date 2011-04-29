#pragma once 

#include "common/Assert.h"

namespace cg
{
    // ядерная арифметика решает!!!
    // если clockwise_ordered подать 3д последовательность, то все плохо :-)))
    // TODO

    // Ф-я возвращает true, если обход контура осуществляется по часовой стрелке (глядя на контур вдоль оси Y, т.е. снизу).
    // На вход подаются НЕзамкнутые полилинии (т.е. последняя и первая вершина из points образуют невырожденное ребро)
    template <class Point>
        inline bool clockwise_ordered( Point const *points, int count )
    {
        // находим самую верхнюю точку
        double maxy = -FLOAT_ETERNITY;
        int idx  = -1;

        for (int i=0; i<count; ++i)
        {
            if (maxy < points[i].y)
            {
                maxy = points[i].y;
                idx = i;
            }
        }

        // определяем, куда смотрят предыдущий и следующий от нее сегменты
        int i_prev = prev(idx, count);
        int i_next = next(idx, count);

        return normalized( point_2( points[i_prev] - points[idx] ) ).x < 
            normalized( point_2( points[i_next] - points[idx] ) ).x;
    }

    template <class FwdIter>
        FwdIter findTopmost (FwdIter first, FwdIter beyond)
    {
        Assert(first != beyond); 

        double maxy =  first->y;
        FwdIter best = first;

        for (++first; first != beyond; ++first)
        {
            if (maxy < first->y)
            {
                maxy = first->y;
                best = first;
            }
        }

        return best;
    }

    // TODO: Все потом переписать через traits
    // От traits будет требоваться bool compare_y(point, point) const
    template <class FwdIter, class Traits>
        FwdIter findTopmost (FwdIter first, FwdIter beyond, Traits const & traits)
    {
        Assert(first != beyond); 

        double maxy =  traits.at(*first).y;
        FwdIter best = first;

        for (++first; first != beyond; ++first)
        {
            if (maxy < traits.at(*first).y)
            {
                maxy = traits.at(*first).y;
                best = first;
            }
        }

        return best;
    }

    // требуем, чтобы первая и последняя точки не совпадали
    template <class FwdIter>
        inline bool clockwise_ordered( FwdIter first, FwdIter beyond )
    {
        if (first != beyond)
        {
           if ( *first == *( beyond - 1 ) )
              --beyond;

           if ( first == beyond )
              return false;

           // находим самую верхнюю точку
           FwdIter topmost = findTopmost(first, beyond);

           // определяем, куда смотрят предыдущий и следующий от нее сегменты
           // TODO: should be used only ++, --
           FwdIter i_prev = topmost == first ? beyond - 1 : topmost - 1;
           FwdIter i_next = topmost == beyond - 1 ? first : topmost + 1;

           return normalized( point_2( *i_prev - *topmost ) ).x < 
                  normalized( point_2( *i_next - *topmost ) ).x;
        }
        else
        {
           // Can't determine orientation for contour without points.
           Assert(0);
           return false;
        }
    }

    // TODO: От traits будет требоваться функция типа right_turn_strict  и compare_y
    template <class FwdIter, class Traits>
        inline bool clockwise_ordered( FwdIter first, FwdIter beyond, Traits const & traits )
    {
        Assert( first != beyond );

        if ( traits.at( *first ) == traits.at( *( beyond - 1 ) ) )
           --beyond;

        if ( first == beyond )
           return false;

        // находим самую верхнюю точку
        FwdIter topmost = findTopmost(first, beyond, traits);

        // определяем, куда смотрят предыдущий и следующий от нее сегменты
        // TODO: should be used only ++, --
        FwdIter i_prev = topmost == first ? beyond - 1 : topmost - 1;
        FwdIter i_next = topmost == beyond - 1 ? first : topmost + 1;

        return normalized( point_2( traits.at(*i_prev) - traits.at(*topmost) ) ).x < 
               normalized( point_2( traits.at(*i_next) - traits.at(*topmost) ) ).x;
    }
}
