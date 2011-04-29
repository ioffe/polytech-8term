#pragma once

#include "geometry\primitives\point.h"

#undef small
#undef big

namespace cg
{
    struct Index2L
    {
        __forceinline Index2L(point_2i const &big, point_2i const &small)
            :   big(big), small(small)
        {}

        __forceinline Index2L()
        {}

        __forceinline bool operator < (Index2L const &other) const
        {
            return 
                this->big  < other.big ||
                this->big == other.big && this->small < other.small;
        }

        __forceinline bool operator != (Index2L const & other) const
        {
            return
                this->big   != other.big ||
                this->small != other.small;
        }

        __forceinline bool operator == (Index2L const & other) const
        {
            return !(*this != other);
        }

        point_2i    big;
        point_2i    small;
    };
}
