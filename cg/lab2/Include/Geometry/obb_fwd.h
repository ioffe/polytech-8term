#pragma once

namespace cg
{
    template <class Point>
        struct OBB_extents_t;

    typedef OBB_extents_t<point_3>  OBB_extents;
    typedef OBB_extents_t<point_3f> OBB_extents_f;

    template <class Point>
        struct OBB_t;

    typedef OBB_t<point_3 > OBB;
    typedef OBB_t<point_3f> OBB_f;
}