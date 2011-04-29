#pragma once

#include "geometry\segment_2_intersection.h"
#include "geometry\point_ops.h" 

struct FindFirstIntersectionObject
{
    typedef cg::segment_2 segment_2;

    __forceinline FindFirstIntersectionObject (segment_2 const & s) 
        :   query_segment_(s) 
        ,   min_t_ (infinity())
        ,   is_min_inside_ (false)
    {}

    // возвращает true, если other стал отрезком, на котором достигается лучшее пересечение
    __forceinline bool operator () (segment_2 const & other) 
    {
        cg::point_2 ipt1, ipt2;
        cg::intersection_type ipt_type = cg::generic_intersection( query_segment_, other, &ipt1, &ipt2 );

//        Assert( ipt_type != cg::overlap );

        if (ipt_type == cg::intersect)
        {
            // находим, в каком отношении точка пересечения делит исходный луч. Выбираем минимум
            double t = query_segment_ (ipt1);
            //Assert( !cg::eq( t, min_t_ ) );
            if (cg::eq (t, min_t_ )) {

                bool is_current_inside = get_inside( other );
    
                if (!is_min_inside_ && is_current_inside) {
                    is_min_inside_ = true;
#ifdef DEBUG
                    last_min_segment_ = other;
#endif 
                    return true;
                }

                return false;
            }

            if (t < min_t_)
            {
                is_min_inside_ = get_inside( other );
                min_t_ = t;
#ifdef DEBUG
                last_min_segment_ = other;
#endif 
                return true;
            }
        }

        return false;
    }

    __forceinline bool was_intersection () const
    {
        return min_t_ != infinity();
    }

    __forceinline bool is_intersection_inside () const
    {
        return is_min_inside_;
    }

private:
    __forceinline bool get_inside ( segment_2 const & seg ) const {
        return cg::right_turn(query_segment_.P0(), seg.P0(), seg.P1());
    }

    segment_2   query_segment_;

#ifdef DEBUG
    segment_2   last_min_segment_;
#endif

    bool        is_min_inside_;

    double      min_t_;

    __forceinline static double infinity () { return std::numeric_limits<double>::max();  }
};
