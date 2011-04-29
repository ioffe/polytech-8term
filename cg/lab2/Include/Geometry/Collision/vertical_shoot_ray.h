#pragma   once

#include "triangle_filter.h"

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct vertical_shoot_ray
        {
            typedef typename Traits::face_id face_id;

            struct VertIntersectionResult
            {
                double height; face_id face;
                cg::barycentric_coords bc;
            };

            bool shootVertical(point_2 const &pt, double hFrom, double hTo, VertIntersectionResult &result,
                               triangle_filter_t const & f = dummy_triangle_filter) const
            {
                Processor vshootray (self().traits(), pt, hFrom, hTo, f, result);

                bool ret = visit (grid(), pt, vshootray);

                return ret;
            }

            static double inf () { return std::numeric_limits<double>::max(); }

        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }

            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            {
                Processor(
                    Traits  const &traits, 
                    point_2 const &pt, 
                    double         hFrom, 
                    double         hTo,
                    triangle_filter_t const & f, 
                    VertIntersectionResult & result)
                    :   traits_(traits)
                    ,   pt_    (pt)
                    ,   hFrom_ (hFrom)
                    ,   hTo_   (hTo)
                    ,   result_(result)
                    ,   filter_( f )
                {
                    result.height = hFrom < hTo  ?  inf() : -inf();
                }

                template <class State>
                    bool operator () (State &, typename Grid::smallcell_type const & column)
                {
                    if (column)
                    {
                        for (unsigned i = 0; i < column->faces().size(); ++i)
                        {
                            size_t const trg = column->faces()[i];

                            double h;
                            cg::barycentric_coords bc;

                            if (filter_( trg ) && traits_.v_intersect(trg, pt_, h, bc))
                            {
                                if (range_2(hFrom_, hTo_).contains(h))
                                {
                                    // ≈сли нашлось пересечение, то в зависимости
                                    // от направлени€ смотрим, улучшает ли оно существующее
                                    if ((hFrom_ < hTo_) ^ (h > result_.height)) 
                                    {
                                        result_.height = h;
                                        result_.face   = trg;
                                        result_.bc     = bc;
                                    } 
                                }
                            }
                        }

                        return cg::abs(result_.height) < inf() / 2;                    
                    }

                    return false;
                }

            private:
                Traits   const & traits_;
                point_2  const & pt_;
                double   const   hFrom_;
                double   const   hTo_;
                VertIntersectionResult  & result_;
                triangle_filter_t const & filter_;
            };
        };
    }
}