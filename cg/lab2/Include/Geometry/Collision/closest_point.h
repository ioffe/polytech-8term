#pragma once

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct closest_point
        {
            closest_point() : circle_traits_(grid()) {}

            typedef typename Traits::face_id face_id;

            struct ClosestPointResult
            {
                double dist; 
                face_id face;
                cg::barycentric_coords bc;
            };

            bool closestPoint(double max_dist, point_3 const &org, ClosestPointResult &result)
            {
                Processor proc(self().traits(), max_dist, org, result);

                visit_grid2l_by_circle(circle_traits_, org, max_dist, proc);

                return proc.result();
            }


        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }

            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            {
                Processor(Traits const &traits, double max_dist, point_3 const &org, ClosestPointResult &result)
                    :   traits_  (traits)
                    ,   max_dist_(max_dist)
                    ,   org_     (org)
                    ,   result_  (result)
                    ,   bres_    (false)
                {
                    result_.dist = 1e100;
                    checked_flag_ = (long)::rand() + (((long)::rand()) << 16);
                }

                template <class State>
                    bool operator () (State &state, typename Grid::smallcell_type const & column)
                {
                    if(result_.dist < 1e-5 || state.dist () > max_dist_) 
                        return true ; 

                    if ( !column )
                       return false;

                    for(DWORD i = 0; i < column->faces().size(); ++i) 
                    {
                        if(traits_.checkedFlag(i) != checked_flag_)
                        {
                            traits_.checkedFlag(i) = checked_flag_;

                            int tr_idx = column->faces()[i];
                            triangle_3 tr = traits_.getTriangle(tr_idx);

                            double dist;
                            point_3 closest;
                            if(traits_.closest(result_.dist, tr_idx, org_, dist, closest))
                            {
                                result_.dist = dist;
                                bres_ = true;

                                // считаем бароцентрические координаты
                                cg::triangle_raster_aux tr_aux(tr);
                                Verify(tr_aux.IsInTriangleBarycentric(closest, result_.bc));
                            }
                        }
                    }

                    return false ;
                }

                bool result() const { return bres_; }

            private:
                Traits const & traits_;
                double max_dist_;
                point_3 const& org_;
                ClosestPointResult & result_;

                bool bres_;

                long checked_flag_;
            };

        private:
            grid2l_by_circle::traits<Grid> circle_traits_;
        };
    }
}
