#pragma once

#include "grid2l_queue_visit.h"
#include "Geometry\naa_rectangle_2.h"

namespace cg
{
    namespace grid2l_by_directed_rect
    {

        template<class GridTraits, class index_type>
            inline rectangle_2 get_cell_bound(index_type const& idx, GridTraits const& grid_traits_)
        {
            if(idx.small.x == -1)   return cellbound(grid_traits_.tform(), idx.big);
            else                    return smallcell_bound(grid_traits_, idx);
        }

        template <class GridTraits>
            struct dist_func
        {
            typedef GridTraits                          grid_traits_type;
            typedef typename GridTraits::index_type     index_type;

            dist_func(grid_traits_type const& grid_traits_) : grid_traits_(grid_traits_) {}

            void init(point_2 const& org, point_2 const& dir) { org_ = org, dir_ = dir; }

            double operator()(index_type const& idx) const
            {
                rectangle_2 rc = get_cell_bound(idx, grid_traits_);

                double d[4];
                d[0] = dist(rc.xy());
                d[1] = dist(rc.Xy());
                d[2] = dist(rc.xY());
                d[3] = dist(rc.XY());

                if(d[0] > 0 && d[1] > 0 && d[2] > 0 && d[3] > 0)
                    return min(min(d[0], d[1]), min(d[2], d[3]));

                return 0;
            }
        private:
            double dist(point_2 const& pt) const
            {
                return dir_ * (pt - org_);
            }

        private:
            grid_traits_type const& grid_traits_;
            point_2 org_;
            point_2 dir_;
        };

        template<class GridTraits>
            struct rectangle_bound_traits
        {
            typedef GridTraits                                 grid_traits_type;
            typedef typename GridTraits::index_type            index_type;

            rectangle_bound_traits(grid_traits_type const& grid_traits_) : grid_traits_(grid_traits_) {}

            void init(naa_rectangle_2 const& rc) { rect_ = rc; }

            bool operator()(index_type const& idx) const
            {
                naa_rectangle_2 rc_cell = get_cell_bound(idx, grid_traits_);
                
                return (has_intersection(rc_cell, rect_));
            }
        private:
            grid_traits_type const& grid_traits_;
            naa_rectangle_2 rect_;            
        };


        template<class Grid> struct grid_type_ {
            typedef Grid type;
        };

        template<class Grid> struct index_type_ {
            typedef typename grid_type_<Grid>::type::index_type  type;
        };

        template<class Grid> struct grid_traits_type_ {
            typedef grid2l_queue_visit::grid_traits <
                typename grid_type_<Grid>::type
            > type;
        };

        template<class Grid> struct dist_func_type_ {
            typedef dist_func <
                typename grid_traits_type_<Grid>::type
            > type;
        };

        template<class Grid> struct visitflag_traits_type_ {
            typedef grid2l_queue_visit::visitflag_traits <
                typename grid_traits_type_<Grid>::type
            > type;
        };

        template<class Grid> struct bound_traits_type_ {
            typedef rectangle_bound_traits <
                typename grid_traits_type_<Grid>::type
            > type;
        };

        template<class Grid> struct queue_type_  {
            typedef grid2l_queue_visit::std_priority_queue <
                typename index_type_<Grid>::type
            > type;
        };

        template<class Grid> struct queue_traits_type_  {
            typedef grid2l_queue_visit::queue_traits <
                typename grid_traits_type_     <Grid>::type, 
                typename queue_type_           <Grid>::type, 
                typename visitflag_traits_type_<Grid>::type, 
                typename dist_func_type_       <Grid>::type, 
                typename bound_traits_type_    <Grid>::type
            > type;
        };


        template<class Grid>
            struct traits 
        {
            typedef typename grid_type_<Grid>::type             grid_type;
            typedef typename index_type_<Grid>::type            index_type;
            typedef typename grid_traits_type_<Grid>::type      grid_traits_type;
            typedef typename dist_func_type_<Grid>::type        dist_func_type;
            typedef typename visitflag_traits_type_<Grid>::type visitflag_traits_type;
            typedef typename bound_traits_type_<Grid>::type     bound_traits_type;
            typedef typename queue_type_<Grid>::type            queue_type;
            typedef typename queue_traits_type_<Grid>::type     queue_traits_type;

            traits(grid_type& grid) 
                : grid_(grid)
                , grid_traits_(grid)
                , dist_func_(grid_traits_)
                , visitflag_traits_(grid_traits_)
                , queue_traits_(grid_traits_, visitflag_traits_, dist_func_, bound_traits_)
                , bound_traits_(grid_traits_)
            {
            }

            void init(naa_rectangle_2 const& rc, point_2 const& org, point_2 const& dir, double dist)
            {
                queue_traits_.init();
                visitflag_traits_.init();
                dist_func_.init(org, dir);
                dist_ = dist;
                bound_traits_.init(rc);
            }

            bool can_stop() const
            {
                return (queue_traits_.top().dist() > dist_);
            }

            double dist(index_type const& idx) const 
            {
                return dist_func_(idx);
            }


            grid_traits_type const&  gtraits() const { return grid_traits_; }
            grid_traits_type&        gtraits()       { return grid_traits_; }
            queue_traits_type const& qtraits() const { return queue_traits_; }
            queue_traits_type&       qtraits()       { return queue_traits_; }

        private:
            grid_type              & grid_;
            dist_func_type           dist_func_;
            grid_traits_type         grid_traits_;
            visitflag_traits_type    visitflag_traits_;
            bound_traits_type        bound_traits_;
            queue_traits_type        queue_traits_;

            double dist_;
        };

        template <class Traits, class Processor>
            inline bool visit(Traits & traits, naa_rectangle_2 const& rc, 
                  point_2 const& org, point_2 const& dir, double dist, Processor &processor)
        {
            traits.init(rc, org, dir, dist);
            return grid2l_queue_visit::queue_visit<Traits>::process(traits, org, processor);
        }

    }


    template <class Traits, class Processor>
        inline bool visit_grid2l_by_directed_rect(Traits & traits, naa_rectangle_2 const& rc, 
            point_2 const& org, point_2 const& dir, double dist, Processor &processor)
    {
        return grid2l_by_directed_rect::visit(traits, rc, org, dir, dist, processor);
    }
}
