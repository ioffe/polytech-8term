#pragma once

#include "grid2l_queue_visit.h"

namespace cg
{
    namespace grid2l_by_circle
    {
       /* template <class G>
            struct traits_for_mapped_grid
        {
            typedef G                                grid_type;
            typedef typename G::smallcell_type       smallcell_type;
            typedef typename G::bigcell_type         bigcell_type;
            typedef typename G::index_type           index_type;

            typedef long visitflag_type;
            typedef cg::array_2d<cg::array_2d<visitflag_type> > visitflag_vec_type;

            traits_for_mapped_grid(grid_type &g) : grid_(g)         { init(); }

            void init()
            {
                // переписать с указательми!!! А то все тормозит.
                if(visit_flags_.extents() != grid_.array().extents())
                    visit_flags_.resize(grid_.array().extents());
                for(int x = 0; x < visit_flags_.extents().x; x++)
                    for(int y = 0; y < visit_flags_.extents().y; y++)
                        if(visit_flags_[point_2i(x, y)].extents() != grid_.array()[point_2i(x, y)].extents())
                            visit_flags_[point_2i(x, y)].resize(grid_.array()[point_2i(x, y)].extents());

                if(visit_flags_big_.extents() != grid_.array().extents())
                    visit_flags_big_.resize(grid_.array().extents());

                visit_flags_ex_.clear();

                cur_visit_flag = (((long)::rand()) << 16) + (long)::rand();
            }


            //Временная мера
            grid_type const& grid() const { return grid_; }
            grid_type      & grid()       { return grid_; }

            bool is_visited(index_type const &idx) const { return visitflag(idx) == cur_visit_flag; }
            void visit     (index_type const &idx)       { visitflag(idx) = cur_visit_flag; }


        private:
            visitflag_type & visitflag(index_type const &idx) 
            {
                if(idx.small.x == -1)
                    return visit_flags_big_[idx.big]; 
                else if(grid_.contains(idx.big) && visit_flags_[idx.big].contains(idx.small))
                    return visit_flags_[idx.big][idx.small]; 
                else
                    return visit_flags_ex_[idx];
            }
            visitflag_type const & visitflag(index_type const &idx) const 
            {
                if(idx.small.x == -1)
                    return visit_flags_big_[idx.big]; 
                else if(grid_.contains(idx.big) && visit_flags_[idx.big].contains(idx.small))
                    return visit_flags_[idx.big][idx.small]; 
                else
                    return visit_flags_ex_[idx];
            }


        private:
            grid_type & grid_;

            mutable visitflag_vec_type visit_flags_;
            mutable cg::array_2d<visitflag_type> visit_flags_big_;
            mutable visitflag_type     cur_visit_flag;
            mutable std::map<index_type, visitflag_type> visit_flags_ex_;
        };*/


        template<class Grid, class index_type>
            inline rectangle_2 get_cell_bound(index_type const& idx, Grid const& grid_)
        {
            if(idx.small.x == -1)   return cellbound(grid_.tform(), idx.big);
            else                    return smallcell_bound(grid_, idx);
        }

        template <class Grid>
            struct dist_func
        {
            typedef Grid                                grid_type;
            typedef typename Grid::index_type           index_type;

            dist_func(grid_type const& grid) : grid_(grid) { init(point_2(), 0); }

            void init(point_2 const &o, double r) { org_ = o, radius_ = r; }

            double operator()(index_type const& idx) const
            {
                rectangle_2 rc = get_cell_bound(idx, grid_);
                return dist_to_rect(org_, rc);
            }
        private:
            double dist_to_rect(point_2 const& o, rectangle_2 const& rc) const 
            {
                return rc.contains(o) ? 0 : distance(rc, o);
            }

        private:
            grid_type const& grid_;
            point_2 org_;
            double radius_;
        };

        template<class Grid>
            struct empty_bound_traits
        {
            typedef Grid                                       grid_type;
            typedef typename grid_type::index_type             index_type;
            
            bool operator()(index_type const& idx) const
            {
                return true;
            }
        };

        template<class Grid>
            struct traits 
        {
            typedef Grid                                            grid_type;
            typedef typename grid_type::index_type                  index_type;
            typedef dist_func<grid_type>                            dist_func_type;
            typedef grid2l_queue_visit::grid_traits<grid_type>      grid_traits_type;
            typedef grid2l_queue_visit::visitflag_traits<
                grid_traits_type>                                   visitflag_traits_type;
            typedef empty_bound_traits<grid_type>                   bound_traits_type;

            typedef grid2l_queue_visit::std_priority_queue<index_type> queue_type;

            typedef grid2l_queue_visit::queue_traits<
                grid_traits_type, queue_type, visitflag_traits_type, 
                dist_func_type, bound_traits_type>                  queue_traits_type;

            traits(grid_type& grid) 
                : grid_(grid)
                , grid_traits_(grid)
                , dist_func_(grid)
                , visitflag_traits_(grid_traits_)
                , queue_traits_(grid_traits_, visitflag_traits_, dist_func_, bound_traits_)
                , radius_(0)
            {
                init(point_2(), 0);
            }

            void init(point_2 const &o, double r)
            {
                queue_traits_.init();
                visitflag_traits_.init();
                dist_func_.init(o, r);
                radius_ = r;
            }

            bool can_stop() const
            {
                return (queue_traits_.top().dist() > radius_);
            }

            double dist(index_type const& idx) const 
            {
                return dist_func_(idx);
            }

            grid_traits_type const& gtraits() const { return grid_traits_; }
            grid_traits_type&       gtraits()       { return grid_traits_; }


            queue_traits_type const& qtraits() const { return queue_traits_; }
            queue_traits_type&       qtraits()       { return queue_traits_; }

        private:
            grid_type&             grid_;
            dist_func_type         dist_func_;
            grid_traits_type       grid_traits_;
            visitflag_traits_type  visitflag_traits_;
            queue_traits_type      queue_traits_;
            bound_traits_type      bound_traits_;

            double                 radius_;
        };


        template <class Traits, class Processor>
            bool visit(Traits & traits, point_2 const &o, double r, Processor &processor)
        {
            traits.init(o, r);
            return grid2l_queue_visit::queue_visit<Traits>::process(traits, o, processor);
        }

    }



    template <class Traits, class Processor>
        inline bool visit_grid2l_by_circle(Traits & traits, point_2 const &o, double r, Processor &processor)
    {
        return grid2l_by_circle::visit(traits, o, r, processor);
    }


    template <class T, class B, class H, class Processor>
        inline bool visit(Grid2L<T,B,H> & grid, point_2 const &o, double r, Processor &processor)
    {
        // Use visit_grid2l_by_circle to optimize creation traits.
        grid2l_by_circle::traits<Grid2L<T,B,H> > traits(grid);
        return grid2l_by_circle::visit(traits, o, r, processor);
    }
}