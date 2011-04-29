#pragma once

#include <queue>

#include "Grid1L_Impl.h"
#include "Geometry\circle_rasterizator.h"

namespace cg 
{
    namespace grid1L_by_circle
    {
        template<class Grid>
            struct std_traits
        {
            typedef point_2i index_type;
            typedef Grid     grid_type;
            typedef typename grid_type::cell_type cell_type;
            typedef int      linear_index;

            struct idx_dist : public index_type
            {
                idx_dist(index_type idx, double dist) : index_type(idx), dist_(dist) {}
                double dist() const { return dist_; }
            private:
                double dist_;
            };

            struct idx_less
            {
                // lower distance have more priority
                inline bool operator()(idx_dist const& _Left, idx_dist const& _Right) const
                {
                    return (_Left.dist() > _Right.dist());
                }
            };

            typedef idx_dist queue_elem_type;
            typedef std::priority_queue<queue_elem_type, std::vector<queue_elem_type>, idx_less> queue_type;
            typedef long visitflag_type;
            typedef std::vector<visitflag_type>           visitflag_vec_type;
            typedef std::map<index_type, visitflag_type>  visitflag_ex_vec_type;

        public:
            std_traits(grid_type const &g)
               : grid_(g)
               , width_(0)
            {
               init();
            }

            void init()
            {
                width_ = grid_.extents().x;
                visit_flags_.resize(grid_.size());
                cur_visit_flag = (((long)::rand()) << 16) + (long)::rand();
                visit_flags_ex_.clear();
            }

            // queue support
            void queue_push(index_type const &idx, point_2 const &org)
            {
                if(!is_visited(idx))
                {
                    visit(idx);
                    queue_.push(idx_dist(idx, dist(idx, org)));
                }
            }
            void queue_push(point_2 const &gpoint, point_2 const &org) 
            {
                index_type idx = floor(grid_.world2local(gpoint));
                queue_push(idx, org);
            }
            bool queue_empty() const 
            {
                return queue_.empty(); 
            }
            queue_elem_type queue_pop() 
            {
                idx_dist idx = queue_.top();
                queue_.pop();
                return idx;
            }

            bool is_valid(index_type const &idx) const
            {
                return grid_.contains(idx);
            }

            cell_type const & cell(index_type const &idx) 
            {
                return grid_[idx];
            }

        private:
            visitflag_type & visitflag(index_type const &idx) 
            {
                if(grid_.contains(idx))   return visit_flags_[to1D(idx)]; 
                else                      return visit_flags_ex_[idx];
            }
            visitflag_type const & visitflag(index_type const &idx) const 
            {
                if(grid_.contains(idx))   return visit_flags_[to1D(idx)]; 
                else                      return visit_flags_ex_[idx];
            }

            bool is_visited(index_type const &idx) const { return visitflag(idx) == cur_visit_flag; }
            void visit     (index_type const &idx)       { visitflag(idx) = cur_visit_flag; }

            double dist(index_type const &idx, point_2 const &pt) const
            {
                rectangle_2 rc = cellbound(grid_.tform(), idx);
                return (rc.contains(pt) ? 0 : distance(rc, pt));
            }

            linear_index to1D(index_type const& idx) const
            {
                return linear_index(idx.y * width_ + idx.x);
            }

        private:
            grid_type const & grid_;
            queue_type        queue_;
            
            mutable visitflag_vec_type      visit_flags_;
            mutable visitflag_ex_vec_type   visit_flags_ex_;
            mutable visitflag_type          cur_visit_flag;
            int width_;
        };

        template <class Traits>
            struct visit_grid1L_by_circle
        {
            struct State : public point_2i
            {
                State(point_2i const &idx, double dist) : point_2i(idx), dist_(dist) { }
                double dist() const { return dist_; }
            private:
                double dist_;
            };

            typedef point_2i   index_type;
            typedef State      state_type;
            
            typedef typename Traits::queue_elem_type queue_elem_type;

            template<class ActualProc>
                struct ProcessorProxy
            {
                ProcessorProxy(ActualProc &proc, Traits &traits) : traits_(traits), proc_(proc) {}

                bool operator() (point_2i const &idx, double dist ) const 
                {
                    if(traits_.is_valid(idx))
                        return proc_(state_type(idx, dist), traits_.cell(idx));

                    return false;
                }

            private:
                Traits     & traits_;
                ActualProc & proc_;
            };

            template <class Processor>
                static bool process(Traits &traits, point_2 const & o, double r, Processor & proc)
            {
                ProcessorProxy<Processor> proc_proxy(proc, traits);
                return rasterize_circle(o, r, proc_proxy, traits);
            }
        };
    }

    template <class Processor, class Traits>
        inline bool visit_by_circle(Traits &traits, point_2 const & org, double r, Processor & proc)
    {
        return grid1L_by_circle::visit_grid1L_by_circle<Traits>::process(traits, org, r, proc);
    }
}
