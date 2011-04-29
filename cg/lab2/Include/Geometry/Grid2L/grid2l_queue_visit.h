#pragma once

#include <queue>

#include "common/safe_bool.h"
#include "Grid2l_Impl.h"

#include <boost/utility/enable_if.hpp>    


namespace cg
{
    namespace grid2l_queue_visit
    {

        template<class Grid> struct grid_type_ {
            typedef Grid type;
        };
        template<class Grid> struct actual_smallcell_type_ {
            typedef typename grid_type_<Grid>::type::smallcell_type  type;
        };
        template<class Grid> struct actual_bigcell_type_ {
            typedef typename grid_type_<Grid>::type::bigcell_type  type;
        };
        template<class Grid> struct index_type_ {
            typedef typename grid_type_<Grid>::type::index_type  type;
        };
        template<class Grid> struct bigidx_type_ {
            typedef typename grid_type_<Grid>::type::bigidx_type  type;
        };
        template<class Grid> struct smallidx_type_ {
            typedef typename grid_type_<Grid>::type::smallidx_type  type;
        };
        template<class Grid> struct header_type_ {
            typedef typename actual_bigcell_type_<Grid>::type::header_type  type;
        };
        template<class Grid> struct tform_type_ {
            typedef typename grid_type_<Grid>::type::tform_type  type;
        };

        // полезные специализации
        struct error_type {};

        template<class T>
                struct tform_type_<MappedGrid2L<T> > {
                    typedef error_type  type;
            };


        // будут необходимы прокси объекты в нетривиальной реализации (типа террейна) :(
        template<class Grid>
            struct smallcell_proxy
        {
            typedef typename grid_type_<Grid>            ::type grid_type;
            typedef typename actual_smallcell_type_<Grid>::type actual_smallcell_type;

            explicit smallcell_proxy(actual_smallcell_type const& smallcell) : smallcell_(smallcell) {}

        private:
            actual_smallcell_type const& smallcell_;
        };

        template<class Grid> struct smallcell_proxy_type_ {
            typedef smallcell_proxy<Grid>  type;
        };

        template<class Grid>
            struct bigcell_proxy
        {
            typedef typename grid_type_<Grid>            ::type grid_type;
            typedef typename actual_bigcell_type_<Grid>  ::type actual_bigcell_type;
            typedef typename smallidx_type_<Grid>        ::type smallidx_type;
            typedef typename smallcell_proxy_type_<grid_type> ::type smallcell_proxy_type;

            explicit bigcell_proxy(actual_bigcell_type const& bigcell) : bigcell_(bigcell) {}

        public:
            smallcell_proxy_type at(smallidx_type const& idx_small) const {
                return smallcell_proxy(bigcell_.at(idx_small)); 
            }
            smallidx_type extents() const { return bigcell_.extents(); }
            SAFE_BOOL_OPERATOR(bigcell_)
            bool contains(smallidx_type const& idx_small) { return bigcell_.contains(idx_small); }

        private:
            actual_bigcell_type const& bigcell_;
        };

        template<class Grid> struct bigcell_proxy_type_ {
            typedef bigcell_proxy<Grid>  type;
        };


        template<class Grid>
            struct grid_traits
        {
            typedef typename grid_type_<Grid>                 ::type grid_type;
            typedef typename actual_smallcell_type_<grid_type>::type actual_smallcell_type;
            typedef typename actual_bigcell_type_<grid_type>  ::type actual_bigcell_type;
            typedef typename index_type_<grid_type>           ::type index_type;
            typedef typename bigidx_type_<grid_type>          ::type bigidx_type;
            typedef typename smallidx_type_<grid_type>        ::type smallidx_type;
            typedef typename header_type_<grid_type>          ::type header_type;
            typedef typename tform_type_<grid_type>           ::type tform_type;
            typedef typename bigcell_proxy_type_<grid_type>   ::type bigcell_proxy_type;
            typedef typename smallcell_proxy_type_<grid_type> ::type smallcell_proxy_type;
            typedef bigcell_proxy_type                               bigcell_type;
            typedef smallcell_proxy_type                             smallcell_type;
            

            struct state : index_type
            {
                state(grid_type const& g, point_2i const& idx_big, point_2i const& idx_small, double dist)
                    :   index_type(idx_big, idx_small), grid_(g), dist_(dist)
                {}
                //rectangle_2 smallcellbound() const { return grid_.bigcellraster(big).domain(small); }
                grid_type       & grid()       { return grid_; }
                grid_type const & grid() const { return grid_; }
                double            dist() const { return dist_; }

            private:
                grid_type const & grid_;
                double dist_ ;
            };


            explicit grid_traits(grid_type& grid) : grid_(grid) {}

            bigidx_type           extents          () const {
                return grid_.extents();
            }
            bool                  contains         (bigidx_type const& idx_big) const {
                return grid_.contains(idx_big); 
            }
            header_type const&    bigcell_header   (bigidx_type const& idx_big) const {
                return bigcell(idx_big).header(); 
            }
            bool                  contains_smallcell(bigidx_type const& idx_big, smallidx_type const& idx_small) const {
                return contains_smallcell(index_type(idx_big, idx_small)); 
            }
            bool                  contains_smallcell(index_type const& idx) const {
                return contains(idx.big) && at(idx.big).contains(idx.small); 
            }
            point_2               world2local      (point_2 const& pt) const {
                return grid_.world2local(pt);
            }
            point_2               local2world      (point_2 const& pt) const {
                return grid_.local2world(pt);
            }
            smallidx_type         extents          (bigidx_type const& idx_big) const {
                return bigcell(idx_big).extents();
            }
            bool                  is_empty         (bigidx_type const& idx_big) const {
                return !bigcell(idx_big);
            }
            tform_type const& tform() const {
                return grid_.tform();
            }
            point_2 const& unit() const {
                return grid_.unit();
            }

            template<class Processor>
                bool                  process          (index_type const& idx, double dist, Processor& proc)
            {
                return proc(state(grid_, idx.big, idx.small, dist), smallcell(idx));
            }

            actual_bigcell_type const&   bigcell          (bigidx_type const& idx_big) const {
                return grid_.at(idx_big); 
            }
            actual_bigcell_type&   bigcell                (bigidx_type const& idx_big) {
                return grid_.at(idx_big); 
            }
            actual_smallcell_type const& smallcell        (index_type const& idx) const {
                return bigcell(idx.big).at(idx.small); 
            }
            actual_smallcell_type& smallcell              (index_type const& idx) {
                return bigcell(idx.big).at(idx.small); 
            }


            bigcell_proxy_type at(bigidx_type const& idx_big) const {
                return bigcell_proxy_type(bigcell(idx_big));
            }


        private:
            const grid_type& grid() const { return grid_; }

        private:
            grid_type& grid_;
        };

        template<class GridTraits>
            struct visitflag_traits
        {
            typedef GridTraits                            grid_traits_type;
            typedef typename grid_traits_type::index_type index_type;
            typedef long                                  visitflag_type;

        public:
            visitflag_traits(grid_traits_type const& g) : grid_traits_(g) { init(); }
            void init() 
            {
                visit_flags_ex_.clear();
                cur_visit_flag_ = (((long)::rand()) << 16) + (long)::rand();
            }

            bool is_visited(index_type const &idx) const { return (visitflag(idx) == cur_visit_flag_); }
            void visit     (index_type const &idx)       { visitflag(idx) = cur_visit_flag_; }

        private:
            visitflag_type&       visitflag(index_type const &idx) const
            {
                if(grid_traits_.contains(idx.big)) 
                {
                    if(idx.small.x == -1)   return grid_traits_.bigcell_header(idx.big).visit();
                    else                    return grid_traits_.smallcell(idx).visit();
                }
                else return visit_flags_ex_[idx];
            } 


        private:
            grid_traits_type const& grid_traits_;
            mutable std::map<index_type, visitflag_type> visit_flags_ex_;
            visitflag_type cur_visit_flag_;
        };

        template<class IndexType>
            struct distanced_idx : public IndexType
        {
            typedef IndexType index_type;
            distanced_idx(index_type idx, double dist) : index_type(idx), dist_(dist) {}
            double dist() const { return dist_; }
        private:
            double dist_;
        };

        template<class DistancedIdx>
            struct queue_less 
        {
            inline bool operator()(DistancedIdx const& _Left, DistancedIdx const& _Right) const
            {	
                return (_Left.dist() > _Right.dist());
            }
        };

        template<class IndexType>
            struct std_priority_queue 
                : std::priority_queue<distanced_idx<IndexType>, std::vector<distanced_idx<IndexType> >, queue_less<distanced_idx<IndexType> > >
        {
            typedef distanced_idx<IndexType> queue_elem_type;
            void clear() { c.clear(); }
        };


        template <class GridTraits, class Queue, class VisitFlagTraits, class DistFunc, class BoundTraits>
            struct queue_traits
        {
            typedef GridTraits                                 grid_traits_type;
            typedef DistFunc                                   dist_func_type;
            typedef VisitFlagTraits                            visitflag_traits_type;
            typedef typename grid_traits_type::index_type      index_type;
            typedef BoundTraits                                bound_traits_type;

            typedef Queue                           queue_type;
            typedef typename Queue::queue_elem_type queue_elem_type;


        public:
            queue_traits(grid_traits_type const& g, visitflag_traits_type& visitflag_traits__, 
                         dist_func_type const& dist_func, bound_traits_type const& bound_traits) 
                : grid_traits_(g)
                , dist_func_(dist_func)  
                , visitflag_traits_(visitflag_traits__)
                , bound_traits_(bound_traits)
            { init(); }

            void init() {
                queue_.clear();
            }

            void push(index_type const &idx)
            {
                if(!visitflag_traits_.is_visited(idx) && bound_traits_(idx))
                {
                    visitflag_traits_.visit(idx);
                    queue_.push(queue_elem_type(idx, dist_func_(idx)));
                }
            }
            void push(point_2 const& world_pt) {
                push(index_by_point(world_pt));
            }
            bool empty() const {
                return queue_.empty(); 
            }
            queue_elem_type pop() 
            {
                queue_elem_type idx = queue_.top();
                queue_.pop();
                return idx;
            }
            queue_elem_type top() const { return queue_.top(); }

        private:
            index_type index_by_point(point_2 const& world_pt) const
            {
                point_2  pt_in_grid = grid_traits_.world2local(world_pt);
                point_2i idx_big = floor(pt_in_grid);

                if(grid_traits_.contains(idx_big))
                {
                    if(!grid_traits_.is_empty(idx_big))
                    {
                        point_2i idx_small = floor((pt_in_grid - idx_big) & grid_traits_.extents(idx_big));
                        return index_type(idx_big, idx_small);
                    }
                    else return index_type(idx_big, point_2i(-1, -1));
                }
                else return index_type(idx_big, point_2i(-1, -1));
            }

        private:
            grid_traits_type const&  grid_traits_;
            dist_func_type const&    dist_func_;
            visitflag_traits_type&   visitflag_traits_;
            bound_traits_type const& bound_traits_;

            queue_type queue_;
        };


        template <class Traits>
            struct queue_visit
        {
            typedef Traits                                      traits_type;
            typedef typename traits_type::grid_traits_type      grid_traits_type;
            //typedef typename grid_traits_type::smallcell_type   smallcell_type;
            typedef typename grid_traits_type::bigcell_type     bigcell_type;
            typedef typename grid_traits_type::index_type       index_type;
            typedef typename grid_traits_type::bigidx_type      bigidx_type;
            typedef typename grid_traits_type::smallidx_type    smallidx_type;
            typedef typename traits_type::queue_traits_type     queue_traits_type;


            template <class Processor>
                static bool process(traits_type& traits, point_2 const& o, Processor & processor)
            {
                typename Processor::SmallCellProcessor &smallcell_proc = processor ;

                // add first cell
                traits.qtraits().push(o);

                while(!traits.qtraits().empty() && !traits.can_stop())
                {
                    index_type idx = traits.qtraits().pop();

                    if(idx.small.x != -1)
                    {
                        if(traits.gtraits().process(idx, traits.dist(idx), smallcell_proc))
                            return true;
                    }

                    // add new cells that border upon this cell
                    processborder(traits, idx.big, idx.small, point_2i(1, 0));
                    processborder(traits, idx.big, idx.small, point_2i(-1, 0));
                    processborder(traits, idx.big, idx.small, point_2i(0, 1));
                    processborder(traits, idx.big, idx.small, point_2i(0, -1));

                    processborder(traits, idx.big, idx.small, point_2i(1, 1));
                    processborder(traits, idx.big, idx.small, point_2i(-1, 1));
                    processborder(traits, idx.big, idx.small, point_2i(1, -1));
                    processborder(traits, idx.big, idx.small, point_2i(-1, -1));
                }

                return false ;
            }

            static void processborder(traits_type& traits,
                bigidx_type &idx_big, smallidx_type &idx_small, point_2i &dir)
            {
                smallidx_type idx_next_smallcell = idx_small + dir;

                if (idx_small.x != -1 && traits.gtraits().contains_smallcell(idx_big, idx_next_smallcell))
                {
                    traits.qtraits().push(index_type(idx_big, idx_next_smallcell));
                }
                else
                {
                    bigidx_type idx_next_bigcell = idx_big + dir; 

                    bool out_of_big_bound = !traits.gtraits().contains(idx_big);
                    bigidx_type bigcell_extents(0, 0);

                    if(!out_of_big_bound)
                        bigcell_extents = traits.gtraits().at(idx_big).extents();


                    if (out_of_big_bound || dir.x == 0 || dir.y == 0 || 
                       (dir.x == 1 && dir.y == 1 && idx_small.x == bigcell_extents.x - 1 && idx_small.y == bigcell_extents.y - 1) ||
                       (dir.x == -1 && dir.y == 1 && idx_small.x == 0 && idx_small.y == bigcell_extents.y - 1) ||
                       (dir.x == 1 && dir.y == -1 && idx_small.x == bigcell_extents.x - 1 && idx_small.y == 0) ||
                       (dir.x == -1 && dir.y == -1 && idx_small.x == 0 && idx_small.y == 0))
                    {
                        if (traits.gtraits().contains(idx_next_bigcell))
                        {
                            bigcell_type next_bigcell = traits.gtraits().at(idx_next_bigcell) ;
                            if (next_bigcell)
                            {
                                // compute iteration cells
                                point_2i dir2(0, 0);

                                if(dir.x && !dir.y) dir2.y = 1;
                                else if (dir.y && !dir.x) dir2.x = 1;

                                smallidx_type idx_begin_smallcell;
                                smallidx_type idx_end_smallcell;

                                if(!out_of_big_bound &&
                                    idx_small.x != -1 && (dir2.x != 0 || dir2.y != 0))
                                {
                                    if (dir2.x)
                                    {
                                        idx_begin_smallcell.y = idx_end_smallcell.y = (dir.y == 1) ? 0 : (next_bigcell.extents().y - 1);
                                        idx_begin_smallcell.x = floor((idx_small.x - 0.01) * next_bigcell.extents().x / bigcell_extents.x);
                                        idx_end_smallcell.x = floor((idx_small.x + 1.1) * next_bigcell.extents().x / bigcell_extents.x);

                                        idx_begin_smallcell.x = min(max(0, idx_begin_smallcell.x), next_bigcell.extents().x);
                                        idx_end_smallcell.x = min(max(0, idx_end_smallcell.x), next_bigcell.extents().x);
                                    }
                                    else
                                    {
                                        idx_begin_smallcell.x = idx_end_smallcell.x = (dir.x == 1) ? 0 : (next_bigcell.extents().x - 1);
                                        idx_begin_smallcell.y = floor((idx_small.y - 0.01) * next_bigcell.extents().y / bigcell_extents.y);
                                        idx_end_smallcell.y = floor((idx_small.y + 1.01) * next_bigcell.extents().y / bigcell_extents.y);

                                        idx_begin_smallcell.y = min(max(0, idx_begin_smallcell.y), next_bigcell.extents().y);
                                        idx_end_smallcell.y = min(max(0, idx_end_smallcell.y), next_bigcell.extents().y);
                                    }
                                }
                                else
                                {
                                    idx_begin_smallcell = point_2i(0, 0);
                                    if      (dir.x == 1 ) idx_begin_smallcell.x = 0;
                                    else if (dir.x == -1) idx_begin_smallcell.x = next_bigcell.extents().x - 1;
                                    if      (dir.y == 1 ) idx_begin_smallcell.y = 0;
                                    else if (dir.y == -1) idx_begin_smallcell.y = next_bigcell.extents().y - 1;
                                    idx_end_smallcell = idx_begin_smallcell; 

                                    if (dir2.x != 0 || dir2.y != 0)
                                        idx_end_smallcell += dir2 & next_bigcell.extents();
                                    else
                                        dir2.x = dir2.y = 1;
                                }

                                // iterate small cells
                                smallidx_type idx_next_smallcell;
                                for (idx_next_smallcell = idx_begin_smallcell; idx_next_smallcell != idx_end_smallcell + dir2; idx_next_smallcell += dir2)
                                {
                                    if(next_bigcell.contains(idx_next_smallcell))
                                        traits.qtraits().push(index_type(idx_next_bigcell, idx_next_smallcell));
                                }
                            }
                            else
                            {
                                traits.qtraits().push(index_type(idx_next_bigcell, smallidx_type(-1, -1)));
                            }
                        }
                        else
                        {
                            traits.qtraits().push(index_type(idx_next_bigcell, smallidx_type(-1, -1)));
                        }
                    }
                }
            }

        };

    }
}
