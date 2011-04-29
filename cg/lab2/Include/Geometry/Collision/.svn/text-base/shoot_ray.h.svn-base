#pragma   once

#include "geometry/grid1L.h"
#include "calc_barycentric_coords_3d.h"

#include "triangle_filter.h"

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct shoot_ray
        {
            typedef typename Traits::face_id face_id;

            struct IntersectionParams
            {
                double ratio; face_id face;
                cg::barycentric_coords bc;
            };

            bool shootRay (point_3 const &from, point_3 const &to, bool nobackface, IntersectionParams &result,
                           triangle_filter_t const & filter = dummy_triangle_filter) const
            {
                segment_2 seg2(from,to);
                rectangle_2 bb = cg::bounding(grid());
                bb.inflate( -1 );

                segment_2 clipped_seg2;
                if (!cull(seg2, bb, clipped_seg2))
                    return false;

                segment_3 seg3( from, to );
                Processor shootray(self().traits(), seg3, seg3( seg2( clipped_seg2[0] ) ), seg3( seg2( clipped_seg2[1] ) ), filter, nobackface, result);

                return visit(grid(), clipped_seg2, shootray);
            }

        private:

            Derived const & self () const { return static_cast<Derived const &>(*this); }

            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            {
               Processor(Traits const &traits, cg::segment_3 const & orig_seg, point_3 const &from, 
                    point_3 const &to, triangle_filter_t const & filter, bool nobackface, IntersectionParams &result)
                    :   traits_ (traits)
                    ,   org_    (from)
                    ,   dir_    (to - from)
                    ,   result_ (result)
                    ,   nobackface_ (nobackface)
                    ,   filter_( filter )
                    ,   orig_seg_( orig_seg )
                {}

                template <class State>
                    bool operator () (State &state, typename Grid::smallcell_type const& column)
                {
                    // если клетка не пуста€
                    if (column)
                    {
                        // определ€ем, какие высоты заметает луч
                        range_2 const hray(
                            org_.z + state.in_ratio  * dir_.z, 
                            org_.z + state.out_ratio * dir_.z);

                        // если диапазоны высот пересекаютс€
                        if (has_intersection(hray, column->zrange()))
                        {
                            // ищем лучшее пересечение с лучом
                            result_.ratio = FLOAT_ETERNITY;

                            double ratio;

                            for (size_t i = 0, size = column->faces().size(); i != size; ++i) 
                            {
                                // если грань пересекаетс€ с лучом
                                // и это пересечение лучше
                                int tr_idx = column->faces()[i];
                                triangle_3 tr = traits_.getTriangle( tr_idx );
                                if (filter_(tr_idx)
                                   && traits_.intersect(tr_idx, org_, dir_, ratio, nobackface_)
                                   && ratio < result_.ratio)  
                                {
                                    // находим точку пересечени€
                                    point_3 const ipt = org_ + dir_ * ratio;

                                    result_.ratio = orig_seg_( ipt );
                                    result_.face  = column->faces()[i];    
                                    
                                    // считаем бароцентрические координаты
                                    if( !calc_barycentric_coords_3d( tr, ipt, result_.bc ) )
                                    {
                                       // ≈сли не получилось - подправл€ем
                                       if( !cg::eq( result_.bc.alpha + result_.bc.beta, 1, 1e-4 ) )
                                       {
                                          Assert( !"¬озможно проблема в погрешности" );
                                       }

                                       // ј вдруг?
                                       if( result_.bc.alpha < 0 )
                                          result_.bc.alpha = 0;

                                       if( result_.bc.beta < 0 )
                                          result_.bc.beta = 0;

                                       if( result_.bc.alpha + result_.bc.beta > 1 )
                                       result_.bc.alpha = 1 - result_.bc.beta;
                                    }
                                }
                            }

                            // eq(FLOAT_ETERNITY)
                            return result_.ratio != FLOAT_ETERNITY;
                        }
                    }

                    return false;
                }

            private:
                Traits const & traits_;
                point_3 const& org_;
                point_3 const  dir_;
                cg::segment_3  orig_seg_;
                bool           nobackface_;
                IntersectionParams & result_;
                triangle_filter_t const & filter_;
            };
        };
    }

}