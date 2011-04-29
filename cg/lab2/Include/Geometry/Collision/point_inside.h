#pragma   once

#include "geometry/grid1L.h"
#include "calc_barycentric_coords_3d.h"

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct point_inside
        {
            typedef typename Traits::face_id face_id;

            struct PointInsideParams
            {
               point_3 normal;
               double  depth;
            };

            bool pointInside (point_3 const & p, point_3 const & dir = point_3(0,0,1), PointInsideParams * params = 0 ) const
            {
                rectangle_2 bb = cg::bounding(grid());
                bb.inflate( -1 );

                point_3 const to = p + dir * cg::norm(bb.XY() - bb.xy()) * 100.;

                Processor pointinside(self().traits(), p, to, params);

                segment_2 seg(p, to);

                if (!cull(seg, bb, seg))
                    return false;

                visit(grid(), seg, pointinside);

                return pointinside.inside();
            }

        private:

            Derived const & self () const { return static_cast<Derived const &>(*this); }

            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            {
                Processor( Traits const &traits, point_3 const &from, point_3 const &to,
                           PointInsideParams * params )
                    :   traits_ (traits)
                    ,   org_    (from)
                    ,   dir_    (to - from)
                    ,   result_ (params)
                    ,   ratio_  (1e3)
                    ,   count_  (0)
                {}

                template <class State>
                    bool operator () (State &state, typename Grid::smallcell_type const & column)
                {
                    // если клетка не пустая
                    if (!column)
                       return false;
                    
                    // определяем, какие высоты заметает луч
                    range_2 const hray(
                        org_.z + state.in_ratio  * dir_.z, 
                        org_.z + state.out_ratio * dir_.z);

                    // если диапазоны высот пересекаются
                    if (!has_intersection(hray, column->zrange()))
                       return false;

                    for (size_t i = 0, size = column->faces().size(); i != size; ++i) 
                    {
                       // если грань пересекается с лучом
                       // и это пересечение лучше
                       int        const tr_idx = column->faces()[i];
                       triangle_3 const tr     = traits_.getTriangle( tr_idx );
                       
                       double ratio;
                       if (!traits_.intersect(tr_idx, org_, dir_, ratio, false))
                          continue;

                       ++count_;

                       if ( result_ && ratio < ratio_)  
                       {
                          ratio_ = ratio;
                          result_->normal  = traits_.getTriangleNormal( tr_idx );
                          result_->depth   = cg::norm( dir_ ) * ratio;
                       }
                    }
                    return false;
                }

                bool inside() const { return (count_ % 2) == 1; }

            private:
                Traits  const &  traits_;
                point_3 const &  org_;
                point_3 const    dir_;
                PointInsideParams * result_;
                size_t           count_;
                double           ratio_;
            };
        };
    }
}
