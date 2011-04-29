#pragma   once

#include "geometry/grid1L.h"
#include "Geometry/grid2l/visit_grid2l_by_directed_rect.h"
#include "geometry/Collision/calc_barycentric_coords_3d.h"

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
        struct shoot_cylinder
        {
            shoot_cylinder() : rectangle_traits_(grid()) {}

            typedef typename Traits::face_id face_id;

            struct IntersectionCylinderParams
            {
                double ratio; 
                face_id face;
                cg::barycentric_coords bc;
            };

            bool shootCylinder (point_3 const &from, point_3 const &to, double radius, bool nobackface, IntersectionCylinderParams &result)
            {
                Processor shootcylinder(self().traits(), from, to, radius, nobackface, result);

                point_2 p0 = from;
                point_2 p1 = to;
                cg::point_2 dir = ( p1 - p0 ) ;
                double dist = norm ( dir ) ;
                if ( cg::eq_zero ( dist ) ) 
                    return FALSE;
                dir /= dist ;
                p0 -= dir * radius ;
                p1 += dir * radius ;
                cg::point_2 n ( -dir.y, dir.x ) ;
                cg::naa_rectangle_2 rect ( ( p1 + p0 ) / 2., ( p1 - p0 ) / 2., n * radius ) ;

                return visit_grid2l_by_directed_rect(rectangle_traits_, rect, p0, dir, dist, shootcylinder);
            }

        private:

            Derived const & self () const { return static_cast<Derived const &>(*this); }

            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            {
                Processor(Traits const &traits, point_3 const &from, 
                    point_3 const &to, double radius, bool nobackface, IntersectionCylinderParams &result)
                    :   traits_ (traits)
                    ,   from_   (from)
                    ,   to_     (to)
                    ,   radius_ (radius)
                    ,   result_ (result)
                    ,   nobackface_ (nobackface)
                {}

                template <class State>
                    bool operator () (State &state, typename Grid::smallcell_type const & column)
                {
                    // если клетка не пустая
                    if (column)
                    {
                        // ищем лучшее пересечение с лучом
                        result_.ratio = FLOAT_ETERNITY;

                        double ratio;

                        for (DWORD i = 0; i < column->faces().size(); ++i) 
                        {
                            // если грань пересекается с лучом                                 
                            // и это пересечение лучше
                            int tr_idx = column->faces()[i];
                            triangle_3 tr = traits_.getTriangle( tr_idx );
                            point_3 resp;
                            if ( traits_.intersect_cylinder(tr_idx, from_, to_, radius_, ratio, resp, nobackface_) 
                              && ratio < result_.ratio) 
                            {
                                 result_.ratio = ratio;
                                 result_.face  = column->faces()[i];    

                                 //// находим точку пересечения
                                 //point_2 ipt = resp;

                                 // считаем бароцентрические координаты
                                 //Verify( calc_barycentric_coords_3d( tr, resp, result_.bc );
                                 if( !calc_barycentric_coords_3d( tr, resp, result_.bc ) )
                                 {
                                    Assert( false );
                                    result_.bc = barycentric_coords( 1., 0. );
                                 }
                                 //cg::triangle_raster_aux tr_aux( tr );
                                 //Verify( tr_aux.IsInTriangleBarycentric( ipt, result_.bc ) );
                            }
                        }

                        // eq(FLOAT_ETERNITY)
                        return result_.ratio != FLOAT_ETERNITY;
                    }

                    return false;
                }

            private:
                Traits const & traits_;
                point_3 from_;
                point_3 to_;
                double radius_;
                bool           nobackface_;
                IntersectionCylinderParams & result_;
            };

            grid2l_by_directed_rect::traits<Grid> rectangle_traits_;
        };
    }
}

