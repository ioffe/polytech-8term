#pragma once

#include "geometry\triangle_3_intersection.h"
#include "geometry\distance_to_triangle.h"
#include "Geometry/cylinder_triangle_intersection.h"
#include "Geometry/raster_2.h"
#include "Geometry/aa_transform.h"
#include "Geometry/triangle_raster_aux.h"
#include "Geometry/simple_triangle_3.h"

namespace cg
{
    template <int N, int M>
    struct collision_grid_initializer
    {
        collision_grid_initializer(raster_2 const & r = raster_2())
            : raster_           ( r )
        {
        }

        int subdivision(int actual_n) const
        {
            int divx = ceil( cg::sqrt( (double)actual_n / N ) );
            make_min(divx, M);
            return divx;
        }
        
        aa_transform transform() const {
            return aa_transform(raster_.origin(), raster_.unit());
        }
        
        point_2i extents() const {
            return raster_.extents();
        }

        raster_2 getRaster() const {
            return raster_;
        }

    private:    
        raster_2    raster_;
    };
    
    template <class Derived, class face_id>
        struct CollisionTraitsImpl
    {
        triangle_2 getTriangle2(face_id id) const 
        {
            return triangle_2(getTriangle(id));
        }

        segment_2  getSegment2( face_id id ) const
        {
           triangle_2 const & triangle = getTriangle2( id );

           segment_2 segments[] = 
           {
               segment_2( triangle[0], triangle[1] ),
               segment_2( triangle[1], triangle[2] ),
               segment_2( triangle[2], triangle[0] )
           };

           std::pair< double, size_t >    
              max_segment( cg::distance( segments[0].P0( ), segments[0].P1( ) ), 0 );

           for ( size_t l = 1; l != 3; ++l )
           {
              const double len = cg::distance( segments[l].P0( ), segments[l].P1( ) );
              if ( len > max_segment.first )
                 max_segment = std::make_pair( len, l );
           }

           return segments[ max_segment.second ];
        }

        double getTriangleSquare(face_id id) const 
        {
            return square(getTriangle(id));
        }

        // IntersectionResult можно брать из Traits
        bool v_intersect(face_id fid, point_2 const &pt, double &h, barycentric_coords &bc) const
        {
            triangle_raster_aux triangle_aux(getTriangle(fid));

            if (triangle_aux.IsInTriangleBarycentric(pt, bc))
            {
                h = triangle_aux.GenerateHeightForPoint(pt.x, pt.y);
                return true;
            }
            return false;
        }
        
        bool intersect(face_id fid, point_3 const &org, point_3 const &dir, double &ratio, bool nobackface) const
        {
            return 
                ray_triangle3_intersection<cartesian<> >(
                    org, dir, pThis().getTriangleRef(fid), ratio, nobackface);
        }

        bool closest(double max_dist, face_id fid, point_3 const &org, double &dist, point_3 & closest) const
        {
            return distance_ex<cartesian<> >(
                max_dist, pThis().getTriangleRef(fid), org, dist, closest, cart_traits);
        }

        bool intersect_cylinder(face_id fid, point_3 const &from, point_3 const &to, double radius, double &ratio, point_3& resp, bool nobackface) const
        {
            cg::segment_3 length(from, to) ;
            return cylinder_triangle_intersection(length, radius, pThis().getTriangleRef(fid), ratio, resp, cylinder_traits_);
        }

        long & checkedFlag(face_id fid) const { return pThis().checkedFlag(fid); }

    private:

        triangle_3 getTriangle(face_id id) const {
            return pThis().getTriangle(id);
        }

    private:
        Derived       & pThis()       { return static_cast<Derived       &>(*this); }
        Derived const & pThis() const { return static_cast<Derived const &>(*this); }

    private:
        cartesian<> cart_traits;
        cylinder_intersection_details::std_traits<triangle_3_rti>  cylinder_traits_ ; 
    };

    struct CollisionStdTraits
        :   collision_grid_initializer<10, 20>
        ,   CollisionTraitsImpl<CollisionStdTraits, int>
    {
        typedef int face_id;
        
        CollisionStdTraits( 
            int vert_num,
            int face_num,
            point_3f          const * vertices,
            simple_triangle_3 const * faces,
            raster_2          const &raster = raster_2()) 
            
            :   vert_num_(vert_num), face_num_(face_num), vertices_(vertices), faces_(faces)
            ,   collision_grid_initializer<10, 20>(raster)
        {}
        
        face_id getFaceId (size_t it) const {
            return it;
        }
        
        triangle_3 getTriangle(face_id i) const 
        {
            simple_triangle_3 const &face = faces_[i];
            
            triangle_3 tr( vertices_[ face.i1 ] , 
                           vertices_[ face.i2 ] ,
                           vertices_[ face.i3 ] );
            
            return tr;
        }
        
        cg::triangle_3_rti getTriangleRef(face_id id) const
        {
            simple_triangle_3 const & tr_simple = faces_[id];
            
            return 
                triangle_3_rti ( 
                    vertices_[tr_simple.i1],
                    vertices_[tr_simple.i2],
                    vertices_[tr_simple.i3],
                    tr_simple.n );
        }
        
    private:
        point_3f          const * vertices_;
        simple_triangle_3 const * faces_;

        int vert_num_;
        int face_num_;
    };
    
    
}