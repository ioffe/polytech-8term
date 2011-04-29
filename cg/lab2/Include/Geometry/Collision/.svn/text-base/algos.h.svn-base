#pragma     once

#include "vertical_shoot_ray.h"
#include "shoot_ray.h"
#include "height_range_in_rect.h"
#include "avg_normal_in_rect.h"
#include "get_triangles_in_contour.h"
//#include "closest_point.h"
//#include "shoot_cylinder.h"
#include "point_inside.h"
#include "obb_trimesh_intersection.h"
#include "sphere_trimesh_intersection.h"
#include "has_materials_in_rect.h"

namespace cg 
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
        struct algos
            :   cdt::vertical_shoot_ray         < Grid, Traits, Derived >
            ,   cdt::shoot_ray                  < Grid, Traits, Derived >
            ,   cdt::height_range_in_rect       < Grid, Traits, Derived >
            ,   cdt::avg_normal_in_rect         < Grid, Traits, Derived >
            ,   cdt::get_triangles_in_contour   < Grid, Traits, Derived >   
//            ,   cdt::closest_point              < Grid, Traits, Derived >
//            ,   cdt::shoot_cylinder             < Grid, Traits, Derived >
            ,   cdt::point_inside               < Grid, Traits, Derived >
            ,   cdt::obb_trimesh_intersection   < Grid, Traits, Derived >
            ,   cdt::sphere_trimesh_intersection< Grid, Traits, Derived >
            ,   cdt::has_materials_in_rect      < Grid, Traits, Derived >
        {};
    }
}