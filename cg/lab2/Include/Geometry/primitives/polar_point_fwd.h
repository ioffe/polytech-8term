#pragma once 

namespace cg
{
   template<typename scalar, int N> struct polar_point_t ; 

   typedef polar_point_t<double,2> polar_point_2 ; 
   typedef polar_point_t<double,3> polar_point_3 ; 

   typedef polar_point_t<float,2> polar_point_2f ; 
   typedef polar_point_t<float,3> polar_point_3f ; 
}

