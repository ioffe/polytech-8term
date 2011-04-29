#pragma once

#include "Streams\aux_traits.h"

namespace cg
{
#pragma pack(push, 1)
   template< class Traits > struct contour_2_t;

   typedef
      contour_2_t< streams::default_traits >
      contour_2;

   typedef
      contour_2_t< streams::mapped_traits >
      mapped_contour_2;

   template < class Traits > struct polygon_2_t;

   typedef
      polygon_2_t< streams::default_traits >
      polygon_2;

   typedef
      polygon_2_t< streams::mapped_traits >
      mapped_polygon_2;
#pragma pack(pop)
}