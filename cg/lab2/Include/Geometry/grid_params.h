#pragma once

#include "Geometry/aa_transform.h"

#pragma pack( push, 1 )

namespace cg
{
   struct grid_params 
   {
      grid_params() {}

      grid_params(point_2 const & org, point_2 const & unt, point_2i const & ext)
         :   tform_  (org, unt)
         ,   extents_(ext)
      {}

      aa_transform const & tform   ()  const { return tform_;           }
      point_2 const &      origin  ()  const { return tform_.origin();  }
      point_2 const &      unit    ()  const { return tform_.unit();    }
      point_2i const &     extents ()  const { return extents_;         }

   private:
      aa_transform    tform_;
      point_2i        extents_;
   };

   inline grid_params calc_grid_params( cg::rectangle_2 const & aabb, double step )
   {
      cg::point_2 const unit( step, step );
      return grid_params( aabb.lo(), unit, cg::ceil( aabb.size() / unit ) );
   }
}

#pragma pack( pop )

