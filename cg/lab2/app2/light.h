#pragma once

struct light
{
   point_3 pos;
   cg::colorf color;
   double power;
   light(point_3 const & pos, cg::colorf const & color, double power);

   double shadow( point_3 p ) const;
};
typedef std::vector<light> lights;
