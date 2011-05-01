#include "stdafx.h"

#include "light.h"

light::light( point_3 const & pos, cg::colorf const & color, double power ) 
   : pos(pos)
   , color(color)
   , power(power)
{}

double light::shadow( point_3 p ) const
{
   point_3 l = pos - p;
   double dist = cg::norm(l);
   if (cg::gt(dist, 0.01))
   {
      double att = 1.f / dist * power;
      //...
      return att * power;
   }
   else
      return 1;
}
