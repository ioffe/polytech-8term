#include "stdafx.h"
#include "common.h"

cg::colorf operator& (cg::colorf const & a, cg::colorf const & b)
{
   return cg::colorf(a.r * b.r, a.g * b.g, a.b * b.b);
}

point_3 rand_vec( double norm /*= 1.0*/ )
{
   point_3 v(cg::symmetric_rand(1.0), cg::symmetric_rand(1.0), cg::symmetric_rand(1.0));
   return norm * cg::normalized_safe(v);
}
