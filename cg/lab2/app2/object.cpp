#include "stdafx.h"

#include "object.h"

material::material() 
   : color(cg::color_random())
   , ka(0.4f)
   , kd(1.f)
   , kr(0.5f)
   , ks(0.5f)
   , p(30)
   , fuzzy_refl(false)
{}


material& object::mat()
{
   return mat_;
}

object::~object()
{

}

//////////////////////////////////////////////////////////////////////////

plane_obj::plane_obj( point_3 const & n, point_3 const & p ) : p_(n, p)
{

}

bool plane_obj::intersect_ray( line_3 const & l, intersect_detail * detail )
{
   double t;
   if (cg::has_intersection(p_, l, &t) && cg::gt(t, 0, 0.001))
   {
      if (detail)
      {
         point_3 view = l.r();
         detail->pos = l(t);
         detail->n = normal(detail->pos);
         detail->t = t;
         double cs = view * detail->n;
         point_3 refl = view - detail->n * (2 * cs);
         detail->reflect = refl;
      }
      return true;
   }
   return false;
}

bool plane_obj::intersect_seg( segment_3 const & l, intersect_detail * detail )
{
   double t;
   if (cg::has_intersection(l, p_, &t) && cg::gt(t, 0, 0.001))
   {
      if (detail)
      {
         point_3 view = direction(l);
         detail->pos = l(t);
         detail->n = normal(detail->pos);
         detail->t = t;
         double cs = view * detail->n;
         point_3 refl = view - detail->n * (2 * cs);
         detail->reflect = refl;
      }
      return true;
   }
   return false;
}

point_3 plane_obj::normal( point_3 const & p )
{
   return p_.n();
}

//////////////////////////////////////////////////////////////////////////

sphere_obj::sphere_obj( point_3 const & c, double r ) 
   : center_(c)
   , radius_(r)
{

}

bool sphere_obj::intersect_ray( line_3 const & line, intersect_detail * detail )
{
   optional<double> t = intersect_ray_impl(line);
   if (t)
   {
      if (detail)
      {
         point_3 view = line.r();
         detail->pos = line(*t);
         detail->n = normal(detail->pos);
         detail->t = *t;
         double cs = view * detail->n;
         point_3 refl = view - detail->n * (2 * cs);
         detail->reflect = refl;       
      }
      return true;
   }
   else
      return false;
}

bool sphere_obj::intersect_seg( segment_3 const & s, intersect_detail * detail )
{
   line_3 l(s.P0(), s.P1(), cg::line::by_points);

   optional<double> t = intersect_ray_impl(l);
   if (t && cg::le(*t, cg::length(s)))
   {
      if (detail)
      {
         point_3 view = direction(s);
         detail->pos = s(*t);
         detail->n = normal(detail->pos);
         detail->t = *t;
         double cs = view * detail->n;
         point_3 refl = view - detail->n * (2 * cs);
         detail->reflect = refl;
      }
      return true;
   }
   else
      return false;
}

point_3 sphere_obj::normal( point_3 const & p )
{
   return cg::normalized_safe(p - center_);
}

optional<double> sphere_obj::intersect_ray_impl( line_3 const & line )
{
   point_3 const l = center_ - line.p(); // direction vector
   double const L2OC = l * l; // squared distance
   double const tca = l * line.r(); // closest dist to center
   double t2hc = radius_ * radius_ - L2OC + tca * tca;
   double t2;

   if (t2hc <= 0.0)
      return boost::none;

   t2hc = sqrt(t2hc);

   double t;
   if (tca < t2hc)
   {
      t = tca + t2hc;
      t2 = tca - t2hc;
   }
   else
   {
      t = tca - t2hc;
      t2 = tca + t2hc;
   }

   if (fabs(t) < 1e-6)
      t = t2;

   if (t > 1e-6)
      return t;
   else
      return boost::none;
}

//////////////////////////////////////////////////////////////////////////

