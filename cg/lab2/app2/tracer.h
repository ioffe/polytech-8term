#pragma once

struct intersect_detail
{
   point_3 pos;
   point_3 n;
   optional<double> t;
   optional<point_3> refract;
   optional<point_3> reflect;
};

struct material
{
   cg::colorf color;
   float ka;
   float kd;
   float kr;
   float ks, p;

   explicit material(cg::colorf const & color = cg::color_white(), float ka = 0.1, float kd = 1, float kr = 0.5, float ks = 0.5, float p = 30)
      : color(cg::color_white())
      , ka(ka)
      , kd(kd)
      , kr(kr)
      , ks(ks)
      , p(p)
   {}
};

cg::colorf operator& (cg::colorf const & a, cg::colorf const & b)
{
   return cg::colorf(a.r * b.r, a.g * b.g, a.b * b.b);
}

point_3 calc_reflection_dir(point_3 const & in_dir, point_3 const & n)
{
   double cs = in_dir * n;
   point_3 refl = in_dir - n * (2 * cs);
   return refl;
}

struct object
{
   virtual bool intersect_ray(line_3 const & l, intersect_detail * detail) = 0;
   virtual bool intersect_seg(segment_3 const & l, intersect_detail * detail) = 0;
   virtual point_3 normal(point_3 const & p) = 0;

   material& mat() { return mat_; }

   virtual ~object(){};

private:
   material mat_;
};

struct plane_obj : public object
{
   plane_obj(point_3 const & n, point_3 const & p)
      : p_(n, p)
   {}

   virtual bool intersect_ray(line_3 const & l, intersect_detail * detail)
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

   virtual bool intersect_seg(segment_3 const & l, intersect_detail * detail)
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

   virtual point_3 normal(point_3 const & p)
   {
      return p_.n();
   }

private:
   plane p_;
};

struct sphere_obj : public object
{
   sphere_obj(point_3 const & c, double r)
      : center_(c)
      , radius_(r)
   {}

   virtual bool intersect_ray(line_3 const & line, intersect_detail * detail)
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

   virtual bool intersect_seg(segment_3 const & s, intersect_detail * detail)
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

   virtual point_3 normal(point_3 const & p)
   {
      return cg::normalized_safe(p - center_);
   }

private:
   optional<double> intersect_ray_impl(line_3 const & line)
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


private:
   point_3 center_;
   double radius_;
};

typedef boost::shared_ptr<object> object_ptr;
typedef std::vector<object_ptr> objects;

struct light
{
   point_3 pos;
   cg::colorf color;
   double power;
   light(point_3 const & pos, cg::colorf const & color, double power)
      : pos(pos), color(color), power(power)
   {}

   double shadow( point_3 p ) const
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
};

//double calc_fuzzy_shadow(point_3 const & v, point_3 const & n, point_3 const & l)
//{
//   point_3 h = cg::normalized_safe(l + v);
//   double c = - i * n;
//   double g = sqrt(1 + c * c - 1);
//   double F = 0.5 * ((c - g) / (c + g))  * (1 + cg:sqr((c * (c + g) - 1) / (c * (c - g) - 1)));
//   double a = cg::angle(n, h);
//   double m = 1;
//   double D = 1 / (4 * cg::pi * m * m * pow(n * h, 4)) * exp(-cg::sqr(tan(a) / m));
//   double G = cg::min<double>(1., 2 
//}

typedef std::vector<light> lights;

struct tracer
{
   tracer()
   {
      load_scene();
   }

   colorf trace(point_3 origin, point_3 dir)
   {
      return do_trace(origin, dir, 1.);
   }

private:
   colorf do_trace(point_3 origin, point_3 dir, double weight)
   {
      line_3 l(origin, dir, cg::line::by_direction);
      optional<intersect_detail> closest_int;
      optional<colorf> res_c;
      object_ptr p;
      for each (object_ptr const & testp in objs_)
      {
         intersect_detail d;
         if (testp->intersect_ray(l, &d) && (*d.t && (!closest_int || closest_int->t > *d.t)))
         {
            closest_int = d;
            p = testp;
         }
      }

      if (closest_int)
      {
         intersect_detail d = *closest_int;
         point_3 n = p->normal(d.pos);
         material m = p->mat();

         cg::colorf c = m.ka * (ambient_ & m.color);
         for each (light const & li in ls_)
         {
            bool visible = true;

            point_3 to_light = cg::normalized_safe(li.pos - d.pos);
            double ln = (to_light * p->normal(d.pos));
            if (ln < 0)
               continue;
            for each (object_ptr const & pp in objs_)
            {
               if (p == pp)
                  continue;
               if (pp->intersect_seg(segment_3(d.pos, li.pos), NULL))
               {
                  visible = false;
                  break;
               }
            }

            float spec = (float)(m.ks * li.shadow(d.pos) * pow((float)(n * (cg::normalized(li.pos - d.pos - dir))), (float)m.p));
            if (visible)
               c += 
                  (float)(li.shadow(d.pos) * m.kd * ln) * (li.color & m.color) + 
                   spec * (li.color);
         }

         //const int steps = 5;

         //if (steps != 1)
         //{
         //   const double max_add_c = 60, max_add_p = 60;

         //   polar_point_3 pn = d.n;

         //   for (int ic = 0; ic <= steps; ++ic)
         //      for (int ip = 0; ip <= steps; ++ip)
         //      {
         //         double add_c_norm = (((double)ic / steps) * 2 - 1);
         //         double add_p_norm = (((double)ip / steps) * 2 - 1);
         //         double add_c = add_c_norm * max_add_c;
         //         double add_p = add_p_norm * max_add_p;

         //         point_3 new_n = polar_point_3(1, pn.course + add_c, pn.pitch + add_p);

         //         double new_weight = m.kr * weight * calc_fuzzy_shadow(dir, n, new_n);
         //         if (d.reflect && new_weight > 0.1)
         //            c += do_trace(d.pos, calc_reflection_dir(dir, new_n), new_weight) * (float)new_weight;
         //      }
         //}
         //else
         //{
            double new_weight = m.kr * weight;
            if (d.reflect && new_weight > 0.1)
               c += do_trace(d.pos, *d.reflect, new_weight) * (float)new_weight;
         //}


         res_c = c;
      }

      return res_c ? *res_c : cg::color_black();
   }



private:
   void load_scene()
   {
      object_ptr plane(new plane_obj(cg::normalized(point_3(0, 0, 1)), point_3(0, 0, -3)));
      //plane->mat().kr = 0;

      objs_.push_back(plane);

      objs_.push_back(object_ptr(new sphere_obj(point_3(0, 5, 1), 2)));
      objs_.push_back(object_ptr(new sphere_obj(point_3(5, 5, 3), 2)));

      ls_.push_back(light(point_3(3, 0, 1), colorf(1, 1, 1), 1));

      ambient_ = cg::color_white();
   }

   objects objs_;
   lights ls_;
   point_3 point_source_;
   cg::colorf ambient_;

};