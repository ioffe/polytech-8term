#pragma once

struct intersect_detail
{
   point_3 pos;
   optional<double> t;
   optional<point_3> refract;
   optional<point_3> reflect;
};

struct material
{
   cg::colorf color;
   float ka;
   float kd;

   explicit material(cg::colorf const & color = cg::color_white(), float ka = 0.1, float kd = 20)
      : color(color)
      , ka(ka)
      , kd(kd)
   {}
};

cg::colorf operator& (cg::colorf const & a, cg::colorf const & b)
{
   return cg::colorf(a.r * b.r, a.g * b.g, a.b * b.b);
}

struct object
{
   virtual bool intersect_ray(line_3 const & l, intersect_detail * detail) = 0;
   virtual bool intersect_seg(segment_3 const & l, intersect_detail * detail) = 0;
   virtual point_3 normal(point_3 const & p) = 0;

   material mat() { return material(); }

   virtual ~object(){};
};

struct plane_obj : public object
{
   plane_obj(point_3 const & n, point_3 const & p)
      : p_(n, p)
   {}

   virtual bool intersect_ray(line_3 const & l, intersect_detail * detail)
   {
      double t;
      if (cg::has_intersection(p_, l, &t) && t > 0)
      {
         if (detail)
         {
            detail->pos = l(t);
            detail->t = t;
         }
         return true;
      }
      return false;
   }

   virtual bool intersect_seg(segment_3 const & l, intersect_detail * detail)
   {
      double t;
      if (cg::has_intersection(l, p_, &t))
      {
         if (detail)
         {
            detail->pos = l(t);
            detail->t = t;
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
          detail->pos = line(*t);
          detail->t = t;
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
            detail->pos = s(*t);
            detail->t = t;
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

typedef std::vector<light> lights;

struct tracer
{
   tracer()
   {
      load_scene();
   }

   colorf trace(point_3 origin, point_3 dir)
   {
      line_3 l(origin, dir, cg::line::by_direction);
      optional<double> min_t;
      optional<colorf> res_c;
      for each (object_ptr const & p in objs_)
      {
         intersect_detail d;
         if (p->intersect_ray(l, &d) && (*d.t && (!min_t || *min_t > *d.t)))
         {
            min_t = d.t;
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

               if (visible)
                  c += (float)(li.shadow(d.pos) * m.kd * ln) * (li.color & m.color);
            }

            res_c = c;
         }
      }

      return res_c ? *res_c : cg::color_black();
   }



private:
   void load_scene()
   {
      objs_.push_back(object_ptr(new plane_obj(cg::normalized(point_3(0, 0, 1)), point_3(0, 0, -3))));
      objs_.push_back(object_ptr(new sphere_obj(point_3(0, 6, 1), 2)));

      ls_.push_back(light(point_3(5, 4, 1), colorf(1, 1, 1), 1.));
      ls_.push_back(light(point_3(0, 0, 1), colorf(1, 1, 1), 1.));

      ambient_ = cg::color_white();
   }

   objects objs_;
   lights ls_;
   point_3 point_source_;
   cg::colorf ambient_;

};