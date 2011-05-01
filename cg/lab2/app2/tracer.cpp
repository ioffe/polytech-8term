#include "stdafx.h"

#include "tracer.h"
#include "common.h"
#include "fuzzy_refl.h"

tracer::tracer()
{
   ::srand(GetTickCount());
   load_scene();
}

colorf tracer::trace( point_3 origin, point_3 dir )
{
   return do_trace(origin, dir, 1.);
}

colorf tracer::do_trace( point_3 origin, point_3 dir, double weight )
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

      double new_weight = m.kr * weight;
      if (m.fuzzy_refl && cg::eq(weight, 1.0))
      {
         int const total_rays = 50;
         int rays = 0;
         new_weight /= total_rays;
         fuzzy_refl_calc fc(-dir, d.n);
         while (rays < total_rays)
         {
            point_3 new_dir = rand_vec();
            double cs = new_dir * d.n;
            if (cs < 0)
               continue;

            double p = fc(new_dir);
            if (cg::rand(1.0) > p)
               continue;

            rays++;
            c += do_trace(d.pos, new_dir, new_weight) * (float)new_weight;
         }
      }
      else
      {
         if (d.reflect && new_weight > 0.1)
            c += do_trace(d.pos, *d.reflect, new_weight) * (float)new_weight;
      }

      res_c = c;
   }

   return res_c ? *res_c : cg::color_black();
}

void tracer::load_scene()
{
   //object_ptr plane(new plane_obj(cg::normalized(point_3(0, 0, 1)), point_3(0, 0, -3)));
   ////plane->mat().kr = 0;
   ////plane->mat().fuzzy_refl = true;
   //objs_.push_back(plane);

   object_ptr s(new sphere_obj(point_3(0, 5, 1), 2));
   //s->mat().fuzzy_refl = true;
   s->mat().color = cg::color_yellow();
   objs_.push_back(s);


   object_ptr s2(new sphere_obj(point_3(5, 5, 3), 2));
   s2->mat().fuzzy_refl = true;
   s->mat().color = cg::color_darkgray();
   objs_.push_back(s2);


   ls_.push_back(light(point_3(3, 0, 1), colorf(1, 1, 1), 1));

   ambient_ = cg::color_white();
}