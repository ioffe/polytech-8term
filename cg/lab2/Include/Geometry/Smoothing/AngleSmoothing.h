#pragma once

#include "boost\next_prior.hpp"

namespace cg
{
   namespace AngleSmoothing
   {
      class EllipseSmootherHelper
      {
      public:
         EllipseSmootherHelper ( cg::point_2 const& p0, cg::point_2 const& p1, cg::point_2 const& p2 )
            : p0_( p0 )
            , p1_( p1 )
            , p2_( p2 )
         {
            reversed_ = false;
            if (cg::distance(p0_, p1_) < cg::distance(p1_, p2_))
            {
               std::swap(p0_, p2_);
               reversed_ = true;
            }

            full_angle_      = cg::norm_pi(cg::angle(cg::normalized_safe(p1_ - p0_), cg::normalized_safe(p1_ - p2_)));
            full_angle_sign_ = cg::sign(full_angle_);

            double const dist_small = cg::distance(p1_, p2_);
            double const dist_big   = cg::distance(p0_, p1_);

            edge_scale_ = dist_small / dist_big;
            p2x_ = p1_ + ( p2_ - p1_ ) / dist_small * dist_big;

            cg::point_2 const a = p0_ - p1_;
            cg::point_2 const b = p2x_ - p1_;
            circle_center_ = b - cg::point_2(-b.y, b.x) / (cg::point_2(-a.y, a.x) * b) * (a * a - a * b) + p1_;

            double const alpha = full_angle_ / 2.0;
            double const r     = dist_big * tan(alpha);
            circle_radius_ = fabs(r);
            radius_sign_   = cg::sign(r);
         }

         template<class OutIter>
            void smooth ( double new_point_angle, OutIter & res )
         {
            double const angle = cg::pi - fabs(full_angle_);
            int          parts = (int)(angle / new_point_angle);

            if (parts * new_point_angle < angle)
               ++parts;

            new_point_angle = angle / parts;

            cg::point_2 sp = reversed_? cg::normalized_safe(p1_ - p2_)
                                      : cg::normalized_safe(p1_ - p0_);

            double         const rot_angle = (reversed_? -new_point_angle : new_point_angle) * cg::rad2grad(full_angle_sign_);
            cg::rotation_2 const rotation (rot_angle);

            for (int i = 0; i != parts + 1; ++i)
            {
               cg::point_2 const tangent = ellipse2circle_tangent(sp);

               cg::point_2 p (-tangent.y, tangent.x);
               if (reversed_)
                  p = p * (-1);

               p = cg::normalized_safe(p) * full_angle_sign_;
               p = p * circle_radius_ + circle_center_;

               res++ = circle_point2ellipse_point(p);
               sp    = sp * rotation;
            }
         }

      private:
         cg::point_2 ellipse2circle_tangent ( cg::point_2 const& r ) const
         {
            cg::point_2 const a = p0_  - p1_;
            cg::point_2 const b = p2x_ - p1_;

            double const xin = r.x;
            double const yin = r.y;

            double const tau = (edge_scale_ - 1) / (cg::point_2(-a.y, a.x) * b);

            double const x = (xin * (1 + tau * b.y * a.x) - yin * tau * b.x * a.x) / edge_scale_;
            double const y = (yin * (1 - tau * b.x * a.y) + xin * tau * b.y * a.y) / edge_scale_;

            return cg::point_2(x, y);
         }

         cg::point_2 circle_point2ellipse_point ( cg::point_2 p ) const
         {
            p -= p1_;

            cg::point_2 const a = p0_  - p1_;
            cg::point_2 const b = p2x_ - p1_;

            cg::point_2 const pp = p + (edge_scale_ - 1) * b * (cg::point_2(-a.y, a.x) * p) / (cg::point_2(-a.y, a.x) * b);
            return pp + p1_;
         }

      private:
         cg::point_2 p0_, p1_, p2_, p2x_;
         cg::point_2 circle_center_;

         double full_angle_,      circle_radius_;
         int    full_angle_sign_, radius_sign_;

         bool   reversed_;
         double edge_scale_;
      };

      template<class OutIter>
         void smooth_angle ( cg::point_2 p0, cg::point_2 p1, cg::point_2 p2,
                             double smoothing_radius, double new_point_angle,
                             OutIter result )
      {
         double const angle = cg::pi - fabs(cg::norm_pi(cg::angle(p1 - p0, p1 - p2)));
         if ((angle < new_point_angle) || (cg::eq(angle, cg::pi)))
         {
            result++ = p1;
            return;
         }

         double const dist1 = cg::distance(p0, p1);
         double const dist2 = cg::distance(p1, p2);

         if (dist1 < 2 * smoothing_radius)
            p0 = (p0 + p1) / 2;
         else
            p0 = p1 + cg::normalized_safe(p0 - p1) * smoothing_radius;

         if (dist2 < 2 * smoothing_radius)
            p2 = (p1 + p2) / 2;
         else
            p2 = p1 + cg::normalized_safe(p2 - p1) * smoothing_radius;

         EllipseSmootherHelper smoother (p0, p1, p2);
         smoother.smooth(new_point_angle, result);
      }

      template<class InIter, class OutIter>
         void smooth_polyline ( InIter p, InIter q, double smoothing_radius, double new_point_angle, OutIter out )
      {
         if (std::distance(p, q) < 3)
         {
            std::copy(p, q, out);
            return;
         }

         bool const closed = (*p == *boost::prior(q));

         if (!closed)
            *out++ = *p;

         InIter p2 = p,
                p0 = p2++,
                p1 = p2++;

         for (; p2 != q; ++p0, ++p1, ++p2)
            AngleSmoothing::smooth_angle(*p0, *p1,  *p2, smoothing_radius, new_point_angle, out);

         if (closed)
            AngleSmoothing::smooth_angle(*p0, *p, *(++p), smoothing_radius, new_point_angle, out);
         else
            *out++ = *p1;
      }
   }
}