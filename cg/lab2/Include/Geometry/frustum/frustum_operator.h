#pragma once

#include "frustum_perspective.h"
#include "frustum_orthographic.h"

namespace cg
{
   template< class S >
   frustum_orthographic_t<S> change_frustum_type( cg::frustum_perspective_t<S> const & frp )
   {
      return cg::frustum_orthographic_t<S>(frp.camera(), frp.near_clip_rect(), frp.clipping());
   }

   template< class S >
   frustum_perspective_t<S> change_frustum_type( cg::frustum_orthographic_t<S> const & fro )
   {
      typedef cg::frustum_orthographic_t<S> frustum_type;

      float const clipping_near = fro.clipping_near(),
                  clipping_far  = fro.clipping_far ();
      rectangle_2f const near_clip_rect = fro.near_clip_rect();
      point_2f const near_clip_center_pos(near_clip_rect.center());

      float const hleft   = cg::rad2grad(atanf(near_clip_rect.lo().x / clipping_near));
      float const hright  = cg::rad2grad(atanf(near_clip_rect.hi().x / clipping_near));
      float const vbottom = cg::rad2grad(atanf(near_clip_rect.lo().y / clipping_near));
      float const vtop    = cg::rad2grad(atanf(near_clip_rect.hi().y / clipping_near));

      float const hfov = hright - hleft;
      float const vfov = vbottom - vtop;
      float const hdef = (hright + hleft) / 2;

      float const vdef = (vbottom + vtop) / 2;

      return cg::frustum_perspective_t<S>(fro.camera(), rectangle_3f(range_2f(hleft, hright), range_2f(vbottom, vtop), range_2f(clipping_near, clipping_far)));
   }

   template< class S >
   rectangle_2f field_of_view_by_near_clip_rect( frustum_t<S> const & fr, rectangle_2f const & rect )
   {
      if (fr.type() == FR_orthographic)
         return rect;

      Assert(fr.type() == cg::FR_perspective);

      float const near_clip = fr.clipping_near();

      point_2f const lo(cg::rad2grad(atan(rect.x.lo() / near_clip)), cg::rad2grad(atan(rect.y.lo() / near_clip)));
      point_2f const hi(cg::rad2grad(atan(rect.x.hi() / near_clip)), cg::rad2grad(atan(rect.y.hi() / near_clip)));

      return rectangle_2f(lo, hi);
   }

   template< class S >
   rectangle_2f near_clip_rect_by_field_of_view( frustum_t<S> const & fr, rectangle_2f const & rect )
   {
      if (fr.type() == FR_orthographic)
         return rect;

      Assert(fr.type() == cg::FR_perspective);

      float const near_clip = fr.clipping_near();

      point_2f const lo = point_2f(tan(cg::grad2rad(rect.x.lo())), tan(cg::grad2rad(rect.y.lo()))) * near_clip;
      point_2f const hi = point_2f(tan(cg::grad2rad(rect.x.hi())), tan(cg::grad2rad(rect.y.hi()))) * near_clip;

      return rectangle_2f(lo, hi);
   }

   template< class S >
   float frustum_z_by_pseudo_depth( frustum_t<S> const & fr, float pseudo_depth )
   {
      matrix_4f const & proj = fr.projection_matrix();

      return proj(2, 3) / (pseudo_depth * proj(3, 2) - proj(2, 2));
   }
}
