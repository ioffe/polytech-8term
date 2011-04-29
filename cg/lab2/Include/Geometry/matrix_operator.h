#pragma once

#include "boost\static_assert.hpp"
#include "boost\type_traits.hpp"

#include "primitives\matrix.h"

namespace cg
{
   //
   // Projection matrices
   //

   /* NOTE:
     clipping_near(near) and clipping_far(far) are directed from user toward screen
     (contrary to OpenGL viewer CS, directed toward user)

         correspondence
     +---------+----------+
     |  param  |  OpenGL  |
     +---------+----------+
     |  near   |  -near   |
     +---------+----------+
     |  far    |  -far    |
     +---------+----------+
   */
   template<typename S>
   matrix_t<S, 4> orthographic_projection_matrix(
      S left,           S right,
      S bottom,         S top,
      S clipping_near,  S clipping_far )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const 
         sx = 1 / (right - left), tx = -sx * (right + left),
         sy = 1 / (top - bottom), ty = -sy * (bottom + top),
         sz = 1 / (clipping_far - clipping_near), tz = -sz * (clipping_far + clipping_near);

      S const data[4][4] = 
      {
         {2 * sx, 0,      0,       tx},
         {0,      2 * sy, 0,       ty},
         {0,      0,      -2 * sz, tz},
         {0,      0,      0,        1},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<typename S>
   matrix_t<S, 4> frustum_perspective_projection_matrix(
      S left,      S right,
      S bottom,    S top,
      S near_clip, S far_clip )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const 
         dxinv = 1 / (right - left), 
         dyinv = 1 / (top - bottom), 
         dzinv = 1 / (far_clip - near_clip), 
         a = (right + left) * dxinv,
         b = (bottom + top) * dyinv,
         c = -(far_clip + near_clip) * dzinv,
         d = -2 * far_clip * near_clip * dzinv,
         e = 2 * near_clip * dxinv,
         f = 2 * near_clip * dyinv;
      
      S const data[4][4] =
      {
         {e,  0,  a,  0},
         {0,  f,  b,  0},
         {0,  0,  c,  d},
         {0,  0, -1,  0},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<typename S>
   matrix_t<S, 4> frustum_perspective_projection_matrix_inversed(
      S left,      S right,
      S bottom,    S top,
      S near_clip, S far_clip )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const 
         &f = far_clip,
         &n = near_clip,
         &r = right,
         &l = left,
         &t = top,
         &b = bottom,
         det = 1 / n / 2;
      
      S const data[4][4] = 
      {
         { (r-l)*det,         0,            0,   (r+l)*det},
         {         0, (t-b)*det,            0,   (t+b)*det},
         {         0,         0,            0,    -2*n*det},
         {         0,         0, -(f-n)/f*det, (f+n)/f*det},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<typename S>
   matrix_t<S, 4> eye_perspective_projection_matrix(
      S tanH,      S tanV,
      S defTanH,   S defTanV,
      S near_clip, S far_clip )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      return frustum_perspective_projection_matrix(
               near_clip * (-tanH + defTanH),
               near_clip * ( tanH + defTanH), 
               near_clip * (-tanV + defTanV),
               near_clip * ( tanV + defTanV), 
               near_clip, far_clip);
   }

   template<typename S>
   matrix_t<S, 4> eye_perspective_projection_matrix_inversed(
      S tanH,      S tanV,
      S defTanH,   S defTanV,
      S near_clip, S far_clip )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      return frustum_perspective_projection_matrix_inversed(
               near_clip * (-tanH + defTanH),
               near_clip * ( tanH + defTanH), 
               near_clip * (-tanV + defTanV),
               near_clip * ( tanV + defTanV), 
               near_clip, far_clip);
   }


   //
   // Viewing matrices
   //

   template<typename S>
   matrix_t<S, 4> camera_matrix(
      point_t<S, 3> const& pos,
      point_t<S, 3> const& dir,
      point_t<S, 3> const& up,
      point_t<S, 3> const& side )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const data[4][4] = 
      {
         { side.x,  side.y, side.z, -pos * side},
         {   up.x,    up.y,   up.z, -pos *   up},
         {  dir.x,   dir.y,  dir.z, -pos *  dir},
         {      0,       0,      0,           1},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<typename S>
   matrix_t<S, 4> camera_matrix(
      point_t<S, 3> const& pos,
      point_t<S, 3> const& look,
      point_t<S, 3> const& upward )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      point_t<S, 3> const 
         dir = normalized_safe(look - pos),
         side = normalized_safe(dir ^ upward),
         up = side ^ dir;
      return camera_matrix(pos, dir, up, side);
   }

   template<typename S>
   matrix_t<S, 4> camera_matrix_inversed(
      point_t<S, 3> const& pos,
      point_t<S, 3> const& dir,
      point_t<S, 3> const& up,
      point_t<S, 3> const& side )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const data[4][4] = 
      {
         { side.x, up.x, dir.x, pos.x},
         { side.y, up.y, dir.y, pos.y},
         { side.z, up.z, dir.z, pos.z},
         {      0,    0,     0,     1},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<typename S>
   matrix_t<S, 4> camera_matrix_inversed(
      point_t<S, 3> const& pos,
      point_t<S, 3> const& look,
      point_t<S, 3> const& upward )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      point_t<S, 3> const 
         dir = normalized_safe(look - pos),
         side = normalized_safe(dir ^ upward),
         up = side ^ dir;
      return camera_matrix_inversed(pos, -dir, up, side);
   }


   //
   // Generic transformation matrices
   //

   template<typename S>
   matrix_t<S, 4> translation_matrix( S tx, S ty, S tz )
   {
      S const data[4][4] = 
      {
         { 1,  0,  0,  tx},
         { 0,  1,  0,  ty},
         { 0,  0,  1,  tz},
         { 0,  0,  0,   1},
      };

      return matrix_t<S, 4>((S *)data);
   }


   template<typename S>
   matrix_t<S, 4> translation_matrix( point_t<S, 3> const& t )
   {
      return translation_matrix(t.x, t.y, t.z);
   }



   //
   // Texture coordinates matrices
   //

   template<class S>
   matrix_t<S, 4> xyzw_to_strq_remap_matrix( S width, S height )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const data[4][4] =
      {
         { (S)0.5 * width,               0,      0,  (S)0.5 * width},
         {              0, (S)0.5 * height,      0, (S)0.5 * height},
         {              0,               0, (S)0.5,          (S)0.5},
         {              0,               0,      0,               1},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<class S>
   matrix_t<S, 4> xyzw_to_strq_remap_matrix( const cg::point_t<S, 2 > & leftBottom,
      const cg::point_t<S, 2 > & rightTop )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      const S width = rightTop.x - leftBottom.x;
      const S height = rightTop.y - leftBottom.y;

      const S xSum = rightTop.x + leftBottom.x;
      const S ySum = rightTop.y + leftBottom.y;

      S const data[4][4] =
      {
         { (S)0.5 * width,               0,      0, (S)0.5 * (xSum)},
         {              0, (S)0.5 * height,      0, (S)0.5 * (ySum)},
         {              0,               0, (S)0.5,          (S)0.5},
         {              0,               0,      0,               1},
      };

      return matrix_t<S, 4>((S *)data);
   }


   //
   // Perspective 2D matrices (by Denis Burkov)
   //

   template<typename S>
   matrix_t<S, 4> perspective_2d_projection_matrix( 
      S left, S right, 
      S znear, S zfar )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const 
         dxinv = 1 / (right - left), 
         dzinv = 1 / (zfar - znear), 
         a = (right + left) * dxinv,
         c = (zfar + znear) * dzinv,
         d = -2 * zfar * znear * dzinv,
         e = 2 * znear * dxinv;

      S const data[4][4] = 
      {
         { e,  a,  0,  0},
         { 0,  c,  0,  d},
         { 0,  0,  1,  0},
         { 0,  1,  0,  0},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template<typename S>
   matrix_t<S, 4> perspective_2d_projection_matrix_inversed( 
      S left, S right, 
      S znear, S zfar )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      S const 
         &f = zfar,
         &n = znear,
         &r = right,
         &l = left,
         det = 1 / n / 2;

      S const data[4][4] = 
      {
         { (r-l)*det,            0, 0,   (r+l)*det},
         {         0,            0, 0,     2*n*det},
         {         0,            0, 1,           0},
         {         0, -(f-n)/f*det, 0, (f+n)/f*det},
      };

      return matrix_t<S, 4>((S *)data);
   }

   template< typename S >
   matrix_t<S, 4> oblique_clipping_projection_matrix( matrix_t<S, 4> const & projection,
      plane_t<S> const & view_space_plane )
   {
      BOOST_STATIC_ASSERT(!boost::is_integral<S>::value);

      point_t<S, 3> const & normal_view = view_space_plane.n();

      cg::matrix_t<S, 4> oblique_projection = projection;

      // calculate the clip-space corner point opposite the clipping plane
      // as (sgn(clipPlane.x), sgn(clipPlane.y), 1, 1) and
      // transform it into camera space by multiplying it
      // by the inverse of the projection matrix
      point_t<S, 4> q;
      q.x = (cg::sign(normal_view.x) + projection(0, 2)) / projection(0, 0);
      q.y = (cg::sign(normal_view.y) + projection(1, 2)) / projection(1, 1);
      q.z = -1.0;
      q.w = (S(1) + projection(2, 2)) / projection(2, 3);

      // Calculate scaled plane vector
      S const D = view_space_plane.d();
      S const dot = 2.0f / (normal_view * q + q.w * D);
      point_t<S, 4> const c(normal_view * dot, D * dot);

      // Replace projection matrix third row for depth remap
      oblique_projection.put_row(2, point_t<S, 4>(c.x, c.y, c.z + 1, c.w));

      return oblique_projection;
   }
}
