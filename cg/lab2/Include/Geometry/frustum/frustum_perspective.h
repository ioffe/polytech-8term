#pragma once

#include "geometry\primitives.h"
#include "geometry\matrix_operator.h"
#include "camera.h"
#include "frustum_base.h"

#pragma pack(push, 1)

namespace cg
{
   template< typename S >
   class frustum_perspective_t;

   struct frustum_perspective_dependent_params
   {
   private:
      frustum_perspective_dependent_params()
      {
      }
   public:
      float half_hfov_tan,
            half_vfov_tan;
      float hdef_tan,
            vdef_tan;

      rectangle_2f near_clip_rect;
      rectangle_2f far_clip_rect;

      template< typename >
      friend class frustum_perspective_t;
   };

   template< typename S >
   class frustum_perspective_t : public frustum_t<S>
   {
   public:
      frustum_perspective_t( camera_t const & cam, range_2f const & near_far,
                             float hfov, float vfov, float hdef = 0, float vdef = 0 );
      frustum_perspective_t( camera_t const & cam, rectangle_3f const & frustum_volume );

      frustum_perspective_t & operator=( frustum_perspective_t const & other );

      template< typename _S >
      frustum_perspective_t( frustum_perspective_t<_S> const & other );

      template< typename _S >
      frustum_perspective_t & operator=( frustum_perspective_t<_S> const & other );

      virtual std::auto_ptr<frustum_t<S>> clone() const;
      virtual frustum_type                type () const;

      virtual void         set_aspect_ratio( float aspect_ratio, aspect_ratio_stretch_type stretch_type );
      virtual float        aspect_ratio    () const;
      virtual rectangle_2f near_clip_rect  () const;

      virtual void set_field_of_view_hor_range( range_2f const & range, bool reset_aspect_ratio );
      virtual void set_field_of_view_ver_range( range_2f const & range, bool reset_aspect_ratio );

      float hfov() const;
      float vfov() const;
      float hdeflection() const;
      float vdeflection() const;

      void set_hfov( float hfov, bool reset_aspect_ratio = true );
      void set_vfov( float vfov, bool reset_aspect_ratio = true );
      void set_hdeflection( float hdeflection );
      void set_vdeflection( float vdeflection );

      frustum_perspective_dependent_params const & dependent_params() const;
   private:
      virtual void reset_optional();

      virtual void validate_projection_matrix             () const;
      virtual void validate_camera_space_gl_frustum_points() const;
      virtual void validate_camera_space_frustum_points   () const;
      virtual void validate_camera_space_gl_face_normals  () const;
      virtual void validate_camera_space_face_normals     () const;

      range_2f     range_by_angles( float rad_fov,  float rad_def ) const;
      rectangle_2f unit_dist_rect ( float rad_hfov, float rad_vfov, float rad_hdef, float rad_vdef ) const;

   private:
      mutable boost::optional<frustum_perspective_dependent_params> frustum_dependent_params_;

      template< typename > friend class frustum_perspective_t; 
   };

   template< typename S >
   frustum_perspective_t<S>::frustum_perspective_t( camera_t const & cam, range_2f const & near_far,
                                                    float hfov, float vfov, float hdef, float vdef )
      : frustum_t(cam, rectangle_3f(point_3f(hdef - 0.5f * hfov, vdef - 0.5f * vfov, near_far.lo()),
                                    point_3f(hdef + 0.5f * hfov, vdef + 0.5f * vfov, near_far.hi())))
   {
   }

   template< typename S >
   frustum_perspective_t<S>::frustum_perspective_t( camera_t const & cam, rectangle_3f const & frustum_volume )
      : frustum_t(cam ,frustum_volume)
   {
   }

   template< typename S >
   frustum_perspective_t<S> & frustum_perspective_t<S>::operator=( frustum_perspective_t<S> const & other )
   {
      frustum_t::operator=(other);

      frustum_dependent_params_ = other.frustum_dependent_params_;

      return *this;
   }

   template< typename S > template< typename _S >
   frustum_perspective_t<S>::frustum_perspective_t( frustum_perspective_t<_S> const & other )
      : frustum_t                (other)
      , frustum_dependent_params_(other.frustum_dependent_params_)
   {
   }

   template< typename S > template< typename _S >
   frustum_perspective_t<S> & frustum_perspective_t<S>::operator=( frustum_perspective_t<_S> const & other )
   {
      frustum_t::operator=(other);

      frustum_dependent_params_ = other.frustum_dependent_params_;

      return *this;
   }

   template< typename S >
   std::auto_ptr<frustum_t<S>> frustum_perspective_t<S>::clone() const
   {
      return std::auto_ptr<frustum_t<S>>(new frustum_perspective_t<S>(*this));
   }

   template< typename S >
   frustum_type frustum_perspective_t<S>::type() const
   {
      return FR_perspective;
   }

   template< typename S >
   void frustum_perspective_t<S>::set_aspect_ratio( float aspect_ratio, aspect_ratio_stretch_type stretch_type )
   {
      rectangle_2f const rect = near_clip_rect() / clipping_near();

      if (stretch_type == FR_stretch_vertical)
      {
         float const rad_vdef = cg::grad2rad(vdeflection());

         float const vertical_size = rect.size().x / aspect_ratio;
         float const voffset = tan(rad_vdef);
         float const bottom  = voffset - vertical_size / 2;
         float const top     = voffset + vertical_size / 2;
         float const vfov = cg::rad2grad(atan(top) - atan(bottom));

         frustum_volume_[1] = range_2f(vdeflection() - 0.5f * vfov, vdeflection() + 0.5f * vfov);
      }
      else if (stretch_type == FR_stretch_horizontal)
      {
         float const rad_hdef = cg::grad2rad(hdeflection());

         float const horizontal_size = rect.size().y * aspect_ratio;
         float const hoffset = tan(rad_hdef);
         float const left  = hoffset - horizontal_size / 2;
         float const right = hoffset + horizontal_size / 2;
         float const hfov = cg::rad2grad(atan(right) - atan(left));

         frustum_volume_[1] = range_2f(hdeflection() - 0.5f * hfov, hdeflection() + 0.5f * hfov);
      }
      else
         Assert(false);

      reset_optional();
      ++version_;
   }

   template< typename S >
   float frustum_perspective_t<S>::aspect_ratio() const
   {
      point_2f const size = dependent_params().near_clip_rect.size();

      return size.x / size.y;
   }

   template< typename S >
   rectangle_2f frustum_perspective_t<S>::near_clip_rect() const
   {
      return dependent_params().near_clip_rect;
   }

   template< typename S >
   void frustum_perspective_t<S>::set_field_of_view_hor_range( range_2f const & range, bool reset_aspect_ratio )
   {
      set_hdeflection(range.center());
      set_hfov(range.size(), reset_aspect_ratio);
   }

   template< typename S >
   void frustum_perspective_t<S>::set_field_of_view_ver_range( range_2f const & range, bool reset_aspect_ratio )
   {
      set_vdeflection(range.center());
      set_vfov(range.size(), reset_aspect_ratio);
   }

   template< typename S >
   float frustum_perspective_t<S>::hfov() const
   {
      return frustum_volume_[0].size();
   }

   template< typename S >
   float frustum_perspective_t<S>::vfov() const
   {
      return frustum_volume_[1].size();
   }

   template< typename S >
   float frustum_perspective_t<S>::hdeflection() const
   {
      return frustum_volume_[0].center();
   }

   template< typename S >
   float frustum_perspective_t<S>::vdeflection() const
   {
      return frustum_volume_[1].center();
   }

   template< typename S >
   void frustum_perspective_t<S>::set_hfov( float hfov, bool reset_aspect_ratio )
   {
      if (!reset_aspect_ratio)
      {
         float const saved_aspect_ratio = aspect_ratio();

         float const rad_hfov = cg::grad2rad(hfov);
         float const rad_vfov = cg::grad2rad(vfov());
         float const rad_hdef = cg::grad2rad(hdeflection());
         float const rad_vdef = cg::grad2rad(vdeflection());

         range_2f const hrange   = range_by_angles(rad_hfov, rad_hdef);
         float const vrange_size = hrange.size() / saved_aspect_ratio;
         float const voffset     = tan(rad_vdef);
         float const bottom      = voffset - vrange_size / 2;
         float const top         = voffset + vrange_size / 2;

         float const vfov = cg::rad2grad(atan(top) - atan(bottom));

         frustum_volume_.y = range_2f(frustum_volume_.y.center()).inflate(vfov / 2);
      }

      frustum_volume_.x = range_2f(frustum_volume_.x.center()).inflate(hfov / 2);

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_perspective_t<S>::set_vfov( float vfov, bool reset_aspect_ratio )
   {
      if (!reset_aspect_ratio)
      {
         float const saved_aspect_ratio = aspect_ratio();

         float const rad_hfov = cg::grad2rad(hfov());
         float const rad_vfov = cg::grad2rad(vfov);
         float const rad_hdef = cg::grad2rad(hdeflection());
         float const rad_vdef = cg::grad2rad(vdeflection());

         range_2f const vrange = range_by_angles(rad_vfov, rad_vdef);
         float const hrange_size = vrange.size() * saved_aspect_ratio;
         float const hoffset = tan(rad_hdef);
         float const left  = hoffset - hrange_size / 2;
         float const right = hoffset + hrange_size / 2;

         float const hfov = cg::rad2grad(atan(right) - atan(left));

         frustum_volume_.x = range_2f(frustum_volume_.x.center()).inflate(hfov / 2);
      }

      frustum_volume_.y = range_2f(frustum_volume_.y.center()).inflate(vfov / 2);

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_perspective_t<S>::set_hdeflection( float hdeflection )
   {
      frustum_volume_.x = range_2f(hdeflection).inflate(frustum_volume_.x.size() / 2);

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_perspective_t<S>::set_vdeflection( float vdeflection )
   {
      frustum_volume_.y = range_2f(vdeflection).inflate(frustum_volume_.y.size() / 2);

      reset_optional();
      ++version_;
   }

   template< typename S >
   frustum_perspective_dependent_params const & frustum_perspective_t<S>::dependent_params() const
   {
      if(!frustum_dependent_params_)
      {
         frustum_dependent_params_ = frustum_perspective_dependent_params();

         float const rad_hfov = cg::grad2rad(hfov());
         float const rad_vfov = cg::grad2rad(vfov());
         float const rad_hdef = cg::grad2rad(hdeflection());
         float const rad_vdef = cg::grad2rad(vdeflection());

         rectangle_2f const & unit_rect = unit_dist_rect(rad_hfov, rad_vfov, rad_hdef, rad_vdef);

         frustum_dependent_params_->near_clip_rect = unit_rect * clipping_near();
         frustum_dependent_params_->far_clip_rect  = unit_rect * clipping_far();
         frustum_dependent_params_->half_hfov_tan  = tan(rad_hfov / 2);
         frustum_dependent_params_->half_vfov_tan  = tan(rad_vfov / 2);
         frustum_dependent_params_->hdef_tan       = tan(rad_hdef);
         frustum_dependent_params_->vdef_tan       = tan(rad_vdef);
      }

      return *frustum_dependent_params_;
   }

   template< typename S >
   void frustum_perspective_t<S>::reset_optional()
   {
      reset_optional_base();
      frustum_dependent_params_ = boost::none;
   }

   template< typename S >
   void frustum_perspective_t<S>::validate_projection_matrix() const
   {
      rectangle_2f const & near_clip_reñt = dependent_params().near_clip_rect;
      projection_matrix_ = frustum_perspective_projection_matrix(
         near_clip_reñt.lo().x,  near_clip_reñt.hi().x,
         near_clip_reñt.lo().y,  near_clip_reñt.hi().y,
         frustum_volume_.lo().z, frustum_volume_.hi().z);
   }

   template< typename S >
   void frustum_perspective_t<S>::validate_camera_space_gl_frustum_points() const
   {
      rectangle_2f const & near_clip_rect = dependent_params().near_clip_rect;
      rectangle_2f const & far_clip_rect  = dependent_params().far_clip_rect;
      float const n = frustum_volume_.lo().z;
      float const f = frustum_volume_.hi().z;

      frustum_points_gl_ = frustum_points_array();
      frustum_points_array & points = frustum_points_gl_.get();
      points[FR_left_bottom_near]  = point_3f(near_clip_rect.lo().x, near_clip_rect.lo().y, -n);
      points[FR_right_bottom_near] = point_3f(near_clip_rect.hi().x, near_clip_rect.lo().y, -n);
      points[FR_left_top_near]     = point_3f(near_clip_rect.lo().x, near_clip_rect.hi().y, -n);
      points[FR_right_top_near]    = point_3f(near_clip_rect.hi().x, near_clip_rect.hi().y, -n);
      points[FR_left_bottom_far]   = point_3f(far_clip_rect.lo().x,  far_clip_rect.lo().y,  -f);
      points[FR_right_bottom_far]  = point_3f(far_clip_rect.hi().x,  far_clip_rect.lo().y,  -f);
      points[FR_left_top_far]      = point_3f(far_clip_rect.lo().x,  far_clip_rect.hi().y,  -f);
      points[FR_right_top_far]     = point_3f(far_clip_rect.hi().x,  far_clip_rect.hi().y,  -f);
   }

   template< typename S >
   void frustum_perspective_t<S>::validate_camera_space_frustum_points() const
   {
      rectangle_2f const & near_clip_rect = dependent_params().near_clip_rect;
      rectangle_2f const & far_clip_rect  = dependent_params().far_clip_rect;
      float const n = frustum_volume_.lo().z;
      float const f = frustum_volume_.hi().z;

      frustum_points_ = frustum_points_array();
      frustum_points_array & points = frustum_points_.get();
      points[FR_left_bottom_near]  = point_3t(near_clip_rect.lo().x, n, near_clip_rect.lo().y);
      points[FR_right_bottom_near] = point_3t(near_clip_rect.hi().x, n, near_clip_rect.lo().y);
      points[FR_left_top_near]     = point_3t(near_clip_rect.lo().x, n, near_clip_rect.hi().y);
      points[FR_right_top_near]    = point_3t(near_clip_rect.hi().x, n, near_clip_rect.hi().y);
      points[FR_left_bottom_far]   = point_3t(far_clip_rect.lo().x,  f, far_clip_rect.lo().y);
      points[FR_right_bottom_far]  = point_3t(far_clip_rect.hi().x,  f, far_clip_rect.lo().y);
      points[FR_left_top_far]      = point_3t(far_clip_rect.lo().x,  f, far_clip_rect.hi().y);
      points[FR_right_top_far]     = point_3t(far_clip_rect.hi().x,  f, far_clip_rect.hi().y);
   }

   template< typename S >
   void frustum_perspective_t<S>::validate_camera_space_gl_face_normals() const
   {
      rectangle_2f const & near_clip_rect = dependent_params().near_clip_rect;
      float const n = frustum_volume_.lo().z;
      float const f = frustum_volume_.hi().z;

      face_normals_gl_ = face_normals_array();
      face_normals_array & normals = face_normals_gl_.get();
      normals[FR_face_left]   = normalized(point_3f(-n,  0, -near_clip_rect.lo().x));
      normals[FR_face_right]  = normalized(point_3f( n,  0,  near_clip_rect.hi().x));
      normals[FR_face_bottom] = normalized(point_3f( 0, -n, -near_clip_rect.lo().y));
      normals[FR_face_top]    = normalized(point_3f( 0,  n,  near_clip_rect.hi().y));
      normals[FR_face_near]   = point_3f(0,  0,  1);
      normals[FR_face_far]    = point_3f(0,  0, -1);
   }

   template< typename S >
   void frustum_perspective_t<S>::validate_camera_space_face_normals() const
   {
      rectangle_2f const & near_clip_rect = dependent_params().near_clip_rect;
      float const n = frustum_volume_.lo().z;
      float const f = frustum_volume_.hi().z;

      face_normals_ = face_normals_array();
      face_normals_array & normals = face_normals_.get();
      normals[FR_face_left]   = normalized(point_3f(-n,  near_clip_rect.lo().x,  0));
      normals[FR_face_right]  = normalized(point_3f( n, -near_clip_rect.hi().x,  0));
      normals[FR_face_bottom] = normalized(point_3f( 0,  near_clip_rect.lo().y, -n));
      normals[FR_face_top]    = normalized(point_3f( 0, -near_clip_rect.hi().y,  n));
      normals[FR_face_near]   = point_3f(0, -1, 0);
      normals[FR_face_far]    = point_3f(0,  1, 0);
   }

   template< typename S >
   range_2f frustum_perspective_t<S>::range_by_angles( float rad_fov, float rad_def ) const
   {
      return range_2f(tanf( rad_fov * 0.5f + rad_def),
                      tanf(-rad_fov * 0.5f + rad_def));
   }

   template< typename S >
   rectangle_2f frustum_perspective_t<S>::unit_dist_rect( float rad_hfov, float rad_vfov,
                                                          float rad_hdef, float rad_vdef ) const
   {
      range_2f const left_right(range_by_angles(rad_hfov, rad_hdef));
      range_2f const bottom_top(range_by_angles(rad_vfov, rad_vdef));

      return rectangle_2f(left_right, bottom_top);
   }
}

#pragma pack(pop)
