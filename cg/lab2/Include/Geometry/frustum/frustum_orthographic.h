#pragma once

#include "geometry\primitives.h"
#include "geometry\matrix_operator.h"
#include "frustum_base.h"

#pragma pack(push, 1)

namespace cg
{
   template< typename S >
   class frustum_orthographic_t : public frustum_t<S>
   {
   public:
      frustum_orthographic_t( camera_t const & cam, rectangle_2f const & field_of_view, range_2f const & near_far );
      frustum_orthographic_t( camera_t const & cam, rectangle_3f const & frustum_volume );

      frustum_orthographic_t & operator=( frustum_orthographic_t const & other );

      template< typename _S >
      frustum_orthographic_t( frustum_orthographic_t<_S> const & other );

      template< typename _S >
      frustum_orthographic_t & operator=( frustum_orthographic_t<_S> const & other );

      virtual std::auto_ptr<frustum_t<S>> clone() const;
      virtual frustum_type                type () const;

      virtual void         set_aspect_ratio( float aspect_ratio, aspect_ratio_stretch_type stretch_type );
      virtual float        aspect_ratio    () const;
      virtual rectangle_2f near_clip_rect  () const;

      virtual void set_field_of_view_hor_range( range_2f const & range, bool reset_aspect_ratio );
      virtual void set_field_of_view_ver_range( range_2f const & range, bool reset_aspect_ratio );
   private:
      virtual void reset_optional();

      virtual void validate_projection_matrix             () const;
      virtual void validate_camera_space_gl_frustum_points() const;
      virtual void validate_camera_space_frustum_points   () const;
      virtual void validate_camera_space_gl_face_normals  () const;
      virtual void validate_camera_space_face_normals     () const;
   private:
      template< typename > friend class frustum_orthographic_t; 
   };

   template< typename S >
   frustum_orthographic_t<S>::frustum_orthographic_t( camera_t const & cam, rectangle_2f const & area,
                                                      range_2f const & near_far )
      : frustum_t(cam, rectangle_3f(area, near_far))
   {
   }

   template< typename S >
   frustum_orthographic_t<S>::frustum_orthographic_t( camera_t const & cam, rectangle_3f const & frustum_volume )
      : frustum_t(cam, frustum_volume)
   {
   }

   template< typename S >
   frustum_orthographic_t<S> & frustum_orthographic_t<S>::operator=( frustum_orthographic_t<S> const & other )
   {
      frustum_t::operator=(other);

      return *this;
   }

   template< typename S > template< typename _S >
   frustum_orthographic_t<S>::frustum_orthographic_t( frustum_orthographic_t<_S> const & other )
      : frustum_t(other)
   {
   }

   template< typename S > template< typename _S >
   frustum_orthographic_t<S> & frustum_orthographic_t<S>::operator=( frustum_orthographic_t<_S> const & other )
   {
      frustum_t::operator=(other);

      return *this;
   }

   template< typename S >
   std::auto_ptr<frustum_t<S>> frustum_orthographic_t<S>::clone() const
   {
      return std::auto_ptr<frustum_t<S>>(new frustum_orthographic_t<S>(*this));
   }

   template< typename S >
   frustum_type frustum_orthographic_t<S>::type() const
   {
      return FR_orthographic;
   }

   template< typename S >
   void frustum_orthographic_t<S>::set_aspect_ratio( float aspect_ratio, aspect_ratio_stretch_type stretch_type )
   {
      if (stretch_type == FR_stretch_vertical)
      {
         float const vertical_size = frustum_volume_.size().x / aspect_ratio;
         frustum_volume_[1] = range_2f(frustum_volume_[1].center() - vertical_size / 2,
                                       frustum_volume_[1].center() + vertical_size / 2);
      }
      else if (stretch_type == FR_stretch_horizontal)
      {
         float const horizontal_size = frustum_volume_.size().y * aspect_ratio;
         frustum_volume_[0] = range_2f(frustum_volume_[0].center() - horizontal_size / 2,
                                       frustum_volume_[0].center() + horizontal_size / 2);
      }
      else
         Assert(false);

      reset_optional();
      ++version_;
   }

   template< typename S >
   float frustum_orthographic_t<S>::aspect_ratio() const
   {
      return frustum_volume_.size().x / frustum_volume_.size().y;
   }

   template< typename S >
   rectangle_2f frustum_orthographic_t<S>::near_clip_rect() const
   {
      return frustum_volume_;
   }

   template< typename S >
   void frustum_orthographic_t<S>::set_field_of_view_hor_range( range_2f const & hrange,
                                                                bool reset_aspect_ratio )
   {
      float const saved_aspect_ratio = aspect_ratio();

      frustum_volume_.x = hrange;

      if (!reset_aspect_ratio)
      {
         float const vsize = hrange.size() / saved_aspect_ratio;
         frustum_volume_.y = range_2f(frustum_volume_.y.center()).inflate(vsize / 2);
      }

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_orthographic_t<S>::set_field_of_view_ver_range( range_2f const & vrange,
                                                                bool reset_aspect_ratio )
   {
      float const saved_aspect_ratio = aspect_ratio();

      frustum_volume_.y = vrange;

      if (!reset_aspect_ratio)
      {
         float const hsize = vrange.size() * saved_aspect_ratio;
         frustum_volume_.x = range_2f(frustum_volume_.x.center()).inflate(hsize / 2);
      }

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_orthographic_t<S>::reset_optional()
   {
      reset_optional_base();
   }


   template< typename S >
   void frustum_orthographic_t<S>::validate_projection_matrix() const
   {
      projection_matrix_ = orthographic_projection_matrix(
         frustum_volume_.lo().x, frustum_volume_.hi().x,
         frustum_volume_.lo().y, frustum_volume_.hi().y,
         frustum_volume_.lo().z, frustum_volume_.hi().z);
   }

   template< typename S >
   void frustum_orthographic_t<S>::validate_camera_space_gl_frustum_points() const
   {
      rectangle_2f const & area = frustum_volume_;
      float        const   n    = frustum_volume_.lo().z;
      float        const   f    = frustum_volume_.hi().z;

      frustum_points_gl_ = frustum_points_array();
      frustum_points_array & points = frustum_points_gl_.get();
      points[FR_left_bottom_near]  = point_3f(area.lo().x, area.lo().y, -n);
      points[FR_right_bottom_near] = point_3f(area.hi().x, area.lo().y, -n);
      points[FR_left_top_near]     = point_3f(area.lo().x, area.hi().y, -n);
      points[FR_right_top_near]    = point_3f(area.hi().x, area.hi().y, -n);
      points[FR_left_bottom_far]   = point_3f(area.lo().x, area.lo().y, -f);
      points[FR_right_bottom_far]  = point_3f(area.hi().x, area.lo().y, -f);
      points[FR_left_top_far]      = point_3f(area.lo().x, area.hi().y, -f);
      points[FR_right_top_far]     = point_3f(area.hi().x, area.hi().y, -f);
   }

   template< typename S >
   void frustum_orthographic_t<S>::validate_camera_space_frustum_points() const
   {
      rectangle_2f const & area = frustum_volume_;
      float        const   n    = frustum_volume_.lo().z;
      float        const   f    = frustum_volume_.hi().z;

      frustum_points_ = frustum_points_array();
      frustum_points_array & points = frustum_points_.get();
      points[FR_left_bottom_near]  = point_3f(area.lo().x, n, area.lo().y);
      points[FR_right_bottom_near] = point_3f(area.hi().x, n, area.lo().y);
      points[FR_left_top_near]     = point_3f(area.lo().x, n, area.hi().y);
      points[FR_right_top_near]    = point_3f(area.hi().x, n, area.hi().y);
      points[FR_left_bottom_far]   = point_3f(area.lo().x, f, area.lo().y);
      points[FR_right_bottom_far]  = point_3f(area.hi().x, f, area.lo().y);
      points[FR_left_top_far]      = point_3f(area.lo().x, f, area.hi().y);
      points[FR_right_top_far]     = point_3f(area.hi().x, f, area.hi().y);
   }

   template< typename S >
   void frustum_orthographic_t<S>::validate_camera_space_gl_face_normals() const
   {
      face_normals_gl_ = face_normals_array();
      face_normals_array & normals = face_normals_gl_.get();
      normals[FR_face_left]   = point_3f(-1,  0,  0);
      normals[FR_face_right]  = point_3f( 1,  0,  0);
      normals[FR_face_bottom] = point_3f( 0, -1,  0);
      normals[FR_face_top]    = point_3f( 0,  1,  0);
      normals[FR_face_near]   = point_3f( 0,  0,  1);
      normals[FR_face_far]    = point_3f( 0,  0, -1);
   }

   template< typename S >
   void frustum_orthographic_t<S>::validate_camera_space_face_normals() const
   {
      face_normals_ = face_normals_array();
      face_normals_array & normals = face_normals_.get();
      normals[FR_face_left]   = point_3f(-1,  0,  0);
      normals[FR_face_right]  = point_3f( 1,  0,  0);
      normals[FR_face_bottom] = point_3f( 0,  0, -1);
      normals[FR_face_top]    = point_3f( 0,  0,  1);
      normals[FR_face_near]   = point_3f( 0, -1,  0);
      normals[FR_face_far]    = point_3f( 0,  1,  0);
   }
}

#pragma pack(pop)
