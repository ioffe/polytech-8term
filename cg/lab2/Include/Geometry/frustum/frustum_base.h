#pragma once

#include <boost\array.hpp>
#include <boost\optional.hpp>

#include "geometry\primitives.h"
#include "geometry\matrix_operator.h"
#include "geometry\primitives\primitives_typedef.h"
#include "camera_base.h"

#pragma pack(push, 1)

namespace cg
{
   enum frustum_point_id
   {
      FR_left_bottom_near = 0,
      FR_right_bottom_near,
      FR_left_top_near,
      FR_right_top_near,
      FR_left_bottom_far,
      FR_right_bottom_far,
      FR_left_top_far,
      FR_right_top_far,

      FR_total_points
   };

   enum frustum_face_id
   {
      FR_face_left = 0,
      FR_face_right,
      FR_face_bottom,
      FR_face_top,
      FR_face_near,
      FR_face_far,

      FR_total_faces
   };

   enum aspect_ratio_stretch_type
   {
      FR_stretch_horizontal = 0,
      FR_stretch_vertical
   };

   template< typename S >
   class frustum_t : public primitives_typedef<S>
   {
   public:
      typedef camera_t<S> camera_t;
   protected:
      frustum_t( camera_t const & cam, rectangle_3f const & frustum_volume );

      frustum_t( frustum_t const & other );

      frustum_t & operator=( frustum_t const & other );

      template< typename _S >
      frustum_t( frustum_t<_S> const & other );

      template< typename _S >
      frustum_t & operator=( frustum_t<_S> const & other );
   public:
      virtual ~frustum_t();

      virtual std::auto_ptr<frustum_t<S>> clone() const = 0;
      virtual frustum_type                type () const = 0;

      camera_t       & camera();
      camera_t const & camera() const;

      void                 set_frustum_volume( rectangle_3f const & frustum_volume );
      rectangle_3f const & frustum_volume    () const;

      matrix_4f const & projection_matrix() const;

      point_3f const & camera_space_gl_frustum_point( frustum_point_id point_id ) const;
      point_3f const & camera_space_frustum_point   ( frustum_point_id point_id ) const;
      point_3t         world_space_frustum_point    ( frustum_point_id point_id ) const;
      point_3t         world_space_frustum_vector   ( frustum_point_id point_id ) const;

      point_3f const & camera_space_gl_face_normal( frustum_face_id face_id ) const;
      point_3f const & camera_space_face_normal   ( frustum_face_id face_id ) const;
      point_3f         world_space_face_normal    ( frustum_face_id face_id ) const;

      void set_clipping     ( range_2f const & near_far );
      void set_clipping_near( float clipping_near );
      void set_clipping_far ( float clipping_far );

      range_2f clipping     () const;
      float    clipping_near() const;
      float    clipping_far () const;

      size_t version() const;

      virtual void         set_aspect_ratio( float aspect_ratio, aspect_ratio_stretch_type stretch_type ) = 0;
      virtual float        aspect_ratio    () const                                                       = 0;
      virtual rectangle_2f near_clip_rect  () const                                                       = 0;

      void                 set_field_of_view( rectangle_2f const & field_of_view );
      rectangle_2f const & field_of_view    () const;

      virtual void set_field_of_view_hor_range( range_2f const & range, bool reset_aspect_ratio = true ) = 0;
      virtual void set_field_of_view_ver_range( range_2f const & range, bool reset_aspect_ratio = true ) = 0;

      void set_field_of_view_hor_range_size( float size, bool reset_aspect_ratio = true );
      void set_field_of_view_ver_range_size( float size, bool reset_aspect_ratio = true );

      void set_field_of_view_hor_range_center( float center );
      void set_field_of_view_ver_range_center( float center );

   protected:
      void reset_optional_base();

      template< typename _S >
      void copy_optional_data( frustum_t<_S> const & other );

      void validate_camera_dependent_data() const;
   private:
      virtual void reset_optional() = 0;

      virtual void validate_projection_matrix             () const = 0;
      virtual void validate_camera_space_gl_frustum_points() const = 0;
      virtual void validate_camera_space_frustum_points   () const = 0;
      virtual void validate_camera_space_gl_face_normals  () const = 0;
      virtual void validate_camera_space_face_normals     () const = 0;
   protected:
      typedef boost::array<point_3f, FR_total_points> frustum_points_array;
      typedef boost::array<point_3f, FR_total_faces>  face_normals_array;

      struct camera_dependent_data
      {
         frustum_points_array frustum_points_world;
         face_normals_array   face_normals_world;
      };

      camera_t     camera_;
      rectangle_3f frustum_volume_;

      mutable optional<matrix_4f>             projection_matrix_;
      mutable optional<frustum_points_array>  frustum_points_;
      mutable optional<frustum_points_array>  frustum_points_gl_;
      mutable optional<face_normals_array>    face_normals_;
      mutable optional<face_normals_array>    face_normals_gl_;
      mutable optional<camera_dependent_data> camera_dependent_data_;

      mutable size_t tracked_camera_version_;
      size_t version_;

      template< typename > friend class frustum_t;
   };

   template< typename S >
   frustum_t<S>::frustum_t( camera_t const & cam, rectangle_3f const & frustum_volume )
      : camera_        (cam)
      , frustum_volume_(frustum_volume)
      , version_       (-1)
   {
      tracked_camera_version_ = camera_.version() - 1;
   }

   template< typename S >
   frustum_t<S>::frustum_t( frustum_t<S> const & other )
      : camera_           (other.camera_)
      , projection_matrix_(other.projection_matrix_)
      , frustum_volume_   (other.frustum_volume_)
      , version_          (-1)
   {
      copy_optional_data(other);

      tracked_camera_version_ = camera_.version() - 1;
   }

   template< typename S >
   frustum_t<S> & frustum_t<S>::operator=( frustum_t<S> const & other )
   {
      camera_            = other.camera_;
      projection_matrix_ = other.projection_matrix_;
      frustum_volume_    = other.frustum_volume_;

      copy_optional_data(other);

      ++version_;

      return *this;
   }

   template< typename S > template< typename _S >
   frustum_t<S>::frustum_t( frustum_t<_S> const & other )
      : camera_           (other.camera_)
      , projection_matrix_(other.projection_matrix_)
      , frustum_volume_   (other.frustum_volume_)
      , version_          (-1)
   {
      copy_optional_data(other);

      tracked_camera_version_ = camera_.version() - 1;
   }

   template< typename S > template< typename _S >
   frustum_t<S> & frustum_t<S>::operator=( frustum_t<_S> const & other )
   {
      camera_            = other.camera_;
      projection_matrix_ = other.projection_matrix_;
      frustum_volume_    = other.frustum_volume_;

      copy_optional_data(other);

      ++version_;

      return *this;
   }

   template< typename S >
   frustum_t<S>::~frustum_t()
   {
   }

   template< typename S >
   typename frustum_t<S>::camera_t & frustum_t<S>::camera()
   {
      return camera_;
   }

   template< typename S >
   typename frustum_t<S>::camera_t const & frustum_t<S>::camera() const
   {
      return camera_;
   }

   template< typename S >
   void frustum_t<S>::set_frustum_volume( rectangle_3f const & frustum_volume )
   {
      frustum_volume_ = frustum_volume;

      reset_optional();
      ++version_;
   }

   template< typename S >
   rectangle_3f const & frustum_t<S>::frustum_volume() const
   {
      return frustum_volume_;
   }

   template< typename S >
   matrix_4f const & frustum_t<S>::projection_matrix() const
   {
      if (!projection_matrix_)
         validate_projection_matrix();

      return *projection_matrix_;
   }

   template< typename S >
   point_3f const & frustum_t<S>::camera_space_gl_frustum_point( frustum_point_id point_id ) const
   {
      if (!frustum_points_gl_)
         validate_camera_space_gl_frustum_points();

      return (*frustum_points_gl_)[point_id];
   }

   template< typename S >
   point_3f const & frustum_t<S>::camera_space_frustum_point( frustum_point_id point_id ) const
   {
      if (!frustum_points_)
         validate_camera_space_frustum_points();

      return (*frustum_points_)[point_id];
   }

   template< typename S >
   typename frustum_t<S>::point_3t frustum_t<S>::world_space_frustum_point( frustum_point_id point_id ) const
   {
      if (!camera_dependent_data_ || tracked_camera_version_ != camera_.version())
      {
         validate_camera_dependent_data();

         tracked_camera_version_ = camera_.version();
      }

      return camera_dependent_data_->frustum_points_world[point_id];
   }

   template< typename S >
   typename frustum_t<S>::point_3t frustum_t<S>::world_space_frustum_vector( frustum_point_id point_id ) const
   {
      return world_space_frustum_point(point_id) - camera_.position();
   }

   template< typename S >
   point_3f const & frustum_t<S>::camera_space_gl_face_normal( frustum_face_id face_id ) const
   {
      if (!face_normals_gl_)
         validate_camera_space_gl_face_normals();

      return (*face_normals_gl_)[face_id];
   }

   template< typename S >
   point_3f const & frustum_t<S>::camera_space_face_normal( frustum_face_id face_id ) const
   {
      if (!face_normals_)
         validate_camera_space_face_normals();

      return (*face_normals_)[face_id];
   }

   template< typename S >
   point_3f frustum_t<S>::world_space_face_normal( frustum_face_id face_id ) const
   {
      if (!camera_dependent_data_ || tracked_camera_version_ != camera_.version())
      {
         validate_camera_dependent_data();

         tracked_camera_version_ = camera_.version();
      }

      return camera_dependent_data_->face_normals_world[face_id];
   }

   template< typename S >
   void frustum_t<S>::set_clipping( range_2f const & near_far )
   {
      frustum_volume_[2] = near_far;

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_t<S>::set_clipping_near( float clipping_near )
   {
      frustum_volume_[2] = range_2f(clipping_near, clipping_far());

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_t<S>::set_clipping_far( float clipping_far )
   {
      frustum_volume_[2] = range_2f(clipping_near(), clipping_far);

      reset_optional();
      ++version_;
   }

   template< typename S >
   range_2f frustum_t<S>::clipping() const
   {
      return frustum_volume_.z;
   }

   template< typename S >
   float frustum_t<S>::clipping_near() const
   {
      return frustum_volume_.lo().z;
   }

   template< typename S >
   float frustum_t<S>::clipping_far() const
   {
      return frustum_volume_.hi().z;
   }

   template< typename S >
   size_t frustum_t<S>::version() const
   {
      return version_ + camera_.version();
   }

   template< typename S >
   void frustum_t<S>::set_field_of_view( rectangle_2f const & field_of_view )
   {
      frustum_volume_.x = field_of_view.x;
      frustum_volume_.y = field_of_view.y;

      reset_optional();
      ++version_;
   }

   template< typename S >
   rectangle_2f const & frustum_t<S>::field_of_view() const
   {
      return frustum_volume_;
   }

   template< typename S >
   void frustum_t<S>::set_field_of_view_hor_range_size( float size, bool reset_aspect_ratio )
   {
      set_field_of_view_hor_range(range_2f(frustum_volume_.x.center()).inflate(size / 2), reset_aspect_ratio);
   }

   template< typename S >
   void frustum_t<S>::set_field_of_view_ver_range_size( float size, bool reset_aspect_ratio )
   {
      set_field_of_view_ver_range(range_2f(frustum_volume_.x.center()).inflate(size / 2), reset_aspect_ratio);
   }

   template< typename S >
   void frustum_t<S>::set_field_of_view_hor_range_center( float center )
   {
      frustum_volume_.x = range_2f(center).inflate(frustum_volume_.x.size() / 2);

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_t<S>::set_field_of_view_ver_range_center( float center )
   {
      frustum_volume_.y = range_2f(center).inflate(frustum_volume_.y.size() / 2);

      reset_optional();
      ++version_;
   }

   template< typename S >
   void frustum_t<S>::reset_optional_base()
   {
      projection_matrix_ = boost::none;

      frustum_points_    = boost::none;
      frustum_points_gl_ = boost::none;

      face_normals_    = boost::none;
      face_normals_gl_ = boost::none;

      camera_dependent_data_ = boost::none;
   }

   template< typename S > template< typename _S >
   void frustum_t<S>::copy_optional_data( frustum_t<_S> const & other )
   {
      if (other.frustum_points_)
      {
         frustum_points_ = frustum_points_array();
         std::copy(other.frustum_points_->begin(), other.frustum_points_->end(),
            stdext::checked_array_iterator<frustum_points_array::iterator>(frustum_points_->begin(), FR_total_points));
      }

      if (other.frustum_points_gl_)
      {
         frustum_points_gl_ = frustum_points_array();
         std::copy(other.frustum_points_gl_->begin(), other.frustum_points_gl_->end(),
            stdext::checked_array_iterator<frustum_points_array::iterator>(frustum_points_gl_->begin(), FR_total_points));
      }

      if (other.face_normals_)
      {
         face_normals_ = face_normals_array();
         std::copy(other.face_normals_->begin(), other.face_normals_->end(),
            stdext::checked_array_iterator<face_normals_array::iterator>(face_normals_->begin(), FR_total_faces));
      }

      if (other.face_normals_gl_)
      {
         face_normals_gl_ = face_normals_array();
         std::copy(other.face_normals_gl_->begin(), other.face_normals_gl_->end(),
            stdext::checked_array_iterator<face_normals_array::iterator>(face_normals_gl_->begin(), FR_total_faces));
      }

      if (other.camera_dependent_data_)
      {
         camera_dependent_data_ = camera_dependent_data();

         std::copy(other.camera_dependent_data_->frustum_points_world.begin(), other.camera_dependent_data_->frustum_points_world.end(),
            stdext::checked_array_iterator<frustum_points_array::iterator>(camera_dependent_data_->frustum_points_world.begin(), FR_total_points));

         std::copy(other.camera_dependent_data_->face_normals_world.begin(), other.camera_dependent_data_->face_normals_world.end(),
            stdext::checked_array_iterator<frustum_points_array::iterator>(camera_dependent_data_->face_normals_world.begin(), FR_total_faces));
      }
   }

   template< typename S >
   void frustum_t<S>::validate_camera_dependent_data() const
   {
      camera_dependent_data_ = camera_dependent_data();

      transform_4f const view_world(camera_.world_view_transform().inverted());

      // Frustum points
      frustum_points_array & points = camera_dependent_data_->frustum_points_world;

      for (size_t i = 0; i < FR_total_points; ++i)
         points[i] = view_world * camera_space_frustum_point((frustum_point_id)i);

      // Face normals
      face_normals_array & normals = camera_dependent_data_->face_normals_world;

      for (size_t i = 0; i < FR_total_faces; ++i)
         normals[i] = view_world.treat_normal(camera_space_face_normal((frustum_face_id)i));
   }
}

#pragma pack(pop)
