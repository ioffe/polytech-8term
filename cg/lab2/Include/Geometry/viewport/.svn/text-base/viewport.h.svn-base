#pragma once

#include "..\frustum.h"
#include "..\lerp.h"

#pragma pack(push, 1)

namespace cg
{
   struct viewport_dependent_params
   {
   private:
      inline viewport_dependent_params() {}
   public:
      float near_clip_pix_per_meter_hor,
            near_clip_pix_per_meter_ver;
      float max_dist_near,
            max_dist_far;

      friend class viewport;
   };

   class viewport
   {
   public:
      viewport();
      viewport( viewport const & other );

      viewport & operator=( viewport const & other );

      void set_frustum( frustum const & frustum );
      void set_layout ( rectangle_2i const & layout );

      rectangle_2i const & layout() const;

      cg::frustum const & frustum() const;
      cg::frustum       & frustum();

      rectangle_2f field_of_view( rectangle_2f const & screen_rect )   const;
      rectangle_2f screen_rect  ( rectangle_2f const & field_of_view ) const;

      float near_clip_pix_per_meter_hor() const;
      float near_clip_pix_per_meter_ver() const;
      float max_dist_near              () const;
      float max_dist_far               () const;

      bool     screen_point_by_world_pos( point_3 const & world_pos, point_2 * screen_point )                     const;
      point_3  world_pos_by_screen_point( point_2 const & screen_point, float camera_depth )                      const;
      void     world_ray_by_screen_point( point_2 const & screen_point, point_3 * near_plane_pos, point_3 * dir ) const;
      point_2f pixel_size_by_world_pos  ( point_3 const & world_pos )                                             const;
   private:
      viewport_dependent_params const & dependent_params() const;
   private:
      typedef boost::scoped_ptr<cg::frustum> frustum_ptr;

      rectangle_2i layout_;
      frustum_ptr  frustum_;

      mutable boost::optional<viewport_dependent_params> dependent_params_;

      mutable size_t tracked_frustum_version_;
   };

   inline viewport::viewport()
      : layout_ (point_2i(), point_2i())
   {
      frustum_.reset(new frustum_perspective(camera(), range_2f(1, 100), 30, 30));

      tracked_frustum_version_ = frustum_->version() - 1;
   }

   inline viewport::viewport( viewport const & other )
      : layout_                 (other.layout_)
      , frustum_                (other.frustum_->clone().release())
      , dependent_params_       (other.dependent_params_)
   {
      tracked_frustum_version_ = frustum_->version() - 1;
   }

   inline viewport & viewport::operator=( viewport const & other )
   {
      layout_ = other.layout();
      frustum_.reset(other.frustum_->clone().release());
      dependent_params_ = other.dependent_params_;

      tracked_frustum_version_ = other.tracked_frustum_version_;

      return *this;
   }

   inline viewport_dependent_params const & viewport::dependent_params() const
   {
      if (!dependent_params_ || tracked_frustum_version_ != frustum_->version())
      {
         tracked_frustum_version_ = frustum_->version();
         dependent_params_ = viewport_dependent_params();

         rectangle_2f const near_clip_rect = frustum_->near_clip_rect();

         // resolutions
         dependent_params_->near_clip_pix_per_meter_hor = layout_[0].size() / near_clip_rect[0].size();
         dependent_params_->near_clip_pix_per_meter_ver = layout_[1].size() / near_clip_rect[1].size();

         // maximal visible distance
         float const clipping_near = (float)frustum_->clipping_near(),
            clipping_far  = (float)frustum_->clipping_far();
         if (frustum_->type() == FR_orthographic)
         {
            dependent_params_->max_dist_near = clipping_near;
            dependent_params_->max_dist_far  = clipping_far;
         }
         else if (frustum_->type() == FR_perspective)
         {
            boost::array<point_3, 4> const corners = 
            {
                 frustum_->camera_space_frustum_point(FR_left_bottom_near)
               , frustum_->camera_space_frustum_point(FR_right_bottom_near)
               , frustum_->camera_space_frustum_point(FR_left_top_near)
               , frustum_->camera_space_frustum_point(FR_right_top_near)
            };

            double max_dist_near = 0;
            for (size_t i = 0; i < 4; ++i)
               max_dist_near = cg::max(max_dist_near, norm(corners[i]));

            dependent_params_->max_dist_near = (float)max_dist_near;
            dependent_params_->max_dist_far  = dependent_params_->max_dist_near * clipping_far / clipping_near;
         }
      }

      return *dependent_params_;
   }

   inline float viewport::near_clip_pix_per_meter_hor() const
   {
      return dependent_params().near_clip_pix_per_meter_hor;
   }

   inline float viewport::near_clip_pix_per_meter_ver() const
   {
      return dependent_params().near_clip_pix_per_meter_ver;
   }

   inline float viewport::max_dist_near() const
   {
      return dependent_params().max_dist_near;
   }

   inline float viewport::max_dist_far() const
   {
      return dependent_params().max_dist_far;
   }

   inline void viewport::set_layout( rectangle_2i const & layout )
   {
      layout_ = layout;

      dependent_params_ = boost::none;
   }

   inline void viewport::set_frustum( cg::frustum const & frustum )
   {
      frustum_.reset(frustum.clone().release());

      dependent_params_ = boost::none;

      tracked_frustum_version_ = frustum_->version() - 1;
   }

   inline rectangle_2i const & viewport::layout() const
   {
      return layout_;
   }

   inline cg::frustum const & viewport::frustum() const
   {
      return *frustum_.get();
   }

   inline cg::frustum & viewport::frustum()
   {
      return *frustum_.get();
   }

   inline rectangle_2f viewport::field_of_view( rectangle_2f const & screen_rect ) const
   {
      rectangle_2f const near_clip_rect = frustum_->near_clip_rect();

      point_2f const lo = lerp_points<point_2f, point_2f>(screen_rect.lo(), layout_.lo(),        layout_.hi(),
                                                                            near_clip_rect.lo(), near_clip_rect.hi());
      point_2f const hi = lerp_points<point_2f, point_2f>(screen_rect.hi(), layout_.lo(),        layout_.hi(),
                                                                            near_clip_rect.lo(), near_clip_rect.hi());

      return field_of_view_by_near_clip_rect(*frustum_, rectangle_2f(lo, hi));
   }

   inline rectangle_2f viewport::screen_rect( rectangle_2f const & field_of_view ) const
   {
      rectangle_2f const rect = near_clip_rect_by_field_of_view(*frustum_, field_of_view);
      rectangle_2f const near_clip_rect = frustum_->near_clip_rect();

      point_2f const lo = lerp_points(rect.lo(), near_clip_rect.lo(), near_clip_rect.hi(),
                                                 layout_.lo(),        layout_.hi());
      point_2f const hi = lerp_points(rect.hi(), near_clip_rect.lo(), near_clip_rect.hi(),
                                                 layout_.lo(),        layout_.hi());

      return rectangle_2f(lo, hi);
   }

   inline bool viewport::screen_point_by_world_pos( point_3 const & world_pos, point_2 * screen_point ) const
   {
      point_3f const point_view = frustum_->camera().world_view_gl_transform().treat_point(world_pos);

      if (ge(point_view.z, 0.0f))
         return false;

      point_4f const homogenous_ndc_pos = frustum_->projection_matrix() * point_4f(point_view, 1.0f);
      point_3f const projected_pos = point_3f(homogenous_ndc_pos) / homogenous_ndc_pos.w;

      screen_point->x = lerp<double, double>(projected_pos.x, -1, 1, layout_.lo().x, layout_.hi().x);
      screen_point->y = lerp<double, double>(projected_pos.y, -1, 1, layout_.lo().y, layout_.hi().y);

      return true;
   }

   inline point_3 viewport::world_pos_by_screen_point( point_2 const & screen_point, float camera_depth ) const
   {
      point_3 const & cam_pos = frustum_->camera().position();

      point_3 near_plane_pos, dir;
      world_ray_by_screen_point(screen_point, &near_plane_pos, &dir);

      point_3 world_pos;

      if (frustum_->type() == FR_perspective)
      {
         world_pos = cam_pos + (near_plane_pos - cam_pos) * camera_depth / frustum_->clipping_near();
      }
      else if (frustum_->type() == FR_orthographic)
      {
         world_pos = near_plane_pos + dir * (camera_depth - frustum_->clipping_near());
      }
      else
         Assert(false);

      return world_pos;
   }

   inline void viewport::world_ray_by_screen_point( point_2 const & screen_point, point_3 * near_plane_pos,
      point_3 * dir ) const
   {
      rectangle_2f const near_clip_rect = frustum_->near_clip_rect();
      point_2f near_clip_xy;
      near_clip_xy.x = (float)lerp<double, double>(screen_point.x, layout_.lo().x, layout_.hi().x,
         near_clip_rect.lo().x, near_clip_rect.hi().x);
      near_clip_xy.y = (float)lerp<double, double>(screen_point.y, layout_.lo().y, layout_.hi().y,
         near_clip_rect.lo().y, near_clip_rect.hi().y);


      point_3 const near_clip_pos(near_clip_xy.x, near_clip_xy.y, -frustum_->clipping_near());
      *near_plane_pos = frustum_->camera().world_view_gl_transform().inverted().treat_point(near_clip_pos);

      if (frustum_->type() == FR_perspective)
         *dir = normalized_safe(*near_plane_pos - point_3(frustum_->camera().position()));
      else if (frustum_->type() == FR_orthographic)
         *dir = frustum_->camera().dir();
      else
         Assert(false);
   }

   inline point_2f viewport::pixel_size_by_world_pos( point_3 const & world_pos ) const
   {
      if (frustum_->type() == FR_orthographic)
         return point_2f(1.0f / dependent_params().near_clip_pix_per_meter_hor,
                         1.0f / dependent_params().near_clip_pix_per_meter_ver);

      point_3f const view_pos = frustum_->camera().world_view_gl_transform().treat_point(world_pos);

      return (fabsf(view_pos.z) / frustum_->clipping_near()) *
         point_2f(1.0f / dependent_params().near_clip_pix_per_meter_hor,
                  1.0f / dependent_params().near_clip_pix_per_meter_ver);
   }
}

#pragma pack(pop)
