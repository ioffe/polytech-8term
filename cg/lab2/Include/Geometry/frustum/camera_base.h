#pragma once

#include <boost\optional.hpp>

#include "geometry\primitives.h"
#include "geometry\matrix_operator.h"
#include "geometry\primitives\primitives_typedef.h"
#include "geometry\common_cs_conversions.h"

#pragma pack(push, 1)

namespace cg
{
   template< typename S >
   class camera_t : public primitives_typedef<S>
   {
   public:
      camera_t();
      camera_t( point_3t const & pos,  cpr_t const & orientation );
      camera_t( point_3t const & look, point_3t const & look_at, point_3t const & up );
      camera_t( point_3t const & pos,  point_3t const & dir,     point_3t const & up, point_3t const & right );

      camera_t( camera_t const & other );

      camera_t & operator=( camera_t const & other);

      template< typename _S >
      camera_t( camera_t<_S> const & other );

      template< typename _S >
      camera_t & operator=( camera_t<_S> const & other );

      void set_position   ( point_3t const & position );
      void set_orientation( cpr_t    const & orientation );

      point_3t const & position   () const;
      cpr_t    const & orientation() const;

      point_3t dir  () const;
      point_3t up   () const;
      point_3t right() const;

      transform_4t const & world_view_transform   () const;
      transform_4t const & world_view_gl_transform() const;

      size_t version() const;
   private:
      void reset_transform    ();
      void calculate_transform() const;
   private:
      point_3t pos_;
      cpr_t    orientation_;
      mutable boost::optional<transform_4t> world_view_;
      mutable boost::optional<transform_4t> world_view_gl_;

      size_t version_;

      template< typename > friend class camera_t; 
   };

   template< typename S >
   camera_t<S>::camera_t()
      : version_(-1)
   {
   }

   template< typename S >
   camera_t<S>::camera_t( point_3t const & pos, cpr_t const & orientation )
      : pos_        (pos)
      , orientation_(orientation)
      , version_    (-1)
   {
   }

   template< typename S >
   camera_t<S>::camera_t( point_3t const & pos, point_3t const & look_at, point_3t const & up )
      : pos_    (pos)
      , version_(-1)
   {
      world_view_ = transform_4t(swapper(camera_matrix(pos, look_at, up)), ss_unscaled);
      world_view_gl_ = axes_swapper_modelview_cg_2_gl<S>()(*world_view_);
      orientation_ = world_view_->inverted().rotation().cpr();
   }

   template< typename S >
   camera_t<S>::camera_t( point_3t const & pos, point_3t const & dir, point_3t const & up, point_3t const & right )
      : pos_    (pos)
      , version_(-1)
   {
      world_view_ = transform_4t(camera_matrix(pos, up, dir, right), ss_unscaled);
      world_view_gl_ = axes_swapper_modelview_cg_2_gl<S>()(*world_view_);
      orientation_ = world_view_->inverted().rotation().cpr();
   }

   template< typename S > 
   camera_t<S>::camera_t( camera_t<S> const & other )
      : pos_          (other.pos_)
      , orientation_  (other.orientation_)
      , world_view_   (other.world_view_)
      , world_view_gl_(other.world_view_gl_)
      , version_      (-1)
   {
   }

   template< typename S >
   camera_t<S> & camera_t<S>::operator=( camera_t<S> const & other )
   {
      pos_           = other.pos_;
      orientation_   = other.orientation_;
      world_view_    = other.world_view_;
      world_view_gl_ = other.world_view_gl_;

      ++version_;

      return *this;
   }

   template< typename S > template < typename _S >
   camera_t<S>::camera_t( camera_t<_S> const & other )
      : pos_        (other.pos_)
      , orientation_(other.orientation_)
      , version_    (-1)
   {
      world_view_    = other.world_view_;
      world_view_gl_ = other.world_view_gl_;
   }

   template< typename S > template< typename _S >
   camera_t<S> & camera_t<S>::operator=( camera_t<_S> const & other )
   {
      pos_           = other.pos_;
      orientation_   = other.orientation_;
      world_view_    = other.world_view_;
      world_view_gl_ = other.world_view_gl_;

      ++version_;

      return *this;
   }

   template< typename S >
   void camera_t<S>::set_orientation( cpr_t const & orientation )
   {
      orientation_ = orientation;

      reset_transform();
      ++version_;
   }

   template< typename S >
   void camera_t<S>::set_position( point_3t const & position )
   {
      pos_ = position;

      reset_transform();
      ++version_;
   }

   template< typename S >
   typename camera_t<S>::point_3t const & camera_t<S>::position() const
   {
      return pos_;
   }

   template< typename S >
   typename camera_t<S>::cpr_t const & camera_t<S>::orientation() const
   {
      return orientation_;
   }

   template< typename S >
   typename camera_t<S>::point_3t camera_t<S>::dir() const
   {
      return world_view_transform().direct_matrix().get_row(1);
   }

   template< typename S >
   typename camera_t<S>::point_3t camera_t<S>::up() const
   {
      return world_view_transform().direct_matrix().get_row(2);
   }

   template< typename S >
   typename camera_t<S>::point_3t camera_t<S>::right() const
   {
      return world_view_transform().direct_matrix().get_row(0);
   }

   template< typename S >
   typename camera_t<S>::transform_4t const & camera_t<S>::world_view_transform() const
   {
      if (!world_view_)
      {
         world_view_ = transform_4t(cg::as_translation(pos_), rotation_3t(orientation_));
         world_view_->invert();
      }

      return *world_view_;
   }

   template< typename S >
   typename camera_t<S>::transform_4t const & camera_t<S>::world_view_gl_transform() const
   {
      if (!world_view_gl_)
      {
         world_view_gl_ = axes_swapper_modelview_cg_2_gl<S>()(world_view_transform());
      }

      return *world_view_gl_;
   }

   template< typename S >
   size_t camera_t<S>::version() const
   {
      return version_;
   }

   template< typename S >
   void camera_t<S>::reset_transform()
   {
      world_view_    = boost::none;
      world_view_gl_ = boost::none;
   }
}

#pragma pack(pop)
