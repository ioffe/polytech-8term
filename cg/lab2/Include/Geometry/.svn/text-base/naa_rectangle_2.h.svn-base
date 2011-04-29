#pragma once

namespace cg
{
   struct naa_rectangle_2
   {
      naa_rectangle_2() {}
      naa_rectangle_2(point_2 const& org, point_2 const& axis0, point_2 const& axis1);
      naa_rectangle_2(rectangle_2 const& rc);
      naa_rectangle_2(point_2 const& origin, rectangle_2 const& rc, double course);

      naa_rectangle_2& operator= (rectangle_2 const& rc);

      bool contains( point_2 const& p ) const ;
      bool contains( naa_rectangle_2 const& other ) const ;
      bool empty() ;

      point_2 org;   // rectangle center
      point_2 axis0; // axes with half length
      point_2 axis1;
   };

   inline naa_rectangle_2::naa_rectangle_2(point_2 const& org, point_2 const& axis0, point_2 const& axis1)
      : org(org), axis0(axis0), axis1(axis1)
   {
   }

   inline naa_rectangle_2::naa_rectangle_2(rectangle_2 const& rc)
      : org((rc.XY() + rc.xy()) / 2.), axis0((rc.XY() - rc.Xy()) / 2.), axis1((rc.Xy() - rc.xy()) / 2.)
   {
   }

   inline naa_rectangle_2::naa_rectangle_2(point_2 const& origin, rectangle_2 const& rc, double course)
   {
      cg::rotation_2 rot(course) ;
      org = ((origin + rot * rc.Xy()) + (origin + rot * rc.xY())) * 0.5;
      axis0 = rot * (rc.XY() - rc.Xy()) * 0.5 ;
      axis1 = rot * (rc.Xy() - rc.xy()) * 0.5 ;
   }

   inline naa_rectangle_2& naa_rectangle_2::operator= (rectangle_2 const& rc)
   {
      org = (rc.XY() - rc.xy()) / 2., axis0 = (rc.XY() - rc.Xy()) / 2., axis1 = (rc.Xy() - rc.xy()) / 2.;
      return *this;
   }

   inline bool naa_rectangle_2::contains( point_2 const& p ) const
   {
      point_2 offset = p - org ;
      double axis0_len_sqr = cg::norm_sqr(axis0) ; 
      double axis1_len_sqr = cg::norm_sqr(axis1) ; 
      return cg::abs(offset * axis0) <= axis0_len_sqr && cg::abs(offset * axis1) <= axis1_len_sqr ;
   }

   inline bool naa_rectangle_2::contains( naa_rectangle_2 const& other ) const
   {
      // todo: make more effective
      return contains(other.org + other.axis0 + other.axis1) &&
             contains(other.org + other.axis0 - other.axis1) &&
             contains(other.org - other.axis0 + other.axis1) &&
             contains(other.org - other.axis0 - other.axis1) ;
   }

   inline bool naa_rectangle_2::empty()
   {
      return axis0 == point_2() || axis0 == point_2() ;
   }

   inline bool has_intersection(naa_rectangle_2 const& rc1, naa_rectangle_2 const& rc2, point_2 const& axis)
   {
      return (cg::abs(rc1.axis0 * axis) + cg::abs(rc1.axis1 * axis) + 
              cg::abs(rc2.axis0 * axis) + cg::abs(rc2.axis1 * axis)) >= 
              cg::abs((rc2.org - rc1.org) * axis);
   }

   inline bool has_intersection(naa_rectangle_2 const& rc1, naa_rectangle_2 const& rc2)
   {
      return (has_intersection(rc1, rc2, rc1.axis0) &&
              has_intersection(rc1, rc2, rc1.axis1) &&
              has_intersection(rc1, rc2, rc1.axis0 + rc1.axis1) &&
              has_intersection(rc1, rc2, rc2.axis0) &&
              has_intersection(rc1, rc2, rc2.axis1) &&
              has_intersection(rc1, rc2, rc2.axis0 + rc2.axis1));
   }
}