#pragma once

#include "geometry\xmath.h"
#include "common\assert.h"

#include "turn.h"   
#include "rotation.h"   

#include "quaternion_fwd.h"   

namespace cg
{
   template < typename scalar > struct quaternion_t;

   // operations
   template<typename S> quaternion_t<S> operator * ( quaternion_t<S> q0, quaternion_t<S> const& q1 ) ; 

   template<typename S> quaternion_t<S> get_rotate_quaternion  ( quaternion_t<S> const& q0, quaternion_t<S> const& q1 );
   template<typename S> quaternion_t<S> slerp_quaternions      ( quaternion_t<S> const& q0, quaternion_t<S> const& q1, double t ); // 0 <= t <= 1
   template<typename S> quaternion_t<S> blend                  ( quaternion_t<S> const& q0, quaternion_t<S> const& q1, double t ); // 0 <= t <= 1
}


namespace cg
{
#pragma pack(push,1)
   template < typename scalar >
      struct quaternion_t
   {
      typedef point_t<scalar, 3>          point_t;
      typedef cpr_t<scalar>               cpr_t;
      typedef rot_axis_t<scalar>          rot_axis_t;
      typedef rotation_t<scalar, 3>       rotation_t;

      //
      quaternion_t () ; 
      quaternion_t ( rot_axis_t const& axis ) ; 
      quaternion_t ( cpr_t const& orient ) ; 

      template < typename _scalar >
         quaternion_t ( quaternion_t<_scalar> const& rhs ) ; 


      //
      cpr_t       cpr      () const ; 
      rot_axis_t  rot_axis () const ; 
      rotation_t  rotation () const ; 

      //
      quaternion_t& operator*= ( quaternion_t const& q ) ;  // NOTE: Произведение кватернионов некоммутативно
      quaternion_t  operator!  () const ;                   // inverted

      //
      point_t rotate_vector ( const point_t & p ) const ; 

      //
   private :
      quaternion_t ( scalar w, point_t const& v ) : w(w), v(v) {}

      quaternion_t& operator*= ( scalar d ) { w *= d; v *= d; return *this; }
      quaternion_t& operator/= ( scalar d ) { w /= d; v /= d; return *this; }
      quaternion_t  operator*  ( scalar d ) const { return quaternion_t(*this) *= d; }
      quaternion_t  operator/  ( scalar d ) const { return quaternion_t(*this) /= d; }
      quaternion_t  operator-  ()           const { return quaternion_t(-w, -v); }

      quaternion_t& operator+= ( const quaternion_t & q ) { w += q.w; v += q.v; return *this; }
      quaternion_t  operator+  ( quaternion_t q )   const { return q += *this; }

      scalar norm_sqr () const { return w*w + cg::norm_sqr(v); }
      scalar norm     () const { return cg::sqrt(norm_sqr()); }
      
      void normalize() { *this /= norm(); }

      scalar dot_product( quaternion_t const& q ) const { return w * q.w + v * q.v; }
      
      scalar get_course () const ;
      scalar get_pitch  () const ;
      scalar get_roll   () const ;

      friend struct rotation_t ; 
      template<typename> friend struct quaternion_t ; 
      template<typename> friend quaternion_t slerp_quaternions ( quaternion_t const& q0, quaternion_t const& q1, double t );

   private :
      scalar  w ;
      point_t v ;
   };
#pragma pack(pop)
}


//////////////////////////////////////////////////////////////////////////
// implementation
namespace cg
{
   template<typename S>
      quaternion_t<S>::quaternion_t ()
         : w(1), v(0, 0, 0)
   {}

   template<typename S>
      quaternion_t<S>::quaternion_t ( rot_axis_t const& axis )
         : w ( cos(grad2rad(axis.angle / 2)) )
         , v ( axis.axis * sin(grad2rad(axis.angle / 2)) )
   {
      Assert(cg::eq(cg::norm(axis.axis), (S)1, (S)1e-4));
      normalize();
   }

   template<typename S>
      quaternion_t<S>::quaternion_t ( cpr_t const& orient )
   {
      S h = -grad2rad(orient.course/ 2);
      S p =  grad2rad(orient.pitch / 2);
      S r =  grad2rad(orient.roll  / 2);

      // TODO: multiply manually
      quaternion_t Qz(cos(h), point_t(0, 0, sin(h)));
      quaternion_t Qx(cos(p), point_t(sin(p), 0, 0));
      quaternion_t Qy(cos(r), point_t(0, sin(r), 0));

      *this = Qz * Qx * Qy;

      //
      normalize();
   }

   template<typename S> template<typename S_>
      quaternion_t<S>::quaternion_t ( quaternion_t<S_> const& rhs )
         : w ( static_cast<S>      (rhs.w) )    
         , v ( static_cast<point_t>(rhs.v) )
   {
   }

   //
   template<typename S>
      typename quaternion_t<S>::cpr_t quaternion_t<S>::cpr () const
   {
      return cpr_t(get_course(), get_pitch(), get_roll());
   }

   template<typename S>
      typename quaternion_t<S>::rot_axis_t quaternion_t<S>::rot_axis() const
   {
      S vl = cg::norm(v);

      if ( eq_zero(vl) )
         return rot_axis_t(point_3(1, 0, 0), 0);

      S angle = 2 * acos(bound(w, -1., 1.));
      point_t axis = point_t(v.x, v.y, v.z) / vl;
      
      if ( angle > pi )
      {
         angle = 2 * pi - angle;
         axis = -axis;
      }

      return rot_axis_t(axis, rad2grad(angle));
   }

   template<typename S>
      typename quaternion_t<S>::rotation_t quaternion_t<S>::rotation () const
   {
      S x2 = v.x + v.x;
      S y2 = v.y + v.y;
      S z2 = v.z + v.z;
      S xx = v.x * x2;
      S xy = v.x * y2;
      S xz = v.x * z2;
      S yy = v.y * y2;
      S yz = v.y * z2;
      S zz = v.z * z2;
      S wx = w   * x2;
      S wy = w   * y2;
      S wz = w   * z2;

      return rotation_t(
         point_t(1-(yy+zz), xy+wz, xz-wy),
         point_t(xy-wz, 1-(xx+zz), yz+wx),
         point_t(xz+wy, yz-wx, 1-(xx+yy))
            );
   }

   template<typename S>
      quaternion_t<S>& quaternion_t<S>::operator*= ( const quaternion_t<S> & q )
   {
      S A = (w   + v.x) * (q.w   + q.v.x);
      S B = (v.z - v.y) * (q.v.y - q.v.z);
      S C = (w   - v.x) * (q.v.y + q.v.z);
      S D = (v.y + v.z) * (q.w   - q.v.x);
      S E = (v.x + v.z) * (q.v.x + q.v.y);
      S F = (v.x - v.z) * (q.v.x - q.v.y);
      S G = (w   + v.y) * (q.w   - q.v.z);
      S H = (w   - v.y) * (q.w   + q.v.z);

      w   = B + (-E - F + G + H) / 2;
      v.x = A - ( E + F + G + H) / 2;
      v.y = C + ( E - F + G - H) / 2;
      v.z = D + ( E - F - G + H) / 2;

      return *this;
   }

   template<typename S>
      quaternion_t<S> quaternion_t<S>::operator! () const
   {
      return quaternion_t(w, -v); // conjugated(сопряженный) quaternion for unary quaternion is the same as inverted
   }

   template<typename S>
      typename quaternion_t<S>::point_t quaternion_t<S>::rotate_vector ( const point_t & p ) const
   {
      return (*this * quaternion_t(0, p) * !*this).v;
   }

   //
   template<typename S>
      S quaternion_t<S>::get_course () const
   {
      S t = v.y * v.z + w * v.x;
      if ( eq(cg::abs(t), S(0.5)) ) // trouble with north-pole and south-pole (pitch = +/-90) 
         return 0;

      S dx = 1 - 2 * v.x * v.x - 2 * v.z * v.z;
      S dy = -2 * (v.x * v.y - w * v.z);

      return rad2grad(-atan2(dy, dx));
   }

   template<typename S>
      S quaternion_t<S>::get_pitch () const
   {
      S ang = bound<S>(2 * (v.y * v.z + w * v.x), -1, 1);
      return rad2grad(asin(ang));
   }

   template<typename S>
      S quaternion_t<S>::get_roll () const
   {
      S t = v.y * v.z + w * v.x;
      if ( eq(cg::abs(t), S(0.5)) ) // trouble with north-pole and south-pole (pitch = +/-90) 
         return rad2grad(sign(t) * 2 * atan2(v.z, w));

      S dx = 1 - 2 * v.y * v.y - 2 * v.x * v.x ;
      S dy = -2 * ( v.x * v.z - w * v.y )      ;

      return rad2grad(atan2(dy, dx));
   }

   //
   template<typename S>
      quaternion_t<S> operator* ( quaternion_t<S> q0, quaternion_t<S> const& q1 )
   {
      return q0 *= q1;
   }

   // spherical lerp by acute arc
   template<typename S>
      quaternion_t<S> slerp_quaternions ( quaternion_t<S> const& q0, quaternion_t<S> const& q1, double t )
   {
      S dp = q0.dot_product(q1) ;

      // choosing correct sign of q1 (to lerp by acute arc)
      quaternion_t<S> signedQ1 = !cg::ge(dp, 0) ? (dp = -dp, -q1) : q1 ;

      S thetha = acos(bound<S>(q0.dot_product(signedQ1), -1, 1));
      S Sin = sin(thetha);
      S a = (S)(1 - t);
      S b = (S)t;
      if ( !eq_zero( Sin ))
      {
         a = sin(a * thetha) / Sin;
         b = sin(b * thetha) / Sin;
      }
      return q0 * a + signedQ1 * b ;   
   }

   template<typename S>
      quaternion_t<S> get_rotate_quaternion ( quaternion_t<S> const& q0, quaternion_t<S> const& q1 )
   {
      return (!q0) * q1;
   }

   template<typename S>
      quaternion_t<S> blend ( quaternion_t<S> const& q0, quaternion_t<S> const& q1, double t )
   {
      return slerp_quaternions(q0, q1, 1 - t);
   }

}
