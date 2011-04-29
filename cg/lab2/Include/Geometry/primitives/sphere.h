#pragma once

#include "point_fwd.h"

namespace cg
{
   template < class S, size_t D >
   struct sphere_t
   {
      enum  { dimension = D };
      typedef S             scalar_type;
      typedef S             scalar_t;
      typedef point_t<S, D> point_type; 
      typedef point_t<S,D>  point_t; 

      sphere_t(point_t const &c, scalar_t r) ; 
      sphere_t() ; 

      template <class Scalar>
         sphere_t( sphere_t<Scalar, D> const& s );

      explicit sphere_t ( sphere_t<S, D+1> const& s ); 

      __forceinline bool contains( point_t const& p ) const ;

      point_t  center;
      scalar_t radius;
   };

   // --------------------------------------------------------------------
   template < class S, size_t D >
      sphere_t< S, D > :: sphere_t( point_t const &c, S r )
      : center( c )
      , radius( r )
   {}

   template < class S, size_t D >
      sphere_t< S, D > :: sphere_t()
      : radius( 0 )
   {}

   template < class S, size_t D > template < class Scalar >
      sphere_t< S, D > :: sphere_t( sphere_t< Scalar, D > const& s )
      : center( s.center )
      , radius( S (s.radius) )
   {}

   template < class S, size_t D >
      sphere_t< S, D >::sphere_t( sphere_t< S, D + 1 > const& s )
      : center( s.c )
      , radius( s.r )
   {}

   template < class S, size_t D >
      __forceinline bool sphere_t< S, D > :: contains( point_t const& p ) const
   {
      return sphere_details::contains(*this, p) ;
   }


   // -------------------------------------------------------------------- sphere_details
   namespace sphere_details
   {
      template < class S, size_t D >
         __forceinline bool contains( sphere_t<S, D> const& s, point_t<S, D> const& p )
      {
         return distance(s.center, p) <= s.radius ;
      }
   }

   template< class Stream, class S, size_t D >
   void write(Stream &stream, sphere_t<S,D> const& data)
   {
      write(stream, data.center) ;
      write(stream, data.radius) ;
   }

   template< class Stream, class S, size_t D >
   void read(Stream &stream, sphere_t<S,D> & data)
   {
      read(stream, data.center) ;
      read(stream, data.radius) ;
   }
}
