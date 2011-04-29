#pragma once

#include "point_fwd.h"
#include "common\assert.h"
#include <common\no_deduce.h>
#include <geometry\xmath.h>

namespace cg
{
   template < class Scalar, size_t Dim >     struct point_t;
   template < class Scalar, size_t Dim >     struct point_decomposition_t;

#define MAX_POINT(scalar_a, scalar_b, dim)   point_t< MAX_TYPE(scalar_a, scalar_b), dim >

   // --------------------------------------------------------------------------------------------------------- addition
   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator + ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator - ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   // --------------------------------------------------------------------------------------------------------- component-wise multiplication
   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator & ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator / ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator % ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   // --------------------------------------------------------------------------------------------------------- constant multiplication
   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator * ( point_t< ScalarT, Dim > const & a, ScalarU alpha );

   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator * ( ScalarT alpha, point_t< ScalarU, Dim > const & a );

   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator / ( point_t< ScalarT, Dim > const & a, ScalarU alpha );

   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_POINT(ScalarT, ScalarU, Dim) operator % ( point_t< ScalarT, Dim > const & a, ScalarU alpha );

   // --------------------------------------------------------------------------------------------------------- scalar product
   template < class ScalarT, class ScalarU, size_t Dim >     
      MAX_TYPE(ScalarT, ScalarU)     operator * ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   // --------------------------------------------------------------------------------------------------------- vector product
   template < class ScalarT, class ScalarU >     
      MAX_TYPE(ScalarT, ScalarU)     operator ^ ( point_t< ScalarT, 2 > const & a, point_t< ScalarU, 2 > const & b );
   template < class ScalarT, class ScalarU >     
      MAX_POINT(ScalarT, ScalarU, 3) operator ^ ( point_t< ScalarT, 3 > const & a, point_t< ScalarU, 3 > const & b );

   // --------------------------------------------------------------------------------------------------------- unary operations
   template < class Scalar, size_t Dim >     point_t< Scalar, Dim > operator - ( point_t< Scalar, Dim > a );

   // --------------------------------------------------------------------------------------------------------- norm and distance operations
   // returns squared norm of point
   template < class Scalar, size_t Dim >     Scalar norm_sqr( point_t< Scalar, Dim > const & a );

   // returns norm of point
   template < class Scalar, size_t Dim >     Scalar norm( point_t< Scalar, Dim > const & a );

   // returns distance between two points
   template < class ScalarT, class ScalarU, size_t Dim >
      MAX_TYPE(ScalarT, ScalarU) distance_sqr( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   // returns distance between two points
   template < class ScalarT, class ScalarU, size_t Dim >
      MAX_TYPE(ScalarT, ScalarU) distance( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b );

   // returns angle in radians between two vectors ( 0..2pi in 2D case, 0..pi otherwise )
   template < class ScalarT, class ScalarU, size_t Dim >
      MAX_TYPE(ScalarT, ScalarU) angle ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b ) ; 

   template < class Scalar >    
      point_t< Scalar, 2 > normal ( point_t< Scalar, 2 > const & a );

   template < class Scalar, size_t Dim >    
      bool is_finite( point_t< Scalar, Dim > const & a );

   // --------------------------------------------------------------------------------------------------------- normalization operations
   // returns norm of point
   template < class Scalar, size_t Dim >     Scalar normalize( point_t< Scalar, Dim > & point );

   // returns normalized point
   template < class Scalar, size_t Dim >     point_t< Scalar, Dim > normalized( point_t< Scalar, Dim > point );

   // returns direction and length: direction * length == point
   //template < class Scalar, size_t Dim >     point_decomposition_t< Scalar, Dim > decompose( point_t< Scalar, Dim > const & point, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) );

   // --------------------------------------------------------------------------------------------------------- comparison
   // fuzzy
   //template < class Scalar, size_t Dim >     bool eq( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) );
   //template < class Scalar, size_t Dim >     bool eq_zero( point_t< Scalar, Dim > const & a, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) );

   //// strong
   //template < class Scalar, size_t Dim >     bool operator == ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b );
   //template < class Scalar, size_t Dim >     bool operator != ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b );

   //// ---------------------------------------------------------------------------------------------------------- 
   //template < class Scalar, size_t Dim >     point_t< int,    Dim > round( point_t< Scalar, Dim > const & );
   //template < class Scalar, size_t Dim >     point_t< Scalar, Dim > floor( point_t< Scalar, Dim > const & );
   //template < class Scalar, size_t Dim >     point_t< Scalar, Dim > ceil ( point_t< Scalar, Dim > const & );

   //// ---------------------------------------------------------------------------------------------------------- comparison operators
   //template < class Scalar, size_t Dim >     bool operator < ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b ) ; 
   //template < class Scalar, size_t Dim >     bool operator > ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b ) ; 
}

namespace cg
{
#pragma pack( push, 1 )

   // ----------------------------------------------------------------------------------------------------- point_2_t
   template < class Scalar >
      struct point_t< Scalar, 2 >
   {
      typedef  Scalar   scalar_type;
      enum  { dimension = 2 };

      // -------------------------------------------------------------- ctor
      point_t( );

      point_t( Scalar x, Scalar y );

      template < class _Scalar >
      point_t( point_t< _Scalar, 2 > const & point );

      // -------------------------------------------------------------- arithmetic
      // vector operations
      point_t & operator += ( point_t const & );
      point_t & operator -= ( point_t const & );

      point_t & operator *= ( Scalar );
      point_t & operator /= ( Scalar );
      point_t & operator %= ( Scalar );

      // component-wise multiplication
      point_t & operator &= ( point_t const & );

      // component-wise division
      point_t & operator /= ( point_t const & );

      // component-wise remainder
      point_t & operator %= ( point_t const & );

      // -------------------------------------------------------------- data
      Scalar   x;
      Scalar   y;

      // -------------------------------------------------------------- data accessors
      Scalar         & operator [] ( size_t i )       { Assert( i < 2 ); return (&x)[i]; }
      Scalar const   & operator [] ( size_t i ) const { Assert( i < 2 ); return (&x)[i]; }

      // -------------------------------------------------------------- access for derived
      point_t const& as_2d () const { return *this ; } 
      point_t &      as_2d ()       { return *this ; } 
   };

   // ----------------------------------------------------------------------------------------------------- point_3_t
   template < class Scalar >
      struct point_t< Scalar, 3 >
         : point_t< Scalar, 2 >
   {
      typedef  Scalar   scalar_type;
      enum  { dimension = 3 };

      // -------------------------------------------------------------- ctor
      point_t( );

      point_t( Scalar x, Scalar y, Scalar z );

      point_t( point_t< Scalar, 2 > const & point, Scalar h );

      explicit point_t( point_t< Scalar, 2 > const & point );

      template < class _Scalar >
      point_t( point_t< _Scalar, 3 > const & point );

      // -------------------------------------------------------------- arithmetic
      // vector operations
      point_t & operator += ( point_t const & );
      point_t & operator -= ( point_t const & );

      point_t & operator *= ( Scalar );
      point_t & operator /= ( Scalar );
      point_t & operator %= ( Scalar );

      // component-wise multiplication
      point_t & operator &= ( point_t const & );

      // component-wise division
      point_t & operator /= ( point_t const & );

      // component-wise remainder
      point_t & operator %= ( point_t const & );

      // -------------------------------------------------------------- data
      Scalar   z;

      // -------------------------------------------------------------- data accessors
      Scalar         & operator [] ( size_t i )       { Assert( i < 3 ); return (&x)[i]; }
      Scalar const   & operator [] ( size_t i ) const { Assert( i < 3 ); return (&x)[i]; }

      // -------------------------------------------------------------- access for derived
      point_t const& as_3d () const { return *this ; } 
      point_t &      as_3d ()       { return *this ; } 
   };


   // ----------------------------------------------------------------------------------------------------- point_4_t
   template < class Scalar >
      struct point_t< Scalar, 4 >
         : point_t< Scalar, 3 >
   {
      typedef  Scalar   scalar_type;
      enum  { dimension = 4 };

      // -------------------------------------------------------------- ctor
      point_t( );

      point_t( Scalar x, Scalar y, Scalar z, Scalar w );

      point_t( point_t< Scalar, 3 > const & point, Scalar w );

      explicit point_t( point_t< Scalar, 3 > const & point );

      template < class _Scalar >
      point_t( point_t< _Scalar, 4 > const & point );

      // -------------------------------------------------------------- arithmetic
      // vector operations
      point_t & operator += ( point_t const & );
      point_t & operator -= ( point_t const & );

      point_t & operator *= ( Scalar );
      point_t & operator /= ( Scalar );
      point_t & operator %= ( Scalar );

      // component-wise multiplication
      point_t & operator &= ( point_t const & );

      // component-wise division
      point_t & operator /= ( point_t const & );

      // component-wise remainder
      point_t & operator %= ( point_t const & );

      // -------------------------------------------------------------- data
      Scalar   w;

      // -------------------------------------------------------------- data accessors
      Scalar         & operator [] ( size_t i )       { Assert( i < 4 ); return (&x)[i]; }
      Scalar const   & operator [] ( size_t i ) const { Assert( i < 4 ); return (&x)[i]; }

      // -------------------------------------------------------------- access for derived
      point_t const& as_4d () const { return *this ; } 
      point_t &      as_4d ()       { return *this ; } 
   };

   template < class Scalar, size_t Dim >
      struct point_decomposition_t
   {
      // -------------------------------------------------------------- ctor
      point_decomposition_t( );
      point_decomposition_t( point_t< Scalar, Dim > const & direction, Scalar length );

      // -------------------------------------------------------------- data
      point_t< Scalar, Dim >     direction;
      Scalar                     length;
   };

#pragma pack( pop )
}

// implementation
namespace cg 
{
   // ----------------------------------------------------------------------------------------------------- point_2_t
   // -------------------------------------------------------------- ctor
   template < class Scalar >
   __forceinline  point_t< Scalar, 2 > :: point_t( )
      : x( 0 )
      , y( 0 )
   {}

   template < class Scalar >
   __forceinline  point_t< Scalar, 2 > :: point_t( Scalar x, Scalar y )
      : x( x )
      , y( y )
   {}

   template < class Scalar >
   template < class _Scalar >
   __forceinline  point_t< Scalar, 2 > :: point_t( point_t< _Scalar, 2 > const & point )
      : x( ( Scalar ) point.x )
      , y( ( Scalar ) point.y )
   {}

   // ----------------------------------------------------------------------------------------------------- point_3_t
   // -------------------------------------------------------------- ctor
   template < class Scalar >
   __forceinline point_t< Scalar, 3 > :: point_t( )
      : z( 0 )
   {}

   template < class Scalar >
   __forceinline point_t< Scalar, 3 > :: point_t( Scalar x, Scalar y, Scalar z )
      : point_t< Scalar, 2 >( x, y )
      , z( z )
   {}

   template < class Scalar >
   __forceinline point_t< Scalar, 3 > :: point_t( point_t< Scalar, 2 > const & point, Scalar h )
      : point_t< Scalar, 2 >( point )
      , z( h )
   {}

   template < class Scalar >
   __forceinline point_t< Scalar, 3 > :: point_t( point_t< Scalar, 2 > const & point )
      : point_t< Scalar, 2 >( point )
      , z( 0 )
   {}

   template < class Scalar >
   template < class _Scalar >
   __forceinline point_t< Scalar, 3 > :: point_t( point_t< _Scalar, 3 > const & point )
      : point_t< Scalar, 2 >( point )
      , z( ( Scalar ) point.z )
   {}

   // ----------------------------------------------------------------------------------------------------- point_4_t
   // -------------------------------------------------------------- ctor
   template < class Scalar >
      __forceinline point_t< Scalar, 4 > :: point_t( )
      : w( 0 )
   {}

   template < class Scalar >
      __forceinline point_t< Scalar, 4 > :: point_t( Scalar x, Scalar y, Scalar z, Scalar w )
      : point_t< Scalar, 3 >( x, y, z )
      , w( w )
   {}

   template < class Scalar >
      __forceinline point_t< Scalar, 4 > :: point_t( point_t< Scalar, 3 > const & point, Scalar w )
      : point_t< Scalar, 3 >( point )
      , w( w )
   {}

   template < class Scalar >
      __forceinline point_t< Scalar, 4 > :: point_t( point_t< Scalar, 3 > const & point )
      : point_t< Scalar, 3 >( point )
      , w( 0 )
   {}

   template < class Scalar >
      template < class _Scalar >
      __forceinline point_t< Scalar, 4 > :: point_t( point_t< _Scalar, 4 > const & point )
      : point_t< Scalar, 3 >( point )
      , w( ( Scalar ) point.w )
   {}


   // ------------------------------------------------------------------------------------------------- point_decomposition_t
   template < class Scalar, size_t Dim >
   __forceinline point_decomposition_t< Scalar, Dim > :: point_decomposition_t( )
      : length( 0 )
   {}

   template < class Scalar, size_t Dim >
   __forceinline point_decomposition_t< Scalar, Dim > :: point_decomposition_t( point_t< Scalar, Dim > const & direction, Scalar length )
      : direction ( direction )
      , length    ( length    )
   {}

   // --------------------------------------------------------------------------------------------------------- addition
   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT( ScalarT, ScalarU, Dim ) operator + ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return MAX_POINT( ScalarT, ScalarU, Dim )( a ) += b;
   }

   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT( ScalarT, ScalarU, Dim ) operator - ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return MAX_POINT( ScalarT, ScalarU, Dim )( a ) -= b;
   }

   // --------------------------------------------------------------------------------------------------------- component-wise multiplication
   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT( ScalarT, ScalarU, Dim ) operator & ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return MAX_POINT( ScalarT, ScalarU, Dim )( a ) &= b;
   }

   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT(ScalarT, ScalarU, Dim) operator % ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return MAX_POINT(ScalarT, ScalarU, Dim) (a) %= b ; 
   }

   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT( ScalarT, ScalarU, Dim ) operator / ( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return MAX_POINT( ScalarT, ScalarU, Dim )( a ) /= b;
   }

   // --------------------------------------------------------------------------------------------------------- constant multiplication
   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT(ScalarT, ScalarU, Dim) operator * ( point_t< ScalarT, Dim > const & a, ScalarU alpha )
   {
      return MAX_POINT(ScalarT, ScalarU, Dim)( a ) *= static_cast< MAX_TYPE( ScalarT, ScalarU ) >( alpha );
   }

   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT(ScalarT, ScalarU, Dim) operator * ( ScalarT alpha, point_t< ScalarU, Dim > const & a )
   {
      return MAX_POINT(ScalarT, ScalarU, Dim)( a ) *= static_cast< MAX_TYPE( ScalarT, ScalarU ) >( alpha );
   }

   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT(ScalarT, ScalarU, Dim) operator / ( point_t< ScalarT, Dim > const & a, ScalarU alpha )
   {
      return MAX_POINT(ScalarT, ScalarU, Dim)( a ) /= static_cast< MAX_TYPE( ScalarT, ScalarU ) >( alpha );
   }

   template < class ScalarT, class ScalarU, size_t Dim >     
      __forceinline MAX_POINT(ScalarT, ScalarU, Dim) operator % ( point_t< ScalarT, Dim > const & a, ScalarU alpha )
   {
      return MAX_POINT(ScalarT, ScalarU, Dim)( a ) %= static_cast< MAX_TYPE( ScalarT, ScalarU ) >( alpha );
   }

   // --------------------------------------------------------------------------------------------------------- scalar product
   template < class ScalarT, class ScalarU >     
      __forceinline MAX_TYPE(ScalarT, ScalarU)     operator * ( point_t< ScalarT, 2 > const & a, point_t< ScalarU, 2 > const & b )
   {
      MAX_TYPE(ScalarT, ScalarU) res = 0;
      res += a.x * b.x;
      res += a.y * b.y;

      return res;
   }

   template < class ScalarT, class ScalarU >     
      __forceinline MAX_TYPE(ScalarT, ScalarU)     operator * ( point_t< ScalarT, 3 > const & a, point_t< ScalarU, 3 > const & b )
   {
      MAX_TYPE(ScalarT, ScalarU) res = 0;
      res += a.x * b.x;
      res += a.y * b.y;
      res += a.z * b.z;

      return res;
   }

   template < class ScalarT, class ScalarU >     
      __forceinline MAX_TYPE(ScalarT, ScalarU)     operator * ( point_t< ScalarT, 4 > const & a, point_t< ScalarU, 4 > const & b )
   {
      MAX_TYPE(ScalarT, ScalarU) res = 0;
      res += a.x * b.x;
      res += a.y * b.y;
      res += a.z * b.z;
      res += a.w * b.w;

      return res;
   }

   // --------------------------------------------------------------------------------------------------------- vector product
   template < class ScalarT, class ScalarU >     
      __forceinline MAX_TYPE(ScalarT, ScalarU)     operator ^ ( point_t< ScalarT, 2 > const & a, point_t< ScalarU, 2 > const & b )
   {
      return a.x * b.y - a.y * b.x;  
   }

   template < class ScalarT, class ScalarU >     
      __forceinline MAX_POINT(ScalarT, ScalarU, 3) operator ^ ( point_t< ScalarT, 3 > const & a, point_t< ScalarU, 3 > const & b )
   {
      return MAX_POINT(ScalarT, ScalarU, 3) ( a.y * b.z - a.z * b.y, 
                                              a.z * b.x - a.x * b.z, 
                                              a.x * b.y - a.y * b.x );
   }

   // --------------------------------------------------------------------------------------------------------- unary operations
   template < class Scalar >
      __forceinline point_t< Scalar, 2 > operator - ( point_t< Scalar, 2 > a )
   {
      a.x = -a.x;
      a.y = -a.y;

      return a;
   }

   template < class Scalar >
      __forceinline point_t< Scalar, 3 > operator - ( point_t< Scalar, 3 > a )
   {
      a.x = -a.x;
      a.y = -a.y;
      a.z = -a.z;

      return a;
   }

   template < class Scalar >
      __forceinline point_t< Scalar, 4 > operator - ( point_t< Scalar, 4 > a )
   {
      a.x = -a.x;
      a.y = -a.y;
      a.z = -a.z;
      a.w = -a.w;

      return a;
   }

   // --------------------------------------------------------------------------------------------------------- norm and distance operations
   // returns squared norm of point
   template < class Scalar, size_t Dim >
      __forceinline Scalar norm_sqr( point_t< Scalar, Dim > const & a )
   {
      return a * a;
   }

   // returns norm of point
   template < class Scalar, size_t Dim >
      __forceinline Scalar norm( point_t< Scalar, Dim > const & a )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );
      return cg::sqrt( a * a );
   }

   // returns distance between two points
   template < class ScalarT, class ScalarU, size_t Dim >
      __forceinline MAX_TYPE(ScalarT, ScalarU) distance_sqr( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return norm_sqr( b - a );
   }

   // returns distance between two points
   template < class ScalarT, class ScalarU, size_t Dim >
      __forceinline MAX_TYPE(ScalarT, ScalarU) distance( point_t< ScalarT, Dim > const & a, point_t< ScalarU, Dim > const & b )
   {
      return norm( b - a );
   }

   template < class ScalarT, class ScalarU>
      __forceinline MAX_TYPE(ScalarT, ScalarU) angle ( point_t< ScalarT, 2 > const & a, point_t< ScalarU, 2 > const & b ) 
   {
      typedef MAX_TYPE(ScalarT, ScalarU) S ; 

      S ab = norm_sqr(a) * norm_sqr(b);  
      
      S ret = !eq_zero(ab) ? acos( bound<S>( (a * b) / cg::sqrt(ab), -1, 1 ) ) : 0;

      return (a ^ b) < 0 ? 2 * (S)pi - ret : ret;
   }

   template < class ScalarT, class ScalarU, size_t D>
      __forceinline MAX_TYPE(ScalarT, ScalarU) angle ( point_t< ScalarT, D > const & a, point_t< ScalarU, D > const & b ) 
   {
      typedef MAX_TYPE(ScalarT, ScalarU) S ; 

      S ab = norm_sqr(a) * norm_sqr(b);  

      return !eq_zero(ab) ? acos( bound<S>( (a * b) / cg::sqrt(ab), -1, 1 ) ) : 0;
   }

   template < class Scalar >    
      __forceinline point_t< Scalar, 2 > normal ( point_t< Scalar, 2 > const & a )
   {
      return point_t< Scalar, 2 > ( - a.y, a.x ) ; 
   }

   template < class Scalar, size_t Dim >    
      __forceinline bool is_finite( point_t< Scalar, Dim > const & a )
   {
      for ( size_t i = 0; i < Dim; ++i )
         if ( !_finite( a[i] ) ) // TODO: Implement cg::is_finite(scalar) and use it here.
            return false;
      return true;
   }

   // --------------------------------------------------------------------------------------------------------- normalization operations
   // returns norm of point
   template < class Scalar, size_t Dim >
      __forceinline Scalar normalize( point_t< Scalar, Dim > & point )
   {
      Scalar point_norm = norm( point );
      point /= point_norm;

      return point_norm;
   }

   // returns normalized point
   template < class Scalar, size_t Dim >
      __forceinline point_t< Scalar, Dim > normalized( point_t< Scalar, Dim > point )
   {
      normalize( point );
      return point;
   }

   // returns normalized point
   template < class Scalar, size_t Dim >
      __forceinline point_t< Scalar, Dim > normalized_safe( point_t< Scalar, Dim > const & point )
   {
      point_decomposition_t<Scalar, Dim> dec = decompose( point );

      if ( dec.length != 0 )
         return dec.direction;

      point_t< Scalar, Dim > res;
      res[Dim - 1] = 1.;

      return res;
   }

   // returns direction and length: direction * length == point
   // if eq_zero( length, eps ) returns point( 0, 0, ... , 0 ), length == 0
   template < class Scalar, size_t Dim >
      __forceinline point_decomposition_t< Scalar, Dim > decompose( point_t< Scalar, Dim > const & point, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      Scalar point_norm = norm( point );
      if ( eq_zero( point_norm, eps ) )
         return point_decomposition_t< Scalar, Dim >( );

      return point_decomposition_t< Scalar, Dim >( point / point_norm, point_norm );
   }

   // --------------------------------------------------------------------------------------------------------- comparison
   // fuzzy
   template < class Scalar >
      __forceinline bool eq( point_t< Scalar, 2 > const & a, point_t< Scalar, 2 > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq( a.x, b.x, eps ) )
         return false;

      if ( !eq( a.y, b.y, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq( point_t< Scalar, 3 > const & a, point_t< Scalar, 3 > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq( a.x, b.x, eps ) )
         return false;

      if ( !eq( a.y, b.y, eps ) )
         return false;

      if ( !eq( a.z, b.z, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq( point_t< Scalar, 4 > const & a, point_t< Scalar, 4 > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq( a.x, b.x, eps ) )
         return false;

      if ( !eq( a.y, b.y, eps ) )
         return false;

      if ( !eq( a.z, b.z, eps ) )
         return false;

      if ( !eq( a.w, b.w, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq_rel( point_t< Scalar, 2 > const & a, point_t< Scalar, 2 > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq_rel( a.x, b.x, eps ) )
         return false;

      if ( !eq_rel( a.y, b.y, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq_rel( point_t< Scalar, 3 > const & a, point_t< Scalar, 3 > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq_rel( a.x, b.x, eps ) )
         return false;

      if ( !eq_rel( a.y, b.y, eps ) )
         return false;

      if ( !eq_rel( a.z, b.z, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq_rel( point_t< Scalar, 4 > const & a, point_t< Scalar, 4 > const & b, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq_rel( a.x, b.x, eps ) )
         return false;

      if ( !eq_rel( a.y, b.y, eps ) )
         return false;

      if ( !eq_rel( a.z, b.z, eps ) )
         return false;

      if ( !eq_rel( a.w, b.w, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq_zero( point_t< Scalar, 2 > const & a, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq_zero( a.x, eps ) )
         return false;

      if ( !eq_zero( a.y, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq_zero( point_t< Scalar, 3 > const & a, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq_zero( a.x, eps ) )
         return false;

      if ( !eq_zero( a.y, eps ) )
         return false;

      if ( !eq_zero( a.z, eps ) )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool eq_zero( point_t< Scalar, 4 > const & a, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      if ( !eq_zero( a.x, eps ) )
         return false;

      if ( !eq_zero( a.y, eps ) )
         return false;

      if ( !eq_zero( a.z, eps ) )
         return false;

      if ( !eq_zero( a.w, eps ) )
         return false;

      return true;
   }

   // strong
   template < class Scalar >
      __forceinline bool operator == ( point_t< Scalar, 2 > const & a, point_t< Scalar, 2 > const & b )
   {
      if ( a.x != b.x )
         return false;

      if ( a.y != b.y )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool operator == ( point_t< Scalar, 3 > const & a, point_t< Scalar, 3 > const & b )
   {
      if ( a.x != b.x )
         return false;

      if ( a.y != b.y )
         return false;

      if ( a.z != b.z )
         return false;

      return true;
   }

   template < class Scalar >
      __forceinline bool operator == ( point_t< Scalar, 4 > const & a, point_t< Scalar, 4 > const & b )
   {
      if ( a.x != b.x )
         return false;

      if ( a.y != b.y )
         return false;

      if ( a.z != b.z )
         return false;

      if ( a.w != b.w )
         return false;

      return true;
   }

   template < class Scalar, size_t Dim >
      __forceinline bool operator != ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b )
   {
      return !( a == b );
   }

   // ----------------------------------------------------------------------------------------------------- 
   template < class Scalar >     
      __forceinline point_t< int, 2 > round( point_t< Scalar, 2 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 2 > ( round( point.x ),
                                 round( point.y ) );
   }

   template < class Scalar >     
      __forceinline point_t< int, 3 > round( point_t< Scalar, 3 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 3 > ( round( point.x ),
                                 round( point.y ),
                                 round( point.z ) );
   }

   template < class Scalar >     
      __forceinline point_t< int, 4 > round( point_t< Scalar, 4 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 4 > ( round( point.x ),
                                 round( point.y ),
                                 round( point.z ),
                                 round( point.w ) );
   }

   template < class Scalar >     
      __forceinline point_t< Scalar, 2 > floor( point_t< Scalar, 2 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 2 > ( floor( point.x ),
                                 floor( point.y ) );
   }

   template < class Scalar >     
      __forceinline point_t< Scalar, 3 > floor( point_t< Scalar, 3 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 3 > ( floor( point.x ),
                                 floor( point.y ),
                                 floor( point.z ) );
   }

   template < class Scalar >     
      __forceinline point_t< Scalar, 4 > floor( point_t< Scalar, 4 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 4 > ( floor( point.x ),
                                 floor( point.y ),
                                 floor( point.z ),
                                 floor( point.w ) );
   }

   template < class Scalar >     
      __forceinline point_t< Scalar, 2 > ceil ( point_t< Scalar, 2 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 2 > ( ceil( point.x ),
                                 ceil( point.y ) );
   }

   template < class Scalar >     
      __forceinline point_t< Scalar, 3 > ceil ( point_t< Scalar, 3 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 3 > ( ceil( point.x ),
                                 ceil( point.y ),
                                 ceil( point.z ) );
   }

   template < class Scalar >     
      __forceinline point_t< Scalar, 4 > ceil ( point_t< Scalar, 4 > const & point )
   {
      STATIC_ASSERT( is_floating_point_f<Scalar>::value, norm_for_integral_points_undefined );

      return point_t< int, 4 > ( ceil( point.x ),
                                 ceil( point.y ),
                                 ceil( point.z ),
                                 ceil( point.w ) );
   }

   // ---------------------------------------------------------------------------------------------------------- operator less
   template < class Scalar >
      __forceinline bool operator < ( point_t< Scalar, 2 > const & a, point_t< Scalar, 2 > const & b )
   {
      if ( a.x < b.x ) 
         return true ; 

      if ( a.x > b.x ) 
         return false ;

      if ( a.y < b.y ) 
         return true ; 

      if ( a.y > b.y ) 
         return false ;

      return false ;
   }

   template < class Scalar >
      __forceinline bool operator < ( point_t< Scalar, 3 > const & a, point_t< Scalar, 3 > const & b )
   {
      if ( a.x < b.x ) 
         return true ; 

      if ( a.x > b.x ) 
         return false ;

      if ( a.y < b.y ) 
         return true ; 

      if ( a.y > b.y ) 
         return false ;

      if ( a.z < b.z ) 
         return true ; 

      if ( a.z > b.z ) 
         return false ;

      return false ;
   }

   template < class Scalar >
      __forceinline bool operator < ( point_t< Scalar, 4 > const & a, point_t< Scalar, 4 > const & b )
   {
      if ( a.x < b.x ) 
         return true ; 

      if ( a.x > b.x ) 
         return false ;

      if ( a.y < b.y ) 
         return true ; 

      if ( a.y > b.y ) 
         return false ;

      if ( a.z < b.z ) 
         return true ; 

      if ( a.z > b.z ) 
         return false ;

      if ( a.w < b.w ) 
         return true ; 

      if ( a.w > b.w ) 
         return false ;

      return false ;
   }

   // ---------------------------------------------------------------------------------------------------------- operator greater
   template < class Scalar, size_t Dim >
      __forceinline bool operator > ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b )
   {
      return !( a < b || a == b );
   }

   template < class Scalar, size_t Dim >
      __forceinline bool operator <= ( point_t< Scalar, Dim > const & a, point_t< Scalar, Dim > const & b )
   {
      return a < b || a == b;
   }


   // ----------------------------------------------------------------------------------------------------- point_2_t

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator += ( point_t< Scalar, 2 > const & point )
   {
      x += point.x;
      y += point.y;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator -= ( point_t< Scalar, 2 > const & point )
   {
      x -= point.x;
      y -= point.y;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator *= ( Scalar alpha )
   {
      x *= alpha;
      y *= alpha;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator /= ( Scalar alpha )
   {
      x /= alpha;
      y /= alpha;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator %= ( Scalar alpha )
   {
      x = cg::mod(x,alpha);
      y = cg::mod(y,alpha);

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator &= ( point_t< Scalar, 2 > const & point )
   {
      x *= point.x;
      y *= point.y;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator /= ( point_t< Scalar, 2 > const & point )
   {
      x /= point.x;
      y /= point.y;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 2 > & point_t< Scalar, 2 > :: operator %= ( point_t< Scalar, 2 > const & point )
   {
      x = cg::mod(x, point.x);
      y = cg::mod(y, point.y);

      return *this;
   }

   // ----------------------------------------------------------------------------------------------------- point_3_t

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator += ( point_t< Scalar, 3 > const & point )
   {
      x += point.x;
      y += point.y;
      z += point.z;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator -= ( point_t< Scalar, 3 > const & point )
   {
      x -= point.x;
      y -= point.y;
      z -= point.z;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator *= ( Scalar alpha )
   {
      x *= alpha;
      y *= alpha;
      z *= alpha;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator /= ( Scalar alpha )
   {
      x /= alpha;
      y /= alpha;
      z /= alpha;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator %= ( Scalar alpha )
   {
      x = cg::mod(x,alpha);
      y = cg::mod(y,alpha);
      z = cg::mod(z,alpha);

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator &= ( point_t< Scalar, 3 > const & point )
   {
      x *= point.x;
      y *= point.y;
      z *= point.z;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator /= ( point_t< Scalar, 3 > const & point )
   {
      x /= point.x;
      y /= point.y;
      z /= point.z;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 3 > & point_t< Scalar, 3 > :: operator %= ( point_t< Scalar, 3 > const & point )
   {
      x = cg::mod(x, point.x);
      y = cg::mod(y, point.y);
      z = cg::mod(z, point.z);

      return *this;
   }

   // ----------------------------------------------------------------------------------------------------- point_4_t

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator += ( point_t< Scalar, 4 > const & point )
   {
      x += point.x;
      y += point.y;
      z += point.z;
      w += point.w;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator -= ( point_t< Scalar, 4 > const & point )
   {
      x -= point.x;
      y -= point.y;
      z -= point.z;
      w -= point.w;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator *= ( Scalar alpha )
   {
      x *= alpha;
      y *= alpha;
      z *= alpha;
      w *= alpha;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator /= ( Scalar alpha )
   {
      x /= alpha;
      y /= alpha;
      z /= alpha;
      w /= alpha;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator %= ( Scalar alpha )
   {
      x = cg::mod(x,alpha);
      y = cg::mod(y,alpha);
      z = cg::mod(z,alpha);
      w = cg::mod(w,alpha);

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator &= ( point_t< Scalar, 4 > const & point )
   {
      x *= point.x;
      y *= point.y;
      z *= point.z;
      w *= point.w;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator /= ( point_t< Scalar, 4 > const & point )
   {
      x /= point.x;
      y /= point.y;
      z /= point.z;
      w /= point.w;

      return *this;
   }

   template < class Scalar > 
      __forceinline point_t< Scalar, 4 > & point_t< Scalar, 4 > :: operator %= ( point_t< Scalar, 4 > const & point )
   {
      x = cg::mod(x, point.x);
      y = cg::mod(y, point.y);
      z = cg::mod(z, point.z);
      w = cg::mod(w, point.w);

      return *this;
   }


#undef MAX_POINT

}

