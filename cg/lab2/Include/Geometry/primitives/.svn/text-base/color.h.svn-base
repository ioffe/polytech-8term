#pragma once

#include "color_fwd.h"
#include "color_hsv.h"
#include "geometry\xmath.h"

#pragma pack(push, 1)

namespace cg
{
   template<typename S>
   struct color_t
   {
      typedef color_scalar_unit<S> unit;

      S r, g, b;

      color_t( S v = unit::get() ) ;
      color_t( S r, S g, S b ) ;

      template<typename _S> color_t( const color_t<_S> &c ) ; 
      template<typename _S> color_t( const color_hsv_t<_S> &c ) ; 

      color_t & operator += ( const color_t &c ) ;
      color_t & operator -= ( const color_t &c ) ;
      color_t & operator *= ( const color_t &c ) ;
      color_t & operator /= ( const color_t &c ) ;

      color_t & operator += ( S v ) ;
      color_t & operator -= ( S v ) ;
      color_t & operator *= ( S v ) ;
      color_t & operator /= ( S v ) ;
   };

   template <typename S> __forceinline color_t<S> operator + ( const color_t<S> &c1, const color_t<S> &c2 ) ; 
   template <typename S> __forceinline color_t<S> operator - ( const color_t<S> &c1, const color_t<S> &c2 ) ; 
   template <typename S> __forceinline color_t<S> operator * ( const color_t<S> &c1, const color_t<S> &c2 ) ; 
   template <typename S> __forceinline color_t<S> operator / ( const color_t<S> &c1, const color_t<S> &c2 ) ; 

   template <typename S> __forceinline color_t<S> operator + ( const color_t<S> &c, S v ) ; 
   template <typename S> __forceinline color_t<S> operator - ( const color_t<S> &c, S v ) ; 
   template <typename S> __forceinline color_t<S> operator * ( const color_t<S> &c, S v ) ; 
   template <typename S> __forceinline color_t<S> operator / ( const color_t<S> &c, S v ) ; 

   template <typename S> __forceinline color_t<S> operator + ( S v, const color_t<S> &c ) ; 
   template <typename S> __forceinline color_t<S> operator * ( S v, const color_t<S> &c ) ; 

   template <typename S> __forceinline bool eq     ( color_t<S> const & a, color_t<S> const & b, S eps = color_epsilon<S>() );
   template <typename S> __forceinline bool eq_zero( color_t<S> const & a, S eps = color_epsilon<S>() );

   template <typename S> __forceinline bool operator == ( const color_t<S> &c1, const color_t<S> &c2 ) ; 
   template <typename S> __forceinline bool operator != ( const color_t<S> &c1, const color_t<S> &c2 ) ; 

   template <typename S> __forceinline bool operator <  ( const color_t<S> &c1, const color_t<S> &c2 ) ; 

   template<typename S>
   struct colora_t : color_t<S>
   {
      S a;

      colora_t() ;
      colora_t( S r, S g, S b, S a = unit::get()) ;
      colora_t( const color_t<S>& c, S a = unit::get() ) ;

      template<typename _S> colora_t( const colora_t<_S> &c ) ; 

      colora_t & operator += ( const color_t<S> &c ) ;
      colora_t & operator -= ( const color_t<S> &c ) ;
      colora_t & operator *= ( const color_t<S> &c ) ;
      colora_t & operator /= ( const color_t<S> &c ) ;

      colora_t & operator += ( const colora_t<S> &c ) ;
      colora_t & operator -= ( const colora_t<S> &c ) ;
      colora_t & operator *= ( const colora_t<S> &c ) ;
      colora_t & operator /= ( const colora_t<S> &c ) ;

      colora_t & operator += ( S v ) ;
      colora_t & operator -= ( S v ) ;
      colora_t & operator *= ( S v ) ;
      colora_t & operator /= ( S v ) ;
   };

   template <typename S> __forceinline colora_t<S> operator + ( const colora_t<S> &c1, const color_t<S> &c2 ) ;
   template <typename S> __forceinline colora_t<S> operator - ( const colora_t<S> &c1, const color_t<S> &c2 ) ;
   template <typename S> __forceinline colora_t<S> operator * ( const colora_t<S> &c1, const color_t<S> &c2 ) ;
   template <typename S> __forceinline colora_t<S> operator / ( const colora_t<S> &c1, const color_t<S> &c2 ) ;

   template <typename S> __forceinline colora_t<S> operator + ( const colora_t<S> &c1, const colora_t<S> &c2 ) ;
   template <typename S> __forceinline colora_t<S> operator - ( const colora_t<S> &c1, const colora_t<S> &c2 ) ;
   template <typename S> __forceinline colora_t<S> operator * ( const colora_t<S> &c1, const colora_t<S> &c2 ) ;
   template <typename S> __forceinline colora_t<S> operator / ( const colora_t<S> &c1, const colora_t<S> &c2 ) ;

   template <typename S> __forceinline colora_t<S> operator + ( const colora_t<S> &c, S v ) ; 
   template <typename S> __forceinline colora_t<S> operator - ( const colora_t<S> &c, S v ) ; 
   template <typename S> __forceinline colora_t<S> operator * ( const colora_t<S> &c, S v ) ; 
   template <typename S> __forceinline colora_t<S> operator / ( const colora_t<S> &c, S v ) ; 

   template <typename S> __forceinline colora_t<S> operator + ( S v, const colora_t<S> &c ) ; 
   template <typename S> __forceinline colora_t<S> operator * ( S v, const colora_t<S> &c ) ; 

   template <typename S> __forceinline bool eq     ( colora_t<S> const & a, colora_t<S> const & b, S eps = color_epsilon<S>() );
   template <typename S> __forceinline bool eq_zero( colora_t<S> const & a, S eps = color_epsilon<S>() );

   template <typename S> __forceinline bool operator == ( const colora_t<S> &c1, const colora_t<S> &c2 ) ; 
   template <typename S> __forceinline bool operator != ( const colora_t<S> &c1, const colora_t<S> &c2 ) ; 

   template <typename S> __forceinline bool operator <  ( const colora_t<S> &c1, const colora_t<S> &c2 ) ; 

#pragma pack (pop)

   colorf color_white  () ;
   colorf color_black  () ;
   colorf color_red    () ;
   colorf color_green  () ;
   colorf color_blue   () ;
   colorf color_yellow () ;
   colorf color_magenta() ;
   colorf color_cyan   () ;
   colorf color_random () ;
}

// implementation 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace cg 
{
   template<> struct color_scalar_unit<unsigned char>
   {
      static __forceinline unsigned char get()     { return 255 ; }
      static __forceinline unsigned char epsilon() { return 0   ; }
   };
   template<> struct color_scalar_unit<float>
   {
      static __forceinline float         get()     { return 1.0f   ; }
      static __forceinline float         epsilon() { return 0.0035f; }
   };

   template<typename T> T color_epsilon() { return color_scalar_unit<T>::epsilon(); }

   template<typename S1, typename S2> 
      __forceinline S1 color_cast ( S2 value ) 
      {
         return static_cast<S1>( ((double)value/color_scalar_unit<S2>::get())*color_scalar_unit<S1>::get() ) ; 
      }

   //
   template<typename S>
      __forceinline color_t<S>::color_t ( S v ) 
         : r(v), g(v), b(v)
      {
      }

   template<typename S>
      __forceinline color_t<S>::color_t ( S r, S g, S b ) 
         : r(r), g(g), b(b)
      {
      }

   template<typename S> template<typename _S> 
      __forceinline color_t<S>::color_t( const color_t<_S> &c ) 
         : r (color_cast<S>(c.r))
         , g (color_cast<S>(c.g))
         , b (color_cast<S>(c.b))
      {
      }

   template<typename S> template<typename _S>
      __forceinline color_t<S>::color_t( const color_hsv_t<_S> &c )
      {
         colorf_hsv source (c);
         colorf dest;

         source.h *= 360 ; 

         if ( source.s == 0 )
            // If s is 0, all colors are the same (this is some flavor of gray)
            dest.r = dest.g = dest.b = source.v;
         else 
         {
            // The color wheel consists of 6 sectors. Figure out which sector you're in.
            float sectorPos = source.h / 60;
            int sectorNumber = cg::floor(sectorPos);

            // Get the fractional part of the sector. That is, how many degrees into the sector are you?
            float fractionalSector = sectorPos - sectorNumber;

            // Calculate values for the three axes of the color. 
            float p = source.v * (1 - source.s);
            float q = source.v * (1 - (source.s * fractionalSector));
            float t = source.v * (1 - (source.s * (1 - fractionalSector)));

            // Assign the fractional colors to r, g, and b based on the sector the angle is in.
            switch (sectorNumber) 
            {
            case 0:
                dest.r = source.v;
                dest.g = t;
                dest.b = p;
                break;
            case 1:
                dest.r = q;
                dest.g = source.v;
                dest.b = p;
                break;
            case 2:
                dest.r = p;
                dest.g = source.v;
                dest.b = t;
                break;
            case 3:
                dest.r = p;
                dest.g = q;
                dest.b = source.v;
                break;
            case 4:
                dest.r = t;
                dest.g = p;
                dest.b = source.v;
                break;
            case 5:
                dest.r = source.v;
                dest.g = p;
                dest.b = q;
                break;
            }
         }

         *this = color_t<S>(dest);
      }

   //
   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator += ( const color_t &c ) 
      {
         r += c.r ; 
         g += c.g ; 
         b += c.b ; 
         return *this ; 
      }

   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator -= ( const color_t &c ) 
      {
         r -= c.r ; 
         g -= c.g ; 
         b -= c.b ; 
         return *this ; 
      }

   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator *= ( const color_t &c ) 
      {
         r *= c.r ; 
         g *= c.g ; 
         b *= c.b ; 
         return *this ; 
      }

   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator /= ( const color_t &c ) 
      {
         r /= c.r ; 
         g /= c.g ; 
         b /= c.b ; 
         return *this ; 
      }


   //
   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator += ( S v ) 
      {
         r += v ; 
         g += v ; 
         b += v ; 
         return *this ; 
      }

   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator -= ( S v ) 
      {
         r -= v ; 
         g -= v ; 
         b -= v ; 
         return *this ; 
      }

   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator *= ( S v ) 
      {
         r *= v ; 
         g *= v ; 
         b *= v; 
         return *this ; 
      }

   template<typename S>
      __forceinline color_t<S> & color_t<S>::operator /= ( S v ) 
      {
         r /= v ; 
         g /= v ; 
         b /= v ; 
         return *this ; 
      }

   //
   template <typename S> 
      __forceinline color_t<S> operator + ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return color_t<S>(c1) += c2 ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator - ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return color_t<S>(c1) -= c2 ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator * ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return color_t<S>(c1) *= c2 ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator / ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return color_t<S>(c1) /= c2 ; 
      }

   //
   template <typename S> 
      __forceinline color_t<S> operator + ( const color_t<S> &c, S v )  
      {
         return color_t<S>(c) += v ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator - ( const color_t<S> &c, S v )  
      {
         return color_t<S>(c) -= v ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator * ( const color_t<S> &c, S v )  
      {
         return color_t<S>(c) *= v ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator / ( const color_t<S> &c, S v )  
      {
         return color_t<S>(c) /= v ; 
      }

   //
   template <typename S> 
      __forceinline color_t<S> operator + ( S v, const color_t<S> &c )  
      {
         return color_t<S>(c) += v ; 
      }

   template <typename S> 
      __forceinline color_t<S> operator * ( S v, const color_t<S> &c )  
      {
         return color_t<S>(c) *= v ; 
      }

   //
   template <typename S> 
      __forceinline bool eq( color_t<S> const & a, color_t<S> const & b, S eps )
      {
         return (abs(a.r - b.r) <= eps && abs(a.g - b.g) <= eps && abs(a.b - b.b) <= eps);
      }

   template <typename S> 
      __forceinline bool eq_zero( color_t<S> const & a, S eps )
      {
         return (abs(a.r) <= eps && abs(a.g) <= eps && abs(a.b) <= eps);
      }

   //
   template <typename S> 
      __forceinline bool operator == ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return memcmp( &c1, &c2, sizeof( color_t<S> ) ) == 0 ;
      }

   template <typename S> 
      __forceinline bool operator != ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return memcmp( &c1, &c2, sizeof( color_t<S> ) ) != 0 ;
      }

   template <typename S> 
      __forceinline bool operator < ( const color_t<S> &c1, const color_t<S> &c2 )  
      {
         return memcmp( &c1, &c2, sizeof( color_t<S> ) ) < 0 ;
      }

   //
   template<typename S>
      __forceinline colora_t<S>::colora_t ()
         : a(unit::get())
      {
      }

   template<typename S>
      __forceinline colora_t<S>::colora_t ( S r, S g, S b, S a)
         : color_t<S>(r, g, b)
         , a(a)
      {
      }

   template<typename S>
      __forceinline colora_t<S>::colora_t( const color_t<S>& c, S a ) 
         : color_t<S>(c)
         , a(a)
      {
      }

   template<typename S> template<typename _S> 
      __forceinline colora_t<S>::colora_t( const colora_t<_S> &c ) 
         : color_t<S>(c)
         , a (color_cast<S>(c.a))
      {
      }

   //
   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator += ( const color_t<S> &c ) 
      {                 
         r += c.r ; 
         g += c.g ; 
         b += c.b ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator -= ( const color_t<S> &c ) 
      {
         r -= c.r ; 
         g -= c.g ; 
         b -= c.b ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator *= ( const color_t<S> &c ) 
      {
         r *= c.r ; 
         g *= c.g ; 
         b *= c.b ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator /= ( const color_t<S> &c ) 
      {
         r /= c.r ; 
         g /= c.g ; 
         b /= c.b ; 
         return *this ; 
      }

      //
   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator += ( const colora_t<S> &c ) 
      {                 
         r += c.r ; 
         g += c.g ; 
         b += c.b ; 
         a += c.a ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator -= ( const colora_t<S> &c ) 
      {
         r -= c.r ; 
         g -= c.g ; 
         b -= c.b ; 
         a -= c.a ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator *= ( const colora_t<S> &c ) 
      {
         r *= c.r ; 
         g *= c.g ; 
         b *= c.b ; 
         a *= c.a ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator /= ( const colora_t<S> &c ) 
      {
         r /= c.r ; 
         g /= c.g ; 
         b /= c.b ; 
         a /= c.a ; 
         return *this ; 
      }

   //
   template <typename S> 
      __forceinline colora_t<S> operator + ( const colora_t<S> &c1, const color_t<S> &c2 )  
      {
         return colora_t<S>(c1) += c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator - ( const colora_t<S> &c1, const color_t<S> &c2 )  
      {
         return colora_t<S>(c1) -= c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator * ( const colora_t<S> &c1, const color_t<S> &c2 )  
      {
         return colora_t<S>(c1) *= c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator / ( const colora_t<S> &c1, const color_t<S> &c2 )  
      {
         return colora_t<S>(c1) /= c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator + ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return colora_t<S>(c1) += c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator - ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return colora_t<S>(c1) -= c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator * ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return colora_t<S>(c1) *= c2 ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator / ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return colora_t<S>(c1) /= c2 ; 
      }

      //
   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator += ( S v ) 
      {
         r += v ; 
         g += v ; 
         b += v ; 
         a += v ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator -= ( S v ) 
      {
         r -= v ; 
         g -= v ; 
         b -= v ; 
         a -= v ; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator *= ( S v ) 
      {
         r *= v ; 
         g *= v ; 
         b *= v; 
         a *= v; 
         return *this ; 
      }

   template<typename S>
      __forceinline colora_t<S> & colora_t<S>::operator /= ( S v ) 
      {
         r /= v ; 
         g /= v ; 
         b /= v ; 
         a /= v ; 
         return *this ; 
      }

      //
      template <typename S> 
   __forceinline colora_t<S> operator + ( const colora_t<S> &c, S v )  
      {
         return colora_t<S>(c) += v ; 
      }

      template <typename S> 
   __forceinline colora_t<S> operator - ( const colora_t<S> &c, S v )  
      {
         return colora_t<S>(c) -= v ; 
      }

      template <typename S> 
   __forceinline colora_t<S> operator * ( const colora_t<S> &c, S v )  
      {
         return colora_t<S>(c) *= v ; 
      }

      template <typename S> 
   __forceinline colora_t<S> operator / ( const colora_t<S> &c, S v )  
      {
         return colora_t<S>(c) /= v ; 
      }

      //
   template <typename S> 
      __forceinline color_t<S> operator + ( S v, const colora_t<S> &c )  
      {
         return colora_t<S>(c) += v ; 
      }

   template <typename S> 
      __forceinline colora_t<S> operator * ( S v, const colora_t<S> &c )  
      {
         return colora_t<S>(c) *= v ; 
      }

      //
   template <typename S> 
      __forceinline bool eq( colora_t<S> const & a, colora_t<S> const & b, S eps )
      {
         return (abs(a.r - b.r) <= eps && abs(a.g - b.g) <= eps && abs(a.b - b.b) <= eps && abs(a.a - b.a) <= eps);
      }

   template <typename S> 
      __forceinline bool eq_zero( colora_t<S> const & a, S eps )
      {
         return (abs(a.r) <= eps && abs(a.g) <= eps && abs(a.b) <= eps && abs(a.a) <= eps);
      }

   //
   template <typename S> 
      __forceinline bool operator == ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return memcmp( &c1, &c2, sizeof( colora_t<S> ) ) == 0 ;
      }

   template <typename S> 
      __forceinline bool operator != ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return memcmp( &c1, &c2, sizeof( colora_t<S> ) ) != 0 ;
      }

   template <typename S> 
      __forceinline bool operator < ( const colora_t<S> &c1, const colora_t<S> &c2 )  
      {
         return memcmp( &c1, &c2, sizeof( colora_t<S> ) ) < 0 ;
      }



   //
   inline colorf color_white  () { return colorf() ; }
   inline colorf color_black  () { return colorf( 0, 0, 0 ) ; }
   inline colorf color_red    () { return colorf( 1, 0, 0 ) ; }
   inline colorf color_green  () { return colorf( 0, 1, 0 ) ; }
   inline colorf color_blue   () { return colorf( 0, 0, 1 ) ; }
   inline colorf color_yellow () { return colorf( 1, 1, 0 ) ; }
   inline colorf color_magenta() { return colorf( 1, 0, 1 ) ; }
   inline colorf color_cyan   () { return colorf( 0, 1, 1 ) ; }

   inline colorf color_brown       () { return colorf( 0.75f, 0.5f, 0.f   ) ; }
   inline colorf color_orange      () { return colorf( 1.f,   0.5f, 0.f   ) ; }
   inline colorf color_darkgray    () { return colorf( 0.5f,  0.5f, 0.5f  ) ; }
   inline colorf color_lightblue   () { return colorf( 0.5f,  0.9f, 1.f   ) ; }
   inline colorf color_lightyellow () { return colorf( 0.85f, 0.8f, 0.58f ) ; }  

   inline colorf color_selected    () { return colorf( 1.f,   0.5f, 0.f   ) ; }

   inline colorf color_random ()
   {
      return colorf( rand( 1.f ), rand( 1.f ), rand( 1.f ) );
   }

   // TODO: correct this (u ~ brightness, v ~ brightness, so they can't be taken randomly this way)
   //inline colorf color_random ( float brigthness )
   //{
   //   // Brightness in [0, 1] (zero brightness will produce dark but not always black color).

   //   // Converting from YUV color model with provided brightness.
   //   // See http://en.wikipedia.org/wiki/YUV for details.

   //   float const u = rand( 0.463f * 2 ) - 0.463f;
   //   float const v = rand( 0.615f * 2 ) - 0.615f;

   //   return colorf( 
   //      brigthness + 1.13983f * v,
   //      brigthness - 0.39465f * u - 0.58060f * v,
   //      brigthness + 2.03211f * u );
   //}

   inline colorf color_random ( float brigthness )
   {
      return colorf (color_hsv_random(brigthness));
   }
}
