#pragma once

#include <math.h>
#include <stdlib.h>
#include <limits>

#include <common\meta.h>

#include "scalar_traits.h"

#undef min
#undef max

#pragma pack ( push, 1 )

/*

   // обертка для получения информации о скалярах
   template <class T> struct scalar_traits;

   // epsilon
   template < class Scalar > Scalar epsilon();

   // pi
   const double pi;
   const int    INT_ETERNITY         = std::numeric_limits<int   >::max();
   const double FLOAT_ETERNITY       = std::numeric_limits<double>::max();
   const double FLOAT_FLOAT_ETERNITY = std::numeric_limits<float >::max();
   const double SENSOR_ETERNITY      = 1e10;

   // some scalar utils
   Scalar   mod               ( Scalar, Scalar );
   Scalar   abs               ( Scalar );
   Scalar   sqr               ( Scalar );
   bool     is_inf            ( Scalar );                // полезно искользовать эту функцию
   T        blend             ( T a, T b, Scalar t );    // t * a + (1 - t) * b
   int      sign              ( Scalar );                // {-1; 0; 1}
   int      sqrt              ( Scalar );
   bool     different_signs   ( Scalar x, Scalar y );    // x * y < 0; неужели используется?
   bool     is_pow2           ( long );
   int      log2i             ( long );
   Scalar   gcd               ( Scalar, Scalar );        // greatest common divisor
   
   // округления
   int      round             ( Scalar );
   double   round             ( double x, double step ); // step * round( x / step )
   int      floor             ( Scalar );                // floor
   int      ceil              ( Scalar );

   // проверка принадлежности отрезку/интервалу
   bool between01( double );                             // [0; 1]
   bool between01so( double );                           // [0; 1)
   bool between01eps( double, double eps = epsilon() );  // [-eps; 1+eps]

   // нечеткое сравнение
   bool     ge       ( Scalar a, Scalar b, Scalar eps = epsilon( ) );
   bool     le       ( Scalar a, Scalar b, Scalar eps = epsilon( ) );
   bool     eq       ( Scalar a, Scalar b, Scalar eps = epsilon( ) );
   bool     eq_zero  ( Scalar a, Scalar eps = epsilon( ) );
   Scalar   adjust   ( Scalar x, Scalar eps = epsilon( ) );                // если x "близко" к 0, делаем его 0
   bool     is_closer( Scalar x, Scalar step, Scalar dist = epsilon( ) );  // находится ли x на расстоянии dist от своего округления по шагу step

   // rand utils
   Scalar rand( Scalar max );             // [0; max)
   Scalar symmetric_rand( Scalar max );   // [-max; max)

   // grad2rad and rad2grad utils
   Scalar grad2rad( Scalar );
   Scalar grad2rad();
   Scalar rad2grad( Scalar );
   Scalar rad2grad();

   // min/max utils
   void     make_min( T &to_be_min, T x );
   void     make_max( T &to_be_max, T x );
   void     make_min( T & to_be_min, T x, D & assign_if, D const & y ); // говно какое-то
   void     make_max( T & to_be_max, T x, D & assign_if, D const & y ); // говно какое-то
   bool     make_min_ret( T &to_be_min, T x ); // возвращает true, если было проведено присваивание
   bool     make_max_ret( T &to_be_max, T x ); // возвращает true, если было проведено присваивание
   Scalar   min( Scalar, Scalar );
   Scalar   max( Scalar, Scalar );
   Scalar   min( Scalar, Scalar, Scalar ); // минимум из 3х
   Scalar   max( Scalar, Scalar, Scalar ); // максимум из 3х
   Scalar   min( Scalar, Scalar, Scalar, Scalar ); // минимум из 4х
   Scalar   max( Scalar, Scalar, Scalar, Scalar ); // максимум из 4х

   // lerp / clamp
   // ------- линейное преобразование [x0,x1] --> [y0,y1]
   template <class T, class D = T>
      struct Lerp
   {
      Lerp( T x0, T x1, D y0, D y1 );
      D operator() ( T x ) const;
   };

   // ------- three-point lerp
   template <class T, class D = T>
      struct Lerp3
   {
      Lerp3(T x0, T x1, T x2, D y0, D y1, D y2)
      D operator() ( T x ) const;
   };

   template <class T, class D = T>
      struct Clamp
   {
      Clamp( T x0, T x1, D y0, D y1 );
      D operator () ( T x ) const;
   };

   // non-linear clamp (three-point clamp)
   template <class T, class D = T>
      struct Clamp3
   {
      Clamp3( T x0, T x1, T x2,  D y0, D y1, D y2 );
      D operator () ( T x ) const;
   };

   Lerp<T>     lerp     ( T x0, T x1, T y0, T y1 );
   Lerp<T,D>   lerp_d   ( T x0, T x1, D y0, D y1 );
   Lerp3<T>    lerp3    ( T x0, T x1, T x2,  T y0, T y1, T y2 );
   Clamp<T>    clamp    ( T x0, T x1, T y0, T y1 );
   Clamp<T,D>  clamp_d  ( T x0, T x1, D y0, D y1 );
   Clamp3<T>   clamp3   ( T x0, T x1, T x2,  T y0, T y1, T y2 );

   V           slerp    ( S x0, S x1, V y0, V y1, S x );
   V           sclamp   ( S x0, S x1, V y0, V y1, S x );
   
   // делает out1 = min(in1,in2); out2 = max(in1,in2);
   void sort2( T in1, T in2, T &out1, T &out2 );
   void sort2( T & v1, T & v2 );

   struct Bound<T>
   {
      Bound( T vmin, T vmax );
      T operator( ) ( T value );
   };

   Bound<T> bound( T vmin, T vmax );
   T bound( T x, T vmin, T vmax ); // x < vmin ? vmin : x > vmax ? vmax : x;

   // some utils
   // TODO :: move them to util.h
   int prev( int index, int size );
   int next( int index, int size );

   // norm utils
   Scalar norm360 ( Scalar ); // приведение произвольной величины к диапазону [0, 360)
   Scalar norm180 ( Scalar ); // приведение произвольной величины к диапазону [-180, 180)
   double norm_2pi( double ); // приведение произвольной величины к диапазону [0, 2*Pi)
   double norm_pi ( double ); // приведение произвольной величины к диапазону [-Pi, Pi)
   Scalar norm    ( Scalar ); // abs

   double distance_sqr  ( double, double );
   double distance      ( double, double );


 */

namespace cg
{
   template<class T>  struct scalar_traits;

   template <> struct scalar_traits<float> 
   {
      static __forceinline float epsilon() {  return 1e-6f; }
   };

   template <> struct scalar_traits<double> 
   {
      static __forceinline double epsilon() {  return 1e-10; }
   };

   template <> struct scalar_traits<int> 
   {
      static __forceinline int epsilon() {  return 0; }
   };

   // глобальная (пока что) используемая при сравнении вещественных чисел точность
   //const double epsilon = scalar_traits<double>::epsilon();
   template < class Scalar > Scalar __forceinline epsilon() { return scalar_traits< Scalar >::epsilon( ); }

   __forceinline void compiler_error_workaround()
   {
      epsilon<float> () ; 
      epsilon<int>   () ; 
      epsilon<double>() ; 
   }

   // число pi
   const double pi = 3.14159265358979323846; 


   template <typename Scalar> __forceinline Scalar mod ( Scalar x, Scalar y ) { return x % y ; }
   
   // сделаю свою реализацию функций 
   __forceinline double mod ( double x, double y ) { return fmod( x, y ); }
   __forceinline float  mod ( float  x, float  y ) { return fmodf( x, y ); }

   __forceinline double sqrt( double x )  { return ::sqrt( x ); }
   __forceinline float  sqrt( float x )   { return ::sqrt( x ); }

   __forceinline int round(double x) { return x > 0 ? int(x + .5) : int(x - .5); }
   __forceinline int round(float  x) { return x > 0 ? int(x + .5f) : int(x - .5f); }

   __forceinline int floor(double f) { return (int)::floor( f ); }
   __forceinline int floor(float f)  { return (int)::floor( f ); }

   __forceinline int ceil(double f) { return (int)::ceil( f ); }
   __forceinline int ceil(float f)  { return (int)::ceil( f ); }

   template <typename Scalar> __forceinline Scalar abs ( Scalar x ) { return x >= 0 ? x : -x; }

   __forceinline double round(double x, double step)
   {
      return step * round(x / step);
   }


   // перевод градусы в радианы
   __forceinline double grad2rad(double grad) { return grad * pi / 180.0; }
   __forceinline float  grad2rad(float  grad) { return grad * float( pi / 180.0f ); }
   __forceinline double grad2rad(int    grad) { return grad * pi / 180.0; }
   __forceinline double grad2rad()            { return pi / 180.0; }

   // перевод из радиан в градусы
   __forceinline double rad2grad(double rad)  { return rad * 180.0 / pi; }
   __forceinline float  rad2grad(float  rad)  { return rad * float( 180.0 / pi ); }
   __forceinline double rad2grad(int    rad)  { return rad * 180.0 / pi; }
   __forceinline double rad2grad()            { return 180.0 / pi; }


   // возводит x в квадрат
   template <class T> __forceinline T sqr(T x) { return x * x; }

   // равномерное распределение на отрезке [0, max)
   __forceinline double rand(double max) { return max * ::rand() / (double)RAND_MAX; }
   __forceinline float  rand(float max)  { return max * ::rand() / (float) RAND_MAX; }
   __forceinline int    rand(int max)    { return ::rand() % max; } // TODO :: это нифига не rand - распределение будет не то же, что rand() / RandMax * max

   // равномерное распределение на отрезке [-max, max)
   __forceinline double symmetric_rand(double max) { return rand(max * 2.0)  - max; }
   __forceinline float  symmetric_rand(float max)  { return rand(max * 2.0f) - max; }
   __forceinline int    symmetric_rand(int max)    { return rand(max * 2)    - max; }

   __forceinline bool is_inf(double x) { return !( -1e30 < x && x < 1e30 ); }

   // определяет принадлежность x отрезку [0,1]
   __forceinline bool between01(double x) { return 0 <= x && x <= 1; }

   // определяет принадлежность полуоткрытому отрезку [0;1)
   __forceinline bool between01so(double x) { return 0 <= x && x < 1; }

   __forceinline bool between01eps(double x, double eps=epsilon< double >( )) { return -eps <= x && x - 1 <= eps; }

   // Линейная комбинация: ta + (1-t)b
   template <class T> T blend(T const & a, T const & b, double t)
   {
      return t*a + (1-t)*b;
   }

   template <class T> T blend(T const & a, T const & b, float t)
   {
      return t*a + (1-t)*b;
   }

   // определяет знак числа
   __forceinline int sign (int    x)  { return x < 0 ? -1 : x > 0 ? 1 : 0; }
   __forceinline int sign (float  x)  { return x < 0 ? -1 : x > 0 ? 1 : 0; }
   __forceinline int sign (double x)  { return x < 0 ? -1 : x > 0 ? 1 : 0; }

   // определяет, имеют ли x и y строго разные знаки
   __forceinline bool different_signs(double x, double y) 
   {   return x * y < 0; }

   // сравнение двух вещественных чисел с точностью epsilon

   // fuzzy greater equal
   __forceinline bool ge (float a,  float b,  float  eps = epsilon< float >( ) ) { return a - b >= - eps; }
   __forceinline bool ge (double a, double b, double eps = epsilon< double >( )) { return a - b >= - eps; }

   // fuzzy greater than
   __forceinline bool gt (float a,  float b,  float  eps = epsilon< float >( ) ) { return a - b >= + eps; }
   __forceinline bool gt (double a, double b, double eps = epsilon< double >( )) { return a - b >= + eps; }

   // fuzzy less equal
   __forceinline bool le (float a,  float b,  float  eps = epsilon< float >( ) ) { return a - b <= + eps; }
   __forceinline bool le (double a, double b, double eps = epsilon< double >( )) { return a - b <= + eps; }

   // fuzzy less than
   __forceinline bool lt (float a,  float b,  float  eps = epsilon< float >( ) ) { return a - b <= - eps; }
   __forceinline bool lt (double a, double b, double eps = epsilon< double >( )) { return a - b <= - eps; }

   // fuzzy equal
   __forceinline bool eq (float a,  float b,  float  eps = epsilon< float >( )) { return abs(a - b) <= eps; }
   __forceinline bool eq (float a,  int   b,  float  eps = epsilon< float >( )) { return abs(a - b) <= eps; }
   __forceinline bool eq (double a,  double b,  double  eps = epsilon< double >( )) { return abs(a - b) <= eps; }
   __forceinline bool eq (double a,  int    b,  double  eps = epsilon< double >( )) { return abs(a - b) <= eps; }
   __forceinline bool eq (float  a,  int    b,  double  eps = epsilon< double >( )) { return abs((double)a - b) <= eps; }

   // fuzzy equality to 0
   __forceinline bool eq_zero (float  a, float  eps = epsilon< float >( ) ) { return abs(a) <= eps; }
   __forceinline bool eq_zero (double a, double eps = epsilon< double >( )) { return abs(a) <= eps; }
   __forceinline bool eq_zero (int a, double eps = epsilon< int >( )) { return abs(double(a)) <= eps; }

   namespace details
   {
      template < class Scalar >
      __forceinline bool eq_rel( Scalar a, Scalar b, Scalar max_relative_error, Scalar max_absolute_error )
      {
          return abs( a - b ) <= std::max( max_absolute_error, max_relative_error * std::max( abs( a ), abs( b ) ) );
      }
   }

   __forceinline bool eq_rel(double a, double b, double max_relative_error = epsilon< double >(), double max_absolute_error = epsilon< double >())
   {
      return details::eq_rel( a, b, max_relative_error, max_absolute_error );
   }

   __forceinline bool eq_rel(float a, float b, float max_relative_error = epsilon< float >(), float max_absolute_error = epsilon< float >())
   {
      return details::eq_rel( a, b, max_relative_error, max_absolute_error );
   }

   __forceinline bool le_rel(double a, double b, double max_relative_error = epsilon< double >(), double max_absolute_error = epsilon< double >())
   {
      return a < b || eq_rel( a, b, max_relative_error, max_absolute_error );
   }

   __forceinline bool ge_rel(float a, float b, float max_relative_error = epsilon< float >(), float max_absolute_error = epsilon< float >())
   {
      return a > b || eq_rel( a, b, max_relative_error, max_absolute_error );
   }

   // если x "близко" к 0, делаем его 0
   template < class Scalar > __forceinline Scalar adjust(Scalar x, Scalar e = epsilon< Scalar >( )) { return eq_zero(x, e) ? 0 : x; }

   __forceinline bool is_closer(double x, double step, double dist = epsilon< double >( ))
   {
      return eq(x, round(x, step), dist);
   }

   // альтернатива обычному floor(double)

   template <class T> __forceinline bool make_min(T &to_be_min, T x) { if (to_be_min > x) {to_be_min = x; return true; } return false; }
   template <class T> __forceinline bool make_max(T &to_be_max, T x) { if (to_be_max < x) {to_be_max = x; return true; } return false; }

   template <typename T, typename D>
   void __forceinline make_min ( T & to_be_min, T x, D & assign_if, D const & y )
   {
      if ( to_be_min > x )
      {
         to_be_min = x;
         assign_if = y;
      }
   }

   template <typename T, typename D>
   void __forceinline make_max ( T & to_be_max, T x, D & assign_if, D const & y )
   {
      if ( to_be_max < x )
      {
         to_be_max = x;
         assign_if = y;
      }
   }

   template <class T> __forceinline bool make_min_ret(T &to_be_min, T x) { if (to_be_min > x) { to_be_min = x; return true; } return false; }
   template <class T> __forceinline bool make_max_ret(T &to_be_max, T x) { if (to_be_max < x) { to_be_max = x; return true; } return false; }

   __forceinline int min(int a, int b) { return a < b ? a : b; }
   __forceinline int max(int a, int b) { return a > b ? a : b; }

   __forceinline unsigned min(unsigned a, unsigned b) { return a < b ? a : b; }
   __forceinline unsigned max(unsigned a, unsigned b) { return a > b ? a : b; }

   __forceinline unsigned long min(unsigned long a, unsigned long b) { return a < b ? a : b; }
   __forceinline unsigned long max(unsigned long a, unsigned long b) { return a > b ? a : b; }

   __forceinline __int64 min(__int64 a, __int64 b) { return a < b ? a : b; }
   __forceinline __int64 max(__int64 a, __int64 b) { return a > b ? a : b; }

   __forceinline unsigned __int64 min(unsigned __int64 a, unsigned __int64 b) { return a < b ? a : b; }
   __forceinline unsigned __int64 max(unsigned __int64 a, unsigned __int64 b) { return a > b ? a : b; }

   __forceinline double min(double a, double b) { return a < b ? a : b; }
   __forceinline double max(double a, double b) { return a > b ? a : b; }

   __forceinline float min(float a, float b) { return a < b ? a : b; }
   __forceinline float max(float a, float b) { return a > b ? a : b; }

   template <class T> __forceinline T max(T a, T b, T c) { return max(a, max(b,c)); }
   template <class T> __forceinline T min(T a, T b, T c) { return min(a, min(b,c)); }

   template <class T> __forceinline T max(T a, T b, T c, T d) { return max(max(a,b),max(c,d)); }
   template <class T> __forceinline T min(T a, T b, T c, T d) { return min(min(a,b),min(c,d)); }

   const int    INT_ETERNITY         = std::numeric_limits<int   >::max();
   const double FLOAT_ETERNITY       = std::numeric_limits<double>::max();
   const double FLOAT_FLOAT_ETERNITY = std::numeric_limits<float >::max();
   const double SENSOR_ETERNITY      = 1e10;

   __forceinline bool is_pow2( long x )
   {
      if ( x <= 0 )
         return false;
      return ( (x & (x - 1)) == 0 );
   }


   __forceinline int log2i( long x )
   {
      if ( x <= 0 )
         return -1;
      int res = 0;
      while ( x >>= 1 )
         res++;
      return res;
   }

   template <class T, class D = T>
      struct Lerp
   {
      typedef 
         typename meta::_if< meta::_is_integral< D >, T, D >::type 
         tform_type;

      // линейное преобразование [x0,x1] --> [y0,y1]
      __forceinline Lerp(T x0, T x1, D y0, D y1)
         : K_(cg::eq_zero(x1 - x0) ? tform_type() * T(0.) : ( y1 - y0 ) * ( T(1) / (x1 - x0) ))
         , D_( y0 - K_ * x0 )
      {}

      __forceinline D operator() (T x) const {
         return ( D ) ( K_*x + D_ );
      }

      __forceinline tform_type const& K() const {
         return K_ ; 
      }

   private:
      tform_type K_;
      tform_type D_;
   };

   template <class T> __forceinline  Lerp<T> lerp(T x0, T x1, T y0, T y1) 
   {
      return Lerp<T>(x0,x1,y0,y1);
   }

   template <class T, class D> __forceinline  Lerp<T,D> lerp_d(T x0, T x1, D y0, D y1) 
   {
      return Lerp<T,D>(x0,x1,y0,y1);
   }

   template <class V, class S> __forceinline  V slerp(S x0, S x1, V y0, V y1, S x) 
   {
      S t = (x - x0) / (x1 - x0);
      S s = (3 - 2 * t) * t * t;

      return y0 * (1 - s) + y1 * s;
   }

   // three-point lerp
   template <class T, class D = T>
      struct Lerp3
   {
      // линейное преобразование [x0,x1] --> [y0,y1]
      __forceinline Lerp3(T x0, T x1, T x2, D y0, D y1, D y2)
         : K1_(cg::eq_zero(x1 - x0) ? D() * T(0.) : (y1 - y0) * ( T(1) / (x1 - x0) ) )
         , D1_(y0 - K1_*x0)
         , K2_(cg::eq_zero(x2 - x1) ? D() * T(0.) : (y2 - y1) * ( T(1) / (x2 - x1) ) )
         , D2_(y1 - K2_*x1)
         , xx_(x1)
      {}

      __forceinline D operator() (T x) const {
         return x < xx_ 
            ?  x*K1_ + D1_
            :  x*K2_ + D2_;
      }

   private:
      T    xx_;
      D    K1_,D1_;
      D    K2_,D2_;
   };

   template <class T> __forceinline  Lerp3<T> lerp3(T x0, T x1, T x2,  T y0, T y1, T y2) 
   {
      return Lerp3<T>(x0,x1,x2, y0,y1,y2);
   }

   // делает out1 = min(in1,in2); out2 = max(in1,in2);
   template <class T>
      __forceinline void sort2(T in1, T in2, T &out1, T &out2)
   {
      if (in1 < in2) { out1 = in1; out2 = in2; }
      else           { out1 = in2; out2 = in1; }
   }

   template <class T>
      __forceinline void sort2(T & v1, T & v2)
   {
      if (v1 > v2)
         std::swap(v1, v2);
   }

   template <class T>
      __forceinline void sort3(T & v1, T & v2, T & v3)
   {
      sort2( v1, v2 );
      sort2( v1, v3 );
      sort2( v2, v3 );
   }

   template <class T, class D = T>
      struct Clamp
   {
      __forceinline Clamp(T x0, T x1, D y0, D y1) 
         : x0(x0), x1(x1), y0(y0), y1(y1)
         , l_( x0, x1, y0, y1 )
      {}

      __forceinline D operator () (T x) const {
         return 
            x <= x0 ? y0 :
            x >= x1 ? y1 :
            l_(x);
      }

   private:
      T   x0, x1;
      D   y0, y1;
      Lerp<T, D> l_;
   };

   template <class T> __forceinline  Clamp<T> clamp(T x0, T x1, T y0, T y1)
   {
      return Clamp<T>(x0, x1, y0, y1);
   }

   template <class T, class D> __forceinline  Clamp<T,D> clamp_d(T x0, T x1, D y0, D y1) 
   {
      return Clamp<T,D>(x0,x1,y0,y1);
   }

   template <class V, class S> __forceinline  V sclamp(S x0, S x1, V y0, V y1, S x) 
   {
      if (x <= x0) return y0;
      if (x >= x1) return y1;
      return slerp<V, S>(x0, x1, y0, y1, x);
   }

   // non-linear clamp (three-point clamp)
   template <class T, class D = T>
      struct Clamp3
   {
      __forceinline Clamp3(T x0, T x1, T x2,  D y0, D y1, D y2) 
         :   x0(x0), x2(x2),  y0(y0), y2(y2)
         ,   l_( x0, x1, x2, y0, y1, y2 )
      {}

      __forceinline D operator () (T x) const {
         return 
            x <= x0 ? y0 :
            x >= x2 ? y2 :
            l_(x);
      }

   private:
      T   x0, x2;
      D   y0, y2;
      Lerp3<T,D> l_;
   };

   template <class T> __forceinline  Clamp3<T> clamp3(T x0, T x1, T x2,  T y0, T y1, T y2)
   {
      return Clamp3<T>(x0, x1, x2,  y0, y1, y2);
   }

   template < class T >
      struct Bound
   {
      Bound( T min_val, T max_val )
         : min_val_( min_val )
         , max_val_( max_val )
      {}

      T operator( ) ( T val ) const
      {
         return val > max_val_ ? max_val_ : val < min_val_ ? min_val_ : val ;
      }

   private:
      T min_val_;
      T max_val_;
   };

   template < class T > __forceinline
      Bound<T> bound( T vmin, T vmax )
   {
      return Bound<T>( vmin, vmax );
   }

   template< class T> __forceinline 
      T bound(T x, T vmin, T vmax)
   {
      return x < vmin ? vmin : x > vmax ? vmax : x;
   }

   /* Next & previous indexes for closed (cyclic) arrays */  
   int __forceinline prev(int index, int size) 
   {
      return (index == 0) ? size - 1 : index - 1;
   }

   int __forceinline next(int index, int size)
   {
      return (index == size - 1) ? 0 : index + 1;
   }

   // приведение произвольной величины к диапазону [0, 360)
   template < class T >
      __forceinline T norm360 ( T x ) 
   {
      if ( x >= 360 )
         x -= ((int)(x/360))*T(360) ; 
      else if ( x < 0 ) 
         x += (1 + (int)(-x/360))*T(360) ; 
      return x ; 
   }

   // приведение произвольной величины к диапазону [-180, 180)
   template < class T >
      __forceinline T norm180 ( T x ) 
   {
      x = norm360(x) ; 
      if ( x >= 180 ) 
         x -= 360 ; 
      return x ; 
   }

   // приведение произвольной величины к диапазону [0, 2*Pi)
   __forceinline double norm_2pi ( double x ) 
   {
      return x - 2*pi * floor(x / (2*pi));
   }

   // приведение произвольной величины к диапазону [-Pi, Pi)
   __forceinline double norm_pi ( double x ) 
   {
      x = norm_2pi(x) ; 
      if ( x >= pi ) 
         x -= 2*pi ; 
      return x ; 
   }

   __forceinline double norm( double value )
   {
      return abs( value ) ;
   }

   __forceinline float norm( float value )
   {
      return abs( value ) ;
   }

   __forceinline double distance_sqr(double A, double B)
   {   return (A - B)*(A - B); }

   __forceinline double distance(double A, double B)
   {   return abs(A - B); }

   // greatest common divisor
   template<class T>
      __forceinline T gcd( T a, T b )
   {
      STATIC_ASSERT(meta::_is_integral<T>::value, T_must_be_integral)
         if ( b == 0 ) return a;
      return gcd( b, a % b );
   }

   __forceinline bool is_power_of_two ( size_t value ) 
   {
      return (value & (value - 1)) == 0 ; 
   }
}

#pragma pack ( pop )
