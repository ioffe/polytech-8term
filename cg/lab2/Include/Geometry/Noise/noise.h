#pragma once

#include "geometry\primitives\point.h"
#include "geometry\splines\catmull-rom.h"
#include "geometry\splines\easecurve.h"
#include "geometry\noise\perlinnoise.h"

#include "common\util.h"

//#include "common\staticptr.h"

namespace cg
{

//////////////////////////////////////////////////////////////////////////
// NOISE COMMON FUNCTIONS
//////////////////////////////////////////////////////////////////////////
__forceinline int RandMax( int max )
{
   return ::rand() % max;
}

template < class S >
   __forceinline S RandVal( S from = 0., S to = 1.)
{
   return cg::clamp<S>(0, 1, from, to)((S)::rand() / RAND_MAX);
}

template < class S >
   __forceinline S RandValSigned( S max = S(1.))
{
   return RandVal< S >(-max, max);
}

//////////////////////////////////////////////////////////////////////////
// class Random is generator of random numbers points of dimension = 1..3
// norm(V) = 1
//////////////////////////////////////////////////////////////////////////
class Random
{
public:
   template < class S >
      __forceinline void operator () ( S  &p ) const 
   {
      p = RandValSigned< S >();
   }   

   template < class S >
      __forceinline void operator () ( cg::point_t< S, 2 >  &p ) const 
   {
      S a( cg::pi * RandValSigned< S >() );
      p.x = cos(a);
      p.y = sin(a);
   }

   template < class S >
      __forceinline void operator () ( point_t< S, 3 > &p ) const 
   {
      point_t< S, 2 > x;
      operator()(x);
      S a( 2 * cg::pi * RandValSigned< S >() );
      p.x = x.y * cos(a);
      p.y = x.y * sin(a);
      p.z = x.x;
   }
};

//////////////////////////////////////////////////////////////////////////
// таблица PermutationTable<T> - случайная перестановка всех чисел целого типа T
// данные из таблицы можно получать по 
// одинарному (int), двойному (point_2i) и тройному (point_3i) индексу 
//////////////////////////////////////////////////////////////////////////
template <typename T> 
   struct PermutationTable
{
   static size_t const TABLE_SIZE = 1 << (sizeof(T) << 3);

public:
   __forceinline T     operator [] ( int x )                const { return table( x & (TABLE_SIZE - 1) ); }
   __forceinline T     operator [] ( point_2i const & x )   const { return (*this)[ (*this)[ x.x ] + x.y ]; }
   __forceinline T     operator [] ( point_3i const & x )   const { return (*this)[ (*this)[ static_cast< cg::point_2i const & >( x ) ] + x.z ]; }

private:
   struct table_type
   {
      table_type () 
      {
         for (size_t i = 0; i != TABLE_SIZE; ++i) 
            data[i] = (T)i;

         srand(0);
         for (size_t i = 0; i != TABLE_SIZE; ++i)
         {
            size_t const j = RandMax(TABLE_SIZE);
            std::swap( data[i], data[j] );
         }
      }
      T data[TABLE_SIZE] ; 
   };

private:
   __forceinline static T table( size_t x )
   {
      Assert( x < TABLE_SIZE );

      static table_type const t;   
      return t.data[x];
      //static StaticPtr<table_type> const t;  
      //return t -> data[x];
   }
};

//////////////////////////////////////////////////////////////////////////
// таблица NoiseTable<T,V> - 
// таблица случайных чисел типа V (float/double/point_2/3) единичной нормы. 
// Размер (периодичность) таблицы определяется типом T
// данные из таблицы можно получать по 
// одинарному (int), двойному (point_2i) и тройному (point_3i) индексу 
// данный класс является детерминированным случайным шумом целочисленного вектора
//////////////////////////////////////////////////////////////////////////
template <typename T, typename V> 
   struct NoiseTable
{
private:
   typedef  PermutationTable<T>   IndTable;
   
   static size_t const TABLE_SIZE = IndTable::TABLE_SIZE;

public:
   __forceinline NoiseTable( ) 
      : seed_( 0 )
   {}

   __forceinline void reseed(int seed = 0)
   {
      seed_ = seed;
   }

   template <typename I> 
      __forceinline V const & operator [] (const I & i) const 
   {
      return table((indTable_[i] + seed_) & (TABLE_SIZE - 1) );
   }

private:
   int seed_;
   IndTable indTable_;

private:
   struct table_type
   {
      table_type()
      {
         srand( 1737031680 );
         std::for_each( data, data + TABLE_SIZE, Random() );
      }

      V data[ TABLE_SIZE ];
   };

private:
   __forceinline static V const & table( size_t i )
   {
      Assert( i < TABLE_SIZE );

      static table_type const t;
      return t.data[ i ];
      //static StaticPtr<table_type> const t;  
      //return t -> data[ i ];
   }
};

template < class Derived >
   struct NoiseBase
      : util::crtp< Derived >
{
   double operator [](double p ) const { return self()[point_2(p,  0)];    }

   long Get1DHistogram ( int n, long * freq_list ) const
   {
      long max = 0;
      memset ( freq_list, 0, n*sizeof(long) );
      for ( double x = 0; x < 1024; x += 0.01 )
      {
         int i = (int)(((*this)[x]+1.)*n/2.);

         if (i>=n) i = n-1;
         if (i<0)  i = 0;
         freq_list[i] ++;
         cg::make_max ( max, freq_list[i] );
      }

      return max;
   }

   long Get2DHistogram ( int n, long * freq_list ) const
   {
      long max = 0;
      memset ( freq_list, 0, n*sizeof(long) );
      for ( double x = 0; x < 3.3; x += 0.01 )
      {
         for ( double y = 0; y < 3.3; y += 0.01 )
         {
            int i = (int)((self()[point_2(x, y)]+1.)*n/2.);

            if (i>=n) i = n-1;
            if (i<0)  i = 0;
            freq_list[i] ++;
            cg::make_max ( max, freq_list[i] );
         }
      }

      return max;
   }
};

//////////////////////////////////////////////////////////////////////////
// Базовый интерфейс работы с различными типами 
// вещественных шумов, распределенных в пространстве (размерности 1..2)
//////////////////////////////////////////////////////////////////////////
//class Noise2D
//{
//public:
//   virtual  ~Noise2D() {}
//
//   virtual  const char *   GetName()            const             = 0;
//   virtual  void           reseed (int seed)                      = 0;
//   
//   virtual  point_2        operator [] (const point_2  &p ) const = 0;
//   virtual  point_2        operator [] (double        p )   const {return (*this)[point_2(p,  0)];}
//};

template <class N2D> 
   struct Noise2D_X
      : NoiseBase< Noise2D_X< N2D > >
{
   const char *   GetName() const 
   { 
      static char buf[1024]; 
      sprintf(buf, "Noise2D_X<%s>", n2d.GetName()); 
      return buf;
   }

   void           reseed      (int seed)                    {return n2d.reseed(seed);}

   using NoiseBase::operator [];
   double         operator [] (const point_2  &p ) const    {return n2d[p].x;}
   double         operator [] (double          p ) const    {return n2d[p].x;}

private:
   N2D n2d;
};

//////////////////////////////////////////////////////////////////////////
// Block noise
//////////////////////////////////////////////////////////////////////////
template <class N> 
   struct BlockNoise
      : NoiseBase< BlockNoise< N > >
{
   BlockNoise(int nLayers = 1, double layerWidth = 0.5) : nLayers(nLayers), layerWidth(layerWidth) {return;}

   const char *   GetName() const 
   { 
      static char buf[1024]; 
      sprintf(buf, "BlockNoise<%s>", n.GetName()); 
      return buf;
   }

   void           reseed      (int seed)                 {return n.reseed(seed);}

   using NoiseBase::operator [];
   double         operator [] (const point_2  &p ) const {return get(p);}
   double         operator [] (double        p )   const {return get(p);}

private:
   template <typename X> double get( const X& p ) const
   {
      double x = (1 + n[p]) / 2.;
      double q = cg::mod(nLayers * x, 1);

      double x1 = x - q / nLayers;
      Assert( x1 >= -cg::epsilon<double>( ) );

      if (q < layerWidth)
         return 2.0 * x1 - 1.0;

      double x2 = x1 + 1.0 / nLayers;
      Assert( x2 <= 1.0 );

      return 2.0 * cg::lerp(layerWidth, 1.0, x1, x2) (q) - 1.0;
   }

   N n;
   int nLayers; 
   double layerWidth;
};

//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
template <class N> 
   struct Noise_Linear
      : NoiseBase< Noise_Linear< N > >
{
   const char  *  GetName()   const    {return "Noise_Linear";}
   void           reseed(int seed)     {n.reseed(seed); }

   using NoiseBase::operator [];

   double operator [] (double p) const 
   {
      int   ip = floor(p);
      double fp = p - ip;

      return cg::lerp<double>(0, 1, n[ip], n[ip + 1])(fp);
   }

   double operator [] (const point_2 & p) const 
   {
      point_2i ip = floor(p);
      point_2  fp = p - ip;

      double f0 = cg::lerp<double>(0, 1, n[ip + point_2i(0, 0)], n[ip + point_2i(1, 0)])(fp.x);
      double f1 = cg::lerp<double>(0, 1, n[ip + point_2i(0, 1)], n[ip + point_2i(1, 1)])(fp.x);
      return cg::lerp<double>(0, 1, f0, f1)(fp.y);
   }     
private:
   N                    n;
};

//////////////////////////////////////////////////////////////////////////
// Noise_EaseCurve noise
// 
// Значения в целых точках сидят в таблице, а между ними - интерполируются 
// по простейшему закону EaseCurve сплайна//поверхности:
// - интерполяция производится по двум//четырем точкам
// - производная в краевых точках равна нулю
//////////////////////////////////////////////////////////////////////////
template <class N> 
   struct Noise_EaseCurve
      : NoiseBase< Noise_EaseCurve< N > >
{
   const char *   GetName()         const {return "Noise_EaseCurve";}
   void           reseed(int seed)        {n.reseed(seed); }

   using NoiseBase::operator [];

   double operator [] (double p) const 
   {
      double val[2];

      int   ip = floor(p);
      double fp = p - ip;

      val[0] = n[ip];
      val[1] = n[ip + 1];

      return spline.Interpolate(2, val, fp);
   }

   double operator [] (const point_2 & p) const 
   {
      point_2i const & ip = floor(p);

      double const val[][2] = { 
         { n[ip + point_2i(0, 0)], n[ip + point_2i(0, 1)] },
         { n[ip + point_2i(1, 0)], n[ip + point_2i(1, 1)] }
      };

      return surf.Interpolate(val, p.x - ip.x, p.y - ip.y);
   }     

private:
   N                    n;
   cg::EaseCurvSpline<double> spline;
   cg::EaseCurvSurf<double>   surf;
};

//////////////////////////////////////////////////////////////////////////
// ValueNoise_CatmullRom noise
// 
// Значения в целых точках сидят в таблице, а между ними - интерполируются 
// по закону CatmullRom сплайна//поверхности:
// - интерполяция производится по четырем//шестнадцати точкам
// - две//двенадцать крайних точек используются для определения касательных.
// значения функции вычисляются по внутренним двум//четырем точкам
// (CatmullRomSpline(x[0..4], t=0)=x[1], CatmullRomSpline(x[0..4], t=1)=x[2]))
//////////////////////////////////////////////////////////////////////////
template <class N> 
   struct Noise_CatmullRom
      : NoiseBase< Noise_CatmullRom< N > >
{
   const char *   GetName()         const {return "Noise_CatmullRom";}
   void           reseed(int seed)        {n.reseed(seed);}

   using NoiseBase::operator [];

   __forceinline double operator [] (double p) const 
   {
      int   ip = floor(p);
      double fp = p - ip;
      
      double val[4];
      for (int i = -1; i <= 2; i++)
         val[i + 1] = n[ip + i];
      return spline.Interpolate(4, val, fp);
   }

   __forceinline double operator [] (const point_2 & p) const 
   {
      point_2i ip = floor(p);
      point_2  fp = p - ip;

      double val[16];
      point_2i i;
      for (i.y = -1; i.y <= 2; i.y++)
         for (i.x = -1; i.x <= 2; i.x++)
            val[4 * (i.x + 1) + i.y + 1] = n[ip + i];
      return surf.Interpolate(4, 4, val, fp.x, fp.y);
   }     

private:
   N                     n;
   cg::CatmullRomSpline<double> spline;
   cg::CatmullRomSurf<double>   surf;
};

//////////////////////////////////////////////////////////////////////////
// Gradient Noise
//////////////////////////////////////////////////////////////////////////
struct GradientNoise
   : NoiseBase< GradientNoise >
{
   const char *   GetName() const   {return "GradientNoise";}
   void           reseed(int seed)  {pn.reseed(seed);}

   using NoiseBase::operator [];
   double operator [] (const point_2 & p) const 
   {
      return cg::clamp< double > ( -1., 1., -1., 1. ) ( 2.f * ((GradientNoise *)this)->pn.noise(p.x, p.y) );
   }

private:
   cg::PerlinNoise pn;
};

//////////////////////////////////////////////////////////////////////////
// Sinus Noise
//////////////////////////////////////////////////////////////////////////
struct SinNoise
   : NoiseBase< SinNoise >
{
   const char *   GetName() const   {return "SinNoise";}
   void           reseed(int)       {}
   
   using NoiseBase::operator [];
   double         operator [](const point_2 & p) const { return sin(p.x + p.y); }
};

//////////////////////////////////////////////////////////////////////////
// Noise combinations
//////////////////////////////////////////////////////////////////////////
template <class LP_N, class HP_N, int LP_FREQ, int HP_WEIGHT> 
   struct NoiseC
      : NoiseBase< NoiseC< LP_N, HP_N, LP_FREQ, HP_WEIGHT > >
{
   const char *   GetName() const 
   { 
      static char buf[1024]; 
#if _MSC_VER >= 1400
      sprintf_s(buf, 1024, "NoiseC<%s,%s>", lpn.GetName(), hpn.GetName()); 
#else
      sprintf(buf, "NoiseC<%s,%s>", lpn.GetName(), hpn.GetName());
#endif
      return buf; 
   }

   void     reseed      (int seed)                 {lpn.reseed(seed); hpn.reseed(seed);}

   using NoiseBase::operator [];
   double   operator [] (const point_2  &p ) const {return phi(p);}
   double   operator [] (double          p ) const {return phi(p);}

private:
   template <typename X> 
      double phi (const X & x) const 
   { 
      return (1. - 1. / HP_WEIGHT) * lpn[x / LP_FREQ] + 1. / HP_WEIGHT * hpn[x]; 
   }

private:
   LP_N lpn;
   HP_N hpn;
};

//////////////////////////////////////////////////////////////////////////
// Value Noise 2D definitions
//////////////////////////////////////////////////////////////////////////
template <class N>
   struct AngleNoise2D
      : NoiseBase< AngleNoise2D< N > >
{
   const char *   GetName() const 
   { 
      static char buf[1024]; 
      sprintf(buf, "AngleNoise2D<%s>", n.GetName()); 
      return buf;
   }

   void  reseed (int seed) {return n.reseed(seed);}

   using NoiseBase::operator [];
   
   point_2 operator [] (const point_2  &p ) const 
   {
      double phi = cg::pi * (mod(n[p]) + 1); 
      return point_2(cos(phi), sin(phi));
   }

   point_2 operator [] (double p ) const 
   {
      double phi = cg::pi * (mod(n[p]) + 1); 
      return point_2(cos(phi), sin(phi));
   }

   double GetAngle ( const point_2 &p ) const 
   {
      return cg::pi * (mod(n[p]) + 1);
   }

private:
   double mod (double x) const 
   { 
      x = (x + 1.) / 2.; 
      x = cg::mod(16. * x, 1); 
      return 2. * x - 1.; 
   }

private:
   N n;
};

//////////////////////////////////////////////////////////////////////////
// Value Noise 2D definitions
//////////////////////////////////////////////////////////////////////////
typedef Noise_CatmullRom< NoiseTable< unsigned short, double > > ValueNoise_CatmullRom;
typedef Noise_EaseCurve < NoiseTable< unsigned short, double > > ValueNoise_EaseCurve;
typedef Noise_Linear    < NoiseTable< unsigned short, double > > ValueNoise_Linear;

typedef NoiseC<ValueNoise_EaseCurve, GradientNoise, 1, 2> ValueGradientNoise;

typedef AngleNoise2D < NoiseC<ValueNoise_EaseCurve, GradientNoise, 128, 32> > ValueGradientAngleNoise2D;
typedef Noise2D_X    < ValueGradientAngleNoise2D >                            ValueGradientAngleNoise2D_X;
typedef BlockNoise   < ValueNoise_Linear >                                    BlockNoise_Linear;

} // end of namespace cg

//
// End of file 'noise.h'
//