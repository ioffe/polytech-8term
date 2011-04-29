#pragma once

#include "noise.h"

/*
  Для генерации распределения в треугольнике с заданной плотностью вероятности, 
  используется следующий метод:
    
  1) Генерация случайной величины a на отрезке [0..1], 
     чтобы ее плотность вероятности была заданная функция p(x)

    Fa(x) = int[0..x]{p(t)dt} - функция распределения
    Fa(x) = P(a < x)
    F(0)=0
    F(1)=1

    G(x) = invF(x) G(F(x))=x
    G(0)=0
    G(1)=1

    b - случайная величина, равномерно распределенная на отрезке 0..1, тогда a = G(b), потому что
    Fa(x) = P(a<x) = P(G(b) < x) = P(b<F(x)) = Fb(F(x)) = F(x) => pa(x) = p(x)

  2) Генерация случайной величины равномерной плотности в треугольнике, (0,0)-(a,0)-(0,b)
     Генерация производится по осям - функция распределения вдоль оси х есть p(a-x)=(1+tgphi)x^tgphi
     где tgphi = b/a, Такая функция выбрана из условий:
     чтобы p(0)=0 p(1)=1
     чтобы tgphi=1 -> p=2x, tgphi=0 ->p = 1
     Вдоль оси y - p=1
     invF(x)=x^(1/(1+tgphi))

    Генерация осуществляется покординатно, сначала находится x, потом y
    Сначала точка получается в треугольнике 0,0-1,0-0,1, а ее переносят в базис другого треугольника 
    при этом функция плотности вероятности не изменяется
    Для упрощения считается, что tgphi=1

  3) Генерация случайной величины заданную линейной функцией плотности вероятности p(0)=c*n1 p(1)=c*n2
    
    p(x) = ax+b
    a=c(n2-n1) b=c*n1
    c: int(0..1){p(x)dx}=1 => c(a/2+b)=1 =>c=1(a/2+b)

    F(x)=a*x*x/2+b*x

    invF(x):
    a=0,b=0 - undefined
    a=0,b<>0: x/b
    a<>0 - (-b + cg::sqrt(b^2+a*x))/(a)

  4) Генерация случайной величины линейной интерполяцией по прямоугольному треугольнику  
     c заданной функцией плотности вероятности в вершинах
     Сначала вычисляется x - как композиция применения 3) и 2)
     Потом 'y' как 3)

  5) Расчет числа точек, которые попадают в заданный треугольник 
     с известной функцией плотности вероятности в углах и известной площадью, 
     и известной средней плотностью точек

    N = density / k * S
    k = 1/int[triangle(0,0;0,1;1,0)]f(x,y)dxdy(1)
	
 */

#include <geometry\primitives\point.h>

#define FRAND_MAX ( (double) RAND_MAX )

namespace cg
{

  class NoiseSetGenerator
  {
  public:

      // Генерирует случайное множетво точек в произвольном треугольнике,
      // согласно плотности вероятноси
      // v1, v2, v3 - вершины треугольника
      // n1, n2, n3 - значение плотности вероятности в вершинах
      // density    - средняя плотность
      // out        - выход
      template < class OutputIterator >
          static size_t GeneratePointsInTriangle( point_2 v1, point_2 v2, point_2 v3,
          double  n1, double  n2, double  n3,
          double  density,
          OutputIterator out)
      {
          size_t size = 0;

          // вычисляем квадраты длин сторон треугольника
          double len12 = distance_sqr( v2, v1 );
          double len23 = distance_sqr( v3, v2 );
          double len31 = distance_sqr( v1, v3 );

          // находим максимальную сторону
          double max_len = max( max(len12,len23), max(len23,len31) );

          // переставляем вершины так, чтобы v2 была при максимальном угле
          // и проверяем треугольник на прямоугольность
          bool is_right = false;

          if ( len12 == max_len )
          {
              std::swap( v3, v2 ), std::swap( n3, n2 );
              is_right = cg::abs( max_len - len23 - len31 ) < epsilon;
          }
          else if ( len23 == max_len )
          {
              std::swap( v1, v2 ), std::swap( n1, n2 );
              is_right = cg::abs( max_len - len12 - len31 ) < epsilon;
          }
          else if ( len31 == max_len )
          {
              is_right = cg::abs( max_len - len12 - len23 ) < epsilon;
          }

          if ( is_right )
              size += GeneratePointsInRightTriangle( v1, v2, v3, n1, n2, n3, density, out );
          else
          {  
              // делим треугольник на два прямоугоных, по длиннейшей стороне
              double  norm = cg::sqrt( max_len );
              point_2 base = (v3-v1) / norm;
              double  proj = (v2-v1) * base;
              double  n = n1 + (n3-n1) * proj/norm;
              point_2 h = v1 + ( base * proj );

              size += GeneratePointsInRightTriangle( v2, h, v1, n2, n, n1, density, out);
              size += GeneratePointsInRightTriangle( v3, h, v2, n3, n, n2, density, out);
          }

          return size;
      }

      // Генерирует множетво точек в прямоугольном треугольнике, 
      // согласно плотности вероятноси,
      // v1, v2, v3 - вершины треугольника, вершина v2 обязательно при прямом угле
      // n1, n2, n3 - значение плотности вероятности в вершинах
      // density    - средняя плотность
      // out        - выход
      template < class OutputIterator >
          static size_t GeneratePointsInRightTriangle( point_2 v1, point_2 v2, point_2 v3,
          double  n1, double  n2, double  n3,
          double  density,
          OutputIterator out)
      {
          double  k;
          point_2 a, b, c;

          Assert( ( distance_sqr( v1, v3 ) 
              - distance_sqr( v2, v1 ) 
              - distance_sqr( v3, v2 ) ) < epsilon * 1e05 );

          Assert( n1 >= 0.0 && n1 <= 1.0 );
          Assert( n2 >= 0.0 && n2 <= 1.0 );
          Assert( n3 >= 0.0 && n3 <= 1.0 );

          if ( ( cg::abs(n1) < epsilon ) && ( cg::abs(n2) < epsilon ) && ( cg::abs(n3) < epsilon ) )
              return 0;

          // если по первому катету значения вероятности совпадают, 
          // то начнем с другого ???
          if ( cg::abs(n2-n1) < epsilon )
              std::swap(v1,v3), std::swap(n1,n3);

          // вычисляем 
          a.x = n2-n1; b.x = n1;
          c.x = 1./(a.x*0.5+b.x);
          a.y = n3-n2; b.y = n2;
          c.y = 1./(a.y*0.5+b.y);

          // вычисляем нормировку для количества точек, по 5 
          k = 1./(a.x/3.+a.y/6.+n1*0.5);

          if ( k < epsilon )
              return 0;

          // количество точек попавших в треугольник, по 5                                        
          double dst = density * .5*cg::abs((v1 - v2) ^ (v3 - v2)) / k;

          int    num = dst;

          num += ( cg::rand(1.) < dst-num ) ? 1 : 0;

          for ( int i = 0; i < num; i ++ )
          {
              point_2 p;

              // равномерное распределение в треугольнике по 2
              // для скорости используется tgphi=1, 
              p.x = cg::sqrt( cg::rand(1.) );
              p.y =       cg::rand(1.);

              // распределение в треугольнике по значениям вероятности, вдоль x по 3
              if ( cg::abs(a.x) < epsilon )
                  if ( cg::abs(b.x) < epsilon )
                      p.x = p.x;
                  else
                      p.x = p.x/(b.x*c.x);
              else
                  p.x = ( -2.*c.x*b.x + 2.*cg::sqrt(c.x*c.x*b.x*b.x+2.*c.x*a.x*p.x) ) / (2.*c.x*a.x);

              // интерполяция значений вероятности для полученного x
              double ny3 = n1*(1-p.x)+n3*p.x;
              double ny2 = n1*(1-p.x)+n2*p.x;

              a.y = ny3-ny2;
              b.y = ny2;
              c.y = 1./(a.y*0.5+b.y);

              // распределение в треугольнике по значениям вероятности, вдоль y  по 3
              if ( cg::abs(a.y) < epsilon )
                  if ( cg::abs(b.y) < epsilon )
                      p.y = p.y;
                  else
                      p.y = p.y/(b.y*c.y);
              else
                  p.y = ( -2.*c.y*b.y + 2.*cg::sqrt(c.y*c.y*b.y*b.y+2*c.y*a.y*p.y) ) / (2.*c.y*a.y);
              p.y = p.y * p.x;

              // переходим в систему координат треугольника
              point_2 x = v2-v1;
              point_2 y = v3-v2;
              point_2 r = p;

              p.x = v1.x + x.x*r.x + y.x*r.y;
              p.y = v1.y + x.y*r.x + y.y*r.y;

              *out ++ = p;
          }

          return num;
      }
  };        

    namespace
    {
        template < class PointType >
            inline typename PointType::scalar_type TriangleArea( PointType const & v1, PointType const & v2, PointType const & v3 )
        {
            return cg::abs( v1.x * ( v2.y - v3.y ) + v2.x * ( v3.y - v1.y ) + v3.x * ( v1.y - v2.y ) ) / 2;
        }

        template < class Scalar >
            inline void SolveQuadraticEq( Scalar a, Scalar b, Scalar c, Scalar & x1, Scalar & x2 )
        {            
            if( cg::abs( a ) < epsilon )
            {
                if( cg::abs( b ) < epsilon )
                    x1 = x2 = 0;
                else
                    x1 = x2 = -c / b;
                return;
            }
            
            a *= 2;
            Scalar d = b * b - 2 * a * c;

            Assert( d >= 0 );
            d = cg::sqrt( d );
            
            x1 = -(d + b) / a;
            x2 =  (d - b) / a;
        }

        range_2i GetQueryRange( size_t num, range_2 const & range )
        {
            cg::range_2 scaledRange = range * num;

            return cg::range_2i( floor( scaledRange.lo( ) ), floor( scaledRange.hi( ) ) );
        }
    }

    //template < class PointType >
    //    unsigned GetPointsNumberInTriangle( PointType const & v1,PointType const & v2, PointType const & v3,
    //                                        double n1, double n2, double n3 )
    //{
    //    return floor( ( n1 + n2 + n3 ) * TriangleArea( v1, v2, v3 ) / 3 );
    //}
    //
    //template < class RandomEngine, class OutputIterator, class PointType, typename DensityType >
    //    void GeneratePointsInTriangle( PointType v1, PointType v2, PointType v3,
    //        DensityType n1, DensityType n2, DensityType n3,
    //        RandomEngine random, OutputIterator out, range_2i range )
    //{       
    //    typedef PointType::scalar_type Scalar;
    //    
    //    double min_n = min( min( n1, n2 ), n3 );

    //    if ( n3 == min_n )
    //    {
    //        std::swap( v3, v1 ), std::swap( n3, n1 );
    //    }
    //    else if ( n2 == min_n )
    //    {
    //        std::swap( v1, v2 ), std::swap( n1, n2 );
    //    }        

    //    Scalar k1 = (n1 + (n2 + n3) / 2) / 2;
    //    Scalar a1 = (n2 - n3) / 4;
    //    Scalar b1 = (n1 + n3) / 2;

    //    for( int i = range.lo( ); i <= range.hi( ); i++ )
    //    {            
    //        Scalar c1 = -k1 * static_cast< Scalar >( random( ) );

    //        Scalar x1, x2;
    //        SolveQuadraticEq( a1, b1, c1, x1, x2 );
    //                                
    //        Scalar x = x1;
    //        if( x < 0 || x > 1 )
    //            x = x2;                        

    //        Scalar k2 = (n1 + n3 + n2 * x - n3 * x) / 2;
    //        Scalar a2 = (n3 - n1 + (2 * n1 - n3 - n2) * x) / 2;
    //        Scalar b2 = (n1 - n1 * x + n2 * x);
    //        Scalar c2 = -k2 * static_cast< Scalar >( random( ) );

    //        Scalar y1, y2;
    //        SolveQuadraticEq( a2, b2, c2, y1, y2 );

    //        Scalar y = y1;
    //        if( y < 0 || y > 1 )
    //            y = y2;

    //        if( x + y > 1 )
    //        {
    //            Scalar k = x + y - 1;
    //            x -= k;
    //            y -= k;
    //        }
    //        
    //        Assert( cg::eq( x, 0 ) || ( x >= 0 && x <= 1 ) || cg::eq( x, 0 ) );
    //        Assert( cg::eq( y, 0 ) || ( y >= 0 && y <= 1 ) || cg::eq( y, 1 ) );            

    //        *out++ = cg::barycentric_interpolate_in_triangle( cg::barycentric_coords( y, x ), v1, v2, v3 );            
    //    }
    //}

    struct NoiseTableRandEngine
    {
        void Reseed( int seed )
        {
            offset = seed;
        }

        double operator ()( int i ) const
        {
            return ( 1 + noiseTable_[ offset + permTable_[i] ] ) / 2;
        }

    private:
        typedef cg::NoiseTable< unsigned short, double >    NoiseTableType;
        typedef cg::PermutationTable< unsigned short >      PermutationTableType;

        int                     offset;
        NoiseTableType          noiseTable_;
        PermutationTableType    permTable_;
    };

    // Generator of random points in a rectangle
    template < class PointType >
        struct RectanglePointsGenerator
    {
        RectanglePointsGenerator( PointType const & v1, PointType const & v2,
            double n1, double n2, double n3, double n4 )
            : v1_( v1 ), v2_( v2 )
            , n1_( n1 ), n2_( n2 )
            , n3_( n3 ), n4_( n4 )
        {}

        // Returnes number of the points to be generated
        unsigned PointCount( ) const
        {
            return floor( ( n1_ + n2_ + n3_ + n4_ ) * abs( v1_.x - v2_.x ) * abs( v1_.y - v2_.y ) / 4 ) / 100;
        }    

        // Generates points from the given range
        template < class RandEngine, class OutputIterator >
            void Generate( range_2i const & range, RandEngine & rand, OutputIterator out ) const
        {        
            double t1 = (n4_ + n2_ - n3_ - n1_) / 2;
            double t2 = (n4_ + n1_ - n3_ - n2_) / 2;
            double t3 = (n1_ + n3_) / 2;
            double t4 = (n3_ - n1_) / 2;

            double k1 = (n1_ + n2_ + n3_ + n4_) / 4;
            double a1 = (n2_ + n4_ - n1_ - n3_) / 4;
            double b1 = (n1_ + n3_) / 2;            

            for( int i = range.lo( ); i < range.hi( ); i++, out++ )
            {            
                // calculating x coordinate
                double x, xx;
                SolveQuadraticEq( a1, b1, -k1 * static_cast< double >( rand( i << 1 ) ), x, xx );
                
                // choosing appropriate root of the quadratic equation
                if( x < 0 || x > 1 )
                    x = xx;                        
                
                // calculating y coordinate
                double y, yy;
                SolveQuadraticEq( t4 + t2 * x, 
                                n1_ + (n2_ - n1_) * x,
                                -(t3 + t1 * x) * static_cast< double >( rand( (i << 1) + 1 ) ),
                                y, yy );
                
                // choosing appropriate root of the quadratic equation
                if( y < 0 || y > 1 )
                    y = yy;                       
                
                // interpolate calculated point in the rectangle
                *out = PointType( v1_.x + (v2_.x - v1_.x) * x, v1_.y + (v2_.y - v1_.y) * y );
            }
        }

        // Generates all points in the rectangle
        template < class RandEngine, class OutputIterator >
            void Generate( RandEngine & rand, OutputIterator out ) const
        {
            unsigned num = PointCount( );            
            Generate( cg::range_2i( 0, num ), rand, out );
        }

    private:        
        PointType       v1_, v2_;               // rectangle bounding points (bottom left and top right corners)
        double          n1_, n2_, n3_, n4_;     // densities in bottom left, bottom right, top left and top right corners
    };
        
    template < class PointType >
        struct UniformRectanglePointsGenerator
    {
        typedef typename PointType::scalar_type  scalar_type;

        UniformRectanglePointsGenerator( PointType const & v1, PointType const & v2, double density )
            : v1_( v1 ), v2_( v2 )
            , density_( density )
        {}

        // Returns number of the points to be generated
        unsigned PointCount( ) const
        {
            return floor( density_ * abs( v1_.x - v2_.x ) * abs( v1_.y - v2_.y ) ) / 5000;
        }    

        // Generates points from the given range
        template < class RandEngine, class OutputIterator >
            void Generate( range_2i const & range, RandEngine & rand, OutputIterator out ) const
        {        
            for( int i = range.lo( ); i < range.hi( ); i++, out++ )
            {            
                // calculating x coordinate
                double x = static_cast< double >( rand( i << 1 ) );
                                                
                // calculating y coordinate
                double y = static_cast< double >( rand( (i << 1) + 1 ) );
                
                // interpolate calculated point in the rectangle
                *out = PointType( static_cast< scalar_type >( v1_.x + (v2_.x - v1_.x) * x ),
                                  static_cast< scalar_type >( v1_.y + (v2_.y - v1_.y) * y ) );
            }
        }

        // Generates all points in the rectangle
        template < class RandEngine, class OutputIterator >
            void Generate( RandEngine & rand, OutputIterator out ) const
        {
            unsigned num = PointCount( );            
            Generate( cg::range_2i( 0, num ), rand, out );
        }

    private:        
        PointType       v1_, v2_;   // rectangle bounding points (bottom left and top right corners)
        double          density_;   // rectangle density
    };
}