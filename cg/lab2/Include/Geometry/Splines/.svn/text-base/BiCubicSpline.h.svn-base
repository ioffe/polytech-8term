#pragma  once

#include <vector>

namespace cg {

   //----------------------------------------------
   // Метод наименьщих квадратов для 2-х мерного случая.
   // Возвращает параметры a, b для уравнения 
   // y = a * x + b
   template <typename FwdIter, typename scalar_type>
      void mnk( FwdIter p, FwdIter q, scalar_type & a, scalar_type & b )
   {
      scalar_type SX, SY, SXY, SX2;
      SX = SY = SXY = SX2 = ( scalar_type ) 0.0;
      int n = 0;
      for ( ; p != q; ++p )
      {
         const scalar_type x  = p->x;
         const scalar_type y  = p->y;

         SX     += x;
         SX2    += x * x;
         SY     += y;
         SXY    += x * y;

         ++n;
      }
      const scalar_type d = n * SX2 - SX * SX;
      //Assert( !cg::eq( d, ( scalar_type ) 0, ( scalar_type ) 1e-2 ) ); 
      a = ( n   * SXY  - SX * SY  ) / d;
      b = ( SX2 * SY   - SX * SXY ) / d;
   }



   //----------------------------------------------
   // Метод наименьщих квадратов для 2-х мерного случая.
   // Возвращает 2 набора параметров a, b для уравнения y = a * x + b:
   // ax, bx для прямой z(x) = ax * x + bx
   // ay, by для прямой z(y) = ay * y + by 
   template < typename FwdIter, typename scalar_type >
      void mnk3d( FwdIter p, FwdIter q, scalar_type & a, scalar_type & b, scalar_type & c )
   {
      int n = 0;
      scalar_type SX, SX2, SXZ, SY, SY2, SYZ, SZ, SXY;
      SX = SX2 = SXZ = SXY = SY = SY2 = SYZ = SZ = ( scalar_type ) 0;
      for ( ; p != q; ++p )
      {
         const scalar_type x = p->x, y = p->y, z = p->z;

         SX  += x;
         SY  += y;
         SZ  += z;

         SX2 += x * x;
         SY2 += y * y;
   
         SXY += x * y;
         SXZ += x * z;
         SYZ += y * z;

         ++n;
      }
      const scalar_type d = SX2 * ( SY * SY - n * SY2 ) + SX * SX * SY2 + SXY * ( - 2 * SX * SY + n * SXY );
      Assert( !cg::eq( d, ( scalar_type ) 0, ( scalar_type ) 1e-2 ) ); 
      if( cg::abs( d ) < 1e-5 )
      {
         a = b = c = 0;
      }
      else
      {
         a = - ( SX  * ( SY  * SYZ - SY2 * SZ ) + SXY * ( SY * SZ  - n   * SYZ ) + SXZ * ( n   * SY2 - SY * SY  ) ) / d;
         b = - ( SY  * ( SX  * SXZ - SX2 * SZ ) + SXY * ( SX * SZ  - n   * SXZ ) + SYZ * ( n   * SX2 - SX * SX  ) ) / d;
         c = - ( SX2 * ( SY2 * SZ  - SY * SYZ ) + SXY * ( SX * SYZ - SXY * SZ  ) + SXZ * ( SXY * SY  - SX * SY2 ) ) / d;
      }
   }

   //----------------------------------------------
   // Точка с производной в ней 
   template <typename Scalar >
      struct point_with_1_derivative: point_t< Scalar, 3 >
   {
      typedef point_t< Scalar, 3 >     point_type;
      typedef Scalar                   scalar_type;

      point_with_1_derivative()
         : dx( 0 )
      {}

      point_with_1_derivative( const point_type & p, const scalar_type & deriv ) 
         : point_type( p )
         , dx( deriv )
      {}

      scalar_type    dx;
   };

   //----------------------------------------------
   // Точка с двумя частными производными
   template <typename Scalar >
      struct point_with_2_derivatives: point_t< Scalar, 3 >
   {     
      typedef point_t< Scalar, 3 >     point_type;
      typedef Scalar                   scalar_type;

      point_with_2_derivatives( )
         : dx( 0 )
         , dy( 0 )
      {}

      point_with_2_derivatives( const point_type & p, scalar_type deriv_x, scalar_type deriv_y )
         : point_type( p )
         , dx( deriv_x )
         , dy( deriv_y )
      {}

      scalar_type dx, dy;
   };

   //----------------------------------------------
   // Двухмерный кубический сплайн для двух точек.
   template < typename scalar_type = float >
      struct TwoPointCubeSpline {

      typedef
         scalar_type
         scalar_type;

      TwoPointCubeSpline( )
         : x1_( 0 )
         , y1_( 0 )
         , x2_( 0 )
         , y2_( 0 )
      {}


      TwoPointCubeSpline( scalar_type x1, scalar_type y1, scalar_type dydx1,
                          scalar_type x2, scalar_type y2, scalar_type dydx2 )
                         : x1_( x1 )
                         , y1_( y1 )
                         , x2_( x2 )
                         , y2_( y2 ) 
      {
         CalcCoef( dydx1, dydx2 );   
      }

      template < typename real_type > 
         scalar_type operator()( real_type x ) const 
      {
         Assert( !cg::eq( x1_, x2_, std::numeric_limits< scalar_type >::epsilon() ));
         scalar_type new_x = ( scalar_type ) ( x - x1_ ) / ( x2_ - x1_ );
         scalar_type rez = y1_;
         scalar_type stx = ( scalar_type ) 1.0;
         for( int i = 0; i < 3; ++i ) 
         {
            stx *= new_x;
            rez += coef_[ i ] * stx;
         } 
         return rez;      
      }

      template < typename real_type > 
         scalar_type derivative( real_type x ) const 
      {
         Assert( !cg::eq( x1_, x2_, std::numeric_limits< scalar_type >::epsilon() ));
         scalar_type new_x = ( scalar_type ) ( x - x1_ ) / ( x2_ - x1_ );

         return (coef_[0] + (2*coef_[1] + 3*coef_[2]*new_x)*new_x) / (x2_ - x1_) ; 
      }

   private:  

      void CalcCoef( scalar_type dxdy1, scalar_type dxdy2 )
      {
         const scalar_type dx = x2_ - x1_;
         const scalar_type dy = y2_ - y1_;
         const scalar_type dxdy1_s = dxdy1 * dx;
         const scalar_type dxdy2_s = dxdy2 * dx;
         coef_[ 0 ] = dxdy1_s;
         coef_[ 1 ] = 3 * dy - dxdy2_s - 2 * dxdy1_s;
         coef_[ 2 ] = dxdy2_s - 2 * dy + dxdy1_s;
      }
   private:
      scalar_type x1_, y1_, x2_, y2_;
      scalar_type coef_[ 3 ];
   };

   //-------------------------------------------
   // Бикубический сплайн. Задается четырьмя точками c производными (частными).
   // !!! WARNING. Считается, что в проекции на плоскость XY 
   //              точки расположены в вершинах прямоугольника ABCD
   //              следующим образом:
   //                  
   //             Y ^
   //               |   B               C
   //               |    o.............o
   //               |    .             .
   //               |    .             .
   //               |    o.............o                        
   //               |   A               D
   //               |-----------------------> 
   //                                       X
   //    Качественная интерполяция возможна только внутри прямоугольника.

   template < typename Scalar >
      struct FourPointBiCubeSpline 
   {
      typedef  point_t< Scalar, 3 >                      point_type;
      typedef  Scalar                                    scalar_type;
      typedef  point_with_2_derivatives<scalar_type>     point_type_ext;
      typedef  TwoPointCubeSpline< scalar_type >         spline_2d;

      FourPointBiCubeSpline( )
      {}

      FourPointBiCubeSpline(const point_type_ext & a, const point_type_ext & b,
                            const point_type_ext & c, const point_type_ext & d )
                            : A_( a )
                            , B_( b ) 
                            , C_( c )
                            , D_( d )
                            , ADSpline_( a.x, a.z, a.dx, d.x, d.z, d.dx )
                            , BCSpline_( b.x, b.z, b.dx, c.x, c.z, c.dx )
      {
         Assert( cg::eq( A_.x, B_.x, ( scalar_type ) 1e-5 ) );
         Assert( cg::eq( A_.y, D_.y, ( scalar_type ) 1e-5 ) );
         Assert( cg::eq( B_.y, C_.y, ( scalar_type ) 1e-5 ) );
         Assert( cg::eq( C_.x, D_.x, ( scalar_type ) 1e-5 ) );
      } 

      point_type_ext const & A( ) const { return A_; }
      point_type_ext const & B( ) const { return B_; }
      point_type_ext const & C( ) const { return C_; }
      point_type_ext const & D( ) const { return D_; }

      template < typename real_type >
         scalar_type operator()( const real_type x, const real_type y ) const
      {       
         const scalar_type z1  = ADSpline_( x );
         const scalar_type z2  = BCSpline_( x );
         const scalar_type dy1 = InterpolateDY( A_, D_, ( scalar_type ) x );
         const scalar_type dy2 = InterpolateDY( B_, C_, ( scalar_type ) x );
         return TwoPointCubeSpline< scalar_type >( A_.y, z1, dy1, B_.y, z2, dy2 )( y );
      }

      template < typename Scalar >
         scalar_type operator()( point_t<Scalar, 2> const & p ) const
      {       
         return (*this)( p.x, p.y );
      }

   private:

      scalar_type InterpolateDY(const point_type_ext & a, const point_type_ext & b, const scalar_type & x) const
      {
         Assert( !cg::eq( b.x, a.x, ( scalar_type ) 1e-2 ) ); 
         return ( b.dy - a.dy ) * ( x - a.x ) / ( b.x - a.x ) + a.dy;
      }

   private:
      point_type_ext A_, B_, C_, D_;
      spline_2d ADSpline_, BCSpline_;
   };
} 