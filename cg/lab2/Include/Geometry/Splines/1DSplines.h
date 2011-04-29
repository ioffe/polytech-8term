#pragma once 

namespace cg
{

template < class ScalarType >
struct TridiagonalSys
{
   // ---------------------------
   // tridiagonal system solving 
   //
   // e.g. for 6x6:
   //
   //        Matrix        B
   //
   //  b1 c1 0  0  0  0 .  d1
   //  a2 b2 c2 0  0  0 .  d2
   //  0  a3 b3 c3 0  0 .  d3
   //  ..................  ..
   //  0  0  0  0 a6 b6 .  d6
   // 

   struct SysString 
   {
      SysString()
         : a ( 0 ), b ( 0 ), c ( 0 )
      {
      }

      SysString ( ScalarType a, ScalarType b, ScalarType c )
         : a ( a ), b ( b ), c ( c )
      {
      }

      ScalarType a, b, c ;
   };

   typedef          
      std::vector < SysString >
      SysValues ;
   
   typedef 
      std::vector < ScalarType >
      ValuesVector ;


   static bool SolveSys( SysValues const& sys, ValuesVector d, ValuesVector & res )
   {
      // checking for requirements 
      for ( unsigned i = 0; i < sys.size(); ++ i )
         if ( cg::abs( sys[ i ].b ) < cg::abs( sys[ i ].a ) + cg::abs( sys[ i ].c ))
            return false ;

      res.resize( sys.size()) ;

      std::vector< ScalarType > alpha ( sys.size() - 1 ) ;
      std::vector< ScalarType > betta ( sys.size() - 1 ) ;

      alpha[ 0 ] = - sys[ 0 ].c / sys[ 0 ].b ;
      betta[ 0 ] =     d[ 0 ]   / sys[ 0 ].b ;

      // forward passing 
      for ( unsigned i = 1 ; i < sys.size() - 1; ++ i )
      {
         ScalarType div = sys[ i ].b + alpha[ i - 1 ] * sys[ i ].a ;

         alpha[ i ] = - sys[ i ].c / div ;
         betta[ i ] = ( d  [ i ] - sys[ i ].a * betta[ i - 1 ] ) / div ;
      }
       
      // backward passing 
      unsigned last = sys.size() - 1 ;
      res[ last ] = ( d[ last ] - sys[ last ].a * betta[ last - 1 ] ) / ( sys[ last ].b + sys [ last ].a * alpha[ last - 1 ] ) ;
      
      for ( int i = sys.size() - 2 ; i >= 0 ; --i )
         res[ i ] = alpha [ i ] * res [ i + 1 ] + betta[ i ] ;
   
      return true ;
   }
} ;


namespace Splines 
{

namespace Details 
{
   template< class ScalarType >  
   struct KnotPoint 
   {
      ScalarType value ;
      ScalarType param ;

      KnotPoint( )
         : value ( 0 )
         , param ( 0 )
      {
      }

      KnotPoint( ScalarType param, ScalarType value )
         : value ( value )
         , param ( param )
      {
      }
   };

   template< class Scalar >
   bool operator == ( KnotPoint< Scalar > const & a, KnotPoint< Scalar > const & b )
   {
      return ( a.param == b.param ) && ( a.value == b.value );
   }

   template < class ScalarType > 
   struct CubicPolynomial  
   {
      CubicPolynomial() 
      {
      }

      ~CubicPolynomial()
      {
      }

      ScalarType operator()( ScalarType x ) const
      {
         return Interpolate( x ) ;
      }

      ScalarType Interpolate( ScalarType x ) const 
      {
         ScalarType t = x - x0 ;
         return ((( a * t ) + b ) * t + c ) * t + d ;
      }

      ScalarType Derivative( ScalarType x ) const 
      {
         ScalarType t = x - x0 ;
         return (( a * ( 3 * t )) + b * 2 ) * t + c ;
      }

      ScalarType SecondDerivative( ScalarType x ) const
      {
         ScalarType t = x - x0 ;
         return ( a * ( 6 * t )) + b * 2 ;
      }

      ScalarType ThirdDerivative( ScalarType x ) const
      {
         ScalarType t = x - x0 ;
         return a * 6 ;
      }

      bool  InBound( ScalarType x ) const 
      {
         return ge( x, x0 ) && le( x, x1 ) ;
      }

      ScalarType x0, x1 ;
      ScalarType a, b, c, d ;
   };

   template< class Scalar >
   bool operator == ( CubicPolynomial< Scalar > const & a, CubicPolynomial< Scalar > const & b )
   {
      return (a.x0 == b.x0) && (a.x1 == b.x1) && (a.a == b.a) && (a.b == b.b) && (a.c == b.c) && (a.d == b.d);
   }

   template < class ScalarType > 
   struct LinearPolynomial  
   {
      LinearPolynomial() 
      {
      }

      ~LinearPolynomial()
      {
      }

      ScalarType operator()( ScalarType x ) const
      {
         return Interpolate( x ) ;
      }

      ScalarType Interpolate( ScalarType x ) const 
      {
         ScalarType t = x - x0 ;
         return a * t + b ;
      }

      ScalarType Derivative( ScalarType x ) const 
      {
         return a ;
      }

      ScalarType SecondDerivative( ScalarType x ) const
      {
         return 0 ;
      }

      ScalarType ThirdDerivative( ScalarType x ) const
      {
         return 0 ;
      }

      bool  InBound( ScalarType x ) const 
      {
         return ge( x, x0 ) && le( x, x1 ) ;
      }

      ScalarType x0, x1 ;
      ScalarType a, b ;
   };

} // Details 

namespace Constructors
{

template < class ScalarType >
struct Traits 
{
   typedef  Details::CubicPolynomial   < ScalarType >   CubicPolynomial  ;
   typedef  Details::KnotPoint         < ScalarType >   KnotPoint        ;
   typedef  Details::LinearPolynomial  < ScalarType >   LinearPolynomial ; 

   typedef  std::vector < CubicPolynomial  >  CubicPolynomials  ;
   typedef  std::vector < KnotPoint        >  KnotPoints        ;
   typedef  std::vector < LinearPolynomial >  LinearPolynomials ;
   typedef  std::vector < ScalarType       >  PointDerivatives  ;
};

//////////////////////////////////////////////////////////////////////////
// Natural spline constructor 
//
// constructs cubic spline with equal first and second(!) derivatives at reference points 
// starts and ends with second derivative equal to zero 

template < class ScalarType >                           
struct Natural
{
   typedef typename Traits< ScalarType >::CubicPolynomial  Polynomial  ;
   typedef typename Traits< ScalarType >::CubicPolynomials Polynomials ;
   
   typedef typename Traits< ScalarType >::KnotPoint        KnotPoint   ; 
   typedef typename Traits< ScalarType >::KnotPoints       KnotPoints  ; 

   typedef typename Traits< ScalarType >::PointDerivatives PointDerivatives ; 

   enum Properties
   {
      HAS_DERIVATIVE = 0,
      SPLINE_TYPE    = 3,
   };

   static Polynomials Construct( KnotPoints const& knots )
   {
      Assert( knots.size() > 1 ) ;

      typedef TridiagonalSys< ScalarType >::SysValues    SysValues ;
      typedef TridiagonalSys< ScalarType >::SysString    SysString ;
      typedef TridiagonalSys< ScalarType >::ValuesVector ValuesVector ;

      SysValues      sys      ( knots.size()) ;
      ValuesVector   momVect  ( knots.size()) ; // moments - third derivative 
      ValuesVector   bVect    ( knots.size()) ;

      for( unsigned i = 1 ; i < knots.size() - 1 ; ++ i )
      {
         ScalarType hi  = knots[ i ].param - knots[ i - 1 ].param ;
         ScalarType hi1 = knots[ i + 1 ].param - knots[ i ].param ;
         ScalarType him = ( hi + hi1 ) / 2 ;

         ScalarType alpha = hi  / ( 2 * him ) ;
         ScalarType betta = hi1 / ( 2 * him ) ;

         ScalarType ki1 = knots[ i + 1 ].value - knots[ i ].value ;
         ScalarType ki  = knots[ i ].value - knots[ i - 1 ].value ;
         ScalarType f = ( ki1 / hi1 - ki / hi ) / him ;

         sys   [ i ] = SysString( alpha, 2, betta ) ;
         bVect [ i ] = 3 * f ;                                                       
      }

      // condition for first and last point ( second derivative equals zero ) 
      sys   [ 0                ] = SysString( 0, 1, 0 ) ; 
      sys   [ knots.size() - 1 ] = sys[ 0 ] ;

      bVect [ 0                ] = 0 ;
      bVect [ knots.size() - 1 ] = 0 ;

      bool ret = TridiagonalSys< ScalarType >::SolveSys( sys, bVect, momVect ) ;

      // coefficients for cubic polynomials:
      Polynomials pols ( knots.size() - 1 ) ;
      
      for( unsigned i = 0, size = pols.size(); i < size ; ++ i )
      {
         pols[ i ].x0 = knots[ i     ].param ;
         pols[ i ].x1 = knots[ i + 1 ].param ;

         ScalarType hi = ( knots[ i + 1 ].param - knots[ i ].param ) ;

         Assert( cg::eq( hi, pols[ i ].x1 - pols[ i ].x0 )) ;

         pols[ i ].a = ( momVect[ i + 1 ] - momVect[ i ] ) / ( 6 * hi ) ;
         pols[ i ].b = momVect[ i ] / 2 ;
         pols[ i ].c = ( knots[ i + 1 ].value - knots[ i ].value ) / hi - hi * ( momVect[ i + 1 ] - momVect[ i ] ) / 6 - hi * momVect[ i ] / 2 ;
         pols[ i ].d = knots[ i ].value ;
      }

      // check :
//       for ( unsigned i = 0 ; i < pols.size() - 1; ++ i )
//       {
//          ScalarType val1 = pols[ i     ].Interpolate( pols[ i     ].x1 ) ;
//          ScalarType val2 = pols[ i + 1 ].Interpolate( pols[ i + 1 ].x0 ) ;
// 
//          ScalarType der1 = pols[ i     ].Derivative( pols[ i     ].x1 ) ;
//          ScalarType der2 = pols[ i + 1 ].Derivative( pols[ i + 1 ].x0 ) ;
// 
//          ScalarType der21 = pols[ i     ].SecondDerivative( pols[ i     ].x1 ) ;
//          ScalarType der22 = pols[ i + 1 ].SecondDerivative( pols[ i + 1 ].x0 ) ;
// 
//          Assert( cg::eq( val1,  val2 , ScalarType( 0.01 ))) ;  // HACK : problems with float precision  
//          Assert( cg::eq( der1,  der2 , ScalarType( 0.01 ))) ;
//          Assert( cg::eq( der21, der22, ScalarType( 0.01 ))) ;
//       }

      return pols ;
   }
};

//////////////////////////////////////////////////////////////////////////
// Catmull-Rom spline constructor 
//
// constructs cubic spline with first derivatives equal to (y[i+1]-y[i-1])/(x[i+1]-x[i-1])
// starts and ends with second derivative equal to zero 

template < class ScalarType >
struct CatmullRom
{
   typedef typename Traits< ScalarType >::CubicPolynomial  Polynomial   ;
   typedef typename Traits< ScalarType >::CubicPolynomials Polynomials  ;
   typedef typename Traits< ScalarType >::KnotPoint        KnotPoint    ; 
   typedef typename Traits< ScalarType >::KnotPoints       KnotPoints   ; 

   typedef typename Traits< ScalarType >::PointDerivatives PointDerivatives ; 

   enum Properties
   {
      HAS_DERIVATIVE = 0,
      SPLINE_TYPE    = 2,
   };

   static Polynomials Construct( KnotPoints const& knots )
   {
      size_t num = knots.size() ;

      Polynomials                pols( num - 1 ) ;
      std::vector< ScalarType >  drv ( num ) ; // derivatives 

      std::vector< ScalarType > dx( num - 1 ) ;
      std::vector< ScalarType > dy( num - 1 ) ;

      // intervals 
      for( size_t i = 0, size = dx.size(); i < size; ++i )
      {
         dx[i] = knots[i+1].param - knots[i].param ;
         dy[i] = knots[i+1].value - knots[i].value ;
      }

      // derivatives      
      drv.front() = 0 ; //  not 
      drv.back () = 0 ; //    used

      for( size_t i = 1, size = drv.size() - 1; i < size; ++i )
         drv.at(i) = ( knots[ i + 1 ].value - knots[ i - 1].value ) / ( knots[ i + 1 ].param - knots[ i - 1 ].param ) ;

      for( size_t i = 0, size = pols.size(); i < size; ++i )
      {
         pols.at( i ).x0 = knots.at(i    ).param ;
         pols.at( i ).x1 = knots.at(i + 1).param ;
      }

      // first and last cubic polynomials are calculated in a way to make second derivative equal to nil 
      Polynomial& first = pols.front() ;
      Polynomial& last  = pols.back () ;

      if ( pols.size() > 1 )
      {
         first.a = ( drv[1] * dx.front() - dy[0] ) / 2 ;
         first.b = 0 ;
         first.c = 3 * dy[0] / 2 - drv[1] * dx.front() / 2 ; 
         first.d = knots[0].value ;

         last .a = drv[num - 2] * dx.back() / 2 - dy.back() / 2 ;
         last .b = - 3 * ( drv[num - 2] * dx.back() - dy.back()) / 2 ;
         last .c = drv[num - 2] * dx.back() ;
         last .d = knots[num - 2].value ; 
      }
      else // only two points 
      {
         first.a = 0 ;
         first.b = 0 ;
         first.c = dy[0] ;
         first.d = knots[0].value ; 
      }

      // intermediate points 
      for( size_t i = 1, size = pols.size() - 1; i < size; ++i )
      {
         pols.at(i).a = ( drv.at(i) + drv.at(i + 1)) * dx.at(i) - 2 * dy.at(i) ;
         pols.at(i).b = 3 * dy.at(i) - ( 2 * drv.at(i) + drv.at(i + 1)) * dx.at(i) ;
         pols.at(i).c = drv.at(i) * dx.at(i);
         pols.at(i).d = knots.at(i).value ;
      }
      
      // scaling 
      for( size_t i = 0, size = pols.size(); i < size; ++i )
      {
         ScalarType dx2 = dx.at(i) * dx.at(i) ;
         ScalarType dx3 = dx2   * dx.at(i) ;

         pols.at(i).a /= dx3  ;
         pols.at(i).b /= dx2  ;
         pols.at(i).c /= dx.at(i);
      }
      
      // check :
//       for ( unsigned i = 0 ; i < pols.size() - 1; ++ i )
//       {
//          ScalarType val1 = pols.at(i    ).Interpolate( pols.at(i    ).x1 ) ;
//          ScalarType val2 = pols.at(i + 1).Interpolate( pols.at(i + 1).x0 ) ;
// 
//          ScalarType der1 = pols.at(i    ).Derivative( pols.at(i    ).x1 ) ;
//          ScalarType der2 = pols.at(i + 1).Derivative( pols.at(i + 1).x0 ) ;
// 
//          Assert( cg::eq( val1,  val2 , ScalarType( 0.01 ))) ; // HACK : problems with float precision  
//          Assert( cg::eq( der1,  der2 , ScalarType( 0.01 ))) ;
//       }

      return pols ;
   }
};

//////////////////////////////////////////////////////////////////////////
// BicubicHermite spline constructor 
//
// constructs cubic spline with specified first derivatives

template < class ScalarType >
struct BicubicHermite
{
   typedef typename Traits< ScalarType >::CubicPolynomial  Polynomial   ;
   typedef typename Traits< ScalarType >::CubicPolynomials Polynomials  ;
   typedef typename Traits< ScalarType >::KnotPoint        KnotPoint    ; 
   typedef typename Traits< ScalarType >::KnotPoints       KnotPoints   ; 

   typedef typename Traits< ScalarType >::PointDerivatives PointDerivatives ; 
   
   enum Properties
   {
      HAS_DERIVATIVE = 1,
      SPLINE_TYPE    = 1,
   };

   static Polynomials Construct( KnotPoints const& knots, PointDerivatives const& drv )
   {
      size_t num = knots.size() ;

      Polynomials               pols( num - 1 ) ;

      std::vector< ScalarType > dx( num - 1 ) ;
      std::vector< ScalarType > dy( num - 1 ) ;

      // intervals 
      for( size_t i = 0, size = dx.size(); i < size; ++i )
      {
         dx[i] = knots[i+1].param - knots[i].param ;
         dy[i] = knots[i+1].value - knots[i].value ;
      }

      // fill
      for( size_t i = 0, size = pols.size(); i < size; ++i )
      {
         pols.at( i ).x0 = knots.at(i    ).param ;
         pols.at( i ).x1 = knots.at(i + 1).param ;
      }

      // intermediate points 
      for( size_t i = 0, size = pols.size(); i < size; ++i )
      {
         pols.at(i).a = ( drv.at(i) + drv.at(i + 1)) * dx.at(i) - 2 * dy.at(i) ;
         pols.at(i).b = 3 * dy.at(i) - ( 2 * drv.at(i) + drv.at(i + 1)) * dx.at(i) ;
         pols.at(i).c = drv.at(i) * dx.at(i) ;
         pols.at(i).d = knots.at(i).value ;
      }
      
      // scaling 
      for( size_t i = 0, size = pols.size(); i < size; ++i )
      {
         ScalarType dx_inv = 0;
         if (!cg::eq_zero(dx.at(i)))
            dx_inv = 1 / dx.at(i);
         ScalarType dx2_inv = dx_inv * dx_inv;

         pols.at(i).c *= dx_inv ;
         pols.at(i).b *= dx2_inv ;
         pols.at(i).a *= dx2_inv * dx_inv  ;
      }
      
      // check :
//       for ( unsigned i = 0 ; i < pols.size() - 1; ++ i )
//       {
//          ScalarType val1 = pols.at(i    ).Interpolate( pols.at(i    ).x1 ) ;
//          ScalarType val2 = pols.at(i + 1).Interpolate( pols.at(i + 1).x0 ) ;
// 
//          ScalarType der1 = pols.at(i    ).Derivative( pols.at(i    ).x1 ) ;
//          ScalarType der2 = pols.at(i + 1).Derivative( pols.at(i + 1).x0 ) ;
// 
//          Assert( cg::eq( val1,  val2 , ScalarType( 0.01 ))) ; // HACK : problems with float precision  
//          Assert( cg::eq( der1,  der2 , ScalarType( 0.01 ))) ;
//       }

      return pols ;
   }
};

//////////////////////////////////////////////////////////////////////////
// linear spline constructor 
//
// very simple!
//

template < class ScalarType >
struct Linear
{
   typedef typename Traits< ScalarType >::LinearPolynomial  Polynomial   ;
   typedef typename Traits< ScalarType >::LinearPolynomials Polynomials  ;
   typedef typename Traits< ScalarType >::KnotPoint         KnotPoint    ; 
   typedef typename Traits< ScalarType >::KnotPoints        KnotPoints   ; 

   typedef typename Traits< ScalarType >::PointDerivatives PointDerivatives ; 

   enum Properties
   {
      HAS_DERIVATIVE = 0,
      SPLINE_TYPE    = 0,
   };

   static Polynomials Construct( KnotPoints const& knots )
   {
      Polynomials pols( knots.size() - 1 ) ;
      
      for( size_t i = 0, size = pols.size(); i < size; ++i )
      {
         pols.at( i ).x0 = knots.at(i    ).param ;
         pols.at( i ).x1 = knots.at(i + 1).param ;

         ScalarType dx = pols[ i ].x1 - pols[ i ].x0 ;
         Assert( !cg::eq_zero( dx )) ;

         pols[ i ].a = ( knots[i + 1].value - knots[i].value ) / dx ;
         pols[ i ].b = knots[i].value ;
      }
      
      return pols ;
   }
};

} // Constructors 

template < class ScalarType, template < class > class Polynomial >
   inline bool operator < ( ScalarType x, Polynomial< ScalarType > const& cp ) 
{                                                       
   return x < cp.x0 ;
}

//////////////////////////////////////////////////////////////////////////
// 
// spline function y(x)  
// all requests are mode in log(n) 
//

template < class ScalarType, template< class > class SplineConstructor > 
  struct Spline1D
{
private:
   typedef 
      SplineConstructor< ScalarType >
      Constructor ;

   typedef 
      typename Constructor::Polynomials  
      Polynomials ;

public:
   typedef 
      typename Constructor::KnotPoint
      KnotPoint ;

   typedef 
      typename Constructor::KnotPoints
      KnotPoints ;

   typedef 
      typename Constructor::PointDerivatives
      PointDerivatives ;

//    enum Properties {
//       HAS_DERIVATIVE = Constructor::HAS_DERIVATIVE,
//    };
   typedef 
      typename Constructor 
      Properties ;

   typedef 
      typename Spline1D::ScalarType
      ScalarType ;

public:
   Spline1D()
   {
   }

   ~Spline1D()
   {
      knots_.clear() ;
      pols_. clear() ;
   }

   Spline1D( KnotPoints const& knots )
   {
      Init( knots ) ;
   }

   void Init( KnotPoints const& knots )
   {
      knots_ = knots ;
      pols_  = Constructor::Construct( knots ) ;
   }

   void Init( KnotPoints const& knots, PointDerivatives const& derivatives )
   {
      knots_ = knots ;
      pols_  = Constructor::Construct( knots, derivatives ) ;
   }

   int FindSegment( ScalarType x ) const 
   {
      Assert( ge( x, pols_.front().x0) && le( x, pols_.back().x1 )) ; 
      
      // binary search 
      int start = 0, stop = pols_.size() - 1 ;
      int mid = ( stop + start ) / 2 ; 

      while ( stop > start )
      {
         if( pols_[ mid ].InBound( x ))
            return mid ;
         else if ( x < pols_[ mid ] )
            stop = mid - 1 ;
         else 
            start = mid + 1 ;

         mid = ( stop + start ) / 2 ;
      }

      Assert( pols_[ mid ].InBound( x )) ;
      return mid ;
   }

   ScalarType Interpolate( ScalarType x ) const 
   {
      Assert( pols_.size() > 0 ) ; 
      return pols_[ FindSegment( x )].Interpolate( x ) ; 
   }

   ScalarType Derivative( ScalarType x ) const
   {
      Assert( pols_.size() > 0 ) ;
      return pols_[ FindSegment( x )].Derivative( x ) ;
   }

   ScalarType SecondDerivative( ScalarType x ) const
   {
      Assert( pols_.size() > 0 ) ;
      return pols_[ FindSegment( x )].SecondDerivative( x ) ;
   }

   ScalarType ThirdDerivative( ScalarType x ) const
   {
      Assert( pols_.size() > 0 ) ;
      return pols_[ FindSegment( x )].ThirdDerivative( x ) ;
   }

   KnotPoints const & knots() const
   {
      return knots_;
   }

   bool operator == (Spline1D< ScalarType, SplineConstructor > const & that) const
   {
      return (knots_ == that.knots_) && (pols_ == that.pols_);
   }

private:
   KnotPoints  knots_ ;
   Polynomials pols_  ;
};

template < class ScalarType, template< class > class SplineConstructor > 
bool operator != (   Spline1D< ScalarType, SplineConstructor > const & a, 
                     Spline1D< ScalarType, SplineConstructor > const & b   )
{
   return !(a == b);
}

} // namespace Splines 
} // namespace cg 
