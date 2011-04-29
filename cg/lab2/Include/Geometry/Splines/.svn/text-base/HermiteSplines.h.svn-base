#pragma once

#include "common\Assert.h"

#include "geometry\primitives\point.h"
#include "Streams\structured_streams.h"

#include "Streams\aux_traits.h"

#pragma pack(push, 1)

// to be redesigned
namespace cg
{
    namespace angles
    {
      namespace details
      {
         inline double angle_12 (point_t<double,2> const &a, point_t<double,2> const &b)
         {
            double nrm_a = norm_sqr(a);
            double nrm_b = norm_sqr(b);
            return 
               nrm_a == 0 || nrm_b == 0 ? 0 
               : acos(a*b / cg::sqrt(nrm_a * nrm_b));
         }
      }

      __declspec (deprecated)
      inline double angle_12 (point_t<double,2> const &a, point_t<double,2> const &b)
      {
         return details::angle_12(a,b) ;
      }

      __declspec (deprecated)
      inline double angle_41 (point_t<double,2> const &a, point_t<double,2> const &b)
      {
         double res = details::angle_12(a, b);
         if (res > pi / 2)
         res = res - pi;
         return res;
      }
    }
}

namespace cg
{

    template< class T > struct SplineTypeTraits
    {
        typedef T diff_type;      
    };

    inline void ModIndex( int & where, int size )
    {
        Assert( size > 0 );
        if ( where < 0 )
        {
            where += size;
            while ( where < 0 )
                where += size;
        } 
        else if (where >= size )
        {
            where -= size;
            while (where >= size )
                where -= size;
        }
    }

    template<class T> struct CubicPolynom
    {
        typedef T point;
        typedef typename SplineTypeTraits< T >::diff_type          diff1_type;
        typedef typename SplineTypeTraits< diff1_type >::diff_type diff2_type;
        typedef typename SplineTypeTraits< diff2_type >::diff_type diff3_type;

        point      D;
        diff1_type C;
        diff2_type B;
        diff3_type A;

        point operator ()( double t ) const
        {
           return (((A * t) + B) * t + C) * t + D;
        }
        diff1_type Derivative( double t ) const 
        {
            return ((A * (3.*t)) + B * 2.) * t + C;
        }
        diff2_type SecondDerivative( double t ) const
        {
            return (A * (6.*t)) + B * 2.;
        }
        diff3_type ThirdDerivative( double t ) const
        {
            return A * 6.;
        }

        point Direction( double t ) const 
        {
            point dir = Derivative ( t ) ; 
            double l = norm ( dir ) ; 
            if ( l == 0. ) 
            {
                dir = SecondDerivative ( t ) ; 
                l = norm ( dir ) ; 
                if ( l == 0. ) 
                {
                    dir = ThirdDerivative ( t ) ; 
                    l = norm ( dir ) ; 
                    if ( l == 0. ) 
                        return point() ; 
                }
            } 
            return dir / l ; 
        } 

        point Binormal( double t ) const 
        {
            // to do: what would be binormal if first derivative (or second) equal to zero ?
            point bn = Derivative ( t ) ^ SecondDerivative ( t ) ; 
            return normalized_safe(bn) ; 
        } 

        double Curvature( double t ) const 
        {
           point fd = Derivative ( t ) ; 
           double nfd = norm_sqr(fd) ; 

           if ( eq_zero(nfd) ) 
              return 0 ; 

           fd /= cg::sqrt(nfd) ; 

           point sd = SecondDerivative ( t ) ; 

           double k = norm_sqr(sd) - (fd*sd)*(fd*sd) ; 
           
           if ( eq_zero( k )) 
              return 0 ; 

           return cg::sqrt( k ) / nfd ;
        } 

        double Radius( double t ) const 
        {
            // to do: code will raise exception, if radius is too big or too small 
            return 1. / Curvature ( t ) ; 
        } 
    };

    template<class Stream, class T>
      void write(Stream &stream, CubicPolynom<T> const &cp)
    {
      write(stream, cp.D);
      write(stream, cp.C);
      write(stream, cp.B);
      write(stream, cp.A);
    }
    template<class Stream, class T>
      void read(Stream &stream, CubicPolynom<T> &cp)
    {
      read(stream, cp.D);
      read(stream, cp.C);
      read(stream, cp.B);
      read(stream, cp.A);
    }

    template< class T, class Traits = streams::default_traits > 
    struct HermiteSpline
    {
        typedef T point;
        typedef typename SplineTypeTraits< T >::diff_type          diff1_type;
        typedef typename SplineTypeTraits< diff1_type >::diff_type diff2_type;
        typedef typename SplineTypeTraits< diff2_type >::diff_type diff3_type;

        typedef typename Traits::vector< CubicPolynom<point> >::type vector_type;

        point Interpolate( double t ) const // t in [0; pols.size()];
        {
            int n = GetIdx( t );
            return pols[n]( t );
        }

        diff1_type Derivative( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Derivative( t );
        }

        diff2_type SecondDerivative( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].SecondDerivative( t );
        }

        diff3_type ThirdDerivative( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].ThirdDerivative( t );
        }

        point Direction( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Direction( t );
        }

        point Binormal( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Binormal( t );
        }

        double Curvature( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Curvature( t );
        }

        double Radius( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Radius(t);
        }

        point operator() ( double t ) const
        {
            return Interpolate( t );
        }

        int GetIdx( double & t ) const
        {
            Assert( pols.size() > 0 );
            int size = static_cast< int >( pols.size() );
            int n = floor( t );
            if ( n < 0 )
            {
                n = 0;
                t = 0;
            }
            else if ( n >= size ) 
            {
                n = size - 1;
                t = 1;
            }
            else
            {
                t -= n;
            }

            return n;
        }

    // Serialization stuff
    public:
        template<class Stream>
           void serialize(Stream &stream) const
        {
           write(stream, pols);
        }

        template<class Stream>
           void deserialize(Stream &stream) 
        {
           read(stream, pols);
        }

        vector_type pols;  
    };

    template<class Stream, class T, class Traits >
      void write(Stream &stream, HermiteSpline<T, Traits> const &spline)
    {
       spline.serialize(stream);
    }

    template<class Stream, class T, class Traits >
      void read(Stream &stream, HermiteSpline<T, Traits> &spline)
    {
       spline.deserialize(stream);
    }

    template<class T> struct ClosedHermiteSpline
    {
        typedef T point;
        typedef typename SplineTypeTraits< T >::diff_type          diff1_type;
        typedef typename SplineTypeTraits< diff1_type >::diff_type diff2_type;
        typedef typename SplineTypeTraits< diff2_type >::diff_type diff3_type;

        point Interpolate( double t ) const // t in [0; pols.size()];
        {
            Assert(pols.size() > 0);
            int n = cg::floor( t );
            t -= n;

            int size = pols.size( );
            Assert( size > 0 );
            if ( n < 0 )
            {
                n += size;
                while ( n < 0 )
                    n += size;
            } 
            else if (n >= size )
            {
                n -= size;
                while (n >= size )
                    n -= size;
            }

            return pols[n](t);      
        }

        diff1_type Derivative( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Derivative(t);
        }

        diff2_type SecondDerivative( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].SecondDerivative(t);
        }

        diff3_type ThirdDerivative( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].ThirdDerivative(t);
        }

        point Direction( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Direction(t);
        }

        point Binormal( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Binormal(t);
        }

        double Curvature( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Curvature(t);
        }

        double Radius( double t ) const
        {
            int n = GetIdx( t );
            return pols[n].Radius( t );
        }

        point operator() ( double t ) const
        {
            return Interpolate( t );
        }

        int GetIdx( double& t ) const
        {
            Assert( pols.size() > 0 );
            int n = floor( t );
            t -= n;

            ModIndex( n, pols.size() );

            return n;
        }

        std::vector< CubicPolynom< point > > pols;  
    };

    template<class T, class Traits = streams::default_traits > 
      class HermiteSplineManager
    {
        typedef T point;
        typedef typename SplineTypeTraits< T >::diff_type          diff1_type;
        typedef typename SplineTypeTraits< diff1_type >::diff_type diff2_type;
        typedef typename SplineTypeTraits< diff2_type >::diff_type diff3_type;

    public:
        HermiteSplineManager( )
            : max_edge_len ( 1000.0 )
            , min_edge_len ( 10.0 )
            , min_angle    ( 3.141592659 )
            , defaultFlatness ( 0.5 )

        {}

        template < class FwdIter >
          HermiteSplineManager( FwdIter p, FwdIter q, double flatness = 0.5 )
            : points ( p, q )
            
            , max_edge_len ( 1000.0 )
            , min_edge_len ( 10.0 )
            , min_angle    ( 3.141592659 )

            , derivatives ( std::distance( p, q ) )
            , defaultFlatness ( flatness )
        {    
            size_t n = std::distance( p, q );
            if (n > 0)
            {
                spline.pols.resize(n - 1);
                for ( unsigned i = 0; i < n;  ++i )
                    SetDefaultDirection( i );
                for ( unsigned j = 0; j < n - 1; ++j )
                    ReCalcCoefs( j );
            }
        }

        void Assign( const point * p, const point * q )
        {
            *this = HermiteSplineManager( p, q );
        }

        void Clear( )
        {
            points.clear( );
            derivatives.clear( );
            spline.pols.clear( );
        }

        int GetNumPoints ( ) const 
        { 
            return static_cast<int>( points.size( ) );
        }

        int GetNumSegments( ) const
        {
           return spline.pols.size( );            
        }

        double GetDefaultFlatness() const
        {
           return defaultFlatness;
        }

        const point&  GetPoint ( int nPoint )   const
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() );
            return points[nPoint];      
        }

        diff1_type GetLeftDirection ( int nPoint ) const
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() );  
            return derivatives[nPoint].l;      
        }

        diff1_type GetRightDirection( int nPoint ) const
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() );  
            return derivatives[nPoint].r;      
        }
  
        void AddPoint   ( int where, const point & p )
        {  
            Assert( 0 <= where && where <= GetNumPoints() ); 
            points.insert( points.begin() + where, p );
            derivatives.insert( derivatives.begin() + where, LRDerivatives() );
            if ( GetNumPoints() > 1 ) 
               spline.pols.insert( spline.pols.begin() + ((where > 0) ? (where - 1) : 0), CubicPolynom<point>() );
            
            if ( where > 0 )
               SetDefaultDirection( where - 1 );
            
            SetDefaultDirection( where );
            
            if ( size_t(where) + 1 < points.size() )
               SetDefaultDirection( where + 1 );

            int recalc_start = where - 2; if ( recalc_start < 0 )                     recalc_start = 0;
            int recalc_end   = where + 1; if ( recalc_end > (int)points.size( ) - 2 ) recalc_end   = points.size( ) - 2;
            for ( int i = recalc_start; i <= recalc_end; ++i )
                ReCalcCoefs( i );  
        }
  
        void SetPoint   ( int where, const point & p )
        {
            Assert( 0 <= where && where < GetNumPoints() );
            points[where] =  p;
            SetDefaultDirection( where );
            SetDefaultDirection( where - 1 );
            SetDefaultDirection( where + 1);
            int recalc_start = where - 2; if ( recalc_start < 0 )                       recalc_start = 0;
            int recalc_end   = where + 1; if ( recalc_end > int( points.size( ) ) - 2 ) recalc_end   = points.size( ) - 2;
            for ( int i = recalc_start; i <= recalc_end; ++i )
                ReCalcCoefs( i );      
        }
  
        void PushBack   ( const point & p )
        {
            points.push_back( p );
            derivatives.push_back( LRDerivatives() );
            if (GetNumPoints() > 1)
                spline.pols.push_back( CubicPolynom<point>() );
            SetDefaultDirection( GetNumPoints() - 2 );
            int recalc_start = GetNumPoints() - 3; if ( recalc_start < 0 ) recalc_start = 0;
            for ( int i = recalc_start; i < GetNumPoints() - 1; ++i )
                ReCalcCoefs( i );        
        }

        void RemovePoint( int where )
        {
            Assert( 0 <= where && where < GetNumPoints() );
            points.erase( points.begin( ) + where );
            derivatives.erase( derivatives.begin() + where );
            if (GetNumPoints() > 0)
            {
                spline.pols.erase( spline.pols.begin( ) + 
                    ( where == GetNumPoints() ? where - 1 : where ) );
            }
            SetDefaultDirection( where );
            SetDefaultDirection( where - 1 );
            int recalc_start = where - 2; if ( recalc_start < 0 )                     recalc_start = 0;
            int recalc_end   = where + 1; if ( recalc_end > (int)points.size( ) - 2 ) recalc_end   = points.size( ) - 2;
            for ( int i = recalc_start; i <= recalc_end; ++i )
                ReCalcCoefs( i );      
        }
  
        point Interpolate( double t ) const  // t in [0; nPoints]
        {
            Assert( GetNumPoints() > 0 );
            if (GetNumPoints() == 1)
                return points[0];
            Assert( static_cast< int >( spline.pols.size() ) == GetNumPoints() - 1 );
            return spline(t);      
        }

        point operator ()( double t ) const { return Interpolate( t ); }

        void SetDirection( int nPoint, const diff1_type & p)
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() );
            derivatives[nPoint].l = derivatives[nPoint].r = p;
            derivatives[nPoint].rdef = derivatives[nPoint].ldef = false ; 
            
            if ( nPoint > 0 )
                ReCalcCoefs( nPoint - 1 );
            if ( nPoint < GetNumPoints() - 1 )
                ReCalcCoefs( nPoint );      
        }

        void SetLeftDirection( int nPoint, const diff1_type & p)
        {
            Assert( 0 < nPoint && nPoint < GetNumPoints() );
            derivatives[nPoint].l = p;
            derivatives[nPoint].ldef = false ;            
            ReCalcCoefs( nPoint - 1 );
        }

        void SetRightDirection( int nPoint, const diff1_type & p)
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() - 1 );
            derivatives[nPoint].r = p;
            derivatives[nPoint].rdef = false ;
            ReCalcCoefs( nPoint );
        }


        void SetDefaultFlatness( double flatness )
        {
            defaultFlatness = flatness;
        }

        void SetDefaultDirection( int nPoint )
        {
            if ( nPoint == 0 || (nPoint == GetNumPoints() - 1 && !points.empty() ) )
                derivatives[ nPoint ] = LRDerivatives();
            else if ( nPoint > 0 && nPoint < GetNumPoints() - 1 )
            {
                diff1_type dir = points[ nPoint + 1 ] - points[ nPoint - 1 ]; //!!!
                double dist  = cg::distance( points[ nPoint + 1 ], points[ nPoint - 1 ] );
                double ldist = cg::distance( points[ nPoint ], points[ nPoint - 1 ] );
                double rdist = cg::distance( points[ nPoint ], points[ nPoint + 1 ] );

//                double max_dist = cg::max( ldist, rdist );
//                if (max_dist < dist)
//                    dir = dir * (max_dist / dist );
//                derivatives[ nPoint ].r = derivatives[ nPoint ].l = dir * defaultFlatness; 

                double rnorm = ( dist > 0 ) ? rdist / dist : 0 ;
                double lnorm = ( dist > 0 ) ? ldist / dist : 0 ;

                derivatives[ nPoint ].r = dir * rnorm * defaultFlatness ;
                derivatives[ nPoint ].l = dir * lnorm * defaultFlatness ;
                derivatives[ nPoint ].rdef = derivatives[ nPoint ].ldef = false ; 
            }
        }
  
  
        /////////////////////////////////////
        // spline division support

        mutable double min_edge_len;
        mutable double max_edge_len;
        mutable double min_angle;

        void SetDivisionParams( double _min_edge_len, double _max_edge_len, double _min_angle ) const
        {
            min_edge_len = _min_edge_len;
            max_edge_len = _max_edge_len;
            min_angle    = _min_angle * 3.141592659 / 180.0;
        }

        diff1_type Derivative( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.Derivative(t);
        }

        diff2_type SecondDerivative( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.SecondDerivative(t);
        }

        diff3_type ThirdDerivative( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.ThirdDerivative(t);
        }

        point Direction( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.Direction(t);
        }

        point Binormal( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.Binormal(t);
        }

        double Curvature( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.Curvature(t);
        }

        double Radius( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints()-1 );
            return spline.Radius(t);
        }

        void GetDivision( const point & left, const point & right,
                          const diff1_type & lder, const diff1_type & rder,
                          std::vector<point> &pDiv,
                          double lt, double rt, int iter_count ) const
        {
            if (iter_count > 100)
                return;

            point delta = right - left;
            double len = cg::norm(delta);

            if (len < max_edge_len)
            {
                if (len <= min_edge_len)
                    return;

                double langle = cg::abs( cg::angles::angle_41(  delta,  lder ) );
                double rangle = cg::abs( cg::angles::angle_41( -delta, -rder ) );
                double max_angle = cg::max(langle, rangle);

                if (max_angle < min_angle)
                    return;
            }

            double mt    = (lt + rt) / 2;
            point      middle = Interpolate( mt );
            diff1_type mder   = Derivative ( mt );

            GetDivision( left,   middle, lder, mder, pDiv, lt, mt, iter_count+1 );
            pDiv.push_back( middle );
            GetDivision( middle, right,  mder, rder, pDiv, mt, rt, iter_count+1 );
        }

        void ReCalcDivision( std::vector<point> &division, bool add_first_point=false ) const
        {
            for (int i = 0; i < GetNumPoints()-1; ++i)
            {
                if (i>0 || add_first_point)
                    division.push_back( points[i] ); 

                int i1 = i+1;

                GetDivision( points     [i],   points     [i1],
                    derivatives[i].r, derivatives[i1].l,
                    division,
                    i,                i + 1,           1 );
            }
            division.push_back( points[i] ); 
        }

        void CalcLengths( std::vector<double> &lengths ) const
        {
            for (int i = 0; i < GetNumPoints()-1; ++i)
            {
                double len = CalcSplineLen( points     [i],   points     [i+1],
                    derivatives[i].r, derivatives[i+1].l,
                    i,                i+1 );
                lengths.push_back(len);
            }
        }
      
    // Serialization stuff
    public:
      template<class Stream, class TempStream>
         void serialize(streams::structured_ostream<Stream,TempStream> &stream) const
      {
         write(stream, min_edge_len);
         write(stream, max_edge_len);
         write(stream, min_angle);

         write(stream, points);
         
         stream.push();
         write( stream, derivatives.size( ) );
         for ( size_t i = 0; i != derivatives.size(); ++i )
            derivatives[i].serialize( stream );
         stream.pop();

         write(stream, spline);
         write(stream, defaultFlatness);
      }

      template<class Stream>
         void serialize(Stream &stream) const
      {
         write(stream, min_edge_len);
         write(stream, max_edge_len);
         write(stream, min_angle);

         write(stream, points);

         write( stream, derivatives );

         write(stream, spline);
         write(stream, defaultFlatness);
      }
      template<class Stream>
         void deserialize(Stream &stream) 
      {
         read(stream, min_edge_len);
         read(stream, max_edge_len);
         read(stream, min_angle);

         read(stream, points);

         read( stream, derivatives );

         read(stream, spline);
         read(stream, defaultFlatness);
      }

    private:
        void ReCalcCoefs( int where )
        {
            Assert(0 <= where && where <= GetNumPoints() - 1);
            diff1_type delta = points[where + 1] - points[where];
            if ( ! derivatives[where].rdef && ! derivatives[where + 1].ldef ) 
            {
                spline.pols[where].D = points[where];
                spline.pols[where].C = derivatives[where].r;
                spline.pols[where].B = ( delta  - derivatives[where].r ) * 2. + ( delta - derivatives[where + 1].l );
                spline.pols[where].A = ( derivatives[where + 1].l - delta ) - ( delta - derivatives[where].r );
            }
            else if ( derivatives[where].rdef && derivatives[where + 1].ldef )     
            {
                spline.pols[where].D = points[where];
                spline.pols[where].C = delta;
                spline.pols[where].B = diff2_type() ; 
                spline.pols[where].A = diff3_type() ; 
            }
            else if ( ! derivatives[where].rdef && derivatives[where + 1].ldef )     
            {
                spline.pols[where].D = points[where];
                spline.pols[where].C = derivatives[where].r;
                spline.pols[where].B = ( delta - derivatives[where].r ) * 1.5 ;
                spline.pols[where].A = (delta - derivatives[where].r) - spline.pols[where].B ; 
            }    
            else if ( derivatives[where].rdef && ! derivatives[where + 1].ldef )     
            {
                spline.pols[where].D = points[where];
                spline.pols[where].B = diff2_type() ; 
                spline.pols[where].A = ((derivatives[where + 1].l - delta) - spline.pols[where].B) * 0.5 ;
                spline.pols[where].C = delta - spline.pols[where].A - spline.pols[where].B  ;
            }
        }

        struct LRDerivatives 
        {
            bool ldef, rdef; 
            point l, r;
            
            LRDerivatives () 
               : ldef(true)
               , rdef(true) 
            {}

            template< class Stream >
            void serialize(Stream & stream) const
            {
               write(stream, ldef);
               write(stream, rdef);
               write(stream, l);
               write(stream, r);
            }

            template <class Stream>
               friend void write(Stream & s, LRDerivatives const &d)
               {
                  d.serialize(s);
               }

            template< class Stream >
            void deserialize(Stream & stream) 
            {
               read(stream, ldef);
               read(stream, rdef);
               read(stream, l);
               read(stream, r);
            }

            template <class Stream>
               friend void read(Stream &s, LRDerivatives &d)
               {
                  d.deserialize(s);
               }
        };

        typedef typename Traits::vector<point>::type         points_vector_type;
        typedef typename Traits::vector<LRDerivatives>::type derivatives_vector_type;

        points_vector_type       points;
        derivatives_vector_type  derivatives;  
        HermiteSpline<T, Traits> spline;

        double defaultFlatness;
    };

    template< class Stream, class T, class Traits >
       void write(Stream &stream, HermiteSplineManager<T, Traits> const &manager)
    {
       manager.serialize(stream);
    }

    template< class Stream, class T, class Traits >
       void read(Stream &stream, HermiteSplineManager<T, Traits> &manager)
    {
       manager.deserialize(stream);
    }

    template<class T> class ClosedHermiteSplineManager
    {
        typedef T point;
        typedef typename SplineTypeTraits< T >::diff_type          diff1_type;
        typedef typename SplineTypeTraits< diff1_type >::diff_type diff2_type;
        typedef typename SplineTypeTraits< diff2_type >::diff_type diff3_type;

    public:

        template < class FwdIter >
          ClosedHermiteSplineManager( FwdIter p, FwdIter q )
            : points( p, q )
            , derivatives ( std::distance( p, q ) )
        { 
            size_t n = std::distance( p, q );
            spline.pols.resize(n);
            for ( int i = 0; i != n;  ++i )
                SetDefaultDirection( i );
            for ( int j = 0; j != n; ++j )
                ReCalcCoefs( j );
        }

        int GetNumPoints( ) const
        {
            return points.size( );
        }

        int GetNumSegments( ) const
        {
           return spline.pols.size( );
        }

        const point&  GetPoint( int nPoint )   const
        {
            return points.at( nPoint );
        }

        diff1_type Derivative( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints() );
            return spline.Derivative(t);
        }

        diff2_type SecondDerivative( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints() );
            return spline.SecondDerivative(t);
        }

        diff3_type ThirdDerivative( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints() );
            return spline.ThirdDerivative(t);
        }

        point Direction( double t ) const
        {
            Assert( spline.pols.size() == GetNumPoints() );
            return spline.Direction(t);
        }

        diff1_type GetLeftDirection ( int nPoint ) const
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() );  
            return derivatives[nPoint].l;
        }

        diff1_type GetRightDirection( int nPoint ) const
        {
            Assert( 0 <= nPoint && nPoint < GetNumPoints() );  
            return derivatives[nPoint].r;      
        }
  
        void AddPoint   ( int where, const point & p )
        {
            ModIndex( where );
            points.insert( &points[where], p );
            derivatives.insert( &derivatives[where], LRDerivatives() );
            spline.pols.insert( &spline.pols[where], CubicPolynom<point>() );
            for ( int i = where - 2; i <= where + 1; ++i )
                ReCalcCoefs( i );      
        }

        void SetPoint   ( int where, const point & p)
        {
            ModIndex( where );
            points[where] =  p;
            SetDefaultDirection( where );
            SetDefaultDirection( where - 1 );
            SetDefaultDirection( where + 1 );
            for ( int i = where - 2; i <= where + 1; ++i )
                ReCalcCoefs( i );
        }

        void PushBack   ( const point & p )
        {
            points.push_back( p );
            derivatives.push_back( LRDerivatives() );
            spline.pols.push_back( CubicPolynom<point>() );
            SetDefaultDirection( GetNumPoints() - 2 );
            SetDefaultDirection( GetNumPoints() - 1 );
            SetDefaultDirection( 0 );
            for (int i = GetNumPoints() - 3; i <= GetNumPoints(); ++i)
                ReCalcCoefs( i );
        }

        void RemovePoint( int where )
        {
            ModIndex( where );
            points.erase( &points[where] );
            derivatives.erase( &derivatives[where] );
            spline.pols.erase( &spline.pols[where] );
            if (GetNumPoints() > 0)
            {
                SetDefaultDirection( where );
                SetDefaultDirection( where - 1 );
                for ( int i = where - 1; i <= where + 1; ++i )
                    ReCalcCoefs( i );
            }      
        }

        point Interpolate( double t ) const  // t in [0; nPoints]
        {
            Assert( spline.pols.size() == GetNumPoints() );
            return spline(t);
        }

        point operator ()( double t ) const { return Interpolate( t ); }

        void SetDirection( int nPoint, const diff1_type & p)
        {
            ModIndex( nPoint );
            derivatives[nPoint].l = derivatives[nPoint].r = p;
            ReCalcCoefs( nPoint - 1);
            ReCalcCoefs( nPoint );      
        }

        void SetLeftDirection( int nPoint, const diff1_type & p)
        {
            ModIndex( nPoint );
            derivatives[nPoint].l = p;
            ReCalcCoefs( nPoint - 1 );
        }

        void SetRightDirection( int nPoint, const diff1_type & p)
        {
            ModIndex( nPoint );
            derivatives[nPoint].r = p;
            ReCalcCoefs( nPoint );
        }

        void SetDefaultDirection( int nPoint )
        {
            ModIndex( nPoint );
            diff1_type dir = points[cg::next(nPoint, GetNumPoints())] - points[cg::prev(nPoint, GetNumPoints())];
            double norma1 = cg::distance( points[cg::next(nPoint, GetNumPoints())], points[cg::prev(nPoint, GetNumPoints())] );
            double norma2 = cg::distance(points[nPoint], points[cg::prev(nPoint, GetNumPoints())]);
            double norma3 = cg::distance(points[nPoint], points[cg::next(nPoint, GetNumPoints())]);
            if (norma2 > norma3)
                norma2 = norma3;
            if (norma2 < norma1)
                dir = dir / norma1 * norma2;
            derivatives[nPoint].r = derivatives[nPoint].l = 0.5 * dir;
        }

    private:
        void ReCalcCoefs( int where )
        {
            ModIndex( where );
            diff1_type delta = points[cg::next(where, GetNumPoints())] - points[where];
            spline.pols[where].A = ( derivatives[cg::next(where, GetNumPoints())].l - delta ) - ( delta -  derivatives[where].r );
            spline.pols[where].B = 2 * ( delta - derivatives[where].r ) + ( delta - derivatives[cg::next(where, GetNumPoints())].l );
            spline.pols[where].C = derivatives[where].r;
            spline.pols[where].D = points[where];
        }

        void ModIndex( int & index ) const
        {
            int size = GetNumPoints( );
            if (size==0)
                return;
            // Assert( size > 0 );
            if ( index < 0 )
            {
                index += size;
                while ( index < 0 )
                    index += size;
            } 
            else if (index >= size )
            {
                index -= size;
                while (index >= size )
                    index -= size;
            }
        }

        int NextIndex( int   index ) const
        {
            return cg::next( index, GetNumPoints() );      
        }

        int PrevIndex( int   index ) const
        {
            return cg::prev( index, GetNumPoints() );      
        }

        struct LRDerivatives 
        {
            point l, r;
        };

        std::vector<point> points;
        std::vector<LRDerivatives> derivatives;  
        ClosedHermiteSpline<T> spline;
    };

    template <class T> class HermiteSplineArcLenDerivative
    {
        typedef T point;
        typedef typename SplineTypeTraits< T >::diff_type          diff1_type;
        typedef typename SplineTypeTraits< diff1_type >::diff_type diff2_type;
        typedef typename SplineTypeTraits< diff2_type >::diff_type diff3_type;

    public:
        HermiteSplineArcLenDerivative( HermiteSplineManager<T> * spline )
            : spline_ ( spline )
        {}

        double operator () ( double t )
        {
            return norm( spline_->Derivative( t ) );
        }

        double GetMax() const
        {
            double maxV = norm( spline_->Derivative( 0 ) );

            for ( int i = 0; i < spline_->GetNumPoints(); i++ )
            {
                diff1_type V1 = spline_->Derivative( i );
                diff2_type D1 = spline_->SecondDerivative( i );
                diff3_type D2 = spline_->ThirdDerivative( i ) * .5f + D1;

                double nV1 = norm( V1 );
                double nV2 = norm( V1 + D1 );
                double nV3 = norm( V1 + D2 );

                double polMaxV = max( max( nV1, nV2 ), nV3 );

                maxV = max( maxV, polMaxV );
            }

            return maxV;
        }

    private:
        HermiteSplineManager< point > * const spline_;
    };

    template <typename T> class HermiteSplineF
    {
    public:
        typedef T point;
    public:
        void   clear()                        {spline.Clear();}
        size_t size()                   const {return spline.GetNumPoints();}
        point  operator [] (size_t idx) const {return spline.GetPoint(idx);}
        point  dir         (size_t idx) const {return spline.GetLeftDirection(idx);}
        double operator () (double x)   const 
        {
            if (x < (*this)[0].x)
                return (*this)[0].y;
            else if (x > (*this)[size() - 1].x)
                return (*this)[size() - 1].y;

            // Interpolate value
            double t0 = 0, t1 = spline.GetNumPoints() - 1;

            while (cg::abs(t1 - t0) > 0.001) 
            {
                double _x = spline.Interpolate((t0 + t1) / 2).x;
                if (_x < x) t0 = (t0 + t1) / 2;
                else        t1 = (t0 + t1) / 2;
            }

            return spline.Interpolate((t0 + t1) / 2).y;
        }

        void erase(int ind) 
        {
            push();
            pnts.erase(pnts.begin() + ind);
            pop();
        }

        void set_direction(int ind, const point &d)
        {
            push();
            pnts[ind].second = d;
            pop();
        }

        void set_point(int ind, const point &p)
        {
            push();
            pnts[ind].first = p;
            pop();
            return;
        }

        void push_back(const point& p, const point &d = point(1, 1))
        {
            push();
            pnts.push_back(std::make_pair(p, d));
            pop();
        }

    private:
        void push()
        {
            pnts.clear();
            for (size_t i = 0; i < size(); i++)
                pnts.push_back(std::make_pair((*this)[i], dir(i)));

            return;
        }

        void pop()
        {
            std::sort(pnts.begin(), pnts.end(), cmp_pnts);
            for (size_t i = 0; i < pnts.size(); i++)
            {
                make_max(pnts[i].second.x, 0.);
            }
            std::unique(pnts.begin(), pnts.end(), equal);

            spline.Clear();
            for (size_t i = 0; i < pnts.size(); i++)
            {
                spline.PushBack(pnts[i].first);
            }
            for (size_t i = 0; i < pnts.size(); i++)
            {
                spline.SetDirection(i, pnts[i].second);
            }

            pnts.clear();
            return;
        }

    private:
        std::vector<std::pair<point, point> > pnts;

        static bool cmp_pnts(const std::pair<point, point> &a, const std::pair<point, point> &b) 
        {
            return a.first.x < b.first.x;
        }
        static bool equal(const std::pair<point, point> &a, const std::pair<point, point> &b) 
        {
            return cg::eq(a.first, b.first);
        }

        typedef HermiteSplineManager<point> Spline;
        Spline spline;
    };

} // end of namespace cg

#pragma pack(pop)
