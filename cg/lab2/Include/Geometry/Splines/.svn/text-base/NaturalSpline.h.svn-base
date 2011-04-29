#pragma once

#include "HermiteSplines.h"
#include "geometry\NumericalMethods.h"

#include "Streams\aux_traits.h"

#pragma pack(push, 1)

namespace cg
{
    //////////////////////////////////////////////////////////////////////////
    template <class T, class Spline> class SplineArcLenDerivative
    {
    public:
        SplineArcLenDerivative( Spline * spline )
            : spline_ ( spline )
        {}
        
        double operator () ( double t ) const
        {
            Assert( spline_ ) ; 
            return norm( spline_->Derivative( t ) ) ; 
        }

        // ѕо умолчанию считает с точностью до миллиметра
        double Length( double eps = .001 ) const
        {
            Assert( spline_ ) ; 
            return Length( 0, spline_->GetNumPoints() - 1 ) ; 
        }

        double Length( double from, double to, double eps = .001 ) const
        {
            Assert( spline_ ) ; 

            int nseg = spline_->GetNumPoints() - 1 ; 
            
            int nfrom = Clip<int   >( 0, nseg, int(from + 1) ) ; 
            from      = Clip<double>( 0, nseg, from          ) ; 
            int nto   = Clip<int   >( 0, nseg, int(to)       ) ; 
            to        = Clip<double>( 0, nseg, to            ) ; 

            double length = 0;

            if ( nto < nfrom )
                length = IntegrateSimpson( *this, from, to, eps ) ; 
            else
            {
                length = IntegrateSimpson( *this, from, nfrom, eps ) ; 
                for ( int i = nfrom; i < nto; i++ )
                    length += IntegrateSimpson( *this, i, i + 1, eps ) ; 
                length += IntegrateSimpson( *this, nto, to, eps ) ; 
            }

            return length;
        }

    private:
        template < class _X > static
            const _X & Clip( const _X &a, const _X &b, const _X &val )
        {
            return val < a ? a : val > b ? b : val ; 
        }

    private:
        Spline * const spline_ ; 
    };

    //////////////////////////////////////////////////////////////////////////
    template <class Point, class Spline = HermiteSplineManager<Point>, class Traits = streams::default_traits > 
       struct NaturalSpline
    {
       typedef Point point_type ;

       typedef typename Traits::vector<double>::type double_vector_type;

       NaturalSpline () {}

       // TODO: Required ability to control not only lowest precision of spline, but also highest precision.
       // Currently there is hack that can be enabled by additional argument 'allowIncreaseStep'.
       NaturalSpline ( Spline const &s, double eps = 1., bool allowIncreaseStep = false ) 
           : spline_( s )
       {
            eps_ = eps;

            SplineArcLenDerivative<Point, Spline> ds_dt( &spline_ ) ; 

            // build temporary index 
            std::vector<point_2> index ; 
            
            double max_t = spline_.GetNumSegments( );//spline_.GetNumPoints() - 1 ; 

            double t      = 0 ; 
            double length = 0 ; 
            double speed  = ds_dt(t) ; 
            
            index.push_back( point_2(t, length) ) ;
             
            while ( t < max_t ) 
            {
                 double dt = 0.5 ; 
                 if ( speed * dt > eps ) 
                      dt = eps / speed ; 
                 
                 int dtChaningDirection = 0; // '+1' for increasing, '-1' for decreasing.
                 double next_t, dlength, next_speed;
                 while ( true ) 
                 {
                      next_t = t + dt ; 
                      if ( next_t > max_t ) 
                           next_t = max_t ; 

                      next_speed = ds_dt(next_t) ; 
                         
                      // double dlength = ds_dt.Length(t, next_t) ; 
                      dlength = .5 * (next_speed + speed) * (next_t - t) ; 
                 
                      if ( dtChaningDirection <= 0 && dlength > 2 * eps ) 
                      {
                           dtChaningDirection = -1 ;
                           dt /= 2 ; 
                      }
                      else if ( allowIncreaseStep && dtChaningDirection >= 0 && next_t != max_t && dlength < 0.5 * eps ) 
                      {
                           dtChaningDirection = +1 ;
                           dt *= 2 ; 
                      }
                      else
                           break ;
                 }

                 double next_length = length + dlength ; 

                 index.push_back( point_2( next_t, next_length ) ) ; 
             
                 speed  = next_speed ; 
                 t      = next_t ; 
                 length = next_length ; 
            }     
            
            length_ = index.back().y ; 
            
            // build final index 
            index_.push_back ( 0 ) ; 
            index_.push_back ( max_t ) ; 
            dlength_ = length_ / (index_.size() - 1) ; 
            
            while ( true ) 
            {
                 BOOL ok = TRUE ; 
                 
                 // check distance
                 DWORD j = 1 ; 
                 for ( DWORD i = 0 ; ok && i < index.size() ; i ++ ) 
                 {
                      while ( j < index_.size () ) 
                           if ( index[i].x < index_[j] ) 
                           {
                                double len = dlength_ * lerp<double>( index_[j-1], index_[j], j - 1, j)(index[i].x) ;   
                                ok = cg::abs(len - index[i].y) < eps ; 
                                break ; 
                           }
                           else j ++ ; 
                 }      
 
                 if ( ok ) 
                      break ; 
                      
                 // redivide index_
                 j = 1 ; 
                 std::vector<double> nindex ((index_.size() - 1) * 2 + 1) ; 
                 for ( DWORD i = 0 ; i < index_.size() - 1 ; i ++ )
                 {
                      nindex[i*2] = index_[i] ; 
                      double len = (i + 0.5) * dlength_ ; 
                      
                      while ( j < index.size() ) 
                           if ( len < index[j].y ) 
                           {
                                nindex[i*2 + 1] = lerp<double>(index[j-1].y, index[j].y, index[j-1].x, index[j].x)( len ) ; 
                                break ; 
                           }    
                           else j ++ ; 
                 }
                 nindex.back () = max_t ; 

                 std::swap ( nindex, index_ ) ; 
                 dlength_ = length_ / (index_.size() - 1) ; 
            }

            dlength_inv_ = 1. / dlength_;
        }

        int ConstraintIndexSize ( int index_size ) 
        {
            if ( IndexSize() < index_size ) 
                return IndexSize() ;

            index_size = limit ( 2, index_size ) ;
            
            double dlen = length_ / ( index_size - 1 )   ;
            std::vector < double > nindex ( index_size ) ;

            for ( size_t i = 0; i < index_size; ++i )
                nindex [i] = GetParameter ( i * dlen ) ;

            index_ = nindex     ;
            return index_size   ; 
        }
        
        Point Interpolate ( double len ) const
        {
            return spline_.Interpolate( GetParameter(len) );
        }

        Point Direction ( double len ) const
        {
            return spline_.Direction( GetParameter(len) );
        }

        Point Binormal ( double len ) const
        {
            return spline_.Binormal( GetParameter(len) );
        }

        void CoordSys( double len, Point& origin, Point& direction, Point &binormal) const
        {
            double p = GetParameter(len);
            origin    = spline_.Interpolate ( p );
            direction = spline_.Direction   ( p );
            binormal  = spline_.Binormal    ( p );

            return;
        }

        double Radius ( double len ) const
        {
            return spline_.Radius( GetParameter(len) );
        }

        double Curvature( double len ) const 
        {
            return spline_.Curvature( GetParameter(len) );
        }

        double Length() const
        {
            return length_;
        }
        
        // requires "Point::operator P () const" 
        template < class P >
        double GetDistance2SplineArea ( P const &p, double& len, double areaLength ) const
        {
            Assert( !index_.empty()) ;

            if ( areaLength < 0 )
               areaLength = 0;

            double start = cg::bound( len - areaLength / 2, 0., length_ );
            double stop  = cg::bound( len + areaLength / 2, 0., length_ );

            size_t nstart = floor( start / dlength_ );
            size_t nstop  = ceil ( stop  / dlength_ );

            // DEBUG 
            Assert( nstart >= 0 && nstart <  index_.size()) ;
            Assert( nstop  >= 0 && nstop  <= index_.size()) ;

            P p0 = spline_.Interpolate( index_.at( nstart )) ; // for exception throwing 

            double dist = norm_sqr(p - p0);
            len = nstart * dlength_ ;           

            for ( size_t i = nstart + 1; ( i <= nstop ) && ( i < index_.size()); i++ )
            {
                P p1 = spline_.Interpolate( index_[i] ) ; 

                P dp1 = p1 - p0 ; 
                P dp  = p  - p0 ; 

                double t = (dp * dp1) / (dp1 * dp1) ; 

                if ( t > 0 ) 
                {
                    if ( t > 1 ) 
                        t = 1 ; 

                    double d = norm_sqr( dp1 * t - dp ) ; 
                    if ( d < dist ) 
                    {
                        dist = d ; 
                        len  = (i - 1 + t) * dlength_ ; 
                    }
                }
                p0 = p1 ; 
            }

            if ( len - eps_ > 0 && len + eps_ < length_ )
               dist = GetDistanceMoreAccurate( p, len - eps_, len + eps_, eps_, len ) ; 
            else
                dist = cg::sqrt(dist);

            return dist ;   
        }

        template<class P>
        double GetDistanceEx( P const &p, double &len ) const
        {
            len = length_ / 2 ;
            return GetDistance2SplineArea ( p, len, length_ + 1 ) ;
        }

        Point GetClosestPoint( Point const &p, double &len ) const
        {
            GetDistanceEx( p, len ) ; 
            return Interpolate( len ) ; 
        }

        BOOL GetDistance( Point const &p, double maxdist, double &dist, double &len ) const
        {
            dist = GetDistanceEx( p, len ) ; 
            return dist < maxdist ; 
        }    

        BOOL GetBoundedDistance ( Point const &p, double maxdist, double mindist, double &dist, double &len ) const
        {
            dist = GetDistanceEx( p, len ) ; 
            return ( dist > mindist && dist < maxdist ) ; 
        }    

        // Find first point on spline within given Euclidean distance from Interpolate(len)
        // forward - direction along spline
        BOOL GetPointWithinDistance( double len, double dist, double &foundLen, bool forward, double eps ) const
        {
           Assert(!index_.empty());
           size_t prev = forward ? bound( floor( len / dlength_ ), 0, (int)index_.size() - 1 ) : 
                                   bound( ceil( len / dlength_ ), 0, (int)index_.size() - 1 );

           if(forward && ((index_.size() - 1 == prev) || ge((prev + 1) * dlength_ - len, dist, eps)))
           {
              foundLen = len + dist;
              return TRUE ;
           }
           else if(!forward && ((0 == prev) || ge(len - (prev - 1) * dlength_, dist, eps)))
           {
              foundLen = len - dist;
              return TRUE ; 
           }
           else
           {
              Point p = Interpolate(len) ;

              Point p0, p1 = spline_.Interpolate( index_[prev + (forward ? 1 : -1)] ) ; 
              
              double d0 = 0;
              double d1 = norm_sqr(p - p1);

              double d2 = dist * dist;
              size_t i = prev + (forward ? 2 : -2);

              if(cg::range_2(d0, d1).contains(d2, eps * eps))
              {
                 foundLen = len + (forward ? +1 : -1) * dist;
                 return TRUE ;
              }

              for ( size_t end = (forward ? index_.size() : -1); i != end; forward ? ++i : --i )
              {
                 p0 = p1 ; 
                 d0 = d1 ;

                 p1 = spline_.Interpolate( index_[i] ) ; 
                 d1 = norm_sqr(p - p1);

                 if(cg::range_2(d0, d1).contains(d2, eps * eps))
                    break;
              }

              if(i >= index_.size())
                 return FALSE;

              Assert((forward && (i > 0)) || (!forward && (i < index_.size())));

              segment_t<Point::scalar_type, Point::dimension> s(p0, p1);
              double t = s(p);
              double l = length(s);
              double f = cg::sqrt(d2 - norm_sqr(s(t) - p)) / l;

              double tFound = (d0 > d1) ? t - f : t + f;
              Assert(range_2(0., 1.).contains(tFound, eps));

              foundLen = forward ? ((i - 1) * dlength_ + tFound * l) : 
                                   (i * dlength_ + (1. - tFound) * l);

              return TRUE ;   
           }
        }

    public : 
        DWORD IndexSize () const 
        {
            return index_.size () ; 
        }    

        double GetParameter( double len ) const
        {
            if ( len < 0 ) 
                len = 0 ; 
            else if ( len >= length_ ) 
                len = length_ - 0.0001 ; 

            double di = len * dlength_inv_;
            DWORD i = (DWORD)(di) ; 
            
            if ( i == index_.size() - 1 )
               --i;

            // lerp<double>(i*dlength_, (i + 1)*dlength_, index_[i], index_[i+1])( len );
            return index_[i] + (index_[i+1] - index_[i]) * (di - i) ;
        }

        double GetLength ( double t ) const 
        {
            if ( t < 0 ) 
                 t = 0 ; 
            else if ( t >= spline_.GetNumPoints() - 1 ) 
                      t = spline_.GetNumPoints() - 1 ; 
                      
            // find segment 
            int lo = 0 ; 
            int hi = index_.size() - 1; 
            
            while ( hi != lo + 1 ) 
            {
                int m = (lo + hi) / 2 ; 
                if ( index_[m] > t )
                     hi = m ; 
                else lo = m ;      
            }           

            return lerp<double>(index_[lo], index_[hi], lo*dlength_, hi*dlength_ )( t ) ; 
        }
        
        double GetDtByDlen ( double len ) const 
        {
            if ( len < 0 ) 
                 len = 0 ; 
            else if ( len >= length_ ) 
                      len = length_ - 0.0001 ; 
                      
            DWORD i = (DWORD)(len / dlength_) ; 
            
            return (index_[i+1] - index_[i]) / dlength_ ; 
        }

        template<class P>
        double GetDistanceMoreAccurate( P const &p0, double s1, double s2, double eps, double & s ) const
        {
            // Linear Minimization Method
            range_2 const validS(s1, s2);

            s = .5 * (s1 + s2) ; 
            double sPrev, DsPrev, Ds1, Ds2 ;
            double Ds = norm_sqr( p0 - P(Interpolate( s )) ) ; 

            while ( true )
            {
                double s21 = s2 - s1;
                Ds1 = norm_sqr( p0 - P(Interpolate( s1 )) ) ; 
                Ds2 = norm_sqr( p0 - P(Interpolate( s2 )) ) ; 

                sPrev  = s  ; 
                DsPrev = Ds ; 

                s = s1 + (Ds2 + s21 * s21 - Ds1) / (2 * s21) ; 
                s = validS.closest_point(s);
                
                Ds = norm_sqr( p0 - P(Interpolate( s )) ) ; 

                if ( !ge( Ds1, Ds, eps ) ) { std::swap( Ds, Ds1 ); std::swap( s, s1 ); }
                if ( !ge( Ds2, Ds, eps ) ) { std::swap( Ds, Ds2 ); std::swap( s, s2 ); }

                if ( ge( Ds, DsPrev, eps ) || cg::eq(s1, s2) ) break ; 
            }

            /*
            // Quadratic Minimization Method
            double sPrev,  s = s2, s23, s31, s12 ; 
            double DsPrev, Ds = norm_sqr( p0 - Interpolate( s2 ) ),
                   Ds1, Ds2, Ds3 ; 
                                 
            while ( true )
            {
                s23 = s2 - s3 ; 
                s31 = s3 - s1 ; 
                s12 = s1 - s2 ; 

                double y23 = s23 * (s2 + s3) ; 
                double y31 = s31 * (s3 + s1) ; 
                double y12 = s12 * (s1 + s2) ; 

                Ds1 = norm_sqr( p0 - Interpolate( s1 ) ) ; 
                Ds2 = norm_sqr( p0 - Interpolate( s2 ) ) ; 
                Ds3 = norm_sqr( p0 - Interpolate( s3 ) ) ; 

                sPrev = s ; 
                s = .5 * ( y23 * Ds1 + y31 * Ds2 + y12 * Ds3 ) / 
                         ( s23 * Ds1 + s31 * Ds2 + s12 * Ds3 ) ; 

                double ss1 = s - s1 ; 
                double ss2 = s - s2 ; 
                double ss3 = s - s3 ; 

                DsPrev = Ds ; 
                Ds = -Ds1 * ss2 * ss3 / (s12 * s31) -
                      Ds2 * ss1 * ss3 / (s12 * s23) -
                      Ds2 * ss1 * ss2 / (s31 * s23) ; 

                if ( ge( Ds, DsPrev, eps ) ) break ; 

                if ( !ge( Ds1, Ds, eps ) ) { std::swap( Ds, Ds1 ); std::swap( s, s1 ); }
                if ( !ge( Ds2, Ds, eps ) ) { std::swap( Ds, Ds2 ); std::swap( s, s2 ); }
                if ( !ge( Ds3, Ds, eps ) ) { std::swap( Ds, Ds3 ); std::swap( s, s3 ); }

            }
            */

            s = sPrev ; 
            return cg::sqrt( DsPrev ) ; 
        }

        template < class point2D >  
        void GetApproximating2DPolyline ( std::vector< point2D > & approximatingPolyline, double deviation ) const // for distance search use 
        {
           std::vector < point2D > polyline2D ( index_.size()) ;
           for ( size_t i = 0; i < index_.size(); ++i )
           {
              point_type point = spline_.Interpolate( index_[i] ) ;           
              polyline2D[i] = point2D ( point.x, point.y ) ;
           }

           SimplifyPolyline( polyline2D, approximatingPolyline, deviation ) ;
        }

    public:
       Spline const& GetSpline() const { return spline_; }

    // Serialization stuff
    public:
       template<class Stream>
       void serialize(Stream &stream) const
       {
          write(stream, spline_);
          write(stream, index_);
          write(stream, length_);
          write(stream, dlength_);
          write(stream, dlength_inv_);
          write(stream, eps_);
       }        

       template<class Stream>
       void deserialize(Stream &stream) 
       {
          read(stream, spline_);
          read(stream, index_);
          read(stream, length_);
          read(stream, dlength_);
          read(stream, dlength_inv_);
          read(stream, eps_);
       }        

    protected:

       template < class point2D >
       void SimplifyPolyline ( std::vector<point2D> const & sourcePolyline, std::vector<point2D> & resultPolyline, double deviation ) const
       {
          //-- creating adaptive fragmentation 
          size_t start = 0 ;
          size_t stop  = 0 ;

          point2D pStart = sourcePolyline[start] ;
          point2D pStop  = sourcePolyline[stop ] ;

          resultPolyline.push_back( pStart ) ;

          while ( stop < sourcePolyline.size() - 1 )
          {
             // segment creation
             point2D seg ;

             double maxAngle = pi ;
             double minAngle = - pi ;
             double angleDir = ( maxAngle + minAngle ) / 2 ;

             double dirAngle = 0 ; 
             double addAngle = 0 ;  

             //-- alpha scissoring 
             while (( maxAngle > minAngle ) && ( stop < sourcePolyline.size() - 1 ))
             {
                angleDir = ( maxAngle + minAngle ) / 2 ;

                ++ stop ;
                pStop = sourcePolyline[stop] ;

                seg = pStop - pStart ;
                
                dirAngle = atan2 ( seg.y, seg.x ) ;
                addAngle = abs ( asin ( bound( deviation / norm ( seg ), -1., 1. ))) ;

                if ( maxAngle > dirAngle + addAngle )
                   maxAngle = dirAngle + addAngle ;

                if ( minAngle < dirAngle - addAngle )
                   minAngle = dirAngle - addAngle ;
             }

             if ( maxAngle < minAngle )
                -- stop ;
             else 
                angleDir = ( maxAngle + minAngle ) / 2 ; 

             point2D dir ( cos( angleDir ), sin( angleDir )) ;
             pStart += ( seg * dir ) * dir ;

             resultPolyline.push_back( pStart ) ;
             start = stop ;   
          }
       }

    private:
        Spline             spline_;
        double_vector_type index_;
        double             length_;
        double             dlength_;
        double             dlength_inv_;
        double             eps_;
    };

    template< class Stream, class Point, class Spline, class Traits >
    void write(Stream &stream, NaturalSpline<Point, Spline, Traits> const &spline)
    {
       spline.serialize(stream);
    }
    template< class Stream, class Point, class Spline, class Traits >
    void read(Stream &stream, NaturalSpline<Point, Spline, Traits> &spline)
    {
       spline.deserialize(stream);
    }
} // cg

#pragma pack(pop)
