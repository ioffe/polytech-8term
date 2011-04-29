#pragma once 

#pragma once

namespace cg
{
    template<typename range>
    struct cycle_value
    {
        double x;

        cycle_value() : x(range::def())   {}
        cycle_value( double x ) : x ( norm(x) ) {}
        cycle_value( cycle_value const& cv ) : x ( cv.x ) {} 
        
        operator double  () const { return x ; } 
        operator double& ()       { return x ; } 
        
        static double norm ( double x ) 
        {
             if ( x >= range::max() )
                x = range::min() + cg::mod ( x - range::min(), range::max() - range::min() ) ;
             else if ( x < range::min() ) 
                x = range::max() - cg::mod ( range::max() - x, range::max() - range::min() );
             return x ;           
        }
    };
    
    struct degree180_range
    {
        static double min() { return -180. ; }
        static double max() { return  180. ; }
        static double def() { return    0. ; }
    } ;
    
    typedef cycle_value<degree180_range> degree180_value ; 

    // ----------------------------------------------------------------------------------
    // basic operations
    template<typename range>
    inline cycle_value<range> operator - (cycle_value<range> const &A)    
    {   return cycle_value<range>( - A.x );  }

    template<typename range>
    inline cycle_value<range> operator + (cycle_value<range> const &A, cycle_value<range> const &B)    
    {   return cycle_value<range>( A.x + B.x );  }

    template<typename range>
    inline cycle_value<range> operator + (cycle_value<range> const &A, double x)    
    {   return cycle_value<range>( A.x + x );  }

    template<typename range>
    inline cycle_value<range> operator + (double x, cycle_value<range> const &A)    
    {   return cycle_value<range>( A.x + x );  }

    template<typename range>
    inline cycle_value<range> operator - (cycle_value<range> const &A, double x)    
    {   return cycle_value<range>( A.x - x );  }

    template<typename range>
    inline double operator - (cycle_value<range> const &A, cycle_value<range> const& B)    
    {   return cycle_value<range>::norm( A.x - B.x );  }

    template<typename range>
    inline cycle_value<range>  operator * (cycle_value<range> const &A, double d)
    {   return cycle_value<range>( A.x * d ); }

    template<typename range>
    inline cycle_value<range>  operator * (cycle_value<range> const &A, cycle_value<range> const &B)
    {   return cycle_value<range>( A.x * B.x ); }

    template<typename range>
    inline cycle_value<range>  operator * ( double d, cycle_value<range> const &A)
    {   return cycle_value<range>( A.x * d ); }

    template<typename range>
    inline cycle_value<range>  operator / (cycle_value<range> const &A, double d)
    {   return cycle_value<range>( A.x / d ); }
    
    template<typename range>
    inline double distance_sqr(cycle_value<range> const &A, cycle_value<range> const &B)
    {   return (A - B)*(A - B); }

    template<typename range>
    inline double distance(cycle_value<range> const &A, cycle_value<range> const &B)
    {   return cg::abs(A - B); }

    template<typename range>
    inline bool eq(cycle_value<range> const &A, cycle_value<range> const &B)
    {   return eq(0., distance(A,B));  }

    template< class T > struct SplineTypeTraits ; 

    template< class T >
    struct SplineTypeTraits< cycle_value < T > >
    {
      typedef double diff_type;      
    };

}
