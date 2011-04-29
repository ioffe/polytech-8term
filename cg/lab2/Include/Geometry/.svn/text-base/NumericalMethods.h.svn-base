#pragma once

namespace cg
{

template < class _Func >
    double IntegrateSimpson( _Func & f, double from, double to, double eps = .00001, int maxIterations = 1024 )
{
    Assert( ge( eps, 0 ) /* && !eq_zero( eps ) */ ) ; 

    sort2( from, to ) ;

    int N = 1 ; 
    double h = (to - from) / 2. ; 
    double prevI, I = (f( from ) + 4. * h * f( from + h ) + f( to )) / 3. ; 

    do
    {
        prevI = I ; 
        N *= 2 ; 
        h /= 2. ; 

        I = f( from ) - f( to );
        for ( int i = 0; i < N; i++ )
            I += 4. * f( from + (2 * i + 1) * h ) + 2. *  f( from + (2 * i + 2) * h );
        I = h * I / 3.;

    } while ( !eq( I, prevI, 7 * eps ) && N < maxIterations ) ; // Runge estimation

    return I; 
}

}
