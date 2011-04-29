#pragma once

namespace cg
{

template<class T> class EaseCurvSpline
{
public:
    typedef T point;

    __forceinline point Interpolate( int nPoints, const point *points, double t ) const
    {
        if (t<0) t = 0;
        else
           if (t>1 - cg::epsilon<double>()) t = 1 - cg::epsilon<double>();

        int it    = (int)(t * (nPoints - 1));
        double ft = t - it;
        double a  = ft * ft * (3 - 2 * ft);
        //double b  = /*1 - a;/*/ (1 - ft) * (1 - ft) * (3 - 2 * (1 - ft));
        //double a = ft;
        //double b = 1 - a;

        return (1 - a) * points[it] + a * points[it + 1];
    }

    __forceinline point Interpolate( const point & pt1, const point & pt2, double t ) const
    {
        if (t<0) t = 0;
        else if (t>1) t = 1;

        double a  = t * t * (3 - 2 * t);
        return (1 - a) * pt1 + a * pt2;
    }
};

template<class T> class EaseCurvSurf
{
public:
    typedef T point;
    typedef EaseCurvSpline<T> Spline;

    __forceinline point Interpolate( const point (&points)[2][2], double u, double v ) const
    {
       point const & p1 = spline.Interpolate(points[0][0], points[0][1], v);
       point const & p2 = spline.Interpolate(points[1][0], points[1][1], v);

       return spline.Interpolate(p1, p2, u);
    }

private:
    Spline spline;
};

} // end of namespace cg

//
// End of file 'Catmull-Rom.h'
//


