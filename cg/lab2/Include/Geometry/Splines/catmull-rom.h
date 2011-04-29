#pragma once

namespace cg
{

template<class T> class CatmullRomSpline
{
public:
    typedef T point;

    __forceinline CatmullRomSpline()
    {
        C[0][0] = -0.5; C[0][1] = 1.5;  C[0][2] = -1.5; C[0][3] = 0.5;
        C[1][0] = 1.0;  C[1][1] = -2.5; C[1][2] = 2.0;  C[1][3] = -0.5;
        C[2][0] = -0.5; C[2][1] = 0.0;  C[2][2] = 0.5;  C[2][3] = 0.0;
        C[3][0] = 0.0;  C[3][1] = 1.0;  C[3][2] = 0.0;  C[3][3] = 0.0;

        return;
    }

    ~CatmullRomSpline()
    {
        return;
    }

    __forceinline point Interpolate( int nPoints, const point *points, double t ) const
    {
        if (t<0) t = 0;
        if (t>1) t = 1;

        Assert(nPoints >= 4);

        int nSpans = nPoints - 3;
        double x = t * nSpans;
        int span = (int)x;
        cg::make_min(span, nSpans);
        x -= span;
        points += span;

        point c[4] = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++) 
                c[3 - i] += C[i][j] * points[j];
        }

        return ((c[3] * x + c[2]) * x + c[1]) * x + c[0];
    }

private:
    double C[4][4];
};

template<class T> class CatmullRomSurf
{
public:
    typedef T point;
    typedef CatmullRomSpline<T> Spline;

    __forceinline point Interpolate( int nPointX, int nPointY, const point* points, double u, double v ) const
    {
        point *tempPoints = (point *)_alloca(sizeof(point) * nPointX);
        for (int i = 0; i < nPointX; i++)
        {
            tempPoints[i] = spline.Interpolate(nPointY, &points[nPointY * i], v);
        }

        return spline.Interpolate(nPointX, tempPoints, u);
    }
private:
    Spline spline;
};

} // end of namespace cg

//
// End of file 'Catmull-Rom.h'
//


