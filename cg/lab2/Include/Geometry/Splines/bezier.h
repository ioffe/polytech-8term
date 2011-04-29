#pragma once

namespace cg
{

template<class T> class BezierSpline
{
public:
    typedef T point;
    typedef std::vector<int>     COEFFS;
    typedef std::vector<double>  SCALARS;

    BezierSpline(int order_) :
    order(order_)
    {
        // calculate coefficients
        coeffs.clear();
        for (int i = 0; i <= order; i++) 
        {
            coeffs.insert(coeffs.begin(), 1.);
            for (int j = 1; j < coeffs.size() - 1; j++) 
            {
                coeffs[j] = coeffs[j] + coeffs[j+1];
            }
        }
        Assert(coeffs.size() == order + 1);
        scalars.resize(coeffs.size());
    }

    ~BezierSpline()
    {
        coeffs.clear();
    }

    void SetT(int nPoints, double t) const
    {
        Assert((nPoints - 1) % order == 0);
        if (t<0) t = 0;
        if (t>1) t = 1;

        // calc offset
        offset = t * nPoints;
        offset -= (offset % (order + 1));
        t = (t * nPoints - offset) / (order + 1);
        t += offset / nPoints;
        offset %= nPoints;

        // scalars[i] = coeffs[i] * t^i * (1-t)^(order-i)
        for (int i = 0; i <= order; i++)
            scalars[i] = coeffs[i];

        double tn = 1;
        double itn = 1;
        for (int i = 0; i <= order; i++)
        {
            scalars[i] *= tn;
            scalars[order-i] *= itn;

            tn *= t;
            itn *= 1-t;
        }
    }

    point Interpolate ( const point *points ) const
    {
        point res(0,0);
        for (int i = 0; i <= order; i++) 
            res += points[offset + i] * scalars[i];

        return res;


    }

    point Interpolate( int nPoints, const point *points, double t ) const
    {
        SetT(nPoints, t);
        return Interpolate(points);
    }

private:
    mutable SCALARS scalars;
    mutable int     offset;
    COEFFS coeffs;
    int order;
};

template<class T> class BezierSurf
{
public:
    typedef T point;
    typedef BezierSpline<T> BSpline;

    BezierSurf(int orderX_ = 3, int orderY_ = 3) :
    orderX(orderX_),
    orderY(orderY_),
    splineX(orderX_),
    splineY(orderY_)
    {
        tempPoints = new point[1 + orderX];
    }

    ~BezierSurf()
    {
        delete [] tempPoints;
    }

    point Interpolate( int nPointX, int nPointY, const point* points, double u, double v ) const
    {
        splineY.SetT(nPointY, v);
        for (int i = 0; i <= orderX; i++)
            tempPoints[i] = splineY.Interpolate(&points[nPointY * i]);
        return splineX.Interpolate(1 + orderX, tempPoints, u);
    }    

private:
    BSpline splineX;
    BSpline splineY;
    point *tempPoints;
    int orderX, orderY;
};

} // end of namespace cg

//
// End of file 'bezier.h'
//


