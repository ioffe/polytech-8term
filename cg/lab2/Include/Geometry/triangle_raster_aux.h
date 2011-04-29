#pragma once

#include "primitives\point.h"
#include "primitives\triangle.h"
#include "primitives\color.h"
#include "primitives\range.h"
#include "primitives\segment.h"

namespace cg
{
    inline point_2 calcBarycentericCoords(
      point_2 const & pt, point_2 const & v0, 
      point_2 const & v1, point_2 const & v2)
    {
       point_2 const & b0 = pt - v0;
       point_2 const & b1 = v1 - v0;
       point_2 const & b2 = v2 - v0;

       double det = b1 ^ b2;//b2.y * b1.x - b2.x * b1.y;

       Assert(!eq_zero (det));

       double alpha = ( b0 ^ b2 ) / det; //(b2.y * b0.x - b0.y * b2.x) / det;
       double beta  = ( b1 ^ b0 ) / det; //(b0.y * b1.x - b0.x * b1.y) / det;

       return point_2(alpha, beta);
    }

    struct barycentric_coords
    {
        double alpha, beta;

        barycentric_coords (
          point_2 const & pt, point_2 const & v0, 
          point_2 const & v1, point_2 const & v2)
        {
          point_2 const & t = calcBarycentericCoords(pt, v0, v1, v2);

          alpha = t.x;
          beta  = t.y;
        }

        barycentric_coords( )
        { }

        barycentric_coords(double alpha, double beta)
          : alpha(alpha)
          , beta (beta)
        { }

        barycentric_coords(point_2 const & pt)
          : alpha(pt.x)
          , beta (pt.y)
        { }

        bool inTriangle() const
        {
          return ge(alpha, 0.) && ge(beta, 0.) && le(alpha + beta, 1.);
        }
    };

    __declspec( deprecated ) struct triangle_raster_aux
    {
        // triangle vertexes
	    point_3 const &v1, &v2, &v3;

	    triangle_raster_aux (triangle_3 const &prototype) 
		    : v1 ( prototype[0] )
          , v2 ( prototype[1] )
          , v3 ( prototype[2] )
	    {
        }

        inline bool IsInTriangleBarycentric(point_2 const &pt, barycentric_coords & bc) const
        {
            // считаем бароцентрические координаты
            double bu0 = pt.x - v1.x;
            double bv0 = pt.y - v1.y;

            double bu1 = v2.x - v1.x;
            double bv1 = v2.y - v1.y;

            double bu2 = v3.x - v1.x;
            double bv2 = v3.y - v1.y;
            double det = bv2*bu1 - bu2*bv1; 

//            Assert( !eq(det, 0.0) );
            if ( eq(det, 0.0) ) // проверка треугольника на вырожденность
                return false;

            double deta = bv2*bu0 - bv0*bu2;
            double detb = bv0*bu1 - bv1*bu0;
            bc.alpha = deta / det;
            bc.beta  = detb / det;

            const double eps = epsilon<double>( ) ;

            if (bc.beta  < -eps || bc.beta > 1.0+eps)
                return false;
            if (bc.alpha < -eps || (bc.alpha + bc.beta) > 1.0+eps)
                return false;

            return true;
        }


        inline double GenerateHeightForPoint(double x, double y) const
        {
            point_3 L = v2 - v1;
            point_3 M = v3 - v1;

            double C = L.x * M.y  -  M.x * L.y;

            if (eq(C, 0.))
            {
                // ћы не можем по этим точкам однозначно построить плоскость - 
                // так как все проекции точек треуголькина на горизонталь
                // лежат на одной линии.
                // —троим по несовпадающим точкам отрезок (верхнюю грань треугольника)
                point_2 p2(x, y);

                double zz1 = CalcZCoordinateForLine( v1, v2, p2 );
                double zz2 = CalcZCoordinateForLine( v2, v3, p2 );
                double zz3 = CalcZCoordinateForLine( v3, v1, p2 );

                return max(zz1, zz2, zz3);
            }

            double A = L.x * M.z  -  M.x * L.z;
            double B = L.y * M.z  -  M.y * L.z;

            return v1.z  +  ( (y - v1.y) * A 
                            - (x - v1.x) * B ) / C;
        }

        inline double GenerateHeightForPoint(point_2 const &pt) const
        {
            return GenerateHeightForPoint(pt.x, pt.y);
        }



    private:
        inline double CalcZCoordinateForLine(point_3 const &p1, point_3 const &p2, point_2 const &query_pt) const
        {
            if (eq((point_2&)p1, (point_2&)p2))
            {
                if (eq(query_pt, (point_2)p1))
                    return max(p1.z,p2.z);

                return -FLOAT_ETERNITY;
            }

            double alpha;
            if (eq(p2.x,p1.x)) alpha = (query_pt.y-p1.y) / (p2.y-p1.y);
                          else alpha = (query_pt.x-p1.x) / (p2.x-p1.x);

            if (alpha<0.0 || alpha>1.0)
                return -FLOAT_ETERNITY;

            return p1.z + (p2.z-p1.z) * alpha;
        }
    };

    inline void update_zrange(triangle_raster_aux const &t, 
                              point_2 const &pt, range_2 &zrange)
    {
        cg::barycentric_coords bc;
        if (t.IsInTriangleBarycentric(pt, bc)) {
            zrange.unite(t.GenerateHeightForPoint(pt.x, pt.y));
        }
    }

    inline void update_zrange_unchecked(triangle_raster_aux const &t, 
                              point_2 const &pt, range_2 &zrange)
    {
          zrange.unite(t.GenerateHeightForPoint(pt.x, pt.y));
    }


    __forceinline float barycentric_interpolate_in_triangle( const barycentric_coords& bc, float a1, float a2, float a3 )
    {
      return  (float)(a1*(1 - bc.alpha - bc.beta) + a2*bc.alpha + a3*bc.beta);
    }

    __forceinline cg::colorf barycentric_interpolate_in_triangle( const barycentric_coords& bc, cg::colorf const & a1,
                                                                  cg::colorf const & a2, cg::colorf const & a3 )
    {
      return  (a1*float(1 - bc.alpha - bc.beta) + a2*float(bc.alpha) + a3*float(bc.beta));
    }

    template< class Value >
       __forceinline Value barycentric_interpolate_in_triangle( const barycentric_coords& bc, Value a1, Value a2, Value a3 )
    {
      return static_cast< Value >( (a1*(1 - bc.alpha - bc.beta) + a2*bc.alpha + a3*bc.beta) );
    }

    inline point_2 robust_calc_barycentric_coords( point_2 const & pt, point_2 const & v0, point_2 const & v1, point_2 const & v2 )
    {
       point_2 const & b0 = pt - v0;
       point_2 const & b1 = v1 - v0;
       point_2 const & b2 = v2 - v0;

       double det = b1 ^ b2;

       if (!cg::eq_zero (det * det)) // must be half-ordered
       {
          double alpha = ( b0 ^ b2 ) / det;
          double beta  = ( b1 ^ b0 ) / det;

          return point_2(alpha, beta);
       }
       else
       {
          cg::segment_2 s1( v0, v1 ), s2( v1, v2 ), s3( v2, v0 );
          double len1( length_sqr( s1 ) ), len2( length_sqr( s2 ) ), len3( length_sqr( s3 ) );

          if ( len1 >= len2 && len1 >= len3 )
             return cg::point_2( s1( pt ), 0 );
          else if ( len3 >= len1 && len3 >= len2 )
             return cg::point_2( 0, s3( pt ) );
          else 
          {
             double t = s2( pt );
             return cg::point_2( 1 - t, t );
          }
       }
    }

    template < class Value >
    float robust_interpolate_in_triangle( cg::point_2 const & pt, cg::triangle_2 const & trg,
       Value  a, Value b, Value c )
    {
       return cg::barycentric_interpolate_in_triangle(
          cg::barycentric_coords( robust_calc_barycentric_coords( pt, trg[0], trg[1], trg[2] ) ), a, b, c );
    }
}