#pragma once

#include "primitives/point.h"

namespace cg
{
/*    // возвращает вектор w сонаправленный v: |w| = L
    // если v == 0, возвращает 0
    inline point_3 normalize(point_3 const &v)
    {
        double n = norm(v);
        return eq(n, 0) ? v : v / n;
    }*/

    inline point_3 to_point3( point_3f const &p )
    {
        return point_3( p.x, p.y, p.z );
    }


    struct simple_triangle_3
    {
        unsigned i1, i2, i3;
        point_3f n;
    //    double orig;

        simple_triangle_3()
        {}

        simple_triangle_3(unsigned i1_, unsigned i2_, unsigned i3_)
            : i1 ( i1_ )
            , i2 ( i2_ )
            , i3 ( i3_ )
        {}

        void CalcNormal(point_3f const *vertices)
        {
    //        Assert( i1<vertices.size() && i2<vertices.size() && i3<vertices.size() );

            point_3 p1 = to_point3(vertices[i1]);
            point_3 p2 = to_point3(vertices[i2]);
            point_3 p3 = to_point3(vertices[i3]);

            point_3 n = normalized( (p2-p1) ^ (p3-p1) );

            double orig = - p1 * n;

            if (orig<0)
            {
                n  = -n;
                orig = -orig;
            }

            this->n = point_3f(n);
        }


        unsigned& operator[] (int index)
        {
            return index==0 ? i1 : (index==1 ? i2 :i3);
        }
    };


    typedef std::vector<point_3f>          Point3fSet;
    typedef std::vector<simple_triangle_3> SimpleTriangleSet;
}