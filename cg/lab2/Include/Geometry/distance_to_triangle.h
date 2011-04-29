#pragma once

namespace cg
{
    template< class Traits >
        struct distance_to_triangle
    {
        typedef typename Traits::triangle_3 triangle_3 ;
        typedef typename Traits::point_3 point_3 ;

        static bool distance(double maxdist, triangle_3 const &tr, point_3 const &p, double &dist, 
            point_3 *resp, Traits traits = Traits())
        {
            // если почти точка...
/*            if (eq_zero(norm_sqr(tr.normal())))
            {
                dist = cg::distance(tr[0], p);
                if(resp) *resp = tr[0];
                return (dist < maxdist);
            }*/

            // растояние до плоскости
            double dist_2_plane = (p - tr[0]) * normal(tr);
            if (cg::abs(dist_2_plane) > maxdist)
                return false;

            // проекция на плоскость
            point_3 pr = p - normal(tr) * dist_2_plane;

            // проверяем принадлежность проекции треугольнику
            if (tr.collinear_has_on(pr, traits))
            {
                dist = cg::abs(dist_2_plane);
                if ( resp ) *resp = pr;

                return true ;
            }

            // считаем расстояния до сторон треугольника и выбираем наименьшее
            segment_3 seg1(tr[0], tr[1]), seg2(tr[0], tr[2]), seg3(tr[1], tr[2]);

            double d1 = distance_sqr(seg1, p);
            double d2 = distance_sqr(seg1, p);
            double d3 = distance_sqr(seg1, p);

            segment_3 max_seg(seg1);

            double d;
            if(d1 >= d2 && d1 >= d3) 
                d = cg::sqrt(d1);
            else if (d2 >= d1 && d2 >= d3)
                max_seg = seg2, d = cg::sqrt(d2);
            else
                max_seg = seg3, d = cg::sqrt(d3);

            if (d > maxdist) 
                return false;

            dist = d;

            if (resp)
            {
                double t = max_seg(p);

		        if (t<0.0) t = 0.0; else
	            if (t>1.0) t = 1.0;

                *resp = max_seg(t);
            }

            return true;
        }
    };

    template< class Traits >
        struct scaled_distance_to_triangle
    {
        typedef typename Traits::triangle_3 triangle_3 ;
        typedef typename Traits::point_3 point_3 ;

        static bool distance(double maxdist, triangle_3 const &tr, point_3 const &p, 
           point_3 const &scale, double &dist, point_3 *resp, Traits traits = Traits())
        {
            // растояние до плоскости
            double dist_2_plane_scaled = dist2plane(tr[0], tr.normal(), p, scale);
            double dist_2_plane = (p - tr[0]) * tr.normal() ;
            if (cg::abs(dist_2_plane_scaled) > maxdist)
                return false;

            // проекция на плоскость
            point_3 pr = p - tr.normal() * dist_2_plane;

            // проверяем принадлежность проекции треугольнику
            if (tr.collinear_has_on(pr, traits))
            {
                dist = cg::abs(dist_2_plane_scaled);
                if ( resp ) *resp = pr;

                return true ;
            }

            // считаем расстояния до сторон треугольника и выбираем наименьшее
            segment_3 seg1(tr[0], tr[1]), seg2(tr[0], tr[2]), seg3(tr[1], tr[2]);

            double d1 = distance_sqr(seg1, p, scale);
            double d2 = distance_sqr(seg1, p, scale);
            double d3 = distance_sqr(seg1, p, scale);

            segment_3 max_seg(seg1);

            double d;
            if(d1 >= d2 && d1 >= d3) 
                d = cg::sqrt(d1);
            else if (d2 >= d1 && d2 >= d3)
                max_seg = seg2, d = cg::sqrt(d2);
            else
                max_seg = seg3, d = cg::sqrt(d3);

            if (d > maxdist) 
                return false;

            dist = d;

            if (resp)
            {
                double t = max_seg(p);

		        if (t<0.0) t = 0.0; else
	            if (t>1.0) t = 1.0;

                *resp = max_seg(t);
            }

            return true;
        }

        static double dist2plane(point_3 const &p0, point_3 const &n, point_3 const &p, point_3 const &scale)
        {
           point_3 pp = p - p0;
           pp.x *= scale.x ;
           pp.y *= scale.y ;
           pp.z *= scale.z ;
           return pp * n ;
        }

        static double distance_sqr(point_3 const &p1, point_3 const &p2, point_3 const &scale)
        {
           point_3 pp ;
           return cg::sqr(pp.x * scale.x) + cg::sqr(pp.y * scale.y) + cg::sqr(pp.z * scale.z) ;
        }

        static double distance_sqr(segment_3 const &S, point_3 const &P, point_3 const &scale)
        {
            if (eq(S.P0(), S.P1()))
                  return distance_sqr(S.P0(), P, scale);
          
	         double t = S(P);

	         point_3 Pt = t < 0 ? S.P0() : t > 1 ? S.P1() : S(t);

	         return distance_sqr(P, Pt, scale);
        }

    };


    template< class Traits >
        inline bool distance_ex(double maxdist, 
                                typename Traits::triangle_3 const &tr, 
                                typename Traits::point_3 const &p, 
                                double &dist, 
                                typename Traits::point_3 &resp, 
                                Traits traits = Traits())
    {
        return distance_to_triangle< Traits >::distance(maxdist, tr, p, dist, &resp, traits);
    }

    template< class Traits >
        inline bool distance(double maxdist, 
                             typename Traits::triangle_3 const &tr, 
                             typename Traits::point_3 const &p, 
                             double &dist, 
                             Traits traits = Traits())
    {
        return distance_to_triangle< Traits >::distance(maxdist, tr, p, dist, NULL, traits);
    }


    template< class Traits >
        inline bool scaled_distance_ex(double maxdist, 
                                       typename Traits::triangle_3 const &tr, 
                                       typename Traits::point_3 const &p, 
                                       double &dist, 
                                       typename Traits::point_3 &resp,
                                       Traits traits = Traits(), 
                                       typename Traits::point_3 scale = typename Traits::point_3(1, 1, 1))
    {
        return scaled_distance_to_triangle< Traits >::distance(maxdist, tr, p, scale, dist, &resp, traits);
    }

    template< class Traits >
        inline bool scaled_distance(double maxdist, 
                                    typename Traits::triangle_3 const &tr, 
                                    typename Traits::point_3 const &p, 
                                    double &dist, 
                                    Traits traits = Traits(), 
                                    typename Traits::point_3 scale = typename Traits::point_3(1, 1, 1))
    {
        return scaled_distance_to_triangle< Traits >::distance(maxdist, tr, p, scale, dist, NULL, traits);
    }

}