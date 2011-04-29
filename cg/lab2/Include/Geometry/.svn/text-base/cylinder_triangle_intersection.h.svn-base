#pragma once

namespace cg
{
    namespace cylinder_intersection_details
    {

        template<class Triangle>
            struct std_traits
        {
            typedef double           FT;
            typedef point_3          point_3;
            typedef segment_3        segment_3;
            typedef Triangle         triangle_3;


            typedef xy_traits<std_traits>    xy_traits;
            typedef xz_traits<std_traits>    xz_traits;
            typedef yz_traits<std_traits>    yz_traits;
            typedef point_in_triangle                      point_in_triangle;

            point_3 v0    (triangle_3 const& tr) const { return tr[0]; }
            point_3 v1    (triangle_3 const& tr) const { return tr[1]; }
            point_3 v2    (triangle_3 const& tr) const { return tr[2]; }
            point_3 normal(triangle_3 const& tr) const { return tr.normal(); }

            static xy_traits xy() { return xy_traits(); }
            static xz_traits xz() { return xz_traits(); }
            static yz_traits yz() { return yz_traits(); }

            bool collinear_has_on(point_3 const &pt, triangle_3 const& tr) const
            {
                cg::point_in_triangle in_triangle;
                if ( !eq( normal(tr).z, 0. ) )
                {
                    return in_triangle( tr, pt, xy() );
                } 
                else if ( !eq( normal(tr).y, 0. ) )
                {
                    return in_triangle( tr, pt, xz() );
                } 
                else
                {
                    return in_triangle( tr, pt, yz() );
                }
            }
        };

        template<class Traits>
            struct intersection
        {
            typedef typename Traits::FT             FT;
            typedef typename Traits::point_3        point_3;
            typedef typename Traits::segment_3      segment_3;
            typedef typename Traits::triangle_3     triangle_3;


            static bool intersect( segment_3 const& length, FT radius, triangle_3 const& tri,
                FT& ratio, point_3& p, Traits const &t = Traits() )
            {
                point_3 org = length.P0();
                double len_norm = norm(length.P1() - length.P0());

                if(eq_zero(len_norm))
                    return false;

                point_3 dir = (length.P1() - length.P0()) / len_norm;

                bool res = false;

                point_3 pp;
                if(cylinder_plane_intersection(length, radius, t.v0(tri), t.normal(tri), pp))
                {
                    if(t.collinear_has_on(pp, tri))
                    {
                        ratio = dist_to_plane(length.P0(), dir, pp) / len_norm;
                        p = pp;
                        res = true;
                    }
                }

                if(!res)
                {
                    std::vector<segment_3> segs;
                    if(!res && get_clipped_segments(segs, length, dir, tri, t))
                    {
                        double ratio_ = 1e10;
                        for(DWORD i = 0; i < segs.size(); i++)
                        {
                            double t1, t2;
                            double r;
                            if(!eq_zero(norm(segs[i].P1() - segs[i].P0())) &&
                                cylinder_line_intersection(length, dir, radius, segs[i], t1, t2))
                            {
                                if(t1 <= 1 && t2 >= 0)
                                {
                                    if(t1 < 0) t1 = 0;
                                    if(t2 > 1) t2 = 1;

                                    point_3 p1 = segs[i](t1);
                                    point_3 p2 = segs[i](t2);

                                    double d0 = max(dist_to_plane(length.P0(), dir, p1), 0.);
                                    double d1 = max(dist_to_plane(length.P0(), dir, p2), 0.);
                                    //Assert(d0 >= 0 && d1 >= 0);
                                    if(d0 <= d1)
                                    {
                                        res = true;
                                        r = d0 / len_norm;
                                        if(r < ratio_)  ratio = ratio_ = r, p = p1;
                                    }
                                    else
                                    {
                                        res = true;
                                        r = d1 / len_norm;
                                        if(r < ratio_)  ratio = ratio_ = r, p = p2;
                                    }
                                }
                            }
                        }
                    }
                }

                /*if(res)
                {
                    double dist = point_line_distance(length.P0(), dir, p);
                    if(!le(dist, radius + 0.00001) || !ge(ratio, 0) || !le(ratio, 1))
                    {
                        DebugBreak();
                        intersect(length, radius, tri, ratio, p, t);
                    }
                }*/


                return res;
            }

            static double dist_to_plane(point_3 const& org, point_3 const& dir, point_3 const& pt)
            {
                return (dir * (pt - org));
            }

            static bool cylinder_plane_intersection(segment_3 const& length, FT radius, 
                point_3 const& org, point_3 const& dir, point_3& p)
            {
                double r;
                if(!line_plane_intersection(length, org, dir, r))
                   return false;

                point_3 l = normalized(length.P1() - length.P0());
                point_3 o = length(r);

                point_3 k = l ^ dir;
                if(eq_zero(norm(k)))
                {
                    segment_3 s(o, o + point_3(1,1,1) - ((o + point_3(1,1,1)) * l) * l);
                    double t1, t2;
                    if(cylinder_line_intersection(length, l, radius, s, t1, t2))
                    {
                        if(t1 < 0) t1 = 0;
                        if(t1 > 1) t1 = 1;
                        p = s(t1);
                        return true;
                    }
                }
                else
                {
                    point_3 m = k ^ dir;
                    segment_3 s(o, o + m);
                    double t1, t2;
                    if(cylinder_line_intersection(length, l, radius, s, t1, t2))
                    {
                        if(t1 <= 1 && t2 >= 0)
                        {
                            if(t1 < 0) t1 = 0;
                            if(t2 > 1) t2 = 1;
                            point_3 p1 = s(t1);
                            point_3 p2 = s(t2);

                            double d0 = dist_to_plane(length.P0(), l, p1);
                            double d1 = dist_to_plane(length.P0(), l, p2);
                            if(d0 > d1)
                                p = p1;
                            else
                                p = p2;
                            return true;
                        }
                    }
                }

                return false;
            }

            static bool cylinder_plane_intersection(segment_3 const& length, FT radius, 
                point_3 const& org, point_3 const& dir, FT& ratio, point_3& p)
            {
                if(cylinder_plane_intersection(length, radius, org, dir, p))
                {
                    ratio = dist_to_plane(length.P0(), dir, p) / norm(length.P1() - length.P0());
                    return true;
                }

                return false;
            }

            static bool cylinder_line_intersection(segment_3 const& c_len, point_3 const& c_dir, 
                double c_radius, segment_3 const& seg, double &t0, double &t1)
            {
                point_3 l = normalized(seg.P1() - seg.P0());
                point_3 n = normalized(c_dir ^ l);

                if(eq_zero(norm_sqr(n)))
                {
                    if(point_line_distance(c_len.P0(), c_dir, seg.P0()) <= c_radius)
                    {
                        t0 = get_int_ratio(seg, c_len.P0(), c_dir);
                        t1 = get_int_ratio(seg, c_len.P0(), c_dir);

                        if(t0 > t1)
                            swap(t0, t1);
                        return true;
                    }
                }
                else
                {
                    point_3 k = c_dir ^ n;
                    double y0 = k * (seg.P0() - c_len.P0());
                    double y1 = k * (seg.P1() - c_len.P0());
                    double z = n * (seg.P0() - c_len.P0());
                    double dy = y1 - y0;
                    if(cg::eq_zero(dy))
                       return false;
                    double d = (c_radius * c_radius - z * z);
                    if(d >= 0)
                    {
                        d = cg::sqrt(d);
                        t0 = (-y0 - d) / dy;
                        t1 = (-y0 + d) / dy;

                        if(t0 > t1)
                            swap(t0, t1);

                        if(!clip_by_plane(c_len.P0(), c_dir, seg, t0, t1))
                            return false;
                        if(!clip_by_plane(c_len.P1(), -c_dir, seg, t0, t1))
                           return false;

                        return true;
                    }
                }

                return false;
            }

            static bool clip_by_plane(point_3 const& org, point_3 const& dir, 
                segment_3 const &seg, double& t0, double& t1)
            {
                point_3 p0 = seg(t0);
                point_3 p1 = seg(t1);
                double dist0, dist1;
                dist0 = dist_to_plane(org, dir, p0);
                dist1 = dist_to_plane(org, dir, p1);
                if(!ge(dist0, 0) && !ge(dist1, 0))
                    return false;
                else if(!ge(dist0, 0))
                    t0 = max(t0, get_int_ratio(seg, org, dir));
                else if(!ge(dist1, 0))
                    t1 = min(t1, get_int_ratio(seg, org, dir));

                return true;
            }

            static double point_line_distance(point_3 const& org, point_3 const& dir, point_3 const& pt)
            {
                return norm(dir ^ (org - pt));
            }

            static bool segment_plane_intersection(segment_3 const& seg, point_3 const& org, point_3 const& dir, double &ratio)
            {
                double term = dir * (seg.P1() - seg.P0());
                if(eq_zero(term))
                    return false;
                ratio = - (dir * (seg.P0() - org)) / term;
                return (ratio >= 0 && ratio <= 1);
            }
            static bool line_plane_intersection(segment_3 const& seg, point_3 const& org, point_3 const& dir, double &ratio)
            {
                double term = dir * (seg.P1() - seg.P0());
                if(eq_zero(term))
                    return false;
                ratio = - (dir * (seg.P0() - org)) / term;
                return true;
            }
            static double get_int_ratio(segment_3 const& seg, point_3 const& org, point_3 const& dir)
            {
                double term = dir * (seg.P1() - seg.P0());
                if(eq_zero(term))
                    return 0;
                double ratio;
                ratio = - (dir * (seg.P0() - org)) / term;
                return ratio;
            }
            static point_3 get_int_point(point_3 const& org, point_3 const& dir, segment_3 const& seg)
            {
                double term = dir * (seg.P1() - seg.P0());
                if(eq_zero(term))
                    return point_3();
                double ratio;
                ratio = - (dir * (seg.P0() - org)) / term;
                Assert(cg::ge(ratio, 0) && cg::le(ratio, 1));
                return seg(ratio);
            }

            template<class T>
                static void swap(T & val1, T & val2)
            {
                T t = val2;
                val2 = val1;
                val1 = t;
            }

            static bool get_clipped_segments(std::vector<segment_3> & segs, segment_3 const& length, 
                point_3 const& dir, triangle_3 const& tri, Traits const& t)
            {
                std::vector<point_3> v;
                v.push_back(t.v0(tri));
                v.push_back(t.v1(tri));
                v.push_back(t.v2(tri));

                double d0[3];
                double d1[3];
                d0[0] = dist_to_plane(length.P0(), dir, v[0]);
                d0[1] = dist_to_plane(length.P0(), dir, v[1]);
                d0[2] = dist_to_plane(length.P0(), dir, v[2]);
                d1[0] = dist_to_plane(length.P1(), -dir, v[0]);
                d1[1] = dist_to_plane(length.P1(), -dir, v[1]);
                d1[2] = dist_to_plane(length.P1(), -dir, v[2]);

                point_3 org0 = length.P0();
                point_3 org1 = length.P1();

                if(d0[0] >= 0 && d0[1] >= 0 && d0[2] >= 0)
                {
                    swap(d0[0], d1[0]), swap(d0[1], d1[1]), swap(d0[2], d1[2]);
                    swap(org0, org1);
                }

                int case_ = 0;
                int pts[3];
                if(d0[0] < 0 && d0[1] < 0 && d0[2] < 0)
                    return false;
                else if(d0[0] >= 0 && d0[1] >= 0 && d0[2] < 0)
                    case_ = 1, pts[0] = 2, pts[1] = 1, pts[2] = 0;
                else if(d0[0] >= 0 && d0[1] < 0 && d0[2] >= 0)
                    case_ = 1, pts[0] = 1, pts[1] = 2, pts[2] = 0;
                else if(d0[0] < 0 && d0[1] >= 0 && d0[2] >= 0)
                    case_ = 1, pts[0] = 0, pts[1] = 1, pts[2] = 2;
                else if(d0[0] < 0 && d0[1] < 0 && d0[2] >= 0)
                    case_ = 2, pts[0] = 0, pts[1] = 1, pts[2] = 2;
                else if(d0[0] >= 0 && d0[1] < 0 && d0[2] < 0)
                    case_ = 2, pts[0] = 1, pts[1] = 2, pts[2] = 0;
                else if(d0[0] < 0 && d0[1] >= 0 && d0[2] < 0)
                    case_ = 2, pts[0] = 0, pts[1] = 2, pts[2] = 1;

                if(case_ == 0)
                {
                    segs.push_back(segment_3(v[0], v[1]));
                    segs.push_back(segment_3(v[0], v[2]));
                    segs.push_back(segment_3(v[2], v[1]));
                }
                else if(case_ == 1) // снаружи одна точка
                {
                    point_3 p1 = get_int_point(org0, dir, segment_3(v[pts[0]], v[pts[1]]));
                    point_3 p2 = get_int_point(org0, dir, segment_3(v[pts[0]], v[pts[2]]));
                    segs.push_back(segment_3(p1, p2));

                    double d1_ = d1[pts[1]];
                    double d2_ = d1[pts[2]];

                    if(d1_ > 0 && d2_ > 0)
                    {
                        segs.push_back(segment_3(p1, v[pts[1]]));
                        segs.push_back(segment_3(p2, v[pts[2]]));
                        segs.push_back(segment_3(v[pts[1]], v[pts[2]]));
                    }
                    else if(d1_ <= 0 && d2_ <= 0)
                    {
                        point_3 p3 = get_int_point(org1, dir, segment_3(v[pts[0]], v[pts[1]]));
                        point_3 p4 = get_int_point(org1, dir, segment_3(v[pts[0]], v[pts[2]]));
                        segs.push_back(segment_3(p3, p4));
                        segs.push_back(segment_3(p2, p4));
                        segs.push_back(segment_3(p1, p3));
                    }
                    else if(d1_ > 0)
                    {
                        point_3 p3 = get_int_point(org1, dir, segment_3(v[pts[1]], v[pts[2]]));
                        point_3 p4 = get_int_point(org1, dir, segment_3(v[pts[0]], v[pts[2]]));
                        segs.push_back(segment_3(p1, v[pts[1]]));
                        segs.push_back(segment_3(p3, p4));
                        segs.push_back(segment_3(v[pts[1]], p3));
                        segs.push_back(segment_3(p2, p4));
                    }
                    else
                    {
                        point_3 p3 = get_int_point(org1, dir, segment_3(v[pts[0]], v[pts[1]]));
                        point_3 p4 = get_int_point(org1, dir, segment_3(v[pts[1]], v[pts[2]]));
                        segs.push_back(segment_3(p2, v[pts[2]]));
                        segs.push_back(segment_3(p3, p4));
                        segs.push_back(segment_3(v[pts[2]], p4));
                        segs.push_back(segment_3(p1, p3));
                    }
                }
                else if(case_ == 2)  // снаружи две точки
                {
                    point_3 p1 = get_int_point(org0, dir, segment_3(v[pts[0]], v[pts[2]]));
                    point_3 p2 = get_int_point(org0, dir, segment_3(v[pts[1]], v[pts[2]]));
                    segs.push_back(segment_3(p1, p2));

                    double d = d1[pts[2]];

                    if(d > 0)
                    {
                        segs.push_back(segment_3(p1, v[pts[2]]));
                        segs.push_back(segment_3(p2, v[pts[2]]));
                    }
                    else
                    {
                        point_3 p3 = get_int_point(org1, dir, segment_3(v[pts[0]], v[pts[2]]));
                        point_3 p4 = get_int_point(org1, dir, segment_3(v[pts[1]], v[pts[2]]));
                        segs.push_back(segment_3(p1, p3));
                        segs.push_back(segment_3(p2, p4));
                        segs.push_back(segment_3(p3, p4));
                    }
                }

                return true;
            }
        };
    }


    template<class Traits>
        inline bool cylinder_triangle_intersection(
            typename Traits::segment_3 const&  length,
            typename Traits::FT                radius,
            typename Traits::triangle_3 const& tri,
            typename Traits::FT&               ratio,
            typename Traits::point_3&          p,
            Traits const &traits = Traits()            )
    {
        return cylinder_intersection_details::intersection<Traits>::
            intersect(length, radius, tri, ratio, p, traits);
    }


    template<class Traits>
        inline bool cylinder_plane_intersection(
            typename Traits::segment_3 const& length, 
            typename Traits::FT radius, 
            typename Traits::point_3 const& org, 
            typename Traits::point_3 const& dir, 
            typename Traits::FT& ratio, 
            typename Traits::point_3& p)
    {
        return cylinder_intersection_details::intersection<Traits>::cylinder_plane_intersection(
            length, radius, org, dir, ratio, p);
    }

}
