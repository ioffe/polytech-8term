#pragma once

#include "primitives\point.h"
#include "primitives\range.h"
#include "segment_2_intersection.h"

#include "Geometry/segment_2_intersection.h"
#include "Geometry/point_ops.h"

namespace cg
{
    // для ускорения перевода из float в double
    struct triangle_3_rti
    {
        point_3 const & normal () const { return n; }
        point_3 const & v1     () const { return v1_; }
        point_3 const & v2     () const { return v2_; }
        point_3 const & v3     () const { return v3_; }

        template< class Point >
            triangle_3_rti( Point const &v1, Point const &v2, Point const &v3, Point const &n )
		         : v1_(v1), v2_(v2), v3_(v3), n(n)
        {}
        
        template <class Traits>
            bool collinear_has_on(point_3 const &pt, Traits const &traits) const
        {
            typename Traits::point_in_triangle 
                point_in_triangle = traits.point_in_triangle_object();

            if ( !eq( normal().z, 0. ) )
            {
                return point_in_triangle( *this, pt, traits.xy() );
            } else
            if ( !eq( normal().y, 0. ) )
            {
                return point_in_triangle( *this, pt, traits.xz() );
            } else
            {
                return point_in_triangle( *this, pt, traits.yz() );
            }
        }

    private:
        point_3 v1_, v2_, v3_, n;
    };

	inline range_2 calc_zrange( triangle_3_rti const & t )
	{
		return range_2( t.v1( ).z, t.v2( ).z ).unite( t.v3( ).z );
	}

    template <class base>
        struct proj_3_to_2_base : base
    {
        typedef typename base::point_3      point_2;
        typedef typename base::triangle_3   triangle_2;
    };

    template <class base>
        struct xy_traits : proj_3_to_2_base<base>
    {
	    static double diff_x(point_2 const &a, point_2 const &b) { return b.x - a.x; }
	    static double diff_y(point_2 const &a, point_2 const &b) { return b.y - a.y; }
    };

    template <class base>
        struct xz_traits : proj_3_to_2_base<base>
    {
	    static double diff_x(point_2 const &a, point_2 const &b) { return b.x - a.x; }
	    static double diff_y(point_2 const &a, point_2 const &b) { return b.z - a.z; }
    };

    template <class base>
        struct yz_traits : proj_3_to_2_base<base>
    {
	    static double diff_x(point_2 const &a, point_2 const &b) { return b.y - a.y; }
	    static double diff_y(point_2 const &a, point_2 const &b) { return b.z - a.z; }
    };

    template <class Traits>
        inline bool right_turn(
            typename Traits::point_2 const &a, 
            typename Traits::point_2 const &b, 
            typename Traits::point_2 const &c, 
            Traits const &traits = Traits())
    {
	    return 
		    traits.diff_x(a,b) * traits.diff_y(a,c) 
	     <= traits.diff_y(a,b) * traits.diff_x(a,c);
    }


    struct point_in_triangle
    {
        template <class Traits>
            bool operator () (
                typename Traits::triangle_2 const &t, 
                typename Traits::point_2 const &pt, 
//                barycentric_coords & bc,
                Traits const &traits) const
        {
//            triangle_raster_aux tr_aux(t);
//            return tr_aux.IsInTriangleBarocenteric( pt, bc );

            // TODO: calc barocenteric coords!!!
            bool r_1 = right_turn(t.v1(), t.v2(), pt, traits),
                 r_2 = right_turn(t.v2(), t.v3(), pt, traits),
                 r_3 = right_turn(t.v3(), t.v1(), pt, traits);

            return (r_1 && r_2 && r_3) || (!r_1 && !r_2 && !r_3);
        }
    };

    struct segment_3_ref
    {
        segment_3_ref(point_3 const &org, point_3 const &dir) : org(org), dir(dir) {}

        point_3 operator () (double t) const  
        {
            return org + dir * t;
        }

        point_3 const org;
        point_3 const dir;
    };

    template <class Traits>
        struct segment_plane_intersection
    {
        typedef typename Traits::FT         FT;
        typedef typename Traits::point_3    point_3;
        typedef typename Traits::segment_3  segment_3;
        typedef typename Traits::triangle_3 triangle_3;

        segment_plane_intersection(
            segment_3  const &segment,
            triangle_3 const &plane) : segment(segment), plane(plane)
        {}

    private:
        void calcdot() const {
            dot = segment.dir * plane.normal();
        }

        // requires: dot calced
        void calcratio() const {
            ratio = (plane.v1() - segment.org) * plane.normal() / dot;
        }

    public:
        bool collinear() const {
            calcdot();
            return eq(dot, 0.);
        }

        bool collinear_nobackface() const {
            calcdot();
//            return dot <= epsilon;
            return dot >= -epsilon<double>( ) ;
        }

        // requires: ratio calced
        bool intersects() const {
            calcratio();
            return 0 <= ratio && ratio <= 1;
        }

        FT getratio() const {
            return ratio;
        }

        point_3 getpoint() const {
            return segment(ratio);
        }

    private:
        segment_3  const &segment;
        triangle_3 const &plane;

        mutable FT        dot;
        mutable FT        ratio;
    };

    template< class Triangle = triangle_3_rti >
        struct cartesian
    {
        typedef double         FT;
	    typedef point_3        point_3;
        typedef segment_3_ref  segment_3;
        typedef Triangle       triangle_3;

        typedef xy_traits<cartesian>    xy_traits;
        typedef xz_traits<cartesian>    xz_traits;
        typedef yz_traits<cartesian>    yz_traits;

	    static xy_traits xy() { return xy_traits(); }
	    static xz_traits xz() { return xz_traits(); }
	    static yz_traits yz() { return yz_traits(); }

        typedef point_in_triangle                                   point_in_triangle;
        typedef segment_plane_intersection< cartesian< Triangle > > segment_plane_intersection;

        point_in_triangle point_in_triangle_object() const {
            return point_in_triangle();
        }

        static segment_plane_intersection 
            segment_plane_intersection_object(
                segment_3 const &segment, triangle_3 const &triangle)
        {
            return segment_plane_intersection(segment, triangle);
        }
    };

    template <class Traits>
	    inline bool ray_triangle3_intersection( 
            typename Traits::segment_3  const &segment,
            typename Traits::triangle_3 const &triangle,
            typename Traits::FT               &ratio,
            bool   nobackface,
            Traits const &traits = Traits()  )
    {
        // ToDo: вставить проверку на вырожденность
        
        // ToDo: правильно считать bc!!!!
//        bc.alpha = bc.beta = 0.3;

        typedef typename Traits::point_3     point_3;
        typedef typename Traits::FT          FT;

        typename Traits::segment_plane_intersection isec
            = traits.segment_plane_intersection_object(segment, triangle);

        if (nobackface)
        {
            if (isec.collinear_nobackface())
                return false;
        } else
        {
            if (isec.collinear())
                return false;
        }

        if (!isec.intersects())
            return false;

        ratio = isec.getratio();

        point_3 ipt = isec.getpoint();

        return triangle.collinear_has_on(ipt, traits);
    }

    template <class Traits>
	    inline bool ray_triangle3_intersection( 
            typename Traits::point_3 const &org,
            typename Traits::point_3 const &dir,
            typename Traits::triangle_3 const &triangle,
            typename Traits::FT            &ratio,
            bool   nobackface,
            Traits const &traits = Traits()  )
    {
        return ray_triangle3_intersection(
            segment_3_ref(org, dir), triangle, ratio, nobackface, traits);
    }

    template <class Traits>
        inline bool ray_triangle3_intersection( 
            typename segment_3 const &seg,
            typename Traits::triangle_3 const &triangle,
            typename Traits::FT            &ratio,
            bool   nobackface,
            Traits const &traits = Traits()  )
    {
        return ray_triangle3_intersection(
            segment_3_ref(seg.P0(), seg.P1() - seg.P0()), triangle, ratio, nobackface, traits);
    }

    inline point_3 pt_triangle( point_2 const & pt, triangle_3 const & tr )
    {
      point_3 n = cg::normalized( ( tr[ 1 ] - tr[ 0 ] ) ^ ( tr[ 2 ] - tr[ 0 ] ) );

      Assert( !cg::eq_zero( n.z ) );

      return point_3( pt, ( ( tr[ 0 ] * n ) - ( pt * ( ( point_2 ) n ) ) ) / n.z );
    }

    inline bool in_triangle( cg::triangle_3 const & t, point_2 const & p )
    {
      point_2 a( t[ 1 ] - t[ 0 ] );
      point_2 b( t[ 2 ] - t[ 0 ] );
      point_2 c(   p    - t[ 0 ] );

      double ab = a ^ b;

      if ( cg::eq_zero( ab ) )
        return false;

      double cb = c ^ b;
      double ca = c ^ a;

      double alpha =   cb / ab;
      double beta  = - ca / ab;

      const double eps = epsilon< double >( ) ;
      return ( alpha > - eps && beta > - eps && ( alpha + beta - 1. < eps ) );
    }

    inline DWORD vertical_plane_intersection( cg::segment_2 const & seg, cg::triangle_3 const & trg, cg::point_3 * p0, cg::point_3 * p1 )
    {
      Assert( p0 );
      Assert( p1 );

      double t[3] = { -1., -1., -1. };

      for ( size_t l = 0; l < 3; ++l )
      {
        segment_2 side( trg[ l ], trg[ ( l + 1 ) % 3 ] );
        point_2 pt1, pt2;

        cg::intersection_type type = cg::generic_intersection( seg, side, &pt1, &pt2 );

        if ( type == cg::intersect )
        {
          t[ l ] = side( pt1 );
        } 
        else if ( type == cg::overlap )
        {
          cg::segment_3 side_3( trg[ l ], trg[ ( l + 1 ) % 3 ] );
          *p0 = side_3( side( pt1 ) ) ;
          *p1 = side_3( side( pt2 ) ) ;

          return 2;
        }
      }

      // overlap'а нет
      DWORD res = 0;

      for ( size_t l = 0; l < 3; ++l )
      {
        if ( t[ l ] > -1e-4 && t[ l ] - 1 < 1e-4 )
        {
          point_3 pt( cg::segment_3( trg[ l ], trg[ ( 1 + l ) % 3 ] )( t[ l ] ) );

          if ( res == 0 )
          {
            *p0 = pt;
            res = 1;
          } 
          else if ( res == 1 )
          {
            if ( cg::eq( *p0, pt, 1e-3 ) )
              continue;

            *p1 = pt;
            res = 2;

            break;
          }
        }
      }

      if ( res == 0 )
      {
        // а не полностью ли в треугольнике лежит сегмент?
        if ( in_triangle( trg, seg.P0( ) ) && in_triangle( trg, seg.P1( ) ) )
        {
          *p0 = pt_triangle( seg.P0( ), trg );
          *p1 = pt_triangle( seg.P1( ), trg );

          res = 2;
        }
      }

      if ( res == 1 )
      {
        // а не лежит ли одна из вершин сегмента в треугольнике?
        if ( in_triangle( trg, seg.P0( ) ) && !cg::eq( static_cast< point_2 const & >( *p0 ), seg.P0( ) ) )
        {
          *p1 = pt_triangle( seg.P0( ), trg );
          res = 2;
        }
        else if ( in_triangle( trg, seg.P1( ) ) && !cg::eq( static_cast< point_2 const & >( *p0 ), seg.P1( ) ) )
        {
          *p1 = pt_triangle( seg.P1( ), trg );
          res = 2;
        }
      }

      return res;
    }

}
