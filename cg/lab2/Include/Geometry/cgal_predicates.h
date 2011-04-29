#pragma once

#pragma warning (push)
#pragma warning (disable : 4244 4541 4996 4670 4673)

#include <CGAL\Exact_predicates_inexact_constructions_kernel.h>

#pragma warning (pop)

#include "Geometry\cgal\enum.h"
#include "Geometry\segment_2_intersection.h"
#include "Geometry\primitives\line.h"

namespace cg
{

enum VecOrientation
{
   VO_LEFT = 0,
   VO_RIGHT,
   VO_COLLINEAR
};

namespace details
{
   typedef CGAL::Exact_predicates_inexact_constructions_kernel CGAL_Kernel;

   inline CGAL_Kernel::Point_2 construct_cgal_point( cg::point_2 const &p )
   {
      return CGAL_Kernel::Point_2 (p.x, p.y);
   }

   inline CGAL_Kernel::Exact_kernel::Point_2 construct_cgal_exact_point( cg::point_2 const &p )
   {
      return CGAL_Kernel::Exact_kernel::Point_2 (p.x, p.y);
   }

   inline CGAL_Kernel::Vector_2 construct_cgal_vector( cg::point_2 const &p )
   {
      return CGAL_Kernel::Vector_2 (p.x, p.y);
   }

   inline CGAL_Kernel::Segment_2 construct_cgal_segment( cg::segment_2 const &s )
   {
      return CGAL_Kernel::Segment_2 (construct_cgal_point(s.P0()),
                                     construct_cgal_point(s.P1()));
   }

   inline CGAL_Kernel::Exact_kernel::Segment_2 construct_cgal_exact_segment( cg::segment_2 const &s )
   {
      return CGAL_Kernel::Exact_kernel::Segment_2 (construct_cgal_exact_point(s.P0()),
                                         construct_cgal_exact_point(s.P1()));
   }

   inline CGAL_Kernel::Line_2 construct_cgal_line( cg::line_2 const &l )
   {
      return CGAL_Kernel::Line_2 (construct_cgal_point(l.p()), CGAL_Kernel::Direction_2 (l.r().x, l.r().y));
   }

   inline bool robust_left_turn( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
   {
      return CGAL::left_turn(construct_cgal_point(a), construct_cgal_point(b), construct_cgal_point(c));
   }

   inline bool robust_right_turn( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
   {
      return CGAL::right_turn(construct_cgal_point(a), construct_cgal_point(b), construct_cgal_point(c));
   }

   inline bool robust_collinear( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
   {
      return CGAL::collinear(construct_cgal_point(a), construct_cgal_point(b), construct_cgal_point(c));
   }

   inline VecOrientation robust_orientation( cg::point_2 const &p, cg::point_2 const &q )
   {
      CGAL_Kernel::Orientation orien = CGAL::orientation(construct_cgal_vector(p), construct_cgal_vector(q));
      switch (orien)
      {
      case LEFT_TURN:
         return VO_LEFT;
      case RIGHT_TURN:
         return VO_RIGHT;
      }
      return VO_COLLINEAR;
   }

   inline VecOrientation robust_orientation( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
   {
      CGAL_Kernel::Orientation orien = CGAL::orientation(construct_cgal_point(a), construct_cgal_point(b), construct_cgal_point(c));
      switch (orien)
      {
      case LEFT_TURN:
         return VO_LEFT;
      case RIGHT_TURN:
         return VO_RIGHT;
      }
      return VO_COLLINEAR;
   }

   inline bool robust_collinear_are_ordered_along_line( cg::point_2 const &p, cg::point_2 const &q, cg::point_2 const &r )
   {
      return CGAL::collinear_are_ordered_along_line(construct_cgal_point(p), construct_cgal_point(q), construct_cgal_point(r));
   }

   inline cg::point_2 convert_point_to_basis( cg::point_2 const &origin,
                                              std::pair< cg::point_2, cg::point_2 > const &basis, // is orthonormal
                                              cg::point_2 const &input )
   {
      return cg::point_2 ((input - origin) * basis.first, (input - origin) * basis.second);
   }

   inline cg::intersection_type exact_isect_segments( cg::segment_2 const &s, cg::segment_2 const &t, cg::point_2 &r1, cg::point_2 &r2 )
   {
      CGAL_Kernel::Exact_kernel::Intersect_2 intersect_obj;
      CGAL::Object result = intersect_obj(construct_cgal_exact_segment(s), construct_cgal_exact_segment(t));

      CGAL_Kernel::Exact_kernel::Point_2 pt;
      CGAL_Kernel::Exact_kernel::Segment_2 iseg;
      if (CGAL::assign(pt, result))
      {
         if (s.P0() == t.P0() || s.P0() == t.P1())
         {
            r1 = s.P0();
            return cg::intersect;
         }
         else if (s.P1() == t.P0() || s.P1() == t.P1())
         {
            r1 = s.P1();
            return cg::intersect;
         }

         CGAL_Kernel::Point_2 ipoint = CGAL::Cartesian_converter<CGAL_Kernel::Exact_kernel, CGAL_Kernel>()(pt);
         r1 = cg::point_2 (ipoint.x(), ipoint.y());

         cg::rectangle_2 sRect (s.P0(), s.P1());
         cg::rectangle_2 tRect (t.P0(), t.P1());

         r1 = (sRect & tRect).closest_point(r1);

         return cg::intersect;
      }
      else if (CGAL::assign(iseg, result))
      {
         CGAL_Kernel::Segment_2 isegment = CGAL::Cartesian_converter<CGAL_Kernel::Exact_kernel, CGAL_Kernel>()(iseg);
         r1 = cg::point_2( isegment[0].x(), isegment[0].y() );
         r2 = cg::point_2( isegment[1].x(), isegment[1].y() );

         return cg::overlap;
      }

      return cg::disjoint;
   }

   inline cg::intersection_type robust_isect_segments( cg::segment_2 const &s, cg::segment_2 const &t, cg::point_2 &r1, cg::point_2 &r2 )
   {
      CGAL_Kernel::Point_2 ipoint;
      CGAL_Kernel::Segment_2 iseg, iseg1, iseg2;

      iseg1 = construct_cgal_segment(s);
      iseg2 = construct_cgal_segment(t);

      typedef CGAL::CGALi::Segment_2_Segment_2_pair<CGAL_Kernel> is_t;
      is_t ispair(&iseg1, &iseg2);
      switch (ispair.intersection_type()) {
       case is_t::NO_INTERSECTION:
       default:
          {
             // Additional check

             cg::rectangle_2 sRect (s.P0(), s.P1());
             cg::rectangle_2 tRect (t.P0(), t.P1());

             if (!cg::has_intersection(sRect, tRect))
                return cg::disjoint;

             VecOrientation orient0 = robust_orientation(s.P0(), s.P1(), t.P0());
             VecOrientation orient1 = robust_orientation(s.P0(), s.P1(), t.P1());

             if (orient0 != VO_COLLINEAR && orient1 != VO_COLLINEAR && orient0 == orient1)
                return cg::disjoint;

             orient0 = robust_orientation(t.P0(), t.P1(), s.P0());
             orient1 = robust_orientation(t.P0(), t.P1(), s.P1());

             if (orient0 != VO_COLLINEAR && orient1 != VO_COLLINEAR && orient0 == orient1)
                return cg::disjoint;

             return exact_isect_segments(s, t, r1, r2);
          }
       case is_t::POINT:
          {
             ipoint = ispair.intersection_point();
             if (s.P0() == t.P0() || s.P0() == t.P1())
             {
                r1 = s.P0();
                return cg::intersect;
             }
             else if (s.P1() == t.P0() || s.P1() == t.P1())
             {
                r1 = s.P1();
                return cg::intersect;
             }

             r1 = cg::point_2 (ipoint.x(), ipoint.y());

             double
                adx = s.P0().x - s.P1().x,
                ady = s.P0().y - s.P1().y,
                bdx = t.P0().x - t.P1().x,
                bdy = t.P0().y - t.P1().y;

             double crossA = adx * bdy;
             double crossB = bdx * ady; // cross = crossA - crossB

             double snorm = 0, tnorm = 0;
             double const cross_check_eps = 1e-5;
             bool nearly_collinear = false;
             if (cg::eq_rel(crossA, crossB, cross_check_eps))
             {
                nearly_collinear = true;
                snorm = cg::norm(s.P1() - s.P0());
                tnorm = cg::norm(t.P1() - t.P0());
             }

             if (nearly_collinear &&
                (!cg::eq_zero(tnorm) || !cg::eq_zero(snorm)))
             {
                // Segments are nearly collinear
                return exact_isect_segments(s, t, r1, r2);
             }

             cg::rectangle_2 sRect (s.P0(), s.P1());
             cg::rectangle_2 tRect (t.P0(), t.P1());

             r1 = (sRect & tRect).closest_point(r1);

             return cg::intersect;
          }
       case is_t::SEGMENT:
          {
             iseg = ispair.intersection_segment();
             r1 = cg::point_2( iseg[0].x(), iseg[0].y() );
             r2 = cg::point_2( iseg[1].x(), iseg[1].y() );

             return cg::overlap;
          }
      }
   }

} // End of 'details' namespace


inline bool robust_left_turn_strict( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c ) 
{
   return details::robust_left_turn(a, b, c);
}

inline bool robust_right_turn_strict( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c ) 
{
   return details::robust_right_turn(a, b, c);
}

inline bool robust_collinear( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c ) 
{
   return details::robust_collinear(a, b, c);
}

inline bool robust_isect_segments( cg::segment_2 const &s, cg::segment_2 const &t, cg::point_2 &res )
{
   cg::point_2 dummy;
   return details::robust_isect_segments(s, t, res, dummy) != cg::disjoint;
}

inline cg::intersection_type robust_isect_segments( cg::segment_2 const &s, cg::segment_2 const &t, cg::point_2 &r1, cg::point_2 &r2 )
{
   return details::robust_isect_segments(s, t, r1, r2);
}

inline bool exact_isect_segments( cg::segment_2 const &s, cg::segment_2 const &t, cg::point_2 &res )
{
   cg::point_2 dummy;
   return details::exact_isect_segments(s, t, res, dummy) != cg::disjoint;
}

inline cg::intersection_type exact_isect_segments( cg::segment_2 const &s, cg::segment_2 const &t, cg::point_2 &r1, cg::point_2 &r2 )
{
   return details::exact_isect_segments(s, t, r1, r2);
}

inline VecOrientation robust_orientation( cg::point_2 const &p, cg::point_2 const &q )
{
   return details::robust_orientation(p, q);
}

inline VecOrientation robust_orientation( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
{
   return details::robust_orientation(a, b, c);
}

inline bool robust_left_turn( cg::point_2 const &p, cg::point_2 const &q )
{
   VecOrientation orien = robust_orientation(p, q);
   return orien == VO_LEFT || orien == VO_COLLINEAR;
}

inline bool robust_left_turn( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
{
   VecOrientation orien = robust_orientation(a, b, c);
   return orien == VO_LEFT || orien == VO_COLLINEAR;
}

inline bool robust_right_turn( cg::point_2 const &p, cg::point_2 const &q )
{
   VecOrientation orien = robust_orientation(p, q);
   return orien == VO_RIGHT || orien == VO_COLLINEAR;
}

inline bool robust_right_turn( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
{
   VecOrientation orien = robust_orientation(a, b, c);
   return orien == VO_RIGHT || orien == VO_COLLINEAR;
}

inline bool robust_collinear_are_ordered_along_line( cg::point_2 const &a, cg::point_2 const &b, cg::point_2 const &c )
{
   return details::robust_collinear_are_ordered_along_line(a, b, c);
}

inline bool is_point_on_segment( cg::point_2 const &p, cg::segment_2 const &seg )
{
   if (!robust_collinear(seg.P0(), p, seg.P1()))
      return false;

   return robust_collinear_are_ordered_along_line(seg.P0(), p, seg.P1());
}

} // End of 'cg' namespace
