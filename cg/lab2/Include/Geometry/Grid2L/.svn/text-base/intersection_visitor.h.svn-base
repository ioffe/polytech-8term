#pragma  once
#include "Grid2l_Impl.h"

namespace cg 
{
  enum IntersectionType {
    IT_INTERSECT  = 1,
    IT_SELFINTERSECT = 2,
    IT_OVERLAP = 3
  };

  // Visits grid, checks intersections between contours
  // You can inherit from it to process "processIntersectionMessage" message.
  template <class Container, class Grid>
  class BaseIntersectionVisitor : public grid2l_visitor_base<Grid, BaseIntersectionVisitor>
  {
  public:
    BaseIntersectionVisitor (Container & c, bool checkE)
      : con(c)
      , checkEnds(checkE)
    {
      eps = 0.0000001;
    }

    void setEps(double ne)
    {
      eps = ne;
    }
    
  	template <class Location>
      bool operator () (Location const & location, typename Grid::smallcell_type const & smallcell) 
    {
      checkIntersect<Grid::smallcell_type::Segments::const_iterator>(smallcell.segments().begin(), smallcell.segments().end(), location.smallcellbound(), con);
		  return false;
	  }

  protected:
    Container & con;
    bool checkEnds;
    double eps;

    virtual void processIntersectionMessage(IntersectionType t, typename Container::segment_id a, typename Container::segment_id b, point_2 & p) = 0;

    template < class FwdIter >
      void checkIntersect( FwdIter p, FwdIter q, rectangle_2 & currentCellBound, Container & gen) 
	  {
		  for ( FwdIter a = p; a != q; ++a )
			  for ( FwdIter b = p; b != q; ++b ) 
			  {
          typename Container::segment_id i = *a;
				  typename Container::segment_id j = *b;

				  bool sameCon = (gen.getContourBySegment(i) == gen.getContourBySegment(j));
				  if (i<j) 
				  {
            cg::point_2 p;
            cg::point_2 p1;
            cg::segment_2 s1 = gen.getSegment(i);
            cg::segment_2 s2 = gen.getSegment(j);

            if (!has_intersection(s1, s2))
              continue;

            cg::intersection_type intersects = generic_intersection<point_2>(s1, s2, &p, &p1);

            if (!checkEnds) {
              if (distance_sqr(s1.P0(), p) < eps*eps || 
                  distance_sqr(s1.P1(), p) < eps*eps ||
                  distance_sqr(s2.P0(), p) < eps*eps || 
                  distance_sqr(s2.P1(), p) < eps*eps  )
                continue;
            }

            if (currentCellBound.contains(p)) {

                point_2 p1b = gen.getPoint(gen.pointsBegin(gen.getContourBySegment(i)));
                point_2 p2b = gen.getPoint(gen.pointsBegin(gen.getContourBySegment(j)));

                if (gen.getContour(gen.getContourBySegment(i)).non_closed()&&
                        (s1.P1() == p1b)
                  )
                {
                  continue; 
                }
                if (gen.getContour(gen.getContourBySegment(j)).non_closed()&&
                        (s2.P1() == p2b)
                  )
                {
                  continue; 
                }


					      if (!sameCon) 
					      {
						      if (intersects != disjoint) 
                    processIntersectionMessage(IT_INTERSECT,i,j,p);
					      } 
					      else 
					      {
						      if ( 
								      (gen.getSegmentEndPoint(i) != gen.getSegmentStartPoint(j)) 
								      && 
								      (gen.getSegmentEndPoint(j) != gen.getSegmentStartPoint(i)) 
							      )
						      {
							      if (intersects != disjoint) 
                      processIntersectionMessage(IT_SELFINTERSECT,i,j,p);
						      } 
						      else 
						      {
							      if (intersects == overlap) 
                      processIntersectionMessage(IT_OVERLAP,i,j,p);
						      }
					      }
					  }
				  }

			  }
      }

  };
}