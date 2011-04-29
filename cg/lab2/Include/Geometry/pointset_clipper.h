#pragma once

//#include "Grid1L.h"
#include "primitives\point.h"

namespace cg
{

    template <class Point> // ,struct PointComparer
        struct PointsetClipper
    {

    private:
        struct PointEx
        {
            Point p;
            int idx;
            PointEx( Point const &p, int idx )
                : p(p), idx(idx)
            {}
        };

        struct StdPointComparer
        {
            bool operator() (PointEx const &p1, PointEx const &p2) const
            {
                return p1.p.y <  p2.p.y
                   ||  p1.p.y == p2.p.y  &&  p1.p.x < p2.p.x;
            }
        };

        typedef std::set<PointEx, StdPointComparer> PointSearcher;
        typedef typename PointSearcher::const_iterator       PCIT;

//        typedef Grid1L<Point> GridType;

    public:

        PointsetClipper( double eps/*, int p_num, rectangle_2 const &domain*/ )
            : m_eps ( eps )
        {
            Assert( eps > 0.0 );
/*
            point_2i ext = 
            point_2  org = domain.xy();

            m_grid = new GridType( aa_transform( domain.origin(), domain.unit() ),
                                   point_2i( 
*/
        }
/*
        int Add(Point &p)
        {

        }
*/
        void SetEps( double eps )
        {
            Assert( eps > 0.0 );
            m_eps = eps;
        }

        void ClearAll()
        {
            m_pts.clear();
        }

        int Add(Point &p)
        {
            ConvertPt(p);
            PointEx p_ex( p, m_pts.size() );
            PCIT it = m_pts.find( p_ex );

            if (m_pts.end() == it)
            {
                m_pts.insert( p_ex );
                return -1;
            }

            return it->idx;
        }
/*
        void AddPoint(Point const &p)
        {
            if (m_pts.end() == m_pts.find( ConvertPt(p) ))
                m_pts.insert( p );
        }

        PCIT begin() const
        {
            return m_pts.begin();
        }

        PCIT end() const
        {
            return m_pts.end();
        }
*/

    private:
        void ConvertPt(Point &p)
        {
            p = Point( floor(p / m_eps) ) * m_eps;
        }


    private:
        PointSearcher m_pts;
        double m_eps;
//        m_ptr<GridType> m_grid;
    };

}

