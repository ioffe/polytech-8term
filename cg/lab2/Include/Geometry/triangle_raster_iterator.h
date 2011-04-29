#pragma once

#include "common/safe_bool.h"
#include "raster_2.h"
#include "primitives\triangle.h"

namespace cg 
{

    struct triangle_rasterization_iterator
    {
        triangle_rasterization_iterator( triangle_2 const &triangle )
        {
            point_2i v1 = floor(triangle[0]);
            point_2i v2 = floor(triangle[1]);
            point_2i v3 = floor(triangle[2]);

            m_iFromY = min(v1.y, v2.y, v3.y);

            // create new SIDE arrays
            m_iTriangleHeight = max(v1.y, v2.y, v3.y) - m_iFromY + 1;

            m_aLeftSide. resize( m_iTriangleHeight, point_2i(  INT_ETERNITY, 0 ) );
            m_aRightSide.resize( m_iTriangleHeight, point_2i( -INT_ETERNITY, 0 ) );

            // fill SIDE arrays
            AddLineToArray( v1, v2, triangle[0], triangle[1] );
            AddLineToArray( v1, v3, triangle[0], triangle[2] );
            AddLineToArray( v2, v3, triangle[1], triangle[2] );

            y = 0;
            x = m_aLeftSide[ y ].x;
        }

        ~triangle_rasterization_iterator()
        {
            Clear();
        }

        point_2i operator * () const {
            return point_2i( x, y+m_iFromY );
        }

        SAFE_BOOL_OPERATOR(y < m_iTriangleHeight)

        triangle_rasterization_iterator& operator ++ () 
        {
    //        NTRACE("x %d, y %d, m_iTriangleHeight %d, m_aRightSide[y].x %d", x, y, m_iTriangleHeight, m_aRightSide[y].x);

            x ++;
            if ( x > m_aRightSide[y].x )
            {
                y ++;

                if ( y < m_iTriangleHeight )
                    x = m_aLeftSide[y].x;
            }

            return *this;
        }

    private:
        void Clear()
        {
            m_aLeftSide. clear();
            m_aRightSide.clear();

            m_iTriangleHeight = 0;
            m_iFromY          = 0;
        }

        inline void AddToLeftRightArrays (point_2i const &v)
        {
            int iY = v.y - m_iFromY;
            if ( iY >= 0  &&  iY < m_iTriangleHeight )
            {
                if (m_aLeftSide [iY].x > v.x) m_aLeftSide [iY] = v;
                if (m_aRightSide[iY].x < v.x) m_aRightSide[iY] = v;
            }
        }


        void AddLineToArray( point_2i const &v1, point_2i const &v2,
                             point_2  const &p1, point_2  const &p2 )
        {
            int x1 = v1.x;
            int y1 = v1.y;
            int x2 = v2.x;
            int y2 = v2.y;

            int x = x1;
            int y = y1;

            AddToLeftRightArrays (point_2i(x,y)); // add the first point

            if (x1==x2 && y1==y2)
                return;

            point_2 pcb1 = v1;
            point_2 pcb2 = v2;

            int dx = x2-x1;
            int dy = y2-y1;
            int deltax = sign(dx);
            int deltay = sign(dy);
            int MDx = abs(dx);
            int MDy = abs(dy);

            // double MDx,MDy
            double fD = max(cg::abs(p2.x - p1.x), cg::abs(p2.y - p1.y));
            double fMDx = cg::abs(p2.x - p1.x) / fD;
            double fMDy = cg::abs(p2.y - p1.y) / fD;

            // proper SIGMA values will be calculated below
            double sigmax = 0;
            double sigmay = 0;

            // get first intersection with grid
            double Dx = p2.x - p1.x;
            double Dy = p2.y - p1.y;

            double x0,y0;
            if (Dx>0) x0 = pcb1.x + 1.0;
                 else x0 = pcb1.x;
            if (Dy>0) y0 = pcb1.y + 1.0;
                 else y0 = pcb1.y;

            const double eps = epsilon<double>( );

            if (cg::abs(Dx) < eps ) Dx = Dx>0.0 ? eps : -eps ;
            if (cg::abs(Dy) < eps ) Dy = Dy>0.0 ? eps : -eps ;

            double k = Dy/Dx;
            double b = p1.y - k*p1.x;

            double xx = x0; // xx,yy - intersection point
            double yy = k*xx + b;
            if ((Dy>=2.*eps && yy>y0) || (Dy<=-2.*eps && yy<y0) || cg::abs(Dx)<2.*eps)
            {
                xx = (y0-b)/k;
                yy = y0;
                // move start point to intersection point
                if (Dx>=2.*eps) sigmax = xx - pcb1.x;
                               else sigmax = 1.0 - (xx-pcb1.x);
                y += deltay;
            } else
            {
                // move start point to intersection point
                if (Dy>=2.*eps) sigmay = yy - pcb1.y;
                           else sigmay = 1.0 - (yy-pcb1.y);
                x += deltax;
            }

            double k1 = cg::abs(k);
            double k2 = 1.0f/k1;

            int i = 0;
            while ((x!=x2 || y!=y2) && ++i<MDx+MDy)
            {
                AddToLeftRightArrays (point_2i (x,y));

                double a = 1.0 - sigmax;
                double b = 1.0 - sigmay;

                if (a+eps>=fMDx && b+eps>=fMDy)
                {
                    sigmax += fMDx;
                    sigmay += fMDy;
                } else
                if (a+eps>=fMDx)
                {
                    double sigmaxstep = b*k2;
                    sigmax += sigmaxstep;
                    sigmay += b;
                } else
                if (b+eps>=fMDy)
                {
                    double sigmaystep = a*k1;
                    sigmax += a;
                    sigmay += sigmaystep;
                } else
                {
                    Assert(0);
                    break;
                }

                if (sigmay+eps>=1.0)
                {
                    sigmay -= 1.0;
                    y += deltay;
                }
                if (sigmax+eps>=1.0)
                {
                    sigmax -= 1.0;
                    x += deltax;
                }
            }

            AddToLeftRightArrays( point_2i(x2,y2) ); // add the last point
        }

    private:
        std::vector<point_2i>   m_aLeftSide;
        std::vector<point_2i>   m_aRightSide;

        int m_iFromY;
        int m_iTriangleHeight;

        int x;
        int y;
    };

}
