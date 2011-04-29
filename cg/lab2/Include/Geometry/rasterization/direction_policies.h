#pragma once

#include "Geometry\rasterization\line_equation.h"

/***********************************************************************
    ������������ �������.                                                                     
    ������������ �������� � ��������� ������������ �������� 
    � ����������� �� ����������� �������.
 ***********************************************************************/

namespace cg
{
    namespace segment_rasterization_2
    {
        struct direction 
        {
            // ����������� �� �����������: �� x ��� �� y

            // ���������� ���������� x
            struct x  
            {
                // ����� ����������, ���������� ���������
                int       & select(point_2i & pt)             { return pt.x; }
                int const & select(point_2i const & pt) const { return pt.x; }

                // �����������  �������� ����������� ������ L � ������ x = t
                double      L_by(line_eq::line_equation_2 const &L, int t) const { return L.y_by_x(t); }

                // ��������� y ��� ��������� x �� �������
                double      L_step(line_eq::line_equation_2 const &L)      const { return L.y_step();  }
            };

            // ���������� ���������� y
            struct y  
            {
                int       & select(point_2i & pt)             { return pt.y; }
                int const & select(point_2i const & pt) const { return pt.y; }

                // �����������  ������� ����������� ������ L � ������ y = t
                double      L_by(line_eq::line_equation_2 const &L, int t) const { return L.x_by_y(t); }

                // ��������� x ��� ��������� y �� �������
                double      L_step(line_eq::line_equation_2 const &L)      const { return L.x_step();  }
            };

            // ����������� �� �������� ��� ������������ ����������

            // ���������� �����������
            struct inc 
            {
                // "�������", ������� ����� ����������
                int    side(int    t) const { return t + 1; }

                // �������� �� ���� ����
                double sign(double t) const { return t; }

                // ���������� ����������
                void advance(int & t)       { ++t; }
            };

            // ���������� �������
            struct dec 
            {
                int     side    (int    t) const { return  t; }
                double  sign    (double t) const { return -t; }
                void    advance (int &  t)       {       --t; }
            };

            // �������� ����� ��� �������
            enum Dir { inc_x, inc_y, dec_x, dec_y };

            struct x_inc : x, inc { enum { value = inc_x }; };
            struct x_dec : x, dec { enum { value = dec_x }; };

            struct y_inc : y, inc { enum { value = inc_y }; };
            struct y_dec : y, dec { enum { value = dec_y }; };

            struct none {};
        };

        // ����������� ���������� ������ ������ L �� ������ A
        template <class Direction>
            double Ipt_Start(Direction dir, 
                line_eq::line_equation_2 const &L, point_2i const & A)
        {
            return dir.L_by(L, dir.side(dir.select(A)));
        }

        // ����������� ���� ����������� 
        template <class Direction>
            double Ipt_Step(Direction dir, line_eq::line_equation_2 const &L)
        {
            return dir.sign(dir.L_step(L));
        }

    }
}