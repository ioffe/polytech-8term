#pragma once

#include "common/safe_bool.h"
#include "geometry/primitives/segment.h"
#include "line_equation.h"

namespace cg
{
    struct segment_rasterization_iterator
    {
        segment_rasterization_iterator( segment_2 const &segment )
            : index1    ( floor(segment.P0()) )
            , index2    ( floor(segment.P1()) )
            , finish    ( false )
        {
            // calc direction vector
    //        d = direction( segment );
            d.x = segment.P1().x - segment.P0().x;
            d.y = segment.P1().y - segment.P0().y;

            const double eps = epsilon<double>( );

            if (cg::abs(d.x) < eps) d.x = eps;
            if (cg::abs(d.y) < eps) d.y = eps;

            d /= norm(d);

            cur = segment.P0();

    //        inv = ~d;
            inv.x = 1.0 / d.x;
            inv.y = 1.0 / d.y;

            c.x = d.x >= 0 ? 1 : 0;
            c.y = d.y >= 0 ? 1 : 0;

            e.x = d.x >= 0 ? 1 : -1;
            e.y = d.y >= 0 ? 1 : -1;
        }

        ~segment_rasterization_iterator()
        {
        }


        point_2i const & operator * () const {
            return index1;
        }


        SAFE_BOOL_OPERATOR(!finish)


        point_2 const &getcurpos() const {
            return cur;
        }

        segment_rasterization_iterator& operator ++ ()
        {
            // go to next cell
            if ( index1 == index2 )
            {
                finish = true;
                return *this;
            } else
            {
                // calculate coordinates inside cell
    //            point_2 p = cur - index1;

    //            point_2 dist = (c - p) & inv;
                double distx = (c.x - (cur.x-index1.x)) * inv.x;
                double disty = (c.y - (cur.y-index1.y)) * inv.y;

                double mindist;
                if (distx < disty) index1.x += e.x, mindist = distx;
                              else index1.y += e.y, mindist = disty;

                // find intersection point
    //            cur += d * mindist;
                cur.x += d.x * mindist;
                cur.y += d.y * mindist;
            }

            return *this;
        }

    private:
        point_2i index1;
        point_2i index2;

        point_2 cur;
        point_2 c;
        point_2i e;

        point_2 d;
        point_2 inv;

        bool finish;
    };

    // bool (*Callback)(point_2i);  // returns true , если надо остановиться
    template <class Callback>
        inline bool rasterize_segment(segment_2 const &segment, Callback const &out)
    {
        point_2i index1 = floor(segment.P0());
        point_2i index2 = floor(segment.P1());

        if (index1 == index2)
            return out(index1, 0.0, 1.0);

        point_2 d (segment.P1().x - segment.P0().x, segment.P1().y - segment.P0().y);

        if (cg::abs(d.x) < epsilon) d.x = epsilon;
        if (cg::abs(d.y) < epsilon) d.y = epsilon;

        double nrm = 1 / norma(d);
        d *= nrm;

        point_2 cur    = segment.P0();
        double  ratio  = 0.0;
        double  dratio = nrm;

        point_2 inv (1.0 / d.x, 1.0 / d.y);

        point_2  c(d.x >= 0 ? 1 :  0,  d.y >= 0 ? 1 :  0);
        point_2i e(d.x >= 0 ? 1 : -1,  d.y >= 0 ? 1 : -1);

        double was_ratio = ratio;

        cg::rectangle_2i rect( index1, index2 );

        do 
        {
            if ( !rect.contains( index1 ) )
               return false;

            double distx = (c.x - (cur.x-index1.x)) * inv.x;
            double disty = (c.y - (cur.y-index1.y)) * inv.y;

            bool flag = distx < disty;
            
            // TODO :: 2Kovalev

            if (eq(distx,disty, 1e-6))
              flag = index1.x != index2.x;

            if (flag)
            {
                ratio += dratio * distx;

                if (out(index1, was_ratio, ratio))
                    return true;

                was_ratio = ratio;

                cur.x += d.x    * distx;
                cur.y += d.y    * distx;
                index1.x += e.x;
            } else
            {
                ratio += dratio * disty;

                if (out(index1, was_ratio, ratio))
                    return true;

                was_ratio = ratio;

                cur.x += d.x    * disty;
                cur.y += d.y    * disty;
                index1.y += e.y;
            }

    //        if (out(index1, was_ratio, ratio))
    //            return true;

        } while (index1 != index2);

        if (out(index1, ratio, 1.0))
            return true;

        return false;
    }

    template <class Callback>
        inline bool rasterize_segment(segment_2 const &segment, Callback &out)
    {
        Verify( is_finite( segment ) );

        const double eps = cg::epsilon< double >( );

        point_2i index1 = floor(segment.P0() + cg::point_2( eps, eps ));
        point_2i const index2 = floor(segment.P1() + cg::point_2( eps, eps ));

        if (index1 == index2)
            return out(index1, 0.0, 1.0);

        point_2 d (segment.P1().x - segment.P0().x, segment.P1().y - segment.P0().y);
        
        if (cg::abs(d.x) < eps) 
           d.x = eps;
        if (cg::abs(d.y) < eps) 
           d.y = eps;

        double nrm = 1 / norm(d);
        d *= nrm;

        point_2 cur    = segment.P0();
        double  ratio  = 0.0;
        double  dratio = nrm;

        point_2  const c(d.x >= 0 ? 1 :  0,  d.y >= 0 ? 1 :  0);
        point_2i const e(d.x >= 0 ? 1 : -1,  d.y >= 0 ? 1 : -1);

        double was_ratio = ratio;

        // обработка горизонтальных и вертикальных отрезков

       if (index1.x == index2.x) // отрезок вертикальный
       {
           range_2i bound (index1.y, index2.y);

           for (; index1.y != index2.y && bound.contains(index1.y) ; index1.y += e.y)
           {
               double const disty = (c.y - (cur.y-index1.y)) / d.y;
               ratio += dratio * disty;

               if (out(index1, was_ratio, ratio))
                   return true;

               was_ratio = ratio;
               cur.y += d.y * disty;
           }
       } else
       if (index1.y == index2.y) // отрезок горизонтальный
       {
           range_2i bound (index1.x, index2.x);

           for (; index1.x != index2.x && bound.contains(index1.x); index1.x += e.x)
           {
               double const distx = (c.x - (cur.x-index1.x)) / d.x;
               ratio += dratio * distx;

               if (out(index1, was_ratio, ratio))
                   return true;

               was_ratio = ratio;
               cur.x += d.x * distx;
           }
       } else
        {
            // общий случай - наклонный отрезок
            point_2 const inv (1.0 / d.x, 1.0 / d.y);

            rectangle_2i rect( index1, index2 );

            do 
            {
                if ( !rect.contains( index1 ) )
                   return false;

                double const distx = (c.x - (cur.x-index1.x)) * inv.x;
                double const disty = (c.y - (cur.y-index1.y)) * inv.y;

                bool flag = distx < disty;
                
                // TODO :: 2Kovalev

                if (eq(distx,disty, 1e-6))
                    flag = index1.x != index2.x;

                if (flag)
                {
                    ratio += dratio * distx;

                    if (out(index1, was_ratio, ratio))
                        return true;

                    was_ratio = ratio;

                    cur.x += d.x * distx;
                    cur.y += d.y * distx;
                    index1.x += e.x;
                } else
                {
                    ratio += dratio * disty;

                    if (out(index1, was_ratio, ratio))
                        return true;

                    was_ratio = ratio;

                    cur.x += d.x * disty;
                    cur.y += d.y * disty;
                    index1.y += e.y;
                }

        //        if (out(index1, was_ratio, ratio))
        //            return true;

            } while (index1 != index2);
        }

        if (out(index1, ratio, 1.0))
            return true;

        return false;
    }


    template <class Processor>
        inline bool rasterize_segment(point_2 const &from, point_2 const &to, Processor & processor)
    {
        point_2i  A = floor(from), B = floor(to);

        // если отрезок вертикальный
        if (A.x == B.x)
        {
            // и направлен вниз
            if (A.y < B.y)
            {
                for (; A.y < B.y; ++A.y)
                    if (processor(A))
                        return true;
            }  else
            // или направлен вверх
            if (A.y > B.y)
            {
                for (; A.y > B.y; --A.y)
                    if (processor(A))
                        return true;
            }

            // обрабытываем последнюю ячейку (случай, когда ячейка одна, сюда входит)
            return processor(A);
        }

        // если отрезок горизонтальный
        if (A.y == B.y)
        {
            // и направлен вправо
            if (A.x < B.x)
            {
                for (; A.x < B.x; ++A.x)
                    if (processor(A))
                        return true;
            } else // или влево
            {
                for (; A.x > B.x; --A.x)
                    if (processor(A))
                        return true;
            }

            return processor(A);
        }

        line_eq::line_equation_2  L(from, to);

        if (A.x < B.x) 
        {
            if (A.y < B.y) 
            {
                ////  EES
                if (B.x - A.x > B.y - A.y)
                {
                    double x_hi = L.x_by_y(A.y + 1);
                    double x_step = L.x_step();

                    for (; A.y < B.y; ++A.y)
                    {
                        for (; A.x < floor(x_hi); ++A.x)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        x_hi += x_step;
                    }

                    for (; A.x < B.x; ++A.x)
                        if (processor(A))
                            return true;

                    return processor(A);
                } else
                ///  ESS
                {
                    double y_hi = L.y_by_x(A.x + 1);
                    double y_step = L.y_step();

                    for (; A.x < B.x; ++A.x)
                    {
                        for (; A.y < floor(y_hi); ++A.y)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        y_hi += y_step;
                    }

                    for (; A.y < B.y; ++A.y)
                        if (processor(A))
                            return true;

                    return processor(A);
                }
            }
            else
            {
                ////  EEN
                if (B.x - A.x > A.y - B.y)
                {
                    double x_hi = L.x_by_y(A.y);
                    double x_step = L.x_step();

                    for (; A.y > B.y; --A.y)
                    {
                        for (; A.x < floor(x_hi); ++A.x)
                            if (processor(A))
                                return true;
                            
                        if (processor(A))
                            return true;

                        x_hi -= x_step;
                    }

                    for (; A.x < B.x; ++A.x)
                        if (processor(A))
                            return true;

                    return processor(A);
                } else
                ///  ENN
                {
                    double y_lo = L.y_by_x(A.x + 1);
                    double y_step = L.y_step();

                    for (; A.x < B.x; ++A.x)
                    {
                        for (; A.y > floor(y_lo); --A.y)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        y_lo += y_step;
                    }

                    for (; A.y > B.y; --A.y)
                        if (processor(A))
                            return true;

                    return processor(A);
                }
            }
        } else
        {
            if (A.y < B.y) 
            {
                ////  WWS
                if (A.x - B.x > B.y - A.y)
                {
                    double x_lo = L.x_by_y(A.y + 1);
                    double x_step = L.x_step();

                    for (; A.y < B.y; ++A.y)
                    {
                        for (; A.x > floor(x_lo); --A.x)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        x_lo += x_step;
                    }

                    for (; A.x > B.x; --A.x)
                        if (processor(A))
                            return true;

                    return processor(A);
                } else
                ///  WSS
                {
                    double y_hi = L.y_by_x(A.x);
                    double y_step = L.y_step();

                    for (; A.x > B.x; --A.x)
                    {
                        for (; A.y < floor(y_hi); ++A.y)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        y_hi -= y_step;
                    }

                    for (; A.y < B.y; ++A.y)
                        if (processor(A))
                            return true;

                    return processor(A);
                }
            }
            else
            {
                ////  WWN
                if (A.x - B.x > A.y - B.y)
                {
                    double x_lo = L.x_by_y(A.y);
                    double x_step = L.x_step();

                    for (; A.y > B.y; --A.y)
                    {
                        for (; A.x > floor(x_lo); --A.x)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        x_lo -= x_step;
                    }

                    for (; A.x > B.x; --A.x)
                        if (processor(A))
                            return true;

                    return processor(A);
                } else
                ///  WNN
                {
                    double y_lo = L.y_by_x(A.x);
                    double y_step = L.y_step();

                    for (; A.x > B.x; --A.x)
                    {
                        for (; A.y > floor(y_lo); --A.y)
                            if (processor(A))
                                return true;

                        if (processor(A))
                            return true;

                        y_lo -= y_step;
                    }

                    for (; A.y > B.y; --A.y)
                        if (processor(A))
                            return true;

                    return processor(A);
                }
            }
        }

        return false;
    }

    namespace segment_rasterization
    {
        struct direction
        {
            struct x  
            {
                int       & select(point_2i & pt)             { return pt.x; }
                int const & select(point_2i const & pt) const { return pt.x; }

                double      L_by(line_eq::line_equation_2 const &L, int t) const { return L.y_by_x(t); }
                double      L_step(line_eq::line_equation_2 const &L)      const { return L.y_step();  }
            };

            struct y  
            {
                int       & select(point_2i & pt)             { return pt.y; }
                int const & select(point_2i const & pt) const { return pt.y; }

                double      L_by(line_eq::line_equation_2 const &L, int t) const { return L.x_by_y(t); }
                double      L_step(line_eq::line_equation_2 const &L)      const { return L.x_step();  }
            };
            

            struct inc 
            {
                int side (int t) const { return t + 1; }
                double sign(double t) const { return t; }
                void advance(int & t) { ++t; }
            };

            struct dec 
            {
                int side (int t) const { return t;     }
                double sign(double t) const { return -t; }
                void advance(int & t) { --t; }
            };

            template <class Master, class Slave>
                struct base
            {
                typedef Master  master;
                typedef Slave   slave;
            };

            enum Dir { inc_x, inc_y, dec_x, dec_y };

            struct x_inc : x, inc { enum { value = inc_x }; };
            struct x_dec : x, dec { enum { value = dec_x }; };

            struct y_inc : y, inc { enum { value = inc_y }; };
            struct y_dec : y, dec { enum { value = dec_y }; };

            template <class T> struct opposite;

            template <> struct opposite<x_inc> { typedef x_dec value; };
            template <> struct opposite<x_dec> { typedef x_inc value; };
            template <> struct opposite<y_inc> { typedef y_dec value; };
            template <> struct opposite<y_dec> { typedef y_inc value; };

            struct nnw : base<x_dec, y_dec> {};
            struct nww : base<y_dec, x_dec> {};

            struct nne : base<x_inc, y_dec> {};
            struct nee : base<y_dec, x_inc> {};

            struct ssw : base<x_dec, y_inc> {};
            struct sww : base<y_inc, x_dec> {};

            struct sse : base<x_inc, y_inc> {} ;
            struct see : base<y_inc, x_inc> {} ;

            struct s { typedef y_inc master; };
            struct n { typedef y_dec master; };

            struct e { typedef x_inc master; };
            struct w { typedef x_dec master; };
        };

        template <class MasterDirection>
            double Ipt_Start(MasterDirection dir, 
                line_eq::line_equation_2 const &L, point_2i const & A)
        {
            return dir.L_by(L, dir.side(dir.select(A)));
        }

        template <class MasterDirection>
            double Ipt_Step(MasterDirection dir,
                line_eq::line_equation_2 const &L)
        {
            return dir.sign(dir.L_step(L));
        }

        struct state
        {
            struct start {};
            struct end   {};
            struct master{};
            struct slave {};
        };

        template <class T> struct side : T {};

        struct inner {};

        template <class Direction, class Processor>
            struct traits 
        {
            typedef Direction  dir;

            typename dir::master  master;
            typename dir::slave   slave;

            traits(line_eq::line_equation_2 const &L, point_2i &A, point_2i const &B, Processor & processor)
                :   L (L), A(A), B(B), processor(processor)
            {
//                direction::Dir a(Direction::master::value);

                ipt       = Ipt_Start(master, L, A);
                ipt_int   = floor(ipt);
                ipt_delta = Ipt_Step(master, L);  

                ipt_slave = Ipt_Start(slave, L, A);
                ipt_slave_delta = Ipt_Step(slave, L);

                Assert(cg::abs(ipt_delta) >= 1);
            }

            bool   master_cond() const { 
                return master.select(A) != master.select(B); 
            }

            void   master_advance()    { 
                master.advance(master.select(A));  
                ipt_int = floor(ipt += ipt_delta);  
            }

            bool   slave_cond() const  { 
                return slave.select(A) != ipt_int; 
            }

            bool   slave_cond_B() const{ 
                return slave.select(A) != slave.select(B);     
            }

            void   slave_advance()     { 
                slave.advance(slave.select(A)); 
                ipt_slave += ipt_slave_delta;
            }

            side<typename dir::slave>    out_side_slave;

            side<typename dir::slave>    in_side_slave;

            side<typename dir::master>   out_side_master;

            side<typename dir::master>   in_side_master;

            // Интерфейс для BigCellProcessor'a при растеризации по двухуровневой сетке

            // текущая ячейка, т.е. A
            point_2i const & currentCell() const;

            // точка входа в ячейку (extents - только для избежания ошибок округления)
            template <class In>
                point_2i in_sub_cell(In, point_2i const & extents);

            point_2i in_sub_cell(inner, point_2i const & extents);

            // точка выхода из ячейки
            template <class Out>
                point_2i out_sub_cell(Out, point_2i const & extents);

            point_2i out_sub_cell(inner, point_2i const & extents);



            bool process(state::start, state::slave)
            {
                if (processor(A, inner(), out_side_slave))
                    return true;

                slave_advance();

                return false;
            }

            bool  process(state::start, state::master)
            {
                if (processor(A, inner(), out_side_slave))
                    return true;

                master_advance();

                return false;
            }

            bool process(state::slave, state::slave)
            {
                if (processor(A, in_side_slave, out_side_slave))
                    return true;

                slave_advance();

                return false;
            }

            bool process(state::slave, state::master)
            {
                if (processor(A, in_side_slave, out_side_master))
                    return true;

                master_advance();

                return false;
            }

            bool process(state::master,state::slave)
            {
                if (processor(A, in_side_master, out_side_slave))
                    return true;

                slave_advance();

                return false;
            }

            bool process(state::master,state::end)
            {
                return processor(A, in_side_master, inner());
            }

            bool process(state::slave, state::end)
            {
                return processor(A, in_side_slave, inner());
            }

        protected:
            line_eq::line_equation_2 const & L;
            point_2i              & A;
            point_2i const        & B; 

            Processor             & processor;
            
            double  ipt;  int ipt_int;
            double  ipt_delta; 

            double  ipt_slave;
            double  ipt_slave_delta;
        };


        template <class Traits>
            bool process(Traits t)
        {
//            direction::Dir a(Direction::master::value);
            

            state::start  start;
            state::master master;
            state::slave  slave;
            state::end    end;

            if (t.slave_cond())
            {
                // start - x_side
                if (t.process(start, slave))
                    return true;

                // x_side - x_side
                while (t.slave_cond())
                    if (t.process(slave, slave))
                        return true;

                // x_side - y_side
                if (t.process(slave, master))
                    return true;
            }
            else
            {
                if (t.process(start, master))
                    return true;
            }

            while (t.master_cond())
            {
                // y_side - x_side
                if (t.process(master,slave))
                    return true;

                // x_side - x_side
                while (t.slave_cond())
                    if (t.process(slave, slave))
                        return true;

                // x_side - y_side
                if (t.process(slave, master))
                    return true;
            }

            // y_side - end
            if (! t.slave_cond_B())
                return t.process(master,end);

            // y_side - x_side
            if (t.process(master,slave))
                return true;

            while (t.slave_cond_B())
                if (t.process(slave, slave))
                    return true;

            return t.process(slave, end);
        }

        template <class Direction, class Processor>
            struct traits_along_axis
        {
            traits_along_axis ( point_2i & A, point_2i const &B, Processor & processor )
                :   A(A), B(B), processor(processor)
            {}

            typename Direction::master  master;

            bool master_cond() const { 
                return master.select(A) != master.select(B); 
            }

            void master_advance() {
                master.advance(master.select(A));
            }

            side<typename Direction::master>   out, in;

            bool process(state::start, state::master)
            {
                if (processor(A, inner(), out))
                    return true;

                master_advance();

                return false;
            }

            bool process(state::master, state::master)
            {
                if (processor(A, in, out))
                    return true;

                master_advance();

                return false;
            }

            bool process(state::master, state::end)
            {
                return processor(A, in, inner());
            }

            
            point_2i        & A;
            point_2i const  & B;
            Processor       & processor;
        };

        template <class Direction, class Processor>
            struct traits_along_axis_true : traits_along_axis<Direction, Processor>
        {
            traits_along_axis_true(
                line_eq::line_equation_2 const &L, 
                point_2i &A, point_2i const & B,
                Processor & processor)

                :   traits_along_axis<Direction,Processor>(A, B, processor)
                ,   L (L)
            {
                ipt_slave = Ipt_Start(Direction::slave(), L, A);
                ipt_slave_delta = Ipt_Step(Direction::slave(), L);
            }

            line_eq::line_equation_2 const & L;
            double ipt_slave;
            double ipt_slave_delta;
        };

        template <class Traits>
            bool process_along_axis(Traits t)
        {
            state::start  start;
            state::master master;
            state::end    end;

            if (t.process(start, master))
                return true;

            while (t.master_cond())
                if (t.process(master, master))
                    return true;

            return t.process(master, end);
        }




        template <class Processor>
            bool process(point_2 const &from, point_2 const &to, Processor & processor)
        {
            point_2i  A = floor(from), B = floor(to);

            // если отрезок вертикальный
            if (A.x == B.x)
            {
                return 
                    A.y == B.y 
                        ?   processor(A, inner(), inner())
                        :   A.y < B.y
                            ?   process_along_axis(traits_along_axis<direction::s,Processor>(A,B, processor))
                            :   process_along_axis(traits_along_axis<direction::n,Processor>(A,B, processor));
            }

            // если отрезок горизонтальный
            if (A.y == B.y)
            {
                return
                    A.x < B.x
                        ?   process_along_axis(traits_along_axis<direction::e,Processor>(A,B, processor))
                        :   process_along_axis(traits_along_axis<direction::w,Processor>(A,B, processor));
            }

            line_eq::line_equation_2  L(from, to);

            return
                A.x < B.x 
                ?   A.y < B.y
                    ?   to.x - from.x > to.y - from.y
                        ?   process(traits<direction::see, Processor>(L, A, B, processor))
                        :   process(traits<direction::sse, Processor>(L, A, B, processor))
                    :   to.x - from.x > from.y - to.y
                        ?   process(traits<direction::nee, Processor>(L, A, B, processor))
                        :   process(traits<direction::nne, Processor>(L, A, B, processor))
                :   A.y < B.y
                    ?   from.x - to.x > to.y - from.y
                        ?   process(traits<direction::sww, Processor>(L, A, B, processor))
                        :   process(traits<direction::ssw, Processor>(L, A, B, processor))
                    :   from.x - to.x > from.y - to.y
                        ?   process(traits<direction::nww, Processor>(L, A, B, processor))
                        :   process(traits<direction::nnw, Processor>(L, A, B, processor));
        }
    };
}