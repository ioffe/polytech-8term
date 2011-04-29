#pragma   once

#include "line_equation.h"

namespace cg
{
    namespace segment_rasterization_2
    {
        namespace states
        {
            struct basic : point_2i
            {
                basic(point_2 const & From, point_2 const & To)
                    :   from_(From)
                    ,   to_  (To)
                    ,   point_2i   (floor(From))
                    ,   B_         (floor(To))
                {}

                point_2i       & A()       { return *this; }
                point_2i const & A() const { return *this; }
                point_2i const & B() const { return B_; }

                point_2  const & from () const { return from_; }
                point_2  const & to   () const { return to_;   }

            private:

                point_2  const & from_;
                point_2  const & to_;       

                point_2i const B_;
            };

            struct basic_naa : basic
            {
                basic_naa (basic b)
                    :   basic (b)
                    ,   L_(b.from(), b.to())
                {}

                line_eq::line_equation_2 const & L() const { return L_; }

            protected:

                double & ipt()       { return ipt_; }
                int    & ipt_int()   { return ipt_int_; }
                double & ipt_delta() { return ipt_delta_; }

                int const & ipt_int() const  { return ipt_int_; }

            private:
               line_eq::line_equation_2 L_; 

                // координата следуюшего перехода по главному направлению
                double  ipt_;  
                // в какой клетке он достигается
                int     ipt_int_;
                // приращение координаты большого перехода
                double  ipt_delta_; 
            };

            template <class Base, class Processor>
                struct proc_mix : Base
            {
                proc_mix(Base b, Processor & proc) : Base(b), processor_(proc) {}

                Processor   & processor() { return processor_; }

            private:

                Processor   & processor_;
            };

        }
    }
}