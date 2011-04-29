#pragma   once

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct avg_normal_in_rect
        {
            typedef typename Traits::face_id face_id;

            void getAveNormalInRect  (rectangle_2 const &rect, point_3 &normal) const
            {
                normal = point_3(0,0,0);

                Processor processor (self().traits(), normal);
                visit (grid(), rect, processor);

                normal /= processor.GetSSum();
            }
        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }
            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            { 
                Processor (Traits const & traits, point_3 &normal)
                    : traits_ ( traits )
                    , normal_ ( normal )
                    , s_sum_  (    0.0 )
                    , tr_num_ (      0 )
                {}

                template <class State>
                    bool operator () (State &, typename Grid::smallcell_type const & column)
                {
                    if (column)
                    {
                        for (DWORD i=0; i<column->faces().size(); i++)
                        {
                            face_id tr_index = column->faces()[i];

                            traits_.check_face_id(tr_index);

                            cg::point_3 n = traits_.getTriangleNormal( tr_index );

                            // вычисляем площадь треугольника
                            double s = traits_.getTriangleSquare( tr_index );

                            n *= s;

                            normal_ += n;
                            s_sum_  += s;
                            tr_num_ ++;

                            // может, еще нужно умножать каждую нормаль на коефициент, 
                            // пропорциональный площади ПЕРЕСЕЧЕНИЯ треугольника и исходного прямоугольника
                        }
                    }
                    return false;
                }

                double GetSSum()
                {
                    return tr_num_>0 ? s_sum_ : 1.0;
                }

            private:
                Traits const  & traits_;
                point_3       & normal_;
                double          s_sum_;
                int             tr_num_;
            };
        };
    }
}