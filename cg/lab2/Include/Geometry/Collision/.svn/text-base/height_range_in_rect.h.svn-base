#pragma    once

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct height_range_in_rect
        {
            range_2 getHeightRangeInRect( rectangle_2 const &rect ) const
            {
                range_2 res;
                Processor proc( res );
                visit( grid(), rect, proc );
                return res;
            }

        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }
            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor : grid2l_visitor_base<Grid, Processor>
            { 
                Processor (range_2 &range)
                    :   range_ (range)
                {}

                template <class State>
                    bool operator () (State &, typename Grid::smallcell_type const & column)
                {
                    if (column)
                    {
                        range_ |= column->zrange() ;

                        // по хорошему, надо клиповать треугольники исходным прямоугольником
                    }
                    return false;
                }

            private:
                range_2 &range_;
            };
       };
    }
}