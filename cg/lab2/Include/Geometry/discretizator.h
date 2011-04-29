#pragma once

#include "primitives\rectangle.h"

namespace cg
{
    template <unsigned Div>
        struct Discretezator_1
    {
        Discretezator_1 () {}

        Discretezator_1 (range_2 const & range) 
            :   org_ (range.lo()), unit_ (range.size() / Div)
        {}

        Discretezator_1 (double o, double u) : org_(o), unit_(u) {}

        double origin()  const { return org_;  }
        double unit  () const  { return unit_; }

        unsigned pack (double x) const
        {
            Assert(x >= org_);
            Assert(x < org_ + unit_ * Div + unit_);

            return static_cast<unsigned>((x - org_) / unit_);
        }

        double unpack (unsigned x) const
        {
            Assert(x < Div);
            return org_ + x * unit_;
        }

    private:
        double  org_;
        double  unit_;
    };

    template <unsigned DivX, unsigned DivY = DivX>
        struct Discretezator_2
    {
        Discretezator_2 (rectangle_2 const & rc) : x(rc.x), y(rc.y) {}

        point_2 origin() const { return point_2(x.origin(), y.origin()); }
        point_2 unit  () const { return point_2(x.unit(), y.unit()); }

        point_2i pack (point_2 const & pt) const
        {
            return point_2i(x.pack(pt.x), y.pack(pt.y));
        }

        point_2 unpack (point_2i const & pt) const
        {
            return point_2(x.unpack(pt.x), y.unpack(pt.y));
        }

    private:
        Discretezator_1<DivX>   x;
        Discretezator_1<DivY>   y;
    };
}
