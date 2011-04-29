#pragma   once

#include "Geometry\Grid2L.h"

#include "Streams\structured_types.h"

#include "algos.h"

#pragma pack ( push, 1 )
namespace cg
{
    template <class Traits>
        struct ColumnMapped
    {
        typedef typename Traits::face_id            face_id;
        typedef typename mapped_vector<face_id>     Faces;

    public:
        range_2 const & zrange() const { return zrange_; }
        Faces   const & faces()  const { return faces_;  }

    private:
        range_2     zrange_;
        Faces       faces_;
    };

    template <class Traits>
        struct cdt_grid_m
    {
       typedef MappedGrid2L < mapped_ptr< ColumnMapped < Traits> > >  value;
    };

    template <class Traits>
        struct Collision3DMapped
            :   cdt::algos < typename cdt_grid_m<Traits>::value, Traits, Collision3DMapped<Traits> >
    {
        typedef typename cdt_grid_m<Traits>::value  Grid;
        typedef Grid    grid_type;

        Collision3DMapped (Traits const & traits, mapped_ptr<Grid> grid)
            :  traits_ (traits)
            ,  grid_ (grid)
        {}

        Grid    const & grid  () const { return *grid_;  }
        Traits  const & traits() const { return  traits_;}

    private:
        Traits       const & traits_;
        mapped_ptr<Grid>     grid_;
    };
}
#pragma pack ( pop )
