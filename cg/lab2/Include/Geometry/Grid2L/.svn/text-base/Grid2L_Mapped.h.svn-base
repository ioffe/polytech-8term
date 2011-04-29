#pragma once

#pragma warning(disable:4200)

#include "Geometry\mapped_array_2d.h"
#include "Geometry\Grid1L\Grid1L_Mapped.h"
#include "Geometry\Empty.h"

namespace cg
{
#pragma pack (push , 1)
   template < class T, class U = Empty >
      struct grid2l_bigcell_mapped :  U,  array_2d_wrapper_const< mapped_ptr < mapped_array_2d<T> > >   
   {
        typedef T                    smallcell_type;
        typedef U                    header_type;

      typedef 
         array_2d_wrapper_const< mapped_ptr < mapped_array_2d<T> > >
         array_type;

      __forceinline header_type const & header() const { return *this; }
      __forceinline array_type  const & array () const { return *this; }
   };

      
   template <class T, class BigCell = grid2l_bigcell_mapped<T>, class GridHeader = Empty>
      struct MappedGrid2L : GridHeader, MappedGrid1L<BigCell>
   {
      typedef typename BigCell::smallcell_type const smallcell_type;

      typedef Index2L         index_type;
      typedef point_2i        bigidx_type;
      typedef point_2i        smallidx_type;
      typedef BigCell const   bigcell_type;

      typedef MappedGrid1L<BigCell> Base;

      typedef GridHeader      header_type;
      
      __forceinline raster_2 bigcellraster(point_2i const & idx_big) const
      {
         bigcell_type const & bcell = at(idx_big);

         point_2     origin = local2world(idx_big);
         point_2i    ext    = bcell.extents();
         point_2     unitt  = unit() / ext;

         return raster_2(origin, unitt, ext);
      }

      using Base::at;
      using Base::operator[];

      __forceinline T const & at (index_type const &idx) const
      {
         Assert(at(idx.big));
         return at(idx.big).at(idx.small);
      }
      
      __forceinline T const & operator [] (index_type const &idx) const
      {
         Assert(at(idx.big));
         return at(idx.big).at(idx.small); 
      }

      __forceinline Base        const & grid  () const { return *this; }
      __forceinline GridHeader  const & header() const { return *this; }
   };

   template <class T, class BigCell>
      rectangle_2 bounding(MappedGrid2L<T, BigCell> const & mapped_grid)
   {
      return
         rectangle_2(
            mapped_grid.local2world(point_2i(0,0)), 
            mapped_grid.local2world(mapped_grid.extents()));
   }

   template <class T, class BigCell>
      rectangle_2 bounding(MappedGrid2L<T, BigCell> const &grid, point_2i const & idx)
   {
      point_2 xy = grid.local2world(idx);
      return rectangle_2(xy, xy + grid.unit());
   }

#pragma pack ( pop )
}
