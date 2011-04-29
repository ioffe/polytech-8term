#pragma once

#include "geometry/array_2d.h"
#include "geometry/aa_transform.h"
#include "Geometry/grid_params.h"

namespace cg
{
    // Grid1L - сумма transform + array_2d
    template 
        <
            class T, 
            class TForm   = aa_transform, 
            class Storage = array_2d<T, extents::rect, extents::rowwise>
        >
        struct Grid1L : TForm, Storage
    {
        typedef T       cell_type;
        typedef TForm   tform_type;
        typedef Storage storage_type;
        
        typedef typename Storage::extents_type extents_type;

        Grid1L(tform_type const &tform, point_2i const &ext)
            :   tform_type    (tform)
            ,   storage_type  (ext)
        {}

        Grid1L(tform_type const &tform, point_2i const &ext, T const &defval)
            :   tform_type  (tform)
            ,   storage_type(ext, defval)
        {}

         explicit Grid1L(grid_params const & gp)
            :   tform_type  (gp.tform())
            ,   storage_type(gp.extents())
        {}

        Grid1L() {}

        template <class U> Grid1L(Grid1L<U> const &other)
            :   tform_type  (other)
            ,   storage_type(other.extents())
        {}

        template <class U> Grid1L(Grid1L<U> const &other, T const &defval)
            :   tform_type  (other)
            ,   storage_type(other.extents(), defval)
        {}

        grid_params        grid_params() const { return cg::grid_params( tform().origin(), tform().unit(), extents() ); }

        tform_type const & tform() const { return *this; }
        tform_type       & tform()       { return *this; }

        storage_type       & array()       { return *this; }
        storage_type const & array() const { return *this; }

        using storage_type::contains;

        bool contains(point_2 const & pt) const
        {   return storage_type::contains(floor(tform_type::world2local(pt))); }
    };
    
    template <class T>
        rectangle_2 bounding(Grid1L<T> const &grid)
    {
        return 
            rectangle_2(
                grid.origin(), 
                grid.origin() + (grid.extents() & grid.unit())
                );
    }

    template <class T>
        rectangle_2 bounding(Grid1L<T> const &grid, point_2i const & idx)
    {
        point_2 xy = grid.local2world(idx);
        return rectangle_2(xy, xy + grid.unit());
    }
      
   template< class T >
   point_2 center(Grid1L<T> const & grid, point_2i const & idx )
   {
      return grid.local2world( idx + point_2(0.5, 0.5 ) );
   }

   template< class T >
   point_2i index(Grid1L<T> const & grid, point_2 const & pt )
   {
      return floor(grid.world2local(pt));
   }

   template <class Stream, class T, class TForm, class Storage>
      inline void write (Stream & stream, cg::Grid1L<T, TForm, Storage> const & grid)
   {
      write(stream, grid.tform());
      write(stream, grid.array());
   }

   template <class Stream, class T, class TForm, class Storage>
      inline void read (Stream & stream, cg::Grid1L<T, TForm, Storage> & grid)
   {
      read(stream, grid.tform());
      read(stream, grid.array());
   }
}
