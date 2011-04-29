#pragma once

#include "geometry/primitives/point.h"
#include "Common/m_ptr.h"
#include "Common/safe_bool.h"
#include "Geometry/Array_2d_Wrapper.h"
#include "Geometry/Empty.h"

namespace cg 
{
   template <class T, class U = Empty >
      struct grid2l_bigcell : U, array_2d_wrapper< m_ptr< array_2d<T> > >
   {
      typedef T                    smallcell_type;
      typedef U                    header_type;

      typedef 
         array_2d_wrapper< m_ptr< array_2d<T> > >
         array_type;

      U const & header() const { return *this; }
      U       & header()       { return *this; }

      array_type const & array() const { return *this; }
      array_type       & array()       { return *this; }

      void subdivide(point_2i const &subdivision)
      {
         if (subdivision.x > 0 && subdivision.y > 0)
               set_ptr( new array_2d<T>(subdivision) );
      }

      void destroy() { reset_ptr( ); }
   };

   template <class Stream, class T, class U >
      inline void write(Stream &out, grid2l_bigcell<T, U> const & bcell)
   {
      write( out, bcell.header() );
      write( out, bcell.array()  );
   }

   template <class Stream, class T, class U >
      inline void read(Stream &out, grid2l_bigcell<T, U> & bcell)
   {
      read( out, bcell.header() );
      read( out, bcell.array()  );
   }

}
