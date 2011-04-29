#pragma once

#include "Geometry\mapped_array_2d.h"
#include "Geometry\grid_params.h"

namespace cg
{
#pragma pack ( push , 1 )
   // Объекты типа T занимают в мапленом потоке одинаковый объем памяти
   template <class T>
      struct MappedGrid1L
         :   aa_transform
         ,   mapped_array_2d<T>
   {
      typedef aa_transform        tform_type ;
      typedef mapped_array_2d<T>  storage_type ;

      tform_type     const &  tform        () const { return *this; }
      storage_type   const &  array        () const { return *this; }
      grid_params             grid_params  () const { return cg::grid_params( tform().origin(), tform().unit(), extents() ); }

      using storage_type::contains;

      bool contains(point_2 const & pt) const
      {   return storage_type::contains(floor(tform_type::world2local(pt))); }
   };
#pragma pack ( pop )
}
