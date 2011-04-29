#pragma once

#include "Streams/structured_types.h"

namespace cg
{

template<class T>
   struct default_spline_traits
{
   typedef typename std::vector<T>       vector_type;
   typedef typename boost::shared_ptr<T> ptr_type;
};

template<class T>
   struct mapped_spline_traits
{
   typedef typename mapped_vector<T> vector_type;
   typedef typename mapped_ptr<T>    ptr_type;
};
   
} // end of namespace cg