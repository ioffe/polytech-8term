#pragma once

#include <boost/function.hpp>

namespace cg   {
namespace cdt  {

   typedef 
      boost::function< bool ( size_t idx ) >
      triangle_filter_t;

   namespace 
   {
      bool dummy_triangle_filter( size_t ) { return true; }
   }

}}