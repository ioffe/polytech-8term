#pragma once

#include <functional>

namespace util
{
   template< class T, template < class > class Pred >
      struct deref_pred
         : std::binary_function< T const *, T const *, bool >
   {
      bool operator()( T const * a, T const * b ) const 
      { 
         return Pred< T >()( *a, *b );
      }
   };

   template < class T > struct deref_equal_to   : deref_pred< T, std::equal_to > {};
   template < class T > struct deref_less       : deref_pred< T, std::less > {};
   template < class T > struct deref_greater    : deref_pred< T, std::greater > {};
}