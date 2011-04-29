#pragma   once

#include "streams\structured_types.h"
#pragma warning ( disable : 4200 ) //nonstandard extension used : zero-sized array in struct/union

namespace cg
{
   template <class T>
      struct mapped_array_1d : ofs_ptr_s
   {
      typedef T           value_type;
      typedef T const &   const_reference;
      typedef T const *   const_pointer;
      typedef T const *   const_iterator;
      typedef size_t      size_type;

      size_t  size () const { return reinterpret_cast<size_t>(ptr()); }
      bool    empty() const { return size() == 0; }

      const_reference operator [] (int idx) const { return begin()[idx]; }

      const_reference at (int idx) const { check_valid(idx); return begin()[idx]; } 

      const_reference front () const { return at(0);        }
      const_reference back  () const { return at(size()-1); }

      bool contains (int idx) const { return 0 <= idx && idx < size(); } 

      void check_valid(int idx) const
      {
         Assert (0 <= idx); 
         Assert (idx < (int)size());
      }

      const_iterator begin () const { return reinterpret_cast<T const *>(ptr() + sizeof(size())); }
      const_iterator end   () const { return begin() + size(); }
   };

   template<class Stream, class T>
      void read ( streams::structured_istream<Stream>& stream, mapped_array_1d<T>& s )
   {
      read ( stream, static_cast<ofs_ptr_s&>(s) ) ; 
   }
}
