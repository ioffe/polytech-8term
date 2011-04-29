#pragma once

#include "streams\structured_types.h"

namespace cg
{
   template <class T>
      struct mapped_array_2d : ofs_ptr_s
   {
      typedef T             value_type;

      typedef T const *   const_iterator;
      typedef T const *         iterator;

      __forceinline const_iterator begin() const { return reinterpret_cast<T const *>(ptr() + sizeof(extents())); }
      __forceinline const_iterator end  () const { return begin() + size(); }

      __forceinline point_2i const & extents() const 
      { 
         return *reinterpret_cast<point_2i const *>(ptr()); 
      }

      __forceinline size_t width () const { return extents().x; }
      __forceinline size_t height() const { return extents().y; }

      __forceinline size_t size() const { return width() * height(); }

      __forceinline bool contains(point_2i const &idx) const
      {
         return
               0 <= idx.x && size_t(idx.x) < width() &&
               0 <= idx.y && size_t(idx.y) < height();
      }

      __forceinline bool is_valid(point_2i const & idx) const
      {
         return contains(idx);
      }

      __forceinline void assert_valid(point_2i const & idx) const
      {
         Assert(0 <= idx.x);
         Assert(0 <= idx.y);
         Assert(size_t(idx.x) < width());
         Assert(size_t(idx.y) < height());
      }

      typedef int linear_index;

      __forceinline linear_index to1D(point_2i const & idx) const
      {   
         return linear_index(idx.y * width() + idx.x);
      }

      __forceinline T const & at (point_2i const & idx) const 
      {
         assert_valid(idx);
         return begin()[to1D(idx)];
      }

      __forceinline T const & operator[] (point_2i const & idx) const 
      {
         assert_valid(idx);
         return begin()[to1D(idx)];
      }

      __forceinline point_2i to2D(linear_index const & idx) const
      {
         return point_2i(idx % (int)width(), idx / (int)width());
      }

      // по итератору получить индекс €чейки
      __forceinline point_2i index(const_iterator p) const 
      { return to2D(int(p - begin())); }
   };

   template<class Stream, class T>
      void read ( streams::structured_istream<Stream>& stream, mapped_array_2d<T>& p )
   {
      read ( stream, static_cast<ofs_ptr_s&>(p) ) ; 
   }
}
