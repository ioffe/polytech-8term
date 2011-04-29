#pragma once
#include "common/safe_bool.h"


namespace cg
{
    // Ptr - тип, при разыменовании которого получается нечто подобное array_2d
    //       хочется, чтобы обертка реализовывала такую же концепцию

    // const - версия
    template <class Ptr>
        struct array_2d_wrapper_const
    {
        __forceinline point_2i    extents () const { return ptr_ ? ptr_->extents() : point_2i(0,0); }
        __forceinline size_t      size    () const { return ptr_ ? ptr_->size() : 0;    }

        __forceinline bool contains(point_2i const & idx) const 
        { 
            return (bool)ptr_ && 
                idx.x >= 0 && idx.y >= 0 && 
                idx.x < ptr_->extents().x && idx.y < ptr_->extents().y; 
        }

        typedef          Ptr                        ptr_type;
        typedef typename Ptr::value_type            array_type;
        typedef typename array_type::value_type     value_type;

        typedef typename array_type::const_iterator const_iterator;

        // имитируем указатель на массив
        SAFE_BOOL_OPERATOR(ptr())

        __forceinline array_type const & operator * () const { return  *ptr(); }

        __forceinline point_2i index(const_iterator it) const { return ptr_->index(it); }
        
        __forceinline const_iterator begin() const { return ptr_->begin(); }
        __forceinline const_iterator end()   const { return ptr_->end();   }

        __forceinline value_type const & at(point_2i const & idx_small)         const { return ptr_->at(idx_small); }
        __forceinline value_type const & operator[](point_2i const & idx_small) const { return ptr_->at(idx_small); }

        __forceinline Ptr const & ptr() const { return ptr_; }

        template< typename P >
           __forceinline void set_ptr( P const& p ) { ptr_ = p; }

        __forceinline void reset_ptr( ) { ptr_ = NULL; }

    protected:
        Ptr     ptr_;
    };

    // не const - версия
    template <class Ptr>
      struct array_2d_wrapper : array_2d_wrapper_const<Ptr>
    {
        typedef          Ptr                        ptr_type;
        typedef typename Ptr::value_type            array_type;
        typedef typename array_type::value_type     value_type;

        typedef typename array_type::iterator       iterator;

        using array_2d_wrapper_const<Ptr>::operator *;
        __forceinline array_type       & operator * ()       { return  *ptr(); }

        using array_2d_wrapper_const<Ptr>::index;
        __forceinline point_2i index(iterator it) const { return ptr_->index(it); }
        
        using array_2d_wrapper_const<Ptr>::begin;
        using array_2d_wrapper_const<Ptr>::end;
        __forceinline iterator begin() { return ptr_->begin(); }
        __forceinline iterator end()   { return ptr_->end();   }

        using array_2d_wrapper_const<Ptr>::at;
        __forceinline value_type       & at(point_2i const & idx_small)       { return ptr_->at(idx_small); }

        using array_2d_wrapper_const<Ptr>::operator[];
        __forceinline value_type       & operator[](point_2i const & idx_small) const { return ptr_->at(idx_small); }

        template <class Stream>
         friend void read(Stream &out, array_2d_wrapper & wr)
        {
           read( out, wr.ptr_ );
        }
    };

   template <class Stream, class Ptr>
      inline void write(Stream &out, array_2d_wrapper_const<Ptr> const & wr)
   {
      write( out, wr.ptr() );
   }

   template <class Stream, class Ptr>
      inline void write(Stream &out, array_2d_wrapper<Ptr> const & wr)
   {
      write( out, wr.ptr() );
   }

}
