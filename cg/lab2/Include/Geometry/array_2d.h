#pragma once

#include "array_2d_extents.h"
#include "primitives\rectangle.h"

namespace streams { template<class, class> struct structured_ostream;}

namespace cg {

    // Одна из реализаций двумерного массива
    // Данные лежат в последовательном куске памяти
    // допустимый диапазон индексов [0, extents)
    template <class T, class Shape = extents::rect, class Dir = extents::rowwise>
        struct array_2d : Extents<Shape, Dir>
    {
        typedef Extents<Shape, Dir> Ext;

        typedef T                   value_type;
        typedef int                 linear_index;
        typedef point_2i            index_type;
        typedef point_2i const &    index_cref;
        typedef T &                 reference;
        typedef T const &           const_reference;
        typedef T *                 pointer;
        typedef T const *           const_pointer;
        typedef T *                 iterator;
        typedef T const *           const_iterator;

        array_2d (index_cref ext, T const &defvalue) 
            :   Ext(ext), data_(new T[size()])
        {
            std::fill(data_, data_ + size(), defvalue);
        }

        array_2d(index_cref ext) 
            :   Ext(ext), data_(new T[size()])
        {}

        array_2d() : data_(0) {}

        array_2d(array_2d const & rhs) 
            :   Ext(rhs.extents()), data_(new T[size()])
        {
           copy( rhs );
        }

        array_2d const & operator = (array_2d const & rhs)
        {
          if ( this != &rhs )
          {
            resize( rhs.extents( ) );
            copy( rhs );
          }
          return *this;
        }

        ~array_2d() { delete [] data_; }
                        
        void resize(point_2i const & ext)
        {
            // обрабатывать случай одинаковых размеров
            Ext::resize(ext);

            delete [] data_;

            data_ = new T[size()];
        }

        void resize(point_2i const & ext, T const & defvalue)
        {
            resize(ext);

            std::fill(data_, data_ + size(), defvalue);
        }

        // доступ к элементам
        reference at (index_cref idx) {
            assert_valid(idx);
            return data_[to1D(idx)]; 
        }

        const_reference at (index_cref idx) const {
            assert_valid(idx);
            return data_[to1D(idx)]; 
        }

        reference       operator [] (index_cref idx)       { return at(idx); }
        const_reference operator [] (index_cref idx) const { return at(idx); }

        // начальный и конечный сквозной итератор
        iterator begin() { return data_; }
        iterator end  () { return data_ + size(); }

        const_iterator begin() const { return data_; }
        const_iterator end  () const { return data_ + size(); }

        // по итератору получить индекс ячейки
        index_type index(const_iterator p) const 
        { return to2D(int(p - data_)); }

        // указывает ли итератор на элемент массива
        bool contains(const_iterator p) const 
        { return data_ <= p && p <= data_ + size(); }

        bool contains(index_cref idx) const
        {   return Ext::contains(idx);  }

        size_t memory_allocated() const { 
            return 
                sizeof(extents_) + 
                sizeof(data_) + 
                sizeof(T) * size(); 
        }

    private:
        void copy( array_2d const & rhs )
        {
          iterator j = begin();
          for ( const_iterator i = rhs.begin(), e = rhs.end(); i != e; ++i, ++j )
            *j = *i;
        }

        T       * data_;
    };

   template< class Stream, class TempStream, class T, class Shape, class Dir >
      inline void write( streams::structured_ostream<Stream,TempStream> & stream, array_2d<T, Shape, Dir> const& array )
   {
      stream.push();

      write( stream, array.extents() );
      write_contiguous_range(stream, array.begin(), array.end(), typename is_moveable<T>::type());

      stream.pop();
   }

   template< class Stream, class T, class Shape, class Dir >
      inline void write( Stream & stream, array_2d<T, Shape, Dir> const& array )
      {
         write( stream, array.extents() );
         write_contiguous_range(stream, array.begin(), array.end(), typename is_moveable<T>::type());
      }

   template< class Stream, class T, class Shape, class Dir >
      inline void read( Stream & stream, array_2d<T, Shape, Dir> & array )
      {
         cg::point_2i ext;
         read( stream, ext );

         array.resize(ext);

         for ( cg::array_2d<T>::iterator v = array.begin(); v != array.end(); ++v )
            read( stream, *v );
      }
}
