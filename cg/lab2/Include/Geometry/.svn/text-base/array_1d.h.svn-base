#pragma   once

#include <ostream>
#include <streams/structured_streams.h>
#include <streams/structured_types.h>
#include <boost/scoped_array.hpp>

namespace streams { template<class T,class D = T> struct structured_ostream;
                    template<class> struct structured_istream; }

namespace cg
{
#pragma pack( push, 1 )

    template <class T>
        struct array_1d
    {
        typedef T           value_type;
        typedef T       &   reference;
        typedef T const &   const_reference;
        typedef T       *   pointer;
        typedef T const *   const_pointer;
        typedef T       *   iterator;
        typedef T const *   const_iterator;
        typedef size_t      size_type;

        array_1d () : size_(0) {}
        array_1d (size_t sz) : size_(sz), data_(new T[sz]) {}

        array_1d (size_t sz, T const & defval) : size_(sz), data_(new T[sz]) 
        {
            std::fill (begin(), end(), defval);
        }

        template< class ForwardIter >
        array_1d( ForwardIter p, ForwardIter q )
        {
           assign( p, q );
        }

        array_1d(const array_1d & rhs)
          : size_(rhs.size_)
        {
          data_.reset( new T[size_] );
#if _MSC_VER >= 1400
          stdext::unchecked_copy( rhs.data_.get(), rhs.data_.get() + size_, data_.get() );
#else
          std::copy( rhs.data_.get(), rhs.data_.get() + size_, data_.get() );
#endif
        }

        template < class ForwardIter >
        void assign( ForwardIter p, ForwardIter q )
        {
          size_ = std::distance( p, q );
          data_.reset( new T[size_] );
#if _MSC_VER >= 1400
          stdext::unchecked_copy( p, q, data_.get() );
#else
          std::copy( p, q, data_.get() );
#endif
        }

        void swap(array_1d & rhs)
        {
          std::swap( size_, rhs.size_ );
          data_.swap( rhs.data_ );
        }

        array_1d & operator = ( const array_1d & rhs )
        {
          array_1d tmp( rhs );
          this->swap( tmp );
          return *this;
        }

        void reset (size_t new_sz = 0)
        {
            data_.reset( (size_ = new_sz) ? new T[new_sz] : 0 );
        }

        void reset (size_t new_sz, T const & defval)
        {
            data_.reset( (size_ = new_sz) ? new T[new_sz] : 0 );
            std::fill (begin(), end(), defval);
        }

        void resize( size_t new_sz )
        {
           boost::scoped_array< T > new_data( new T[ new_sz ] );
          DWORD num_copy_obj = new_sz > size_ ? size_ : new_sz;
#if _MSC_VER >= 1400
          stdext::unchecked_copy( data_.get(), data_.get() + num_copy_obj, new_data.get() );
#else
          std::copy( data_.get(), data_.get() + num_copy_obj, new_data );
#endif
          size_ = new_sz;
          data_.swap( new_data );
        }

        void clear()
        {
           reset();
        }

        size_t  size () const { return size_; }
        bool    empty() const { return size_ == 0; }

        const_reference operator [] (int idx) const { return data_[idx]; }
        reference       operator [] (int idx)       { return data_[idx]; }

        reference       at (int idx)       { check_valid(idx); return data_[idx]; } 
        const_reference at (int idx) const { check_valid(idx); return data_[idx]; } 

        bool contains (int idx) const { return 0 <= idx && idx < size_; } 

        void check_valid(int idx) const
        {
            Assert (0 <= idx); 
            Assert (idx < (int)size_);
        }

        iterator    begin () { return data_.get();         }
        iterator    end   () { return data_.get() + size_; }

        const_iterator begin () const { return data_.get(); }
        const_iterator end   () const { return data_.get() + size_; }

        reference front () { return *data_.get(); }
        reference back  () { return *(data_.get() + size_ - 1); }

        const_reference front () const { return *data_.get(); }
        const_reference back  () const { return *(data_.get() + size_ - 1); }

        bool contains (iterator it)       const { return data_.get() <= it && it < data_.get() + size_; }
        bool contains (const_iterator it) const { return data_.get() <= it && it < data_.get() + size_; }

    private:
        size_t  size_;
        boost::scoped_array< T > data_;
    };

    template < class T > 
    size_t memory_used( array_1d< T > const & a )
    {
       return a.size() * sizeof( T );
    }

#pragma pack (pop)

   template < class T >
      bool operator ==( array_1d< T > const & lhs, array_1d< T > const & rhs )
   {
#if _MSC_VER >= 1400
      return lhs.size() == rhs.size() && stdext::unchecked_equal( lhs.begin(), lhs.end(), rhs.begin() );
#else
      return lhs.size() == rhs.size() && std::equal( lhs.begin(), lhs.end(), rhs.begin() );
#endif
   }

   template <class T>
      bool operator < (array_1d< T > const & lhs, array_1d< T > const & rhs )
      {
         return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
      }

   template< class Stream, class TempStream, class T >
      void write( streams::structured_ostream<Stream, TempStream> & stream, array_1d<T> const& array )
   {
      stream.push();

      write( stream, array.size( ) );
      for ( cg::array_1d<T>::const_iterator it = array.begin( ); it != array.end( ); ++it )
         write( stream, *it );

      stream.pop();
   }

   template< class Stream, class T >
   void write( Stream & os, const array_1d< T > & array )
   {
      write( os, array.size( ) );
      for ( cg::array_1d< T >::const_iterator it = array.begin( ); it != array.end( ); ++it )
         write( os, *it );
   }

   template< class Stream, class T >
      void read( structured_istream<Stream> & is, array_1d< T > & array )
   {
      ofs_ptr_s p;
      read( is, p );

      cg::array_1d< T >::size_type size = *reinterpret_cast<DWORD const *>(p.ptr());      

      T const * ptr = reinterpret_cast<T const *>(p.ptr() + sizeof(DWORD));
      array = array_1d< T >( ptr, ptr + size );
   }

   template< class Stream, class T >
      void read( Stream & is, array_1d< T > & array )
   {
      cg::array_1d< T >::size_type size;
      read( is, size );

      array.resize( size );
      for ( cg::array_1d< T >::iterator it = array.begin( ); it != array.end( ); ++it )
         read( is, *it );
   }

   template < class T >
      std::ostream & operator <<( std::ostream & out, cg::array_1d< T > const & arr )
   {
      out << '(';
      std::copy( arr.begin(), arr.end(), std::ostream_iterator< T >( out, ", " ) );
      out << ')';
      return out;
   }
}
