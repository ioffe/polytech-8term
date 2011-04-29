#pragma once

#include <boost/shared_array.hpp>

#pragma pack ( push, 1 )

namespace util {
// Обёртка на boost/shared_array.hpp

template< class T >
struct const_shared_vector
{
   const_shared_vector() : size_(0) {}

   template<class It>
   const_shared_vector(It p, It q)
      : size_(std::distance(p, q))
      , data_(size_ ? new T[size_] : 0)
   {
      std::copy(p, q, data());
   }

   explicit const_shared_vector(std::vector<T> const& v)
      : size_(v.size())
      , data_(size_ ? new T[size_] : 0)
   {
      std::copy(v.begin(), v.end(), data());
   }

   explicit const_shared_vector(size_t size)
      : size_(size)
      , data_(size_ ? new T[size_] : 0)
   {
   }

   explicit const_shared_vector(size_t size, T const& defVal)
      : size_(size)
      , data_(size_ ? new T[size_] : 0)
   {
      std::fill(begin(), end(), defVal);
   }

   typedef T const*  const_iterator;

   const_iterator    begin()  const { return size_ ? data() : 0; }
   const_iterator    end()    const { return begin() + size_; }

   T const& operator[]( size_t idx ) const   { return data()[idx]; }

   T const& at( size_t idx ) const
   { 
      if (idx >= size) 
         throw std::out_of_range("invalid const_shared_vector<T> subscript");

      return (*this)[idx]; 
   }

   bool     empty() const { return size_ == 0; }
   size_t   size()  const { return size_; }

   void     swap(const_shared_vector<T> & s)  // never throws
   {
      using std::swap;
      swap(size_, s.size_);
      swap(data_, s.data_);
   }

   static const_shared_vector<T> from_vector( std::vector<T> const& v ) { return const_shared_vector<T>(v); }

protected:
   T       * data()       { return data_.get(); }
   T const * data() const { return data_.get(); }

private:
   size_t                  size_;
   boost::shared_array<T>  data_;
};
  
// паттерн Паблик Морозов
template< class T >
struct shared_vector : const_shared_vector< T >
{
   shared_vector() {}

   template<class It>
   shared_vector(It p, It q)
      : const_shared_vector(p, q)
   { }

   explicit shared_vector(std::vector<T> const& v)
      : const_shared_vector(v)
   { }

   explicit shared_vector(size_t size)
      : const_shared_vector(size)
   { }

   shared_vector(size_t size, T const& defVal)
      : const_shared_vector(size, defVal)
   { }

   explicit shared_vector( const_shared_vector<T> const& v )
      : const_shared_vector(v.begin(), v.end())
   {}

   typedef T*        iterator;

   using const_shared_vector< T >::begin;
   using const_shared_vector< T >::end;

   iterator          begin()        { return size() ? data() : 0; }
   iterator          end()          { return begin() + size(); }

   using const_shared_vector< T >::operator[];
   T &      operator[]( size_t idx )         { return data()[idx]; }

   using const_shared_vector< T >::at;
   T &      at( size_t idx )         
   { 
      if (idx >= size()) 
         throw std::out_of_range("invalid shared_vector<T> subscript");

      return (*this)[idx]; 
   }

   void     clear() { shared_vector<T>().swap(*this); }

   void     swap(shared_vector<T> & s)  // never throws
   {
      const_shared_vector<T>::swap(s);
   }

   static shared_vector<T> from_vector( std::vector<T> const& v ) { return shared_vector<T>(v); }
};

template<class T> void swap(shared_vector<T> & a, shared_vector<T> & b) // never throws
{
   a.swap(b);
}

template<class T> void swap(const_shared_vector<T> & a, const_shared_vector<T> & b) // never throws
{
   a.swap(b);
}

template<class T> shared_vector<T> join(const_shared_vector<T> const& a, const_shared_vector<T> const& b)
{
   size_t const size = a.size() + b.size();
   shared_vector<T> c(size);
   shared_vector<T>::iterator 
      it = std::copy(a.begin(), a.end(), c.begin() );
   it = std::copy(b.begin(), b.end(), it );
   Assert( it == c.end() );
   return c;
}

template<class T> shared_vector<T> join(const_shared_vector<T> const& a, shared_vector<T> const& b)
{
   return join(static_cast<const_shared_vector<T> const& >(a),
               static_cast<const_shared_vector<T> const& >(b));
}

} // namespace util

#pragma pack ( pop )
