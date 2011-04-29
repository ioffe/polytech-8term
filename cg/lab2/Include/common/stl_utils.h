#pragma once

#include <algorithm>
#include <utility>
#include <functional>
#include <numeric>
#include <vector>
#include <boost/range.hpp>

namespace std
{
   using namespace tr1;
   using namespace tr1::placeholders;

   template < class Range >
      void sort( Range & c )
   {
      sort( boost::begin( c ), boost::end( c ) );
   }

   template < class Range, class Comparator >
      void sort( Range & c, Comparator f )
   {
      sort( boost::begin( c ), boost::end( c ), f );
   }

   template < class Range >
      typename boost::range_iterator< Range >::type
         unique( Range & c )
   {
      return unique( boost::begin( c ), boost::end( c ) );
   }

   //template < class Range, class Functor >
   //   typename boost::range_iterator< Range >::type
   //      unique( Range & c, Functor func )
   //{
   //   return unique( boost::begin( c ), boost::end( c ), func );
   //}

   template < class T >
      void leave_unique( vector< T > & v )
   {
      v.erase( unique( v ), v.end() );
   }

   template < class T, class Functor >
      void leave_unique( vector< T > & v, Functor func )
   {
      v.erase( unique( v.begin(), v.end(), func ), v.end() );
   }

   template < class Range, class Proc >
      Proc for_each( Range const & c, Proc proc )
   {
      return for_each( boost::begin( c ), boost::end( c ), proc );
   }

   template < class Range, class Proc >
      Proc for_each( Range & c, Proc proc )
   {
      return for_each( boost::begin( c ), boost::end( c ), proc );
   }

   template < class Range1, class Range2 >
   	bool equal( Range1 const & a, Range2 const & b )
   {
      return boost::size( a ) == boost::size( b )
          && equal( boost::begin( a ), boost::end( a ), boost::begin( b ) );
   }

   template < class T, class Range >
   	void assign( T & a, Range const & b )
   {
      a.assign( boost::begin( b ), boost::end( b ) );
   }

   template< class Range, class T >
      typename boost::range_iterator< Range >::type
         find( Range & c, const T& value )
   {
      return find( boost::begin( c ), boost::end( c ), value );
   }

   template< class Range, class T >
      typename boost::range_const_iterator< const Range >::type
         find( const Range& c, const T& value )
   {
      return find( boost::begin( c ), boost::end( c ), value );
   }

   template< class Range, class Functor >
      typename boost::range_iterator< Range >::type
         find_if( Range & c, Functor f )
   {
      return find_if( boost::begin( c ), boost::end( c ), f );
   }

   template< class Range, class Functor >
      typename boost::range_const_iterator< const Range >::type
         find_if( const Range & c, Functor f )
   {
      return find_if( boost::begin( c ), boost::end( c ), f );
   }

   template< class Range, class T >
      typename boost::range_iterator< Range >::type
         partition( Range & c, const T & value )
   {
      return partition( boost::begin( c ), boost::end( c ), value );
   }

   template< class Range >
      void reverse( Range & c )
   {
      return reverse( boost::begin( c ), boost::end( c ) );
   }

   template< class Range, class OutIter >
	   OutIter copy( Range const & c, OutIter out )
   {
      return copy( boost::begin( c ), boost::end( c ), out );
   }

   template < class Range, class OutIter, class Functor >
	   OutIter transform( Range & c, OutIter out, Functor func )
   {
      return transform( boost::begin( c ), boost::end( c ), out, func );
   }

   template < class Range, class OutIter, class Functor >
	   OutIter transform( Range const & c, OutIter out, Functor func )
   {
      return transform( boost::begin( c ), boost::end( c ), out, func );
   }

   template < class Range, class T >
	   T accumulate( Range const & c, T t )
   {
      return std::accumulate( boost::begin( c ), boost::end( c ), t );
   }

   template < class Range, class T, class Functor >
	   T accumulate( Range const & c, T t, Functor func )
   {
      return accumulate( boost::begin( c ), boost::end( c ), t, func );
   }

   template< class Range, class T >
      typename boost::range_iterator< Range >::type
         remove( Range & c, T const & t )
   {
      return remove( boost::begin( c ), boost::end( c ), t );
   }

   template< class Range, class Functor >
      typename boost::range_iterator< Range >::type
         remove_if( Range & c, Functor f )
   {
      return remove_if( boost::begin( c ), boost::end( c ), f );
   }

   template< class Range, class OutIter, class Functor >
      OutIter remove_copy_if( Range const & c, OutIter out, Functor f )
   {
      return remove_copy_if( boost::begin( c ), boost::end( c ), out, f );
   }

   template < class T, class Functor >
      void erase_if( vector< T > & v, Functor func )
   {
      v.erase( remove_if( v, func ), v.end() );
   }

   template < class Range >
	   void random_shuffle( Range & c )
   {
      return random_shuffle( boost::begin( c ), boost::end( c ) );
   }

   template< class Range, class T >
      typename boost::range_const_iterator< const Range >::type
         lower_bound( const Range & c, T const & t )
   {
      return lower_bound( boost::begin( c ), boost::end( c ), t );
   }

   template< class Range, class T >
      void fill( Range & c, T const & t )
   {
      return fill( boost::begin( c ), boost::end( c ), t );
   }
}