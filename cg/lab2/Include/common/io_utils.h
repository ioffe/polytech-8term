#pragma once

#include <string>
#include <istream>
#include <ostream>
#include <boost/algorithm/string.hpp>
#include <vector>

namespace util
{
   template < class Char >
   void skip_char(std::basic_istream< Char > &in, int ch )
   {
      Char read_char = 0;
      in >> read_char;
      if ( read_char != ch )
      {
         in.putback( (Char)ch );
         in.clear( std::ios_base::badbit );
      }
   }

   template < class Char >
   std::basic_string< Char > first_nonempty_line( std::basic_istream< Char > & in )
   {
      std::basic_string< Char > s;

      do 
      {
         getline( in, s );
         boost::trim( s );
      }
      while ( in && s.empty() );

      return s;
   }

   template < class Char >
   void skip_spaces( std::basic_istream< Char > & in )
   {
      int c;
      for ( ; in && ( c = in.get(), isspace(c) ); );
      if ( !in )
         return;
      in.unget();
   }

   template < class Char >
   std::basic_string< Char > read_all( std::basic_istream< Char > & in )
   {
      return std::basic_string< Char >( (std::istreambuf_iterator<char>( in )), std::istreambuf_iterator<char>() );
   }

   template < class Char, class FwdIter >
   std::basic_ostream< Char > & write_range( std::basic_ostream< Char > & out, FwdIter p, FwdIter q )
   {
      if ( p == q )
         return out << "(empty)";

      out << "(";
      while( p != q )
      {
         out << *p;
         if ( ++p != q )
            out << ", ";
      }

      return out << ")";
   }

   template < class Char, class T >
   std::basic_ostream< Char > & operator <<( std::basic_ostream< Char > & out, std::vector< T > const & v )
   {
      return write_range( out, v.begin(), v.end() );
   }
}